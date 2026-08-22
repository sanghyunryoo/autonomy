/*
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * ROS-free Super-LIO core adapted for this autonomy runtime. The original
 * Super-LIO project supplies the ESKF, voxel-map, and point-to-plane design;
 * this file deliberately replaces its ROSWrapper with SensorManager inputs.
 */

#include "autonomy/algorithm/slam/super_lio.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

namespace autonomy {
namespace {

constexpr double kGravity = 9.7946;
constexpr double kPi = 3.14159265358979323846;
constexpr int kPlaneNeighbors = 5;
constexpr int kDescriptorRings = 20;
constexpr int kDescriptorSectors = 60;
constexpr float kDescriptorRange = 80.0F;

using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

double seconds(const std::uint64_t nanoseconds)
{
  return static_cast<double>(nanoseconds) * 1.0e-9;
}

double normalizeAngle(double angle)
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle <= -kPi) angle += 2.0 * kPi;
  return angle;
}

Eigen::Quaterniond quaternionFromRotationVector(const Vector3 & rotation)
{
  const double angle = rotation.norm();
  if (angle < 1.0e-12) return Eigen::Quaterniond::Identity();
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation / angle));
}

gtsam::Pose3 pose3From(const Eigen::Quaterniond & orientation, const Vector3 & position)
{
  return gtsam::Pose3(
    gtsam::Rot3(orientation.toRotationMatrix()),
    gtsam::Point3(position.x(), position.y(), position.z()));
}

gtsam::SharedNoiseModel odometryNoise(const double rotation, const double translation)
{
  gtsam::Vector6 sigma;
  sigma << rotation, rotation, rotation, translation, translation, translation;
  return gtsam::noiseModel::Diagonal::Sigmas(sigma);
}

pcl::PointXYZI pclPoint(const float x, const float y, const float z, const float intensity)
{
  pcl::PointXYZI result;
  result.x = x;
  result.y = y;
  result.z = z;
  result.intensity = intensity;
  return result;
}

Vector3 rotateVector(const Transform & transform, const std::array<double, 3> & value)
{
  const Vector3 input(value[0], value[1], value[2]);
  Matrix3 rotation;
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = 0U; column < 3U; ++column) {
      rotation(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) =
        transform.rotation[row * 3U + column];
    }
  }
  return rotation * input;
}

}  // namespace

struct SuperLioSlam::Impl {
  struct State {
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Vector3 position{Vector3::Zero()};
    Vector3 velocity{Vector3::Zero()};
    Vector3 gyro_bias{Vector3::Zero()};
    Vector3 acceleration_bias{Vector3::Zero()};
    double stamp_seconds{-1.0};
  };

  explicit Impl(SlamConfig value)
  : config(std::move(value)),
    map(new Cloud()),
    kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>()),
    graph(gtsam::ISAM2Params{})
  {
    pose.frame_id = config.map_frame;
  }

  SlamConfig config{};
  State state{};
  Pose2d pose{};
  bool imu_initialized{false};
  bool map_initialized{false};
  std::uint64_t last_imu_stamp_nanoseconds{0};
  std::uint32_t imu_initialization_count{0};
  Vector3 imu_mean_gyro{Vector3::Zero()};
  Vector3 imu_mean_acceleration{Vector3::Zero()};
  Cloud::Ptr map{};
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree{};
  gtsam::ISAM2 graph;
  gtsam::Values graph_estimate;
  struct Keyframe {
    gtsam::Pose3 pose{};
    Cloud::Ptr cloud{};
    std::vector<float> descriptor{};
  };
  std::vector<Keyframe> keyframes{};

  [[nodiscard]] SlamResult waiting(std::string text) const
  {
    SlamResult result;
    result.status = SlamStatus::WaitingForData;
    result.pose = pose;
    result.message = std::move(text);
    return result;
  }

  [[nodiscard]] SlamResult failed(std::string text) const
  {
    SlamResult result;
    result.status = SlamStatus::Failed;
    result.pose = pose;
    result.message = std::move(text);
    return result;
  }

  [[nodiscard]] SlamResult ready(std::string text, const std::chrono::steady_clock::time_point now)
  {
    pose.x = state.position.x();
    pose.y = state.position.y();
    const Matrix3 rotation = state.orientation.toRotationMatrix();
    pose.yaw = normalizeAngle(std::atan2(rotation(1, 0), rotation(0, 0)));
    pose.frame_id = config.map_frame;
    pose.received_at = now;
    SlamResult result;
    result.status = SlamStatus::Ready;
    result.pose = pose;
    result.linear_velocity.x = state.velocity.x();
    result.linear_velocity.y = state.velocity.y();
    result.linear_velocity.z = state.velocity.z();
    result.linear_velocity.frame_id = config.map_frame;
    result.linear_velocity.received_at = now;
    result.message = std::move(text);
    return result;
  }

  [[nodiscard]] bool ingestImu(const SensorSnapshot & sensors, std::string & error)
  {
    struct TimedImu {
      ImuData data{};
      Transform target_from_sensor{};
    };
    std::vector<TimedImu> samples;
    for (const LidarSnapshot & lidar : sensors.lidars) {
      if (lidar.health != SensorHealth::Ready) continue;
      for (const ImuData & sample : lidar.data.imu_samples) {
        samples.push_back(TimedImu{sample, lidar.target_from_sensor});
      }
    }
    std::sort(samples.begin(), samples.end(), [](const TimedImu & left, const TimedImu & right) {
      return left.data.stamp_nanoseconds < right.data.stamp_nanoseconds;
    });
    for (const TimedImu & sample : samples) {
      if (!sample.data.valid || sample.data.stamp_nanoseconds == 0U ||
        sample.data.stamp_nanoseconds <= last_imu_stamp_nanoseconds) {
        continue;
      }
      const Vector3 gyro = rotateVector(sample.target_from_sensor, sample.data.angular_velocity);
      const Vector3 acceleration = rotateVector(sample.target_from_sensor, sample.data.linear_acceleration);
      if (!gyro.allFinite() || !acceleration.allFinite() || gyro.norm() > config.maximum_yaw_rate * 2.0 ||
        acceleration.norm() < 1.0e-6) {
        error = "Super-LIO received invalid LiDAR IMU data";
        return false;
      }
      const double stamp = seconds(sample.data.stamp_nanoseconds);
      if (!imu_initialized) {
        ++imu_initialization_count;
        imu_mean_gyro += (gyro - imu_mean_gyro) / static_cast<double>(imu_initialization_count);
        imu_mean_acceleration +=
          (acceleration - imu_mean_acceleration) / static_cast<double>(imu_initialization_count);
        if (imu_initialization_count >= config.minimum_imu_samples) {
          const Vector3 gravity_body = -imu_mean_acceleration.normalized() * kGravity;
          state.orientation = Eigen::Quaterniond::FromTwoVectors(gravity_body, Vector3(0.0, 0.0, -kGravity));
          state.orientation.normalize();
          state.gyro_bias = imu_mean_gyro;
          state.stamp_seconds = stamp;
          imu_initialized = true;
          LOG(INFO) << "Super-LIO ESKF IMU initialization completed";
        }
      } else {
        propagate(gyro, acceleration, stamp, error);
        if (!error.empty()) return false;
      }
      last_imu_stamp_nanoseconds = sample.data.stamp_nanoseconds;
    }
    return true;
  }

  void propagate(
    const Vector3 & raw_gyro,
    const Vector3 & raw_acceleration,
    const double stamp,
    std::string & error)
  {
    if (state.stamp_seconds < 0.0) {
      state.stamp_seconds = stamp;
      return;
    }
    const double dt = stamp - state.stamp_seconds;
    if (dt <= 0.0) return;
    if (!std::isfinite(dt) || dt > 0.20) {
      error = "Super-LIO ESKF IMU interval is invalid";
      return;
    }
    const Vector3 angular_velocity = raw_gyro - state.gyro_bias;
    const Vector3 body_acceleration = raw_acceleration - state.acceleration_bias;
    const Eigen::Quaterniond delta_orientation = quaternionFromRotationVector(angular_velocity * dt);
    state.orientation = (state.orientation * delta_orientation).normalized();
    const Vector3 world_acceleration = state.orientation * body_acceleration + Vector3(0.0, 0.0, -kGravity);
    state.position += state.velocity * dt + 0.5 * world_acceleration * dt * dt;
    state.velocity += world_acceleration * dt;
    state.stamp_seconds = stamp;
  }

  [[nodiscard]] Cloud::Ptr makeDeskewedScan(
    const MergedSensorData & lidar_data,
    const State & scan_start,
    std::string & error) const
  {
    Cloud::Ptr raw(new Cloud());
    raw->reserve(lidar_data.points.size());
    std::vector<float> time_offsets;
    time_offsets.reserve(lidar_data.points.size());
    float maximum_offset = 0.0F;
    for (const Point3f & point : lidar_data.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        !std::isfinite(point.time_offset_seconds)) {
        error = "Super-LIO scan contains a non-finite point";
        return raw;
      }
      const double range = std::hypot(point.x, point.y);
      if (range < config.minimum_range || range > config.maximum_range) continue;
      maximum_offset = std::max(maximum_offset, point.time_offset_seconds);
      raw->push_back(pclPoint(point.x, point.y, point.z, point.intensity));
      time_offsets.push_back(point.time_offset_seconds);
    }
    if (raw->size() < config.minimum_points) return raw;

    Cloud::Ptr deskewed(new Cloud());
    deskewed->resize(raw->size());
    const double scan_duration = std::max(1.0e-4, static_cast<double>(maximum_offset));
    const State scan_end = state;
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0U, raw->size()),
      [&raw, &deskewed, &time_offsets, &scan_start, &scan_end, scan_duration, maximum_offset](
        const tbb::blocked_range<std::size_t> & range) {
        for (std::size_t index = range.begin(); index < range.end(); ++index) {
          const pcl::PointXYZI & source = raw->points[index];
          const double ratio = maximum_offset > 0.0F ? std::clamp(
            static_cast<double>(time_offsets[index]) / scan_duration, 0.0, 1.0) : 1.0;
          const Eigen::Quaterniond orientation = scan_start.orientation.slerp(ratio, scan_end.orientation);
          const Vector3 position = (1.0 - ratio) * scan_start.position + ratio * scan_end.position;
          const Vector3 local(source.x, source.y, source.z);
          const Vector3 end_frame = scan_end.orientation.conjugate() *
            (orientation * local + position - scan_end.position);
          deskewed->points[index] = pclPoint(
            static_cast<float>(end_frame.x()), static_cast<float>(end_frame.y()),
            static_cast<float>(end_frame.z()), source.intensity);
        }
      });
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setInputCloud(deskewed);
    filter.setLeafSize(
      static_cast<float>(config.scan_voxel_size), static_cast<float>(config.scan_voxel_size),
      static_cast<float>(config.scan_voxel_size));
    Cloud::Ptr downsampled(new Cloud());
    filter.filter(*downsampled);
    return downsampled;
  }

  [[nodiscard]] bool pointToPlaneRegistration(const Cloud::Ptr & scan, std::string & error)
  {
    if (map->size() < static_cast<std::size_t>(kPlaneNeighbors) || scan->empty()) return true;
    kdtree->setInputCloud(map);
    const State predicted = state;
    for (std::uint32_t iteration = 0U; iteration < config.registration_iterations; ++iteration) {
      Matrix6 information = Matrix6::Zero();
      Vector6 residual = Vector6::Zero();
      std::size_t constraints = 0U;
      const Matrix3 rotation = state.orientation.toRotationMatrix();
      for (const pcl::PointXYZI & point : scan->points) {
        const Vector3 body(point.x, point.y, point.z);
        const Vector3 world = rotation * body + state.position;
        pcl::PointXYZI query;
        query.x = static_cast<float>(world.x());
        query.y = static_cast<float>(world.y());
        query.z = static_cast<float>(world.z());
        std::vector<int> indices(kPlaneNeighbors);
        std::vector<float> squared_distances(kPlaneNeighbors);
        if (kdtree->nearestKSearch(query, kPlaneNeighbors, indices, squared_distances) < kPlaneNeighbors ||
          squared_distances.back() > 25.0F) {
          continue;
        }
        Eigen::Matrix<double, kPlaneNeighbors, 3> matrix;
        Eigen::Matrix<double, kPlaneNeighbors, 1> target;
        target.setConstant(-1.0);
        for (int neighbor = 0; neighbor < kPlaneNeighbors; ++neighbor) {
          const pcl::PointXYZI & map_point = map->points[static_cast<std::size_t>(indices[neighbor])];
          matrix.row(neighbor) << map_point.x, map_point.y, map_point.z;
        }
        Vector3 normal = matrix.colPivHouseholderQr().solve(target);
        const double norm = normal.norm();
        if (!normal.allFinite() || norm < 1.0e-6) continue;
        normal /= norm;
        const double offset = 1.0 / norm;
        bool valid_plane = true;
        for (int neighbor = 0; neighbor < kPlaneNeighbors; ++neighbor) {
          if (std::abs(normal.dot(matrix.row(neighbor).transpose()) + offset) > 0.10) {
            valid_plane = false;
            break;
          }
        }
        if (!valid_plane) continue;
        const double distance = normal.dot(world) + offset;
        const double length = body.norm();
        if (!std::isfinite(distance) || length <= 81.0 * distance * distance) continue;
        Vector6 jacobian;
        const Vector3 normal_body = rotation.transpose() * normal;
        jacobian.head<3>() = body.cross(normal_body);
        jacobian.tail<3>() = normal;
        information.noalias() += 1000.0 * jacobian * jacobian.transpose();
        residual.noalias() -= 1000.0 * jacobian * distance;
        ++constraints;
      }
      if (constraints < static_cast<std::size_t>(kPlaneNeighbors * 2)) {
        error = "Super-LIO found too few point-to-plane constraints";
        return false;
      }
      information.diagonal().array() += 1.0e-6;
      const Vector6 update = information.ldlt().solve(residual);
      if (!update.allFinite()) {
        error = "Super-LIO ESKF registration solve failed";
        return false;
      }
      state.orientation = (state.orientation * quaternionFromRotationVector(update.head<3>())).normalized();
      state.position += update.tail<3>();
      if (update.norm() < 1.0e-4) break;
    }
    if ((state.position - predicted.position).norm() > config.maximum_scan_translation) {
      error = "Super-LIO registration exceeded the configured translation limit";
      return false;
    }
    return true;
  }

  void updateMap(const Cloud::Ptr & scan)
  {
    Cloud::Ptr world(new Cloud());
    world->resize(scan->size());
    const Matrix3 rotation = state.orientation.toRotationMatrix();
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0U, scan->size()),
      [&scan, &world, &rotation, this](const tbb::blocked_range<std::size_t> & range) {
        for (std::size_t index = range.begin(); index < range.end(); ++index) {
          const pcl::PointXYZI & source = scan->points[index];
          const Vector3 transformed = rotation * Vector3(source.x, source.y, source.z) + state.position;
          world->points[index] = pclPoint(
            static_cast<float>(transformed.x()), static_cast<float>(transformed.y()),
            static_cast<float>(transformed.z()), source.intensity);
        }
      });
    *map += *world;
    if (map->size() > config.maximum_map_points) {
      pcl::VoxelGrid<pcl::PointXYZI> filter;
      filter.setInputCloud(map);
      filter.setLeafSize(
        static_cast<float>(config.map_voxel_size), static_cast<float>(config.map_voxel_size),
        static_cast<float>(config.map_voxel_size));
      Cloud reduced;
      filter.filter(reduced);
      *map = std::move(reduced);
    }
  }

  [[nodiscard]] static std::vector<float> descriptor(const Cloud::Ptr & cloud)
  {
    std::vector<float> result(kDescriptorRings * kDescriptorSectors,
      -std::numeric_limits<float>::infinity());
    for (const pcl::PointXYZI & point : cloud->points) {
      const float range = std::hypot(point.x, point.y);
      if (!std::isfinite(range) || range >= kDescriptorRange) continue;
      const float angle = std::atan2(point.y, point.x) + static_cast<float>(kPi);
      const int ring = std::min(kDescriptorRings - 1,
        static_cast<int>(kDescriptorRings * range / kDescriptorRange));
      const int sector = std::min(kDescriptorSectors - 1,
        static_cast<int>(kDescriptorSectors * angle / static_cast<float>(2.0 * kPi)));
      result[ring * kDescriptorSectors + sector] = std::max(
        result[ring * kDescriptorSectors + sector], point.z);
    }
    for (float & value : result) {
      if (!std::isfinite(value)) value = 0.0F;
    }
    return result;
  }

  [[nodiscard]] static float descriptorDistance(
    const std::vector<float> & first,
    const std::vector<float> & second)
  {
    float best = std::numeric_limits<float>::infinity();
    for (int shift = 0; shift < kDescriptorSectors; ++shift) {
      float sum = 0.0F;
      for (int ring = 0; ring < kDescriptorRings; ++ring) {
        for (int sector = 0; sector < kDescriptorSectors; ++sector) {
          const float difference = first[ring * kDescriptorSectors + sector] -
            second[ring * kDescriptorSectors + (sector + shift) % kDescriptorSectors];
          sum += difference * difference;
        }
      }
      best = std::min(best, std::sqrt(sum / static_cast<float>(first.size())));
    }
    return best;
  }

  [[nodiscard]] int loopCandidate(const std::vector<float> & current) const
  {
    if (keyframes.size() <= config.loop_min_keyframes) return -1;
    float best_score = static_cast<float>(config.loop_descriptor_threshold);
    int best = -1;
    const std::size_t limit = keyframes.size() - config.loop_min_keyframes;
    for (std::size_t index = 0U; index < limit; ++index) {
      const float score = descriptorDistance(current, keyframes[index].descriptor);
      if (score < best_score) {
        best_score = score;
        best = static_cast<int>(index);
      }
    }
    return best;
  }

  [[nodiscard]] bool addLoopFactor(const int candidate, const Keyframe & current)
  {
    const Keyframe & target = keyframes[static_cast<std::size_t>(candidate)];
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    gicp.setInputSource(current.cloud);
    gicp.setInputTarget(target.cloud);
    gicp.setMaxCorrespondenceDistance(config.loop_max_correspondence);
    gicp.setMaximumIterations(32);
    gicp.setTransformationEpsilon(1.0e-4F);
    gicp.setEuclideanFitnessEpsilon(1.0e-4);
    Cloud aligned;
    const Eigen::Matrix4f initial =
      (target.pose.between(current.pose).matrix()).cast<float>();
    gicp.align(aligned, initial);
    if (!gicp.hasConverged() || gicp.getFitnessScore() > config.loop_fitness_threshold) {
      return false;
    }
    const Eigen::Matrix4f transform = gicp.getFinalTransformation();
    gtsam::NonlinearFactorGraph factors;
    factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('x', static_cast<std::size_t>(candidate)),
      gtsam::Symbol('x', keyframes.size() - 1U),
      gtsam::Pose3(
        gtsam::Rot3(transform.block<3, 3>(0, 0).cast<double>()),
        gtsam::Point3(transform(0, 3), transform(1, 3), transform(2, 3))),
      odometryNoise(0.03, 0.15)));
    graph.update(factors);
    return true;
  }

  void applyLatestPoseGraphCorrection(const gtsam::Pose3 & raw_pose)
  {
    graph_estimate = graph.calculateEstimate();
    const gtsam::Pose3 corrected = graph_estimate.at<gtsam::Pose3>(
      gtsam::Symbol('x', keyframes.size() - 1U));
    const gtsam::Pose3 correction = corrected.compose(raw_pose.inverse());
    const Eigen::Matrix3d correction_rotation = correction.rotation().matrix();
    const auto correction_point = correction.translation();
    const Vector3 correction_translation(
      correction_point.x(), correction_point.y(), correction_point.z());
    state.orientation = Eigen::Quaterniond(correction_rotation) * state.orientation;
    state.orientation.normalize();
    state.position = correction_rotation * state.position + correction_translation;
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0U, map->size()),
      [this, &correction_rotation, &correction_translation](const tbb::blocked_range<std::size_t> & range) {
        for (std::size_t index = range.begin(); index < range.end(); ++index) {
          pcl::PointXYZI & point = map->points[index];
          const Vector3 corrected_point = correction_rotation *
            Vector3(point.x, point.y, point.z) + correction_translation;
          point.x = static_cast<float>(corrected_point.x());
          point.y = static_cast<float>(corrected_point.y());
          point.z = static_cast<float>(corrected_point.z());
        }
      });
  }

  void updatePoseGraph(const Cloud::Ptr & scan)
  {
    const gtsam::Pose3 current = pose3From(state.orientation, state.position);
    if (keyframes.empty()) {
      gtsam::NonlinearFactorGraph factors;
      gtsam::Values initial;
      factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        gtsam::Symbol('x', 0U), current, odometryNoise(1.0e-3, 1.0e-3)));
      initial.insert(gtsam::Symbol('x', 0U), current);
      graph.update(factors, initial);
      graph_estimate = graph.calculateEstimate();
      keyframes.push_back(Keyframe{current, scan, descriptor(scan)});
      return;
    }
    const gtsam::Pose3 & previous = keyframes.back().pose;
    const gtsam::Pose3 relative = previous.between(current);
    const auto translation = relative.translation();
    const double translation_norm = std::sqrt(
      translation.x() * translation.x() + translation.y() * translation.y() + translation.z() * translation.z());
    if (translation_norm < config.keyframe_distance) return;
    const std::size_t index = keyframes.size();
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values initial;
    factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('x', index - 1U), gtsam::Symbol('x', index), relative, odometryNoise(0.05, 0.20)));
    initial.insert(gtsam::Symbol('x', index), current);
    graph.update(factors, initial);
    Keyframe keyframe{current, scan, descriptor(scan)};
    const int candidate = loopCandidate(keyframe.descriptor);
    keyframes.push_back(std::move(keyframe));
    if (candidate >= 0 && addLoopFactor(candidate, keyframes.back())) {
      LOG(INFO) << "Super-LIO loop closure accepted between keyframes " << candidate << " and " << index;
    }
    applyLatestPoseGraphCorrection(current);
  }
};

SuperLioSlam::SuperLioSlam(SlamConfig config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
  const SlamConfig & value = impl_->config;
  if (value.map_frame.empty() || value.minimum_points == 0U || value.minimum_imu_samples == 0U ||
    value.minimum_range < 0.0 || value.maximum_range <= value.minimum_range ||
    value.maximum_scan_translation <= 0.0 || value.maximum_yaw_rate <= 0.0 ||
    value.scan_voxel_size <= 0.0 || value.map_voxel_size <= 0.0 || value.maximum_map_points == 0U ||
    value.registration_iterations == 0U || value.keyframe_distance <= 0.0 ||
    value.loop_min_keyframes == 0U || value.loop_descriptor_threshold <= 0.0 ||
    value.loop_fitness_threshold <= 0.0 || value.loop_max_correspondence <= 0.0) {
    throw std::invalid_argument("Super-LIO configuration is invalid");
  }
}

SuperLioSlam::~SuperLioSlam() = default;

SlamResult SuperLioSlam::process(
  const SensorSnapshot & sensors,
  const std::chrono::steady_clock::time_point now)
{
  Impl & impl = *impl_;
  if (sensors.lidar_merged.points.empty()) return impl.waiting("waiting for transformed LiDAR points");
  const bool has_fresh_lidar_imu = std::any_of(
    sensors.lidars.begin(), sensors.lidars.end(), [&now](const LidarSnapshot & lidar) {
      return lidar.health == SensorHealth::Ready && !lidar.data.imu_samples.empty() &&
        lidar.imu_received_at != std::chrono::steady_clock::time_point{} &&
        now >= lidar.imu_received_at && now - lidar.imu_received_at <= lidar.timeout;
    });
  if (!has_fresh_lidar_imu) return impl.waiting("waiting for fresh LiDAR IMU samples");
  const Impl::State scan_start = impl.state;
  std::string error;
  if (!impl.ingestImu(sensors, error)) return impl.failed(std::move(error));
  if (!impl.imu_initialized) {
    return impl.waiting("waiting for Super-LIO stationary IMU initialization");
  }
  Cloud::Ptr scan = impl.makeDeskewedScan(sensors.lidar_merged, scan_start, error);
  if (!error.empty()) return impl.failed(std::move(error));
  if (scan->size() < impl.config.minimum_points) {
    return impl.waiting("waiting for enough valid LiDAR points for Super-LIO");
  }
  if (!impl.map_initialized) {
    impl.updateMap(scan);
    impl.map_initialized = true;
    impl.updatePoseGraph(scan);
    LOG(INFO) << "Super-LIO voxel map initialized with " << scan->size() << " points";
    return impl.ready("Super-LIO ESKF and voxel map initialized", now);
  }
  if (!impl.pointToPlaneRegistration(scan, error)) return impl.failed(std::move(error));
  impl.updateMap(scan);
  impl.updatePoseGraph(scan);
  return impl.ready("Super-LIO ESKF point-to-plane odometry updated", now);
}

const Pose2d & SuperLioSlam::pose() const
{
  return impl_->pose;
}

void SuperLioSlam::reset()
{
  const SlamConfig config = impl_->config;
  impl_ = std::make_unique<Impl>(config);
}

}  // namespace autonomy
