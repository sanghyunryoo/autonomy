#include "autonomy/parameter/parameter_manager.hpp"

#include <cmath>
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace autonomy {
namespace {

constexpr char kConfigDirectoryEnvironment[] = "AUTONOMY_CONFIG_DIR";
constexpr char kSourceConfigDirectory[] = "include/autonomy/parameter/config";

std::string defaultConfigDirectory()
{
  const char * const configured_directory = std::getenv(kConfigDirectoryEnvironment);
  if (configured_directory != nullptr && configured_directory[0] != '\0') {
    return configured_directory;
  }
  return kSourceConfigDirectory;
}

template<typename T>
T required(const YAML::Node & node, const char * key, const std::string & path)
{
  const YAML::Node value = node[key];
  if (!value) {
    throw std::invalid_argument(path + "." + key + " is required");
  }
  try {
    return value.as<T>();
  } catch (const std::exception &) {
    throw std::invalid_argument(path + "." + key + " is invalid");
  }
}

template<typename T>
T optional(const YAML::Node & node, const char * key, T fallback)
{
  const YAML::Node value = node[key];
  return value ? value.as<T>() : fallback;
}

std::chrono::milliseconds timeoutFor(const YAML::Node & node, const std::string & path)
{
  const int timeout_ms = optional<int>(node, "timeout_ms", 500);
  if (timeout_ms <= 0) {
    throw std::invalid_argument(path + ".timeout_ms must be positive");
  }
  return std::chrono::milliseconds(timeout_ms);
}

std::string resolvePath(const std::string & config_directory, const std::string & value)
{
  std::filesystem::path path(value);
  if (path.is_relative()) {
    path = std::filesystem::absolute(std::filesystem::path(config_directory) / path);
  }
  return path.lexically_normal().string();
}

YAML::Node loadSection(
  const std::string & config_directory,
  const char * file_name,
  const char * section_name)
{
  const std::filesystem::path path = std::filesystem::path(config_directory) / file_name;
  if (!std::filesystem::is_regular_file(path)) {
    throw std::invalid_argument("configuration file does not exist: " + path.string());
  }
  const YAML::Node root = YAML::LoadFile(path.string());
  const YAML::Node section = root[section_name];
  if (!section || !section.IsMap()) {
    throw std::invalid_argument(path.string() + " requires a " + section_name + " map");
  }
  return section;
}

void validateUniqueId(
  const std::string & id,
  std::unordered_set<std::string> & ids,
  const std::string & path)
{
  if (id.empty()) {
    throw std::invalid_argument(path + ".id must not be empty");
  }
  if (!ids.insert(id).second) {
    throw std::invalid_argument("duplicate sensor id: " + id);
  }
}

void validateGrid(const ElevationConfig & grid)
{
  if (!std::isfinite(grid.resolution) || grid.resolution <= 0.0 ||
    grid.x_max <= grid.x_min || grid.y_max <= grid.y_min || grid.max_z <= grid.min_z ||
    !std::isfinite(grid.maximum_range) || grid.maximum_range <= 0.0 || grid.point_stride == 0U ||
    !std::isfinite(grid.noise_alpha) || grid.noise_alpha < 0.0 ||
    !std::isfinite(grid.minimum_measurement_variance) || grid.minimum_measurement_variance <= 0.0 ||
    !std::isfinite(grid.initial_cell_variance) || grid.initial_cell_variance <= 0.0 ||
    !std::isfinite(grid.minimum_cell_variance) || grid.minimum_cell_variance <= 0.0 ||
    !std::isfinite(grid.maximum_cell_variance) ||
      grid.maximum_cell_variance < grid.minimum_cell_variance ||
    grid.initial_cell_variance < grid.minimum_cell_variance ||
    grid.initial_cell_variance > grid.maximum_cell_variance ||
    !std::isfinite(grid.time_variance_rate) || grid.time_variance_rate < 0.0 ||
    !std::isfinite(grid.mahalanobis_threshold) || grid.mahalanobis_threshold <= 0.0 ||
    !std::isfinite(grid.dynamic_environment_variance_bump) || grid.dynamic_environment_variance_bump < 0.0 ||
    !std::isfinite(grid.dynamic_reset_delta) || grid.dynamic_reset_delta <= 0.0 ||
    !std::isfinite(grid.robust_height_gate) || grid.robust_height_gate <= 0.0 ||
    !std::isfinite(grid.intra_cell_min_support_gap) || grid.intra_cell_min_support_gap <= 0.0 ||
    grid.intra_cell_min_support_count == 0U ||
    !std::isfinite(grid.edge_mix_height_difference) || grid.edge_mix_height_difference <= 0.0 ||
    grid.edge_prefer_previous_support_count == 0U ||
    !std::isfinite(grid.isolated_support_height_difference) ||
      grid.isolated_support_height_difference <= 0.0 ||
    !std::isfinite(grid.isolated_outlier_height_difference) ||
      grid.isolated_outlier_height_difference <= 0.0 ||
    grid.isolated_filter_every_n_frames == 0U || grid.hole_fill_min_neighbors == 0U ||
    !std::isfinite(grid.hole_fill_max_height_difference) ||
      grid.hole_fill_max_height_difference <= 0.0 ||
    !std::isfinite(grid.bilateral_sigma_spatial) || grid.bilateral_sigma_spatial <= 0.0 ||
    !std::isfinite(grid.bilateral_sigma_height) || grid.bilateral_sigma_height <= 0.0 ||
    !std::isfinite(grid.bilateral_max_height_difference) ||
      grid.bilateral_max_height_difference <= 0.0 ||
    grid.bilateral_every_n_frames == 0U) {
    throw std::invalid_argument("algorithm.elevation geometry is invalid");
  }
}

void validatePlanner(const PlannerConfig & planner)
{
  if (planner.input_timeout.count() <= 0 || planner.hybrid_astar.wheelbase <= 0.0 ||
    planner.hybrid_astar.vehicle_radius <= 0.0 || planner.hybrid_astar.step_size <= 0.0 ||
    planner.hybrid_astar.goal_tolerance <= 0.0 || planner.hybrid_astar.heading_bins == 0U ||
    planner.hybrid_astar.max_expansions == 0U || planner.mppi.sample_count == 0U ||
    planner.mppi.horizon_steps == 0U || planner.mppi.dt <= 0.0 ||
    planner.mppi.max_linear_velocity <= 0.0 || planner.mppi.max_angular_velocity <= 0.0 ||
    planner.mppi.temperature <= 0.0 || planner.mppi.vehicle_radius <= 0.0) {
    throw std::invalid_argument("algorithm.planner configuration is invalid");
  }
}

void validateAlgorithms(
  const double loop_hz,
  const ElevationConfig & elevation,
  const SlamConfig & slam,
  const MappingConfig & mapping,
  const PlannerConfig & planner,
  const DetectionConfig & detection)
{
  if (!std::isfinite(elevation.hz) || !std::isfinite(slam.hz) || !std::isfinite(planner.hz) ||
    !std::isfinite(detection.hz) || elevation.hz <= 0.0 || slam.hz <= 0.0 || planner.hz <= 0.0 || detection.hz <= 0.0 ||
    elevation.hz > loop_hz || slam.hz > loop_hz || planner.hz > loop_hz || detection.hz > loop_hz ||
    slam.map_frame.empty() || mapping.map_frame.empty() || slam.map_frame != mapping.map_frame ||
    slam.minimum_points == 0U || slam.minimum_imu_samples == 0U || slam.minimum_range < 0.0 ||
    slam.maximum_range <= slam.minimum_range || slam.maximum_scan_translation <= 0.0 ||
    slam.maximum_yaw_rate <= 0.0 || slam.scan_voxel_size <= 0.0 || slam.map_voxel_size <= 0.0 ||
    slam.maximum_map_points == 0U || slam.registration_iterations == 0U || slam.keyframe_distance <= 0.0 ||
    slam.loop_min_keyframes == 0U || slam.loop_descriptor_threshold <= 0.0 ||
    slam.loop_fitness_threshold <= 0.0 || slam.loop_max_correspondence <= 0.0 || mapping.resolution <= 0.0 ||
    mapping.x_max <= mapping.x_min || mapping.y_max <= mapping.y_min ||
    mapping.obstacle_max_z <= mapping.obstacle_min_z || mapping.maximum_range <= 0.0 ||
    detection.camera_id.empty() || detection.minimum_depth <= 0.0 ||
    detection.maximum_depth <= detection.minimum_depth ||
    detection.sample_stride == 0U || detection.minimum_valid_samples == 0U) {
    throw std::invalid_argument("algorithm configuration is invalid");
  }
}

void validateMiddleware(const MiddlewareConfig & middleware, const double loop_hz)
{
  const Ros2Config & ros2 = middleware.ros2;
  const Ros2DebugConfig & debug = ros2.debug;
  if (ros2.robot_report_topic.empty() || ros2.navigation_goal_topic.empty() ||
    ros2.command_topic.empty() || ros2.command_filter_topic.empty() ||
    !std::isfinite(ros2.autopilot_hz) || ros2.autopilot_hz <= 0.0 || ros2.autopilot_hz > loop_hz ||
    !std::isfinite(debug.hz) || debug.hz <= 0.0 || debug.hz > loop_hz ||
    debug.max_points_per_cloud == 0U || debug.max_path_poses == 0U ||
    debug.max_image_width == 0U || debug.max_image_height == 0U) {
    throw std::invalid_argument("middleware.ros2 configuration is invalid");
  }
  const std::string * const debug_topics[] = {
    &debug.elevation_points_topic, &debug.sensor_tf_topic, &debug.slam_odom_topic,
    &debug.slam_path_topic, &debug.livox_points_topic, &debug.ai_rgb_topic, &debug.ai_bbox_topic,
  };
  for (const std::string * topic_name : debug_topics) {
    if (topic_name->rfind("/autonomy/", 0) != 0U) {
      throw std::invalid_argument("middleware.ros2.debug topics must start with /autonomy/");
    }
  }
  if (middleware.dds.height_map_topic.empty() || middleware.dds.linear_velocity_topic.empty()) {
    throw std::invalid_argument("middleware.dds configuration is invalid");
  }
}

std::string lowercase(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

bool isSupportedCameraModel(const std::string & model)
{
  return model == "d435" || model == "d435i" || model == "d435if";
}

bool isSupportedLivoxModel(const std::string & model)
{
  return model == "mid360" || model == "mid360s";
}

}  // namespace

ParameterManager::ParameterManager()
: ParameterManager(defaultConfigDirectory())
{
}

ParameterManager::ParameterManager(std::string config_directory)
: config_directory_(std::move(config_directory))
{
}

void ParameterManager::load()
{
  const std::filesystem::path directory = std::filesystem::absolute(config_directory_);
  if (!std::filesystem::is_directory(directory)) {
    throw std::invalid_argument("configuration directory does not exist: " + directory.string());
  }
  config_directory_ = directory.lexically_normal().string();
  const YAML::Node core = loadSection(config_directory_, "core.yaml", "core");
  const YAML::Node sensor = loadSection(config_directory_, "sensor.yaml", "sensor");
  const YAML::Node algorithm = loadSection(config_directory_, "algorithm.yaml", "algorithm");
  const YAML::Node middleware = loadSection(config_directory_, "middleware.yaml", "middleware");

  RuntimeConfig loaded;
  loaded.loop_hz = required<double>(core, "loop_hz", "core");
  if (!std::isfinite(loaded.loop_hz) || loaded.loop_hz <= 0.0) {
    throw std::invalid_argument("core.loop_hz must be positive");
  }
  const YAML::Node state = core["state"];
  if (state) {
    const int timeout_ms = optional<int>(state, "robot_report_timeout_ms", 500);
    if (timeout_ms <= 0) throw std::invalid_argument("core.state.robot_report_timeout_ms must be positive");
    loaded.state.robot_report_timeout = std::chrono::milliseconds(timeout_ms);
    loaded.state.require_robot_report = optional<bool>(state, "require_robot_report", true);
  }

  const YAML::Node network = core["network"];
  if (network) {
    loaded.network.lidar_local_ip = optional<std::string>(
      network, "lidar_local_ip", loaded.network.lidar_local_ip);
    loaded.network.sbc_local_ip = optional<std::string>(
      network, "sbc_local_ip", loaded.network.sbc_local_ip);
    loaded.network.sbc_peer_ip = optional<std::string>(
      network, "sbc_peer_ip", loaded.network.sbc_peer_ip);
  }

  loaded.base_frame = required<std::string>(sensor, "base_frame", "sensor");
  loaded.target_frame = optional<std::string>(sensor, "target_frame", loaded.base_frame);
  loaded.urdf_path = resolvePath(config_directory_, required<std::string>(sensor, "urdf_path", "sensor"));
  if (loaded.base_frame.empty() || loaded.target_frame.empty()) {
    throw std::invalid_argument("sensor base_frame and target_frame must not be empty");
  }
  if (!std::filesystem::is_regular_file(loaded.urdf_path)) {
    throw std::invalid_argument("sensor.urdf_path does not exist: " + loaded.urdf_path);
  }
  const YAML::Node elevation = algorithm["elevation"];
  if (elevation) {
    loaded.elevation.hz = optional<double>(elevation, "hz", loaded.elevation.hz);
    loaded.elevation.resolution = optional<double>(elevation, "resolution", loaded.elevation.resolution);
    loaded.elevation.x_min = optional<double>(elevation, "x_min", loaded.elevation.x_min);
    loaded.elevation.x_max = optional<double>(elevation, "x_max", loaded.elevation.x_max);
    loaded.elevation.y_min = optional<double>(elevation, "y_min", loaded.elevation.y_min);
    loaded.elevation.y_max = optional<double>(elevation, "y_max", loaded.elevation.y_max);
    loaded.elevation.min_z = optional<double>(elevation, "min_z", loaded.elevation.min_z);
    loaded.elevation.max_z = optional<double>(elevation, "max_z", loaded.elevation.max_z);
    loaded.elevation.maximum_range = optional<double>(
      elevation, "maximum_range", loaded.elevation.maximum_range);
    loaded.elevation.point_stride = optional<std::uint32_t>(
      elevation, "point_stride", loaded.elevation.point_stride);
    loaded.elevation.noise_alpha = optional<double>(elevation, "noise_alpha", loaded.elevation.noise_alpha);
    loaded.elevation.minimum_measurement_variance = optional<double>(
      elevation, "minimum_measurement_variance", loaded.elevation.minimum_measurement_variance);
    loaded.elevation.initial_cell_variance = optional<double>(
      elevation, "initial_cell_variance", loaded.elevation.initial_cell_variance);
    loaded.elevation.minimum_cell_variance = optional<double>(
      elevation, "minimum_cell_variance", loaded.elevation.minimum_cell_variance);
    loaded.elevation.maximum_cell_variance = optional<double>(
      elevation, "maximum_cell_variance", loaded.elevation.maximum_cell_variance);
    loaded.elevation.time_variance_rate = optional<double>(
      elevation, "time_variance_rate", loaded.elevation.time_variance_rate);
    loaded.elevation.mahalanobis_threshold = optional<double>(
      elevation, "mahalanobis_threshold", loaded.elevation.mahalanobis_threshold);
    loaded.elevation.dynamic_environment_variance_bump = optional<double>(
      elevation, "dynamic_environment_variance_bump", loaded.elevation.dynamic_environment_variance_bump);
    loaded.elevation.dynamic_reset_delta = optional<double>(
      elevation, "dynamic_reset_delta", loaded.elevation.dynamic_reset_delta);
    loaded.elevation.robust_height_gate = optional<double>(
      elevation, "robust_height_gate", loaded.elevation.robust_height_gate);
    loaded.elevation.intra_cell_min_support_gap = optional<double>(
      elevation, "intra_cell_min_support_gap", loaded.elevation.intra_cell_min_support_gap);
    loaded.elevation.intra_cell_min_support_count = optional<std::uint32_t>(
      elevation, "intra_cell_min_support_count", loaded.elevation.intra_cell_min_support_count);
    loaded.elevation.edge_mix_height_difference = optional<double>(
      elevation, "edge_mix_height_difference", loaded.elevation.edge_mix_height_difference);
    loaded.elevation.edge_prefer_previous_support_count = optional<std::uint32_t>(
      elevation, "edge_prefer_previous_support_count", loaded.elevation.edge_prefer_previous_support_count);
    loaded.elevation.isolated_radius = optional<std::uint32_t>(
      elevation, "isolated_radius", loaded.elevation.isolated_radius);
    loaded.elevation.isolated_min_support_neighbors = optional<std::uint32_t>(
      elevation, "isolated_min_support_neighbors", loaded.elevation.isolated_min_support_neighbors);
    loaded.elevation.isolated_support_height_difference = optional<double>(
      elevation, "isolated_support_height_difference", loaded.elevation.isolated_support_height_difference);
    loaded.elevation.isolated_outlier_height_difference = optional<double>(
      elevation, "isolated_outlier_height_difference", loaded.elevation.isolated_outlier_height_difference);
    loaded.elevation.isolated_filter_every_n_frames = optional<std::uint32_t>(
      elevation, "isolated_filter_every_n_frames", loaded.elevation.isolated_filter_every_n_frames);
    loaded.elevation.hole_fill_radius = optional<std::uint32_t>(
      elevation, "hole_fill_radius", loaded.elevation.hole_fill_radius);
    loaded.elevation.hole_fill_min_neighbors = optional<std::uint32_t>(
      elevation, "hole_fill_min_neighbors", loaded.elevation.hole_fill_min_neighbors);
    loaded.elevation.hole_fill_max_height_difference = optional<double>(
      elevation, "hole_fill_max_height_difference", loaded.elevation.hole_fill_max_height_difference);
    loaded.elevation.bilateral_radius = optional<std::uint32_t>(
      elevation, "bilateral_radius", loaded.elevation.bilateral_radius);
    loaded.elevation.bilateral_sigma_spatial = optional<double>(
      elevation, "bilateral_sigma_spatial", loaded.elevation.bilateral_sigma_spatial);
    loaded.elevation.bilateral_sigma_height = optional<double>(
      elevation, "bilateral_sigma_height", loaded.elevation.bilateral_sigma_height);
    loaded.elevation.bilateral_max_height_difference = optional<double>(
      elevation, "bilateral_max_height_difference", loaded.elevation.bilateral_max_height_difference);
    loaded.elevation.bilateral_passes = optional<std::uint32_t>(
      elevation, "bilateral_passes", loaded.elevation.bilateral_passes);
    loaded.elevation.bilateral_every_n_frames = optional<std::uint32_t>(
      elevation, "bilateral_every_n_frames", loaded.elevation.bilateral_every_n_frames);
  }
  validateGrid(loaded.elevation);

  const YAML::Node slam = algorithm["slam"];
  if (slam) {
    loaded.slam.hz = optional<double>(slam, "hz", loaded.slam.hz);
    loaded.slam.map_frame = optional<std::string>(slam, "map_frame", loaded.slam.map_frame);
    loaded.slam.minimum_points = optional<std::uint32_t>(slam, "minimum_points", loaded.slam.minimum_points);
    loaded.slam.minimum_imu_samples = optional<std::uint32_t>(slam, "minimum_imu_samples", loaded.slam.minimum_imu_samples);
    loaded.slam.minimum_range = optional<double>(slam, "minimum_range", loaded.slam.minimum_range);
    loaded.slam.maximum_range = optional<double>(slam, "maximum_range", loaded.slam.maximum_range);
    loaded.slam.maximum_scan_translation = optional<double>(slam, "maximum_scan_translation", loaded.slam.maximum_scan_translation);
    loaded.slam.maximum_yaw_rate = optional<double>(slam, "maximum_yaw_rate", loaded.slam.maximum_yaw_rate);
    loaded.slam.scan_voxel_size = optional<double>(slam, "scan_voxel_size", loaded.slam.scan_voxel_size);
    loaded.slam.map_voxel_size = optional<double>(slam, "map_voxel_size", loaded.slam.map_voxel_size);
    loaded.slam.maximum_map_points = optional<std::uint32_t>(slam, "maximum_map_points", loaded.slam.maximum_map_points);
    loaded.slam.registration_iterations = optional<std::uint32_t>(slam, "registration_iterations", loaded.slam.registration_iterations);
    loaded.slam.keyframe_distance = optional<double>(slam, "keyframe_distance", loaded.slam.keyframe_distance);
    loaded.slam.loop_min_keyframes = optional<std::uint32_t>(slam, "loop_min_keyframes", loaded.slam.loop_min_keyframes);
    loaded.slam.loop_descriptor_threshold = optional<double>(slam, "loop_descriptor_threshold", loaded.slam.loop_descriptor_threshold);
    loaded.slam.loop_fitness_threshold = optional<double>(slam, "loop_fitness_threshold", loaded.slam.loop_fitness_threshold);
    loaded.slam.loop_max_correspondence = optional<double>(slam, "loop_max_correspondence", loaded.slam.loop_max_correspondence);
    const YAML::Node mapping = slam["mapping"];
    if (mapping) {
      loaded.mapping.map_frame = optional<std::string>(mapping, "map_frame", loaded.mapping.map_frame);
      loaded.mapping.resolution = optional<double>(mapping, "resolution", loaded.mapping.resolution);
      loaded.mapping.x_min = optional<double>(mapping, "x_min", loaded.mapping.x_min);
      loaded.mapping.x_max = optional<double>(mapping, "x_max", loaded.mapping.x_max);
      loaded.mapping.y_min = optional<double>(mapping, "y_min", loaded.mapping.y_min);
      loaded.mapping.y_max = optional<double>(mapping, "y_max", loaded.mapping.y_max);
      loaded.mapping.obstacle_min_z = optional<double>(mapping, "obstacle_min_z", loaded.mapping.obstacle_min_z);
      loaded.mapping.obstacle_max_z = optional<double>(mapping, "obstacle_max_z", loaded.mapping.obstacle_max_z);
      loaded.mapping.maximum_range = optional<double>(mapping, "maximum_range", loaded.mapping.maximum_range);
    }
  }
  const YAML::Node detection = algorithm["detection"];
  if (detection) {
    loaded.detection.hz = optional<double>(detection, "hz", loaded.detection.hz);
    loaded.detection.camera_id = optional<std::string>(
      detection, "camera_id", loaded.detection.camera_id);
    loaded.detection.minimum_depth = optional<double>(detection, "minimum_depth", loaded.detection.minimum_depth);
    loaded.detection.maximum_depth = optional<double>(detection, "maximum_depth", loaded.detection.maximum_depth);
    loaded.detection.sample_stride = optional<std::uint32_t>(detection, "sample_stride", loaded.detection.sample_stride);
    loaded.detection.minimum_valid_samples = optional<std::uint32_t>(detection, "minimum_valid_samples", loaded.detection.minimum_valid_samples);
  }
  const YAML::Node planner = algorithm["planner"];
  if (planner) {
    loaded.planner.hz = optional<double>(planner, "hz", loaded.planner.hz);
    const int input_timeout_ms = optional<int>(planner, "input_timeout_ms", 500);
    if (input_timeout_ms <= 0) throw std::invalid_argument("algorithm.planner.input_timeout_ms must be positive");
    loaded.planner.input_timeout = std::chrono::milliseconds(input_timeout_ms);
    const YAML::Node global = planner["hybrid_astar"];
    if (global) {
      loaded.planner.hybrid_astar.wheelbase = optional<double>(global, "wheelbase", loaded.planner.hybrid_astar.wheelbase);
      loaded.planner.hybrid_astar.vehicle_radius = optional<double>(global, "vehicle_radius", loaded.planner.hybrid_astar.vehicle_radius);
      loaded.planner.hybrid_astar.step_size = optional<double>(global, "step_size", loaded.planner.hybrid_astar.step_size);
      loaded.planner.hybrid_astar.max_steering_radians = optional<double>(global, "max_steering_radians", loaded.planner.hybrid_astar.max_steering_radians);
      loaded.planner.hybrid_astar.goal_tolerance = optional<double>(global, "goal_tolerance", loaded.planner.hybrid_astar.goal_tolerance);
      loaded.planner.hybrid_astar.heading_bins = optional<std::uint32_t>(global, "heading_bins", loaded.planner.hybrid_astar.heading_bins);
      loaded.planner.hybrid_astar.max_expansions = optional<std::uint32_t>(global, "max_expansions", loaded.planner.hybrid_astar.max_expansions);
      loaded.planner.hybrid_astar.occupied_threshold = static_cast<std::int8_t>(optional<int>(global, "occupied_threshold", loaded.planner.hybrid_astar.occupied_threshold));
      loaded.planner.hybrid_astar.unknown_is_obstacle = optional<bool>(global, "unknown_is_obstacle", loaded.planner.hybrid_astar.unknown_is_obstacle);
      loaded.planner.hybrid_astar.allow_reverse = optional<bool>(global, "allow_reverse", loaded.planner.hybrid_astar.allow_reverse);
    }
    const YAML::Node local = planner["mppi"];
    if (local) {
      loaded.planner.mppi.sample_count = optional<std::uint32_t>(local, "sample_count", loaded.planner.mppi.sample_count);
      loaded.planner.mppi.horizon_steps = optional<std::uint32_t>(local, "horizon_steps", loaded.planner.mppi.horizon_steps);
      loaded.planner.mppi.dt = optional<double>(local, "dt", loaded.planner.mppi.dt);
      loaded.planner.mppi.max_linear_velocity = optional<double>(local, "max_linear_velocity", loaded.planner.mppi.max_linear_velocity);
      loaded.planner.mppi.max_angular_velocity = optional<double>(local, "max_angular_velocity", loaded.planner.mppi.max_angular_velocity);
      loaded.planner.mppi.linear_noise = optional<double>(local, "linear_noise", loaded.planner.mppi.linear_noise);
      loaded.planner.mppi.angular_noise = optional<double>(local, "angular_noise", loaded.planner.mppi.angular_noise);
      loaded.planner.mppi.temperature = optional<double>(local, "temperature", loaded.planner.mppi.temperature);
      loaded.planner.mppi.vehicle_radius = optional<double>(local, "vehicle_radius", loaded.planner.mppi.vehicle_radius);
      loaded.planner.mppi.occupied_threshold = static_cast<std::int8_t>(optional<int>(local, "occupied_threshold", loaded.planner.mppi.occupied_threshold));
      loaded.planner.mppi.unknown_is_obstacle = optional<bool>(local, "unknown_is_obstacle", loaded.planner.mppi.unknown_is_obstacle);
    }
  }
  validatePlanner(loaded.planner);
  validateAlgorithms(
    loaded.loop_hz, loaded.elevation, loaded.slam, loaded.mapping, loaded.planner, loaded.detection);

  const YAML::Node ros2 = middleware["ros2"];
  if (!ros2 || !ros2.IsMap()) {
    throw std::invalid_argument("middleware.ros2 must be a map");
  }
  loaded.middleware.ros2.robot_report_topic = optional<std::string>(
    ros2, "robot_report_topic", loaded.middleware.ros2.robot_report_topic);
  loaded.middleware.ros2.command_topic = optional<std::string>(
    ros2, "command_topic", loaded.middleware.ros2.command_topic);
  loaded.middleware.ros2.autopilot_hz = optional<double>(
    ros2, "autopilot_hz", loaded.middleware.ros2.autopilot_hz);
  loaded.middleware.ros2.command_filter_topic = optional<std::string>(
    ros2, "command_filter_topic", loaded.middleware.ros2.command_filter_topic);
  loaded.middleware.ros2.navigation_goal_topic = optional<std::string>(
    ros2, "navigation_goal_topic", loaded.middleware.ros2.navigation_goal_topic);
  const YAML::Node debug = ros2["debug"];
  if (debug) {
    if (!debug.IsMap()) throw std::invalid_argument("middleware.ros2.debug must be a map");
    loaded.middleware.ros2.debug.enabled = optional<bool>(
      debug, "enabled", loaded.middleware.ros2.debug.enabled);
    loaded.middleware.ros2.debug.hz = optional<double>(debug, "hz", loaded.middleware.ros2.debug.hz);
    loaded.middleware.ros2.debug.max_points_per_cloud = optional<std::uint32_t>(
      debug, "max_points_per_cloud", loaded.middleware.ros2.debug.max_points_per_cloud);
    loaded.middleware.ros2.debug.max_path_poses = optional<std::uint32_t>(
      debug, "max_path_poses", loaded.middleware.ros2.debug.max_path_poses);
    loaded.middleware.ros2.debug.max_image_width = optional<std::uint32_t>(
      debug, "max_image_width", loaded.middleware.ros2.debug.max_image_width);
    loaded.middleware.ros2.debug.max_image_height = optional<std::uint32_t>(
      debug, "max_image_height", loaded.middleware.ros2.debug.max_image_height);
    loaded.middleware.ros2.debug.elevation_points_topic = optional<std::string>(
      debug, "elevation_points_topic", loaded.middleware.ros2.debug.elevation_points_topic);
    loaded.middleware.ros2.debug.sensor_tf_topic = optional<std::string>(
      debug, "sensor_tf_topic", loaded.middleware.ros2.debug.sensor_tf_topic);
    loaded.middleware.ros2.debug.slam_odom_topic = optional<std::string>(
      debug, "slam_odom_topic", loaded.middleware.ros2.debug.slam_odom_topic);
    loaded.middleware.ros2.debug.slam_path_topic = optional<std::string>(
      debug, "slam_path_topic", loaded.middleware.ros2.debug.slam_path_topic);
    loaded.middleware.ros2.debug.livox_points_topic = optional<std::string>(
      debug, "livox_points_topic", loaded.middleware.ros2.debug.livox_points_topic);
    loaded.middleware.ros2.debug.ai_rgb_topic = optional<std::string>(
      debug, "ai_rgb_topic", loaded.middleware.ros2.debug.ai_rgb_topic);
    loaded.middleware.ros2.debug.ai_bbox_topic = optional<std::string>(
      debug, "ai_bbox_topic", loaded.middleware.ros2.debug.ai_bbox_topic);
  }
  const YAML::Node dds = middleware["dds"];
  if (!dds || !dds.IsMap()) {
    throw std::invalid_argument("middleware.dds must be a map");
  }
  loaded.middleware.dds.enabled = optional<bool>(dds, "enabled", loaded.middleware.dds.enabled);
  loaded.middleware.dds.domain_id = optional<std::uint32_t>(dds, "domain_id", loaded.middleware.dds.domain_id);
  loaded.middleware.dds.height_map_topic = optional<std::string>(dds, "height_map_topic", loaded.middleware.dds.height_map_topic);
  loaded.middleware.dds.linear_velocity_topic = optional<std::string>(dds, "linear_velocity_topic", loaded.middleware.dds.linear_velocity_topic);
  validateMiddleware(loaded.middleware, loaded.loop_hz);

  std::unordered_set<std::string> ids;
  const YAML::Node cameras = sensor["cameras"];
  if (cameras) {
    if (!cameras.IsSequence()) {
      throw std::invalid_argument("sensor.cameras must be a sequence");
    }
    for (const YAML::Node & node : cameras) {
      const std::string path = "sensor.cameras[]";
      CameraConfig camera;
      camera.id = required<std::string>(node, "id", path);
      validateUniqueId(camera.id, ids, path);
      camera.model = lowercase(required<std::string>(node, "model", path));
      camera.serial = optional<std::string>(node, "serial", "");
      camera.port = required<std::string>(node, "port", path);
      camera.role = required<std::string>(node, "role", path);
      camera.frame = required<std::string>(node, "frame", path);
      camera.width = optional<std::uint32_t>(node, "width", camera.width);
      camera.height = optional<std::uint32_t>(node, "height", camera.height);
      camera.fps = optional<std::uint32_t>(node, "fps", camera.fps);
      camera.enable_imu = optional<bool>(node, "enable_imu", camera.enable_imu);
      camera.enabled = optional<bool>(node, "enabled", true);
      camera.timeout = timeoutFor(node, path);
      if (!isSupportedCameraModel(camera.model) || (camera.serial.empty() && camera.port.empty()) ||
        camera.role.empty() || camera.frame.empty() || camera.width == 0U || camera.height == 0U ||
        camera.fps == 0U) {
        throw std::invalid_argument(
                path + " requires a d435/d435i/d435if model, serial or port, role, frame, and stream geometry");
      }
      loaded.cameras.push_back(std::move(camera));
    }
  }

  const YAML::Node lidars = sensor["lidars"];
  if (lidars) {
    if (!lidars.IsSequence()) {
      throw std::invalid_argument("sensor.lidars must be a sequence");
    }
    for (const YAML::Node & node : lidars) {
      const std::string path = "sensor.lidars[]";
      LidarConfig lidar;
      lidar.id = required<std::string>(node, "id", path);
      validateUniqueId(lidar.id, ids, path);
      lidar.model = lowercase(required<std::string>(node, "model", path));
      lidar.ip = required<std::string>(node, "ip", path);
      lidar.role = required<std::string>(node, "role", path);
      lidar.frame = required<std::string>(node, "frame", path);
      lidar.sdk_config_path = resolvePath(
        config_directory_, required<std::string>(node, "sdk_config_path", path));
      lidar.enabled = optional<bool>(node, "enabled", true);
      lidar.timeout = timeoutFor(node, path);
      if (!isSupportedLivoxModel(lidar.model) || lidar.ip.empty() || lidar.role.empty() ||
        lidar.frame.empty() || !std::filesystem::is_regular_file(lidar.sdk_config_path)) {
        throw std::invalid_argument(
                path + " requires a mid360/mid360s model, IP, role, frame, and Livox SDK JSON file");
      }
      loaded.lidars.push_back(std::move(lidar));
    }
  }

  if (loaded.cameras.empty() && loaded.lidars.empty()) {
    throw std::invalid_argument("at least one camera or lidar must be configured");
  }

  runtime_ = std::move(loaded);
  loaded_ = true;
}

const RuntimeConfig & ParameterManager::runtime() const
{
  if (!loaded_) {
    throw std::logic_error("ParameterManager::load must be called first");
  }
  return runtime_;
}

const std::string & ParameterManager::configDirectory() const
{
  return config_directory_;
}

}  // namespace autonomy
