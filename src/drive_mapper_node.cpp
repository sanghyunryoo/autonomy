#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <optional>
#include <regex>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace autonomy
{
namespace
{

struct CameraBinding
{
  std::string role;
  std::string camera_name;
  std::string frame;
  std::string depth_topic;
  std::string camera_info_topic;
  std::string imu_topic;
};

std::string stripSlash(std::string text)
{
  while (!text.empty() && text.front() == '/') {
    text.erase(text.begin());
  }
  while (!text.empty() && text.back() == '/') {
    text.pop_back();
  }
  return text;
}

std::string jsonEscape(const std::string & text)
{
  std::ostringstream out;
  for (const char ch : text) {
    switch (ch) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }
  return out.str();
}

std::vector<double> parseVector3(const std::string & text)
{
  std::istringstream input(text);
  std::vector<double> values;
  double value = 0.0;
  while (input >> value) {
    values.push_back(value);
  }
  values.resize(3, 0.0);
  return values;
}

std::string regexGroup(const std::string & text, const std::regex & pattern)
{
  std::smatch match;
  return std::regex_search(text, match, pattern) && match.size() > 1 ? match[1].str() : "";
}

}  // namespace

class DriveMapperNode final : public rclcpp::Node
{
public:
  DriveMapperNode()
  : Node("drive_mapper_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    static_tf_broadcaster_(this)
  {
    loadParameters();
    binding_pub_ = create_publisher<std_msgs::msg::String>("~/camera_bindings", 10);
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>("/autonomy/heartbeat/drive_mapper_node", 10);
    attitude_correction_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(attitude_correction_topic_, 10);

    if (publish_static_tf_) {
      publishUrdfStaticTf();
    }
    if (publish_attitude_correction_ && !front_imu_topic_.empty()) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        front_imu_topic_,
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::SharedPtr msg) { onImu(std::move(msg)); });
    }

    status_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publishStatus(); });
    heartbeat_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { publishHeartbeat(); });
    correction_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      [this]() { publishAttitudeCorrection(); });
    publishStatus();
  }

private:
  void loadParameters()
  {
    simulation_ = declare_parameter<bool>("simulation", false);
    publish_static_tf_ = declare_parameter<bool>("publish_static_tf", true);
    publish_attitude_correction_ = declare_parameter<bool>("publish_attitude_correction", true);
    attitude_correction_topic_ =
      declare_parameter<std::string>("attitude_correction_topic", "~/attitude_correction");
    low_pass_alpha_ = std::clamp(declare_parameter<double>("low_pass_alpha", 0.08), 0.0, 1.0);
    min_accel_norm_ = std::max(0.1, declare_parameter<double>("min_accel_norm", 3.0));
    max_accel_norm_ = std::max(min_accel_norm_, declare_parameter<double>("max_accel_norm", 20.0));
    roll_sign_ = declare_parameter<double>("roll_sign", -1.0);
    pitch_sign_ = declare_parameter<double>("pitch_sign", -1.0);

    model_ = declare_parameter<std::string>("model", "f4");
    frame_prefix_ = declare_parameter<std::string>("frame_prefix", model_ + "/");
    topic_prefix_ = declare_parameter<std::string>("topic_prefix", "/" + model_);
    urdf_path_ = declare_parameter<std::string>(
      "urdf_path", "src/autonomy/resources/urdf/" + model_ + ".urdf");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", frame_prefix_ + "base_link");
    front_imu_topic_ = declare_parameter<std::string>("front_imu_topic", "");
    front_imu_frame_id_ = declare_parameter<std::string>("front_imu_frame_id", "");

    const auto roles =
      declare_parameter<std::vector<std::string>>("camera_roles", std::vector<std::string>{});
    const auto names =
      declare_parameter<std::vector<std::string>>("camera_names", std::vector<std::string>{});
    const auto frames =
      declare_parameter<std::vector<std::string>>("camera_frames", std::vector<std::string>{});
    const auto depth_topics =
      declare_parameter<std::vector<std::string>>("camera_depth_topics", std::vector<std::string>{});
    const auto info_topics =
      declare_parameter<std::vector<std::string>>("camera_info_topics", std::vector<std::string>{});
    const auto imu_topics =
      declare_parameter<std::vector<std::string>>("camera_imu_topics", std::vector<std::string>{});

    cameras_.clear();
    for (std::size_t i = 0; i < roles.size(); ++i) {
      CameraBinding binding;
      binding.role = roles[i];
      binding.camera_name = i < names.size() ? names[i] : roles[i] + "_camera";
      binding.frame = i < frames.size() ? frames[i] : mergeFrame(binding.camera_name);
      const auto base = topic_prefix_ + "/" + stripSlash(binding.camera_name);
      binding.depth_topic = i < depth_topics.size() ? depth_topics[i] : base + "/depth/image_rect_raw";
      binding.camera_info_topic = i < info_topics.size() ? info_topics[i] : base + "/depth/camera_info";
      binding.imu_topic = i < imu_topics.size() ? imu_topics[i] : base + "/imu";
      cameras_.push_back(std::move(binding));
    }
  }

  std::string mergeFrame(const std::string & frame) const
  {
    const auto text = stripSlash(frame);
    if (text.empty() || text.find('/') != std::string::npos || frame_prefix_.empty()) {
      return text;
    }
    return frame_prefix_ + text;
  }

  geometry_msgs::msg::TransformStamped makeTransform(
    const std::string & parent,
    const std::string & child,
    const std::array<double, 3> & xyz,
    const std::array<double, 3> & rpy) const
  {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = parent;
    msg.child_frame_id = child;
    msg.transform.translation.x = xyz[0];
    msg.transform.translation.y = xyz[1];
    msg.transform.translation.z = xyz[2];

    tf2::Quaternion quaternion;
    quaternion.setRPY(rpy[0], rpy[1], rpy[2]);
    msg.transform.rotation = tf2::toMsg(quaternion);
    return msg;
  }

  void publishUrdfStaticTf()
  {
    std::ifstream input(urdf_path_);
    if (!input) {
      RCLCPP_WARN(get_logger(), "URDF does not exist; mapper cannot publish static TF: %s", urdf_path_.c_str());
      return;
    }

    const std::string xml((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    const std::regex joint_re("<joint\\b([^>]*)>([\\s\\S]*?)</joint>");
    const std::regex type_re("type\\s*=\\s*\"([^\"]+)\"");
    const std::regex parent_re("<parent\\b[^>]*link\\s*=\\s*\"([^\"]+)\"");
    const std::regex child_re("<child\\b[^>]*link\\s*=\\s*\"([^\"]+)\"");
    const std::regex origin_re("<origin\\b([^>]*)/?>");
    const std::regex xyz_re("xyz\\s*=\\s*\"([^\"]+)\"");
    const std::regex rpy_re("rpy\\s*=\\s*\"([^\"]+)\"");

    for (std::sregex_iterator it(xml.begin(), xml.end(), joint_re), end; it != end; ++it) {
      const auto attrs = (*it)[1].str();
      const auto body = (*it)[2].str();
      if (regexGroup(attrs, type_re) != "fixed") {
        continue;
      }

      const auto parent = mergeFrame(regexGroup(body, parent_re));
      const auto child = mergeFrame(regexGroup(body, child_re));
      if (parent.empty() || child.empty() || parent == child) {
        continue;
      }
      std::array<double, 3> xyz{0.0, 0.0, 0.0};
      std::array<double, 3> rpy{0.0, 0.0, 0.0};
      const auto origin_attrs = regexGroup(body, origin_re);
      if (!origin_attrs.empty()) {
        const auto xyz_values = parseVector3(regexGroup(origin_attrs, xyz_re));
        const auto rpy_values = parseVector3(regexGroup(origin_attrs, rpy_re));
        std::copy_n(xyz_values.begin(), 3, xyz.begin());
        std::copy_n(rpy_values.begin(), 3, rpy.begin());
      }
      transforms.push_back(makeTransform(parent, child, xyz, rpy));
    }

    std::unordered_set<std::string> aliases;
    for (const auto & camera : cameras_) {
      const auto child = stripSlash(camera.frame);
      if (child.empty() || child.find('/') != std::string::npos) {
        continue;
      }
      const auto parent = mergeFrame(child);
      const auto key = parent + "->" + child;
      if (parent != child && aliases.insert(key).second) {
        transforms.push_back(makeTransform(parent, child, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}));
      }
    }

    if (!transforms.empty()) {
      static_tf_broadcaster_.sendTransform(transforms);
      RCLCPP_INFO(get_logger(), "Published %zu mapper static TF transforms from URDF", transforms.size());
    }
  }

  void onImu(sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto accel = accelerationInBaseFrame(*msg);
    if (!accel.has_value()) {
      return;
    }

    const auto & vector = accel.value();
    const double norm = std::sqrt(vector.x() * vector.x() + vector.y() * vector.y() + vector.z() * vector.z());
    if (norm < min_accel_norm_ || norm > max_accel_norm_) {
      return;
    }

    const double roll = std::atan2(vector.y(), vector.z());
    const double pitch = std::atan2(-vector.x(), std::hypot(vector.y(), vector.z()));
    if (!has_attitude_estimate_) {
      roll_ = roll;
      pitch_ = pitch;
      initial_roll_ = roll;
      initial_pitch_ = pitch;
      has_attitude_estimate_ = true;
    } else {
      roll_ = (1.0 - low_pass_alpha_) * roll_ + low_pass_alpha_ * roll;
      pitch_ = (1.0 - low_pass_alpha_) * pitch_ + low_pass_alpha_ * pitch;
    }

    last_imu_frame_ = msg->header.frame_id;
  }

  std::optional<tf2::Vector3> accelerationInBaseFrame(const sensor_msgs::msg::Imu & msg)
  {
    const auto source_frame = front_imu_frame_id_.empty() ? msg.header.frame_id : front_imu_frame_id_;
    last_transform_frame_ = source_frame;
    tf2::Vector3 vector(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    if (source_frame.empty() || source_frame == base_frame_id_) {
      return vector;
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(base_frame_id_, source_frame, tf2::TimePointZero);
      tf2::Quaternion rotation;
      tf2::fromMsg(transform.transform.rotation, rotation);
      return tf2::quatRotate(rotation, vector);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Waiting for IMU TF %s -> %s: %s",
        source_frame.c_str(),
        base_frame_id_.c_str(),
        ex.what());
      return std::nullopt;
    }
  }

  void publishAttitudeCorrection()
  {
    if (!publish_attitude_correction_) {
      return;
    }
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_id_;
    if (has_attitude_estimate_) {
      msg.vector.x = roll_sign_ * (roll_ - initial_roll_);
      msg.vector.y = pitch_sign_ * (pitch_ - initial_pitch_);
    }
    attitude_correction_pub_->publish(msg);
  }

  void publishHeartbeat()
  {
    std_msgs::msg::String msg;
    if (publish_attitude_correction_ && !has_attitude_estimate_) {
      msg.data = "waiting_for_imu";
    } else {
      const double roll = has_attitude_estimate_ ? roll_sign_ * (roll_ - initial_roll_) : 0.0;
      const double pitch = has_attitude_estimate_ ? pitch_sign_ * (pitch_ - initial_pitch_) : 0.0;
      std::ostringstream out;
      out << std::fixed << std::setprecision(6)
          << "ready:roll=" << roll
          << ":pitch=" << pitch;
      msg.data = out.str();
    }
    heartbeat_pub_->publish(msg);
  }

  void publishStatus()
  {
    std::ostringstream out;
    out << "{\"base_frame_id\":\"" << jsonEscape(base_frame_id_) << "\",";
    out << "\"imu_topic\":\"" << jsonEscape(front_imu_topic_) << "\",";
    out << "\"imu_frame_id\":\"" << jsonEscape(front_imu_frame_id_) << "\",";
    out << "\"runtime\":\"" << (simulation_ ? "simulation" : "real") << "\",";
    out << "\"status\":\"ready\",";
    out << "\"bindings\":[";
    for (std::size_t i = 0; i < cameras_.size(); ++i) {
      const auto & camera = cameras_[i];
      if (i > 0) {
        out << ",";
      }
      out << "{";
      out << "\"role\":\"" << jsonEscape(camera.role) << "\",";
      out << "\"camera_name\":\"" << jsonEscape(camera.camera_name) << "\",";
      out << "\"frame\":\"" << jsonEscape(camera.frame) << "\",";
      out << "\"depth_topic\":\"" << jsonEscape(camera.depth_topic) << "\",";
      out << "\"camera_info_topic\":\"" << jsonEscape(camera.camera_info_topic) << "\",";
      out << "\"imu_topic\":\"" << jsonEscape(camera.imu_topic) << "\"";
      out << "}";
    }
    out << "]}";

    std_msgs::msg::String msg;
    msg.data = out.str();
    binding_pub_->publish(msg);
  }

  bool simulation_{false};
  bool publish_static_tf_{true};
  bool publish_attitude_correction_{true};
  double low_pass_alpha_{0.08};
  double min_accel_norm_{3.0};
  double max_accel_norm_{20.0};
  double roll_sign_{-1.0};
  double pitch_sign_{-1.0};
  double roll_{0.0};
  double pitch_{0.0};
  double initial_roll_{0.0};
  double initial_pitch_{0.0};
  bool has_attitude_estimate_{false};
  std::string model_{"f4"};
  std::string frame_prefix_{"f4/"};
  std::string topic_prefix_{"/f4"};
  std::string urdf_path_;
  std::string base_frame_id_{"f4/base_link"};
  std::string attitude_correction_topic_{"~/attitude_correction"};
  std::string front_imu_topic_;
  std::string front_imu_frame_id_;
  std::string last_imu_frame_;
  std::string last_transform_frame_;
  std::vector<CameraBinding> cameras_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr binding_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_correction_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr correction_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
};

}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::DriveMapperNode>());
  rclcpp::shutdown();
  return 0;
}
