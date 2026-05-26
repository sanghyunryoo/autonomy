#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <dds/dds.h>
#include <rclcpp/rclcpp.hpp>

#include <core/msg/command_filter.hpp>
#include <core/msg/command_user.hpp>
#include <core/msg/event_user.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "HeightMap.h"
#include "LinearVelocity.h"

namespace
{

using namespace std::chrono_literals;

std::atomic_bool g_running{true};

void onSignal(int)
{
  g_running = false;
}

struct Options
{
  int ros_domain_id{0};
  dds_domainid_t dds_domain_id{1};
  std::string command_filter_topic{"/command_filter"};
  std::string user_odom_topic{"/control_command/user_odom"};
  std::string height_map_topic{"height_map"};
  std::string lin_vel_topic{"lin_vel"};
  double ros_hz{10.0};
  double height_map_hz{50.0};
  double lin_vel_hz{10.0};
  std::size_t height_map_count{144};
  bool filter_default{true};
  bool fixed_lin_vel{false};
  std::array<float, 3> lin_vel_value{1.0F, 1.0F, 1.0F};
};

struct MotionState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double linear_x{0.0};
  double angular_z{0.0};
  std::array<bool, 4> filter{true, true, true, true};
};

class TerminalRawMode
{
public:
  TerminalRawMode()
  {
    enabled_ = isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &old_attr_) == 0;
    if (!enabled_) {
      return;
    }

    auto new_attr = old_attr_;
    new_attr.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    new_attr.c_cc[VMIN] = 0;
    new_attr.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_attr);

    old_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, old_flags_ | O_NONBLOCK);
  }

  ~TerminalRawMode()
  {
    if (enabled_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &old_attr_);
      fcntl(STDIN_FILENO, F_SETFL, old_flags_);
    }
  }

private:
  bool enabled_{false};
  int old_flags_{0};
  termios old_attr_{};
};

void printUsage(const char * argv0)
{
  std::cout
    << "Usage: " << argv0 << " [options]\n\n"
    << "Options:\n"
    << "  --ros-domain-id N       ROS_DOMAIN_ID. Default: 0\n"
    << "  --dds-domain-id N       CycloneDDS domain id. Default: 1\n"
    << "  --filter-topic NAME     CommandFilter topic. Default: /command_filter\n"
    << "  --odom-topic NAME       CommandUser odom topic. Default: /control_command/user_odom\n"
    << "  --height-map-topic NAME DDS HeightMap topic. Default: height_map\n"
    << "  --lin-vel-topic NAME    DDS LinearVelocity topic. Default: lin_vel\n"
    << "  --height-map-count N    HeightMap data length. Default: 144\n"
    << "  --lin-vel X Y Z         Publish fixed DDS lin_vel value instead of keyboard command\n"
    << "  --lin-vel-from-keyboard Publish DDS lin_vel from keyboard command. Default\n"
    << "  --filter-default true|false\n";
}

bool takeArg(int & index, const int argc, char ** argv, std::string & value)
{
  if (index + 1 >= argc) {
    std::cerr << "missing value for " << argv[index] << '\n';
    return false;
  }
  value = argv[++index];
  return true;
}

bool parseBool(const std::string & value)
{
  return value == "1" || value == "true" || value == "TRUE" || value == "on";
}

bool parseOptions(const int argc, char ** argv, Options & options)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg{argv[i]};
    std::string value;
    if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      return false;
    } else if (arg == "--ros-domain-id" && takeArg(i, argc, argv, value)) {
      options.ros_domain_id = std::stoi(value);
    } else if (arg == "--dds-domain-id" && takeArg(i, argc, argv, value)) {
      options.dds_domain_id = static_cast<dds_domainid_t>(std::stoul(value));
    } else if (arg == "--filter-topic" && takeArg(i, argc, argv, value)) {
      options.command_filter_topic = value;
    } else if (arg == "--odom-topic" && takeArg(i, argc, argv, value)) {
      options.user_odom_topic = value;
    } else if (arg == "--height-map-topic" && takeArg(i, argc, argv, value)) {
      options.height_map_topic = value;
    } else if (arg == "--lin-vel-topic" && takeArg(i, argc, argv, value)) {
      options.lin_vel_topic = value;
    } else if (arg == "--height-map-count" && takeArg(i, argc, argv, value)) {
      options.height_map_count = std::stoul(value);
    } else if (arg == "--lin-vel") {
      if (i + 3 >= argc) {
        std::cerr << "missing values for --lin-vel X Y Z\n";
        return false;
      }
      options.lin_vel_value[0] = std::stof(argv[++i]);
      options.lin_vel_value[1] = std::stof(argv[++i]);
      options.lin_vel_value[2] = std::stof(argv[++i]);
      options.fixed_lin_vel = true;
    } else if (arg == "--lin-vel-from-keyboard") {
      options.fixed_lin_vel = false;
    } else if (arg == "--filter-default" && takeArg(i, argc, argv, value)) {
      options.filter_default = parseBool(value);
    } else {
      std::cerr << "unknown option: " << arg << '\n';
      printUsage(argv[0]);
      return false;
    }
  }
  return true;
}

std::string ddsError(const char * action, const int ret)
{
  return std::string(action) + ": " + dds_strretcode(-ret);
}

dds_entity_t createBestEffortWriter(const dds_entity_t participant, const dds_entity_t topic)
{
  dds_qos_t * qos = dds_create_qos();
  if (qos == nullptr) {
    return DDS_RETCODE_ERROR;
  }
  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(0));
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
  const dds_entity_t writer = dds_create_writer(participant, topic, qos, nullptr);
  dds_delete_qos(qos);
  return writer;
}

geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

core::msg::CommandFilter makeFilterMsg(const MotionState & state)
{
  core::msg::CommandFilter msg;
  msg.allow_linear_vel_forward_x = state.filter[0];
  msg.allow_linear_vel_backward_x = state.filter[1];
  msg.allow_linear_vel_forward_y = state.filter[2];
  msg.allow_linear_vel_backward_y = state.filter[3];
  return msg;
}

core::msg::CommandUser makeCommandUserMsg(
  rclcpp::Node & node,
  const MotionState & state)
{
  core::msg::CommandUser msg;
  msg.odom.header.stamp = node.get_clock()->now();
  msg.odom.header.frame_id = "odom";
  msg.odom.child_frame_id = "base_link";
  msg.odom.pose.pose.position.x = state.x;
  msg.odom.pose.pose.position.y = state.y;
  msg.odom.pose.pose.position.z = 0.0;
  msg.odom.pose.pose.orientation = yawToQuaternion(state.yaw);
  msg.odom.twist.twist.linear.x = state.linear_x;
  msg.odom.twist.twist.linear.y = 0.0;
  msg.odom.twist.twist.linear.z = 0.0;
  msg.odom.twist.twist.angular.z = state.angular_z;

  msg.event.estop = false;
  msg.event.wake = false;
  msg.event.sleep = false;
  msg.event.rough_drive_toggle = false;
  return msg;
}

void fillHeightMap(std::vector<float> & data, const std::uint64_t tick)
{
  const float phase = static_cast<float>(tick) * 0.02F;
  for (std::size_t i = 0; i < data.size(); ++i) {
    data[i] = 0.10F * std::sin(phase + static_cast<float>(i) * 0.04F);
  }
}

int writeHeightMap(const dds_entity_t writer, std::vector<float> & data)
{
  core_dds_HeightMap sample{};
  sample.data._maximum = static_cast<std::uint32_t>(data.size());
  sample.data._length = static_cast<std::uint32_t>(data.size());
  sample.data._buffer = data.data();
  sample.data._release = false;
  return dds_write(writer, &sample);
}

int writeLinVel(
  const dds_entity_t writer,
  const MotionState & state,
  const Options & options)
{
  std::array<float, 3> data = options.fixed_lin_vel ?
    options.lin_vel_value :
    std::array<float, 3>{static_cast<float>(state.linear_x), 0.0F, 0.0F};
  core_dds_LinearVelocity sample{};
  sample.data._maximum = static_cast<std::uint32_t>(data.size());
  sample.data._length = static_cast<std::uint32_t>(data.size());
  sample.data._buffer = data.data();
  sample.data._release = false;
  return dds_write(writer, &sample);
}

void handleKey(MotionState & state, const char key)
{
  constexpr double kSpeedStep = 0.1;
  constexpr double kMaxLinearX = 1.0;
  constexpr double kMaxAngularZ = 1.0;

  switch (key) {
    case '1':
      state.filter[0] = !state.filter[0];
      break;
    case '2':
      state.filter[1] = !state.filter[1];
      break;
    case '3':
      state.filter[2] = !state.filter[2];
      break;
    case '4':
      state.filter[3] = !state.filter[3];
      break;
    case 'w':
    case 'W':
      state.linear_x = std::clamp(state.linear_x + kSpeedStep, -kMaxLinearX, kMaxLinearX);
      state.angular_z = 0.0;
      break;
    case 's':
    case 'S':
      state.linear_x = std::clamp(state.linear_x - kSpeedStep, -kMaxLinearX, kMaxLinearX);
      state.angular_z = 0.0;
      break;
    case 'a':
    case 'A':
      state.linear_x = 0.0;
      state.angular_z = std::clamp(state.angular_z + kSpeedStep, -kMaxAngularZ, kMaxAngularZ);
      break;
    case 'd':
    case 'D':
      state.linear_x = 0.0;
      state.angular_z = std::clamp(state.angular_z - kSpeedStep, -kMaxAngularZ, kMaxAngularZ);
      break;
    case ' ':
      state.linear_x = 0.0;
      state.angular_z = 0.0;
      break;
    default:
      break;
  }
}

void printGuide(const Options & options)
{
  std::cout
    << "\nIntegrated keyboard ROS/DDS test\n"
    << "--------------------------------\n"
    << "ROS  domain: " << options.ros_domain_id << '\n'
    << "DDS  domain: " << options.dds_domain_id << '\n'
    << "ROS  filter: " << options.command_filter_topic << " @ " << options.ros_hz << "Hz\n"
    << "ROS  odom  : " << options.user_odom_topic << " @ " << options.ros_hz << "Hz\n"
    << "DDS  map   : " << options.height_map_topic << " @ " << options.height_map_hz << "Hz, len="
    << options.height_map_count << '\n'
    << "DDS  vel   : " << options.lin_vel_topic << " @ " << options.lin_vel_hz << "Hz";
  if (options.fixed_lin_vel) {
    std::cout
      << " fixed=[" << options.lin_vel_value[0] << ',' << options.lin_vel_value[1] << ','
      << options.lin_vel_value[2] << ']';
  } else {
    std::cout << " from_keyboard";
  }
  std::cout << "\n\n"
    << "1: toggle forward_x filter\n"
    << "2: toggle backward_x filter\n"
    << "3: toggle forward_y filter\n"
    << "4: toggle backward_y filter\n"
    << "w/s: forward/backward velocity\n"
    << "a/d: rotate left/right\n"
    << "space: stop motion\n"
    << "q or ESC: quit\n\n";
}

void printState(const MotionState & state)
{
  std::cout
    << std::fixed << std::setprecision(2)
    << "filter=[" << state.filter[0] << ',' << state.filter[1] << ','
    << state.filter[2] << ',' << state.filter[3] << "] "
    << "lin_x=" << state.linear_x
    << " ang_z=" << state.angular_z
    << " pose=(" << state.x << ',' << state.y << ',' << state.yaw << ")\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  if (!parseOptions(argc, argv, options)) {
    return 2;
  }

  setenv("ROS_DOMAIN_ID", std::to_string(options.ros_domain_id).c_str(), 1);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  const dds_entity_t participant = dds_create_participant(options.dds_domain_id, nullptr, nullptr);
  if (participant < 0) {
    std::cerr << ddsError("failed to create DDS participant", participant) << '\n';
    return 1;
  }

  const dds_entity_t height_map_topic = dds_create_topic(
    participant, &core_dds_HeightMap_desc, options.height_map_topic.c_str(), nullptr, nullptr);
  const dds_entity_t lin_vel_topic = dds_create_topic(
    participant, &core_dds_LinearVelocity_desc, options.lin_vel_topic.c_str(), nullptr, nullptr);
  if (height_map_topic < 0 || lin_vel_topic < 0) {
    std::cerr << "failed to create DDS topics\n";
    dds_delete(participant);
    return 1;
  }

  const dds_entity_t height_map_writer = createBestEffortWriter(participant, height_map_topic);
  const dds_entity_t lin_vel_writer = createBestEffortWriter(participant, lin_vel_topic);
  if (height_map_writer < 0 || lin_vel_writer < 0) {
    std::cerr << "failed to create DDS writers\n";
    dds_delete(participant);
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("keyboard_filter_odom_dds_test");

  auto filter_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto command_qos = rclcpp::QoS(rclcpp::KeepLast(16)).reliable().durability_volatile();
  auto filter_pub = node->create_publisher<core::msg::CommandFilter>(
    options.command_filter_topic, filter_qos);
  auto command_pub = node->create_publisher<core::msg::CommandUser>(
    options.user_odom_topic, command_qos);

  TerminalRawMode terminal;
  MotionState state;
  state.filter.fill(options.filter_default);
  std::vector<float> height_map(options.height_map_count, 0.0F);

  printGuide(options);
  printState(state);

  using clock = std::chrono::steady_clock;
  auto last_pose_update = clock::now();
  auto next_ros = last_pose_update;
  auto next_height_map = last_pose_update;
  auto next_lin_vel = last_pose_update;
  auto next_report = last_pose_update + 1s;
  std::uint64_t height_map_ticks = 0;

  const auto ros_period = std::chrono::duration<double>(1.0 / options.ros_hz);
  const auto height_map_period = std::chrono::duration<double>(1.0 / options.height_map_hz);
  const auto lin_vel_period = std::chrono::duration<double>(1.0 / options.lin_vel_hz);

  while (g_running && rclcpp::ok()) {
    char key = '\0';
    while (read(STDIN_FILENO, &key, 1) == 1) {
      if (key == 'q' || key == 'Q' || key == '\x1b') {
        g_running = false;
        break;
      }
      handleKey(state, key);
      printState(state);
    }

    const auto now = clock::now();
    const std::chrono::duration<double> dt = now - last_pose_update;
    last_pose_update = now;
    state.x += state.linear_x * std::cos(state.yaw) * dt.count();
    state.y += state.linear_x * std::sin(state.yaw) * dt.count();
    state.yaw += state.angular_z * dt.count();

    if (now >= next_ros) {
      filter_pub->publish(makeFilterMsg(state));
      command_pub->publish(makeCommandUserMsg(*node, state));
      next_ros += std::chrono::duration_cast<clock::duration>(ros_period);
    }

    if (now >= next_height_map) {
      fillHeightMap(height_map, height_map_ticks++);
      const int ret = writeHeightMap(height_map_writer, height_map);
      if (ret < 0) {
        std::cerr << ddsError("failed to write height_map", ret) << '\n';
      }
      next_height_map += std::chrono::duration_cast<clock::duration>(height_map_period);
    }

    if (now >= next_lin_vel) {
      const int ret = writeLinVel(lin_vel_writer, state, options);
      if (ret < 0) {
        std::cerr << ddsError("failed to write lin_vel", ret) << '\n';
      }
      next_lin_vel += std::chrono::duration_cast<clock::duration>(lin_vel_period);
    }

    if (now >= next_report) {
      printState(state);
      next_report += 1s;
    }

    rclcpp::spin_some(node);
    std::this_thread::sleep_for(2ms);
  }

  state.linear_x = 0.0;
  state.angular_z = 0.0;
  filter_pub->publish(makeFilterMsg(state));
  command_pub->publish(makeCommandUserMsg(*node, state));
  writeLinVel(lin_vel_writer, state, options);

  dds_delete(participant);
  rclcpp::shutdown();
  std::cout << "\nIntegrated keyboard ROS/DDS test stopped.\n";
  return 0;
}
