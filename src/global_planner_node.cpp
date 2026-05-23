#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace autonomy
{
namespace
{

constexpr int kUnknown = -1;
constexpr int kLethal = 100;

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

struct GridCell
{
  int x{0};
  int y{0};
};

struct QueueNode
{
  int index{0};
  double priority{0.0};

  bool operator<(const QueueNode & other) const
  {
    return priority > other.priority;
  }
};

}  // namespace

class GlobalPlannerNode final : public rclcpp::Node
{
public:
  GlobalPlannerNode()
  : Node("global_planner_node")
  {
    declare_parameter<bool>("enabled", false);
    declare_parameter<std::string>("planner_backend", "astar");
    declare_parameter<std::string>("current_pose_topic", "/localization/current_pose");
    declare_parameter<std::string>("goal_topic", "/goal_pose");
    declare_parameter<std::string>("map_topic", "/planning/global_costmap");
    declare_parameter<std::string>("map_file", "");
    declare_parameter<std::string>("path_topic", "/planning/global_path");
    declare_parameter<std::string>("target_pose_topic", "/planning/target_pose");
    declare_parameter<bool>("use_example_map", true);
    declare_parameter<std::string>("map_frame_id", "map");
    declare_parameter<int>("example_map_width", 120);
    declare_parameter<int>("example_map_height", 80);
    declare_parameter<double>("example_map_resolution", 0.1);
    declare_parameter<double>("example_map_origin_x", -2.0);
    declare_parameter<double>("example_map_origin_y", -4.0);
    declare_parameter<bool>("allow_diagonal", true);
    declare_parameter<bool>("path_smoothing", true);
    declare_parameter<double>("local_target_lookahead", 1.0);
    declare_parameter<int>("lethal_cost_threshold", 100);
    declare_parameter<std::string>("unknown_policy", "avoid");
    declare_parameter<int>("unknown_cost", 80);
    declare_parameter<double>("cost_weight", 3.0);
    declare_parameter<double>("publish_rate_hz", 2.0);

    loadParameters();
    createIo();
    if (use_example_map_) {
      map_ = createExampleMap();
      has_map_ = true;
      map_pub_->publish(map_);
    } else if (loadMapFromFile()) {
      has_map_ = true;
      map_pub_->publish(map_);
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { tick(); });
  }

private:
  void loadParameters()
  {
    enabled_ = get_parameter("enabled").as_bool();
    planner_backend_ = get_parameter("planner_backend").as_string();
    use_example_map_ = get_parameter("use_example_map").as_bool();
    map_file_ = get_parameter("map_file").as_string();
    map_frame_id_ = get_parameter("map_frame_id").as_string();
    example_map_width_ = static_cast<int>(get_parameter("example_map_width").as_int());
    example_map_height_ = static_cast<int>(get_parameter("example_map_height").as_int());
    example_map_resolution_ = get_parameter("example_map_resolution").as_double();
    example_map_origin_x_ = get_parameter("example_map_origin_x").as_double();
    example_map_origin_y_ = get_parameter("example_map_origin_y").as_double();
    allow_diagonal_ = get_parameter("allow_diagonal").as_bool();
    path_smoothing_ = get_parameter("path_smoothing").as_bool();
    local_target_lookahead_ = get_parameter("local_target_lookahead").as_double();
    lethal_cost_threshold_ = static_cast<int>(get_parameter("lethal_cost_threshold").as_int());
    unknown_policy_ = get_parameter("unknown_policy").as_string();
    unknown_cost_ = static_cast<int>(get_parameter("unknown_cost").as_int());
    cost_weight_ = get_parameter("cost_weight").as_double();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  }

  void createIo()
  {
    current_pose_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
      get_parameter("current_pose_topic").as_string(),
      10,
      [this](geometry_msgs::msg::Pose2D::SharedPtr msg) {
        current_pose_ = *msg;
        has_current_pose_ = true;
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("goal_topic").as_string(),
      10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        latest_goal_ = std::move(msg);
        needs_replan_ = true;
      });
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("map_topic").as_string(),
      1,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (!use_example_map_) {
          map_ = *msg;
          has_map_ = true;
          needs_replan_ = true;
        }
      });
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      get_parameter("map_topic").as_string(),
      rclcpp::QoS(1).transient_local());
    path_pub_ = create_publisher<nav_msgs::msg::Path>(get_parameter("path_topic").as_string(), 10);
    target_pose_pub_ = create_publisher<geometry_msgs::msg::Pose2D>(
      get_parameter("target_pose_topic").as_string(),
      10);
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/global_planner_node",
      10);
  }

  void tick()
  {
    std_msgs::msg::String heartbeat;
    if (!enabled_) {
      heartbeat.data = "disabled";
      heartbeat_pub_->publish(heartbeat);
      return;
    }
    if (use_example_map_ && has_map_) {
      map_.header.stamp = now();
      map_pub_->publish(map_);
    }
    if (!has_map_ || !has_current_pose_ || !latest_goal_) {
      heartbeat.data = "waiting_for_inputs";
      heartbeat_pub_->publish(heartbeat);
      return;
    }
    if (planner_backend_ != "astar") {
      heartbeat.data = "astar:fallback_backend";
    } else {
      heartbeat.data = "astar";
    }

    if (needs_replan_) {
      latest_path_ = planAstar();
      needs_replan_ = false;
    }
    if (!latest_path_.poses.empty()) {
      latest_path_.header.stamp = now();
      path_pub_->publish(latest_path_);
      publishLocalTarget(latest_path_);
    } else {
      heartbeat.data += ":no_path";
    }
    heartbeat_pub_->publish(heartbeat);
  }

  [[nodiscard]] nav_msgs::msg::Path planAstar() const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_.header.frame_id.empty() ? map_frame_id_ : map_.header.frame_id;
    path.header.stamp = now();

    const auto start = worldToCell(current_pose_.x, current_pose_.y);
    const auto goal = worldToCell(latest_goal_->pose.position.x, latest_goal_->pose.position.y);
    if (!start || !goal || !isTraversable(cellToIndex(*start)) || !isTraversable(cellToIndex(*goal))) {
      return path;
    }

    const int cell_count = static_cast<int>(map_.info.width * map_.info.height);
    std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
    std::vector<int> parent(cell_count, -1);
    std::priority_queue<QueueNode> open;
    const int start_index = cellToIndex(*start);
    const int goal_index = cellToIndex(*goal);
    g_score[start_index] = 0.0;
    open.push({start_index, heuristic(*start, *goal)});

    while (!open.empty()) {
      const auto current = open.top();
      open.pop();
      if (current.index == goal_index) {
        return cellsToPath(smoothPath(reconstructPath(parent, goal_index)));
      }

      const auto current_cell = indexToCell(current.index);
      for (const auto & neighbor : neighbors(current_cell)) {
        const int neighbor_index = cellToIndex(neighbor);
        if (!isTraversable(neighbor_index)) {
          continue;
        }
        const double move_cost = heuristic(current_cell, neighbor) * traversalCost(neighbor_index);
        const double tentative_g = g_score[current.index] + move_cost;
        if (tentative_g < g_score[neighbor_index]) {
          parent[neighbor_index] = current.index;
          g_score[neighbor_index] = tentative_g;
          open.push({neighbor_index, tentative_g + heuristic(neighbor, *goal)});
        }
      }
    }

    return path;
  }

  [[nodiscard]] std::vector<int> reconstructPath(const std::vector<int> & parent, int index) const
  {
    std::vector<int> path;
    while (index >= 0) {
      path.push_back(index);
      index = parent[index];
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  [[nodiscard]] std::vector<int> smoothPath(const std::vector<int> & path) const
  {
    if (!path_smoothing_ || path.size() < 3) {
      return path;
    }
    std::vector<int> smoothed;
    std::size_t anchor = 0;
    smoothed.push_back(path.front());
    while (anchor + 1 < path.size()) {
      std::size_t next = path.size() - 1;
      while (next > anchor + 1 && !hasLineOfSight(path[anchor], path[next])) {
        --next;
      }
      smoothed.push_back(path[next]);
      anchor = next;
    }
    return smoothed;
  }

  [[nodiscard]] bool hasLineOfSight(const int start_index, const int end_index) const
  {
    auto start = indexToCell(start_index);
    auto end = indexToCell(end_index);
    int x0 = start.x;
    int y0 = start.y;
    const int x1 = end.x;
    const int y1 = end.y;
    const int dx = std::abs(x1 - x0);
    const int dy = -std::abs(y1 - y0);
    const int sx = x0 < x1 ? 1 : -1;
    const int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    while (true) {
      if (!isTraversable(cellToIndex({x0, y0}))) {
        return false;
      }
      if (x0 == x1 && y0 == y1) {
        return true;
      }
      const int error2 = 2 * error;
      if (error2 >= dy) {
        error += dy;
        x0 += sx;
      }
      if (error2 <= dx) {
        error += dx;
        y0 += sy;
      }
    }
  }

  [[nodiscard]] nav_msgs::msg::Path cellsToPath(const std::vector<int> & cells) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_.header.frame_id.empty() ? map_frame_id_ : map_.header.frame_id;
    path.header.stamp = now();
    for (const int index : cells) {
      const auto cell = indexToCell(index);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      const auto world = cellToWorld(cell);
      pose.pose.position.x = world.first;
      pose.pose.position.y = world.second;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    if (!path.poses.empty()) {
      path.poses.back().pose.orientation = latest_goal_->pose.orientation;
    }
    return path;
  }

  void publishLocalTarget(const nav_msgs::msg::Path & path) const
  {
    if (path.poses.empty()) {
      return;
    }

    geometry_msgs::msg::Pose2D target;
    const auto selected_index = selectLookaheadIndex(path);
    const auto & selected = path.poses[selected_index].pose.position;
    target.x = selected.x;
    target.y = selected.y;
    if (selected_index + 1 < path.poses.size()) {
      const auto & next = path.poses[selected_index + 1].pose.position;
      target.theta = std::atan2(next.y - selected.y, next.x - selected.x);
    } else {
      target.theta = current_pose_.theta;
    }
    target_pose_pub_->publish(target);
  }

  [[nodiscard]] std::size_t selectLookaheadIndex(const nav_msgs::msg::Path & path) const
  {
    for (std::size_t i = 0; i < path.poses.size(); ++i) {
      const auto & p = path.poses[i].pose.position;
      if (std::hypot(p.x - current_pose_.x, p.y - current_pose_.y) >= local_target_lookahead_) {
        return i;
      }
    }
    return path.poses.size() - 1;
  }

  [[nodiscard]] std::vector<GridCell> neighbors(const GridCell & cell) const
  {
    static constexpr int kFourConnected[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    static constexpr int kEightConnected[8][2] = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    const auto & offsets = allow_diagonal_ ? kEightConnected : kFourConnected;
    const int count = allow_diagonal_ ? 8 : 4;
    std::vector<GridCell> result;
    result.reserve(count);
    for (int i = 0; i < count; ++i) {
      GridCell neighbor{cell.x + offsets[i][0], cell.y + offsets[i][1]};
      if (isInside(neighbor)) {
        result.push_back(neighbor);
      }
    }
    return result;
  }

  [[nodiscard]] double heuristic(const GridCell & a, const GridCell & b) const
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  [[nodiscard]] double traversalCost(const int index) const
  {
    const int cost = normalizedCellCost(index);
    return 1.0 + cost_weight_ * (static_cast<double>(cost) / 100.0);
  }

  [[nodiscard]] bool isTraversable(const int index) const
  {
    if (index < 0 || index >= static_cast<int>(map_.data.size())) {
      return false;
    }
    const int cost = map_.data[static_cast<std::size_t>(index)];
    if (cost == kUnknown) {
      return unknown_policy_ != "avoid";
    }
    return cost < lethal_cost_threshold_ && cost < kLethal;
  }

  [[nodiscard]] int normalizedCellCost(const int index) const
  {
    const int cost = map_.data[static_cast<std::size_t>(index)];
    if (cost == kUnknown) {
      return std::clamp(unknown_cost_, 0, 99);
    }
    return std::clamp(cost, 0, 99);
  }

  [[nodiscard]] bool isInside(const GridCell & cell) const
  {
    return cell.x >= 0 && cell.y >= 0 &&
           cell.x < static_cast<int>(map_.info.width) &&
           cell.y < static_cast<int>(map_.info.height);
  }

  [[nodiscard]] int cellToIndex(const GridCell & cell) const
  {
    return cell.y * static_cast<int>(map_.info.width) + cell.x;
  }

  [[nodiscard]] GridCell indexToCell(const int index) const
  {
    return {index % static_cast<int>(map_.info.width), index / static_cast<int>(map_.info.width)};
  }

  [[nodiscard]] std::optional<GridCell> worldToCell(const double x, const double y) const
  {
    if (map_.info.resolution <= 0.0F) {
      return std::nullopt;
    }
    const int cell_x = static_cast<int>((x - map_.info.origin.position.x) / map_.info.resolution);
    const int cell_y = static_cast<int>((y - map_.info.origin.position.y) / map_.info.resolution);
    GridCell cell{cell_x, cell_y};
    if (!isInside(cell)) {
      return std::nullopt;
    }
    return cell;
  }

  [[nodiscard]] std::pair<double, double> cellToWorld(const GridCell & cell) const
  {
    return {
      map_.info.origin.position.x + (static_cast<double>(cell.x) + 0.5) * map_.info.resolution,
      map_.info.origin.position.y + (static_cast<double>(cell.y) + 0.5) * map_.info.resolution};
  }

  [[nodiscard]] nav_msgs::msg::OccupancyGrid createExampleMap() const
  {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = map_frame_id_;
    map.header.stamp = now();
    map.info.width = static_cast<std::uint32_t>(std::max(1, example_map_width_));
    map.info.height = static_cast<std::uint32_t>(std::max(1, example_map_height_));
    map.info.resolution = static_cast<float>(std::max(0.01, example_map_resolution_));
    map.info.origin.position.x = example_map_origin_x_;
    map.info.origin.position.y = example_map_origin_y_;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(static_cast<std::size_t>(map.info.width) * map.info.height, 0);

    addExampleWall(map, 45, 0, 45, 55);
    addExampleWall(map, 75, 25, 75, static_cast<int>(map.info.height) - 1);
    addExampleWall(map, 45, 55, 70, 55);
    inflateExampleMap(map, 5, 85);
    return map;
  }

  void addExampleWall(
    nav_msgs::msg::OccupancyGrid & map,
    const int x0,
    const int y0,
    const int x1,
    const int y1) const
  {
    const int dx = x1 == x0 ? 0 : (x1 > x0 ? 1 : -1);
    const int dy = y1 == y0 ? 0 : (y1 > y0 ? 1 : -1);
    int x = x0;
    int y = y0;
    while (true) {
      if (x >= 0 && y >= 0 && x < static_cast<int>(map.info.width) && y < static_cast<int>(map.info.height)) {
        map.data[static_cast<std::size_t>(y) * map.info.width + static_cast<std::size_t>(x)] = 100;
      }
      if (x == x1 && y == y1) {
        break;
      }
      x += dx;
      y += dy;
    }
  }

  void inflateExampleMap(nav_msgs::msg::OccupancyGrid & map, const int radius, const int inflated_cost) const
  {
    auto inflated = map.data;
    for (int y = 0; y < static_cast<int>(map.info.height); ++y) {
      for (int x = 0; x < static_cast<int>(map.info.width); ++x) {
        const auto index = static_cast<std::size_t>(y) * map.info.width + static_cast<std::size_t>(x);
        if (map.data[index] != 100) {
          continue;
        }
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= static_cast<int>(map.info.width) || ny >= static_cast<int>(map.info.height)) {
              continue;
            }
            if (std::hypot(dx, dy) > radius) {
              continue;
            }
            const auto nindex = static_cast<std::size_t>(ny) * map.info.width + static_cast<std::size_t>(nx);
            if (inflated[nindex] != 100) {
              inflated[nindex] = std::max<int8_t>(inflated[nindex], static_cast<int8_t>(inflated_cost));
            }
          }
        }
      }
    }
    map.data = inflated;
  }

  [[nodiscard]] bool loadMapFromFile()
  {
    if (map_file_.empty()) {
      return false;
    }

    const std::filesystem::path yaml_path(map_file_);
    std::ifstream yaml(yaml_path);
    if (!yaml) {
      RCLCPP_WARN(get_logger(), "Global map file does not exist yet: %s", map_file_.c_str());
      return false;
    }

    std::string image_name;
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    std::string line;
    while (std::getline(yaml, line)) {
      if (line.rfind("image:", 0) == 0) {
        image_name = trim(line.substr(6));
      } else if (line.rfind("resolution:", 0) == 0) {
        resolution = std::stod(trim(line.substr(11)));
      } else if (line.rfind("origin:", 0) == 0) {
        parseOrigin(line, origin_x, origin_y);
      }
    }

    if (image_name.empty() || resolution <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid global map yaml: %s", map_file_.c_str());
      return false;
    }

    std::filesystem::path image_path(image_name);
    if (image_path.is_relative()) {
      image_path = yaml_path.parent_path() / image_path;
    }
    return loadPgm(image_path, resolution, origin_x, origin_y);
  }

  [[nodiscard]] static std::string trim(const std::string & value)
  {
    const auto first = value.find_first_not_of(" \t\"'");
    if (first == std::string::npos) {
      return "";
    }
    const auto last = value.find_last_not_of(" \t\"'");
    return value.substr(first, last - first + 1);
  }

  static void parseOrigin(const std::string & line, double & x, double & y)
  {
    const auto begin = line.find('[');
    const auto end = line.find(']');
    if (begin == std::string::npos || end == std::string::npos || end <= begin) {
      return;
    }
    std::string values = line.substr(begin + 1, end - begin - 1);
    std::replace(values.begin(), values.end(), ',', ' ');
    std::istringstream stream(values);
    stream >> x >> y;
  }

  [[nodiscard]] bool loadPgm(
    const std::filesystem::path & image_path,
    const double resolution,
    const double origin_x,
    const double origin_y)
  {
    std::ifstream pgm(image_path, std::ios::binary);
    if (!pgm) {
      RCLCPP_WARN(get_logger(), "Global map image does not exist: %s", image_path.string().c_str());
      return false;
    }

    std::string magic;
    int width = 0;
    int height = 0;
    int max_value = 0;
    pgm >> magic >> width >> height >> max_value;
    pgm.get();
    if (magic != "P5" || width <= 0 || height <= 0 || max_value <= 0) {
      RCLCPP_WARN(get_logger(), "Unsupported PGM map image: %s", image_path.string().c_str());
      return false;
    }

    nav_msgs::msg::OccupancyGrid loaded;
    loaded.header.frame_id = map_frame_id_;
    loaded.header.stamp = now();
    loaded.info.width = static_cast<std::uint32_t>(width);
    loaded.info.height = static_cast<std::uint32_t>(height);
    loaded.info.resolution = static_cast<float>(resolution);
    loaded.info.origin.position.x = origin_x;
    loaded.info.origin.position.y = origin_y;
    loaded.info.origin.orientation.w = 1.0;
    loaded.data.assign(static_cast<std::size_t>(width) * height, 0);

    for (int image_row = 0; image_row < height; ++image_row) {
      for (int col = 0; col < width; ++col) {
        unsigned char pixel = 254;
        pgm.read(reinterpret_cast<char *>(&pixel), 1);
        const int map_row = height - 1 - image_row;
        const auto index = static_cast<std::size_t>(map_row) * width + static_cast<std::size_t>(col);
        loaded.data[index] = pixel < 65 ? 100 : 0;
      }
    }

    map_ = loaded;
    return true;
  }

  bool enabled_{false};
  bool use_example_map_{true};
  bool allow_diagonal_{true};
  bool path_smoothing_{true};
  bool has_map_{false};
  bool has_current_pose_{false};
  bool needs_replan_{true};
  std::string planner_backend_{"astar"};
  std::string map_frame_id_{"map"};
  std::string map_file_;
  std::string unknown_policy_{"avoid"};
  int example_map_width_{120};
  int example_map_height_{80};
  double example_map_resolution_{0.1};
  double example_map_origin_x_{-2.0};
  double example_map_origin_y_{-4.0};
  double local_target_lookahead_{1.0};
  int lethal_cost_threshold_{100};
  int unknown_cost_{80};
  double cost_weight_{3.0};
  double publish_rate_hz_{2.0};

  geometry_msgs::msg::Pose2D current_pose_;
  geometry_msgs::msg::PoseStamped::SharedPtr latest_goal_;
  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::Path latest_path_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
