#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <dds/dds.h>

#include "HeightMap.h"
#include "LinearVelocity.h"

namespace
{

std::atomic_bool g_running{true};

void onSignal(int)
{
  g_running = false;
}

struct Options
{
  dds_domainid_t domain_id{1};
  std::string height_map_topic{"height_map"};
  std::string linear_velocity_topic{"lin_vel"};
  std::string type{"both"};
  std::size_t max_values{12};
  std::size_t height_map_width{12};
  std::size_t height_map_height{12};
  double refresh_hz{4.0};
  std::string csv_path;
  bool render{true};
};

void printUsage(const char * argv0)
{
  std::cout
    << "Usage: " << argv0 << " [options]\n"
    << "\n"
    << "Options:\n"
    << "  --domain-id N                 DDS domain id. Default: 1\n"
    << "  --type both|height_map|linear_velocity\n"
    << "  --height-map-topic NAME       HeightMap topic. Default: height_map\n"
    << "  --height-map-width N          HeightMap grid width. Default: 12\n"
    << "  --height-map-height N         HeightMap grid height. Default: 12\n"
    << "  --linear-velocity-topic NAME  LinearVelocity topic. Default: lin_vel\n"
    << "  --max-values N                Values printed per sample. Default: 12\n"
    << "  --refresh-hz HZ               Table refresh rate. Default: 4\n"
    << "  --csv PATH                    Save height_map rows as time_ns,s_t_* CSV\n"
    << "  --no-render                   Disable terminal table output\n";
}

std::size_t envSizeT(const char * name, const std::size_t fallback)
{
  const char * value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') {
    return fallback;
  }
  try {
    const auto parsed = std::stoul(value);
    return parsed > 0 ? parsed : fallback;
  } catch (...) {
    return fallback;
  }
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

bool parseOptions(const int argc, char ** argv, Options & options)
{
  options.height_map_width = envSizeT("AUTONOMY_DDS_ECHO_HEIGHT_MAP_WIDTH", options.height_map_width);
  options.height_map_height = envSizeT("AUTONOMY_DDS_ECHO_HEIGHT_MAP_HEIGHT", options.height_map_height);

  for (int i = 1; i < argc; ++i) {
    const std::string arg{argv[i]};
    std::string value;
    if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      return false;
    } else if (arg == "--domain-id" && takeArg(i, argc, argv, value)) {
      options.domain_id = static_cast<dds_domainid_t>(std::stoul(value));
    } else if (arg == "--type" && takeArg(i, argc, argv, value)) {
      options.type = value;
    } else if (arg == "--height-map-topic" && takeArg(i, argc, argv, value)) {
      options.height_map_topic = value;
    } else if (arg == "--height-map-width" && takeArg(i, argc, argv, value)) {
      options.height_map_width = std::stoul(value);
    } else if (arg == "--height-map-height" && takeArg(i, argc, argv, value)) {
      options.height_map_height = std::stoul(value);
    } else if (arg == "--linear-velocity-topic" && takeArg(i, argc, argv, value)) {
      options.linear_velocity_topic = value;
    } else if (arg == "--max-values" && takeArg(i, argc, argv, value)) {
      options.max_values = std::stoul(value);
    } else if (arg == "--refresh-hz" && takeArg(i, argc, argv, value)) {
      options.refresh_hz = std::stod(value);
    } else if (arg == "--csv" && takeArg(i, argc, argv, value)) {
      options.csv_path = value;
    } else if (arg == "--no-render") {
      options.render = false;
    } else {
      std::cerr << "unknown option: " << arg << '\n';
      printUsage(argv[0]);
      return false;
    }
  }

  if (options.type != "both" && options.type != "height_map" && options.type != "linear_velocity") {
    std::cerr << "--type must be one of: both, height_map, linear_velocity\n";
    return false;
  }
  if (options.refresh_hz <= 0.0) {
    std::cerr << "--refresh-hz must be > 0\n";
    return false;
  }
  if (options.height_map_width == 0 || options.height_map_height == 0) {
    std::cerr << "--height-map-width and --height-map-height must be > 0\n";
    return false;
  }
  return true;
}

std::string ddsError(const char * action, const int ret)
{
  return std::string(action) + ": " + dds_strretcode(-ret);
}

dds_entity_t createBestEffortReader(
  const dds_entity_t participant,
  const dds_entity_t topic)
{
  dds_qos_t * qos = dds_create_qos();
  if (qos == nullptr) {
    std::cerr << "failed to allocate reader QoS\n";
    return DDS_RETCODE_ERROR;
  }

  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(0));
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 512);

  const dds_entity_t reader = dds_create_reader(participant, topic, qos, nullptr);
  dds_delete_qos(qos);
  return reader;
}

struct TopicStats
{
  std::size_t length{0};
  std::vector<float> values;
  std::uint64_t total{0};
  std::uint64_t window_count{0};
  double hz{0.0};
  std::chrono::steady_clock::time_point last_seen{};
  bool seen{false};
};

std::string formatValues(const TopicStats & stats, const std::size_t max_values)
{
  if (!stats.seen) {
    return "-";
  }
  std::ostringstream out;
  out << '[';
  const std::size_t printed = std::min<std::size_t>(stats.values.size(), max_values);
  for (std::size_t i = 0; i < printed; ++i) {
    if (i > 0) {
      out << ", ";
    }
    out << std::fixed << std::setprecision(4) << stats.values[i];
  }
  if (printed < stats.length) {
    out << ", ...";
  }
  out << ']';
  return out.str();
}

std::string ageText(const TopicStats & stats, const std::chrono::steady_clock::time_point now)
{
  if (!stats.seen) {
    return "-";
  }
  const auto age = std::chrono::duration<double>(now - stats.last_seen).count();
  std::ostringstream out;
  out << std::fixed << std::setprecision(2) << age << 's';
  return out.str();
}

std::string fitText(std::string text, const std::size_t width)
{
  if (text.size() <= width) {
    return text;
  }
  if (width <= 3) {
    return text.substr(0, width);
  }
  text.resize(width - 3);
  text += "...";
  return text;
}

void updateStats(TopicStats & stats, const dds_sequence_float & data)
{
  stats.length = data._length;
  stats.values.clear();
  if (data._buffer != nullptr && data._length > 0) {
    stats.values.assign(data._buffer, data._buffer + data._length);
  }
  ++stats.total;
  ++stats.window_count;
  stats.last_seen = std::chrono::steady_clock::now();
  stats.seen = true;
}

class CsvLogger
{
public:
  explicit CsvLogger(std::string path)
  : path_(std::move(path))
  {
    if (path_.empty()) {
      return;
    }

    const std::filesystem::path csv_path(path_);
    if (csv_path.has_parent_path()) {
      std::filesystem::create_directories(csv_path.parent_path());
    }
    file_.open(path_, std::ios::out | std::ios::trunc);
    if (!file_) {
      throw std::runtime_error("failed to open CSV file: " + path_);
    }
  }

  bool enabled() const
  {
    return file_.is_open();
  }

  void writeState(const dds_sequence_float & data)
  {
    if (!enabled()) {
      return;
    }
    if (!header_written_) {
      state_count_ = data._length;
      writeHeader();
    }

    file_ << wallTimeNs();
    for (std::size_t i = 0; i < state_count_; ++i) {
      const float value = (data._buffer != nullptr && i < data._length) ? data._buffer[i] : 0.0F;
      file_ << ',' << std::setprecision(9) << value;
    }
    file_ << '\n';
    file_.flush();
  }

  const std::string & path() const
  {
    return path_;
  }

private:
  static std::int64_t wallTimeNs()
  {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  }

  void writeHeader()
  {
    file_ << "time_ns";
    for (std::size_t i = 0; i < state_count_; ++i) {
      file_ << ",s_t_" << i;
    }
    file_ << '\n';
    header_written_ = true;
  }

  std::string path_;
  std::ofstream file_;
  std::size_t state_count_{0};
  bool header_written_{false};
};

int takeHeightMap(const dds_entity_t reader, TopicStats & stats, CsvLogger * csv_logger)
{
  constexpr std::size_t max_samples = 512;
  void * samples[max_samples]{};
  dds_sample_info_t infos[max_samples]{};
  for (auto & sample : samples) {
    sample = core_dds_HeightMap__alloc();
  }
  const int ret = dds_take(reader, samples, infos, max_samples, max_samples);
  if (ret < 0) {
    std::cerr << ddsError("failed to take HeightMap", ret) << '\n';
  } else {
    for (int i = 0; i < ret; ++i) {
      if (infos[i].valid_data) {
        const auto * sample = static_cast<const core_dds_HeightMap *>(samples[i]);
        updateStats(stats, sample->data);
        if (csv_logger != nullptr) {
          csv_logger->writeState(sample->data);
        }
      }
    }
  }
  for (auto * sample : samples) {
    core_dds_HeightMap_free(sample, DDS_FREE_ALL);
  }
  return ret;
}

int takeLinearVelocity(const dds_entity_t reader, TopicStats & stats)
{
  constexpr std::size_t max_samples = 512;
  void * samples[max_samples]{};
  dds_sample_info_t infos[max_samples]{};
  for (auto & sample : samples) {
    sample = core_dds_LinearVelocity__alloc();
  }
  const int ret = dds_take(reader, samples, infos, max_samples, max_samples);
  if (ret < 0) {
    std::cerr << ddsError("failed to take LinearVelocity", ret) << '\n';
  } else {
    for (int i = 0; i < ret; ++i) {
      if (infos[i].valid_data) {
        const auto * sample = static_cast<const core_dds_LinearVelocity *>(samples[i]);
        updateStats(stats, sample->data);
      }
    }
  }
  for (auto * sample : samples) {
    core_dds_LinearVelocity_free(sample, DDS_FREE_ALL);
  }
  return ret;
}

void printRow(
  const std::string & name,
  const std::string & topic,
  const TopicStats & stats,
  const std::chrono::steady_clock::time_point now,
  const std::size_t max_values)
{
  const std::string values = fitText(formatValues(stats, max_values), 48);
  std::cout
    << "| " << std::left << std::setw(16) << name
    << " | " << std::setw(14) << topic
    << " | " << std::right << std::setw(8) << std::fixed << std::setprecision(2) << stats.hz
    << " | " << std::setw(6) << (stats.seen ? std::to_string(stats.length) : "-")
    << " | " << std::setw(8) << stats.total
    << " | " << std::setw(7) << ageText(stats, now)
    << " | " << std::left << std::setw(48) << values
    << " |\n";
}

std::pair<std::size_t, std::size_t> heightMapShape(const Options & options, const TopicStats & stats)
{
  if (options.height_map_width * options.height_map_height == stats.length) {
    return {options.height_map_width, options.height_map_height};
  }

  const auto side = static_cast<std::size_t>(std::llround(std::sqrt(static_cast<double>(stats.length))));
  if (side > 0 && side * side == stats.length) {
    return {side, side};
  }

  return {stats.length, 1};
}

void printHeightMapGrid(const Options & options, const TopicStats & stats)
{
  if (!stats.seen) {
    std::cout << "\nheight_map grid: waiting for data\n";
    return;
  }
  if (stats.values.empty()) {
    std::cout << "\nheight_map grid: empty sample\n";
    return;
  }

  const auto [width, height] = heightMapShape(options, stats);
  const bool shape_matches = width * height == stats.length;
  std::cout
    << "\nheight_map grid"
    << "  shape=" << width << "x" << height
    << "  len=" << stats.length
    << "  row=y col=x";
  if (!shape_matches) {
    std::cout << "  note=length_does_not_match_config";
  }
  std::cout << '\n';

  std::cout << "        ";
  for (std::size_t col = 0; col < width; ++col) {
    std::cout << " c" << std::left << std::setw(8) << col;
  }
  std::cout << '\n';

  for (std::size_t row = 0; row < height; ++row) {
    std::cout << "r" << std::left << std::setw(6) << row;
    for (std::size_t col = 0; col < width; ++col) {
      const auto index = row * width + col;
      if (index < stats.values.size()) {
        std::cout << ' ' << std::right << std::setw(8) << std::fixed << std::setprecision(4) << stats.values[index];
      } else {
        std::cout << ' ' << std::right << std::setw(8) << "-";
      }
    }
    std::cout << '\n';
  }
}

void render(
  const Options & options,
  const TopicStats & height_map_stats,
  const TopicStats & linear_velocity_stats)
{
  const auto now = std::chrono::steady_clock::now();
  const char * dds_summary = std::getenv("AUTONOMY_DDS_ECHO_CONFIG_SUMMARY");
  const char * cyclonedds_uri = std::getenv("CYCLONEDDS_URI");
  std::cout
    << "\033[2J\033[H"
    << "DDS monitor  domain=" << options.domain_id
    << "  type=" << options.type
    << "  Ctrl-C to quit\n";
  if (dds_summary != nullptr && dds_summary[0] != '\0') {
    std::cout << "DDS config   " << dds_summary << '\n';
  }
  if (cyclonedds_uri != nullptr && cyclonedds_uri[0] != '\0') {
    std::cout << "CYCLONEDDS_URI=" << cyclonedds_uri << '\n';
  }
  std::cout
    << '\n'
    << "+------------------+----------------+----------+--------+----------+---------+--------------------------------------------------+\n"
    << "| Topic            | DDS name       | Hz       | Len    | Total    | Age     | Latest values                                    |\n"
    << "+------------------+----------------+----------+--------+----------+---------+--------------------------------------------------+\n";
  if (options.type == "both" || options.type == "height_map") {
    printRow("height_map", options.height_map_topic, height_map_stats, now, options.max_values);
  }
  if (options.type == "both" || options.type == "linear_velocity") {
    printRow("linear_velocity", options.linear_velocity_topic, linear_velocity_stats, now, options.max_values);
  }
  std::cout
    << "+------------------+----------------+----------+--------+----------+---------+--------------------------------------------------+\n"
    << std::flush;
  if (options.type == "both" || options.type == "height_map") {
    printHeightMapGrid(options, height_map_stats);
  }
  std::cout << std::flush;
}

}  // namespace

int main(const int argc, char ** argv)
{
  Options options;
  if (!parseOptions(argc, argv, options)) {
    return 2;
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  const dds_entity_t participant = dds_create_participant(options.domain_id, nullptr, nullptr);
  if (participant < 0) {
    std::cerr << ddsError("failed to create participant", participant) << '\n';
    return 1;
  }

  dds_entity_t height_map_reader = 0;
  dds_entity_t linear_velocity_reader = 0;

  if (options.type == "both" || options.type == "height_map") {
    const dds_entity_t topic = dds_create_topic(
      participant, &core_dds_HeightMap_desc, options.height_map_topic.c_str(), nullptr, nullptr);
    if (topic < 0) {
      std::cerr << ddsError("failed to create HeightMap topic", topic) << '\n';
      dds_delete(participant);
      return 1;
    }
    height_map_reader = createBestEffortReader(participant, topic);
    if (height_map_reader < 0) {
      std::cerr << ddsError("failed to create HeightMap reader", height_map_reader) << '\n';
      dds_delete(participant);
      return 1;
    }
  }

  if (options.type == "both" || options.type == "linear_velocity") {
    const dds_entity_t topic = dds_create_topic(
      participant,
      &core_dds_LinearVelocity_desc,
      options.linear_velocity_topic.c_str(),
      nullptr,
      nullptr);
    if (topic < 0) {
      std::cerr << ddsError("failed to create LinearVelocity topic", topic) << '\n';
      dds_delete(participant);
      return 1;
    }
    linear_velocity_reader = createBestEffortReader(participant, topic);
    if (linear_velocity_reader < 0) {
      std::cerr << ddsError("failed to create LinearVelocity reader", linear_velocity_reader) << '\n';
      dds_delete(participant);
      return 1;
    }
  }

  TopicStats height_map_stats;
  TopicStats linear_velocity_stats;
  CsvLogger csv_logger(options.csv_path);
  if (csv_logger.enabled()) {
    std::cout << "CSV logger writing " << csv_logger.path() << '\n'
              << "Rows are written on HeightMap samples as time_ns,s_t_*.\n";
  }

  auto next_render = std::chrono::steady_clock::now();
  auto next_hz_update = next_render + std::chrono::seconds(1);
  const auto render_period = std::chrono::duration<double>(1.0 / options.refresh_hz);

  while (g_running) {
    if (linear_velocity_reader > 0) {
      takeLinearVelocity(linear_velocity_reader, linear_velocity_stats);
    }
    if (height_map_reader > 0) {
      takeHeightMap(height_map_reader, height_map_stats, &csv_logger);
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= next_hz_update) {
      height_map_stats.hz = static_cast<double>(height_map_stats.window_count);
      linear_velocity_stats.hz = static_cast<double>(linear_velocity_stats.window_count);
      height_map_stats.window_count = 0;
      linear_velocity_stats.window_count = 0;
      next_hz_update += std::chrono::seconds(1);
    }
    if (options.render && now >= next_render) {
      render(options, height_map_stats, linear_velocity_stats);
      next_render += std::chrono::duration_cast<std::chrono::steady_clock::duration>(render_period);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  dds_delete(participant);
  return 0;
}
