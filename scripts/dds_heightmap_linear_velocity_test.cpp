#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
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
  double height_map_hz{50.0};
  double linear_velocity_hz{10.0};
  std::size_t height_map_count{144};
  std::size_t linear_velocity_count{3};
};

void printUsage(const char * argv0)
{
  std::cout
    << "Usage: " << argv0 << " [options]\n"
    << "\n"
    << "Options:\n"
    << "  --domain-id N                 DDS domain id. Default: 1\n"
    << "  --height-map-topic NAME       HeightMap topic. Default: height_map\n"
    << "  --linear-velocity-topic NAME  LinearVelocity topic. Default: lin_vel\n"
    << "  --height-map-hz HZ            HeightMap publish rate. Default: 50\n"
    << "  --linear-velocity-hz HZ       LinearVelocity publish rate. Default: 10\n"
    << "  --height-map-count N          HeightMap data length. Default: 144\n"
    << "  --linear-velocity-count N     LinearVelocity data length. Default: 3\n";
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
  for (int i = 1; i < argc; ++i) {
    const std::string arg{argv[i]};
    std::string value;
    if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      return false;
    } else if (arg == "--domain-id" && takeArg(i, argc, argv, value)) {
      options.domain_id = static_cast<dds_domainid_t>(std::stoul(value));
    } else if (arg == "--height-map-topic" && takeArg(i, argc, argv, value)) {
      options.height_map_topic = value;
    } else if (arg == "--linear-velocity-topic" && takeArg(i, argc, argv, value)) {
      options.linear_velocity_topic = value;
    } else if (arg == "--height-map-hz" && takeArg(i, argc, argv, value)) {
      options.height_map_hz = std::stod(value);
    } else if (arg == "--linear-velocity-hz" && takeArg(i, argc, argv, value)) {
      options.linear_velocity_hz = std::stod(value);
    } else if (arg == "--height-map-count" && takeArg(i, argc, argv, value)) {
      options.height_map_count = std::stoul(value);
    } else if (arg == "--linear-velocity-count" && takeArg(i, argc, argv, value)) {
      options.linear_velocity_count = std::stoul(value);
    } else {
      std::cerr << "unknown option: " << arg << '\n';
      printUsage(argv[0]);
      return false;
    }
  }
  return options.height_map_hz > 0.0 && options.linear_velocity_hz > 0.0;
}

std::string ddsError(const char * action, const int ret)
{
  return std::string(action) + ": " + dds_strretcode(-ret);
}

dds_entity_t createBestEffortWriter(
  const dds_entity_t participant,
  const dds_entity_t topic)
{
  dds_qos_t * qos = dds_create_qos();
  if (qos == nullptr) {
    std::cerr << "failed to allocate writer QoS\n";
    return DDS_RETCODE_ERROR;
  }

  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(0));
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);

  const dds_entity_t writer = dds_create_writer(participant, topic, qos, nullptr);
  dds_delete_qos(qos);
  return writer;
}

void fillHeightMap(std::vector<float> & data, const std::uint64_t tick)
{
  const float phase = static_cast<float>(tick) * 0.02F;
  for (std::size_t i = 0; i < data.size(); ++i) {
    data[i] = 0.15F * std::sin(phase + static_cast<float>(i) * 0.015F);
  }
}

void fillLinearVelocity(std::vector<float> & data, const std::uint64_t tick)
{
  if (data.empty()) {
    return;
  }

  data[0] = 0.4F + 0.05F * std::sin(static_cast<float>(tick) * 0.1F);
  if (data.size() > 1) {
    data[1] = 0.02F * std::cos(static_cast<float>(tick) * 0.07F);
  }
  if (data.size() > 2) {
    data[2] = 0.0F;
  }
  for (std::size_t i = 3; i < data.size(); ++i) {
    data[i] = 0.0F;
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

int writeLinearVelocity(const dds_entity_t writer, std::vector<float> & data)
{
  core_dds_LinearVelocity sample{};
  sample.data._maximum = static_cast<std::uint32_t>(data.size());
  sample.data._length = static_cast<std::uint32_t>(data.size());
  sample.data._buffer = data.data();
  sample.data._release = false;
  return dds_write(writer, &sample);
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

  const dds_entity_t height_map_topic = dds_create_topic(
    participant,
    &core_dds_HeightMap_desc,
    options.height_map_topic.c_str(),
    nullptr,
    nullptr);
  if (height_map_topic < 0) {
    std::cerr << ddsError("failed to create HeightMap topic", height_map_topic) << '\n';
    dds_delete(participant);
    return 1;
  }

  const dds_entity_t linear_velocity_topic = dds_create_topic(
    participant,
    &core_dds_LinearVelocity_desc,
    options.linear_velocity_topic.c_str(),
    nullptr,
    nullptr);
  if (linear_velocity_topic < 0) {
    std::cerr << ddsError("failed to create LinearVelocity topic", linear_velocity_topic) << '\n';
    dds_delete(participant);
    return 1;
  }

  const dds_entity_t height_map_writer = createBestEffortWriter(participant, height_map_topic);
  if (height_map_writer < 0) {
    std::cerr << ddsError("failed to create HeightMap writer", height_map_writer) << '\n';
    dds_delete(participant);
    return 1;
  }

  const dds_entity_t linear_velocity_writer = createBestEffortWriter(participant, linear_velocity_topic);
  if (linear_velocity_writer < 0) {
    std::cerr << ddsError("failed to create LinearVelocity writer", linear_velocity_writer) << '\n';
    dds_delete(participant);
    return 1;
  }

  std::vector<float> height_map_data(options.height_map_count, 0.0F);
  std::vector<float> linear_velocity_data(options.linear_velocity_count, 0.0F);

  using clock = std::chrono::steady_clock;
  const auto height_map_period = std::chrono::duration<double>(1.0 / options.height_map_hz);
  const auto linear_velocity_period = std::chrono::duration<double>(1.0 / options.linear_velocity_hz);

  auto next_height_map = clock::now();
  auto next_linear_velocity = next_height_map;
  auto next_report = next_height_map + std::chrono::seconds(1);
  std::uint64_t height_map_ticks = 0;
  std::uint64_t linear_velocity_ticks = 0;
  std::uint64_t height_map_total = 0;
  std::uint64_t linear_velocity_total = 0;

  std::cout
    << "Publishing DDS test samples: domain=" << options.domain_id
    << " height_map=" << options.height_map_topic << "@" << options.height_map_hz << "Hz"
    << " linear_velocity=" << options.linear_velocity_topic << "@" << options.linear_velocity_hz << "Hz"
    << '\n';

  while (g_running) {
    const auto now = clock::now();
    if (now >= next_height_map) {
      fillHeightMap(height_map_data, height_map_ticks);
      const int ret = writeHeightMap(height_map_writer, height_map_data);
      if (ret < 0) {
        std::cerr << ddsError("failed to write HeightMap", ret) << '\n';
      } else {
        ++height_map_total;
      }
      ++height_map_ticks;
      next_height_map += std::chrono::duration_cast<clock::duration>(height_map_period);
    }

    if (now >= next_linear_velocity) {
      fillLinearVelocity(linear_velocity_data, linear_velocity_ticks);
      const int ret = writeLinearVelocity(linear_velocity_writer, linear_velocity_data);
      if (ret < 0) {
        std::cerr << ddsError("failed to write LinearVelocity", ret) << '\n';
      } else {
        ++linear_velocity_total;
      }
      ++linear_velocity_ticks;
      next_linear_velocity += std::chrono::duration_cast<clock::duration>(linear_velocity_period);
    }

    if (now >= next_report) {
      std::cout
        << "sent height_map=" << height_map_total
        << " linear_velocity=" << linear_velocity_total
        << '\n';
      next_report += std::chrono::seconds(1);
    }

    std::this_thread::sleep_until(std::min(next_height_map, next_linear_velocity));
  }

  dds_delete(participant);
  return 0;
}
