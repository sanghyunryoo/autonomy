#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

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
    << "  --linear-velocity-topic NAME  LinearVelocity topic. Default: lin_vel\n"
    << "  --max-values N                Values printed per sample. Default: 12\n";
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
    } else if (arg == "--type" && takeArg(i, argc, argv, value)) {
      options.type = value;
    } else if (arg == "--height-map-topic" && takeArg(i, argc, argv, value)) {
      options.height_map_topic = value;
    } else if (arg == "--linear-velocity-topic" && takeArg(i, argc, argv, value)) {
      options.linear_velocity_topic = value;
    } else if (arg == "--max-values" && takeArg(i, argc, argv, value)) {
      options.max_values = std::stoul(value);
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
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 10);

  const dds_entity_t reader = dds_create_reader(participant, topic, qos, nullptr);
  dds_delete_qos(qos);
  return reader;
}

void printSequence(const dds_sequence_float & data, const std::size_t max_values)
{
  std::cout << "len=" << data._length << " data=[";
  const std::size_t printed = std::min<std::size_t>(data._length, max_values);
  for (std::size_t i = 0; i < printed; ++i) {
    if (i > 0) {
      std::cout << ", ";
    }
    std::cout << std::fixed << std::setprecision(4) << data._buffer[i];
  }
  if (printed < data._length) {
    std::cout << ", ...";
  }
  std::cout << "]\n";
}

void takeHeightMap(const dds_entity_t reader, const std::size_t max_values)
{
  void * samples[1]{nullptr};
  samples[0] = core_dds_HeightMap__alloc();
  dds_sample_info_t infos[1]{};
  const int ret = dds_take(reader, samples, infos, 1, 1);
  if (ret < 0) {
    std::cerr << ddsError("failed to take HeightMap", ret) << '\n';
  } else if (ret > 0 && infos[0].valid_data) {
    const auto * sample = static_cast<const core_dds_HeightMap *>(samples[0]);
    std::cout << "height_map ";
    printSequence(sample->data, max_values);
  }
  core_dds_HeightMap_free(samples[0], DDS_FREE_ALL);
}

void takeLinearVelocity(const dds_entity_t reader, const std::size_t max_values)
{
  void * samples[1]{nullptr};
  samples[0] = core_dds_LinearVelocity__alloc();
  dds_sample_info_t infos[1]{};
  const int ret = dds_take(reader, samples, infos, 1, 1);
  if (ret < 0) {
    std::cerr << ddsError("failed to take LinearVelocity", ret) << '\n';
  } else if (ret > 0 && infos[0].valid_data) {
    const auto * sample = static_cast<const core_dds_LinearVelocity *>(samples[0]);
    std::cout << "linear_velocity ";
    printSequence(sample->data, max_values);
  }
  core_dds_LinearVelocity_free(samples[0], DDS_FREE_ALL);
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

  std::cout
    << "Echoing DDS samples: domain=" << options.domain_id
    << " type=" << options.type
    << " height_map_topic=" << options.height_map_topic
    << " linear_velocity_topic=" << options.linear_velocity_topic
    << '\n';

  while (g_running) {
    if (height_map_reader > 0) {
      takeHeightMap(height_map_reader, options.max_values);
    }
    if (linear_velocity_reader > 0) {
      takeLinearVelocity(linear_velocity_reader, options.max_values);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  dds_delete(participant);
  return 0;
}
