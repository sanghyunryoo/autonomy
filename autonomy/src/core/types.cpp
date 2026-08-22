#include "autonomy/core/types.hpp"

#include <algorithm>
#include <cctype>

namespace autonomy {
namespace {

std::string upper(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    if (character == ' ' || character == '-') {
      return '_';
    }
    return static_cast<char>(std::toupper(character));
  });
  return value;
}

}  // namespace

const char * autonomyModeName(const AutonomyMode mode)
{
  switch (mode) {
    case AutonomyMode::Idle:
      return "IDLE";
    case AutonomyMode::Adas:
      return "ADAS";
    case AutonomyMode::Fsd:
      return "FSD";
    case AutonomyMode::Mapping:
      return "MAPPING";
    case AutonomyMode::Tracking:
      return "TRACKING";
    case AutonomyMode::Error:
    default:
      return "ERROR";
  }
}

std::optional<AutonomyMode> parseAutonomyMode(const std::string & text)
{
  const std::string value = upper(text);
  if (value == "IDLE") {
    return AutonomyMode::Idle;
  }
  if (value == "ADAS") {
    return AutonomyMode::Adas;
  }
  if (value == "FSD") {
    return AutonomyMode::Fsd;
  }
  if (value == "MAPPING") {
    return AutonomyMode::Mapping;
  }
  if (value == "TRACKING") {
    return AutonomyMode::Tracking;
  }
  return std::nullopt;
}

bool isExternalAutonomyMode(const AutonomyMode mode)
{
  return mode == AutonomyMode::Fsd || mode == AutonomyMode::Mapping ||
         mode == AutonomyMode::Tracking;
}

RobotMode robotModeFromReport(const std::uint8_t value, const std::string & name)
{
  const std::string normalized_name = upper(name);
  if (normalized_name == "IDLE" || value == 0U) {
    return RobotMode::Idle;
  }
  if (normalized_name == "INIT" || value == 1U) {
    return RobotMode::Init;
  }
  if (normalized_name == "READY" || value == 2U) {
    return RobotMode::Ready;
  }
  if (normalized_name == "STAND" || value == 3U) {
    return RobotMode::Stand;
  }
  if (normalized_name == "FLAT_DRIVE" || value == 4U) {
    return RobotMode::FlatDrive;
  }
  if (normalized_name == "ROUGH_DRIVE" || value == 5U) {
    return RobotMode::RoughDrive;
  }
  if (normalized_name == "CUSTOM_DRIVE" || value == 6U) {
    return RobotMode::CustomDrive;
  }
  if (normalized_name == "SDK" || value == 7U) {
    return RobotMode::Sdk;
  }
  if (normalized_name == "FREEZE" || value == 8U) {
    return RobotMode::Freeze;
  }
  if (normalized_name == "SIT" || value == 9U) {
    return RobotMode::Sit;
  }
  if (normalized_name == "LIE" || value == 10U) {
    return RobotMode::Lie;
  }
  return RobotMode::Unknown;
}

bool robotModeRequiresIdleAutonomy(const RobotMode mode)
{
  return mode == RobotMode::Idle || mode == RobotMode::Init ||
         mode == RobotMode::Freeze || mode == RobotMode::Sit || mode == RobotMode::Lie;
}

const char * workloadName(const Workload workload)
{
  switch (workload) {
    case Workload::Sensors:
      return "sensors";
    case Workload::Elevation:
      return "elevation";
    case Workload::Slam:
      return "slam";
    case Workload::GlobalPlanner:
      return "hybrid_astar";
    case Workload::LocalPlanner:
      return "mppi";
    case Workload::AiDetection:
      return "ai_detection";
    case Workload::MapWriter:
      return "map_writer";
    case Workload::ObjectTracking:
      return "object_tracking";
    case Workload::CommandOutput:
      return "command_output";
    default:
      return "unknown";
  }
}

}  // namespace autonomy
