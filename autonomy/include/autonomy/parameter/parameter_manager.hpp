#pragma once

#include <string>

#include "autonomy/parameter/config.hpp"

namespace autonomy {

/**
 * Loads and validates all ROS-free runtime configuration before the five
 * runtime managers are constructed. ROS parameters are intentionally not read here.
 */
class ParameterManager final {
public:
  ParameterManager();
  explicit ParameterManager(std::string config_directory);

  void load();
  [[nodiscard]] const RuntimeConfig & runtime() const;
  [[nodiscard]] const std::string & configDirectory() const;

private:
  std::string config_directory_;
  RuntimeConfig runtime_{};
  bool loaded_{false};
};

}  // namespace autonomy
