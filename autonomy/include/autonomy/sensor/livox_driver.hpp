#pragma once

#include <memory>
#include <vector>

#include "autonomy/parameter/config.hpp"

namespace autonomy {

class SensorManager;

/** Direct Livox SDK2 acquisition for MID-360 and MID-360S. */
class LivoxDriver final {
public:
  LivoxDriver(
    const NetworkConfig & network,
    const std::vector<LidarConfig> & lidars,
    SensorManager & sensor_manager);
  ~LivoxDriver();

  LivoxDriver(const LivoxDriver &) = delete;
  LivoxDriver & operator=(const LivoxDriver &) = delete;

  /** Gates SDK callbacks and asks known LiDARs to enable/disable data sending. */
  void setAcquisitionEnabled(bool enabled);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_{};
};

}  // namespace autonomy
