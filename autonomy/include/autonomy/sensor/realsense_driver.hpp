#pragma once

#include <memory>
#include <vector>

#include "autonomy/parameter/config.hpp"

namespace autonomy {

class SensorManager;

/** Direct librealsense2 acquisition for every configured USB RealSense. */
class RealSenseDriver final {
public:
  RealSenseDriver(const std::vector<CameraConfig> & cameras, SensorManager & sensor_manager);
  ~RealSenseDriver();

  RealSenseDriver(const RealSenseDriver &) = delete;
  RealSenseDriver & operator=(const RealSenseDriver &) = delete;

  /** Starts/stops USB streams on the IDLE boundary. */
  void setAcquisitionEnabled(bool enabled);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_{};
};

}  // namespace autonomy
