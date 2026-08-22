#pragma once

#include <cstdint>

#include "autonomy/algorithm/planner/types.hpp"

namespace autonomy {

/** Sampling-based MPPI local controller constrained by the same occupancy map. */
class MppiPlanner final {
public:
  explicit MppiPlanner(MppiConfig config);

  [[nodiscard]] LocalPlanResult solve(
    const OccupancyGrid & map,
    const Pose2d & current_pose,
    const std::vector<Pose2d> & global_path);
  [[nodiscard]] DirectionalMotionFilter evaluateDirectionalFilter(
    const OccupancyGrid & map,
    const Pose2d & current_pose) const;

private:
  MppiConfig config_{};
  std::uint32_t sequence_{1};
};

}  // namespace autonomy
