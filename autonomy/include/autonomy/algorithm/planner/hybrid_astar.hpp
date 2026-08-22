#pragma once

#include "autonomy/algorithm/planner/types.hpp"

namespace autonomy {

/** Collision-checked Hybrid A* on a 2D occupancy grid. */
class HybridAStarPlanner final {
public:
  explicit HybridAStarPlanner(HybridAStarConfig config);

  [[nodiscard]] GlobalPlanResult plan(
    const OccupancyGrid & map,
    const Pose2d & start,
    const Pose2d & goal) const;

private:
  HybridAStarConfig config_{};
};

}  // namespace autonomy
