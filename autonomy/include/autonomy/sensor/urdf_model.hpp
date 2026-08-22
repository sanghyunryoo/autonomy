#pragma once

#include <optional>
#include <string>

#include "autonomy/sensor/types.hpp"

namespace autonomy {

/** Parses fixed URDF joints and resolves a static transform through the tree. */
class UrdfModel final {
public:
  void load(const std::string & path);
  [[nodiscard]] std::optional<Transform> findTransform(
    const std::string & parent_frame,
    const std::string & child_frame) const;

private:
  struct Edge {
    std::string from{};
    std::string to{};
    Transform transform{};
  };
  std::vector<Edge> edges_{};
};

}  // namespace autonomy
