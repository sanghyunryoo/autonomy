#include "autonomy/sensor/urdf_model.hpp"

#include <array>
#include <cmath>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <tinyxml2.h>

namespace autonomy {

namespace {

std::array<double, 9> identityRotation()
{
  return {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
}

std::array<double, 3> parseVector3(const char * text, const std::string & label)
{
  std::istringstream stream(text == nullptr ? "0 0 0" : text);
  std::array<double, 3> values{};
  double value = 0.0;
  std::size_t index = 0;
  while (stream >> value) {
    if (index == values.size()) {
      throw std::invalid_argument(label + " must contain exactly three values");
    }
    values[index++] = value;
  }
  if (index != values.size()) {
    throw std::invalid_argument(label + " must contain exactly three values");
  }
  return values;
}

std::array<double, 9> rotationFromRpy(const std::array<double, 3> & rpy)
{
  const double roll = rpy[0];
  const double pitch = rpy[1];
  const double yaw = rpy[2];
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  return {
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr,
  };
}

std::array<double, 9> multiplyRotation(
  const std::array<double, 9> & left,
  const std::array<double, 9> & right)
{
  std::array<double, 9> result{};
  for (std::size_t row = 0; row < 3U; ++row) {
    for (std::size_t column = 0; column < 3U; ++column) {
      for (std::size_t index = 0; index < 3U; ++index) {
        result[row * 3U + column] += left[row * 3U + index] * right[index * 3U + column];
      }
    }
  }
  return result;
}

std::array<double, 3> multiplyVector(
  const std::array<double, 9> & rotation,
  const std::array<double, 3> & vector)
{
  return {
    rotation[0] * vector[0] + rotation[1] * vector[1] + rotation[2] * vector[2],
    rotation[3] * vector[0] + rotation[4] * vector[1] + rotation[5] * vector[2],
    rotation[6] * vector[0] + rotation[7] * vector[1] + rotation[8] * vector[2],
  };
}

Transform inverse(const Transform & value)
{
  Transform result;
  result.parent_frame = value.child_frame;
  result.child_frame = value.parent_frame;
  for (std::size_t row = 0; row < 3U; ++row) {
    for (std::size_t column = 0; column < 3U; ++column) {
      result.rotation[row * 3U + column] = value.rotation[column * 3U + row];
    }
  }
  const std::array<double, 3> rotated_translation = multiplyVector(result.rotation, value.translation);
  result.translation = {
    -rotated_translation[0], -rotated_translation[1], -rotated_translation[2],
  };
  return result;
}

Transform compose(const Transform & left, const Transform & right)
{
  Transform result;
  result.parent_frame = left.parent_frame;
  result.child_frame = right.child_frame;
  result.rotation = multiplyRotation(left.rotation, right.rotation);
  const std::array<double, 3> rotated_translation = multiplyVector(left.rotation, right.translation);
  result.translation = {
    left.translation[0] + rotated_translation[0],
    left.translation[1] + rotated_translation[1],
    left.translation[2] + rotated_translation[2],
  };
  return result;
}

}  // namespace

void UrdfModel::load(const std::string & path)
{
  tinyxml2::XMLDocument document;
  if (document.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::invalid_argument("unable to parse URDF: " + path);
  }
  const tinyxml2::XMLElement * robot = document.FirstChildElement("robot");
  if (robot == nullptr) {
    throw std::invalid_argument("URDF robot element is missing: " + path);
  }

  std::vector<Edge> loaded_edges;
  for (const tinyxml2::XMLElement * joint = robot->FirstChildElement("joint"); joint != nullptr;
       joint = joint->NextSiblingElement("joint")) {
    const char * type = joint->Attribute("type");
    if (type == nullptr || std::string(type) != "fixed") {
      continue;
    }
    const tinyxml2::XMLElement * parent = joint->FirstChildElement("parent");
    const tinyxml2::XMLElement * child = joint->FirstChildElement("child");
    const char * parent_link = parent == nullptr ? nullptr : parent->Attribute("link");
    const char * child_link = child == nullptr ? nullptr : child->Attribute("link");
    if (parent_link == nullptr || child_link == nullptr) {
      throw std::invalid_argument("fixed URDF joint is missing parent or child link");
    }

    const tinyxml2::XMLElement * origin = joint->FirstChildElement("origin");
    Transform transform;
    transform.parent_frame = parent_link;
    transform.child_frame = child_link;
    const char * joint_name = joint->Attribute("name");
    const std::string label = joint_name == nullptr ? "URDF fixed joint" :
      std::string("URDF joint '") + joint_name + "'";
    transform.translation = parseVector3(origin == nullptr ? nullptr : origin->Attribute("xyz"),
      label + " xyz");
    transform.rotation = rotationFromRpy(parseVector3(
      origin == nullptr ? nullptr : origin->Attribute("rpy"),
      label + " rpy"));
    loaded_edges.push_back(Edge{transform.parent_frame, transform.child_frame, transform});
    const Transform reverse = inverse(transform);
    loaded_edges.push_back(Edge{reverse.parent_frame, reverse.child_frame, reverse});
  }
  edges_ = std::move(loaded_edges);
}

std::optional<Transform> UrdfModel::findTransform(
  const std::string & parent_frame,
  const std::string & child_frame) const
{
  if (parent_frame.empty() || child_frame.empty()) {
    return std::nullopt;
  }
  if (parent_frame == child_frame) {
    Transform identity;
    identity.parent_frame = parent_frame;
    identity.child_frame = child_frame;
    identity.rotation = identityRotation();
    return identity;
  }

  Transform identity;
  identity.parent_frame = parent_frame;
  identity.child_frame = parent_frame;
  identity.rotation = identityRotation();
  std::queue<std::pair<std::string, Transform>> pending;
  std::unordered_set<std::string> visited;
  pending.emplace(parent_frame, identity);
  visited.insert(parent_frame);

  while (!pending.empty()) {
    const std::string current = pending.front().first;
    const Transform parent_from_current = pending.front().second;
    pending.pop();
    if (current == child_frame) {
      return parent_from_current;
    }
    for (const Edge & edge : edges_) {
      if (edge.from != current || visited.count(edge.to) != 0U) {
        continue;
      }
      visited.insert(edge.to);
      pending.emplace(edge.to, compose(parent_from_current, edge.transform));
    }
  }
  return std::nullopt;
}

}  // namespace autonomy
