#pragma once

#include <cstdint>
#include <string>

#include "height_map_model.hpp"

namespace autonomy
{

class DdsHeightMapPublisher final
{
public:
  DdsHeightMapPublisher(
    std::uint32_t domain_id,
    std::string topic_name,
    std::string type_name);
  ~DdsHeightMapPublisher();

  DdsHeightMapPublisher(const DdsHeightMapPublisher &) = delete;
  DdsHeightMapPublisher & operator=(const DdsHeightMapPublisher &) = delete;

  [[nodiscard]] bool isReady() const;
  [[nodiscard]] const std::string & error() const;
  void publish(const DdsHeightMap & sample);

private:
  void initialize();
  void cleanup();

  std::uint32_t domain_id_;
  std::string topic_name_;
  std::string type_name_;
  std::string error_;

  int participant_{0};
  int topic_{0};
  int writer_{0};
};

}  // namespace autonomy
