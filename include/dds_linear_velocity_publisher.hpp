#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace autonomy
{

class DdsLinearVelocityPublisher final
{
public:
  DdsLinearVelocityPublisher(
    std::uint32_t domain_id,
    std::string topic_name,
    std::string type_name);
  ~DdsLinearVelocityPublisher();

  DdsLinearVelocityPublisher(const DdsLinearVelocityPublisher &) = delete;
  DdsLinearVelocityPublisher & operator=(const DdsLinearVelocityPublisher &) = delete;

  [[nodiscard]] bool isReady() const;
  [[nodiscard]] const std::string & error() const;
  void publish(const std::vector<float> & sample);

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
