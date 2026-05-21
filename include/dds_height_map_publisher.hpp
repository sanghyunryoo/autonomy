#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "height_map_model.hpp"

namespace eprosima::fastdds::dds
{
class DataWriter;
class DomainParticipant;
class Publisher;
class Topic;
class TypeSupport;
}  // namespace eprosima::fastdds::dds

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

  eprosima::fastdds::dds::DomainParticipant * participant_{nullptr};
  eprosima::fastdds::dds::Publisher * publisher_{nullptr};
  eprosima::fastdds::dds::Topic * topic_{nullptr};
  eprosima::fastdds::dds::DataWriter * writer_{nullptr};
  std::unique_ptr<eprosima::fastdds::dds::TypeSupport> type_support_;
};

}  // namespace autonomy
