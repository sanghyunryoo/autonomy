#include "dds_height_map_publisher.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>
#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TopicDataType.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>
#include <fastrtps/rtps/common/SerializedPayload.h>

namespace autonomy
{
namespace
{

class HeightMapPubSubType final : public eprosima::fastdds::dds::TopicDataType
{
public:
  explicit HeightMapPubSubType(const std::string & type_name)
  {
    setName(type_name.c_str());
    m_typeSize = kMaxSerializedSize;
    m_isGetKeyDefined = false;
  }

  bool serialize(void * data, eprosima::fastrtps::rtps::SerializedPayload_t * payload) override
  {
    auto * sample = static_cast<DdsHeightMap *>(data);
    const auto required_size = serializedSize(*sample);
    if (required_size > payload->max_size) {
      return false;
    }

    eprosima::fastcdr::FastBuffer fast_buffer(
      reinterpret_cast<char *>(payload->data), payload->max_size);
    eprosima::fastcdr::Cdr cdr(
      fast_buffer,
      eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
      eprosima::fastcdr::Cdr::DDS_CDR);

    try {
      cdr.serialize_encapsulation();
      cdr << sample->data;
    } catch (const std::exception &) {
      return false;
    }

    payload->length = static_cast<std::uint32_t>(cdr.getSerializedDataLength());
    return true;
  }

  bool deserialize(eprosima::fastrtps::rtps::SerializedPayload_t * payload, void * data) override
  {
    auto * sample = static_cast<DdsHeightMap *>(data);
    eprosima::fastcdr::FastBuffer fast_buffer(
      reinterpret_cast<char *>(payload->data), payload->length);
    eprosima::fastcdr::Cdr cdr(
      fast_buffer,
      eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
      eprosima::fastcdr::Cdr::DDS_CDR);

    try {
      cdr.read_encapsulation();
      cdr >> sample->data;
    } catch (const std::exception &) {
      return false;
    }

    return true;
  }

  std::function<std::uint32_t()> getSerializedSizeProvider(void * data) override
  {
    auto * sample = static_cast<DdsHeightMap *>(data);
    return [sample]() {
      return static_cast<std::uint32_t>(serializedSize(*sample));
    };
  }

  void * createData() override
  {
    return new DdsHeightMap();
  }

  void deleteData(void * data) override
  {
    delete static_cast<DdsHeightMap *>(data);
  }

  bool getKey(
    void *,
    eprosima::fastrtps::rtps::InstanceHandle_t *,
    bool) override
  {
    return false;
  }

  bool is_bounded() const override
  {
    return false;
  }

  bool is_plain() const override
  {
    return false;
  }

  bool construct_sample(void * memory) const override
  {
    new (memory) DdsHeightMap();
    return true;
  }

private:
  static constexpr std::uint32_t kMaxSerializedSize{1024U * 1024U};

  static std::size_t serializedSize(const DdsHeightMap & sample)
  {
    return 4 + 4 + eprosima::fastcdr::Cdr::alignment(4, 4) +
           sample.data.size() * sizeof(float);
  }
};

}  // namespace

DdsHeightMapPublisher::DdsHeightMapPublisher(
  const std::uint32_t domain_id,
  std::string topic_name,
  std::string type_name)
: domain_id_(domain_id),
  topic_name_(std::move(topic_name)),
  type_name_(std::move(type_name))
{
  initialize();
}

DdsHeightMapPublisher::~DdsHeightMapPublisher()
{
  cleanup();
}

bool DdsHeightMapPublisher::isReady() const
{
  return writer_ != nullptr;
}

const std::string & DdsHeightMapPublisher::error() const
{
  return error_;
}

void DdsHeightMapPublisher::publish(const DdsHeightMap & sample)
{
  if (writer_ == nullptr) {
    return;
  }

  auto writable_sample = sample;
  writer_->write(&writable_sample);
}

void DdsHeightMapPublisher::initialize()
{
  using namespace eprosima::fastdds::dds;

  auto * factory = DomainParticipantFactory::get_instance();
  participant_ = factory->create_participant(domain_id_, PARTICIPANT_QOS_DEFAULT);
  if (participant_ == nullptr) {
    error_ = "failed to create DDS participant";
    return;
  }

  type_support_ = std::make_unique<TypeSupport>(new HeightMapPubSubType(type_name_));
  if (type_support_->register_type(participant_) != eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK) {
    error_ = "failed to register DDS type";
    cleanup();
    return;
  }

  publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT);
  if (publisher_ == nullptr) {
    error_ = "failed to create DDS publisher";
    cleanup();
    return;
  }

  topic_ = participant_->create_topic(topic_name_, type_name_, TOPIC_QOS_DEFAULT);
  if (topic_ == nullptr) {
    error_ = "failed to create DDS topic";
    cleanup();
    return;
  }

  auto writer_qos = DATAWRITER_QOS_DEFAULT;
  writer_qos.history().kind = KEEP_LAST_HISTORY_QOS;
  writer_qos.history().depth = 1;
  writer_qos.reliability().kind = RELIABLE_RELIABILITY_QOS;
  writer_qos.resource_limits().max_samples = 1;
  writer_qos.resource_limits().allocated_samples = 1;
  writer_qos.resource_limits().extra_samples = 1;

  writer_ = publisher_->create_datawriter(topic_, writer_qos);
  if (writer_ == nullptr) {
    error_ = "failed to create DDS writer";
    cleanup();
  }
}

void DdsHeightMapPublisher::cleanup()
{
  auto * factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();

  if (publisher_ != nullptr && writer_ != nullptr) {
    publisher_->delete_datawriter(writer_);
    writer_ = nullptr;
  }
  if (participant_ != nullptr && topic_ != nullptr) {
    participant_->delete_topic(topic_);
    topic_ = nullptr;
  }
  if (participant_ != nullptr && publisher_ != nullptr) {
    participant_->delete_publisher(publisher_);
    publisher_ = nullptr;
  }
  if (participant_ != nullptr) {
    factory->delete_participant(participant_);
    participant_ = nullptr;
  }
  type_support_.reset();
}

}  // namespace autonomy
