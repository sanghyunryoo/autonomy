#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include "autonomy/msg/detected_object.hpp"
#include "autonomy/msg/detected_object_array.hpp"

namespace autonomy
{

namespace
{

constexpr std::array<const char *, 80> kCocoLabels = {
  "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
  "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
  "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
  "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
  "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
  "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
  "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
  "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
  "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
  "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
  "toothbrush"};

std::string labelForClass(int class_id)
{
  if (class_id >= 0 && class_id < static_cast<int>(kCocoLabels.size())) {
    return kCocoLabels[static_cast<std::size_t>(class_id)];
  }
  return "class_" + std::to_string(class_id);
}

float readDepthMeters(const sensor_msgs::msg::Image & depth, int x, int y)
{
  if (x < 0 || y < 0 || x >= static_cast<int>(depth.width) || y >= static_cast<int>(depth.height)) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  const auto offset = static_cast<std::size_t>(y) * depth.step +
    static_cast<std::size_t>(x) * (depth.encoding == sensor_msgs::image_encodings::TYPE_32FC1 ? 4U : 2U);

  if (depth.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    float value = std::numeric_limits<float>::quiet_NaN();
    std::memcpy(&value, depth.data.data() + offset, sizeof(float));
    return value;
  }

  if (depth.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
    depth.encoding == sensor_msgs::image_encodings::MONO16)
  {
    uint16_t value = 0;
    std::memcpy(&value, depth.data.data() + offset, sizeof(uint16_t));
    return static_cast<float>(value) * 0.001f;
  }

  return std::numeric_limits<float>::quiet_NaN();
}

float medianValidDepth(
  const sensor_msgs::msg::Image & depth,
  int center_x,
  int center_y,
  int radius)
{
  std::vector<float> values;
  values.reserve(static_cast<std::size_t>((radius * 2 + 1) * (radius * 2 + 1)));
  for (int y = center_y - radius; y <= center_y + radius; ++y) {
    for (int x = center_x - radius; x <= center_x + radius; ++x) {
      const float value = readDepthMeters(depth, x, y);
      if (std::isfinite(value) && value > 0.05f && value < 100.0f) {
        values.push_back(value);
      }
    }
  }
  if (values.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

}  // namespace

class AiDetectionNode final : public rclcpp::Node
{
public:
  AiDetectionNode()
  : Node("ai_detection_node"),
    ort_env_(ORT_LOGGING_LEVEL_WARNING, "ai_detection_node")
  {
    declare_parameter<bool>("enabled", true);
    declare_parameter<std::string>(
      "model_path",
      "/root/ros2_ws/src/autonomy/resources/weights/yolo11n_640x480x3.onnx");
    declare_parameter<std::string>("image_topic", "/adas_camera/color/image_raw");
    declare_parameter<std::string>("camera_info_topic", "/adas_camera/color/camera_info");
    declare_parameter<std::string>("depth_topic", "/adas_camera/depth/image_rect_raw");
    declare_parameter<std::string>("annotated_image_topic", "~/annotated_image");
    declare_parameter<std::string>("detections_topic", "~/detections");
    declare_parameter<double>("confidence_threshold", 0.35);
    declare_parameter<double>("nms_threshold", 0.45);
    declare_parameter<int>("input_width", 640);
    declare_parameter<int>("input_height", 480);
    declare_parameter<int>("depth_sample_radius", 3);
    declare_parameter<bool>("log_inference", true);

    enabled_ = get_parameter("enabled").as_bool();
    model_path_ = get_parameter("model_path").as_string();
    image_topic_ = get_parameter("image_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    depth_topic_ = get_parameter("depth_topic").as_string();
    confidence_threshold_ = static_cast<float>(get_parameter("confidence_threshold").as_double());
    nms_threshold_ = static_cast<float>(get_parameter("nms_threshold").as_double());
    input_width_ = static_cast<int>(get_parameter("input_width").as_int());
    input_height_ = static_cast<int>(get_parameter("input_height").as_int());
    depth_sample_radius_ = std::max<int>(0, static_cast<int>(get_parameter("depth_sample_radius").as_int()));
    log_inference_ = get_parameter("log_inference").as_bool();

    loadModel();

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr msg) { onImage(std::move(msg)); });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        latest_info_ = std::move(msg);
        ++camera_info_count_;
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        latest_depth_ = std::move(msg);
        ++depth_count_;
      });

    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(
      get_parameter("annotated_image_topic").as_string(),
      10);
    detections_pub_ = create_publisher<autonomy::msg::DetectedObjectArray>(
      get_parameter("detections_topic").as_string(),
      10);
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
      "/autonomy/heartbeat/ai_detection_node",
      10);
    heartbeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() { publishHeartbeat(); });
    diagnostic_timer_ = create_wall_timer(
      std::chrono::seconds(2),
      [this]() { publishDiagnostics(); });

    RCLCPP_INFO(
      get_logger(),
      "AI detection subscriptions: image=%s camera_info=%s depth=%s",
      image_topic_.c_str(),
      camera_info_topic_.c_str(),
      depth_topic_.c_str());
  }

private:
  struct Detection
  {
    cv::Rect box;
    int class_id{0};
    float confidence{0.0f};
  };

  void loadModel()
  {
    if (!enabled_) {
      status_ = "disabled";
      return;
    }
    if (model_path_.empty() || !std::filesystem::exists(model_path_)) {
      status_ = "error:model_not_found";
      RCLCPP_ERROR(get_logger(), "YOLO ONNX model not found: %s", model_path_.c_str());
      return;
    }

    try {
      Ort::SessionOptions session_options;
      session_options.SetIntraOpNumThreads(1);
      session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
      session_ = std::make_unique<Ort::Session>(ort_env_, model_path_.c_str(), session_options);

      Ort::AllocatorWithDefaultOptions allocator;
      const std::size_t input_count = session_->GetInputCount();
      const std::size_t output_count = session_->GetOutputCount();
      input_name_storage_.clear();
      output_name_storage_.clear();
      input_names_.clear();
      output_names_.clear();
      input_name_storage_.reserve(input_count);
      output_name_storage_.reserve(output_count);
      input_names_.reserve(input_count);
      output_names_.reserve(output_count);

      for (std::size_t i = 0; i < input_count; ++i) {
        input_name_storage_.push_back(session_->GetInputNameAllocated(i, allocator));
        input_names_.push_back(input_name_storage_.back().get());
      }
      for (std::size_t i = 0; i < output_count; ++i) {
        output_name_storage_.push_back(session_->GetOutputNameAllocated(i, allocator));
        output_names_.push_back(output_name_storage_.back().get());
      }

      if (input_names_.empty() || output_names_.empty()) {
        status_ = "error:model_io_missing";
        RCLCPP_ERROR(get_logger(), "ONNX model has no input or output tensors: %s", model_path_.c_str());
        session_.reset();
        return;
      }

      model_loaded_ = true;
      status_ = "ready";
      RCLCPP_INFO(
        get_logger(),
        "Loaded YOLO ONNX model with ONNX Runtime: %s input=%s output=%s",
        model_path_.c_str(),
        input_names_.front(),
        output_names_.front());
    } catch (const Ort::Exception & error) {
      status_ = "error:model_load_failed";
      RCLCPP_ERROR(get_logger(), "Failed to load YOLO ONNX model with ONNX Runtime: %s", error.what());
    }
  }

  void onImage(sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!enabled_ || !model_loaded_) {
      return;
    }
    ++image_count_;
    last_image_stamp_ = stampToSeconds(msg->header.stamp);

    cv_bridge::CvImageConstPtr bridge;
    try {
      bridge = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & error) {
      status_ = "error:image_decode_failed";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge failed: %s", error.what());
      return;
    }

    cv::Mat annotated = bridge->image.clone();
    std::vector<Detection> detections;
    try {
      detections = infer(bridge->image);
    } catch (const Ort::Exception & error) {
      status_ = "error:inference_failed";
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "AI inference failed with ONNX Runtime: %s",
        error.what());
      publishEmptyOutputs(*msg, annotated);
      return;
    } catch (const std::exception & error) {
      status_ = "error:inference_failed";
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "AI inference failed: %s",
        error.what());
      publishEmptyOutputs(*msg, annotated);
      return;
    }
    if (log_inference_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "AI inference running: stamp=%d.%09u image=%ux%u detections=%zu",
        msg->header.stamp.sec,
        msg->header.stamp.nanosec,
        msg->width,
        msg->height,
        detections.size());
    }
    autonomy::msg::DetectedObjectArray output;
    output.header = msg->header;

    for (const auto & detection : detections) {
      drawDetection(annotated, detection);
      output.objects.push_back(toMessage(detection, *msg));
    }

    detections_pub_->publish(output);
    auto annotated_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, annotated).toImageMsg();
    annotated_pub_->publish(*annotated_msg);
    status_ = "tracking:" + std::to_string(output.objects.size());
  }

  void publishEmptyOutputs(const sensor_msgs::msg::Image & image, const cv::Mat & annotated)
  {
    autonomy::msg::DetectedObjectArray output;
    output.header = image.header;
    detections_pub_->publish(output);
    auto annotated_msg =
      cv_bridge::CvImage(image.header, sensor_msgs::image_encodings::BGR8, annotated).toImageMsg();
    annotated_pub_->publish(*annotated_msg);
  }

  std::vector<Detection> infer(const cv::Mat & bgr)
  {
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(input_width_, input_height_));
    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

    std::vector<float> input_tensor_values(
      static_cast<std::size_t>(3 * input_width_ * input_height_));
    const int plane_size = input_width_ * input_height_;
    for (int y = 0; y < input_height_; ++y) {
      const auto * row = rgb.ptr<cv::Vec3b>(y);
      for (int x = 0; x < input_width_; ++x) {
        const int hw_index = y * input_width_ + x;
        input_tensor_values[static_cast<std::size_t>(hw_index)] =
          static_cast<float>(row[x][0]) / 255.0f;
        input_tensor_values[static_cast<std::size_t>(plane_size + hw_index)] =
          static_cast<float>(row[x][1]) / 255.0f;
        input_tensor_values[static_cast<std::size_t>(2 * plane_size + hw_index)] =
          static_cast<float>(row[x][2]) / 255.0f;
      }
    }

    std::array<int64_t, 4> input_shape{
      1,
      3,
      static_cast<int64_t>(input_height_),
      static_cast<int64_t>(input_width_)};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto input_tensor = Ort::Value::CreateTensor<float>(
      memory_info,
      input_tensor_values.data(),
      input_tensor_values.size(),
      input_shape.data(),
      input_shape.size());

    auto outputs = session_->Run(
      Ort::RunOptions{nullptr},
      input_names_.data(),
      &input_tensor,
      1,
      output_names_.data(),
      output_names_.size());
    if (outputs.empty()) {
      if (log_inference_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "AI inference produced no output tensors");
      }
      return {};
    }

    auto & output_tensor = outputs.front();
    const auto shape = output_tensor.GetTensorTypeAndShapeInfo().GetShape();
    if (log_inference_) {
      std::ostringstream shape;
      shape << "[";
      for (std::size_t i = 0; i < output_tensor.GetTensorTypeAndShapeInfo().GetShape().size(); ++i) {
        if (i > 0) {
          shape << ",";
        }
        shape << output_tensor.GetTensorTypeAndShapeInfo().GetShape()[i];
      }
      shape << "]";
      RCLCPP_DEBUG_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "YOLO output tensor shape=%s",
        shape.str().c_str());
    }

    return parseYoloOutput(output_tensor.GetTensorData<float>(), shape, bgr.size());
  }

  std::vector<Detection> parseYoloOutput(
    const float * output,
    const std::vector<int64_t> & shape,
    const cv::Size & original_size)
  {
    if (shape.size() != 3) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Unexpected YOLO output dims: %zu",
        shape.size());
      return {};
    }

    const int a = static_cast<int>(shape[1]);
    const int b = static_cast<int>(shape[2]);
    const bool channels_first = a < b;
    const int num_features = channels_first ? a : b;
    const int num_predictions = channels_first ? b : a;
    const int num_classes = num_features - 4;
    if (num_classes <= 0) {
      return {};
    }

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;
    boxes.reserve(static_cast<std::size_t>(num_predictions));
    confidences.reserve(static_cast<std::size_t>(num_predictions));
    class_ids.reserve(static_cast<std::size_t>(num_predictions));

    const float scale_x = static_cast<float>(original_size.width) / static_cast<float>(input_width_);
    const float scale_y = static_cast<float>(original_size.height) / static_cast<float>(input_height_);

    for (int i = 0; i < num_predictions; ++i) {
      auto value_at = [&](int feature) -> float {
          if (channels_first) {
            return output[static_cast<std::size_t>(feature) * num_predictions + i];
          }
          return output[static_cast<std::size_t>(i) * num_features + feature];
        };

      int class_id = 0;
      float confidence = 0.0f;
      for (int c = 0; c < num_classes; ++c) {
        const float score = value_at(4 + c);
        if (score > confidence) {
          confidence = score;
          class_id = c;
        }
      }
      if (confidence < confidence_threshold_) {
        continue;
      }

      const float cx = value_at(0) * scale_x;
      const float cy = value_at(1) * scale_y;
      const float w = value_at(2) * scale_x;
      const float h = value_at(3) * scale_y;
      int left = static_cast<int>(std::round(cx - w * 0.5f));
      int top = static_cast<int>(std::round(cy - h * 0.5f));
      int width = static_cast<int>(std::round(w));
      int height = static_cast<int>(std::round(h));
      left = std::clamp(left, 0, std::max(0, original_size.width - 1));
      top = std::clamp(top, 0, std::max(0, original_size.height - 1));
      width = std::clamp(width, 0, original_size.width - left);
      height = std::clamp(height, 0, original_size.height - top);
      if (width <= 1 || height <= 1) {
        continue;
      }

      boxes.emplace_back(left, top, width, height);
      confidences.push_back(confidence);
      class_ids.push_back(class_id);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold_, indices);

    std::vector<Detection> detections;
    detections.reserve(indices.size());
    for (int index : indices) {
      detections.push_back(Detection{boxes[static_cast<std::size_t>(index)],
        class_ids[static_cast<std::size_t>(index)],
        confidences[static_cast<std::size_t>(index)]});
    }
    return detections;
  }

  autonomy::msg::DetectedObject toMessage(
    const Detection & detection,
    const sensor_msgs::msg::Image & image) const
  {
    autonomy::msg::DetectedObject object;
    object.header = image.header;
    object.label = labelForClass(detection.class_id);
    object.confidence = detection.confidence;
    object.bbox_x = static_cast<uint32_t>(detection.box.x);
    object.bbox_y = static_cast<uint32_t>(detection.box.y);
    object.bbox_width = static_cast<uint32_t>(detection.box.width);
    object.bbox_height = static_cast<uint32_t>(detection.box.height);
    fill3dPosition(detection.box, cv::Size(static_cast<int>(image.width), static_cast<int>(image.height)), object);
    return object;
  }

  void fill3dPosition(
    const cv::Rect & box,
    const cv::Size & image_size,
    autonomy::msg::DetectedObject & object) const
  {
    if (!latest_depth_ || !latest_info_) {
      object.has_3d_position = false;
      return;
    }
    if (latest_depth_->width == 0 || latest_depth_->height == 0 || latest_info_->k[0] == 0.0 ||
      latest_info_->k[4] == 0.0)
    {
      object.has_3d_position = false;
      return;
    }

    const double scale_x = static_cast<double>(latest_depth_->width) /
      static_cast<double>(std::max(1, image_size.width));
    const double scale_y = static_cast<double>(latest_depth_->height) /
      static_cast<double>(std::max(1, image_size.height));
    const int center_x = static_cast<int>(std::round((box.x + box.width * 0.5) * scale_x));
    const int center_y = static_cast<int>(std::round((box.y + box.height * 0.5) * scale_y));
    const float z = medianValidDepth(*latest_depth_, center_x, center_y, depth_sample_radius_);
    if (!std::isfinite(z)) {
      object.has_3d_position = false;
      return;
    }

    const double fx = latest_info_->k[0];
    const double fy = latest_info_->k[4];
    const double cx = latest_info_->k[2];
    const double cy = latest_info_->k[5];
    object.position.x = (static_cast<double>(center_x) - cx) * static_cast<double>(z) / fx;
    object.position.y = (static_cast<double>(center_y) - cy) * static_cast<double>(z) / fy;
    object.position.z = z;
    object.has_3d_position = true;
  }

  void drawDetection(cv::Mat & image, const Detection & detection) const
  {
    const std::string label = labelForClass(detection.class_id);
    std::ostringstream text;
    text.precision(2);
    text << std::fixed << label << " " << detection.confidence;

    cv::rectangle(image, detection.box, cv::Scalar(40, 220, 40), 2);
    int baseline = 0;
    const cv::Size text_size = cv::getTextSize(text.str(), cv::FONT_HERSHEY_SIMPLEX, 0.55, 2, &baseline);
    const int top = std::max(detection.box.y, text_size.height + 6);
    cv::rectangle(
      image,
      cv::Point(detection.box.x, top - text_size.height - 6),
      cv::Point(detection.box.x + text_size.width + 6, top + baseline - 2),
      cv::Scalar(40, 220, 40),
      cv::FILLED);
    cv::putText(
      image,
      text.str(),
      cv::Point(detection.box.x + 3, top - 4),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      cv::Scalar(0, 0, 0),
      2,
      cv::LINE_AA);
  }

  void publishHeartbeat()
  {
    std_msgs::msg::String msg;
    msg.data = enabled_ ? status_ : "disabled";
    heartbeat_pub_->publish(msg);
  }

  double stampToSeconds(const builtin_interfaces::msg::Time & stamp) const
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  void publishDiagnostics()
  {
    if (!enabled_) {
      return;
    }

    const auto image_publishers = count_publishers(image_topic_);
    const auto info_publishers = count_publishers(camera_info_topic_);
    const auto depth_publishers = count_publishers(depth_topic_);
    if (image_count_ == 0) {
      RCLCPP_WARN(
        get_logger(),
        "AI detection waiting for RGB image: topic=%s publishers=%zu camera_info_count=%zu "
        "depth_count=%zu info_publishers=%zu depth_publishers=%zu",
        image_topic_.c_str(),
        image_publishers,
        camera_info_count_,
        depth_count_,
        info_publishers,
        depth_publishers);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "AI detection input status: images=%zu last_stamp=%.6f camera_info=%zu depth=%zu "
      "image_publishers=%zu",
      image_count_,
      last_image_stamp_,
      camera_info_count_,
      depth_count_,
      image_publishers);
  }

  bool enabled_{true};
  bool model_loaded_{false};
  bool log_inference_{true};
  int input_width_{640};
  int input_height_{480};
  int depth_sample_radius_{3};
  float confidence_threshold_{0.35f};
  float nms_threshold_{0.45f};
  std::string model_path_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string depth_topic_;
  std::string status_{"starting"};
  std::size_t image_count_{0};
  std::size_t camera_info_count_{0};
  std::size_t depth_count_{0};
  double last_image_stamp_{0.0};
  Ort::Env ort_env_;
  std::unique_ptr<Ort::Session> session_;
  std::vector<Ort::AllocatedStringPtr> input_name_storage_;
  std::vector<Ort::AllocatedStringPtr> output_name_storage_;
  std::vector<const char *> input_names_;
  std::vector<const char *> output_names_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_info_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<autonomy::msg::DetectedObjectArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace autonomy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy::AiDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
