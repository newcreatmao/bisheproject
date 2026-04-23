#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <onnxruntime/core/session/onnxruntime_cxx_api.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

namespace fs = std::filesystem;

constexpr double kDefaultCapturePeriodSec = 1.0 / 3.0;
constexpr double kDefaultPhotoMinIntervalSec = 2.0;
constexpr std::size_t kMaxSavedPhotos = 10;
constexpr int kLiveFrameJpegQuality = 72;
constexpr char kAllfileMountPrefix[] = "/allfile";
constexpr char kLiveFrameFileName[] = "rgb_yolo_live.jpg";

struct Detection
{
  int class_id = -1;
  float confidence = 0.0f;
  cv::Rect box;
};

std::string expandUserPath(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }

  const char * home = std::getenv("HOME");
  if (home == nullptr) {
    return path;
  }

  if (path.size() == 1) {
    return std::string(home);
  }

  if (path[1] == '/') {
    return std::string(home) + path.substr(1);
  }

  return path;
}

std::vector<std::string> loadLabels(const std::string & path)
{
  std::vector<std::string> labels;
  std::ifstream input(path);
  if (!input.is_open()) {
    return labels;
  }

  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty()) {
      labels.push_back(line);
    }
  }
  return labels;
}

cv::Mat letterbox(
  const cv::Mat & image,
  int input_width,
  int input_height,
  float & scale,
  int & pad_w,
  int & pad_h)
{
  const int width = image.cols;
  const int height = image.rows;
  const float ratio = std::min(
    static_cast<float>(input_width) / static_cast<float>(width),
    static_cast<float>(input_height) / static_cast<float>(height));

  const int resized_w = static_cast<int>(std::round(width * ratio));
  const int resized_h = static_cast<int>(std::round(height * ratio));

  cv::Mat resized;
  cv::resize(image, resized, cv::Size(resized_w, resized_h));

  pad_w = (input_width - resized_w) / 2;
  pad_h = (input_height - resized_h) / 2;
  scale = ratio;

  cv::Mat output(input_height, input_width, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(output(cv::Rect(pad_w, pad_h, resized_w, resized_h)));
  return output;
}

std::vector<float> toInputTensor(const cv::Mat & image)
{
  cv::Mat rgb;
  cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);

  cv::Mat float_image;
  rgb.convertTo(float_image, CV_32F, 1.0 / 255.0);

  const int rows = float_image.rows;
  const int cols = float_image.cols;
  const std::size_t channel_size = static_cast<std::size_t>(rows * cols);

  std::vector<float> tensor(3 * channel_size);
  std::vector<cv::Mat> chw;
  chw.reserve(3);
  for (int c = 0; c < 3; ++c) {
    chw.emplace_back(rows, cols, CV_32F, tensor.data() + c * channel_size);
  }
  cv::split(float_image, chw);
  return tensor;
}

std::vector<Detection> decodeOutput(
  const float * data,
  const std::vector<int64_t> & shape,
  std::size_t element_count,
  const cv::Size & original_size,
  float scale,
  int pad_w,
  int pad_h,
  float conf_threshold)
{
  std::vector<Detection> detections;

  if (element_count >= 6 && shape.size() >= 2) {
    const bool last_dim_is_six = shape.back() == 6;
    const bool second_dim_is_six = shape.size() == 2 && shape.front() == 6;

    if (last_dim_is_six || second_dim_is_six) {
      const int rows = static_cast<int>(element_count / 6);
      cv::Mat detections_mat;

      if (second_dim_is_six) {
        cv::Mat tmp(6, rows, CV_32F, const_cast<float *>(data));
        cv::transpose(tmp, detections_mat);
      } else {
        detections_mat = cv::Mat(rows, 6, CV_32F, const_cast<float *>(data)).clone();
      }

      detections.reserve(detections_mat.rows);
      for (int i = 0; i < detections_mat.rows; ++i) {
        const float * row = detections_mat.ptr<float>(i);
        const float score = row[4];
        if (score < conf_threshold) {
          continue;
        }

        int left = static_cast<int>(std::round((row[0] - static_cast<float>(pad_w)) / scale));
        int top = static_cast<int>(std::round((row[1] - static_cast<float>(pad_h)) / scale));
        int right = static_cast<int>(std::round((row[2] - static_cast<float>(pad_w)) / scale));
        int bottom = static_cast<int>(std::round((row[3] - static_cast<float>(pad_h)) / scale));

        left = std::clamp(left, 0, original_size.width - 1);
        top = std::clamp(top, 0, original_size.height - 1);
        right = std::clamp(right, left + 1, original_size.width);
        bottom = std::clamp(bottom, top + 1, original_size.height);

        Detection detection;
        detection.class_id = static_cast<int>(std::round(row[5]));
        detection.confidence = score;
        detection.box = cv::Rect(left, top, right - left, bottom - top);
        detections.push_back(detection);
      }
      return detections;
    }
  }

  cv::Mat predictions;
  if (shape.size() == 3) {
    const int rows = static_cast<int>(shape[1]);
    const int cols = static_cast<int>(shape[2]);
    if (rows < cols) {
      cv::Mat tmp(rows, cols, CV_32F, const_cast<float *>(data));
      cv::transpose(tmp, predictions);
    } else {
      predictions = cv::Mat(rows, cols, CV_32F, const_cast<float *>(data)).clone();
    }
  } else if (shape.size() == 2) {
    predictions = cv::Mat(
      static_cast<int>(shape[0]), static_cast<int>(shape[1]),
      CV_32F, const_cast<float *>(data)).clone();
    if (predictions.rows < predictions.cols) {
      cv::transpose(predictions, predictions);
    }
  } else {
    return detections;
  }

  detections.reserve(predictions.rows);
  for (int i = 0; i < predictions.rows; ++i) {
    const float * row = predictions.ptr<float>(i);
    if (predictions.cols < 6) {
      continue;
    }

    float best_score = 0.0f;
    int best_class = -1;
    for (int c = 4; c < predictions.cols; ++c) {
      if (row[c] > best_score) {
        best_score = row[c];
        best_class = c - 4;
      }
    }

    if (best_score < conf_threshold) {
      continue;
    }

    const float cx = row[0];
    const float cy = row[1];
    const float w = row[2];
    const float h = row[3];

    int left = static_cast<int>(std::round((cx - 0.5f * w - static_cast<float>(pad_w)) / scale));
    int top = static_cast<int>(std::round((cy - 0.5f * h - static_cast<float>(pad_h)) / scale));
    int box_w = static_cast<int>(std::round(w / scale));
    int box_h = static_cast<int>(std::round(h / scale));

    left = std::clamp(left, 0, original_size.width - 1);
    top = std::clamp(top, 0, original_size.height - 1);
    box_w = std::clamp(box_w, 1, original_size.width - left);
    box_h = std::clamp(box_h, 1, original_size.height - top);

    Detection detection;
    detection.class_id = best_class;
    detection.confidence = best_score;
    detection.box = cv::Rect(left, top, box_w, box_h);
    detections.push_back(detection);
  }

  return detections;
}

std::string jsonEscape(const std::string & input)
{
  std::ostringstream out;
  for (const unsigned char ch : input) {
    switch (ch) {
      case '\"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\b':
        out << "\\b";
        break;
      case '\f':
        out << "\\f";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (ch < 0x20) {
          out << "\\u"
              << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(ch)
              << std::dec << std::setfill(' ');
        } else {
          out << static_cast<char>(ch);
        }
        break;
    }
  }
  return out.str();
}

std::string boolJson(bool value)
{
  return value ? "true" : "false";
}

std::string doubleJson(double value)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << value;
  return out.str();
}

std::string nowIsoSeconds()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm {};
  localtime_r(&t, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S");
  return out.str();
}

std::string nowCompactTimestamp()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm {};
  localtime_r(&t, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return out.str();
}

bool looksLikeInteger(const std::string & value)
{
  if (value.empty()) {
    return false;
  }

  std::size_t start = 0;
  if (value[0] == '+' || value[0] == '-') {
    start = 1;
  }
  if (start >= value.size()) {
    return false;
  }

  for (std::size_t i = start; i < value.size(); ++i) {
    if (!std::isdigit(static_cast<unsigned char>(value[i]))) {
      return false;
    }
  }
  return true;
}

std::string sanitizeFilenameToken(const std::string & value, const std::string & fallback)
{
  std::string cleaned;
  cleaned.reserve(value.size());
  for (const unsigned char ch : value) {
    if (std::isalnum(ch) || ch == '_' || ch == '-' || ch == '.') {
      cleaned.push_back(static_cast<char>(ch));
    } else if (!std::isspace(ch)) {
      cleaned.push_back('_');
    }
  }

  while (!cleaned.empty() && cleaned.front() == '.') {
    cleaned.erase(cleaned.begin());
  }
  while (!cleaned.empty() && cleaned.back() == '.') {
    cleaned.pop_back();
  }

  return cleaned.empty() ? fallback : cleaned;
}

std::string classNameForDetection(
  const Detection & detection,
  const std::vector<std::string> & labels)
{
  if (detection.class_id >= 0 && detection.class_id < static_cast<int>(labels.size())) {
    return labels[detection.class_id];
  }
  return "id=" + std::to_string(detection.class_id);
}

std::string detectionsJson(
  const std::vector<Detection> & detections,
  const std::vector<std::string> & labels)
{
  std::ostringstream out;
  out << "[";
  for (std::size_t i = 0; i < detections.size(); ++i) {
    const auto & detection = detections[i];
    const std::string class_name = classNameForDetection(detection, labels);
    const int center_x = detection.box.x + detection.box.width / 2;
    const int center_y = detection.box.y + detection.box.height / 2;
    out << "{"
        << "\"class_id\":" << detection.class_id << ","
        << "\"class_name\":\"" << jsonEscape(class_name) << "\","
        << "\"confidence\":" << doubleJson(detection.confidence) << ","
        << "\"bbox_x\":" << detection.box.x << ","
        << "\"bbox_y\":" << detection.box.y << ","
        << "\"bbox_w\":" << detection.box.width << ","
        << "\"bbox_h\":" << detection.box.height << ","
        << "\"center\":\"(" << center_x << ", " << center_y << ")\","
        << "\"size\":\"" << detection.box.width << " × " << detection.box.height << "\""
        << "}";
    if (i + 1 < detections.size()) {
      out << ",";
    }
  }
  out << "]";
  return out.str();
}

fs::path metadataPathForPhoto(const fs::path & photo_path)
{
  fs::path metadata_path = photo_path;
  metadata_path.replace_extension(".json");
  return metadata_path;
}

std::string mountedUrlFor(
  const fs::path & root,
  const fs::path & path,
  const std::string & mount_prefix)
{
  std::error_code error;
  fs::path relative = fs::relative(path, root, error);
  if (error || relative.empty()) {
    return "";
  }

  std::string relative_text = relative.generic_string();
  if (relative_text == ".") {
    relative_text.clear();
  }
  while (!relative_text.empty() && relative_text.front() == '/') {
    relative_text.erase(relative_text.begin());
  }

  if (relative_text.empty()) {
    return mount_prefix;
  }
  return mount_prefix + "/" + relative_text;
}

void pruneSavedPhotos(const fs::path & photo_output_dir, const fs::path & newest_photo_path)
{
  std::error_code directory_error;
  if (!fs::exists(photo_output_dir, directory_error) || directory_error) {
    return;
  }

  struct PhotoEntry
  {
    fs::path path;
    fs::file_time_type write_time {};
  };

  std::vector<PhotoEntry> photos;
  for (const auto & entry : fs::directory_iterator(photo_output_dir, directory_error)) {
    if (directory_error || !entry.is_regular_file()) {
      continue;
    }

    const fs::path path = entry.path();
    if (path.extension() != ".jpg" && path.extension() != ".jpeg") {
      continue;
    }

    std::error_code time_error;
    const auto write_time = fs::last_write_time(path, time_error);
    if (time_error) {
      continue;
    }

    photos.push_back(PhotoEntry{path, write_time});
  }

  if (photos.size() <= kMaxSavedPhotos) {
    return;
  }

  std::sort(
    photos.begin(),
    photos.end(),
    [](const PhotoEntry & lhs, const PhotoEntry & rhs) {
      if (lhs.write_time == rhs.write_time) {
        return lhs.path.filename().string() < rhs.path.filename().string();
      }
      return lhs.write_time < rhs.write_time;
    });

  std::size_t remove_count = photos.size() - kMaxSavedPhotos;
  for (const auto & photo : photos) {
    if (remove_count == 0) {
      break;
    }
    if (photo.path == newest_photo_path) {
      continue;
    }
    std::error_code remove_error;
    fs::remove(photo.path, remove_error);
    if (!remove_error) {
      std::error_code metadata_remove_error;
      fs::remove(metadataPathForPhoto(photo.path), metadata_remove_error);
      --remove_count;
    }
  }
}

}  // namespace

class RgbYoloDetector : public rclcpp::Node
{
public:
  RgbYoloDetector()
  : Node("rgb_yolo_detector")
  {
    camera_source_ = declare_parameter<std::string>("camera_source", "/dev/video0");
    capture_width_ = std::max(0, static_cast<int>(declare_parameter<int>("capture_width", 640)));
    capture_height_ = std::max(0, static_cast<int>(declare_parameter<int>("capture_height", 480)));
    capture_fps_ = std::max(0, static_cast<int>(declare_parameter<int>("capture_fps", 30)));
    capture_period_sec_ = std::max(
      0.2,
      declare_parameter<double>("capture_period_sec", kDefaultCapturePeriodSec));
    yolo_model_path_ = expandUserPath(
      declare_parameter<std::string>("yolo_model_path", "~/use/project/source/pt/best.onnx"));
    yolo_classes_path_ = expandUserPath(
      declare_parameter<std::string>("yolo_classes_path", "~/use/project/source/pt/classes.txt"));
    yolo_conf_threshold_ = declare_parameter<double>("yolo_conf_threshold", 0.35);
    result_topic_ = declare_parameter<std::string>("result_topic", "/rgb_yolo/result");
    photo_output_dir_ = fs::path(expandUserPath(
      declare_parameter<std::string>("photo_output_dir", "~/use/project/source/allfile/photos")));
    photo_root_dir_ = photo_output_dir_.parent_path();
    live_output_dir_ = photo_root_dir_ / "live";
    live_frame_path_ = live_output_dir_ / kLiveFrameFileName;
    live_frame_url_ = mountedUrlFor(photo_root_dir_, live_frame_path_, kAllfileMountPrefix);
    photo_min_interval_sec_ = std::max(
      0.0,
      declare_parameter<double>("photo_min_interval_sec", kDefaultPhotoMinIntervalSec));

    fs::create_directories(live_output_dir_);

    result_pub_ = create_publisher<std_msgs::msg::String>(result_topic_, 10);

    initYolo();

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(capture_period_sec_)),
      std::bind(&RgbYoloDetector::captureAndDetect, this));

    RCLCPP_INFO(
      get_logger(),
      "rgb_yolo_detector ready: source=%s topic=%s period=%.2fs live_frame=%s",
      camera_source_.c_str(),
      result_topic_.c_str(),
      capture_period_sec_,
      live_frame_path_.string().c_str());
  }

  ~RgbYoloDetector() override
  {
    releaseCamera();
  }

private:
  void initYolo()
  {
    model_ready_ = false;
    yolo_labels_ = loadLabels(yolo_classes_path_);
    if (yolo_labels_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "YOLO classes file is empty or unreadable: %s",
        yolo_classes_path_.c_str());
    }

    try {
      ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "rgb_yolo_detector");
      ort_session_options_.SetIntraOpNumThreads(1);
      ort_session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
      yolo_session_ = std::make_unique<Ort::Session>(
        *ort_env_, yolo_model_path_.c_str(), ort_session_options_);

      Ort::AllocatorWithDefaultOptions allocator;
      for (std::size_t i = 0; i < yolo_session_->GetInputCount(); ++i) {
        auto name = yolo_session_->GetInputNameAllocated(i, allocator);
        input_names_storage_.emplace_back(name.get());
      }
      for (std::size_t i = 0; i < yolo_session_->GetOutputCount(); ++i) {
        auto name = yolo_session_->GetOutputNameAllocated(i, allocator);
        output_names_storage_.emplace_back(name.get());
      }
      for (const auto & name : input_names_storage_) {
        input_names_.push_back(name.c_str());
      }
      for (const auto & name : output_names_storage_) {
        output_names_.push_back(name.c_str());
      }

      const auto input_shape =
        yolo_session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
      if (input_shape.size() == 4) {
        if (input_shape[2] > 0) {
          yolo_input_height_ = static_cast<int>(input_shape[2]);
        }
        if (input_shape[3] > 0) {
          yolo_input_width_ = static_cast<int>(input_shape[3]);
        }
      }

      model_ready_ = true;
      last_error_.clear();
      RCLCPP_INFO(
        get_logger(),
        "RGB YOLO ready: model=%s input=%dx%d classes=%zu",
        yolo_model_path_.c_str(),
        yolo_input_width_,
        yolo_input_height_,
        yolo_labels_.size());
    } catch (const Ort::Exception & ex) {
      last_error_ = std::string("YOLO init failed: ") + ex.what();
      RCLCPP_ERROR(get_logger(), "%s", last_error_.c_str());
    } catch (const std::exception & ex) {
      last_error_ = std::string("YOLO init failed: ") + ex.what();
      RCLCPP_ERROR(get_logger(), "%s", last_error_.c_str());
    }
  }

  bool ensureCameraOpen()
  {
    if (capture_.isOpened()) {
      return true;
    }

    const int backend = cv::CAP_V4L2;
    bool opened = false;
    try {
      if (looksLikeInteger(camera_source_)) {
        opened = capture_.open(std::stoi(camera_source_), backend);
      } else {
        opened = capture_.open(camera_source_, backend);
      }
    } catch (const std::exception & ex) {
      last_error_ = std::string("camera open failed: ") + ex.what();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", last_error_.c_str());
      return false;
    }

    if (!opened) {
      last_error_ = "camera open failed for source=" + camera_source_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", last_error_.c_str());
      return false;
    }

    if (capture_width_ > 0) {
      capture_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(capture_width_));
    }
    if (capture_height_ > 0) {
      capture_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(capture_height_));
    }
    if (capture_fps_ > 0) {
      capture_.set(cv::CAP_PROP_FPS, static_cast<double>(capture_fps_));
    }

    last_error_.clear();
    RCLCPP_INFO(
      get_logger(),
      "RGB camera opened: source=%s actual=%dx%d fps=%.2f",
      camera_source_.c_str(),
      static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH)),
      static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT)),
      capture_.get(cv::CAP_PROP_FPS));
    return true;
  }

  void releaseCamera()
  {
    if (capture_.isOpened()) {
      capture_.release();
    }
  }

  void captureAndDetect()
  {
    const bool camera_open = ensureCameraOpen();
    const bool online = camera_open && model_ready_;

    std::vector<Detection> detections;
    bool detected = false;
    int detections_count = 0;
    int frame_width = 0;
    int frame_height = 0;
    int bbox_x = 0;
    int bbox_y = 0;
    int bbox_w = 0;
    int bbox_h = 0;
    float confidence = 0.0f;
    std::string class_name;
    std::string frame_time;

    if (!model_ready_) {
      publishStatus(
        online, camera_open, false, detected, detections_count, class_name, confidence,
        bbox_x, bbox_y, bbox_w, bbox_h, frame_width, frame_height, frame_time, detections);
      return;
    }

    if (!camera_open) {
      publishStatus(
        online, false, true, detected, detections_count, class_name, confidence,
        bbox_x, bbox_y, bbox_w, bbox_h, frame_width, frame_height, frame_time, detections);
      return;
    }

    cv::Mat frame;
    if (!capture_.read(frame) || frame.empty()) {
      last_error_ = "failed to capture RGB frame from " + camera_source_;
      releaseCamera();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", last_error_.c_str());
      publishStatus(
        false, false, true, detected, detections_count, class_name, confidence,
        bbox_x, bbox_y, bbox_w, bbox_h, frame_width, frame_height, frame_time, detections);
      return;
    }

    frame_width = frame.cols;
    frame_height = frame.rows;
    frame_time = nowIsoSeconds();
    last_frame_time_ = frame_time;
    last_error_.clear();

    try {
      float scale = 1.0f;
      int pad_w = 0;
      int pad_h = 0;
      cv::Mat input = letterbox(frame, yolo_input_width_, yolo_input_height_, scale, pad_w, pad_h);
      std::vector<float> input_tensor_values = toInputTensor(input);
      const std::array<int64_t, 4> input_shape{1, 3, yolo_input_height_, yolo_input_width_};

      Ort::MemoryInfo memory_info =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
      Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        input_tensor_values.data(),
        input_tensor_values.size(),
        input_shape.data(),
        input_shape.size());

      auto output_tensors = yolo_session_->Run(
        Ort::RunOptions{nullptr},
        input_names_.data(),
        &input_tensor,
        1,
        output_names_.data(),
        output_names_.size());

      if (!output_tensors.empty()) {
        const auto & output = output_tensors.front();
        auto info = output.GetTensorTypeAndShapeInfo();
        std::vector<int64_t> shape = info.GetShape();
        const std::size_t element_count = info.GetElementCount();
        const float * data = output.GetTensorData<float>();
        detections = decodeOutput(
          data,
          shape,
          element_count,
          frame.size(),
          scale,
          pad_w,
          pad_h,
          static_cast<float>(yolo_conf_threshold_));

        detections_count = static_cast<int>(detections.size());
        if (!detections.empty()) {
          const auto best_it = std::max_element(
            detections.begin(),
            detections.end(),
            [](const Detection & lhs, const Detection & rhs) {
              return lhs.confidence < rhs.confidence;
            });
          const Detection & best = *best_it;
          detected = true;
          confidence = best.confidence;
          bbox_x = best.box.x;
          bbox_y = best.box.y;
          bbox_w = best.box.width;
          bbox_h = best.box.height;
          class_name = classNameForDetection(best, yolo_labels_);
          last_detection_time_ = frame_time;
          saveDetectionPhoto(frame, detections, class_name, frame_time);
        }
      }
    } catch (const Ort::Exception & ex) {
      last_error_ = std::string("YOLO inference failed: ") + ex.what();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "%s", last_error_.c_str());
    } catch (const std::exception & ex) {
      last_error_ = std::string("YOLO inference failed: ") + ex.what();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "%s", last_error_.c_str());
    }

    writeLiveFrame(frame, detections, frame_time);

    publishStatus(
      online, true, true, detected, detections_count, class_name, confidence,
      bbox_x, bbox_y, bbox_w, bbox_h, frame_width, frame_height, frame_time, detections);
  }

  cv::Mat annotatedFrame(const cv::Mat & frame, const std::vector<Detection> & detections) const
  {
    cv::Mat annotated = frame.clone();
    const cv::Scalar accent(35, 115, 230);
    for (const auto & detection : detections) {
      cv::rectangle(annotated, detection.box, accent, 2);

      std::string class_name;
      if (detection.class_id >= 0 && detection.class_id < static_cast<int>(yolo_labels_.size())) {
        class_name = yolo_labels_[detection.class_id];
      } else {
        class_name = "id=" + std::to_string(detection.class_id);
      }

      std::ostringstream label_text;
      label_text << class_name << " " << std::fixed << std::setprecision(2) << detection.confidence;
      int baseline = 0;
      const cv::Size label_size = cv::getTextSize(
        label_text.str(),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        2,
        &baseline);
      const int label_x = detection.box.x;
      const int label_y = std::max(detection.box.y - 8, label_size.height + 8);
      cv::rectangle(
        annotated,
        cv::Rect(
          label_x,
          label_y - label_size.height - 8,
          label_size.width + 12,
          label_size.height + baseline + 10),
        accent,
        cv::FILLED);
      cv::putText(
        annotated,
        label_text.str(),
        cv::Point(label_x + 6, label_y - 4),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(255, 255, 255),
        2,
        cv::LINE_AA);
    }
    return annotated;
  }

  void writeLiveFrame(
    const cv::Mat & frame,
    const std::vector<Detection> & detections,
    const std::string & frame_time)
  {
    if (frame.empty()) {
      return;
    }

    try {
      fs::create_directories(live_output_dir_);
      const cv::Mat annotated = annotatedFrame(frame, detections);
      const fs::path tmp_path = live_frame_path_.parent_path() / "rgb_yolo_live.tmp.jpg";
      const std::vector<int> jpeg_params{cv::IMWRITE_JPEG_QUALITY, kLiveFrameJpegQuality};
      if (!cv::imwrite(tmp_path.string(), annotated, jpeg_params)) {
        throw std::runtime_error("cv::imwrite returned false");
      }

      std::error_code rename_error;
      fs::rename(tmp_path, live_frame_path_, rename_error);
      if (rename_error) {
        std::error_code remove_error;
        fs::remove(live_frame_path_, remove_error);
        rename_error.clear();
        fs::rename(tmp_path, live_frame_path_, rename_error);
      }
      if (rename_error) {
        throw std::runtime_error(rename_error.message());
      }

      last_live_frame_url_ = live_frame_url_;
      last_live_frame_time_ = frame_time;
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "failed to update RGB YOLO live frame: %s",
        ex.what());
    }
  }

  void saveDetectionPhoto(
    const cv::Mat & frame,
    const std::vector<Detection> & detections,
    const std::string & primary_class_name,
    const std::string & frame_time)
  {
    if (detections.empty()) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (last_photo_saved_steady_ != std::chrono::steady_clock::time_point {}) {
      const double age_sec =
        std::chrono::duration<double>(now - last_photo_saved_steady_).count();
      if (age_sec < photo_min_interval_sec_ && primary_class_name == last_photo_class_name_) {
        return;
      }
    }

    try {
      fs::create_directories(photo_output_dir_);

      const cv::Mat annotated = annotatedFrame(frame, detections);

      const std::string class_token =
        sanitizeFilenameToken(primary_class_name.empty() ? "unknown" : primary_class_name, "unknown");
      const fs::path photo_path =
        photo_output_dir_ / (nowCompactTimestamp() + "_" + class_token + ".jpg");

      if (!cv::imwrite(photo_path.string(), annotated)) {
        throw std::runtime_error("cv::imwrite returned false");
      }

      last_photo_path_ = photo_path.string();
      last_photo_url_ = mountedUrlFor(photo_root_dir_, photo_path, kAllfileMountPrefix);
      last_photo_time_ = frame_time;
      last_photo_class_name_ = primary_class_name;
      last_photo_saved_steady_ = now;
      writePhotoMetadata(photo_path, frame_time, detections, frame.cols, frame.rows);
      pruneSavedPhotos(photo_output_dir_, photo_path);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "failed to save RGB YOLO photo: %s",
        ex.what());
    }
  }

  void writePhotoMetadata(
    const fs::path & photo_path,
    const std::string & frame_time,
    const std::vector<Detection> & detections,
    int frame_width,
    int frame_height) const
  {
    std::ofstream output(metadataPathForPhoto(photo_path));
    if (!output.is_open()) {
      throw std::runtime_error("failed to open photo metadata for writing");
    }

    const auto best_it = std::max_element(
      detections.begin(),
      detections.end(),
      [](const Detection & lhs, const Detection & rhs) {
        return lhs.confidence < rhs.confidence;
      });
    const bool detected = best_it != detections.end();
    const std::string class_name =
      detected ? classNameForDetection(*best_it, yolo_labels_) : std::string();
    const double confidence = detected ? best_it->confidence : 0.0;
    const int bbox_x = detected ? best_it->box.x : 0;
    const int bbox_y = detected ? best_it->box.y : 0;
    const int bbox_w = detected ? best_it->box.width : 0;
    const int bbox_h = detected ? best_it->box.height : 0;

    output << "{"
           << "\"node\":\"rgb_yolo_detector\","
           << "\"online\":true,"
           << "\"camera_open\":true,"
           << "\"model_ready\":true,"
           << "\"metadata_source\":\"capture\","
           << "\"history_photo\":true,"
           << "\"source\":\"" << jsonEscape(camera_source_) << "\","
           << "\"detected\":" << boolJson(detected) << ","
           << "\"detections_count\":" << detections.size() << ","
           << "\"class_name\":\"" << jsonEscape(class_name) << "\","
           << "\"confidence\":" << doubleJson(confidence) << ","
           << "\"bbox_x\":" << bbox_x << ","
           << "\"bbox_y\":" << bbox_y << ","
           << "\"bbox_w\":" << bbox_w << ","
           << "\"bbox_h\":" << bbox_h << ","
           << "\"detections\":" << detectionsJson(detections, yolo_labels_) << ","
           << "\"frame_width\":" << frame_width << ","
           << "\"frame_height\":" << frame_height << ","
           << "\"photo_output_dir\":\"" << jsonEscape(photo_output_dir_.string()) << "\","
           << "\"last_photo_path\":\"" << jsonEscape(photo_path.string()) << "\","
           << "\"last_photo_url\":\"" << jsonEscape(last_photo_url_) << "\","
           << "\"last_photo_time\":\"" << jsonEscape(frame_time) << "\","
           << "\"last_frame_time\":\"" << jsonEscape(frame_time) << "\","
           << "\"last_detection_time\":\"" << jsonEscape(frame_time) << "\","
           << "\"error\":\"\","
           << "\"report\":\"saved photo metadata\""
           << "}";
  }

  void publishStatus(
    bool online,
    bool camera_open,
    bool model_ready,
    bool detected,
    int detections_count,
    const std::string & class_name,
    float confidence,
    int bbox_x,
    int bbox_y,
    int bbox_w,
    int bbox_h,
    int frame_width,
    int frame_height,
    const std::string & frame_time,
    const std::vector<Detection> & detections)
  {
    std_msgs::msg::String msg;
    msg.data = buildPayload(
      online,
      camera_open,
      model_ready,
      detected,
      detections_count,
      class_name,
      confidence,
      bbox_x,
      bbox_y,
      bbox_w,
      bbox_h,
      frame_width,
      frame_height,
      frame_time,
      detections);
    result_pub_->publish(msg);
  }

  std::string buildPayload(
    bool online,
    bool camera_open,
    bool model_ready,
    bool detected,
    int detections_count,
    const std::string & class_name,
    float confidence,
    int bbox_x,
    int bbox_y,
    int bbox_w,
    int bbox_h,
    int frame_width,
    int frame_height,
    const std::string & frame_time,
    const std::vector<Detection> & detections) const
  {
    std::ostringstream report;
    report << "========== RGB YOLO ==========\n"
           << "online      : " << (online ? "true" : "false") << "\n"
           << "camera_open : " << (camera_open ? "true" : "false") << "\n"
           << "model_ready : " << (model_ready ? "true" : "false") << "\n"
           << "source      : " << camera_source_ << "\n"
           << "resolution  : " << frame_width << "x" << frame_height << "\n"
           << "detected    : " << (detected ? "true" : "false") << "\n"
           << "class       : " << (class_name.empty() ? "-" : class_name) << "\n"
           << "confidence  : " << std::fixed << std::setprecision(3) << confidence << "\n"
           << "bbox        : " << bbox_x << "," << bbox_y << "," << bbox_w << "," << bbox_h << "\n"
           << "count       : " << detections_count << "\n"
           << "last_frame  : " << (frame_time.empty() ? "-" : frame_time) << "\n"
           << "last_detect : " << (last_detection_time_.empty() ? "-" : last_detection_time_) << "\n"
           << "live_frame  : " << (last_live_frame_url_.empty() ? "-" : last_live_frame_url_) << "\n"
           << "error       : " << (last_error_.empty() ? "-" : last_error_) << "\n"
           << "==============================";

    std::ostringstream out;
    out << "{"
        << "\"node\":\"rgb_yolo_detector\","
        << "\"online\":" << boolJson(online) << ","
        << "\"camera_open\":" << boolJson(camera_open) << ","
        << "\"model_ready\":" << boolJson(model_ready) << ","
        << "\"source\":\"" << jsonEscape(camera_source_) << "\","
        << "\"detected\":" << boolJson(detected) << ","
        << "\"detections_count\":" << detections_count << ","
        << "\"class_name\":\"" << jsonEscape(class_name) << "\","
        << "\"confidence\":" << doubleJson(confidence) << ","
        << "\"bbox_x\":" << bbox_x << ","
        << "\"bbox_y\":" << bbox_y << ","
        << "\"bbox_w\":" << bbox_w << ","
        << "\"bbox_h\":" << bbox_h << ","
        << "\"detections\":" << detectionsJson(detections, yolo_labels_) << ","
        << "\"frame_width\":" << frame_width << ","
        << "\"frame_height\":" << frame_height << ","
        << "\"live_frame_url\":\"" << jsonEscape(last_live_frame_url_) << "\","
        << "\"live_frame_time\":\"" << jsonEscape(last_live_frame_time_) << "\","
        << "\"live_stream_fps\":" << doubleJson(capture_period_sec_ > 0.0 ? 1.0 / capture_period_sec_ : 0.0) << ","
        << "\"photo_output_dir\":\"" << jsonEscape(photo_output_dir_.string()) << "\","
        << "\"last_photo_path\":\"" << jsonEscape(last_photo_path_) << "\","
        << "\"last_photo_url\":\"" << jsonEscape(last_photo_url_) << "\","
        << "\"last_photo_time\":\"" << jsonEscape(last_photo_time_) << "\","
        << "\"last_frame_time\":\"" << jsonEscape(frame_time) << "\","
        << "\"last_detection_time\":\"" << jsonEscape(last_detection_time_) << "\","
        << "\"error\":\"" << jsonEscape(last_error_) << "\","
        << "\"report\":\"" << jsonEscape(report.str()) << "\""
        << "}";
    return out.str();
  }

private:
  std::string camera_source_;
  int capture_width_ = 640;
  int capture_height_ = 480;
  int capture_fps_ = 30;
  double capture_period_sec_ = kDefaultCapturePeriodSec;
  std::string yolo_model_path_;
  std::string yolo_classes_path_;
  double yolo_conf_threshold_ = 0.35;
  std::string result_topic_;
  fs::path photo_output_dir_;
  fs::path photo_root_dir_;
  fs::path live_output_dir_;
  fs::path live_frame_path_;
  std::string live_frame_url_;
  double photo_min_interval_sec_ = kDefaultPhotoMinIntervalSec;
  bool model_ready_ = false;
  int yolo_input_width_ = 640;
  int yolo_input_height_ = 640;
  std::string last_frame_time_;
  std::string last_detection_time_;
  std::string last_photo_path_;
  std::string last_photo_url_;
  std::string last_photo_time_;
  std::string last_photo_class_name_;
  std::string last_live_frame_url_;
  std::string last_live_frame_time_;
  std::string last_error_;
  std::chrono::steady_clock::time_point last_photo_saved_steady_ {};

  cv::VideoCapture capture_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> yolo_labels_;
  std::unique_ptr<Ort::Env> ort_env_;
  Ort::SessionOptions ort_session_options_;
  std::unique_ptr<Ort::Session> yolo_session_;
  std::vector<std::string> input_names_storage_;
  std::vector<std::string> output_names_storage_;
  std::vector<const char *> input_names_;
  std::vector<const char *> output_names_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RgbYoloDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
