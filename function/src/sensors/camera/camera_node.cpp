#include "sensors/camera/camera_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

namespace
{

constexpr float kInvalidDistance = 999.0f;
constexpr float kDepthNoDataFloorEpsilonMeters = 0.005f;

}  // namespace

namespace bishe::sensors::camera
{

CameraSensor::CameraSensor(
    rclcpp::Node * node,
    const std::shared_ptr<bishe::sensors::SensorDataPool> & data_pool,
    const std::shared_ptr<bishe::common::SensorSnapshotPool> & snapshot_pool)
    : node_(node),
      data_pool_(data_pool),
      snapshot_pool_(snapshot_pool),
      online_timeout_sec_(1.0),
      obstacle_max_distance_m_(2.0),
      depth_min_valid_m_(0.50),
      roi_top_ratio_(0.20),
      roi_bottom_ratio_(0.60),
      roi_left_ratio_(0.20),
      roi_right_ratio_(0.80),
      min_obstacle_pixels_(80),
      sample_sequence_(0),
      has_depth_frame_(false),
      last_image_time_(0, 0, RCL_ROS_TIME),
      encoding_(""),
      width_(0),
      height_(0)
{
    depth_topic_ = node_->declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    online_timeout_sec_ = node_->declare_parameter<double>("online_timeout_sec", 1.0);
    obstacle_max_distance_m_ = node_->declare_parameter<double>("obstacle_max_distance_m", 2.0);
    depth_min_valid_m_ = node_->declare_parameter<double>("depth_min_valid_m", 0.50);
    roi_top_ratio_ = node_->declare_parameter<double>("roi_top_ratio", 0.20);
    roi_bottom_ratio_ = node_->declare_parameter<double>("roi_bottom_ratio", 0.60);
    roi_left_ratio_ = node_->declare_parameter<double>("roi_left_ratio", 0.20);
    roi_right_ratio_ = node_->declare_parameter<double>("roi_right_ratio", 0.80);
    min_obstacle_pixels_ = node_->declare_parameter<int>("min_obstacle_pixels", 80);

    callback_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;

    depth_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        depth_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&CameraSensor::depthCallback, this, std::placeholders::_1),
        subscription_options);

    resetVisionObstacleState();

    RCLCPP_INFO(node_->get_logger(), "camera depth obstacle sensor ready");
    RCLCPP_INFO(node_->get_logger(), "camera depth topic: %s", depth_topic_.c_str());
}

bool CameraSensor::isOnline() const
{
    rclcpp::Time last_image_time(0, 0, RCL_ROS_TIME);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_image_time = last_image_time_;
    }

    if (last_image_time.nanoseconds() == 0) {
        return false;
    }

    const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(online_timeout_sec_);
    return (node_->now() - last_image_time) <= timeout;
}

std::string CameraSensor::getDepthTopic() const
{
    return depth_topic_;
}

std::string CameraSensor::getEncoding() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return encoding_;
}

int CameraSensor::getWidth() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return width_;
}

int CameraSensor::getHeight() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return height_;
}

bishe::vehicle::VisionObstacleState CameraSensor::getVisionObstacleState() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return vision_obstacle_state_;
}

void CameraSensor::publishSample(const rclcpp::Time & stamp, std::uint64_t sequence)
{
    if (!data_pool_) {
        return;
    }

    bishe::sensors::CameraSample sample;
    sample.header.stamp = stamp;
    sample.header.sequence = sequence;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        sample.depth_frame_received = has_depth_frame_;
        sample.encoding = encoding_;
        sample.width = width_;
        sample.height = height_;
        sample.vision_obstacle = vision_obstacle_state_;
    }

    data_pool_->pushCamera(sample);
}

void CameraSensor::resetVisionObstacleState()
{
    vision_obstacle_state_.detected = false;
    vision_obstacle_state_.min_distance = kInvalidDistance;
    vision_obstacle_state_.lateral_offset = 0.0f;
    vision_obstacle_state_.points = 0;
    depth_obstacle_summary_.valid = false;
    depth_obstacle_summary_.obstacle_detected = false;
    depth_obstacle_summary_.front_min_distance_m = kInvalidDistance;
    depth_obstacle_summary_.obstacle_ratio = 0.0f;
    depth_obstacle_summary_.sequence = 0;
    depth_obstacle_summary_.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void CameraSensor::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    const rclcpp::Time receive_time = node_->now();
    const rclcpp::Time sample_stamp =
        (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ?
        receive_time : rclcpp::Time(msg->header.stamp);
    const std::uint64_t sequence = ++sample_sequence_;
    bishe::common::DepthObstacleSummary depth_summary;
    depth_summary.stamp = sample_stamp;
    depth_summary.sequence = sequence;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_image_time_ = receive_time;
        has_depth_frame_ = true;
        width_ = static_cast<int>(msg->width);
        height_ = static_cast<int>(msg->height);
        encoding_ = msg->encoding;
        resetVisionObstacleState();
    }

    if (msg->width == 0 || msg->height == 0) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            depth_obstacle_summary_ = depth_summary;
        }
        if (snapshot_pool_) {
            snapshot_pool_->pushDepth(depth_summary);
        }
        publishSample(receive_time, sequence);
        return;
    }

    if (msg->encoding == "16UC1" || msg->encoding == "mono16" || msg->encoding == "Y11") {
        depth_summary = updateVisionObstacleFrom16UC1(msg);
        depth_summary.stamp = sample_stamp;
        depth_summary.sequence = sequence;
        if (snapshot_pool_) {
            snapshot_pool_->pushDepth(depth_summary);
        }
        publishSample(receive_time, sequence);
        return;
    }

    if (msg->encoding == "32FC1") {
        depth_summary = updateVisionObstacleFrom32FC1(msg);
        depth_summary.stamp = sample_stamp;
        depth_summary.sequence = sequence;
        if (snapshot_pool_) {
            snapshot_pool_->pushDepth(depth_summary);
        }
        publishSample(receive_time, sequence);
        return;
    }

    RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        3000,
        "Unsupported depth encoding: %s",
        msg->encoding.c_str());
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        depth_obstacle_summary_ = depth_summary;
    }
    if (snapshot_pool_) {
        snapshot_pool_->pushDepth(depth_summary);
    }
    publishSample(receive_time, sequence);
}

bishe::common::DepthObstacleSummary CameraSensor::updateVisionObstacleFrom16UC1(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    const auto * depth_data = reinterpret_cast<const uint16_t *>(msg->data.data());
    const std::size_t row_step = msg->step / sizeof(uint16_t);
    const int x_start = std::clamp(
        static_cast<int>(msg->width * roi_left_ratio_), 0, static_cast<int>(msg->width));
    const int x_end = std::clamp(
        static_cast<int>(msg->width * roi_right_ratio_), 0, static_cast<int>(msg->width));
    const int y_start = std::clamp(
        static_cast<int>(msg->height * roi_top_ratio_), 0, static_cast<int>(msg->height));
    const int y_end = std::clamp(
        static_cast<int>(msg->height * roi_bottom_ratio_), 0, static_cast<int>(msg->height));

    float min_distance = kInvalidDistance;
    int valid_pixels = 0;
    double x_sum = 0.0;
    bishe::vehicle::VisionObstacleState vision_state;
    bishe::common::DepthObstacleSummary summary;
    const int roi_width = std::max(0, x_end - x_start);
    const int roi_height = std::max(0, y_end - y_start);
    const double roi_pixels = std::max(1, roi_width * roi_height);

    for (int y = y_start; y < y_end; ++y) {
        const std::size_t row_offset = static_cast<std::size_t>(y) * row_step;
        for (int x = x_start; x < x_end; ++x) {
            const uint16_t depth_mm = depth_data[row_offset + static_cast<std::size_t>(x)];
            if (depth_mm == 0) {
                continue;
            }

            const float depth_m = static_cast<float>(depth_mm) / 1000.0f;
            if (depth_m < depth_min_valid_m_ || depth_m > obstacle_max_distance_m_) {
                continue;
            }
            if (depth_m <= static_cast<float>(depth_min_valid_m_) + kDepthNoDataFloorEpsilonMeters) {
                continue;
            }

            ++valid_pixels;
            x_sum += static_cast<double>(x);
            min_distance = std::min(min_distance, depth_m);
        }
    }

    vision_state.points = valid_pixels;
    summary.valid = true;
    summary.front_min_distance_m =
        valid_pixels > 0 ? min_distance : bishe::common::kNoObstacleDistanceMeters;
    summary.obstacle_ratio = static_cast<float>(
        static_cast<double>(valid_pixels) / static_cast<double>(roi_pixels));
    if (valid_pixels < min_obstacle_pixels_) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        vision_obstacle_state_ = vision_state;
        depth_obstacle_summary_ = summary;
        return summary;
    }

    const double center_x = static_cast<double>(x_start + x_end) / 2.0;
    const double half_width = std::max(1.0, static_cast<double>(x_end - x_start) / 2.0);
    vision_state.detected = true;
    vision_state.min_distance = min_distance;
    vision_state.lateral_offset =
        static_cast<float>((x_sum / static_cast<double>(valid_pixels) - center_x) / half_width);
    summary.obstacle_detected = true;

    std::lock_guard<std::mutex> lock(state_mutex_);
    vision_obstacle_state_ = vision_state;
    depth_obstacle_summary_ = summary;
    return summary;
}

bishe::common::DepthObstacleSummary CameraSensor::updateVisionObstacleFrom32FC1(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    const auto * depth_data = reinterpret_cast<const float *>(msg->data.data());
    const std::size_t row_step = msg->step / sizeof(float);
    const int x_start = std::clamp(
        static_cast<int>(msg->width * roi_left_ratio_), 0, static_cast<int>(msg->width));
    const int x_end = std::clamp(
        static_cast<int>(msg->width * roi_right_ratio_), 0, static_cast<int>(msg->width));
    const int y_start = std::clamp(
        static_cast<int>(msg->height * roi_top_ratio_), 0, static_cast<int>(msg->height));
    const int y_end = std::clamp(
        static_cast<int>(msg->height * roi_bottom_ratio_), 0, static_cast<int>(msg->height));

    float min_distance = kInvalidDistance;
    int valid_pixels = 0;
    double x_sum = 0.0;
    bishe::vehicle::VisionObstacleState vision_state;
    bishe::common::DepthObstacleSummary summary;
    const int roi_width = std::max(0, x_end - x_start);
    const int roi_height = std::max(0, y_end - y_start);
    const double roi_pixels = std::max(1, roi_width * roi_height);

    for (int y = y_start; y < y_end; ++y) {
        const std::size_t row_offset = static_cast<std::size_t>(y) * row_step;
        for (int x = x_start; x < x_end; ++x) {
            const float depth_m = depth_data[row_offset + static_cast<std::size_t>(x)];
            if (!std::isfinite(depth_m)) {
                continue;
            }

            if (depth_m < depth_min_valid_m_ || depth_m > obstacle_max_distance_m_) {
                continue;
            }
            if (depth_m <= static_cast<float>(depth_min_valid_m_) + kDepthNoDataFloorEpsilonMeters) {
                continue;
            }

            ++valid_pixels;
            x_sum += static_cast<double>(x);
            min_distance = std::min(min_distance, depth_m);
        }
    }

    vision_state.points = valid_pixels;
    summary.valid = true;
    summary.front_min_distance_m =
        valid_pixels > 0 ? min_distance : bishe::common::kNoObstacleDistanceMeters;
    summary.obstacle_ratio = static_cast<float>(
        static_cast<double>(valid_pixels) / static_cast<double>(roi_pixels));
    if (valid_pixels < min_obstacle_pixels_) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        vision_obstacle_state_ = vision_state;
        depth_obstacle_summary_ = summary;
        return summary;
    }

    const double center_x = static_cast<double>(x_start + x_end) / 2.0;
    const double half_width = std::max(1.0, static_cast<double>(x_end - x_start) / 2.0);
    vision_state.detected = true;
    vision_state.min_distance = min_distance;
    vision_state.lateral_offset =
        static_cast<float>((x_sum / static_cast<double>(valid_pixels) - center_x) / half_width);
    summary.obstacle_detected = true;

    std::lock_guard<std::mutex> lock(state_mutex_);
    vision_obstacle_state_ = vision_state;
    depth_obstacle_summary_ = summary;
    return summary;
}

}  // namespace bishe::sensors::camera
