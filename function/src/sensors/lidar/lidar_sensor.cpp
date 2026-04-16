#include "sensors/lidar/lidar_sensor.hpp"

#include <cmath>
#include <functional>
#include <limits>

namespace bishe::sensors::lidar
{

double LidarSensor::normalizeAngleDeg(double angle_deg)
{
    while (angle_deg < 0.0) {
        angle_deg += 360.0;
    }
    while (angle_deg >= 360.0) {
        angle_deg -= 360.0;
    }
    return angle_deg;
}

bool LidarSensor::isAngleInSector(
    double angle_deg,
    double angle_center_deg,
    double angle_half_width_deg)
{
    const double normalized_angle = normalizeAngleDeg(angle_deg);
    const double start_deg = normalizeAngleDeg(angle_center_deg - angle_half_width_deg);
    const double end_deg = normalizeAngleDeg(angle_center_deg + angle_half_width_deg);

    if (start_deg <= end_deg) {
        return normalized_angle >= start_deg && normalized_angle <= end_deg;
    }

    return normalized_angle >= start_deg || normalized_angle <= end_deg;
}

LidarSensor::LidarSensor(
    rclcpp::Node * node,
    const std::shared_ptr<bishe::sensors::SensorDataPool> & data_pool,
    const std::shared_ptr<bishe::common::SensorSnapshotPool> & snapshot_pool)
    : node_(node),
      data_pool_(data_pool),
      snapshot_pool_(snapshot_pool)
{
    refreshParameters();
    callback_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;

    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        10,
        std::bind(&LidarSensor::scanCallback, this, std::placeholders::_1),
        subscription_options);

    RCLCPP_INFO(
        node_->get_logger(),
        "lidar sensor ready, topic=/scan enabled=%s center=%.1f half_width=%.1f threshold=%.2f",
        obstacle_detection_enabled_ ? "true" : "false",
        angle_center_deg_,
        angle_half_width_deg_,
        distance_threshold_);
}

void LidarSensor::refreshParameters()
{
    bool obstacle_detection_enabled = obstacle_detection_enabled_;
    double angle_center_deg = angle_center_deg_;
    double angle_half_width_deg = angle_half_width_deg_;
    double distance_threshold = distance_threshold_;
    double local_front_sector_deg = local_front_sector_deg_;
    double local_left_sector_deg = local_left_sector_deg_;
    double local_right_sector_deg = local_right_sector_deg_;
    node_->get_parameter_or("lidar_obstacle_detection_enabled", obstacle_detection_enabled, true);
    node_->get_parameter_or("obstacle_angle_center_deg", angle_center_deg, 270.0);
    node_->get_parameter_or("obstacle_angle_half_width_deg", angle_half_width_deg, 30.0);
    node_->get_parameter_or("obstacle_ignore_distance_m", distance_threshold, 2.0);
    node_->get_parameter_or("auto_local_front_sector_deg", local_front_sector_deg, 40.0);
    node_->get_parameter_or("auto_local_left_sector_deg", local_left_sector_deg, 35.0);
    node_->get_parameter_or("auto_local_right_sector_deg", local_right_sector_deg, 35.0);

    std::lock_guard<std::mutex> lock(state_mutex_);
    obstacle_detection_enabled_ = obstacle_detection_enabled;
    angle_center_deg_ = angle_center_deg;
    angle_half_width_deg_ = angle_half_width_deg;
    distance_threshold_ = distance_threshold;
    local_front_sector_deg_ = std::max(local_front_sector_deg, 1.0);
    local_left_sector_deg_ = std::max(local_left_sector_deg, 1.0);
    local_right_sector_deg_ = std::max(local_right_sector_deg, 1.0);
}

bishe::vehicle::ObstacleState LidarSensor::getObstacleState() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return obstacle_state_;
}

void LidarSensor::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    const rclcpp::Time sample_stamp =
        (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ?
        node_->now() : rclcpp::Time(msg->header.stamp);
    const std::uint64_t sequence = ++sample_sequence_;

    bool obstacle_detection_enabled = false;
    double angle_center_deg = 270.0;
    double angle_half_width_deg = 30.0;
    double distance_threshold = 2.0;
    double local_front_sector_deg = 40.0;
    double local_left_sector_deg = 35.0;
    double local_right_sector_deg = 35.0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        obstacle_detection_enabled = obstacle_detection_enabled_;
        angle_center_deg = angle_center_deg_;
        angle_half_width_deg = angle_half_width_deg_;
        distance_threshold = distance_threshold_;
        local_front_sector_deg = local_front_sector_deg_;
        local_left_sector_deg = local_left_sector_deg_;
        local_right_sector_deg = local_right_sector_deg_;
    }

    bishe::vehicle::ObstacleState obstacle_state;
    bishe::common::LidarObstacleSummary summary;
    summary.stamp = sample_stamp;
    summary.sequence = sequence;

    if (!obstacle_detection_enabled) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            obstacle_state_ = obstacle_state;
        }
        if (snapshot_pool_) {
            snapshot_pool_->pushLidar(summary);
        }
        bishe::sensors::LidarSample sample;
        sample.header.stamp = sample_stamp;
        sample.header.sequence = sequence;
        sample.obstacle = obstacle_state;
        if (data_pool_) {
            data_pool_->pushLidar(sample);
        }
        return;
    }

    if (msg->ranges.empty()) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            obstacle_state_ = obstacle_state;
        }
        if (snapshot_pool_) {
            snapshot_pool_->pushLidar(summary);
        }
        bishe::sensors::LidarSample sample;
        sample.header.stamp = sample_stamp;
        sample.header.sequence = sequence;
        sample.obstacle = obstacle_state;
        if (data_pool_) {
            data_pool_->pushLidar(sample);
        }
        return;
    }

    float min_distance = std::numeric_limits<float>::max();
    float min_angle_deg = 0.0f;
    int valid_points = 0;
    float front_min_distance = std::numeric_limits<float>::max();
    float left_min_distance = std::numeric_limits<float>::max();
    float right_min_distance = std::numeric_limits<float>::max();
    int front_points = 0;
    int left_points = 0;
    int right_points = 0;
    const double front_half_width_deg = std::max(local_front_sector_deg * 0.5, 1.0);
    const double side_half_width_deg_left = std::max(local_left_sector_deg * 0.5, 1.0);
    const double side_half_width_deg_right = std::max(local_right_sector_deg * 0.5, 1.0);
    const double left_center_deg = normalizeAngleDeg(angle_center_deg + 45.0);
    const double right_center_deg = normalizeAngleDeg(angle_center_deg - 45.0);

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        const float range = msg->ranges[i];
        if (!std::isfinite(range)) {
            continue;
        }
        if (range < msg->range_min || range > msg->range_max) {
            continue;
        }

        const double angle_rad = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
        const double angle_deg = angle_rad * 180.0 / M_PI;
        if (!isAngleInSector(angle_deg, angle_center_deg, angle_half_width_deg)) {
            if (isAngleInSector(angle_deg, angle_center_deg, front_half_width_deg)) {
                ++front_points;
                if (range < front_min_distance) {
                    front_min_distance = range;
                }
            }
            if (isAngleInSector(angle_deg, left_center_deg, side_half_width_deg_left)) {
                ++left_points;
                if (range < left_min_distance) {
                    left_min_distance = range;
                }
            }
            if (isAngleInSector(angle_deg, right_center_deg, side_half_width_deg_right)) {
                ++right_points;
                if (range < right_min_distance) {
                    right_min_distance = range;
                }
            }
            continue;
        }

        ++valid_points;
        if (range < min_distance) {
            min_distance = range;
            min_angle_deg = static_cast<float>(angle_deg);
        }
        if (isAngleInSector(angle_deg, angle_center_deg, front_half_width_deg)) {
            ++front_points;
            if (range < front_min_distance) {
                front_min_distance = range;
            }
        }
        if (isAngleInSector(angle_deg, left_center_deg, side_half_width_deg_left)) {
            ++left_points;
            if (range < left_min_distance) {
                left_min_distance = range;
            }
        }
        if (isAngleInSector(angle_deg, right_center_deg, side_half_width_deg_right)) {
            ++right_points;
            if (range < right_min_distance) {
                right_min_distance = range;
            }
        }
    }

    obstacle_state.points = valid_points;
    obstacle_state.front_points = front_points;
    obstacle_state.left_points = left_points;
    obstacle_state.right_points = right_points;
    if (valid_points > 0) {
        obstacle_state.min_distance = min_distance;
        obstacle_state.angle_deg = min_angle_deg;
        if (min_distance < distance_threshold) {
            obstacle_state.detected = true;
        }
    }
    if (front_points > 0) {
        obstacle_state.front_min_distance = front_min_distance;
    }
    if (left_points > 0) {
        obstacle_state.left_min_distance = left_min_distance;
    }
    if (right_points > 0) {
        obstacle_state.right_min_distance = right_min_distance;
    }
    summary.valid = true;
    summary.front_min_distance_m =
        front_points > 0 ? front_min_distance : bishe::common::kNoObstacleDistanceMeters;
    summary.left_min_distance_m =
        left_points > 0 ? left_min_distance : bishe::common::kNoObstacleDistanceMeters;
    summary.right_min_distance_m =
        right_points > 0 ? right_min_distance : bishe::common::kNoObstacleDistanceMeters;
    summary.front_valid_points = front_points;
    summary.left_valid_points = left_points;
    summary.right_valid_points = right_points;
    summary.obstacle_detected =
        obstacle_state.detected ||
        (front_points > 0 && front_min_distance < distance_threshold);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        obstacle_state_ = obstacle_state;
    }

    if (snapshot_pool_) {
        snapshot_pool_->pushLidar(summary);
    }

    bishe::sensors::LidarSample sample;
    sample.header.stamp = sample_stamp;
    sample.header.sequence = sequence;
    sample.obstacle = obstacle_state;
    if (data_pool_) {
        data_pool_->pushLidar(sample);
    }
}

}  // namespace bishe::sensors::lidar
