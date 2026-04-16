#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "common/sensor_snapshot_pool.hpp"
#include "sensors/sensor_data_pool.hpp"
#include "vehicle/vehicle_state.hpp"

namespace bishe::sensors::camera
{

class CameraSensor
{
public:
    CameraSensor(
        rclcpp::Node * node,
        const std::shared_ptr<bishe::sensors::SensorDataPool> & data_pool,
        const std::shared_ptr<bishe::common::SensorSnapshotPool> & snapshot_pool);

    bool isOnline() const;
    std::string getDepthTopic() const;
    std::string getEncoding() const;
    int getWidth() const;
    int getHeight() const;
    bishe::vehicle::VisionObstacleState getVisionObstacleState() const;

private:
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void resetVisionObstacleState();
    bishe::common::DepthObstacleSummary updateVisionObstacleFrom16UC1(
        const sensor_msgs::msg::Image::SharedPtr msg);
    bishe::common::DepthObstacleSummary updateVisionObstacleFrom32FC1(
        const sensor_msgs::msg::Image::SharedPtr msg);
    void publishSample(const rclcpp::Time & stamp, std::uint64_t sequence);

private:
    rclcpp::Node * node_;
    std::shared_ptr<bishe::sensors::SensorDataPool> data_pool_;
    std::shared_ptr<bishe::common::SensorSnapshotPool> snapshot_pool_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    mutable std::mutex state_mutex_;

    std::string depth_topic_;
    double online_timeout_sec_;
    double obstacle_max_distance_m_;
    double depth_min_valid_m_;
    double roi_top_ratio_;
    double roi_bottom_ratio_;
    double roi_left_ratio_;
    double roi_right_ratio_;
    int min_obstacle_pixels_;
    std::uint64_t sample_sequence_;
    bool has_depth_frame_;

    rclcpp::Time last_image_time_;
    std::string encoding_;
    int width_;
    int height_;
    bishe::vehicle::VisionObstacleState vision_obstacle_state_;
    bishe::common::DepthObstacleSummary depth_obstacle_summary_;
};

}  // namespace bishe::sensors::camera
