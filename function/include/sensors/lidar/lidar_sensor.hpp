#pragma once

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "common/sensor_snapshot_pool.hpp"
#include "sensors/sensor_data_pool.hpp"
#include "vehicle/vehicle_state.hpp"

namespace bishe::sensors::lidar
{

class LidarSensor
{
public:
    LidarSensor(
        rclcpp::Node * node,
        const std::shared_ptr<bishe::sensors::SensorDataPool> & data_pool,
        const std::shared_ptr<bishe::common::SensorSnapshotPool> & snapshot_pool);

    void refreshParameters();
    bishe::vehicle::ObstacleState getObstacleState() const;

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    static double normalizeAngleDeg(double angle_deg);
    static bool isAngleInSector(
        double angle_deg,
        double angle_center_deg,
        double angle_half_width_deg);

private:
    rclcpp::Node * node_;
    std::shared_ptr<bishe::sensors::SensorDataPool> data_pool_;
    std::shared_ptr<bishe::common::SensorSnapshotPool> snapshot_pool_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    mutable std::mutex state_mutex_;

    bishe::vehicle::ObstacleState obstacle_state_;
    std::uint64_t sample_sequence_ = 0;

    bool obstacle_detection_enabled_ = true;
    double angle_center_deg_ = 270.0;
    double angle_half_width_deg_ = 30.0;
    double distance_threshold_ = 2.0;
    double local_front_sector_deg_ = 40.0;
    double local_left_sector_deg_ = 35.0;
    double local_right_sector_deg_ = 35.0;
};

}  // namespace bishe::sensors::lidar
