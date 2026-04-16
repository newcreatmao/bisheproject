#pragma once

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "sensors/sensor_data_pool.hpp"

namespace bishe::sensors::gps
{

class GpsSensor
{
public:
    GpsSensor(
        rclcpp::Node * node,
        const std::shared_ptr<bishe::sensors::SensorDataPool> & data_pool);

    bool hasFix() const;
    double getLatitude() const;
    double getLongitude() const;
    double getAltitude() const;
    int getStatus() const;

private:
    void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

private:
    rclcpp::Node * node_;
    std::shared_ptr<bishe::sensors::SensorDataPool> data_pool_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
    mutable std::mutex state_mutex_;

    std::string fix_topic_;
    std::uint64_t sample_sequence_;

    bool has_fix_;
    bool last_fix_valid_;
    double latitude_;
    double longitude_;
    double altitude_;
    int status_;
};

}  // namespace bishe::sensors::gps
