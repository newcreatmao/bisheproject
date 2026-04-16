#include "sensors/gps/gps_node.hpp"

#include <cmath>
#include <functional>

namespace bishe::sensors::gps
{

GpsSensor::GpsSensor(
    rclcpp::Node * node,
    const std::shared_ptr<bishe::sensors::SensorDataPool> & data_pool)
    : node_(node),
      data_pool_(data_pool),
      sample_sequence_(0),
      has_fix_(false),
      last_fix_valid_(false),
      latitude_(0.0),
      longitude_(0.0),
      altitude_(0.0),
      status_(-1)
{
    fix_topic_ = node_->declare_parameter<std::string>("fix_topic", "/fix");
    callback_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;

    fix_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        fix_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&GpsSensor::fixCallback, this, std::placeholders::_1),
        subscription_options);

    RCLCPP_INFO(node_->get_logger(), "gps sensor ready");
    RCLCPP_INFO(node_->get_logger(), "gps subscribe topic: %s", fix_topic_.c_str());
}

bool GpsSensor::hasFix() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return has_fix_;
}

double GpsSensor::getLatitude() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latitude_;
}

double GpsSensor::getLongitude() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return longitude_;
}

double GpsSensor::getAltitude() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return altitude_;
}

int GpsSensor::getStatus() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return status_;
}

void GpsSensor::fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    const double latitude = msg->latitude;
    const double longitude = msg->longitude;
    const double altitude = msg->altitude;
    const int status = msg->status.status;
    const bool has_fix = (status != -1);

    const bool valid_position =
        std::isfinite(latitude) &&
        std::isfinite(longitude) &&
        std::isfinite(altitude);
    const bool current_fix_valid = has_fix && valid_position;
    bool last_fix_valid = false;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latitude_ = latitude;
        longitude_ = longitude;
        altitude_ = altitude;
        status_ = status;
        has_fix_ = has_fix;
        last_fix_valid = last_fix_valid_;
        last_fix_valid_ = current_fix_valid;
    }

    bishe::sensors::GpsSample sample;
    sample.header.stamp =
        (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ?
        node_->now() : rclcpp::Time(msg->header.stamp);
    sample.header.sequence = ++sample_sequence_;
    sample.has_fix = has_fix;
    sample.valid_position = valid_position;
    sample.latitude = latitude;
    sample.longitude = longitude;
    sample.altitude = altitude;
    sample.status = status;
    if (data_pool_) {
        data_pool_->pushGps(sample);
    }

    if (!current_fix_valid) {
        if (last_fix_valid) {
            RCLCPP_WARN(
                node_->get_logger(),
                "GPS fix lost: status=%d",
                status);
        }
        return;
    }

    if (!last_fix_valid) {
        RCLCPP_INFO(
            node_->get_logger(),
            "GPS fix ready: status=%d lat=%.8f lon=%.8f alt=%.3f",
            status,
            latitude,
            longitude,
            altitude);
    }

    RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        5000,
        "GPS update: status=%d lat=%.8f lon=%.8f alt=%.3f",
        status,
        latitude,
        longitude,
        altitude);
}

}  // namespace bishe::sensors::gps
