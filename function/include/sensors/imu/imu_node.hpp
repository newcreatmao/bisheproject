#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "common/sensor_snapshot_pool.hpp"
#include "sensors/sensor_data_pool.hpp"

namespace bishe::sensors::imu
{

class ImuSensor
{
public:
    ImuSensor(
        rclcpp::Node * node,
        const std::shared_ptr<bishe::sensors::SensorDataPool> & data_pool,
        const std::shared_ptr<bishe::common::SensorSnapshotPool> & snapshot_pool);

    double getRoll() const;
    double getPitch() const;
    double getYaw() const;

    double getRollOffset() const;
    double getPitchOffset() const;
    double getYawOffset() const;

    bool isCalibrated() const;

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    static void quaternionToEuler(
        double x, double y, double z, double w,
        double &roll, double &pitch, double &yaw);
    static void eulerToQuaternion(
        double roll, double pitch, double yaw,
        double &x, double &y, double &z, double &w);

    static double normalizeAngle(double angle);
    static bool normalizeQuaternion(double & x, double & y, double & z, double & w);
    static void alignQuaternionHemisphere(
        double reference_x,
        double reference_y,
        double reference_z,
        double reference_w,
        double & x,
        double & y,
        double & z,
        double & w);
    static void quaternionConjugate(
        double x, double y, double z, double w,
        double & out_x, double & out_y, double & out_z, double & out_w);
    static void quaternionMultiply(
        double ax, double ay, double az, double aw,
        double bx, double by, double bz, double bw,
        double & out_x, double & out_y, double & out_z, double & out_w);
    static geometry_msgs::msg::Vector3 rotateVector(
        double qx, double qy, double qz, double qw,
        const geometry_msgs::msg::Vector3 & vector);
    static void printRotationMatrix(
        rclcpp::Logger logger,
        double qx, double qy, double qz, double qw);

private:
    rclcpp::Node * node_;
    std::shared_ptr<bishe::sensors::SensorDataPool> data_pool_;
    std::shared_ptr<bishe::common::SensorSnapshotPool> snapshot_pool_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr corrected_imu_pub_;
    mutable std::mutex state_mutex_;

    std::string imu_topic_;
    std::string corrected_imu_topic_;
    std::string corrected_imu_frame_;
    bool publish_corrected_imu_;
    bool mount_correction_enabled_;
    bool preserve_yaw_during_mount_correction_;
    int calibration_samples_;
    std::uint64_t sample_sequence_;

    bool calibrated_;
    int current_samples_;

    double roll_;
    double pitch_;
    double yaw_;

    double roll_offset_;
    double pitch_offset_;
    double yaw_offset_;

    double roll_sum_;
    double pitch_sum_;
    double yaw_sum_;

    bool mount_quaternion_ready_;
    double mount_qx_;
    double mount_qy_;
    double mount_qz_;
    double mount_qw_;
    double quat_sum_x_;
    double quat_sum_y_;
    double quat_sum_z_;
    double quat_sum_w_;
    double quat_reference_x_;
    double quat_reference_y_;
    double quat_reference_z_;
    double quat_reference_w_;
    bool quat_reference_initialized_;
};

}  // namespace bishe::sensors::imu
