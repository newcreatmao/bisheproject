#include "sensors/imu/imu_node.hpp"

#include <cmath>
#include <functional>

namespace bishe::sensors::imu
{

ImuSensor::ImuSensor(
    rclcpp::Node * node,
    const std::shared_ptr<bishe::sensors::SensorDataPool> & data_pool,
    const std::shared_ptr<bishe::common::SensorSnapshotPool> & snapshot_pool)
    : node_(node),
      data_pool_(data_pool),
      snapshot_pool_(snapshot_pool),
      corrected_imu_topic_("/imu/data_corrected"),
      corrected_imu_frame_("base_link"),
      publish_corrected_imu_(true),
      mount_correction_enabled_(true),
      preserve_yaw_during_mount_correction_(true),
      sample_sequence_(0),
      calibrated_(false),
      current_samples_(0),
      roll_(0.0),
      pitch_(0.0),
      yaw_(0.0),
      roll_offset_(0.0),
      pitch_offset_(0.0),
      yaw_offset_(0.0),
      roll_sum_(0.0),
      pitch_sum_(0.0),
      yaw_sum_(0.0),
      mount_quaternion_ready_(false),
      mount_qx_(0.0),
      mount_qy_(0.0),
      mount_qz_(0.0),
      mount_qw_(1.0),
      quat_sum_x_(0.0),
      quat_sum_y_(0.0),
      quat_sum_z_(0.0),
      quat_sum_w_(0.0),
      quat_reference_x_(0.0),
      quat_reference_y_(0.0),
      quat_reference_z_(0.0),
      quat_reference_w_(1.0),
      quat_reference_initialized_(false)
{
    imu_topic_ = node_->declare_parameter<std::string>("imu_topic", "/imu/data_raw");
    corrected_imu_topic_ =
        node_->declare_parameter<std::string>("corrected_imu_topic", "/imu/data_corrected");
    corrected_imu_frame_ =
        node_->declare_parameter<std::string>("corrected_imu_frame", "base_link");
    publish_corrected_imu_ =
        node_->declare_parameter<bool>("publish_corrected_imu", true);
    mount_correction_enabled_ =
        node_->declare_parameter<bool>("imu_mount_correction_enabled", true);
    preserve_yaw_during_mount_correction_ =
        node_->declare_parameter<bool>("imu_mount_preserve_yaw", true);
    calibration_samples_ = node_->declare_parameter<int>("calibration_samples", 100);
    if (calibration_samples_ < 1) {
        calibration_samples_ = 1;
    }
    callback_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_,
        50,
        std::bind(&ImuSensor::imuCallback, this, std::placeholders::_1),
        subscription_options);

    euler_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "/imu/euler_corrected", 10);
    if (publish_corrected_imu_) {
        corrected_imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
            corrected_imu_topic_, 10);
    }

    RCLCPP_INFO(node_->get_logger(), "imu sensor ready");
    RCLCPP_INFO(node_->get_logger(), "imu topic: %s", imu_topic_.c_str());
    RCLCPP_INFO(node_->get_logger(), "imu mount correction: %s",
                mount_correction_enabled_ ? "enabled" : "disabled");
    RCLCPP_INFO(node_->get_logger(), "imu mount preserve yaw: %s",
                preserve_yaw_during_mount_correction_ ? "enabled" : "disabled");
    RCLCPP_INFO(node_->get_logger(), "imu calibration samples: %d", calibration_samples_);
    RCLCPP_INFO(node_->get_logger(), "imu corrected topic: %s", corrected_imu_topic_.c_str());
    RCLCPP_INFO(node_->get_logger(), "imu corrected frame: %s", corrected_imu_frame_.c_str());
}

double ImuSensor::getRoll() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return roll_;
}

double ImuSensor::getPitch() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return pitch_;
}

double ImuSensor::getYaw() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return yaw_;
}

double ImuSensor::getRollOffset() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return roll_offset_;
}

double ImuSensor::getPitchOffset() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return pitch_offset_;
}

double ImuSensor::getYawOffset() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return yaw_offset_;
}

bool ImuSensor::isCalibrated() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return calibrated_;
}

void ImuSensor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    const rclcpp::Time sample_stamp =
        (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ?
        node_->now() : rclcpp::Time(msg->header.stamp);
    const std::uint64_t sequence = ++sample_sequence_;

    double raw_qx = msg->orientation.x;
    double raw_qy = msg->orientation.y;
    double raw_qz = msg->orientation.z;
    double raw_qw = msg->orientation.w;
    const bool raw_orientation_valid = normalizeQuaternion(raw_qx, raw_qy, raw_qz, raw_qw);

    double raw_roll = 0.0;
    double raw_pitch = 0.0;
    double raw_yaw = 0.0;
    if (raw_orientation_valid) {
        quaternionToEuler(raw_qx, raw_qy, raw_qz, raw_qw, raw_roll, raw_pitch, raw_yaw);
    }

    bool just_calibrated = false;
    bool calibrated = false;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    double roll_offset = 0.0;
    double pitch_offset = 0.0;
    double yaw_offset = 0.0;
    double corrected_qx = raw_orientation_valid ? raw_qx : 0.0;
    double corrected_qy = raw_orientation_valid ? raw_qy : 0.0;
    double corrected_qz = raw_orientation_valid ? raw_qz : 0.0;
    double corrected_qw = raw_orientation_valid ? raw_qw : 1.0;
    geometry_msgs::msg::Vector3 corrected_angular_velocity = msg->angular_velocity;
    geometry_msgs::msg::Vector3 corrected_linear_acceleration = msg->linear_acceleration;
    double mount_qx = 0.0;
    double mount_qy = 0.0;
    double mount_qz = 0.0;
    double mount_qw = 1.0;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!mount_correction_enabled_) {
            calibrated_ = raw_orientation_valid;
            mount_quaternion_ready_ = false;
            current_samples_ = 0;
            roll_offset_ = 0.0;
            pitch_offset_ = 0.0;
            yaw_offset_ = 0.0;
        }

        if (!calibrated_ && raw_orientation_valid) {
            double aligned_qx = raw_qx;
            double aligned_qy = raw_qy;
            double aligned_qz = raw_qz;
            double aligned_qw = raw_qw;

            if (!quat_reference_initialized_) {
                quat_reference_x_ = raw_qx;
                quat_reference_y_ = raw_qy;
                quat_reference_z_ = raw_qz;
                quat_reference_w_ = raw_qw;
                quat_reference_initialized_ = true;
            } else {
                alignQuaternionHemisphere(
                    quat_reference_x_,
                    quat_reference_y_,
                    quat_reference_z_,
                    quat_reference_w_,
                    aligned_qx,
                    aligned_qy,
                    aligned_qz,
                    aligned_qw);
            }

            quat_sum_x_ += aligned_qx;
            quat_sum_y_ += aligned_qy;
            quat_sum_z_ += aligned_qz;
            quat_sum_w_ += aligned_qw;
            roll_sum_ += raw_roll;
            pitch_sum_ += raw_pitch;
            yaw_sum_ += raw_yaw;
            current_samples_++;

            if (current_samples_ >= calibration_samples_) {
                double reference_qx = quat_sum_x_ / static_cast<double>(current_samples_);
                double reference_qy = quat_sum_y_ / static_cast<double>(current_samples_);
                double reference_qz = quat_sum_z_ / static_cast<double>(current_samples_);
                double reference_qw = quat_sum_w_ / static_cast<double>(current_samples_);

                if (!normalizeQuaternion(reference_qx, reference_qy, reference_qz, reference_qw)) {
                    mount_qx_ = 0.0;
                    mount_qy_ = 0.0;
                    mount_qz_ = 0.0;
                    mount_qw_ = 1.0;
                } else if (preserve_yaw_during_mount_correction_) {
                    // Keep the startup heading and only remove the tilt portion.
                    // This avoids treating the vehicle's world yaw as a mount error.
                    double reference_roll = 0.0;
                    double reference_pitch = 0.0;
                    double reference_yaw = 0.0;
                    quaternionToEuler(
                        reference_qx,
                        reference_qy,
                        reference_qz,
                        reference_qw,
                        reference_roll,
                        reference_pitch,
                        reference_yaw);

                    double desired_qx = 0.0;
                    double desired_qy = 0.0;
                    double desired_qz = 0.0;
                    double desired_qw = 1.0;
                    eulerToQuaternion(
                        0.0,
                        0.0,
                        reference_yaw,
                        desired_qx,
                        desired_qy,
                        desired_qz,
                        desired_qw);

                    double desired_inverse_x = 0.0;
                    double desired_inverse_y = 0.0;
                    double desired_inverse_z = 0.0;
                    double desired_inverse_w = 1.0;
                    quaternionConjugate(
                        desired_qx,
                        desired_qy,
                        desired_qz,
                        desired_qw,
                        desired_inverse_x,
                        desired_inverse_y,
                        desired_inverse_z,
                        desired_inverse_w);

                    quaternionMultiply(
                        desired_inverse_x,
                        desired_inverse_y,
                        desired_inverse_z,
                        desired_inverse_w,
                        reference_qx,
                        reference_qy,
                        reference_qz,
                        reference_qw,
                        mount_qx_,
                        mount_qy_,
                        mount_qz_,
                        mount_qw_);
                    if (!normalizeQuaternion(mount_qx_, mount_qy_, mount_qz_, mount_qw_)) {
                        mount_qx_ = 0.0;
                        mount_qy_ = 0.0;
                        mount_qz_ = 0.0;
                        mount_qw_ = 1.0;
                    }
                } else {
                    mount_qx_ = reference_qx;
                    mount_qy_ = reference_qy;
                    mount_qz_ = reference_qz;
                    mount_qw_ = reference_qw;
                }

                mount_quaternion_ready_ = true;
                quaternionToEuler(
                    mount_qx_, mount_qy_, mount_qz_, mount_qw_,
                    roll_offset_, pitch_offset_, yaw_offset_);
                calibrated_ = true;
                just_calibrated = true;
            }
        }

        if (calibrated_ && mount_quaternion_ready_ && raw_orientation_valid) {
            double mount_inverse_x = 0.0;
            double mount_inverse_y = 0.0;
            double mount_inverse_z = 0.0;
            double mount_inverse_w = 1.0;
            quaternionConjugate(
                mount_qx_, mount_qy_, mount_qz_, mount_qw_,
                mount_inverse_x, mount_inverse_y, mount_inverse_z, mount_inverse_w);
            quaternionMultiply(
                raw_qx, raw_qy, raw_qz, raw_qw,
                mount_inverse_x, mount_inverse_y, mount_inverse_z, mount_inverse_w,
                corrected_qx, corrected_qy, corrected_qz, corrected_qw);
            normalizeQuaternion(corrected_qx, corrected_qy, corrected_qz, corrected_qw);
            quaternionToEuler(
                corrected_qx, corrected_qy, corrected_qz, corrected_qw,
                roll_, pitch_, yaw_);
            corrected_angular_velocity = rotateVector(
                mount_qx_, mount_qy_, mount_qz_, mount_qw_, msg->angular_velocity);
            corrected_linear_acceleration = rotateVector(
                mount_qx_, mount_qy_, mount_qz_, mount_qw_, msg->linear_acceleration);
        } else if (raw_orientation_valid) {
            roll_ = raw_roll;
            pitch_ = raw_pitch;
            yaw_ = raw_yaw;
        } else {
            roll_ = 0.0;
            pitch_ = 0.0;
            yaw_ = 0.0;
        }

        calibrated = calibrated_;
        roll = roll_;
        pitch = pitch_;
        yaw = yaw_;
        roll_offset = roll_offset_;
        pitch_offset = pitch_offset_;
        yaw_offset = yaw_offset_;
        mount_qx = mount_qx_;
        mount_qy = mount_qy_;
        mount_qz = mount_qz_;
        mount_qw = mount_qw_;
    }

    geometry_msgs::msg::Vector3Stamped euler_msg;
    euler_msg.header = msg->header;
    if (calibrated) {
        euler_msg.header.frame_id = corrected_imu_frame_;
    }
    euler_msg.vector.x = roll;
    euler_msg.vector.y = pitch;
    euler_msg.vector.z = yaw;
    euler_pub_->publish(euler_msg);

    if (publish_corrected_imu_ && corrected_imu_pub_) {
        sensor_msgs::msg::Imu corrected_msg = *msg;
        if (calibrated) {
            corrected_msg.header.frame_id = corrected_imu_frame_;
        }
        corrected_msg.orientation.x = corrected_qx;
        corrected_msg.orientation.y = corrected_qy;
        corrected_msg.orientation.z = corrected_qz;
        corrected_msg.orientation.w = corrected_qw;
        corrected_msg.angular_velocity = corrected_angular_velocity;
        corrected_msg.linear_acceleration = corrected_linear_acceleration;
        corrected_imu_pub_->publish(corrected_msg);
    }

    if (just_calibrated) {
        RCLCPP_INFO(node_->get_logger(),
                    "IMU mount calibration ready. offsets(rad): roll=%.6f pitch=%.6f yaw=%.6f",
                    roll_offset, pitch_offset, yaw_offset);
        RCLCPP_INFO(node_->get_logger(),
                    "IMU mount calibration ready. offsets(deg): roll=%.3f pitch=%.3f yaw=%.3f",
                    roll_offset * 180.0 / M_PI,
                    pitch_offset * 180.0 / M_PI,
                    yaw_offset * 180.0 / M_PI);
        printRotationMatrix(node_->get_logger(), mount_qx, mount_qy, mount_qz, mount_qw);
    }

    bishe::sensors::ImuSample sample;
    sample.header.stamp = sample_stamp;
    sample.header.sequence = sequence;
    sample.calibrated = calibrated;
    sample.orientation_x = corrected_qx;
    sample.orientation_y = corrected_qy;
    sample.orientation_z = corrected_qz;
    sample.orientation_w = corrected_qw;
    sample.angular_velocity_x = corrected_angular_velocity.x;
    sample.angular_velocity_y = corrected_angular_velocity.y;
    sample.angular_velocity_z = corrected_angular_velocity.z;
    sample.linear_acceleration_x = corrected_linear_acceleration.x;
    sample.linear_acceleration_y = corrected_linear_acceleration.y;
    sample.linear_acceleration_z = corrected_linear_acceleration.z;
    sample.roll = roll;
    sample.pitch = pitch;
    sample.yaw = yaw;
    sample.roll_offset = roll_offset;
    sample.pitch_offset = pitch_offset;
    sample.yaw_offset = yaw_offset;
    if (data_pool_) {
        data_pool_->pushImu(sample);
    }

    if (snapshot_pool_) {
        bishe::common::ImuStateSummary summary;
        summary.stamp = sample_stamp;
        summary.valid = calibrated;
        summary.roll = roll;
        summary.pitch = pitch;
        summary.yaw = yaw;
        summary.sequence = sequence;
        snapshot_pool_->pushImu(summary);
    }
}

double ImuSensor::normalizeAngle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

bool ImuSensor::normalizeQuaternion(double & x, double & y, double & z, double & w)
{
    const double norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (norm <= 1e-9) {
        return false;
    }
    x /= norm;
    y /= norm;
    z /= norm;
    w /= norm;
    return true;
}

void ImuSensor::alignQuaternionHemisphere(
    double reference_x,
    double reference_y,
    double reference_z,
    double reference_w,
    double & x,
    double & y,
    double & z,
    double & w)
{
    const double dot =
        reference_x * x +
        reference_y * y +
        reference_z * z +
        reference_w * w;
    if (dot < 0.0) {
        x = -x;
        y = -y;
        z = -z;
        w = -w;
    }
}

void ImuSensor::quaternionConjugate(
    double x, double y, double z, double w,
    double & out_x, double & out_y, double & out_z, double & out_w)
{
    out_x = -x;
    out_y = -y;
    out_z = -z;
    out_w = w;
}

void ImuSensor::quaternionMultiply(
    double ax, double ay, double az, double aw,
    double bx, double by, double bz, double bw,
    double & out_x, double & out_y, double & out_z, double & out_w)
{
    out_w = aw * bw - ax * bx - ay * by - az * bz;
    out_x = aw * bx + ax * bw + ay * bz - az * by;
    out_y = aw * by - ax * bz + ay * bw + az * bx;
    out_z = aw * bz + ax * by - ay * bx + az * bw;
}

geometry_msgs::msg::Vector3 ImuSensor::rotateVector(
    double qx, double qy, double qz, double qw,
    const geometry_msgs::msg::Vector3 & vector)
{
    double vx = vector.x;
    double vy = vector.y;
    double vz = vector.z;
    double tmp_x = 0.0;
    double tmp_y = 0.0;
    double tmp_z = 0.0;
    double tmp_w = 0.0;
    double out_x = 0.0;
    double out_y = 0.0;
    double out_z = 0.0;
    double out_w = 0.0;
    double q_conj_x = 0.0;
    double q_conj_y = 0.0;
    double q_conj_z = 0.0;
    double q_conj_w = 1.0;

    quaternionMultiply(qx, qy, qz, qw, vx, vy, vz, 0.0, tmp_x, tmp_y, tmp_z, tmp_w);
    quaternionConjugate(qx, qy, qz, qw, q_conj_x, q_conj_y, q_conj_z, q_conj_w);
    quaternionMultiply(
        tmp_x, tmp_y, tmp_z, tmp_w,
        q_conj_x, q_conj_y, q_conj_z, q_conj_w,
        out_x, out_y, out_z, out_w);

    geometry_msgs::msg::Vector3 rotated;
    rotated.x = out_x;
    rotated.y = out_y;
    rotated.z = out_z;
    return rotated;
}

void ImuSensor::printRotationMatrix(
    rclcpp::Logger logger,
    double qx, double qy, double qz, double qw)
{
    const double r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double r01 = 2.0 * (qx * qy - qz * qw);
    const double r02 = 2.0 * (qx * qz + qy * qw);
    const double r10 = 2.0 * (qx * qy + qz * qw);
    const double r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
    const double r12 = 2.0 * (qy * qz - qx * qw);
    const double r20 = 2.0 * (qx * qz - qy * qw);
    const double r21 = 2.0 * (qy * qz + qx * qw);
    const double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);

    RCLCPP_INFO(logger, "IMU mount rotation matrix:");
    RCLCPP_INFO(logger, "[%.6f %.6f %.6f]", r00, r01, r02);
    RCLCPP_INFO(logger, "[%.6f %.6f %.6f]", r10, r11, r12);
    RCLCPP_INFO(logger, "[%.6f %.6f %.6f]", r20, r21, r22);
}

void ImuSensor::quaternionToEuler(
    double x, double y, double z, double w,
    double &roll, double &pitch, double &yaw)
{
    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

void ImuSensor::eulerToQuaternion(
    double roll, double pitch, double yaw,
    double &x, double &y, double &z, double &w)
{
    const double half_roll = roll * 0.5;
    const double half_pitch = pitch * 0.5;
    const double half_yaw = yaw * 0.5;

    const double sin_roll = std::sin(half_roll);
    const double cos_roll = std::cos(half_roll);
    const double sin_pitch = std::sin(half_pitch);
    const double cos_pitch = std::cos(half_pitch);
    const double sin_yaw = std::sin(half_yaw);
    const double cos_yaw = std::cos(half_yaw);

    w = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
    x = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
    y = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
    z = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
    normalizeQuaternion(x, y, z, w);
}

}  // namespace bishe::sensors::imu
