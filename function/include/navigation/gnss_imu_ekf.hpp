#pragma once

#include <array>
#include <string>

namespace navigation {

struct GnssImuEkfConfig {
  double gps_expected_period_s = 0.25;
  double gps_fresh_s = 0.80;
  double max_gps_hold_s = 1.50;
  double max_degraded_hold_s = 3.00;

  double max_predict_dt_s = 0.20;

  double gps_jump_gate_m = 3.0;
  double static_speed_gate_mps = 0.05;
  double static_drift_gate_m = 0.8;

  double yaw_align_min_speed_mps = 0.25;
  double yaw_align_alpha = 0.025;
  int yaw_align_min_stable_frames = 3;

  double pos_meas_stddev_min_m = 0.8;
  double pos_meas_stddev_max_m = 5.0;
  double speed_meas_stddev_mps = 0.20;
  double imu_yaw_meas_stddev_deg = 1.5;
  double gps_course_stddev_deg = 8.0;

  double process_pos_noise = 0.05;
  double process_speed_noise = 0.30;
  double process_yaw_noise_deg = 1.0;
  double process_bias_noise = 0.0005;

  double gps_hold_speed_limit_cm_s = 30.0;
  double degraded_speed_limit_cm_s = 18.0;
};

enum class FusionMode {
  kUninitialized,
  kAligning,
  kFused,
  kGpsHold,
  kDegraded,
  kInvalid
};

struct GnssPositionSample {
  double stamp_s = 0.0;
  double x_m = 0.0;
  double y_m = 0.0;
  double horizontal_stddev_m = 0.0;
  bool valid = false;
};

struct GnssVelocitySample {
  double stamp_s = 0.0;
  double speed_mps = 0.0;
  double course_yaw_rad = 0.0;
  bool speed_valid = false;
  bool course_valid = false;
};

struct ImuSample {
  double stamp_s = 0.0;
  double yaw_rad = 0.0;
  double gyro_z_rad_s = 0.0;
  bool yaw_valid = false;
  bool gyro_valid = false;
};

struct MotionHint {
  double command_speed_mps = 0.0;
  int steering_encoder = 0;
  bool auto_task_running = false;
  bool avoidance_active = false;
  bool emergency_active = false;
};

struct GnssImuEkfState {
  double stamp_s = 0.0;
  double x_m = 0.0;
  double y_m = 0.0;
  double speed_mps = 0.0;
  double yaw_rad = 0.0;
  double heading_deg = 0.0;
  double gyro_bias_rad_s = 0.0;

  bool valid = false;
  bool initialized = false;
  bool yaw_aligned = false;
  FusionMode mode = FusionMode::kUninitialized;

  double quality = 0.0;
  double yaw_offset_rad = 0.0;
  double holdover_ms = -1.0;
  double gps_age_ms = -1.0;
  double imu_age_ms = -1.0;

  bool last_gps_used = false;
  bool last_gps_rejected = false;
  int gps_used_count = 0;
  int gps_rejected_count = 0;
  double last_gps_innovation_m = 0.0;
  std::string last_reject_reason;
};

class GnssImuEkf {
 public:
  explicit GnssImuEkf(const GnssImuEkfConfig& config = {});

  void reset();
  void resetToPose(double stamp_s, double x_m, double y_m, double yaw_rad);
  void setConfig(const GnssImuEkfConfig& config);
  void setMotionHint(const MotionHint& hint);

  void updateImu(const ImuSample& imu);
  void updateGnssPosition(const GnssPositionSample& gps);
  void updateGnssVelocity(
      const GnssVelocitySample& vel,
      bool allow_course_alignment,
      bool vehicle_commanded_moving);

  GnssImuEkfState state(double now_s) const;

 private:
  static constexpr std::size_t kStateSize = 5;
  using Vec5 = std::array<double, kStateSize>;
  using Mat5 = std::array<std::array<double, kStateSize>, kStateSize>;

  static double wrapAngleRad(double value_rad);
  static double headingDegFromYawRad(double yaw_rad);
  static double clampGpsStddev(
      double stddev_m,
      const GnssImuEkfConfig& config);

  void initializeCovariance(
      double pos_stddev_m,
      double yaw_stddev_rad);
  void predictToStamp(
      double stamp_s,
      bool gyro_valid,
      double gyro_z_rad_s);
  void predictStep(double dt_s, double gyro_z_rad_s);
  void applyScalarMeasurement(
      std::size_t state_index,
      double measurement,
      double variance,
      bool angular_residual);
  void applyPositionMeasurement(
      double x_m,
      double y_m,
      double variance_x,
      double variance_y);
  void finalizeStateAfterUpdate();
  double currentSpeedReferenceMps() const;
  bool imuFresh(double now_s) const;

  GnssImuEkfConfig config_;
  MotionHint motion_hint_;

  Vec5 x_{};
  Mat5 p_{};

  bool initialized_ = false;
  bool have_cached_yaw_ = false;
  bool last_imu_yaw_valid_ = false;
  bool last_gyro_valid_ = false;
  bool yaw_aligned_ = false;
  bool last_gps_used_ = false;
  bool last_gps_rejected_ = false;

  double cached_yaw_measurement_rad_ = 0.0;
  double last_imu_yaw_rad_ = 0.0;
  double last_gyro_z_rad_s_ = 0.0;
  double yaw_offset_rad_ = 0.0;
  double last_predict_stamp_s_ = 0.0;
  double last_imu_stamp_s_ = 0.0;
  double last_imu_obs_stamp_s_ = 0.0;
  double last_gps_position_stamp_s_ = 0.0;
  double last_gps_used_stamp_s_ = 0.0;
  double last_velocity_stamp_s_ = 0.0;
  double last_speed_measurement_mps_ = 0.0;
  double last_horizontal_stddev_m_ = 0.0;
  double last_course_yaw_rad_ = 0.0;
  bool last_course_valid_ = false;
  int stable_course_frames_ = 0;

  int gps_used_count_ = 0;
  int gps_rejected_count_ = 0;
  double last_gps_innovation_m_ = 0.0;
  std::string last_reject_reason_;
};

const char* fusionModeToString(FusionMode mode);

}  // namespace navigation
