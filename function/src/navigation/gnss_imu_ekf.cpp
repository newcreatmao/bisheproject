#include "navigation/gnss_imu_ekf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation {

namespace {

constexpr double kPi = 3.14159265358979323846;

double degToRad(double value_deg) {
  return value_deg * kPi / 180.0;
}

double radToDeg(double value_rad) {
  return value_rad * 180.0 / kPi;
}

double clampNonNegative(double value) {
  return value < 0.0 ? 0.0 : value;
}

}  // namespace

const char* fusionModeToString(FusionMode mode) {
  switch (mode) {
    case FusionMode::kUninitialized:
      return "kUninitialized";
    case FusionMode::kAligning:
      return "kAligning";
    case FusionMode::kFused:
      return "kFused";
    case FusionMode::kGpsHold:
      return "kGpsHold";
    case FusionMode::kDegraded:
      return "kDegraded";
    case FusionMode::kInvalid:
      return "kInvalid";
  }
  return "kInvalid";
}

GnssImuEkf::GnssImuEkf(const GnssImuEkfConfig& config) : config_(config) {
  reset();
}

double GnssImuEkf::wrapAngleRad(double value_rad) {
  while (value_rad > kPi) {
    value_rad -= 2.0 * kPi;
  }
  while (value_rad < -kPi) {
    value_rad += 2.0 * kPi;
  }
  return value_rad;
}

double GnssImuEkf::headingDegFromYawRad(double yaw_rad) {
  return wrapAngleRad((kPi * 0.5) - yaw_rad) * 180.0 / kPi;
}

double GnssImuEkf::clampGpsStddev(
    double stddev_m,
    const GnssImuEkfConfig& config) {
  const double fallback =
      0.5 * (config.pos_meas_stddev_min_m + config.pos_meas_stddev_max_m);
  const double finite =
      (std::isfinite(stddev_m) && stddev_m > 0.0) ? stddev_m : fallback;
  return std::clamp(
      finite,
      config.pos_meas_stddev_min_m,
      config.pos_meas_stddev_max_m);
}

void GnssImuEkf::reset() {
  motion_hint_ = MotionHint{};
  x_.fill(0.0);
  for (auto& row : p_) {
    row.fill(0.0);
  }
  initialized_ = false;
  have_cached_yaw_ = false;
  last_imu_yaw_valid_ = false;
  last_gyro_valid_ = false;
  yaw_aligned_ = false;
  last_gps_used_ = false;
  last_gps_rejected_ = false;
  cached_yaw_measurement_rad_ = 0.0;
  last_imu_yaw_rad_ = 0.0;
  last_gyro_z_rad_s_ = 0.0;
  yaw_offset_rad_ = 0.0;
  last_predict_stamp_s_ = 0.0;
  last_imu_stamp_s_ = 0.0;
  last_imu_obs_stamp_s_ = 0.0;
  last_gps_position_stamp_s_ = 0.0;
  last_gps_used_stamp_s_ = 0.0;
  last_velocity_stamp_s_ = 0.0;
  last_speed_measurement_mps_ = 0.0;
  last_horizontal_stddev_m_ = 0.0;
  last_course_yaw_rad_ = 0.0;
  last_course_valid_ = false;
  stable_course_frames_ = 0;
  gps_used_count_ = 0;
  gps_rejected_count_ = 0;
  last_gps_innovation_m_ = 0.0;
  last_reject_reason_.clear();
}

void GnssImuEkf::initializeCovariance(
    double pos_stddev_m,
    double yaw_stddev_rad) {
  for (auto& row : p_) {
    row.fill(0.0);
  }
  const double pos_var = std::max(0.25, pos_stddev_m * pos_stddev_m);
  const double speed_var =
      std::max(0.04, config_.speed_meas_stddev_mps * config_.speed_meas_stddev_mps);
  const double yaw_var = std::max(1e-4, yaw_stddev_rad * yaw_stddev_rad);
  const double bias_var = std::max(1e-6, degToRad(3.0) * degToRad(3.0));
  p_[0][0] = pos_var;
  p_[1][1] = pos_var;
  p_[2][2] = speed_var;
  p_[3][3] = yaw_var;
  p_[4][4] = bias_var;
}

void GnssImuEkf::resetToPose(
    double stamp_s,
    double x_m,
    double y_m,
    double yaw_rad) {
  x_.fill(0.0);
  x_[0] = x_m;
  x_[1] = y_m;
  x_[2] = clampNonNegative(last_speed_measurement_mps_);
  x_[3] = wrapAngleRad(yaw_rad);
  x_[4] = 0.0;
  initializeCovariance(
      clampGpsStddev(last_horizontal_stddev_m_, config_),
      degToRad(6.0));
  initialized_ = true;
  have_cached_yaw_ = true;
  cached_yaw_measurement_rad_ = x_[3];
  last_predict_stamp_s_ = stamp_s;
  last_gps_position_stamp_s_ = stamp_s;
  last_gps_used_stamp_s_ = stamp_s;
  last_gps_used_ = true;
  last_gps_rejected_ = false;
  last_reject_reason_.clear();
}

void GnssImuEkf::setConfig(const GnssImuEkfConfig& config) {
  config_ = config;
}

void GnssImuEkf::setMotionHint(const MotionHint& hint) {
  motion_hint_ = hint;
}

void GnssImuEkf::predictStep(double dt_s, double gyro_z_rad_s) {
  if (!initialized_ || !(dt_s > 0.0)) {
    return;
  }

  const double yaw = x_[3];
  const double speed = x_[2];
  const double bias = x_[4];
  const double omega = gyro_z_rad_s - bias;
  const double yaw_mid = wrapAngleRad(yaw + 0.5 * omega * dt_s);

  x_[0] += speed * std::cos(yaw_mid) * dt_s;
  x_[1] += speed * std::sin(yaw_mid) * dt_s;
  x_[3] = wrapAngleRad(yaw + omega * dt_s);

  Mat5 f{};
  for (std::size_t row = 0; row < kStateSize; ++row) {
    for (std::size_t col = 0; col < kStateSize; ++col) {
      f[row][col] = row == col ? 1.0 : 0.0;
    }
  }
  f[0][2] = std::cos(yaw_mid) * dt_s;
  f[0][3] = -speed * std::sin(yaw_mid) * dt_s;
  f[1][2] = std::sin(yaw_mid) * dt_s;
  f[1][3] = speed * std::cos(yaw_mid) * dt_s;
  f[3][4] = -dt_s;

  Mat5 q{};
  const double pos_q =
      std::pow(config_.process_pos_noise * dt_s, 2.0);
  const double speed_q =
      std::pow(config_.process_speed_noise * dt_s, 2.0);
  const double yaw_q =
      std::pow(degToRad(config_.process_yaw_noise_deg) * dt_s, 2.0);
  const double bias_q =
      std::pow(config_.process_bias_noise * dt_s, 2.0);
  q[0][0] = pos_q;
  q[1][1] = pos_q;
  q[2][2] = speed_q;
  q[3][3] = yaw_q;
  q[4][4] = bias_q;

  Mat5 fp{};
  for (std::size_t row = 0; row < kStateSize; ++row) {
    for (std::size_t col = 0; col < kStateSize; ++col) {
      for (std::size_t k = 0; k < kStateSize; ++k) {
        fp[row][col] += f[row][k] * p_[k][col];
      }
    }
  }

  Mat5 updated_p{};
  for (std::size_t row = 0; row < kStateSize; ++row) {
    for (std::size_t col = 0; col < kStateSize; ++col) {
      for (std::size_t k = 0; k < kStateSize; ++k) {
        updated_p[row][col] += fp[row][k] * f[col][k];
      }
      updated_p[row][col] += q[row][col];
    }
  }
  p_ = updated_p;
}

void GnssImuEkf::predictToStamp(
    double stamp_s,
    bool gyro_valid,
    double gyro_z_rad_s) {
  if (!initialized_ || !std::isfinite(stamp_s)) {
    return;
  }
  if (last_predict_stamp_s_ <= 0.0) {
    last_predict_stamp_s_ = stamp_s;
    return;
  }
  if (stamp_s <= last_predict_stamp_s_) {
    return;
  }

  double effective_gyro = x_[4];
  if (gyro_valid && std::isfinite(gyro_z_rad_s)) {
    effective_gyro = gyro_z_rad_s;
  } else if (last_gyro_valid_) {
    effective_gyro = last_gyro_z_rad_s_;
  }

  double remaining = stamp_s - last_predict_stamp_s_;
  while (remaining > 1e-6) {
    const double step = std::min(remaining, config_.max_predict_dt_s);
    predictStep(step, effective_gyro);
    last_predict_stamp_s_ += step;
    remaining -= step;
  }
  finalizeStateAfterUpdate();
}

void GnssImuEkf::applyScalarMeasurement(
    std::size_t state_index,
    double measurement,
    double variance,
    bool angular_residual) {
  if (!initialized_ || state_index >= kStateSize ||
      !std::isfinite(measurement) || !(variance > 0.0)) {
    return;
  }

  const double residual = angular_residual
      ? wrapAngleRad(measurement - x_[state_index])
      : (measurement - x_[state_index]);
  const double s = p_[state_index][state_index] + variance;
  if (!(s > 1e-12) || !std::isfinite(s)) {
    return;
  }

  std::array<double, kStateSize> k{};
  for (std::size_t row = 0; row < kStateSize; ++row) {
    k[row] = p_[row][state_index] / s;
    x_[row] += k[row] * residual;
  }

  Mat5 a{};
  for (std::size_t row = 0; row < kStateSize; ++row) {
    for (std::size_t col = 0; col < kStateSize; ++col) {
      a[row][col] = row == col ? 1.0 : 0.0;
      if (col == state_index) {
        a[row][col] -= k[row];
      }
    }
  }

  Mat5 ap{};
  for (std::size_t row = 0; row < kStateSize; ++row) {
    for (std::size_t col = 0; col < kStateSize; ++col) {
      for (std::size_t idx = 0; idx < kStateSize; ++idx) {
        ap[row][col] += a[row][idx] * p_[idx][col];
      }
    }
  }

  Mat5 updated_p{};
  for (std::size_t row = 0; row < kStateSize; ++row) {
    for (std::size_t col = 0; col < kStateSize; ++col) {
      for (std::size_t idx = 0; idx < kStateSize; ++idx) {
        updated_p[row][col] += ap[row][idx] * a[col][idx];
      }
      updated_p[row][col] += k[row] * variance * k[col];
    }
  }
  p_ = updated_p;
  finalizeStateAfterUpdate();
}

void GnssImuEkf::applyPositionMeasurement(
    double x_m,
    double y_m,
    double variance_x,
    double variance_y) {
  if (!initialized_ ||
      !std::isfinite(x_m) ||
      !std::isfinite(y_m) ||
      !(variance_x > 0.0) ||
      !(variance_y > 0.0)) {
    return;
  }

  const double y0 = x_m - x_[0];
  const double y1 = y_m - x_[1];
  const double s00 = p_[0][0] + variance_x;
  const double s01 = p_[0][1];
  const double s10 = p_[1][0];
  const double s11 = p_[1][1] + variance_y;
  const double det = s00 * s11 - s01 * s10;
  if (!(det > 1e-12) || !std::isfinite(det)) {
    return;
  }

  std::array<std::array<double, 2>, kStateSize> k{};
  for (std::size_t row = 0; row < kStateSize; ++row) {
    k[row][0] = (p_[row][0] * s11 - p_[row][1] * s01) / det;
    k[row][1] = (p_[row][1] * s00 - p_[row][0] * s10) / det;
    x_[row] += k[row][0] * y0 + k[row][1] * y1;
  }

  Mat5 a{};
  for (std::size_t row = 0; row < kStateSize; ++row) {
    for (std::size_t col = 0; col < kStateSize; ++col) {
      a[row][col] = row == col ? 1.0 : 0.0;
      if (col == 0) {
        a[row][col] -= k[row][0];
      } else if (col == 1) {
        a[row][col] -= k[row][1];
      }
    }
  }

  Mat5 ap{};
  for (std::size_t row = 0; row < kStateSize; ++row) {
    for (std::size_t col = 0; col < kStateSize; ++col) {
      for (std::size_t idx = 0; idx < kStateSize; ++idx) {
        ap[row][col] += a[row][idx] * p_[idx][col];
      }
    }
  }

  Mat5 updated_p{};
  for (std::size_t row = 0; row < kStateSize; ++row) {
    for (std::size_t col = 0; col < kStateSize; ++col) {
      for (std::size_t idx = 0; idx < kStateSize; ++idx) {
        updated_p[row][col] += ap[row][idx] * a[col][idx];
      }
      updated_p[row][col] +=
          k[row][0] * variance_x * k[col][0] +
          k[row][1] * variance_y * k[col][1];
    }
  }
  p_ = updated_p;
  finalizeStateAfterUpdate();
}

void GnssImuEkf::finalizeStateAfterUpdate() {
  x_[2] = clampNonNegative(x_[2]);
  x_[3] = wrapAngleRad(x_[3]);
  for (std::size_t row = 0; row < kStateSize; ++row) {
    for (std::size_t col = row + 1; col < kStateSize; ++col) {
      const double sym = 0.5 * (p_[row][col] + p_[col][row]);
      p_[row][col] = sym;
      p_[col][row] = sym;
    }
    p_[row][row] = std::max(p_[row][row], 1e-9);
  }
}

double GnssImuEkf::currentSpeedReferenceMps() const {
  double speed_ref = clampNonNegative(x_[2]);
  speed_ref = std::max(speed_ref, clampNonNegative(last_speed_measurement_mps_));
  speed_ref = std::max(speed_ref, clampNonNegative(motion_hint_.command_speed_mps));
  return speed_ref;
}

bool GnssImuEkf::imuFresh(double now_s) const {
  if (!(last_imu_stamp_s_ > 0.0) || !std::isfinite(now_s)) {
    return false;
  }
  const double max_imu_gap_s = std::max(0.60, config_.max_predict_dt_s * 3.0);
  return (now_s - last_imu_stamp_s_) <= max_imu_gap_s;
}

void GnssImuEkf::updateImu(const ImuSample& imu) {
  if (!std::isfinite(imu.stamp_s) || imu.stamp_s <= 0.0) {
    return;
  }

  if (initialized_) {
    predictToStamp(imu.stamp_s, imu.gyro_valid, imu.gyro_z_rad_s);
  } else if (last_predict_stamp_s_ <= 0.0) {
    last_predict_stamp_s_ = imu.stamp_s;
  }

  last_imu_stamp_s_ = imu.stamp_s;
  if (imu.gyro_valid && std::isfinite(imu.gyro_z_rad_s)) {
    last_gyro_valid_ = true;
    last_gyro_z_rad_s_ = imu.gyro_z_rad_s;
  }

  if (imu.yaw_valid && std::isfinite(imu.yaw_rad)) {
    last_imu_yaw_valid_ = true;
    last_imu_yaw_rad_ = wrapAngleRad(imu.yaw_rad);
    have_cached_yaw_ = true;
    cached_yaw_measurement_rad_ =
        wrapAngleRad(last_imu_yaw_rad_ + yaw_offset_rad_);
    last_imu_obs_stamp_s_ = imu.stamp_s;
    if (initialized_) {
      const double variance =
          std::pow(degToRad(config_.imu_yaw_meas_stddev_deg), 2.0);
      applyScalarMeasurement(3, cached_yaw_measurement_rad_, variance, true);
    }
  }
}

void GnssImuEkf::updateGnssPosition(const GnssPositionSample& gps) {
  if (!gps.valid || !std::isfinite(gps.stamp_s) || gps.stamp_s <= 0.0 ||
      !std::isfinite(gps.x_m) || !std::isfinite(gps.y_m)) {
    return;
  }

  const double previous_gps_stamp_s = last_gps_position_stamp_s_;
  last_gps_position_stamp_s_ = gps.stamp_s;
  last_horizontal_stddev_m_ = gps.horizontal_stddev_m;
  last_gps_innovation_m_ = 0.0;

  if (!initialized_) {
    if (!have_cached_yaw_) {
      return;
    }
    resetToPose(gps.stamp_s, gps.x_m, gps.y_m, cached_yaw_measurement_rad_);
    gps_used_count_ = 1;
    last_gps_used_ = true;
    last_gps_rejected_ = false;
    last_reject_reason_.clear();
    return;
  }

  predictToStamp(gps.stamp_s, false, 0.0);

  const double innovation_x = gps.x_m - x_[0];
  const double innovation_y = gps.y_m - x_[1];
  const double innovation_distance_m =
      std::hypot(innovation_x, innovation_y);
  last_gps_innovation_m_ = innovation_distance_m;
  const double speed_ref_mps = currentSpeedReferenceMps();
  const double gps_dt_s =
      previous_gps_stamp_s > 0.0
          ? std::clamp(
                gps.stamp_s - previous_gps_stamp_s,
                0.05,
                config_.max_degraded_hold_s)
          : config_.gps_expected_period_s;
  const double supported_motion_m =
      speed_ref_mps * std::max(gps_dt_s, config_.gps_expected_period_s) + 0.60;
  if (innovation_distance_m > config_.gps_jump_gate_m &&
      innovation_distance_m > supported_motion_m) {
    last_gps_used_ = false;
    last_gps_rejected_ = true;
    ++gps_rejected_count_;
    last_reject_reason_ = "gps_jump";
    return;
  }

  const bool vehicle_static =
      !motion_hint_.auto_task_running &&
      !motion_hint_.avoidance_active &&
      clampNonNegative(motion_hint_.command_speed_mps) <=
          config_.static_speed_gate_mps &&
      speed_ref_mps <= config_.static_speed_gate_mps;
  if (vehicle_static &&
      innovation_distance_m <= config_.static_drift_gate_m) {
    last_gps_used_ = false;
    last_gps_rejected_ = true;
    ++gps_rejected_count_;
    last_reject_reason_ = "static_drift_hold";
    return;
  }

  const double sigma = clampGpsStddev(gps.horizontal_stddev_m, config_);
  const double variance = sigma * sigma;
  applyPositionMeasurement(gps.x_m, gps.y_m, variance, variance);

  last_gps_used_stamp_s_ = gps.stamp_s;
  last_gps_used_ = true;
  last_gps_rejected_ = false;
  ++gps_used_count_;
  last_reject_reason_.clear();
}

void GnssImuEkf::updateGnssVelocity(
    const GnssVelocitySample& vel,
    bool allow_course_alignment,
    bool vehicle_commanded_moving) {
  if (!std::isfinite(vel.stamp_s) || vel.stamp_s <= 0.0) {
    return;
  }

  last_velocity_stamp_s_ = vel.stamp_s;
  if (vel.speed_valid && std::isfinite(vel.speed_mps)) {
    last_speed_measurement_mps_ = clampNonNegative(vel.speed_mps);
    if (initialized_) {
      predictToStamp(vel.stamp_s, false, 0.0);
      applyScalarMeasurement(
          2,
          last_speed_measurement_mps_,
          config_.speed_meas_stddev_mps * config_.speed_meas_stddev_mps,
          false);
    }
  }

  const bool allow_alignment =
      allow_course_alignment &&
      vehicle_commanded_moving &&
      vel.course_valid &&
      vel.speed_valid &&
      std::isfinite(vel.course_yaw_rad) &&
      std::isfinite(vel.speed_mps) &&
      vel.speed_mps > config_.yaw_align_min_speed_mps &&
      last_imu_yaw_valid_ &&
      !motion_hint_.emergency_active;
  if (!allow_alignment) {
    stable_course_frames_ = 0;
    last_course_valid_ = false;
    return;
  }

  const double course_yaw_rad = wrapAngleRad(vel.course_yaw_rad);
  const double stable_threshold_rad =
      std::max(degToRad(config_.gps_course_stddev_deg * 1.5), degToRad(6.0));
  if (!last_course_valid_ ||
      std::abs(wrapAngleRad(course_yaw_rad - last_course_yaw_rad_)) >
          stable_threshold_rad) {
    stable_course_frames_ = 1;
  } else {
    ++stable_course_frames_;
  }
  last_course_valid_ = true;
  last_course_yaw_rad_ = course_yaw_rad;

  if (stable_course_frames_ < config_.yaw_align_min_stable_frames) {
    return;
  }

  const double target_offset_rad =
      wrapAngleRad(course_yaw_rad - last_imu_yaw_rad_);
  const double offset_delta_rad =
      wrapAngleRad(target_offset_rad - yaw_offset_rad_);
  yaw_offset_rad_ = wrapAngleRad(
      yaw_offset_rad_ + config_.yaw_align_alpha * offset_delta_rad);
  yaw_aligned_ = true;
  if (have_cached_yaw_) {
    cached_yaw_measurement_rad_ =
        wrapAngleRad(last_imu_yaw_rad_ + yaw_offset_rad_);
  }
}

GnssImuEkfState GnssImuEkf::state(double now_s) const {
  GnssImuEkfState result;
  result.stamp_s = std::isfinite(now_s) ? now_s : last_predict_stamp_s_;
  result.x_m = x_[0];
  result.y_m = x_[1];
  result.speed_mps = x_[2];
  result.yaw_rad = x_[3];
  result.heading_deg = headingDegFromYawRad(x_[3]);
  result.gyro_bias_rad_s = x_[4];
  result.initialized = initialized_;
  result.yaw_aligned = yaw_aligned_;
  result.yaw_offset_rad = yaw_offset_rad_;
  result.last_gps_used = last_gps_used_;
  result.last_gps_rejected = last_gps_rejected_;
  result.gps_used_count = gps_used_count_;
  result.gps_rejected_count = gps_rejected_count_;
  result.last_gps_innovation_m = last_gps_innovation_m_;
  result.last_reject_reason = last_reject_reason_;
  result.gps_age_ms =
      last_gps_position_stamp_s_ > 0.0 && std::isfinite(now_s)
          ? std::max(0.0, (now_s - last_gps_position_stamp_s_) * 1000.0)
          : -1.0;
  result.imu_age_ms =
      last_imu_stamp_s_ > 0.0 && std::isfinite(now_s)
          ? std::max(0.0, (now_s - last_imu_stamp_s_) * 1000.0)
          : -1.0;
  result.holdover_ms =
      last_gps_used_stamp_s_ > 0.0 && std::isfinite(now_s)
          ? std::max(0.0, (now_s - last_gps_used_stamp_s_) * 1000.0)
          : -1.0;

  if (!initialized_) {
    result.mode = FusionMode::kUninitialized;
    result.valid = false;
    result.quality = 0.0;
    return result;
  }

  if (!imuFresh(now_s)) {
    result.mode = FusionMode::kInvalid;
    result.valid = false;
    result.quality = 0.0;
    return result;
  }

  const double gps_age_s =
      result.gps_age_ms >= 0.0 ? result.gps_age_ms / 1000.0
                               : std::numeric_limits<double>::infinity();
  if (gps_age_s > config_.max_degraded_hold_s) {
    result.mode = FusionMode::kInvalid;
    result.valid = false;
  } else if (gps_age_s > config_.max_gps_hold_s) {
    result.mode = FusionMode::kDegraded;
    result.valid = true;
  } else if (gps_age_s > config_.gps_fresh_s) {
    result.mode = FusionMode::kGpsHold;
    result.valid = true;
  } else if (!yaw_aligned_) {
    result.mode = FusionMode::kAligning;
    result.valid = true;
  } else {
    result.mode = FusionMode::kFused;
    result.valid = true;
  }

  double quality = 0.0;
  switch (result.mode) {
    case FusionMode::kUninitialized:
      quality = 0.0;
      break;
    case FusionMode::kAligning:
      quality = 0.62;
      break;
    case FusionMode::kFused:
      quality = 0.92;
      break;
    case FusionMode::kGpsHold:
      quality = 0.72;
      break;
    case FusionMode::kDegraded:
      quality = 0.45;
      break;
    case FusionMode::kInvalid:
      quality = 0.0;
      break;
  }
  if (std::isfinite(last_horizontal_stddev_m_) && last_horizontal_stddev_m_ > 0.0) {
    const double sigma = clampGpsStddev(last_horizontal_stddev_m_, config_);
    const double normalized =
        1.0 -
        (sigma - config_.pos_meas_stddev_min_m) /
            std::max(0.1,
                     config_.pos_meas_stddev_max_m -
                         config_.pos_meas_stddev_min_m);
    quality *= std::clamp(0.35 + normalized * 0.65, 0.35, 1.0);
  }
  if (!yaw_aligned_ && result.mode == FusionMode::kFused) {
    quality *= 0.85;
  }
  result.quality = std::clamp(quality, 0.0, 1.0);
  return result;
}

}  // namespace navigation
