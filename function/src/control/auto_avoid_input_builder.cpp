#include "control/auto_avoid_input_builder.hpp"

#include "control/Judgment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace {

constexpr double kLidarMinUsefulRangeMeters = 0.25;
constexpr double kLidarBlockedAngleMinDeg = 90.0;
constexpr double kLidarBlockedAngleMaxDeg = 270.0;
constexpr double kLidarObstacleDetectDistanceMeters = 2.0;
constexpr double kAvoidanceNegativeSectorMinDeg = -90.0;
constexpr double kAvoidanceNegativeSectorMaxDeg = -62.0;
constexpr double kAvoidanceFrontSectorMinDeg = -58.0;
constexpr double kAvoidanceFrontSectorMaxDeg = 58.0;
constexpr double kAvoidancePositiveSectorMinDeg = 62.0;
constexpr double kAvoidancePositiveSectorMaxDeg = 90.0;
constexpr int kAutoAvoidFrontClusterMinPoints = 3;
constexpr double kAutoAvoidFrontClusterMaxGapDeg = 2.5;
constexpr double kAutoAvoidFrontClusterAdjacentRangeDeltaMeters = 0.25;
constexpr double kAutoAvoidFrontClusterAngleAverageBandMeters = 0.05;
constexpr char kFrontTargetRoleDiscretePrimary[] = "discrete_primary";
constexpr char kFrontTargetRoleFallbackFront[] = "fallback_front";

double normalizeAngleDeg(double angle_deg) {
    if (!std::isfinite(angle_deg)) {
        return 0.0;
    }
    double normalized = std::fmod(angle_deg, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }
    if (normalized > 180.0) {
        normalized -= 360.0;
    }
    return normalized;
}

bool angleInWrappedRangeDeg(double angle_deg, double min_deg, double max_deg) {
    const double normalized_angle = std::fmod(angle_deg, 360.0) < 0.0 ?
        std::fmod(angle_deg, 360.0) + 360.0 :
        std::fmod(angle_deg, 360.0);
    if (min_deg <= max_deg) {
        return normalized_angle >= min_deg && normalized_angle <= max_deg;
    }
    return normalized_angle >= min_deg || normalized_angle <= max_deg;
}

struct FrontSectorPoint {
    double range = 0.0;
    double angle_deg = 0.0;
};

struct FrontClusterCandidate {
    bool valid = false;
    int cluster_id = -1;
    double median_range_m = 0.0;
    double nearest_m = 0.0;
    double nearest_angle_deg = 0.0;
    int support_points = 0;
};

using SectorState = AutoAvoidInputBuilder::SectorState;

SectorState clusteredBoundarySector(
    const std::vector<FrontSectorPoint>& points,
    int raw_valid_points) {
    SectorState sector;
    sector.valid_points = raw_valid_points;
    if (points.empty()) {
        return sector;
    }

    std::vector<FrontSectorPoint> sorted_points = points;
    std::sort(
        sorted_points.begin(),
        sorted_points.end(),
        [](const FrontSectorPoint& lhs, const FrontSectorPoint& rhs) {
            return lhs.angle_deg < rhs.angle_deg;
        });

    FrontClusterCandidate best_cluster;
    std::vector<FrontSectorPoint> current_cluster;
    const auto consider_cluster =
        [&best_cluster](const std::vector<FrontSectorPoint>& cluster) {
            if (static_cast<int>(cluster.size()) < kAutoAvoidFrontClusterMinPoints) {
                return;
            }

            FrontClusterCandidate candidate;
            candidate.valid = true;
            candidate.support_points = static_cast<int>(cluster.size());

            std::vector<double> sorted_ranges;
            sorted_ranges.reserve(cluster.size());
            double nearest_range_m = std::numeric_limits<double>::infinity();
            double nearest_angle_deg = 0.0;
            for (const auto& point : cluster) {
                sorted_ranges.push_back(point.range);
                if (point.range < nearest_range_m) {
                    nearest_range_m = point.range;
                    nearest_angle_deg = point.angle_deg;
                }
            }

            std::sort(sorted_ranges.begin(), sorted_ranges.end());
            candidate.median_range_m = sorted_ranges[sorted_ranges.size() / 2];
            candidate.nearest_m = nearest_range_m;
            double angle_sum_deg = 0.0;
            int angle_count = 0;
            for (const auto& point : cluster) {
                if (point.range <=
                    nearest_range_m + kAutoAvoidFrontClusterAngleAverageBandMeters) {
                    angle_sum_deg += point.angle_deg;
                    ++angle_count;
                }
            }
            if (angle_count > 0) {
                nearest_angle_deg = angle_sum_deg / static_cast<double>(angle_count);
            }
            candidate.nearest_angle_deg = nearest_angle_deg;

            if (!best_cluster.valid ||
                candidate.median_range_m < best_cluster.median_range_m - 1e-6 ||
                (std::abs(candidate.median_range_m - best_cluster.median_range_m) < 1e-6 &&
                    candidate.nearest_m < best_cluster.nearest_m)) {
                best_cluster = candidate;
            }
        };

    for (const auto& point : sorted_points) {
        if (!current_cluster.empty()) {
            const auto& previous_point = current_cluster.back();
            const bool same_cluster =
                std::abs(point.angle_deg - previous_point.angle_deg) <=
                    kAutoAvoidFrontClusterMaxGapDeg &&
                std::abs(point.range - previous_point.range) <=
                    kAutoAvoidFrontClusterAdjacentRangeDeltaMeters;
            if (!same_cluster) {
                consider_cluster(current_cluster);
                current_cluster.clear();
            }
        }
        current_cluster.push_back(point);
    }
    consider_cluster(current_cluster);

    if (best_cluster.valid) {
        sector.valid = true;
        sector.nearest_m = best_cluster.nearest_m;
        sector.nearest_angle_deg = best_cluster.nearest_angle_deg;
        sector.support_points = best_cluster.support_points;
    }
    return sector;
}

}  // namespace

bool AutoAvoidInputBuilder::updateSector(
    SectorState& sector,
    double range,
    double angle_deg) {
    ++sector.valid_points;
    if (!sector.valid || range < sector.nearest_m) {
        sector.valid = true;
        sector.nearest_m = range;
        sector.nearest_angle_deg = angle_deg;
        return true;
    }
    return false;
}

std::string AutoAvoidInputBuilder::frontNearestZoneFromAngle(
    double angle_deg,
    const std::string& fallback) {
    switch (Judgment::frontObstacleZoneFromAngle(angle_deg)) {
        case Judgment::FrontObstacleZone::Left:
            return "左";
        case Judgment::FrontObstacleZone::Center:
            return "中";
        case Judgment::FrontObstacleZone::Right:
            return "右";
        case Judgment::FrontObstacleZone::Unknown:
        default:
            return fallback;
    }
}

AutoAvoidInputBuilder::LidarInputFrame AutoAvoidInputBuilder::buildLidarInputFrame(
    const sensor_msgs::msg::LaserScan::SharedPtr& msg,
    const std::string& previous_front_nearest_zone) const {
    LidarInputFrame frame;
    if (!msg) {
        return frame;
    }

    std::vector<FrontSectorPoint> negative_front_points;
    std::vector<FrontSectorPoint> positive_front_points;
    negative_front_points.reserve(msg->ranges.size() / 4);
    positive_front_points.reserve(msg->ranges.size() / 4);
    double nearest = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
        const double range = static_cast<double>(msg->ranges[i]);
        if (!std::isfinite(range) ||
            range < std::max(static_cast<double>(msg->range_min), kLidarMinUsefulRangeMeters) ||
            range > static_cast<double>(msg->range_max)) {
            continue;
        }

        const double angle_rad =
            static_cast<double>(msg->angle_min) +
            static_cast<double>(i) * static_cast<double>(msg->angle_increment);
        const double raw_angle_deg = angle_rad * 180.0 / std::acos(-1.0);
        if (angleInWrappedRangeDeg(
                raw_angle_deg,
                kLidarBlockedAngleMinDeg,
                kLidarBlockedAngleMaxDeg)) {
            continue;
        }

        const double normalized_angle_deg = normalizeAngleDeg(raw_angle_deg);
        ++frame.valid_points;
        if (range < nearest) {
            nearest = range;
            frame.nearest_angle_deg = normalized_angle_deg;
        }

        if (normalized_angle_deg >= kAvoidanceNegativeSectorMinDeg &&
            normalized_angle_deg <= kAvoidanceNegativeSectorMaxDeg) {
            negative_front_points.push_back({range, normalized_angle_deg});
            updateSector(frame.negative_front_sector, range, normalized_angle_deg);
        } else if (
            normalized_angle_deg >= kAvoidanceFrontSectorMinDeg &&
            normalized_angle_deg <= kAvoidanceFrontSectorMaxDeg) {
            if (updateSector(frame.front_sector, range, normalized_angle_deg)) {
                frame.front_nearest_zone =
                    frontNearestZoneFromAngle(normalized_angle_deg, previous_front_nearest_zone);
            }
        } else if (
            normalized_angle_deg >= kAvoidancePositiveSectorMinDeg &&
            normalized_angle_deg <= kAvoidancePositiveSectorMaxDeg) {
            positive_front_points.push_back({range, normalized_angle_deg});
            updateSector(frame.positive_front_sector, range, normalized_angle_deg);
        } else if (
            (normalized_angle_deg > kAvoidanceNegativeSectorMaxDeg &&
                normalized_angle_deg < kAvoidanceFrontSectorMinDeg) ||
            (normalized_angle_deg > kAvoidanceFrontSectorMaxDeg &&
                normalized_angle_deg < kAvoidancePositiveSectorMinDeg)) {
            updateSector(frame.avoidance_buffer_sector, range, normalized_angle_deg);
        }
    }

    frame.valid = frame.valid_points > 0;
    frame.obstacle_detected =
        frame.valid_points > 0 && nearest <= kLidarObstacleDetectDistanceMeters;
    if (frame.valid) {
        frame.nearest_m = nearest;
    }

    // Side boundary sectors use the same adjacent-point cluster gate as the
    // front sector, so isolated edge noise stays observable but not actionable.
    frame.negative_front_sector = clusteredBoundarySector(
        negative_front_points,
        frame.negative_front_sector.valid_points);
    frame.positive_front_sector = clusteredBoundarySector(
        positive_front_points,
        frame.positive_front_sector.valid_points);

    frame.auto_avoid_front_sector = frame.front_sector;
    frame.auto_avoid_front_sector.support_points =
        frame.front_sector.valid ? 1 : 0;
    frame.auto_avoid_front_sector.valid_points = frame.front_sector.valid_points;
    frame.front_target_selection = FrontTargetSelection{};
    if (frame.auto_avoid_front_sector.valid) {
        frame.front_target_selection.valid = true;
        frame.front_target_selection.selected_front_cluster_points = 1;
        frame.front_target_selection.selected_front_cluster_median_range =
            frame.auto_avoid_front_sector.nearest_m;
        frame.front_target_selection.selected_front_cluster_nearest_range =
            frame.auto_avoid_front_sector.nearest_m;
        frame.front_target_selection.selected_front_cluster_is_discrete_primary = true;
        frame.front_target_selection.raw_zone_from_discrete_target = true;
        frame.front_target_selection.front_target_role = kFrontTargetRoleDiscretePrimary;
        frame.front_target_selection.front_target_selection_reason = "nearest_front_only";
        frame.front_target_selection.raw_zone_source = "nearest_front_only";
    } else {
        frame.front_target_selection.front_target_role = kFrontTargetRoleFallbackFront;
        frame.front_target_selection.front_target_selection_reason =
            "front_sector_empty";
        frame.front_target_selection.raw_zone_source = "front_sector_empty";
    }

    if (frame.auto_avoid_front_sector.valid) {
        frame.front_nearest_zone = frontNearestZoneFromAngle(
            frame.auto_avoid_front_sector.nearest_angle_deg,
            previous_front_nearest_zone);
    }
    return frame;
}

AutoAvoidController::SensorSnapshot AutoAvoidInputBuilder::buildSnapshot(
    const LidarInputFrame& lidar_frame,
    bool lidar_fresh,
    bool imu_fresh,
    double yaw_deg,
    std::int64_t timestamp_steady_ms) const {
    const auto sector_snapshot = [lidar_fresh](const SectorState& sector) {
        AutoAvoidController::SectorSample sample = sector;
        sample.valid = lidar_fresh && sector.valid;
        return sample;
    };

    AutoAvoidController::SensorSnapshot snapshot;
    snapshot.timestamp_steady_ms = timestamp_steady_ms;
    snapshot.lidar_valid = lidar_fresh;
    snapshot.negative_front = sector_snapshot(lidar_frame.negative_front_sector);
    snapshot.front = sector_snapshot(lidar_frame.auto_avoid_front_sector);
    snapshot.positive_front = sector_snapshot(lidar_frame.positive_front_sector);
    snapshot.sector_buffer = sector_snapshot(lidar_frame.avoidance_buffer_sector);
    snapshot.imu_valid = imu_fresh;
    snapshot.yaw_deg = yaw_deg;
    snapshot.snapshot_fresh = lidar_fresh || imu_fresh;
    snapshot.front_nearest_valid = snapshot.front.valid;
    snapshot.front_nearest_m = snapshot.front.valid ? snapshot.front.nearest_m : 0.0;
    snapshot.front_angle_deg =
        snapshot.front.valid ? snapshot.front.nearest_angle_deg : 0.0;
    snapshot.front_support_points =
        snapshot.front.valid ? snapshot.front.support_points : 0;
    snapshot.front_target_selection =
        lidar_fresh ? lidar_frame.front_target_selection : FrontTargetSelection{};
    return snapshot;
}
