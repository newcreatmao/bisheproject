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
constexpr int kWallLikeClusterMinPoints = 6;
constexpr double kWallLikeClusterMinSpanDeg = 10.0;
constexpr double kWallLikeClusterEdgeAngleDeg = 18.0;
constexpr double kWallLikeClusterMaxAverageDeltaMeters = 0.10;
constexpr double kWallLikePrimaryTargetPenalty = 2.55;
constexpr char kFrontTargetRoleDiscretePrimary[] = "discrete_primary";
constexpr char kFrontTargetRoleWallConstraint[] = "wall_constraint";
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
    double center_angle_deg = 0.0;
    int support_points = 0;
    double span_deg = 0.0;
    double average_adjacent_range_delta_m = 0.0;
    bool wall_like = false;
    double score = 0.0;
};

using SectorState = AutoAvoidInputBuilder::SectorState;
using FrontTargetSelection = AutoAvoidInputBuilder::FrontTargetSelection;

double meanAdjacentRangeDeltaM(const std::vector<FrontSectorPoint>& cluster) {
    if (cluster.size() < 2) {
        return 0.0;
    }

    double delta_sum_m = 0.0;
    for (std::size_t i = 1; i < cluster.size(); ++i) {
        delta_sum_m += std::abs(cluster[i].range - cluster[i - 1].range);
    }
    return delta_sum_m / static_cast<double>(cluster.size() - 1);
}

bool isWallLikeFrontCluster(
    const FrontClusterCandidate& candidate,
    const SectorState& negative_front,
    const SectorState& positive_front) {
    if (!candidate.valid) {
        return false;
    }

    const bool broad_cluster =
        candidate.support_points >= kWallLikeClusterMinPoints &&
        candidate.span_deg >= kWallLikeClusterMinSpanDeg;
    if (!broad_cluster) {
        return false;
    }

    const bool smooth_range =
        candidate.average_adjacent_range_delta_m <= kWallLikeClusterMaxAverageDeltaMeters;
    if (!smooth_range) {
        return false;
    }

    const bool near_sector_edge =
        std::abs(candidate.center_angle_deg) >= kWallLikeClusterEdgeAngleDeg;
    const SectorState& likely_side_sector =
        candidate.center_angle_deg < 0.0 ? negative_front : positive_front;
    const bool continuous_with_side =
        likely_side_sector.valid &&
        std::isfinite(likely_side_sector.nearest_m) &&
        std::abs(likely_side_sector.nearest_m - candidate.median_range_m) <= 0.30;
    const bool very_broad_cluster =
        candidate.support_points >= 9 || candidate.span_deg >= 16.0;

    return near_sector_edge || continuous_with_side || very_broad_cluster;
}

double frontClusterSelectionScore(const FrontClusterCandidate& candidate) {
    if (!candidate.valid) {
        return -std::numeric_limits<double>::infinity();
    }

    const double clamped_median_m =
        std::clamp(candidate.median_range_m, 0.35, 4.0);
    const double distance_score = 4.0 / clamped_median_m;
    const double center_bonus =
        1.6 * (1.0 - std::min(std::abs(candidate.nearest_angle_deg), 58.0) / 58.0);
    const double compact_bonus =
        0.8 * (1.0 - std::clamp(candidate.span_deg / 24.0, 0.0, 1.0));
    const double support_bonus =
        0.07 * std::min(candidate.support_points, 10);
    const double wall_penalty =
        candidate.wall_like ? kWallLikePrimaryTargetPenalty : 0.0;
    return distance_score + center_bonus + compact_bonus + support_bonus - wall_penalty;
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

    std::vector<FrontSectorPoint> front_sector_points;
    front_sector_points.reserve(msg->ranges.size());
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
            updateSector(frame.negative_front_sector, range, normalized_angle_deg);
        } else if (
            normalized_angle_deg >= kAvoidanceFrontSectorMinDeg &&
            normalized_angle_deg <= kAvoidanceFrontSectorMaxDeg) {
            front_sector_points.push_back({range, normalized_angle_deg});
            if (updateSector(frame.front_sector, range, normalized_angle_deg)) {
                frame.front_nearest_zone =
                    frontNearestZoneFromAngle(normalized_angle_deg, previous_front_nearest_zone);
            }
        } else if (
            normalized_angle_deg >= kAvoidancePositiveSectorMinDeg &&
            normalized_angle_deg <= kAvoidancePositiveSectorMaxDeg) {
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

    frame.auto_avoid_front_sector.valid_points = frame.front_sector.valid_points;
    if (!front_sector_points.empty()) {
        std::sort(
            front_sector_points.begin(),
            front_sector_points.end(),
            [](const FrontSectorPoint& lhs, const FrontSectorPoint& rhs) {
                return lhs.angle_deg < rhs.angle_deg;
            });

        FrontClusterCandidate best_cluster;
        FrontClusterCandidate best_discrete_cluster;
        FrontClusterCandidate best_wall_cluster;
        bool wall_like_cluster_suppressed = false;
        std::vector<FrontSectorPoint> current_cluster;
        int next_cluster_id = 0;
        const auto consider_cluster =
            [&best_cluster,
                &best_discrete_cluster,
                &best_wall_cluster,
                &wall_like_cluster_suppressed,
                &next_cluster_id,
                &frame](const std::vector<FrontSectorPoint>& cluster) {
                if (static_cast<int>(cluster.size()) < kAutoAvoidFrontClusterMinPoints) {
                    return;
                }

                FrontClusterCandidate candidate;
                candidate.valid = true;
                candidate.cluster_id = next_cluster_id++;
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
                const double median_range_m = sorted_ranges[sorted_ranges.size() / 2];

                candidate.median_range_m = median_range_m;
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
                candidate.center_angle_deg =
                    0.5 * (cluster.front().angle_deg + cluster.back().angle_deg);
                candidate.span_deg =
                    std::max(0.0, cluster.back().angle_deg - cluster.front().angle_deg);
                candidate.average_adjacent_range_delta_m = meanAdjacentRangeDeltaM(cluster);
                candidate.wall_like = isWallLikeFrontCluster(
                    candidate,
                    frame.negative_front_sector,
                    frame.positive_front_sector);
                candidate.score = frontClusterSelectionScore(candidate);

                const auto better_than =
                    [](const FrontClusterCandidate& lhs,
                        const FrontClusterCandidate& rhs) {
                        return !rhs.valid ||
                            lhs.score > rhs.score + 1e-6 ||
                            (std::abs(lhs.score - rhs.score) < 1e-6 &&
                                lhs.median_range_m < rhs.median_range_m) ||
                            (std::abs(lhs.score - rhs.score) < 1e-6 &&
                                std::abs(lhs.median_range_m - rhs.median_range_m) < 1e-6 &&
                                lhs.nearest_m < rhs.nearest_m);
                    };

                if (candidate.wall_like) {
                    if (better_than(candidate, best_wall_cluster)) {
                        best_wall_cluster = candidate;
                    }
                } else if (better_than(candidate, best_discrete_cluster)) {
                    best_discrete_cluster = candidate;
                }

                if (candidate.wall_like &&
                    best_cluster.valid &&
                    !best_cluster.wall_like &&
                    candidate.median_range_m + 1e-6 < best_cluster.median_range_m) {
                    wall_like_cluster_suppressed = true;
                }

                if (better_than(candidate, best_cluster)) {
                    if (best_cluster.valid &&
                        best_cluster.wall_like &&
                        !candidate.wall_like &&
                        best_cluster.median_range_m <= candidate.median_range_m + 0.20) {
                        wall_like_cluster_suppressed = true;
                    }
                    best_cluster = candidate;
                }
            };

        for (const auto& point : front_sector_points) {
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

        FrontClusterCandidate selected_cluster = best_cluster;
        std::string selected_target_role;
        std::string selection_reason;
        std::string raw_zone_source;
        bool selected_is_discrete_primary = false;
        bool selected_is_wall_constraint = false;
        bool raw_zone_from_discrete_target = false;
        bool wall_like_suppressed_from_zone = false;
        if (best_discrete_cluster.valid) {
            selected_cluster = best_discrete_cluster;
            selected_target_role = kFrontTargetRoleDiscretePrimary;
            selected_is_discrete_primary = true;
            raw_zone_from_discrete_target = true;
            wall_like_suppressed_from_zone = best_wall_cluster.valid;
            wall_like_cluster_suppressed =
                wall_like_cluster_suppressed || best_wall_cluster.valid;
            selection_reason =
                best_wall_cluster.valid ?
                    "discrete_cluster_over_wall" :
                    "scored_discrete_cluster";
            raw_zone_source = "discrete_primary_cluster";
        } else if (best_cluster.valid) {
            selected_cluster = best_cluster;
            if (selected_cluster.wall_like) {
                selected_target_role = kFrontTargetRoleWallConstraint;
                selected_is_wall_constraint = true;
                selection_reason = "wall_constraint_cluster";
                raw_zone_source = "wall_constraint_cluster";
            } else {
                selected_target_role = kFrontTargetRoleDiscretePrimary;
                selected_is_discrete_primary = true;
                raw_zone_from_discrete_target = true;
                selection_reason = "scored_discrete_cluster";
                raw_zone_source = "discrete_primary_cluster";
            }
        }

        if (selected_cluster.valid) {
            frame.auto_avoid_front_sector.valid = true;
            frame.auto_avoid_front_sector.nearest_m = selected_cluster.nearest_m;
            frame.auto_avoid_front_sector.nearest_angle_deg =
                selected_cluster.nearest_angle_deg;
            frame.auto_avoid_front_sector.support_points = selected_cluster.support_points;
            frame.front_target_selection.valid = true;
            frame.front_target_selection.selected_front_cluster_id =
                selected_cluster.cluster_id;
            frame.front_target_selection.selected_front_cluster_score =
                selected_cluster.score;
            frame.front_target_selection.selected_front_cluster_wall_like =
                selected_cluster.wall_like;
            frame.front_target_selection.selected_front_cluster_is_discrete_primary =
                selected_is_discrete_primary;
            frame.front_target_selection.selected_front_cluster_is_wall_like =
                selected_is_wall_constraint;
            frame.front_target_selection.selected_front_cluster_points =
                selected_cluster.support_points;
            frame.front_target_selection.selected_front_cluster_span_deg =
                selected_cluster.span_deg;
            frame.front_target_selection.selected_front_cluster_median_range =
                selected_cluster.median_range_m;
            frame.front_target_selection.selected_front_cluster_nearest_range =
                selected_cluster.nearest_m;
            frame.front_target_selection.wall_like_cluster_suppressed =
                wall_like_cluster_suppressed;
            frame.front_target_selection.raw_zone_from_discrete_target =
                raw_zone_from_discrete_target;
            frame.front_target_selection.wall_like_suppressed_from_zone =
                wall_like_suppressed_from_zone;
            frame.front_target_selection.front_target_role =
                selected_target_role;
            frame.front_target_selection.front_target_selection_reason =
                selection_reason;
            frame.front_target_selection.raw_zone_source = raw_zone_source;
        } else {
            frame.auto_avoid_front_sector = frame.front_sector;
            frame.auto_avoid_front_sector.support_points =
                frame.front_sector.valid ? 1 : 0;
            frame.auto_avoid_front_sector.valid_points = frame.front_sector.valid_points;
            frame.front_target_selection = FrontTargetSelection{};
            frame.front_target_selection.front_target_role = kFrontTargetRoleFallbackFront;
            frame.front_target_selection.front_target_selection_reason =
                "front_sector_fallback";
            frame.front_target_selection.raw_zone_source = "front_sector_fallback";
        }
    }
    if (front_sector_points.empty()) {
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
