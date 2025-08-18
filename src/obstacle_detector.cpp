#include "lattice_planner_pkg/obstacle_detector.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <numeric>

namespace lattice_planner_pkg {
namespace advanced {

ObstacleDetector::ObstacleDetector(const ObstacleDetectionConfig& config) 
    : config_(config), next_track_id_(1) {}

void ObstacleDetector::detectObstaclesFromScan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan,
    double ego_x, double ego_y, double ego_yaw,
    const rclcpp::Clock::SharedPtr& clock) {
    
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    // 1단계: 라이다 데이터 전처리
    std::vector<double> filtered_ranges = preprocessLidarData(scan);
    
    // 2단계: 유효한 포인트들을 수집하고 글로벌 좌표로 변환
    std::vector<std::pair<double, double>> valid_points;
    
    for (size_t i = 0; i < filtered_ranges.size(); ++i) {
        double range = filtered_ranges[i];
        
        if (range < config_.min_range_threshold || 
            range > config_.max_range_threshold ||
            range > config_.max_detection_range ||
            std::isnan(range) || std::isinf(range)) {
            continue;
        }
        
        double angle = scan->angle_min + i * scan->angle_increment;
        
        // 시야각 필터링
        if (std::abs(angle) > config_.lateral_range) {
            continue;
        }
        
        // 차량 진행 방향 기준 필터링
        double forward_distance = range * std::cos(angle);
        double lateral_distance = std::abs(range * std::sin(angle));
        
        if (forward_distance < 0.0 || 
            forward_distance > config_.forward_distance_max || 
            lateral_distance > config_.lateral_distance_max) {
            continue;
        }
        
        // 글로벌 좌표로 변환
        double global_x = ego_x + range * std::cos(ego_yaw + angle);
        double global_y = ego_y + range * std::sin(ego_yaw + angle);
        
        valid_points.emplace_back(global_x, global_y);
    }
    
    // 3단계: 클러스터링을 통한 장애물 감지
    std::vector<AdvancedObstacle> new_obstacles = clusterLidarPoints(valid_points, clock);
    
    // 4단계: 추적 시스템 업데이트
    updateTracking(new_obstacles, clock);
    
    // 5단계: 속도 추정
    estimateVelocities();
    
    // 현재 감지된 장애물 업데이트
    detected_obstacles_.clear();
    for (const auto& track : tracked_obstacles_) {
        if (track.is_confirmed && track.lost_frames < config_.max_lost_frames) {
            detected_obstacles_.push_back(track.current_state);
        }
    }
    
    last_scan_time_ = clock->now();
}

void ObstacleDetector::updateOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(grid_mutex_);
    latest_grid_ = msg;
}

bool ObstacleDetector::hasOccupancyGrid() const {
    std::lock_guard<std::mutex> lk(grid_mutex_);
    return latest_grid_ != nullptr;
}

bool ObstacleDetector::hasLidarObstacles() const {
    return !detected_obstacles_.empty();
}

bool ObstacleDetector::hasOccupancyObstacles(double ego_x, double ego_y) const {
    std::lock_guard<std::mutex> lk(grid_mutex_);
    if (!latest_grid_) return false;
    
    const auto & grid = *latest_grid_;
    if (grid.info.resolution <= 0 || grid.info.width == 0 || grid.info.height == 0) return false;
    
    // 차량 주변 일정 범위에서 장애물 검사
    double search_radius = 5.0; // 8m 범위
    double res = grid.info.resolution;
    int w = (int)grid.info.width;
    int h = (int)grid.info.height;
    double origin_x = grid.info.origin.position.x;
    double origin_y = grid.info.origin.position.y;
    
    // 차량 위치를 그리드 좌표로 변환
    int ego_mx = (int)std::floor((ego_x - origin_x) / res);
    int ego_my = (int)std::floor((ego_y - origin_y) / res);
    int search_cells = (int)std::ceil(search_radius / res);
    
    for (int dy = -search_cells; dy <= search_cells; ++dy) {
        for (int dx = -search_cells; dx <= search_cells; ++dx) {
            int mx = ego_mx + dx;
            int my = ego_my + dy;
            
            if (mx < 0 || my < 0 || mx >= w || my >= h) continue;
            
            int8_t v = grid.data[my * w + mx];
            if (v < 0) continue; // unknown
            if (v >= config_.occupancy_threshold) return true; // 장애물 발견
        }
    }
    return false;
}

bool ObstacleDetector::hasObstacles(double ego_x, double ego_y) const {
    return hasLidarObstacles() || hasOccupancyObstacles(ego_x, ego_y);
}

advanced::ObstacleDetector::NormalizedCosts ObstacleDetector::calculateNormalizedCosts(const std::vector<geometry_msgs::msg::Point>& path_points) const {
    std::lock_guard<std::mutex> lk(grid_mutex_);
    NormalizedCosts costs = {0.0, 0.0, 0.0};
    
    if (!latest_grid_) return costs;
    
    const auto & grid = *latest_grid_;
    if (grid.info.resolution <= 0 || grid.info.width == 0 || grid.info.height == 0) return costs;
    
    double res = grid.info.resolution;
    int w = (int)grid.info.width;
    int h = (int)grid.info.height;
    double origin_x = grid.info.origin.position.x;
    double origin_y = grid.info.origin.position.y;
    
    int obstacle_points = 0;
    int unknown_points = 0;
    double total_proximity = 0.0;
    int valid_points = 0;
    
    // 각 경로 포인트에 대해 분석
    for (const auto &p : path_points) {
        int mx = (int)std::floor((p.x - origin_x) / res);
        int my = (int)std::floor((p.y - origin_y) / res);
        
        if (mx < 0 || my < 0 || mx >= w || my >= h) {
            obstacle_points++; // 맵 밖은 장애물로 취급
            valid_points++;
            continue;
        }
        
        int8_t v = grid.data[my * w + mx];
        
        if (v < 0) {
            // Unknown 영역
            unknown_points++;
        } else if (v >= config_.occupancy_threshold) {
            // 장애물
            obstacle_points++;
        } else {
            // 자유 공간 - 주변 장애물과의 거리 계산
            double proximity = calculateProximityCost(mx, my, grid);
            total_proximity += proximity;
        }
        valid_points++;
    }
    
    if (valid_points > 0) {
        // 정규화: 0-1 범위로 변환
        costs.obstacle_existence = (double)obstacle_points / valid_points;
        costs.unknown_area = (double)unknown_points / valid_points;
        costs.obstacle_distance = total_proximity / (valid_points * 6.0); // 최대 proximity 6.0으로 정규화 (3m 범위)
        costs.obstacle_distance = std::min(1.0, costs.obstacle_distance);
    }
    
    return costs;
}

double ObstacleDetector::calculateOccupancyCost(const std::vector<geometry_msgs::msg::Point>& path_points) const {
    // 기존 함수 유지 (하위 호환성)
    NormalizedCosts norm_costs = calculateNormalizedCosts(path_points);
    
    // 가중치 합을 통한 최종 비용 계산
    double weighted_cost = 
        norm_costs.obstacle_existence * 3.0 +    // 장애물 존재 시 높은 가중치
        norm_costs.unknown_area * 1.0 +         // 미지 영역 적당한 가중치
        norm_costs.obstacle_distance * 2.0;     // 거리 기반 중간 가중치
    
    return weighted_cost;
}

bool ObstacleDetector::pathCollidesWithLidar(const std::vector<geometry_msgs::msg::Point>& path_points) const {
    if (detected_obstacles_.empty()) return false;
    
    // 차량/장애물 반경 및 여유 거리 설정 (간단 상수, 추후 파라미터화 가능)
    constexpr double kVehicleRadius = 0.18;      // F1TENTH 대략 반경 (차폭~0.31m 기준)
    constexpr double kMinObstacleRadius = 0.05;  // 최소 장애물 반경 가정 (아주 작은 물체 보호)
    constexpr double kSafetyMargin = 0.07;       // 추가 여유 거리

    for (const auto &p : path_points) {
        for (const auto &obs : detected_obstacles_) {
            double dx = p.x - obs.x;
            double dy = p.y - obs.y;
            // 장애물의 대략적 반경(클러스터 size의 절반)과 차량 반경, 여유를 합산해 충돌 반경 구성
            double obs_radius = std::max(kMinObstacleRadius, obs.size * 0.5);
            double collide_r = kVehicleRadius + obs_radius + kSafetyMargin;
            if (dx*dx + dy*dy < collide_r * collide_r) {
                return true;
            }
        }
    }
    return false;
}

bool ObstacleDetector::pathCollidesWithOccupancy(const std::vector<geometry_msgs::msg::Point>& path_points) const {
    std::lock_guard<std::mutex> lk(grid_mutex_);
    if (!latest_grid_) return false; // grid 없으면 충돌 아님
    
    const auto & grid = *latest_grid_;
    if (grid.info.resolution <= 0 || grid.info.width == 0 || grid.info.height == 0) return false;

    double res = grid.info.resolution;
    double inflate_r = config_.occupancy_inflation_radius;
    int inflate_cells = std::max(0, (int)std::ceil(inflate_r / res));
    int w = (int)grid.info.width;
    int h = (int)grid.info.height;
    double origin_x = grid.info.origin.position.x;
    double origin_y = grid.info.origin.position.y;

    auto is_occ = [&](int mx, int my)->bool {
        if (mx < 0 || my < 0 || mx >= w || my >= h) return false; // 바깥은 비점유로 취급
        int8_t v = grid.data[my * w + mx];
        if (v < 0) return config_.unknown_is_obstacle; // unknown 처리 정책
        return v >= config_.occupancy_threshold;
    };

    // 각 경로 포인트 검사 (팽창 포함 박스 스캔)
    for (const auto &p : path_points) {
        int mx = (int)std::floor((p.x - origin_x) / res);
        int my = (int)std::floor((p.y - origin_y) / res);
        if (mx < -inflate_cells || my < -inflate_cells || mx >= w + inflate_cells || my >= h + inflate_cells) continue;
        
        for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
            for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
                int qx = mx + dx;
                int qy = my + dy;
                if (!is_occ(qx, qy)) continue;
                
                // 원형 팽창: 중심 거리 체크
                double cx = origin_x + (qx + 0.5) * res;
                double cy = origin_y + (qy + 0.5) * res;
                double ddx = p.x - cx; 
                double ddy = p.y - cy;
                if (ddx*ddx + ddy*ddy <= inflate_r * inflate_r + 1e-6) {
                    return true; // 충돌
                }
            }
        }
    }
    return false;
}

bool ObstacleDetector::pathCollides(const std::vector<geometry_msgs::msg::Point>& path_points) const {
    return pathCollidesWithLidar(path_points) || pathCollidesWithOccupancy(path_points);
}

double ObstacleDetector::calculateProximityCost(int mx, int my, const nav_msgs::msg::OccupancyGrid& grid) const {
    int w = (int)grid.info.width;
    int h = (int)grid.info.height;
    
    // 설정 가능한 범위로 확장된 장애물 탐지 (해상도에 따라 동적 계산)
    int search_radius = std::min(50, (int)std::ceil(config_.proximity_search_radius / grid.info.resolution));
    
    double min_distance = search_radius + 1;
    
    // 주변 셀들을 검사하여 가장 가까운 장애물까지의 거리 계산
    for (int dy = -search_radius; dy <= search_radius; ++dy) {
        for (int dx = -search_radius; dx <= search_radius; ++dx) {
            int qx = mx + dx;
            int qy = my + dy;
            
            if (qx < 0 || qy < 0 || qx >= w || qy >= h) continue;
            
            int8_t v = grid.data[qy * w + qx];
            if (v >= config_.occupancy_threshold) {
                double distance = std::sqrt(dx*dx + dy*dy);
                min_distance = std::min(min_distance, distance);
            }
        }
    }
    
    // 거리가 가까울수록 높은 cost (설정된 범위에서 정규화)
    if (min_distance <= search_radius) {
        // 설정된 범위에서 정규화된 비용 계산
        double distance_ratio = min_distance / search_radius;
        return (1.0 - distance_ratio) * 6.0; // 최대 6.0점, 가까울수록 높은 비용
    }
    return 0.0;
}

// 새로운 라이다 전처리 함수들 구현
std::vector<double> ObstacleDetector::preprocessLidarData(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    std::vector<double> ranges(scan->ranges.begin(), scan->ranges.end());
    
    // 1단계: 중앙값 필터링 (노이즈 제거)
    ranges = medianFilter(ranges, config_.median_filter_size);
    
    // 2단계: 이상치 제거
    ranges = removeOutliers(ranges, *scan);
    
    return ranges;
}

std::vector<double> ObstacleDetector::medianFilter(const std::vector<double>& ranges, int window_size) {
    if (window_size <= 1) return ranges;
    
    std::vector<double> filtered = ranges;
    int half_window = window_size / 2;
    
    for (size_t i = half_window; i < ranges.size() - half_window; ++i) {
        std::vector<double> window;
        for (int j = -half_window; j <= half_window; ++j) {
            double val = ranges[i + j];
            if (!std::isnan(val) && !std::isinf(val)) {
                window.push_back(val);
            }
        }
        
        if (!window.empty()) {
            std::sort(window.begin(), window.end());
            filtered[i] = window[window.size() / 2];
        }
    }
    
    return filtered;
}

std::vector<double> ObstacleDetector::removeOutliers(const std::vector<double>& ranges, 
                                                    const sensor_msgs::msg::LaserScan& scan) {
    std::vector<double> cleaned = ranges;
    
    for (size_t i = 1; i < ranges.size() - 1; ++i) {
        if (std::isnan(ranges[i]) || std::isinf(ranges[i])) continue;
        
        double prev = ranges[i-1];
        double curr = ranges[i];
        double next = ranges[i+1];
        
        // 이웃 포인트들과의 거리 차이가 임계값보다 크면 이상치로 판단
        bool is_outlier = false;
        if (!std::isnan(prev) && !std::isinf(prev)) {
            if (std::abs(curr - prev) > config_.outlier_threshold) {
                is_outlier = true;
            }
        }
        if (!std::isnan(next) && !std::isinf(next)) {
            if (std::abs(curr - next) > config_.outlier_threshold) {
                is_outlier = true;
            }
        }
        
        if (is_outlier) {
            cleaned[i] = std::numeric_limits<double>::quiet_NaN();
        }
    }
    
    return cleaned;
}

std::vector<AdvancedObstacle> ObstacleDetector::clusterLidarPoints(
    const std::vector<std::pair<double, double>>& points,
    const rclcpp::Clock::SharedPtr& clock) {
    
    std::vector<AdvancedObstacle> obstacles;
    if (points.empty()) return obstacles;
    
    std::vector<bool> clustered(points.size(), false);
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (clustered[i]) continue;
        
        // 새로운 클러스터 시작
        std::vector<std::pair<double, double>> cluster;
        std::vector<size_t> to_check = {i};
        clustered[i] = true;
        
        while (!to_check.empty()) {
            size_t current_idx = to_check.back();
            to_check.pop_back();
            cluster.push_back(points[current_idx]);
            
            // 근처 포인트들 찾기
            for (size_t j = 0; j < points.size(); ++j) {
                if (clustered[j]) continue;
                
                double dx = points[current_idx].first - points[j].first;
                double dy = points[current_idx].second - points[j].second;
                double distance = std::sqrt(dx*dx + dy*dy);
                
                if (distance < config_.cluster_distance_threshold) {
                    clustered[j] = true;
                    to_check.push_back(j);
                }
            }
        }
        
        // 클러스터 검증 및 장애물 생성
        if (isValidCluster(cluster)) {
            AdvancedObstacle obstacle;
            
            // 클러스터 중심 계산
            double sum_x = 0, sum_y = 0;
            for (const auto& point : cluster) {
                sum_x += point.first;
                sum_y += point.second;
            }
            obstacle.x = sum_x / cluster.size();
            obstacle.y = sum_y / cluster.size();
            
            // 거리 및 각도 계산
            obstacle.distance = std::sqrt(obstacle.x*obstacle.x + obstacle.y*obstacle.y);
            obstacle.angle = std::atan2(obstacle.y, obstacle.x);
            
            // 크기 계산
            obstacle.size = calculateObstacleSize(cluster);
            
            // 기타 속성 초기화
            obstacle.velocity_x = 0.0;
            obstacle.velocity_y = 0.0;
            obstacle.intensity = 0;
            obstacle.is_dynamic = false;
            obstacle.track_id = -1;
            obstacle.timestamp = clock->now();
            obstacle.cluster_points = cluster;
            
            obstacles.push_back(obstacle);
        }
    }
    
    return obstacles;
}

void ObstacleDetector::updateTracking(const std::vector<AdvancedObstacle>& new_obstacles,
                                     const rclcpp::Clock::SharedPtr& clock) {
    
    // 기존 트랙들의 lost_frames 증가
    for (auto& track : tracked_obstacles_) {
        track.lost_frames++;
    }
    
    // 새로운 장애물과 기존 트랙 매칭
    std::vector<bool> obstacle_matched(new_obstacles.size(), false);
    
    for (auto& track : tracked_obstacles_) {
        double min_distance = std::numeric_limits<double>::max();
        int best_match = -1;
        
        for (size_t i = 0; i < new_obstacles.size(); ++i) {
            if (obstacle_matched[i]) continue;
            
            double distance = distanceBetween(track.current_state, new_obstacles[i]);
            if (distance < config_.tracking_distance_threshold && distance < min_distance) {
                min_distance = distance;
                best_match = i;
            }
        }
        
        if (best_match >= 0) {
            // 매칭된 경우 트랙 업데이트
            obstacle_matched[best_match] = true;
            track.history.push_back(track.current_state);
            track.current_state = new_obstacles[best_match];
            track.current_state.track_id = track.track_id;
            track.lost_frames = 0;
            track.is_confirmed = true;
            
            // 히스토리 크기 제한
            if (track.history.size() > 10) {
                track.history.erase(track.history.begin());
            }
        }
    }
    
    // 매칭되지 않은 새로운 장애물들을 새 트랙으로 추가
    for (size_t i = 0; i < new_obstacles.size(); ++i) {
        if (!obstacle_matched[i]) {
            ObstacleTrack new_track;
            new_track.track_id = next_track_id_++;
            new_track.current_state = new_obstacles[i];
            new_track.current_state.track_id = new_track.track_id;
            new_track.lost_frames = 0;
            new_track.is_confirmed = false;  // 여러 프레임에서 확인되어야 함
            
            tracked_obstacles_.push_back(new_track);
        }
    }
    
    // 오래된 트랙들 제거
    tracked_obstacles_.erase(
        std::remove_if(tracked_obstacles_.begin(), tracked_obstacles_.end(),
            [this](const ObstacleTrack& track) {
                return track.lost_frames > config_.max_lost_frames;
            }),
        tracked_obstacles_.end()
    );
}

void ObstacleDetector::estimateVelocities() {
    for (auto& track : tracked_obstacles_) {
        if (track.history.size() < 2) continue;
        
        const auto& current = track.current_state;
        const auto& prev = track.history.back();
        
        double dt = (current.timestamp - prev.timestamp).seconds();
        if (dt > 0.001 && dt < config_.velocity_estimation_window) {
            track.current_state.velocity_x = (current.x - prev.x) / dt;
            track.current_state.velocity_y = (current.y - prev.y) / dt;
            
            double speed = std::sqrt(track.current_state.velocity_x * track.current_state.velocity_x +
                                   track.current_state.velocity_y * track.current_state.velocity_y);
            
            track.current_state.is_dynamic = (speed > config_.min_dynamic_velocity);
        }
    }
}

// 헬퍼 함수들 구현
double ObstacleDetector::calculateObstacleSize(const std::vector<std::pair<double, double>>& points) {
    if (points.size() < 2) return config_.min_obstacle_size;
    
    double min_x = points[0].first, max_x = points[0].first;
    double min_y = points[0].second, max_y = points[0].second;
    
    for (const auto& point : points) {
        min_x = std::min(min_x, point.first);
        max_x = std::max(max_x, point.first);
        min_y = std::min(min_y, point.second);
        max_y = std::max(max_y, point.second);
    }
    
    return std::max(max_x - min_x, max_y - min_y);
}

bool ObstacleDetector::isValidCluster(const std::vector<std::pair<double, double>>& cluster) {
    if (cluster.size() < static_cast<size_t>(config_.min_cluster_size)) {
        return false;
    }
    
    double size = calculateObstacleSize(cluster);
    return (size >= config_.min_obstacle_size && size <= config_.max_obstacle_size);
}

double ObstacleDetector::distanceBetween(const AdvancedObstacle& a, const AdvancedObstacle& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

// 새로운 접근자 함수들 구현
std::vector<AdvancedObstacle> ObstacleDetector::getTrackedObstacles() const {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    std::vector<AdvancedObstacle> obstacles;
    
    for (const auto& track : tracked_obstacles_) {
        if (track.is_confirmed && track.lost_frames < config_.max_lost_frames) {
            obstacles.push_back(track.current_state);
        }
    }
    
    return obstacles;
}

std::vector<AdvancedObstacle> ObstacleDetector::getDynamicObstacles() const {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    std::vector<AdvancedObstacle> dynamic_obstacles;
    
    for (const auto& track : tracked_obstacles_) {
        if (track.is_confirmed && track.lost_frames < config_.max_lost_frames && 
            track.current_state.is_dynamic) {
            dynamic_obstacles.push_back(track.current_state);
        }
    }
    
    return dynamic_obstacles;
}

std::vector<AdvancedObstacle> ObstacleDetector::getStaticObstacles() const {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    std::vector<AdvancedObstacle> static_obstacles;
    
    for (const auto& track : tracked_obstacles_) {
        if (track.is_confirmed && track.lost_frames < config_.max_lost_frames && 
            !track.current_state.is_dynamic) {
            static_obstacles.push_back(track.current_state);
        }
    }
    
    return static_obstacles;
}

} // namespace advanced
} // namespace lattice_planner_pkg