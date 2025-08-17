#include "lattice_planner_pkg/obstacle_detector.hpp"
#include <cmath>
#include <limits>

namespace lattice_planner_pkg {
namespace advanced {

ObstacleDetector::ObstacleDetector(const ObstacleDetectionConfig& config) : config_(config) {}

void ObstacleDetector::detectObstaclesFromScan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan,
    double ego_x, double ego_y, double ego_yaw,
    const rclcpp::Clock::SharedPtr& clock) {
    
    detected_obstacles_.clear();
    
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double range = scan->ranges[i];
        
        if (range < scan->range_min || range > scan->range_max || 
            range > config_.max_detection_range) {
            continue;
        }
        
        double angle = scan->angle_min + i * scan->angle_increment;
        
        // 전방 ±90도 범위만 고려 (후방 장애물 무시)
        if (std::abs(angle) > config_.lateral_range) {
            continue;
        }
        
        double obstacle_x = ego_x + range * std::cos(ego_yaw + angle);
        double obstacle_y = ego_y + range * std::sin(ego_yaw + angle);
        
        // 차량 진행 방향 기준 필터링 추가
        double forward_distance = range * std::cos(angle);  // 전방 거리
        double lateral_distance = std::abs(range * std::sin(angle));  // 좌우 거리
        
        // 전방 6m 이내, 좌우 3m 이내만 장애물로 인식
        if (forward_distance < 0.0 || forward_distance > config_.forward_distance_max || 
            lateral_distance > config_.lateral_distance_max) {
            continue;
        }
        
        AdvancedObstacle obs;
        obs.x = obstacle_x;
        obs.y = obstacle_y;
        obs.distance = range;
        obs.angle = angle;
        obs.timestamp = clock->now();
        
        detected_obstacles_.push_back(obs);
    }
}

void ObstacleDetector::updateOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(grid_mutex_);
    latest_grid_ = msg;
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
    double search_radius = 8.0; // 8m 범위
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
    
    for (const auto &p : path_points) {
        for (const auto &obs : detected_obstacles_) {
            double dx = p.x - obs.x;
            double dy = p.y - obs.y;
            if (dx*dx + dy*dy < 0.16) { // 0.4m 충돌 반경
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

} // namespace advanced
} // namespace lattice_planner_pkg