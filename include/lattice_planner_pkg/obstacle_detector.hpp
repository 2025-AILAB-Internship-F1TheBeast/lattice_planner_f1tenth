#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <mutex>

namespace lattice_planner_pkg {
namespace advanced {

// Advanced obstacle detection - separate from core obstacle detector
struct AdvancedObstacle {
    double x, y;           // 위치
    double distance;       // 차량과의 거리
    double angle;          // 차량 기준 각도
    rclcpp::Time timestamp; // 감지 시간
};

struct ObstacleDetectionConfig {
    double max_detection_range = 8.0;
    double lateral_range = M_PI / 2;  // ±90도
    double forward_distance_max = 6.0;
    double lateral_distance_max = 3.0;
    int occupancy_threshold = 50;
    double occupancy_inflation_radius = 0.4;
    double proximity_search_radius = 3.0;  // 근접 비용 계산 범위 (m)
    bool unknown_is_obstacle = false;
};

class ObstacleDetector {
public:
    ObstacleDetector(const ObstacleDetectionConfig& config = ObstacleDetectionConfig());
    
    // LiDAR 기반 장애물 탐지
    void detectObstaclesFromScan(
        const sensor_msgs::msg::LaserScan::SharedPtr scan,
        double ego_x, double ego_y, double ego_yaw,
        const rclcpp::Clock::SharedPtr& clock
    );
    
    // Occupancy grid 업데이트
    void updateOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    // 장애물 존재 여부 확인
    bool hasLidarObstacles() const;
    bool hasOccupancyObstacles(double ego_x, double ego_y) const;
    bool hasObstacles(double ego_x, double ego_y) const;
    
    // 경로 비용 계산
    double calculateOccupancyCost(const std::vector<geometry_msgs::msg::Point>& path_points) const;
    
    // 정규화된 비용 계산 (장애물 존재, 미지 영역, 거리 기반)
    struct NormalizedCosts {
        double obstacle_existence;  // 0-1 정규화된 장애물 존재 비용
        double unknown_area;       // 0-1 정규화된 미지 영역 비용  
        double obstacle_distance;  // 0-1 정규화된 장애물 거리 비용
    };
    NormalizedCosts calculateNormalizedCosts(const std::vector<geometry_msgs::msg::Point>& path_points) const;
    
    // 경로 충돌 검사
    bool pathCollidesWithLidar(const std::vector<geometry_msgs::msg::Point>& path_points) const;
    bool pathCollidesWithOccupancy(const std::vector<geometry_msgs::msg::Point>& path_points) const;
    bool pathCollides(const std::vector<geometry_msgs::msg::Point>& path_points) const;
    
    // 탐지된 장애물 정보 접근
    const std::vector<AdvancedObstacle>& getDetectedObstacles() const { return detected_obstacles_; }
    
    // 설정 업데이트
    void updateConfig(const ObstacleDetectionConfig& config) { config_ = config; }

private:
    ObstacleDetectionConfig config_;
    std::vector<AdvancedObstacle> detected_obstacles_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;
    mutable std::mutex grid_mutex_;
    
    // 근접도 기반 비용 계산
    double calculateProximityCost(int mx, int my, const nav_msgs::msg::OccupancyGrid& grid) const;
};

} // namespace advanced
} // namespace lattice_planner_pkg