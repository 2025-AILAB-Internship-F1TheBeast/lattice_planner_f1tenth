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
    double velocity_x, velocity_y; // 속도 (동적 장애물용)
    double size;           // 장애물 크기 (클러스터링 결과)
    int intensity;         // 라이다 반사 강도
    bool is_dynamic;       // 동적 장애물 여부
    int track_id;          // 추적 ID
    rclcpp::Time timestamp; // 감지 시간
    std::vector<std::pair<double, double>> cluster_points; // 클러스터 구성 점들
};

struct ObstacleDetectionConfig {
    // 기본 감지 설정
    double max_detection_range = 8.0;     // 라이다 최대 감지 범위
    double lateral_range = M_PI / 2.5;    // 좌우 144도 (넓은 시야각)
    double forward_distance_max = 6.0;    // 전방 거리
    double lateral_distance_max = 3.0;    // 좌우 거리
    
    // 라이다 전처리 설정
    double min_range_threshold = 0.1;     // 최소 거리 (노이즈 제거)
    double max_range_threshold = 10.0;    // 최대 거리
    int median_filter_size = 3;           // 중앙값 필터 크기
    double outlier_threshold = 0.5;       // 이상치 제거 임계값
    
    // 클러스터링 설정
    double cluster_distance_threshold = 0.3; // 클러스터링 거리 임계값
    int min_cluster_size = 3;             // 최소 클러스터 크기
    double min_obstacle_size = 0.05;      // 최소 장애물 크기
    double max_obstacle_size = 3.0;       // 최대 장애물 크기
    
    // 동적 장애물 추적 설정
    double tracking_distance_threshold = 0.5; // 추적 거리 임계값
    int max_lost_frames = 5;              // 최대 손실 프레임 수
    double velocity_estimation_window = 0.5; // 속도 추정 시간 윈도우 (초)
    double min_dynamic_velocity = 0.3;    // 동적 판정 최소 속도 (m/s)
    
    // Occupancy grid 설정 - 더 현실적인 값으로 조정
    int occupancy_threshold = 60;  // 35 -> 60으로 상향 조정 (덜 민감하게)
    double occupancy_inflation_radius = 0.25;  // 0.4 -> 0.25로 축소
    double proximity_search_radius = 3.0;
    bool unknown_is_obstacle = false;
};

// 장애물 추적 정보
struct ObstacleTrack {
    int track_id;
    AdvancedObstacle current_state;
    std::vector<AdvancedObstacle> history;  // 최근 상태 이력
    int lost_frames;
    bool is_confirmed;  // 여러 프레임에서 확인된 장애물
};

class ObstacleDetector {
public:
    ObstacleDetector(const ObstacleDetectionConfig& config = ObstacleDetectionConfig());
    
    // 향상된 LiDAR 기반 장애물 탐지
    void detectObstaclesFromScan(
        const sensor_msgs::msg::LaserScan::SharedPtr scan,
        double ego_x, double ego_y, double ego_yaw,
        const rclcpp::Clock::SharedPtr& clock
    );
    
    // Occupancy grid 업데이트
    void updateOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    bool hasOccupancyGrid() const;
    
    // 장애물 존재 여부 확인
    bool hasLidarObstacles() const;
    bool hasOccupancyObstacles(double ego_x, double ego_y) const;
    bool hasObstacles(double ego_x, double ego_y) const;
    
    // 새로운 고급 기능들
    std::vector<AdvancedObstacle> getTrackedObstacles() const;
    std::vector<AdvancedObstacle> getDynamicObstacles() const;
    std::vector<AdvancedObstacle> getStaticObstacles() const;
    
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
    std::vector<ObstacleTrack> tracked_obstacles_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;
    mutable std::mutex grid_mutex_;
    mutable std::mutex obstacles_mutex_;
    
    int next_track_id_;
    rclcpp::Time last_scan_time_;
    
    // 라이다 데이터 전처리
    std::vector<double> preprocessLidarData(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    std::vector<double> medianFilter(const std::vector<double>& ranges, int window_size);
    std::vector<double> removeOutliers(const std::vector<double>& ranges, const sensor_msgs::msg::LaserScan& scan);
    
    // 클러스터링 및 추적
    std::vector<AdvancedObstacle> clusterLidarPoints(
        const std::vector<std::pair<double, double>>& points,
        const rclcpp::Clock::SharedPtr& clock);
    void updateTracking(const std::vector<AdvancedObstacle>& new_obstacles,
                       const rclcpp::Clock::SharedPtr& clock);
    void estimateVelocities();
    
    // 헬퍼 함수들
    double calculateObstacleSize(const std::vector<std::pair<double, double>>& points);
    bool isValidCluster(const std::vector<std::pair<double, double>>& cluster);
    double distanceBetween(const AdvancedObstacle& a, const AdvancedObstacle& b);
    
    // 근접도 기반 비용 계산
    double calculateProximityCost(int mx, int my, const nav_msgs::msg::OccupancyGrid& grid) const;
};

} // namespace advanced
} // namespace lattice_planner_pkg