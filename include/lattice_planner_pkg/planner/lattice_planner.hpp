#pragma once

#include "lattice_planner_pkg/core/types.hpp"
#include "lattice_planner_pkg/core/frenet_coordinate.hpp"
#include "lattice_planner_pkg/core/path_generator.hpp"
#include "lattice_planner_pkg/core/obstacle_detector.hpp"
#include "lattice_planner_pkg/core/spline_utils.hpp"
#include "lattice_planner_pkg/obstacle_detector.hpp"
#include "lattice_planner_pkg/path_selector.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <planning_custom_msgs/msg/path_with_velocity.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


#include <memory>
#include <vector>
#include <mutex>

namespace lattice_planner_pkg {

class LatticePlanner : public rclcpp::Node {
public:
    LatticePlanner();
    ~LatticePlanner();
    
private:
    // 첨부 코드 기반 초기화 멤버들
    void initialize_hysteresis_variables();

private:
    // ROS2 Publishers and Subscribers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<planning_custom_msgs::msg::PathWithVelocity>::SharedPtr path_with_velocity_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    
    rclcpp::TimerBase::SharedPtr planning_timer_;
    rclcpp::TimerBase::SharedPtr ref_path_timer_;
    
    // Core modules
    PlannerConfig config_;
    std::shared_ptr<FrenetCoordinate> frenet_coord_;
    std::shared_ptr<PathGenerator> path_generator_;
    std::shared_ptr<ObstacleDetector> obstacle_detector_;
    
    // Advanced planning modules
    std::unique_ptr<advanced::ObstacleDetector> advanced_obstacle_detector_;
    std::unique_ptr<advanced::PathSelector> path_selector_;
    
    // Vehicle state
    Point2D vehicle_position_;
    double vehicle_yaw_;
    double vehicle_velocity_;
    bool odom_received_;
    std::mutex vehicle_state_mutex_;
    
    // Obstacles
    std::vector<Obstacle> current_obstacles_;
    std::mutex obstacles_mutex_;
    
    // Occupancy Grid for track boundary detection
    nav_msgs::msg::OccupancyGrid::SharedPtr current_grid_;
    std::mutex grid_mutex_;
    
    // Reference path
    std::vector<RefPoint> reference_path_;
    
    // 🎯 첨부 코드 기반 Path Selection Stability (히스테리시스)
    double cached_chosen_offset_;           // 현재 선택된 경로 offset 캐시
    int stable_frames_since_switch_;        // 마지막 전환 이후 안정 프레임 수
    const int min_stable_frames_ = 3;       // 최소 안정 프레임 수
    double last_closest_s_;                 // 이전 closest s 값
    
    // 🏁 첨부 코드 기반 Path Execution Mode (장애물 회피 지속성)
    bool path_execution_mode_;              // 경로 실행 모드 (고정 경로 유지)
    bool in_additional_execution_;          // 추가 실행 모드 (2m 더 가기)
    bool obstacle_avoidance_active_;        // 장애물 회피 모드 활성화
    bool force_path_reevaluation_;          // 강제 경로 재평가 플래그
    
    // 🛡️ Adaptive Commit System (벽 박기 방지 + 동적 대응)
    PathCandidate committed_path_;          // 현재 commit된 경로
    rclcpp::Time commit_time_;              // commit 시간
    bool has_committed_path_;               // commit된 경로가 있는지
    double commit_duration_safe_;           // 안전 상황에서 commit 유지 시간 (초)
    double commit_duration_danger_;         // 위험 상황에서 commit 유지 시간 (초)
    double fixed_path_end_s_;               // 고정 경로 끝 지점 s
    double fixed_path_offset_;              // 고정 경로 lateral offset
    double additional_execution_start_s_;   // 추가 실행 시작 s
    const double additional_execution_distance_ = 2.0; // 추가 실행 거리 (2m)
    
    // 📊 첨부 코드 기반 Selection Context (closest_s 기반 경로 선택)
    double closest_s_search_range_ = 10.0;  // closest point 탐색 범위
    const double reference_offset_target_ = 0.0;   // reference path target offset
    const double reference_offset_tolerance_ = 0.05; // reference path tolerance
    
    // 🎨 시각화를 위한 advanced candidates 캐시
    std::vector<advanced::CandidateResult> advanced_candidates_cache_;
    
    // 🚧 Detour (우회) 상태 관리
    bool detour_active_;                    // 현재 우회 모드 여부
    int detour_clear_frames_;               // reference path가 깨끗한 연속 프레임 수
    const int detour_return_clear_frames_threshold_ = 3; // 복귀 필요 임계값
    
    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void planning_timer_callback();
    void ref_path_timer_callback();
    
    // Core functions
    bool initialize();
    bool load_reference_path();
    void plan_paths();
    
    // Path selection functions
    PathCandidate select_best_path_adaptive(const std::vector<PathCandidate>& candidates);
    PathCandidate select_best_path(const std::vector<PathCandidate>& candidates);
    
    // Adaptive commit functions
    bool should_keep_committed_path(const std::vector<PathCandidate>& candidates);
    bool is_safe_to_commit(const PathCandidate& path, const std::vector<PathCandidate>& candidates);
    bool is_path_still_safe(const PathCandidate& path);
    
    // 첨부 코드 기반 closest point 계산
    double calculate_closest_s(const Point2D& vehicle_pos);
    
    // 첨부 코드 기반 히스테리시스 로직
    void apply_hysteresis_logic(PathCandidate& selected_path, double closest_s);
    
    // Publishing functions
    void publish_selected_path(const PathCandidate& path);
    void publish_path_visualization(const std::vector<PathCandidate>& candidates, 
                                   const PathCandidate& selected);
    void publish_reference_path();
    
    // Utility functions
    nav_msgs::msg::Path convert_to_nav_path(const PathCandidate& path);
    planning_custom_msgs::msg::PathWithVelocity convert_to_path_with_velocity(const PathCandidate& path);
    visualization_msgs::msg::MarkerArray create_path_markers(
        const std::vector<PathCandidate>& candidates,
        const PathCandidate& selected
    );
};

} // namespace lattice_planner_pkg