#include "lattice_planner_pkg/planner/lattice_planner.hpp"
#include "planning_custom_msgs/msg/path_point_array.hpp"
#include "planning_custom_msgs/msg/path_with_velocity.hpp"
#include "lattice_planner_pkg/obstacle_detector.hpp"
#include "lattice_planner_pkg/path_selector.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace lattice_planner_pkg {

LatticePlanner::LatticePlanner() 
    : Node("lattice_planner"),
      vehicle_yaw_(0.0),
      vehicle_velocity_(0.0),
      odom_received_(false) {
    
    if (!initialize()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize lattice planner");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Lattice Planner initialized successfully");
}

LatticePlanner::~LatticePlanner() {
}

bool LatticePlanner::initialize() {
    // Initialize TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Load configuration parameters
    this->declare_parameter("reference_path_file", std::string(""));
    this->declare_parameter("path_resolution", 0.1);
    this->declare_parameter("lateral_step", 0.5);
    this->declare_parameter("max_lateral_offset", 2.0);
    this->declare_parameter("planning_horizon", 2.0);
    this->declare_parameter("dt", 0.1);
    this->declare_parameter("max_velocity", 8.0);
    this->declare_parameter("planning_frequency", 10.0);
    this->declare_parameter("max_curvature", 1.0);
    this->declare_parameter("lateral_cost_weight", 1.0);
    this->declare_parameter("curvature_cost_weight", 1.0);
    this->declare_parameter("longitudinal_cost_weight", 0.1);
    this->declare_parameter("obstacle_cost_weight", 10.0);
    this->declare_parameter("obstacle_existence_weight", 3.0);
    this->declare_parameter("unknown_area_weight", 1.0);
    this->declare_parameter("obstacle_distance_weight", 2.0);
    this->declare_parameter("occupancy_grid_topic", std::string("/dynamic_map"));
    
    config_.reference_path_file = this->get_parameter("reference_path_file").as_string();
    config_.path_resolution = this->get_parameter("path_resolution").as_double();
    config_.lateral_step = this->get_parameter("lateral_step").as_double();
    config_.max_lateral_offset = this->get_parameter("max_lateral_offset").as_double();
    config_.planning_horizon = this->get_parameter("planning_horizon").as_double();
    config_.dt = this->get_parameter("dt").as_double();
    config_.max_velocity = this->get_parameter("max_velocity").as_double();
    config_.max_curvature = this->get_parameter("max_curvature").as_double();
    config_.lateral_cost_weight = this->get_parameter("lateral_cost_weight").as_double();
    config_.curvature_cost_weight = this->get_parameter("curvature_cost_weight").as_double();
    config_.longitudinal_cost_weight = this->get_parameter("longitudinal_cost_weight").as_double();
    config_.obstacle_cost_weight = this->get_parameter("obstacle_cost_weight").as_double();
    config_.obstacle_existence_weight = this->get_parameter("obstacle_existence_weight").as_double();
    config_.unknown_area_weight = this->get_parameter("unknown_area_weight").as_double();
    config_.obstacle_distance_weight = this->get_parameter("obstacle_distance_weight").as_double();
    
    double planning_frequency = this->get_parameter("planning_frequency").as_double();
    std::string occupancy_grid_topic = this->get_parameter("occupancy_grid_topic").as_string();
    
    // Load reference path
    if (!load_reference_path()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load reference path");
        return false;
    }
    
    // Initialize modules
    frenet_coord_ = std::make_shared<FrenetCoordinate>();
    if (!frenet_coord_->initialize(reference_path_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Frenet coordinate system");
        return false;
    }
    
    path_generator_ = std::make_shared<PathGenerator>(config_);
    path_generator_->set_frenet_coordinate(frenet_coord_);
    
    // Initialize enhanced LiDAR obstacle detector
    advanced::ObstacleDetectionConfig obs_config;
    // 기본 감지 설정 - 향상된 범위
    obs_config.max_detection_range = 8.0;         // 더 넓은 감지 범위
    obs_config.lateral_range = M_PI / 2.0;        // 180도 시야각
    obs_config.forward_distance_max = 6.0;       // 전방 6m
    obs_config.lateral_distance_max = 3.0;       // 좌우 3m
    
    // 라이다 전처리 설정
    obs_config.min_range_threshold = 0.1;        // 최소 거리
    obs_config.max_range_threshold = 10.0;       // 최대 거리
    obs_config.median_filter_size = 3;           // 노이즈 제거
    obs_config.outlier_threshold = 0.5;          // 이상치 제거
    
    // 클러스터링 설정
    obs_config.cluster_distance_threshold = 0.3; // 클러스터링 거리
    obs_config.min_cluster_size = 3;             // 최소 클러스터 크기
    obs_config.min_obstacle_size = 0.05;         // 최소 장애물 크기
    obs_config.max_obstacle_size = 3.0;          // 최대 장애물 크기
    
    // 동적 장애물 추적 설정
    obs_config.tracking_distance_threshold = 0.5; // 추적 거리
    obs_config.max_lost_frames = 5;              // 최대 손실 프레임
    obs_config.velocity_estimation_window = 0.5; // 속도 추정 윈도우
    obs_config.min_dynamic_velocity = 0.3;       // 동적 판정 속도
    
    // Occupancy grid 설정
    obs_config.occupancy_threshold = 40;         // 임계값 완화
    obs_config.occupancy_inflation_radius = 0.3; // 팽창 반경
    obs_config.proximity_search_radius = 2.5;    // 근접 탐지 범위
    obs_config.unknown_is_obstacle = false;      // 미지 영역은 장애물 아님
    
    advanced_obstacle_detector_ = std::make_unique<advanced::ObstacleDetector>(obs_config);
    
    // 안전한 경로 선택을 위한 향상된 설정
    advanced::PathSelectionConfig sel_config;
    sel_config.commit_min_progress = 1.0;
    sel_config.commit_min_time_sec = 0.8;
    
    // 장애물 회피 지속성 강화
    sel_config.path_length = 4.0;                       // 기본 커밋 길이 증가
    sel_config.obstacle_path_length_multiplier = 1.5;   // 장애물 상황에서 1.5배 연장
    sel_config.path_length_commit_mode = true;          // 경로 길이 기반 커밋 활성화
    
    // 더 안정적인 detour 설정
    sel_config.detour_return_clear_frames_threshold = 8; // 더 많은 프레임 확인 후 복귀
    sel_config.reference_offset_tolerance = 0.05;       // raceline 허용 범위 약간 확대
    
    path_selector_ = std::make_unique<advanced::PathSelector>(sel_config);
    
    // Keep old detector for compatibility
    obstacle_detector_ = std::make_shared<ObstacleDetector>(config_);
    
    // Initialize publishers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/planned_path", 10);
    path_with_velocity_pub_ = this->create_publisher<planning_custom_msgs::msg::PathWithVelocity>(
        "/planned_path_with_velocity", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/path_candidates", 10);
    ref_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/reference_path", 10);
    
    // Initialize subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/pf/pose/odom", 10, 
        std::bind(&LatticePlanner::odom_callback, this, std::placeholders::_1));

    
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&LatticePlanner::laser_callback, this, std::placeholders::_1));
    
    grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        occupancy_grid_topic, 10,
        std::bind(&LatticePlanner::grid_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Subscribing to occupancy grid topic: %s", occupancy_grid_topic.c_str());
    
    // Initialize planning timer
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / planning_frequency));
    planning_timer_ = this->create_wall_timer(
        timer_period, std::bind(&LatticePlanner::planning_timer_callback, this));
    
    // Initialize reference path publishing timer (2 Hz)
    auto ref_path_timer_period = std::chrono::milliseconds(500); // 2 Hz = 500ms
    ref_path_timer_ = this->create_wall_timer(
        ref_path_timer_period, std::bind(&LatticePlanner::ref_path_timer_callback, this));
    
    // Publish reference path immediately
    publish_reference_path();
    
    return true;
}

bool LatticePlanner::load_reference_path() {
    if (config_.reference_path_file.empty()) {
        // Create a simple test path if no file specified
        RCLCPP_WARN(this->get_logger(), "No reference path file specified, creating test path");
        
        reference_path_.clear();
        for (int i = 0; i < 100; ++i) {
            RefPoint point;
            point.x = i * 0.5;
            point.y = 2.0 * std::sin(i * 0.1);
            point.velocity = 5.0;
            reference_path_.push_back(point);
        }
        
        SplineUtils::calculate_arc_length(reference_path_);
        SplineUtils::calculate_heading(reference_path_);
        SplineUtils::calculate_curvature(reference_path_);
    } else {
        // Resolve reference path file - similar to f1tenth_gym_ros approach
        std::string resolved_path;
        if (config_.reference_path_file.find('/') != std::string::npos) {
            // If full path is provided, use it directly
            resolved_path = config_.reference_path_file;
        } else {
            // Use package share directory for proper path resolution
            std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("lattice_planner_pkg");
            resolved_path = pkg_share_dir + "/config/reference_paths/" + config_.reference_path_file + ".csv";
        }
        
        reference_path_ = SplineUtils::load_reference_path_from_csv(
            resolved_path, config_.path_resolution);
    }
    
    if (reference_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Reference path is empty");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Reference path loaded successfully");
    return true;
}

void LatticePlanner::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
    
    vehicle_position_.x = msg->pose.pose.position.x;
    vehicle_position_.y = msg->pose.pose.position.y;
    
    // Convert quaternion to yaw
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    vehicle_yaw_ = yaw;
    
    // Calculate velocity from twist
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    vehicle_velocity_ = std::sqrt(vx*vx + vy*vy);
    
    odom_received_ = true;
}

void LatticePlanner::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!odom_received_) return;
    
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    Point2D vehicle_pos;
    double vehicle_yaw;
    {
        std::lock_guard<std::mutex> state_lock(vehicle_state_mutex_);
        vehicle_pos = vehicle_position_;
        vehicle_yaw = vehicle_yaw_;
    }
    
    // Use enhanced advanced obstacle detector
    advanced_obstacle_detector_->detectObstaclesFromScan(
        msg, vehicle_pos.x, vehicle_pos.y, vehicle_yaw, this->get_clock());
    
    // Get enhanced obstacle information
    auto tracked_obstacles = advanced_obstacle_detector_->getTrackedObstacles();
    auto dynamic_obstacles = advanced_obstacle_detector_->getDynamicObstacles();
    auto static_obstacles = advanced_obstacle_detector_->getStaticObstacles();
    
    // Enhanced logging with obstacle details
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[ENHANCED LIDAR] Tracked: %zu, Dynamic: %zu, Static: %zu obstacles", 
        tracked_obstacles.size(), dynamic_obstacles.size(), static_obstacles.size());
    
    if (!dynamic_obstacles.empty()) {
        for (const auto& obs : dynamic_obstacles) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "[DYNAMIC OBSTACLE] ID:%d at (%.2f,%.2f), vel:(%.2f,%.2f) m/s, size:%.2f", 
                obs.track_id, obs.x, obs.y, obs.velocity_x, obs.velocity_y, obs.size);
        }
    }
    
    // Keep old detector for compatibility
    current_obstacles_ = obstacle_detector_->detect_from_laser_scan(
        msg, vehicle_pos, vehicle_yaw);
}

void LatticePlanner::grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!odom_received_) return;
    
    // Store the current grid for track boundary detection
    {
        std::lock_guard<std::mutex> grid_lock(grid_mutex_);
        current_grid_ = msg;
    }
    
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    Point2D vehicle_pos;
    {
        std::lock_guard<std::mutex> state_lock(vehicle_state_mutex_);
        vehicle_pos = vehicle_position_;
    }
    
    // Count occupied cells for debugging
    int occupied_cells = 0;
    for (const auto& cell : msg->data) {
        if (cell > 50) occupied_cells++; // threshold for occupied
    }
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[OCCUPANCY GRID] Updated: %dx%d grid, %d occupied cells, resolution=%.3fm", 
        msg->info.width, msg->info.height, occupied_cells, msg->info.resolution);
    
    // Update advanced obstacle detector with occupancy grid
    advanced_obstacle_detector_->updateOccupancyGrid(msg);
    
    // Keep old detector for compatibility
    auto grid_obstacles = obstacle_detector_->detect_from_occupancy_grid(msg, vehicle_pos);
    
    // Merge with laser obstacles (simple approach - replace for now)
    // In practice, you might want to fuse multiple obstacle sources
    if (current_obstacles_.empty()) {
        current_obstacles_ = grid_obstacles;
    }
}

void LatticePlanner::planning_timer_callback() {
    if (!odom_received_) {
        return; // Skip planning until odometry is available
    }
    
    plan_paths();
}

void LatticePlanner::plan_paths() {
    Point2D vehicle_pos;
    double vehicle_yaw;
    double vehicle_vel;
    std::vector<Obstacle> obstacles;
    
    {
        std::lock_guard<std::mutex> state_lock(vehicle_state_mutex_);
        vehicle_pos = vehicle_position_;
        vehicle_yaw = vehicle_yaw_;
        vehicle_vel = vehicle_velocity_;
    }
    
    {
        std::lock_guard<std::mutex> obs_lock(obstacles_mutex_);
        obstacles = current_obstacles_;
    }
    
    // Generate path candidates
    std::vector<PathCandidate> candidates = path_generator_->generate_paths(
        vehicle_pos, vehicle_yaw, vehicle_vel, obstacles);
    
    if (candidates.empty()) {
        RCLCPP_WARN(this->get_logger(), "No valid path candidates generated");
        return;
    }
    
    
    // Select best path
    PathCandidate selected_path = select_best_path(candidates);
    
    // Publish selected path
    publish_selected_path(selected_path);

     // Debug: Log selected path info
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[SELECTED PATH] lateral_offset=%.3f, points=%zu, safe=%s, cost=%.3f", 
        selected_path.lateral_offset, selected_path.points.size(), 
        selected_path.is_safe ? "YES" : "NO", selected_path.cost);
    
    // Publish visualization
    publish_path_visualization(candidates, selected_path);
}

PathCandidate LatticePlanner::select_best_path(const std::vector<PathCandidate>& candidates) {
    if (candidates.empty()) {
        RCLCPP_WARN(this->get_logger(), "[DEBUG] No path candidates to select from");
        return PathCandidate();
    }
    
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Starting path selection with %zu candidates", candidates.size());
    
    // Convert PathCandidate to CandidateResult for advanced path selection
    std::vector<advanced::CandidateResult> advanced_candidates;
    
    Point2D vehicle_pos;
    {
        std::lock_guard<std::mutex> state_lock(vehicle_state_mutex_);
        vehicle_pos = vehicle_position_;
    }
    
    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& candidate = candidates[i];
        advanced::CandidateResult result;
        result.cost = candidate.cost;
        result.collided = !candidate.is_safe;
        
        // Use already computed out_of_track flag
        result.out_of_track = candidate.out_of_track;
        result.d_offset = candidate.lateral_offset; // Set the actual lateral offset
        
        // Convert PathPoint to geometry_msgs::Point
        for (const auto& point : candidate.points) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            result.path_points.push_back(p);
        }
        
        double original_cost = result.cost;
        
        // IMMEDIATE COLLISION DETECTION: 현재 경로만 체크 (미래 예측 완전 제거)
        if (!result.path_points.empty()) {
            // 현재 경로에서만 직접 충돌 체크 - 미래 연장 완전 제거
            bool direct_collision = advanced_obstacle_detector_->pathCollides(result.path_points);
            double base_obstacle_cost = advanced_obstacle_detector_->calculateOccupancyCost(result.path_points);
            
            // 충돌 시에만 페널티 적용 (미래 예측 제거로 현재 안전 경로 보호)
            if (direct_collision) {
                base_obstacle_cost += 100.0;  // 실제 충돌에만 큰 페널티
                RCLCPP_ERROR(this->get_logger(), "[IMMEDIATE COLLISION] Path %zu (offset=%.3f) has REAL collision NOW!", 
                           i, candidate.lateral_offset);
            }
            
            result.cost += base_obstacle_cost;
            result.collided = result.collided || direct_collision;
            
            // 상세 로깅 (미래 예측 제거)
            RCLCPP_INFO(this->get_logger(), "[IMMEDIATE CHECK] Path %zu: offset=%.3f, collision=%s, cost=%.3f", 
                       i, candidate.lateral_offset, 
                       direct_collision ? "YES" : "NO", 
                       base_obstacle_cost);
        }
        
        advanced_candidates.push_back(result);
    }
    
    // SIMPLE PATH SELECTION: 복잡한 로직 제거하고 단순하게
    double min_cost = std::numeric_limits<double>::max();
    size_t best_idx = 0;
    bool found_safe_path = false;
    
    RCLCPP_WARN(this->get_logger(), "[PATH SELECTION] Starting with %zu candidates", candidates.size());
    
    // 1단계: 안전한 경로들만 필터링
    std::vector<size_t> safe_indices;
    for (size_t i = 0; i < candidates.size(); ++i) {
        bool is_safe = candidates[i].is_safe && !advanced_candidates[i].collided;
        
        RCLCPP_WARN(this->get_logger(), "[PATH %zu] offset=%.3f, cost=%.3f, safe=%s, collided=%s -> %s", 
                   i, candidates[i].lateral_offset, advanced_candidates[i].cost, 
                   candidates[i].is_safe ? "YES" : "NO", 
                   advanced_candidates[i].collided ? "YES" : "NO",
                   is_safe ? "SAFE" : "UNSAFE");
        
        if (is_safe) {
            safe_indices.push_back(i);
        }
    }
    
    if (safe_indices.empty()) {
        RCLCPP_ERROR(this->get_logger(), "[EMERGENCY] No safe paths found! Using cost-based fallback");
        // 안전한 경로가 없으면 최소 비용 선택
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (advanced_candidates[i].cost < min_cost) {
                min_cost = advanced_candidates[i].cost;
                best_idx = i;
                found_safe_path = true;
            }
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "[SAFE PATHS] Found %zu safe paths", safe_indices.size());
        
        // 2단계: 안전한 경로들 중에서 레이스라인 우선, 그 다음 최소 비용
        bool found_raceline = false;
        
        // 레이스라인 우선 검색 (d ≈ 0)
        for (size_t idx : safe_indices) {
            if (std::abs(candidates[idx].lateral_offset) < 0.1) {  // raceline
                best_idx = idx;
                found_safe_path = true;
                found_raceline = true;
                RCLCPP_WARN(this->get_logger(), "[RACELINE SELECTED] Path %zu with offset %.3f", idx, candidates[idx].lateral_offset);
                break;
            }
        }
        
        // 레이스라인이 없으면 최소 비용 선택
        if (!found_raceline) {
            for (size_t idx : safe_indices) {
                if (advanced_candidates[idx].cost < min_cost) {
                    min_cost = advanced_candidates[idx].cost;
                    best_idx = idx;
                    found_safe_path = true;
                }
            }
            RCLCPP_WARN(this->get_logger(), "[COST SELECTED] Path %zu with cost %.3f", best_idx, min_cost);
        }
    }
    
    if (found_safe_path) {
        // 🚨 최종 안전성 재검증
        const auto& final_choice = candidates[best_idx];
        if (!final_choice.is_safe || (best_idx < advanced_candidates.size() && 
            (advanced_candidates[best_idx].collided || advanced_candidates[best_idx].out_of_track))) {
            
            RCLCPP_FATAL(this->get_logger(), 
                "🚨🚨🚨 [SAFETY VIOLATION] Final choice is UNSAFE! offset=%.3f, is_safe=%s", 
                final_choice.lateral_offset, final_choice.is_safe ? "true" : "false");
            
            // 다시 안전한 경로 찾기
            for (size_t i = 0; i < candidates.size(); ++i) {
                if (candidates[i].is_safe && i < advanced_candidates.size() && 
                    !advanced_candidates[i].collided && !advanced_candidates[i].out_of_track) {
                    RCLCPP_WARN(this->get_logger(), 
                        "🛡️ [SAFETY RECOVERY] Using verified safe path %zu: offset=%.3f", 
                        i, candidates[i].lateral_offset);
                    return candidates[i];
                }
            }
            
            RCLCPP_FATAL(this->get_logger(), "💀 [CRITICAL] NO SAFE PATHS FOUND!");
            return PathCandidate();  // 빈 경로 반환
        }
        
        RCLCPP_ERROR(this->get_logger(), 
            "✅ [FINAL SELECTION] *** VERIFIED SAFE PATH %zu: offset=%.3f, cost=%.3f ***", 
            best_idx, final_choice.lateral_offset, 
            best_idx < advanced_candidates.size() ? advanced_candidates[best_idx].cost : -1.0);
        
        return final_choice;
    }
    
    // If no safe path found, emergency selection
    RCLCPP_ERROR(this->get_logger(), "[EMERGENCY] No collision-free path found! Selecting best available path");
    min_cost = std::numeric_limits<double>::max();
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (advanced_candidates[i].cost < min_cost) {
            min_cost = advanced_candidates[i].cost;
            best_idx = i;
        }
    }
    
    RCLCPP_WARN(this->get_logger(), "[EMERGENCY] Selected path %zu: lateral_offset=%.3f, cost=%.3f", 
               best_idx, candidates[best_idx].lateral_offset, min_cost);
    return candidates[best_idx];
}

void LatticePlanner::publish_selected_path(const PathCandidate& path) {
    if (path.points.empty()) {
        RCLCPP_ERROR(this->get_logger(), "[PUBLISH ERROR] Empty path, not publishing anything!");
        return;
    }
    
    RCLCPP_ERROR(this->get_logger(), "[PUBLISH START] Publishing path with %zu points, lateral_offset=%.3f", 
                path.points.size(), path.lateral_offset);
    
    // Publish path as nav_msgs::Path (for path_follower)
    auto nav_path = convert_to_nav_path(path);
    path_pub_->publish(nav_path);
    
    // Publish path as PathWithVelocity (for velocity-aware control)
    auto velocity_path = convert_to_path_with_velocity(path);
    path_with_velocity_pub_->publish(velocity_path);
    
    // 첫 번째와 마지막 점 로깅
    if (nav_path.poses.size() >= 2) {
        const auto& first = nav_path.poses.front();
        const auto& last = nav_path.poses.back();
        RCLCPP_ERROR(this->get_logger(), 
            "[PUBLISH DETAILS] nav_path: %zu points, first=(%.2f,%.2f), last=(%.2f,%.2f)",
            nav_path.poses.size(), 
            first.pose.position.x, first.pose.position.y,
            last.pose.position.x, last.pose.position.y);
    }
    
    if (velocity_path.points.size() >= 2) {
        const auto& first_vel = velocity_path.points.front();
        const auto& last_vel = velocity_path.points.back();
        RCLCPP_ERROR(this->get_logger(), 
            "[PUBLISH DETAILS] velocity_path: %zu points, first=(%.2f,%.2f,v=%.2f), last=(%.2f,%.2f,v=%.2f)",
            velocity_path.points.size(), 
            first_vel.x, first_vel.y, first_vel.velocity,
            last_vel.x, last_vel.y, last_vel.velocity);
    }
    
    RCLCPP_ERROR(this->get_logger(), "[PUBLISH SUCCESS] Both topics published successfully!");
}

void LatticePlanner::publish_path_visualization(
    const std::vector<PathCandidate>& candidates,
    const PathCandidate& selected) {
    
    auto markers = create_path_markers(candidates, selected);
    marker_pub_->publish(markers);
}

void LatticePlanner::publish_reference_path() {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->get_clock()->now();
    
    for (const auto& point : reference_path_) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, point.heading);
        tf2::convert(q, pose.pose.orientation);
        
        path_msg.poses.push_back(pose);
    }
    
    ref_path_pub_->publish(path_msg);
}

void LatticePlanner::ref_path_timer_callback() {
    // Continuously publish reference path at 2 Hz
    publish_reference_path();
}


nav_msgs::msg::Path LatticePlanner::convert_to_nav_path(const PathCandidate& path) {
    nav_msgs::msg::Path nav_path;
    nav_path.header.frame_id = "map";
    nav_path.header.stamp = this->get_clock()->now();
    
    for (const auto& point : path.points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = nav_path.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, point.yaw);
        tf2::convert(q, pose.pose.orientation);
        
        nav_path.poses.push_back(pose);
    }
    
    return nav_path;
}

planning_custom_msgs::msg::PathWithVelocity LatticePlanner::convert_to_path_with_velocity(const PathCandidate& path) {
    planning_custom_msgs::msg::PathWithVelocity velocity_path;
    velocity_path.header.frame_id = "map";
    velocity_path.header.stamp = this->get_clock()->now();
    
    // Set path ID for tracking (could be based on path cost or lateral offset)
    velocity_path.path_id = static_cast<uint32_t>(std::hash<double>{}(path.lateral_offset) % 10000);
    
    // Calculate maximum velocity in path
    double max_vel = 0.0;
    for (const auto& point : path.points) {
        max_vel = std::max(max_vel, point.velocity);
        
        planning_custom_msgs::msg::PathPoint path_point;
        path_point.x = point.x;
        path_point.y = point.y;
        path_point.yaw = point.yaw;
        path_point.velocity = point.velocity;
        path_point.curvature = point.curvature;
        path_point.time_from_start = point.time;
        
        velocity_path.points.push_back(path_point);
    }
    
    velocity_path.max_velocity = max_vel;
    
    return velocity_path;
}

visualization_msgs::msg::MarkerArray LatticePlanner::create_path_markers(
    const std::vector<PathCandidate>& candidates,
    const PathCandidate& selected) {
    
    visualization_msgs::msg::MarkerArray markers;
    
    // Create markers for candidates
    for (size_t i = 0; i < candidates.size(); ++i) {
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "path_candidates";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.scale.x = 0.05;  // Line width
        
        // Color coding: green for safe, red for unsafe
        if (candidates[i].is_safe) {
            // Green for safe paths
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.7;
        } else {
            // Red for unsafe but in-track paths  
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.4;
        }
        
        for (const auto& point : candidates[i].points) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }
        
        markers.markers.push_back(marker);
    }
    
    // Highlight selected path
    if (!selected.points.empty()) {
        visualization_msgs::msg::Marker selected_marker;
        selected_marker.header.frame_id = "map";
        selected_marker.header.stamp = this->get_clock()->now();
        selected_marker.ns = "selected_path";
        selected_marker.id = 0;
        selected_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        selected_marker.action = visualization_msgs::msg::Marker::ADD;
        
        selected_marker.scale.x = 0.1;  // Thicker line
        
        // Blue for selected path
        selected_marker.color.r = 0.0;
        selected_marker.color.g = 0.0;
        selected_marker.color.b = 1.0;
        selected_marker.color.a = 1.0;
        
        for (const auto& point : selected.points) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.1;  // Slightly elevated
            selected_marker.points.push_back(p);
        }
        
        markers.markers.push_back(selected_marker);
    }
    
    return markers;
}

} // namespace lattice_planner_pkg
