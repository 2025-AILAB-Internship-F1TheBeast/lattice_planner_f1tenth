#include "lattice_planner_pkg/planner/lattice_planner.hpp"
#include "planning_custom_msgs/msg/path_point_array.hpp"
#include "planning_custom_msgs/msg/path_with_velocity.hpp"
#include "lattice_planner_pkg/obstacle_detector.hpp"
#include "lattice_planner_pkg/path_selector.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <chrono>

// Local helpers for inflated occupancy checks used by planner and visualization
namespace {
// Inline occupancy inflation radius (meters) used by simple grid checks in planner/viz
// Set from ROS parameter in initialize(); default 0.3 m
static double g_inline_inflation_radius = 0.3;

// Check if any cell within inflation radius around (x,y) is occupied
inline bool isOccupiedInflated(const nav_msgs::msg::OccupancyGrid &grid,
                               double x, double y,
                               double inflation_radius_m,
                               int occupancy_threshold = 75,
                               bool treat_unknown_as_occupied = false) {
    const int w = static_cast<int>(grid.info.width);
    const int h = static_cast<int>(grid.info.height);
    const double res = grid.info.resolution;
    const double ox = grid.info.origin.position.x;
    const double oy = grid.info.origin.position.y;

    // Map coords of the query point
    const int mx = static_cast<int>(std::floor((x - ox) / res));
    const int my = static_cast<int>(std::floor((y - oy) / res));
    if (mx < 0 || my < 0 || mx >= w || my >= h) {
        // Caller decides how to handle OOB
        return false;
    }

    const int r_cells = std::max(1, static_cast<int>(std::ceil(inflation_radius_m / res)));
    const int r2 = r_cells * r_cells;
    for (int dy = -r_cells; dy <= r_cells; ++dy) {
        for (int dx = -r_cells; dx <= r_cells; ++dx) {
            if (dx*dx + dy*dy > r2) continue; // inside disk only
            const int nx = mx + dx;
            const int ny = my + dy;
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue; // skip OOB neighbors
            const int idx = ny * w + nx;
            if (idx < 0 || idx >= static_cast<int>(grid.data.size())) continue;
            const int8_t v = grid.data[idx];
            if (v >= occupancy_threshold) return true;
            if (v < 0 && treat_unknown_as_occupied) return true; // unknown cell
        }
    }
    return false;
}
} // namespace

namespace lattice_planner_pkg {

LatticePlanner::LatticePlanner() 
    : Node("lattice_planner"),
      vehicle_yaw_(0.0),
      vehicle_velocity_(0.0),
      odom_received_(false),
      last_selected_offset_(0.0),
      last_path_change_time_(this->get_clock()->now()) {
    
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
    
    // Load configuration parameters - YAML will override these defaults
    this->declare_parameter("reference_path_file", std::string(""));
    this->declare_parameter("path_resolution", 0.1);
    this->declare_parameter("lateral_step", 0.3);
    this->declare_parameter("max_lateral_offset", 1.2);
    this->declare_parameter("planning_horizon", 1.0);
    this->declare_parameter("dt", 0.1);
    this->declare_parameter("max_velocity", 5.0);
    this->declare_parameter("planning_frequency", 40.0);
    this->declare_parameter("max_curvature", 1.0);
    this->declare_parameter("lateral_cost_weight", 5.0);
    this->declare_parameter("curvature_cost_weight", 0.05);
    this->declare_parameter("longitudinal_cost_weight", 0.02);
    this->declare_parameter("obstacle_cost_weight", 10.0);
    this->declare_parameter("obstacle_existence_weight", 3.0);
    this->declare_parameter("unknown_area_weight", 1.0);
    this->declare_parameter("obstacle_distance_weight", 2.0);
    this->declare_parameter("collision_radius", 0.35);
    this->declare_parameter("occupancy_grid_topic", std::string("/map"));
    this->declare_parameter("occupancy_threshold", 75);
    this->declare_parameter("safety_margin", 0.15);
    this->declare_parameter("obstacle_detection_range", 8.0);
    
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
    config_.collision_radius = this->get_parameter("collision_radius").as_double();
    config_.safety_margin = this->get_parameter("safety_margin").as_double();
    config_.obstacle_detection_range = this->get_parameter("obstacle_detection_range").as_double();

    // Declare inline occupancy parameter
    this->declare_parameter("inline_occupancy_inflation_radius", 0.3);
    g_inline_inflation_radius = this->get_parameter("inline_occupancy_inflation_radius").as_double();
    
    // Get occupancy threshold for use in other parts
    int occupancy_threshold = this->get_parameter("occupancy_threshold").as_int();
    
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
    
    // Initialize enhanced LiDAR obstacle detector with YAML parameters
    advanced::ObstacleDetectionConfig obs_config;
    // Use YAML parameters instead of hardcoded values
    obs_config.max_detection_range = config_.obstacle_detection_range;
    obs_config.lateral_range = M_PI / 2.0;        // 180도 시야각 (고정)
    obs_config.forward_distance_max = config_.obstacle_detection_range * 0.75;  // 75% of max range
    obs_config.lateral_distance_max = config_.obstacle_detection_range * 0.5;   // 50% of max range
    
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
    
    // Occupancy grid 설정 - YAML 파라미터 사용
    obs_config.occupancy_threshold = occupancy_threshold;
    obs_config.occupancy_inflation_radius = g_inline_inflation_radius;
    obs_config.proximity_search_radius = config_.obstacle_detection_range * 0.5;
    obs_config.unknown_is_obstacle = false;      // 미지 영역은 장애물 아님
    
    advanced_obstacle_detector_ = std::make_unique<advanced::ObstacleDetector>(obs_config);
    
    // 안전한 경로 선택을 위한 향상된 설정
    advanced::PathSelectionConfig sel_config;
    sel_config.commit_min_progress = 1.0;
    sel_config.commit_min_time_sec = 0.8;
    
    // 장애물 회피 지속성 강화
    sel_config.path_length = 2.0;                       // 기본 커밋 길이 증가
    sel_config.obstacle_path_length_multiplier = 2.0;   // 장애물 상황에서 2배 연장
    sel_config.path_length_commit_mode = true;          // 경로 길이 기반 커밋 활성화
    
    // 더 안정적인 detour 설정
    sel_config.detour_return_clear_frames_threshold = 5; // 더 많은 프레임 확인 후 복귀
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
        "/odom", 10, 
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
    auto start = std::chrono::high_resolution_clock::now();
    
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
    
    // Minimal timing logging removed for performance
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
    
    // Reduced logging for performance
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "[LIDAR] %zu tracked, %zu dynamic obstacles", 
        tracked_obstacles.size(), dynamic_obstacles.size());
    
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
    
    // Minimal occupancy grid logging
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "[GRID] %dx%d, res=%.3fm", 
        msg->info.width, msg->info.height, msg->info.resolution);
    
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
    auto total_start = std::chrono::high_resolution_clock::now();
    
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
    
    // Generate path candidates with minimal timing overhead
    std::vector<PathCandidate> candidates = path_generator_->generate_paths(
        vehicle_pos, vehicle_yaw, vehicle_vel, obstacles);
    
    if (candidates.empty()) {
        RCLCPP_WARN(this->get_logger(), "No valid path candidates generated");
        return;
    }
    
    
    // Select and publish best path
    PathCandidate selected_path = select_best_path(candidates);
    publish_selected_path(selected_path);
    
    auto total_end = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(total_end - total_start);
    
    // Reduced logging for performance
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[PLAN] %.1fms | %zu candidates | offset=%.2f", 
        total_duration.count() / 1000.0, candidates.size(), selected_path.lateral_offset);
    
    // Publish visualization
    publish_path_visualization(candidates, selected_path);
}

PathCandidate LatticePlanner::select_best_path(const std::vector<PathCandidate>& candidates) {
    if (candidates.empty()) {
        return PathCandidate();
    }
    
    // Simplified path selection for performance
    size_t best_idx = 0;
    double min_cost = std::numeric_limits<double>::max();
    bool found_safe_path = false;
    
    // First pass: Find safe paths with lowest cost
    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& candidate = candidates[i];
        
        // Skip out-of-track paths
        if (candidate.out_of_track) continue;
        
        // Prioritize safe paths
        if (candidate.is_safe) {
            if (candidate.cost < min_cost || !found_safe_path) {
                min_cost = candidate.cost;
                best_idx = i;
                found_safe_path = true;
            }
        }
    }
    
    // If no safe path found, fallback to lowest cost path
    if (!found_safe_path) {
        min_cost = std::numeric_limits<double>::max();
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (!candidates[i].out_of_track && candidates[i].cost < min_cost) {
                min_cost = candidates[i].cost;
                best_idx = i;
            }
        }
    }
    
    // Apply hysteresis to prevent oscillation
    const auto& selected = candidates[best_idx];
    if (should_switch_path(selected.lateral_offset, last_selected_offset_)) {
        last_selected_offset_ = selected.lateral_offset;
        last_path_change_time_ = this->get_clock()->now();
    } else {
        // Try to keep previous path if still available and safe
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (std::abs(candidates[i].lateral_offset - last_selected_offset_) < 0.05 && 
                candidates[i].is_safe && !candidates[i].out_of_track) {
                return candidates[i];
            }
        }
        // Force switch if previous path no longer safe
        last_selected_offset_ = selected.lateral_offset;
        last_path_change_time_ = this->get_clock()->now();
    }
    
    return selected;
}

void LatticePlanner::publish_selected_path(const PathCandidate& path) {
    if (path.points.empty()) {
        return;
    }
    
    // Trim path to remove points behind vehicle
    PathCandidate trimmed = path;
    size_t start_idx = 0;
    {
        std::lock_guard<std::mutex> state_lock(vehicle_state_mutex_);
        double min_dist = std::numeric_limits<double>::max();
        size_t closest = 0;
        for (size_t i = 0; i < trimmed.points.size(); ++i) {
            double dx = trimmed.points[i].x - vehicle_position_.x;
            double dy = trimmed.points[i].y - vehicle_position_.y;
            double d = std::sqrt(dx*dx + dy*dy);
            if (d < min_dist) { min_dist = d; closest = i; }
        }
        
        // Find first point ahead of vehicle
        start_idx = closest;
        for (size_t i = closest; i < trimmed.points.size(); ++i) {
            double vx = std::cos(trimmed.points[i].yaw);
            double vy = std::sin(trimmed.points[i].yaw);
            double px = trimmed.points[i].x - vehicle_position_.x;
            double py = trimmed.points[i].y - vehicle_position_.y;
            double dot = vx*px + vy*py;
            double dist = std::sqrt(px*px + py*py);
            if (dot > 0.0 && dist > 0.2) { start_idx = i; break; }
        }
    }
    if (start_idx > 0 && start_idx < trimmed.points.size()) {
        trimmed.points.erase(trimmed.points.begin(), trimmed.points.begin() + static_cast<long>(start_idx));
    }

    // Ensure path starts at vehicle position
    {
        std::lock_guard<std::mutex> state_lock(vehicle_state_mutex_);
        if (!trimmed.points.empty()) {
            double dx0 = trimmed.points.front().x - vehicle_position_.x;
            double dy0 = trimmed.points.front().y - vehicle_position_.y;
            double d0 = std::sqrt(dx0*dx0 + dy0*dy0);
            if (d0 > 0.05) {
                CartesianPoint anchor;
                anchor.x = vehicle_position_.x;
                anchor.y = vehicle_position_.y;
                anchor.yaw = vehicle_yaw_;
                anchor.velocity = std::max(0.0, vehicle_velocity_);
                anchor.curvature = 0.0;
                anchor.time = 0.0;
                trimmed.points.insert(trimmed.points.begin(), anchor);
            }
        }
    }
    
    // Publish paths
    auto nav_path = convert_to_nav_path(trimmed);
    path_pub_->publish(nav_path);
    
    auto velocity_path = convert_to_path_with_velocity(trimmed);
    path_with_velocity_pub_->publish(velocity_path);
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
    static int last_candidate_count = 0;
    auto now = this->get_clock()->now();
    
    // Copy occupancy grid once for consistent checks during marker creation
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid_copy;
    {
        std::lock_guard<std::mutex> grid_lock(grid_mutex_);
        grid_copy = current_grid_;
    }
    
    // Create markers for candidates
    for (size_t i = 0; i < candidates.size(); ++i) {
        // Build geometry points for advanced checks
        std::vector<geometry_msgs::msg::Point> geom_pts;
        geom_pts.reserve(candidates[i].points.size());
        for (const auto& pt : candidates[i].points) {
            geometry_msgs::msg::Point p;
            p.x = pt.x; p.y = pt.y; p.z = 0.0;
            geom_pts.push_back(p);
        }
        
        // Use PathGenerator's safety assessment for visualization
        bool collided = !candidates[i].is_safe;  // Use basic is_safe from PathGenerator
        bool out_of_track = candidates[i].out_of_track;
        
        // Optional: Additional real-time checks for visualization only
        if (!geom_pts.empty() && grid_copy) {
            if (true) {  // Enable grid checks for visualization
                const auto &grid = *grid_copy;
                const int w = static_cast<int>(grid.info.width);
                const int h = static_cast<int>(grid.info.height);
                const double res = grid.info.resolution;
                const double ox = grid.info.origin.position.x;
                const double oy = grid.info.origin.position.y;
                
                // Check first few points only for visualization (to reduce computation)
                size_t check_points = std::min(static_cast<size_t>(10), geom_pts.size());
                for (size_t k = 0; k < check_points; ++k) {
                    const auto &p = geom_pts[k];
                    const int mx = static_cast<int>(std::floor((p.x - ox) / res));
                    const int my = static_cast<int>(std::floor((p.y - oy) / res));
                    if (mx < 0 || my < 0 || mx >= w || my >= h) { 
                        out_of_track = true; 
                        break; 
                    }
                    const int idx = my * w + mx;
                    if (idx < 0 || idx >= static_cast<int>(grid.data.size())) { 
                        out_of_track = true; 
                        break; 
                    }
                    // Use relaxed threshold for visualization
                    if (isOccupiedInflated(grid, p.x, p.y, g_inline_inflation_radius, 75, false)) { 
                        collided = true; 
                        break; 
                    }
                }
            }
        }
        const bool is_safe_now = (!collided && !out_of_track);

        // If this candidate goes out of track, don't visualize it at all.
        // Also publish a DELETE to remove any leftover marker with the same id from previous frames.
        if (out_of_track) {
            visualization_msgs::msg::Marker del;
            del.header.frame_id = "map";
            del.header.stamp = now;
            del.ns = "path_candidates";
            del.id = static_cast<int>(i);
            del.action = visualization_msgs::msg::Marker::DELETE;
            markers.markers.push_back(del);
            continue; // skip rendering this candidate
        }

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = now;
        marker.ns = "path_candidates";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.scale.x = 0.05;  // Line width
        
        // Color coding based on real-time checks: green=safe, red=collided
        if (is_safe_now) {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.7;
        } else {
            // Red for unsafe (collision). Out-of-track paths are not visualized above.
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.6;
        }
        
        marker.points = std::move(geom_pts);
        
        markers.markers.push_back(marker);
    }
    
    // Highlight selected path
    if (!selected.points.empty()) {
        visualization_msgs::msg::Marker selected_marker;
        selected_marker.header.frame_id = "map";
        selected_marker.header.stamp = now;
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
    } else {
        // 선택 경로가 없으면 이전 선택 마커 삭제
        visualization_msgs::msg::Marker del;
        del.header.frame_id = "map";
        del.header.stamp = now;
        del.ns = "selected_path";
        del.id = 0;
        del.action = visualization_msgs::msg::Marker::DELETE;
        markers.markers.push_back(del);
    }

    // 이전 프레임의 잔여 candidate 마커 삭제
    if (last_candidate_count > static_cast<int>(candidates.size())) {
        for (int id = static_cast<int>(candidates.size()); id < last_candidate_count; ++id) {
            visualization_msgs::msg::Marker del;
            del.header.frame_id = "map";
            del.header.stamp = now;
            del.ns = "path_candidates";
            del.id = id;
            del.action = visualization_msgs::msg::Marker::DELETE;
            markers.markers.push_back(del);
        }
    }
    last_candidate_count = static_cast<int>(candidates.size());
    
    return markers;
}

bool LatticePlanner::should_switch_path(double new_offset, double current_offset) {
    auto current_time = this->get_clock()->now();
    
    // Check if enough time has passed since last path change
    double time_since_change = (current_time - last_path_change_time_).seconds();
    if (time_since_change < PATH_CHANGE_COOLDOWN) {
        return false;
    }
    
    // Check if the offset change is significant enough
    double offset_diff = std::abs(new_offset - current_offset);
    return offset_diff > OFFSET_CHANGE_THRESHOLD;
}

} // namespace lattice_planner_pkg