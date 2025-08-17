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
    
    // Initialize new obstacle detector and path selector
    advanced::ObstacleDetectionConfig obs_config;
    obs_config.max_detection_range = 8.0;
    obs_config.forward_distance_max = 6.0;
    obs_config.lateral_distance_max = 3.0;
    obs_config.proximity_search_radius = 3.0;  // 3m 범위로 확장된 근접 탐지
    advanced_obstacle_detector_ = std::make_unique<advanced::ObstacleDetector>(obs_config);
    
    advanced::PathSelectionConfig sel_config;
    sel_config.commit_min_progress = 1.0;
    sel_config.commit_min_time_sec = 0.8;
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
    
    // Use new advanced obstacle detector
    advanced_obstacle_detector_->detectObstaclesFromScan(
        msg, vehicle_pos.x, vehicle_pos.y, vehicle_yaw, this->get_clock());
    
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
        
        // Enhanced cost calculation with obstacle detection
        if (!result.path_points.empty()) {
            double obstacle_cost = advanced_obstacle_detector_->calculateOccupancyCost(result.path_points);
            bool path_collides = advanced_obstacle_detector_->pathCollides(result.path_points);
            
            result.cost += obstacle_cost;
            result.collided = result.collided || path_collides;
            
            // Enhanced logging for collision detection
            RCLCPP_INFO(this->get_logger(), "[COLLISION CHECK] Candidate %zu: lateral_offset=%.3f, occupancy_cost=%.3f, path_collides=%s, total_collided=%s", 
                       i, candidate.lateral_offset, obstacle_cost, 
                       path_collides ? "YES" : "NO",
                       result.collided ? "YES" : "NO");
        }
        
        advanced_candidates.push_back(result);
    }
    
    // OBSTACLE-AWARE RACELINE SELECTION: Prefer longer/faster paths when obstacles detected
    bool obstacles_ahead = advanced_obstacle_detector_->hasObstacles(vehicle_pos.x, vehicle_pos.y);
    
    size_t raceline_candidate = std::numeric_limits<size_t>::max();
    bool found_safe_raceline = false;
    
    // If obstacles detected, prefer higher velocity paths for better look-ahead
    if (obstacles_ahead) {
        RCLCPP_WARN(this->get_logger(), "[OBSTACLE AHEAD] Obstacles detected, prioritizing longer/faster paths");
        
        // Find the longest/fastest safe raceline path
        double best_path_length = 0.0;
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (std::abs(candidates[i].lateral_offset) < 0.05 && // raceline candidate
                candidates[i].is_safe && !advanced_candidates[i].collided) {
                
                // Calculate path length
                double path_length = 0.0;
                if (!candidates[i].points.empty()) {
                    const auto& last_point = candidates[i].points.back();
                    const auto& first_point = candidates[i].points.front();
                    path_length = std::sqrt(
                        std::pow(last_point.x - first_point.x, 2) + 
                        std::pow(last_point.y - first_point.y, 2));
                }
                
                if (path_length > best_path_length) {
                    best_path_length = path_length;
                    raceline_candidate = i;
                    found_safe_raceline = true;
                    RCLCPP_INFO(this->get_logger(), "[LONG PATH] Selected longer raceline: candidate %zu, length=%.2fm", i, path_length);
                }
            }
        }
    } else {
        // No obstacles: use standard raceline selection
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (std::abs(candidates[i].lateral_offset) < 0.05) { // raceline candidate
            RCLCPP_INFO(this->get_logger(), "[RACELINE DEBUG] Checking raceline candidate %zu:", i);
            RCLCPP_INFO(this->get_logger(), "  - is_safe: %s", candidates[i].is_safe ? "true" : "false");
            RCLCPP_INFO(this->get_logger(), "  - advanced_collided: %s", advanced_candidates[i].collided ? "true" : "false");
            RCLCPP_INFO(this->get_logger(), "  - lateral_offset: %.6f", candidates[i].lateral_offset);
            
            // Additional check: ensure raceline path has reasonable curvature for corners
            double max_curvature = 0.0;
            for (const auto& point : candidates[i].points) {
                max_curvature = std::max(max_curvature, std::abs(point.curvature));
            }
            RCLCPP_INFO(this->get_logger(), "  - max_curvature: %.3f", max_curvature);
            
            // ONLY select raceline if it's collision-free (check BOTH old and new collision detection)
            bool is_collision_free = candidates[i].is_safe && !advanced_candidates[i].collided;
            bool curvature_ok = max_curvature < 0.8;
            
            RCLCPP_INFO(this->get_logger(), "  - is_collision_free: %s", is_collision_free ? "true" : "false");
            RCLCPP_INFO(this->get_logger(), "  - curvature_ok: %s", curvature_ok ? "true" : "false");
            
            if (is_collision_free && curvature_ok) {
                raceline_candidate = i;
                found_safe_raceline = true;
                RCLCPP_INFO(this->get_logger(), "[SAFE RACELINE] Safe raceline found: candidate %zu", raceline_candidate);
                break;
            } else {
                RCLCPP_WARN(this->get_logger(), "[COLLISION DETECTED] Raceline candidate %zu has collision (is_safe=%s, advanced_collided=%s), skipping", 
                           i, candidates[i].is_safe ? "true" : "false", 
                           advanced_candidates[i].collided ? "true" : "false");
            }
        }
    }
    
    // Only use raceline if it's safe
    if (found_safe_raceline && raceline_candidate != std::numeric_limits<size_t>::max()) {
        RCLCPP_INFO(this->get_logger(), "[SAFE RACELINE] Using safe raceline: candidate %zu", raceline_candidate);
        return candidates[raceline_candidate];
    } else {
        RCLCPP_WARN(this->get_logger(), "[NO SAFE RACELINE] No collision-free raceline available, using path selector");
    }
    
    // Use advanced path selector
    bool has_obstacles = advanced_obstacle_detector_->hasObstacles(vehicle_pos.x, vehicle_pos.y);
    double current_s = 0.0; // TODO: calculate current arc length position
    
    RCLCPP_INFO(this->get_logger(), "[DEBUG] has_obstacles=%s, vehicle_pos=(%.2f, %.2f)", 
               has_obstacles ? "true" : "false", vehicle_pos.x, vehicle_pos.y);
    
    // SAFETY OVERRIDE: If obstacles detected, force has_obstacles=false to prevent raceline recovery
    bool safe_has_obstacles = false; // Disable raceline recovery by forcing no obstacles
    RCLCPP_WARN(this->get_logger(), "[SAFETY OVERRIDE] Disabling raceline recovery to prioritize safety");
    
    auto* selected = path_selector_->selectOptimalPath(
        advanced_candidates, safe_has_obstacles, current_s, this->get_clock());
    
    if (selected) {
        // SAFETY CHECK: Reject selected path if it's raceline with collision
        bool is_selected_raceline = std::abs(selected->d_offset) < 0.05;
        if (is_selected_raceline && selected->collided) {
            RCLCPP_ERROR(this->get_logger(), "[SAFETY OVERRIDE] Path selector chose collision raceline, rejecting and using fallback");
            selected = nullptr; // Force fallback selection
        } else {
            path_selector_->updateCommitState(selected, current_s, this->get_clock());
            
            // Find corresponding original candidate
            for (size_t i = 0; i < candidates.size(); ++i) {
                if (&advanced_candidates[i] == selected) {
                    RCLCPP_INFO(this->get_logger(), "[DEBUG] Selected advanced candidate %zu with cost=%.3f, collided=%s", 
                               i, selected->cost, selected->collided ? "true" : "false");
                    return candidates[i];
                }
            }
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "[DEBUG] Advanced path selector returned null, falling back to simple selection");
    }
    
    // Fallback to collision-aware cost-based selection
    double min_cost = std::numeric_limits<double>::max();
    size_t best_idx = 0;
    
    // First priority: Find collision-free paths only
    for (size_t i = 0; i < candidates.size(); ++i) {
        bool is_collision_free = candidates[i].is_safe && !advanced_candidates[i].collided;
        if (is_collision_free && candidates[i].cost < min_cost) {
            min_cost = candidates[i].cost;
            best_idx = i;
        }
    }
    
    // If no collision-free path found, emergency selection (prefer safest path)
    if (min_cost == std::numeric_limits<double>::max()) {
        RCLCPP_ERROR(this->get_logger(), "[EMERGENCY] No collision-free path found! Selecting safest available path");
        min_cost = std::numeric_limits<double>::max();
        for (size_t i = 0; i < candidates.size(); ++i) {
            // Choose path with least collision risk (prioritize is_safe over occupancy collision)
            double emergency_cost = candidates[i].cost;
            if (advanced_candidates[i].collided) emergency_cost += 10000.0; // Heavy penalty for occupancy collision
            if (!candidates[i].is_safe) emergency_cost += 5000.0; // Penalty for other collision
            
            if (emergency_cost < min_cost) {
                min_cost = emergency_cost;
                best_idx = i;
            }
        }
        RCLCPP_ERROR(this->get_logger(), "[EMERGENCY] Selected path %zu with collision risk", best_idx);
    } else {
        RCLCPP_INFO(this->get_logger(), "[FALLBACK] Selected collision-free path %zu", best_idx);
    }
    
    return candidates[best_idx];
}

void LatticePlanner::publish_selected_path(const PathCandidate& path) {
    if (path.points.empty()) return;
    
    // Publish path as nav_msgs::Path (for backward compatibility)
    auto nav_path = convert_to_nav_path(path);
    path_pub_->publish(nav_path);
    
    // Publish path as PathWithVelocity (for velocity-aware path following)
    auto velocity_path = convert_to_path_with_velocity(path);
    path_with_velocity_pub_->publish(velocity_path);
    
    // Log velocity path information for debugging
    if (!velocity_path.points.empty()) {
        double avg_velocity = 0.0;
        for (const auto& point : velocity_path.points) {
            avg_velocity += point.velocity;
        }
        avg_velocity /= velocity_path.points.size();
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[VELOCITY PUBLISH] PathWithVelocity published: %zu points, avg_vel=%.2f, max_vel=%.2f", 
            velocity_path.points.size(), avg_velocity, velocity_path.max_velocity);
    }
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
