#include "lattice_planner_pkg/core/obstacle_detector.hpp"
#include <cmath>
#include <algorithm>

namespace lattice_planner_pkg {

ObstacleDetector::ObstacleDetector(const PlannerConfig& config) 
    : config_(config) {
}

ObstacleDetector::~ObstacleDetector() {
}

std::vector<Obstacle> ObstacleDetector::detect_from_laser_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan,
    const Point2D& vehicle_position,
    double vehicle_yaw) {
    
    std::vector<Obstacle> obstacles;
    
    if (!scan) {
        return obstacles;
    }
    
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double range = scan->ranges[i];
        
        // Check if range is valid
        if (range < scan->range_min || range > scan->range_max || 
            std::isnan(range) || std::isinf(range)) {
            continue;
        }
        
        // Skip if too far
        if (range > config_.obstacle_detection_range) {
            continue;
        }
        
        // Calculate angle
        double angle = scan->angle_min + i * scan->angle_increment;
        
        // Convert to global coordinates
        Point2D obstacle_pos = laser_point_to_global(range, angle, vehicle_position, vehicle_yaw);
        
        // Create obstacle
        Obstacle obstacle;
        obstacle.x = obstacle_pos.x;
        obstacle.y = obstacle_pos.y;
        obstacle.distance = range;
        obstacle.angle = angle;
        obstacle.timestamp = rclcpp::Clock().now();
        
        obstacles.push_back(obstacle);
    }
    
    // Cluster nearby obstacles
    obstacles = cluster_obstacles(obstacles);
    
    return obstacles;
}

std::vector<Obstacle> ObstacleDetector::detect_from_occupancy_grid(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& grid,
    const Point2D& vehicle_position) {
    
    std::vector<Obstacle> obstacles;
    
    if (!grid) {
        return obstacles;
    }
    
    // Reduced search radius for better performance
    int search_radius = static_cast<int>(std::min(5.0, config_.obstacle_detection_range) / grid->info.resolution);
    
    int center_x, center_y;
    if (!world_to_grid(*grid, vehicle_position, center_x, center_y)) {
        return obstacles;
    }
    
    for (int dx = -search_radius; dx <= search_radius; ++dx) {
        for (int dy = -search_radius; dy <= search_radius; ++dy) {
            int x = center_x + dx;
            int y = center_y + dy;
            
            if (is_cell_occupied(*grid, x, y)) {
                Point2D obstacle_pos = grid_to_world(*grid, x, y);
                
                // Calculate distance
                double distance = std::sqrt(
                    std::pow(obstacle_pos.x - vehicle_position.x, 2) +
                    std::pow(obstacle_pos.y - vehicle_position.y, 2)
                );
                
                if (distance <= config_.obstacle_detection_range) {
                    Obstacle obstacle;
                    obstacle.x = obstacle_pos.x;
                    obstacle.y = obstacle_pos.y;
                    obstacle.distance = distance;
                    obstacle.angle = std::atan2(
                        obstacle_pos.y - vehicle_position.y,
                        obstacle_pos.x - vehicle_position.x
                    );
                    obstacle.timestamp = rclcpp::Clock().now();
                    
                    obstacles.push_back(obstacle);
                }
            }
        }
    }
    
    // Cluster nearby obstacles
    obstacles = cluster_obstacles(obstacles);
    
    return obstacles;
}

std::vector<Obstacle> ObstacleDetector::filter_obstacles_by_distance(
    const std::vector<Obstacle>& obstacles,
    double max_distance) {
    
    std::vector<Obstacle> filtered;
    
    for (const auto& obstacle : obstacles) {
        if (obstacle.distance <= max_distance) {
            filtered.push_back(obstacle);
        }
    }
    
    return filtered;
}

std::vector<Obstacle> ObstacleDetector::cluster_obstacles(const std::vector<Obstacle>& obstacles) {
    if (obstacles.empty()) {
        return obstacles;
    }
    
    std::vector<Obstacle> clustered;
    std::vector<bool> used(obstacles.size(), false);
    double cluster_threshold = 0.5;  // 0.5 meters
    
    for (size_t i = 0; i < obstacles.size(); ++i) {
        if (used[i]) continue;
        
        // Start new cluster
        Obstacle cluster_center = obstacles[i];
        std::vector<Obstacle> cluster_members;
        cluster_members.push_back(obstacles[i]);
        used[i] = true;
        
        // Find nearby obstacles
        for (size_t j = i + 1; j < obstacles.size(); ++j) {
            if (used[j]) continue;
            
            double distance = std::sqrt(
                std::pow(obstacles[i].x - obstacles[j].x, 2) +
                std::pow(obstacles[i].y - obstacles[j].y, 2)
            );
            
            if (distance < cluster_threshold) {
                cluster_members.push_back(obstacles[j]);
                used[j] = true;
            }
        }
        
        // Calculate cluster center
        if (cluster_members.size() > 1) {
            double sum_x = 0.0, sum_y = 0.0;
            for (const auto& member : cluster_members) {
                sum_x += member.x;
                sum_y += member.y;
            }
            
            cluster_center.x = sum_x / cluster_members.size();
            cluster_center.y = sum_y / cluster_members.size();
            
            // Recalculate distance and angle
            cluster_center.distance = std::sqrt(
                cluster_center.x * cluster_center.x +
                cluster_center.y * cluster_center.y
            );
            cluster_center.angle = std::atan2(cluster_center.y, cluster_center.x);
        }
        
        clustered.push_back(cluster_center);
    }
    
    return clustered;
}

Point2D ObstacleDetector::laser_point_to_global(
    double range, double angle,
    const Point2D& vehicle_position,
    double vehicle_yaw) {
    
    // Convert laser scan point to vehicle frame
    double local_x = range * std::cos(angle);
    double local_y = range * std::sin(angle);
    
    // Transform to global frame
    double cos_yaw = std::cos(vehicle_yaw);
    double sin_yaw = std::sin(vehicle_yaw);
    
    Point2D global_point;
    global_point.x = vehicle_position.x + local_x * cos_yaw - local_y * sin_yaw;
    global_point.y = vehicle_position.y + local_x * sin_yaw + local_y * cos_yaw;
    
    return global_point;
}

bool ObstacleDetector::is_cell_occupied(const nav_msgs::msg::OccupancyGrid& grid, int x, int y) {
    if (x < 0 || x >= static_cast<int>(grid.info.width) ||
        y < 0 || y >= static_cast<int>(grid.info.height)) {
        return false;
    }
    
    int index = y * grid.info.width + x;
    if (index < 0 || index >= static_cast<int>(grid.data.size())) {
        return false;
    }
    
    int8_t occupancy_value = grid.data[index];
    
    // Consider cells with occupancy > 50 as occupied
    return occupancy_value > 50;
}

Point2D ObstacleDetector::grid_to_world(const nav_msgs::msg::OccupancyGrid& grid, int x, int y) {
    Point2D world_point;
    
    world_point.x = grid.info.origin.position.x + (x + 0.5) * grid.info.resolution;
    world_point.y = grid.info.origin.position.y + (y + 0.5) * grid.info.resolution;
    
    return world_point;
}

bool ObstacleDetector::world_to_grid(const nav_msgs::msg::OccupancyGrid& grid, 
                                    const Point2D& world_point, int& x, int& y) {
    x = static_cast<int>((world_point.x - grid.info.origin.position.x) / grid.info.resolution);
    y = static_cast<int>((world_point.y - grid.info.origin.position.y) / grid.info.resolution);
    
    return (x >= 0 && x < static_cast<int>(grid.info.width) &&
            y >= 0 && y < static_cast<int>(grid.info.height));
}

} // namespace lattice_planner_pkg