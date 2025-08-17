#pragma once

#include "lattice_planner_pkg/core/types.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>

namespace lattice_planner_pkg {

class ObstacleDetector {
public:
    ObstacleDetector(const PlannerConfig& config);
    ~ObstacleDetector();
    
    // Detect obstacles from laser scan
    std::vector<Obstacle> detect_from_laser_scan(
        const sensor_msgs::msg::LaserScan::SharedPtr& scan,
        const Point2D& vehicle_position,
        double vehicle_yaw
    );
    
    // Detect obstacles from occupancy grid
    std::vector<Obstacle> detect_from_occupancy_grid(
        const nav_msgs::msg::OccupancyGrid::SharedPtr& grid,
        const Point2D& vehicle_position
    );
    
    // Filter obstacles by distance
    std::vector<Obstacle> filter_obstacles_by_distance(
        const std::vector<Obstacle>& obstacles,
        double max_distance
    );
    
    // Cluster nearby obstacles
    std::vector<Obstacle> cluster_obstacles(const std::vector<Obstacle>& obstacles);
    
    // Check if grid cell is occupied
    bool is_cell_occupied(const nav_msgs::msg::OccupancyGrid& grid, int x, int y);
    
    // World coordinates to grid coordinates
    bool world_to_grid(const nav_msgs::msg::OccupancyGrid& grid, 
                      const Point2D& world_point, int& x, int& y);

private:
    PlannerConfig config_;
    
    // Convert laser scan point to global coordinate
    Point2D laser_point_to_global(
        double range, double angle,
        const Point2D& vehicle_position,
        double vehicle_yaw
    );
    
    // Grid coordinates to world coordinates
    Point2D grid_to_world(const nav_msgs::msg::OccupancyGrid& grid, int x, int y);
};

} // namespace lattice_planner_pkg