#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <memory>
#include <vector>
#include <mutex>

// Real obstacle_detection_pkg message includes
#include <obstacle_detection_pkg/msg/obstacle.hpp>
#include <obstacle_detection_pkg/msg/obstacle_array.hpp>

namespace lattice_planner_pkg {

class ObstacleSubscriber : public rclcpp::Node {
public:
    ObstacleSubscriber();
    
    // Getter methods for latest data
    const obstacle_detection_pkg::msg::ObstacleArray& getLatestObstacles() const;
    const nav_msgs::msg::OccupancyGrid& getUpdatedMap() const;
    
    // Check if data is available
    bool hasObstacleData() const { return obstacles_received_; }
    bool hasMapData() const { return map_received_; }
    
    // Get obstacle count and closest obstacle info
    size_t getObstacleCount() const;
    double getClosestObstacleDistance() const;
    obstacle_detection_pkg::msg::Obstacle getClosestObstacle() const;

private:
    // ROS2 subscribers
    rclcpp::Subscription<obstacle_detection_pkg::msg::ObstacleArray>::SharedPtr obstacles_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr updated_map_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr markers_sub_;
    
    // Callback functions
    void obstaclesCallback(const obstacle_detection_pkg::msg::ObstacleArray::SharedPtr msg);
    void updatedMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void markersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    
    // Latest received data
    obstacle_detection_pkg::msg::ObstacleArray latest_obstacles_;
    nav_msgs::msg::OccupancyGrid latest_updated_map_;
    visualization_msgs::msg::MarkerArray latest_markers_;
    
    // Data availability flags
    bool obstacles_received_;
    bool map_received_;
    bool markers_received_;
    
    // Thread safety
    mutable std::mutex obstacles_mutex_;
    mutable std::mutex map_mutex_;
    mutable std::mutex markers_mutex_;
    
    // ROS parameters
    std::string obstacles_topic_;
    std::string updated_map_topic_;
    std::string markers_topic_;
};

} // namespace lattice_planner_pkg