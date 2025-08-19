#include "lattice_planner_pkg/obstacle_subscriber.hpp"
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <algorithm>
#include <limits>

namespace lattice_planner_pkg {

ObstacleSubscriber::ObstacleSubscriber() 
    : Node("obstacle_subscriber"),
      obstacles_received_(false),
      map_received_(false), 
      markers_received_(false)
{
    // Declare parameters for topic names
    this->declare_parameter("obstacles_topic", "/detected_obstacles");
    this->declare_parameter("updated_map_topic", "/updated_map");
    this->declare_parameter("markers_topic", "/obstacle_markers");
    
    // Get parameter values
    obstacles_topic_ = this->get_parameter("obstacles_topic").as_string();
    updated_map_topic_ = this->get_parameter("updated_map_topic").as_string();
    markers_topic_ = this->get_parameter("markers_topic").as_string();
    
    // Create subscribers with appropriate QoS
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // Create all subscribers with obstacle_detection_pkg messages available
    RCLCPP_INFO(this->get_logger(), 
                "Creating subscribers with obstacle_detection_pkg messages");
    
    // Obstacles subscriber
    obstacles_sub_ = this->create_subscription<obstacle_detection_pkg::msg::ObstacleArray>(
        obstacles_topic_, qos,
        std::bind(&ObstacleSubscriber::obstaclesCallback, this, std::placeholders::_1));
    
    // Updated map subscriber  
    updated_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        updated_map_topic_, qos,
        std::bind(&ObstacleSubscriber::updatedMapCallback, this, std::placeholders::_1));
    
    // Visualization markers subscriber
    markers_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        markers_topic_, qos,
        std::bind(&ObstacleSubscriber::markersCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Obstacle subscriber initialized");
    RCLCPP_INFO(this->get_logger(), "Listening to:");
    RCLCPP_INFO(this->get_logger(), "  - Obstacles: %s", obstacles_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Updated map: %s", updated_map_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Markers: %s", markers_topic_.c_str());
}

void ObstacleSubscriber::obstaclesCallback(const obstacle_detection_pkg::msg::ObstacleArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    latest_obstacles_ = *msg;
    obstacles_received_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), "Received obstacle array with %d obstacles", 
                 msg->total_obstacles);
}

void ObstacleSubscriber::updatedMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_updated_map_ = *msg;
    map_received_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), "Received updated occupancy grid: %dx%d", 
                 msg->info.width, msg->info.height);
}

void ObstacleSubscriber::markersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(markers_mutex_);
    latest_markers_ = *msg;
    markers_received_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), "Received %zu visualization markers", 
                 msg->markers.size());
}

const obstacle_detection_pkg::msg::ObstacleArray& ObstacleSubscriber::getLatestObstacles() const
{
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    return latest_obstacles_;
}

const nav_msgs::msg::OccupancyGrid& ObstacleSubscriber::getUpdatedMap() const
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    return latest_updated_map_;
}

size_t ObstacleSubscriber::getObstacleCount() const
{
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    if (!obstacles_received_) {
        return 0;
    }
    return latest_obstacles_.total_obstacles;
}

double ObstacleSubscriber::getClosestObstacleDistance() const
{
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    if (!obstacles_received_ || latest_obstacles_.obstacles.empty()) {
        return std::numeric_limits<double>::max();
    }
    
    double min_distance = std::numeric_limits<double>::max();
    for (const auto& obstacle : latest_obstacles_.obstacles) {
        min_distance = std::min(min_distance, obstacle.distance);
    }
    
    return min_distance;
}

obstacle_detection_pkg::msg::Obstacle ObstacleSubscriber::getClosestObstacle() const
{
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    if (!obstacles_received_ || latest_obstacles_.obstacles.empty()) {
        // Return empty obstacle
        obstacle_detection_pkg::msg::Obstacle empty_obstacle;
        empty_obstacle.distance = std::numeric_limits<double>::max();
        return empty_obstacle;
    }
    
    auto closest_it = std::min_element(
        latest_obstacles_.obstacles.begin(), 
        latest_obstacles_.obstacles.end(),
        [](const auto& a, const auto& b) {
            return a.distance < b.distance;
        });
    
    return *closest_it;
}

} // namespace lattice_planner_pkg