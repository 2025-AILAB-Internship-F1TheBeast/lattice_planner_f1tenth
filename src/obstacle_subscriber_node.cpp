#include "lattice_planner_pkg/obstacle_subscriber.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    // Create obstacle subscriber node
    auto obstacle_subscriber = std::make_shared<lattice_planner_pkg::ObstacleSubscriber>();
    
    // Create a timer to periodically print obstacle information
    auto timer = obstacle_subscriber->create_wall_timer(
        1s, [obstacle_subscriber]() {
            if (obstacle_subscriber->hasObstacleData()) {
                size_t count = obstacle_subscriber->getObstacleCount();
                double closest_distance = obstacle_subscriber->getClosestObstacleDistance();
                
                RCLCPP_INFO(obstacle_subscriber->get_logger(),
                           "Detected %zu obstacles, closest at %.2f meters",
                           count, closest_distance);
                           
                if (count > 0) {
                    auto closest = obstacle_subscriber->getClosestObstacle();
                    RCLCPP_INFO(obstacle_subscriber->get_logger(),
                               "Closest obstacle at (%.2f, %.2f), size: %.2f",
                               closest.position.x, closest.position.y, closest.size);
                }
            } else {
                RCLCPP_WARN(obstacle_subscriber->get_logger(),
                           "No obstacle data received yet");
            }
            
            if (obstacle_subscriber->hasMapData()) {
                const auto& map = obstacle_subscriber->getUpdatedMap();
                RCLCPP_DEBUG(obstacle_subscriber->get_logger(),
                            "Updated map: %dx%d, resolution: %.3f",
                            map.info.width, map.info.height, map.info.resolution);
            }
        });
    
    RCLCPP_INFO(obstacle_subscriber->get_logger(), "Obstacle subscriber node started");
    
    // Spin the node
    rclcpp::spin(obstacle_subscriber);
    
    rclcpp::shutdown();
    return 0;
}