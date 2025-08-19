#include "lattice_planner_pkg/core/frenet_coordinate.hpp"
#include <cmath>
#include <algorithm>

namespace lattice_planner_pkg {

FrenetCoordinate::FrenetCoordinate() 
    : total_length_(0.0), is_initialized_(false) {
}

FrenetCoordinate::~FrenetCoordinate() {
}

bool FrenetCoordinate::initialize(const std::vector<RefPoint>& reference_path) {
    if (reference_path.size() < 2) {
        return false;
    }
    
    reference_path_ = reference_path;
    total_length_ = reference_path_.back().s;
    is_initialized_ = true;
    
    return true;
}

FrenetPoint FrenetCoordinate::cartesian_to_frenet(const Point2D& cartesian_point) const {
    if (!is_initialized_) {
        return FrenetPoint();
    }
    
    // Find closest reference point
    int closest_idx = find_closest_reference_index(cartesian_point);
    
    if (closest_idx < 0 || closest_idx >= static_cast<int>(reference_path_.size())) {
        return FrenetPoint();
    }
    
    const RefPoint& ref_point = reference_path_[closest_idx];
    
    // Calculate relative position
    double dx = cartesian_point.x - ref_point.x;
    double dy = cartesian_point.y - ref_point.y;
    
    // Check if point is too far from reference path
    double distance = std::sqrt(dx*dx + dy*dy);
    
    // Transform to frenet frame
    double cos_theta = std::cos(ref_point.heading);
    double sin_theta = std::sin(ref_point.heading);
    
    FrenetPoint frenet_point;
    frenet_point.s = ref_point.s;
    
    // Calculate lateral distance
    double raw_d = -dx * sin_theta + dy * cos_theta;  // Lateral distance (left positive)
    
    // Debug logging removed for performance
    
    // If too far from reference path, limit lateral offset
    if (distance > 10.0) {
        // Clamp lateral offset to reasonable range
        frenet_point.d = std::max(-5.0, std::min(5.0, raw_d));
    } else {
        frenet_point.d = raw_d;
    }
    
    return frenet_point;
}

CartesianPoint FrenetCoordinate::frenet_to_cartesian(const FrenetPoint& frenet_point) const {
    if (!is_initialized_) {
        return CartesianPoint();
    }
    
    // Get reference point at s
    RefPoint ref_point = get_reference_point(frenet_point.s);
    
    // Transform to cartesian
    double cos_theta = std::cos(ref_point.heading);
    double sin_theta = std::sin(ref_point.heading);
    
    CartesianPoint cartesian_point;
    cartesian_point.x = ref_point.x - frenet_point.d * sin_theta;
    cartesian_point.y = ref_point.y + frenet_point.d * cos_theta;
    cartesian_point.yaw = ref_point.heading;
    cartesian_point.velocity = ref_point.velocity;
    cartesian_point.curvature = ref_point.curvature;
    
    return cartesian_point;
}

RefPoint FrenetCoordinate::get_reference_point(double s) const {
    if (!is_initialized_ || reference_path_.empty()) {
        return RefPoint();
    }
    
    // Handle wraparound for closed loop raceline
    if (s > total_length_) {
        s = std::fmod(s, total_length_);
    } else if (s < 0.0) {
        s = total_length_ + std::fmod(s, total_length_);
    }
    
    return interpolate_reference_point(s);
}

int FrenetCoordinate::find_closest_reference_index(const Point2D& point) const {
    if (!is_initialized_ || reference_path_.empty()) {
        return -1;
    }
    
    double min_distance = std::numeric_limits<double>::max();
    int closest_idx = 0;
    
    for (size_t i = 0; i < reference_path_.size(); ++i) {
        double distance = calculate_distance(point, Point2D(reference_path_[i].x, reference_path_[i].y));
        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = static_cast<int>(i);
        }
    }
    
    return closest_idx;
}

bool FrenetCoordinate::is_valid_s(double s) const {
    return is_initialized_ && s >= 0.0 && s <= total_length_;
}

double FrenetCoordinate::calculate_distance(const Point2D& p1, const Point2D& p2) const {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

double FrenetCoordinate::normalize_angle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

RefPoint FrenetCoordinate::interpolate_reference_point(double s) const {
    if (reference_path_.empty()) {
        return RefPoint();
    }
    
    // Find surrounding points
    size_t i = 0;
    for (i = 0; i < reference_path_.size() - 1; ++i) {
        if (reference_path_[i + 1].s > s) {
            break;
        }
    }
    
    if (i >= reference_path_.size() - 1) {
        // Handle wraparound at the end of the path
        const RefPoint& p1 = reference_path_.back();
        const RefPoint& p2 = reference_path_.front(); // Wrap to beginning
        
        double remaining_s = s - p1.s;
        double segment_length = total_length_ - p1.s; // Distance to end
        
        if (segment_length > 1e-6) {
            double ratio = remaining_s / segment_length;
            ratio = std::max(0.0, std::min(1.0, ratio));
            
            RefPoint interpolated;
            interpolated.x = p1.x + ratio * (p2.x - p1.x);
            interpolated.y = p1.y + ratio * (p2.y - p1.y);
            interpolated.s = s;
            interpolated.heading = p1.heading + ratio * normalize_angle(p2.heading - p1.heading);
            interpolated.curvature = p1.curvature + ratio * (p2.curvature - p1.curvature);
            interpolated.velocity = p1.velocity + ratio * (p2.velocity - p1.velocity);
            
            return interpolated;
        }
        
        return reference_path_.back();
    }
    
    // Linear interpolation
    const RefPoint& p1 = reference_path_[i];
    const RefPoint& p2 = reference_path_[i + 1];
    
    double ratio = (s - p1.s) / (p2.s - p1.s);
    ratio = std::max(0.0, std::min(1.0, ratio));
    
    RefPoint interpolated;
    interpolated.x = p1.x + ratio * (p2.x - p1.x);
    interpolated.y = p1.y + ratio * (p2.y - p1.y);
    interpolated.s = s;
    interpolated.heading = p1.heading + ratio * normalize_angle(p2.heading - p1.heading);
    interpolated.curvature = p1.curvature + ratio * (p2.curvature - p1.curvature);
    interpolated.velocity = p1.velocity + ratio * (p2.velocity - p1.velocity);
    
    return interpolated;
}

} // namespace lattice_planner_pkg