#include "lattice_planner_pkg/core/spline_utils.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cctype>

namespace lattice_planner_pkg {

SplineUtils::SplineUtils() {
}

SplineUtils::~SplineUtils() {
}

std::vector<RefPoint> SplineUtils::load_reference_path_from_csv(
    const std::string& file_path, double resolution) {
    
    std::vector<RefPoint> path;
    std::ifstream file(file_path);
    
    if (!file.is_open()) {
        std::cerr << "Failed to open CSV file: " << file_path << std::endl;
        return path;
    }
    
    std::string line;
    std::vector<std::string> headers;
    int x_col = -1, y_col = -1, vel_col = -1, width_left_col = -1, width_right_col = -1;
    bool first_line = true;
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;
        
        while (std::getline(ss, cell, ',')) {
            row.push_back(cell);
        }
        
        // Parse header line to find column indices
        if (first_line) {
            headers = row;
            
            // Find column indices based on header names
            for (int i = 0; i < headers.size(); ++i) {
                std::string header = headers[i];
                
                // Convert to lowercase for case-insensitive comparison
                std::transform(header.begin(), header.end(), header.begin(), ::tolower);
                
                if (header == "x_m" || header == "x") {
                    x_col = i;
                } else if (header == "y_m" || header == "y") {
                    y_col = i;
                } else if (header == "vx_mps" || header == "v" || header == "velocity" || 
                          header == "speed" || header.find("vel") != std::string::npos) {
                    vel_col = i;
                } else if (header == "width_left_m" || header == "width_left" || header.find("width_left") != std::string::npos) {
                    width_left_col = i;
                } else if (header == "width_right_m" || header == "width_right" || header.find("width_right") != std::string::npos) {
                    width_right_col = i;
                }
            }
            
            // Fallback: if specific headers not found, try positional mapping
            if (x_col == -1 || y_col == -1 || width_left_col == -1 || width_right_col == -1) {
                std::cout << "Headers not found by name, trying positional mapping..." << std::endl;
                // For Spielberg format: x_m,y_m,width_left_m,width_right_m,vx_mps
                if (headers.size() >= 5 && headers[0].find("x") != std::string::npos) {
                    x_col = 0; y_col = 1; width_left_col = 2; width_right_col = 3; vel_col = 4;
                    std::cout << "Using Spielberg format mapping" << std::endl;
                }
                // For slam format: s_m,x_m,y_m,vx_mps,width_left_m,width_right_m
                else if (headers.size() >= 6 && headers[1].find("x") != std::string::npos) {
                    x_col = 1; y_col = 2; vel_col = 3; width_left_col = 4; width_right_col = 5;
                    std::cout << "Using slam format mapping" << std::endl;
                }
            }
            
            if (x_col == -1 || y_col == -1) {
                std::cerr << "Could not find x_m and y_m columns in CSV header" << std::endl;
                return path;
            }
            
            std::cout << "=== CSV 헤더 파싱 결과 ===" << std::endl;
            std::cout << "전체 헤더 개수: " << headers.size() << std::endl;
            for (int i = 0; i < headers.size(); ++i) {
                std::cout << "  [" << i << "] = '" << headers[i] << "'" << std::endl;
            }
            
            std::cout << "CSV column mapping - X: " << x_col << " (" << headers[x_col] 
                      << "), Y: " << y_col << " (" << headers[y_col] << ")";
            if (vel_col >= 0) {
                std::cout << ", Velocity: " << vel_col << " (" << headers[vel_col] << ")";
            }
            if (width_left_col >= 0) {
                std::cout << ", Width_Left: " << width_left_col << " (" << headers[width_left_col] << ")";
            } else {
                std::cout << ", Width_Left: NOT FOUND!";
            }
            if (width_right_col >= 0) {
                std::cout << ", Width_Right: " << width_right_col << " (" << headers[width_right_col] << ")";
            } else {
                std::cout << ", Width_Right: NOT FOUND!";
            }
            std::cout << std::endl;
            
            first_line = false;
            continue;
        }
        
        // Parse data rows
        if (row.size() > std::max(x_col, y_col)) {
            RefPoint point;
            try {
                point.x = std::stod(row[x_col]);
                point.y = std::stod(row[y_col]);
                point.velocity = (vel_col >= 0 && vel_col < row.size()) ? 
                                std::stod(row[vel_col]) : 5.0;  // Default velocity
                
                // Read width information if available
                point.width_left = (width_left_col >= 0 && width_left_col < row.size()) ? 
                                  std::stod(row[width_left_col]) : 1.0;  // Default 1m
                point.width_right = (width_right_col >= 0 && width_right_col < row.size()) ? 
                                   std::stod(row[width_right_col]) : 1.0;  // Default 1m
                
                // Debug: 처음 몇 개 점의 width 값 확인
                static int width_log_count = 0;
                if (width_log_count < 5) {
                    std::cout << "CSV 파싱: x=" << point.x << ", y=" << point.y 
                              << ", width_left=" << point.width_left << ", width_right=" << point.width_right 
                              << ", col_indices: left=" << width_left_col << ", right=" << width_right_col << std::endl;
                    width_log_count++;
                }
            } catch (const std::exception& e) {
                std::cerr << "Error parsing CSV line: " << line << std::endl;
                continue;
            }
            
            path.push_back(point);
        }
    }
    
    file.close();
    
    if (path.empty()) {
        std::cerr << "No valid points loaded from CSV file" << std::endl;
        return path;
    }
    
    // Calculate derived properties
    calculate_arc_length(path);
    calculate_heading(path);
    calculate_curvature(path);
    
    // Resample path if needed
    if (resolution > 0.0) {
        path = smooth_path(path, resolution);
    }
    
    std::cout << "Loaded " << path.size() << " reference points from " << file_path << std::endl;
    
    return path;
}

void SplineUtils::calculate_arc_length(std::vector<RefPoint>& path) {
    if (path.empty()) return;
    
    path[0].s = 0.0;
    
    for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i].x - path[i-1].x;
        double dy = path[i].y - path[i-1].y;
        double ds = std::sqrt(dx*dx + dy*dy);
        
        path[i].s = path[i-1].s + ds;
    }
}

void SplineUtils::calculate_heading(std::vector<RefPoint>& path) {
    if (path.size() < 2) return;
    
    // Calculate heading for each point
    for (size_t i = 0; i < path.size(); ++i) {
        if (i == 0) {
            // First point: use direction to next point
            double dx = path[i+1].x - path[i].x;
            double dy = path[i+1].y - path[i].y;
            path[i].heading = std::atan2(dy, dx);
        } else if (i == path.size() - 1) {
            // Last point: use direction from previous point
            double dx = path[i].x - path[i-1].x;
            double dy = path[i].y - path[i-1].y;
            path[i].heading = std::atan2(dy, dx);
        } else {
            // Middle points: average of directions
            double dx1 = path[i].x - path[i-1].x;
            double dy1 = path[i].y - path[i-1].y;
            double heading1 = std::atan2(dy1, dx1);
            
            double dx2 = path[i+1].x - path[i].x;
            double dy2 = path[i+1].y - path[i].y;
            double heading2 = std::atan2(dy2, dx2);
            
            // Handle angle wrapping
            double diff = heading2 - heading1;
            while (diff > M_PI) diff -= 2.0 * M_PI;
            while (diff < -M_PI) diff += 2.0 * M_PI;
            
            path[i].heading = normalize_angle(heading1 + diff * 0.5);
        }
    }
}

void SplineUtils::calculate_curvature(std::vector<RefPoint>& path) {
    if (path.size() < 3) return;
    
    for (size_t i = 1; i < path.size() - 1; ++i) {
        path[i].curvature = calculate_curvature_at_point(path[i-1], path[i], path[i+1]);
    }
    
    // Handle boundary points
    if (path.size() >= 3) {
        path[0].curvature = path[1].curvature;
        path[path.size()-1].curvature = path[path.size()-2].curvature;
    }
}

std::vector<RefPoint> SplineUtils::smooth_path(
    const std::vector<RefPoint>& raw_path, double resolution) {
    
    if (raw_path.size() < 2) {
        return raw_path;
    }
    
    std::vector<RefPoint> smoothed_path;
    double total_length = raw_path.back().s;
    
    // Resample at uniform intervals
    for (double s = 0.0; s <= total_length; s += resolution) {
        RefPoint interpolated = interpolate_at_s(raw_path, s);
        smoothed_path.push_back(interpolated);
    }
    
    // Ensure we include the last point
    if (!smoothed_path.empty() && 
        std::abs(smoothed_path.back().s - total_length) > resolution * 0.5) {
        smoothed_path.push_back(raw_path.back());
    }
    
    return smoothed_path;
}

RefPoint SplineUtils::interpolate_at_s(const std::vector<RefPoint>& path, double s) {
    if (path.empty()) {
        return RefPoint();
    }
    
    if (s <= path[0].s) {
        return path[0];
    }
    
    if (s >= path.back().s) {
        return path.back();
    }
    
    // Find surrounding points
    size_t i = 0;
    for (i = 0; i < path.size() - 1; ++i) {
        if (path[i + 1].s > s) {
            break;
        }
    }
    
    if (i >= path.size() - 1) {
        return path.back();
    }
    
    // Linear interpolation
    const RefPoint& p1 = path[i];
    const RefPoint& p2 = path[i + 1];
    
    double ratio = (s - p1.s) / (p2.s - p1.s);
    ratio = std::max(0.0, std::min(1.0, ratio));
    
    RefPoint interpolated;
    interpolated.x = p1.x + ratio * (p2.x - p1.x);
    interpolated.y = p1.y + ratio * (p2.y - p1.y);
    interpolated.s = s;
    interpolated.velocity = p1.velocity + ratio * (p2.velocity - p1.velocity);
    interpolated.heading = p1.heading + ratio * normalize_angle(p2.heading - p1.heading);
    interpolated.curvature = p1.curvature + ratio * (p2.curvature - p1.curvature);
    interpolated.width_left = p1.width_left + ratio * (p2.width_left - p1.width_left);
    interpolated.width_right = p1.width_right + ratio * (p2.width_right - p1.width_right);
    
    return interpolated;
}

double SplineUtils::calculate_distance(const Point2D& p1, const Point2D& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

double SplineUtils::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double SplineUtils::calculate_curvature_at_point(
    const RefPoint& prev, const RefPoint& curr, const RefPoint& next) {
    
    // Use finite differences to approximate curvature
    double dx1 = curr.x - prev.x;
    double dy1 = curr.y - prev.y;
    double dx2 = next.x - curr.x;
    double dy2 = next.y - curr.y;
    
    double cross_product = dx1 * dy2 - dy1 * dx2;
    double ds1 = std::sqrt(dx1*dx1 + dy1*dy1);
    double ds2 = std::sqrt(dx2*dx2 + dy2*dy2);
    
    if (ds1 < 1e-6 || ds2 < 1e-6) {
        return 0.0;
    }
    
    // Approximate curvature
    double curvature = 2.0 * cross_product / (ds1 * ds2 * (ds1 + ds2));
    
    return curvature;
}

} // namespace lattice_planner_pkg