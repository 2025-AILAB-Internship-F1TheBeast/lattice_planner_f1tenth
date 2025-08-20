#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <limits>

namespace lattice_planner_pkg {
namespace advanced {

// Advanced candidate result structure
struct CandidateResult {
    std::vector<geometry_msgs::msg::Point> path_points;
    double cost{0.0};
    double merge_s{0.0};
    double d_offset{0.0};
    double dist{0.0};
    size_t points{0};
    bool collided{false};
    bool out_of_track{false};
    double ref_alignment{0.0};
    double d_start{0.0};
    double d_end{0.0};
};

struct PathSelectionConfig {
    // 단순한 커밋 설정
    double path_length = 3.0;                     // 커밋 경로 길이 (m) - 단순하게
};

struct PathCommitState {
    bool has_commit = false;
    double committed_offset = 0.0;
    double committed_cost = 0.0;
    double commit_start_s = 0.0;
    double committed_path_end_s = 0.0;
};

class PathSelector {
public:
    PathSelector(const PathSelectionConfig& config = PathSelectionConfig());
    
    // 최적 경로 선택
    CandidateResult* selectOptimalPath(
        std::vector<CandidateResult>& candidates,
        bool has_obstacles,
        double current_s,
        const rclcpp::Clock::SharedPtr& clock
    );
    
    // 커밋 상태 업데이트
    void updateCommitState(
        const CandidateResult* chosen_path,
        double current_s,
        const rclcpp::Clock::SharedPtr& clock
    );
    
    // 상태 접근자
    const PathCommitState& getCommitState() const { return commit_state_; }
    
    // 설정 업데이트
    void updateConfig(const PathSelectionConfig& config) { config_ = config; }
    
    // 상태 초기화
    void resetCommit() { commit_state_.has_commit = false; }

private:
    PathSelectionConfig config_;
    PathCommitState commit_state_;
};

} // namespace advanced
} // namespace lattice_planner_pkg