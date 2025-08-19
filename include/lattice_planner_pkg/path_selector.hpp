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
    // 기본 커밋 파라미터 (더 짧은 커밋 시간으로 빠른 raceline 복귀)
    double commit_min_progress = 0.5;              // 최소 진행 거리 (m) - 단축
    double commit_min_time_sec = 0.3;              // 최소 경과 시간 (s) - 대폭 단축
    double commit_cost_improve_ratio = 0.1;        // 비용 개선 비율 - 더 민감하게
    double commit_lateral_change_min = 0.05;       // 최소 lateral 변화 (m) - 더 민감하게
    
    // 장애물 상황에서의 커밋 파라미터 (극단적으로 단축)
    double commit_obstacle_min_progress = 0.8;     // 장애물 상황 최소 진행 거리 (m) - 대폭 단축
    double commit_obstacle_min_time_sec = 0.5;     // 장애물 상황 최소 경과 시간 (s) - 대폭 단축
    double commit_obstacle_improve_ratio = 0.2;    // 장애물 상황 비용 개선 비율 - 더 민감하게
    
    // Detour 복귀 설정 (더 빠른 복귀)
    int detour_return_clear_frames_threshold = 5;  // 복귀를 위한 clear 프레임 수 - 대폭 단축
    double reference_offset_target = 0.0;          // 기본 복귀 목표 offset
    double reference_offset_tolerance = 0.03;      // reference로 간주하는 tolerance
    
    // 경로 길이 기반 커밋 (장애물 회피 지속성 강화)
    bool path_length_commit_mode = true;           // 경로 길이 기반 커밋 활성화
    double path_length = 4.0;                     // 커밋 경로 길이 (m) - 장애물 회피 안정성 향상
    double obstacle_path_length_multiplier = 1.5;  // 장애물 상황에서 경로 길이 배수
    
    // Hysteresis/Smoothing parameters for path stability
    double cost_difference_threshold = 0.15;      // 경로 변경을 위한 최소 cost 차이
    int stability_frame_count = 3;                // 안정성을 위한 연속 프레임 수
    double lateral_change_penalty = 0.5;          // 횡방향 변화에 대한 패널티 가중치
};

struct PathCommitState {
    bool has_commit = false;
    double committed_offset = 0.0;
    double committed_cost = std::numeric_limits<double>::infinity();
    double committed_ref_alignment = std::numeric_limits<double>::infinity();
    double committed_merge_s = 0.0;
    double commit_start_s = 0.0;
    rclcpp::Time commit_start_time;
    double committed_path_end_s = 0.0;
    bool committed_during_obstacle = false;  // 장애물 상황에서 커밋되었는지
    double obstacle_commit_extra_distance = 0.0;  // 장애물 상황 추가 거리
};

struct DetourState {
    bool detour_active = false;
    int detour_clear_frames = 0;
};

struct PathStabilityState {
    double last_chosen_offset = 0.0;              // 마지막으로 선택된 경로의 offset
    double last_chosen_cost = 0.0;                // 마지막으로 선택된 경로의 cost
    int stability_counter = 0;                    // 안정성 카운터
    bool has_previous_choice = false;             // 이전 선택 기록이 있는지
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
    const DetourState& getDetourState() const { return detour_state_; }
    bool isDetourActive() const { return detour_state_.detour_active; }
    
    // 설정 업데이트
    void updateConfig(const PathSelectionConfig& config) { config_ = config; }
    
    // 상태 초기화
    void resetCommit() { commit_state_.has_commit = false; }

private:
    PathSelectionConfig config_;
    PathCommitState commit_state_;
    DetourState detour_state_;
    PathStabilityState stability_state_;
    
    // 내부 헬퍼 함수들
    CandidateResult* selectPrimaryPath(std::vector<CandidateResult>& candidates, bool has_obstacles);
    CandidateResult* findReferenceCandidate(std::vector<CandidateResult>& candidates);
    bool shouldAllowSwitch(
        const CandidateResult* committed_path,
        const CandidateResult* primary_path,
        bool has_obstacles,
        double current_s,
        const rclcpp::Clock::SharedPtr& clock
    );
    void updateDetourState(const CandidateResult* reference_candidate);
    bool canReturnFromDetour(const CandidateResult* reference_candidate) const;
    
    // Hysteresis/stability functions
    CandidateResult* applyPathStabilityFilter(CandidateResult* candidate, std::vector<CandidateResult>& candidates);
    void updateStabilityState(const CandidateResult* chosen_path);
};

} // namespace advanced
} // namespace lattice_planner_pkg