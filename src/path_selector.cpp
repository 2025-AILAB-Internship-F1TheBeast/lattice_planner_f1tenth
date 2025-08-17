#include "lattice_planner_pkg/path_selector.hpp"
#include <algorithm>
#include <cmath>

namespace lattice_planner_pkg {
namespace advanced {

PathSelector::PathSelector(const PathSelectionConfig& config) : config_(config) {}

CandidateResult* PathSelector::selectOptimalPath(
    std::vector<CandidateResult>& candidates,
    bool has_obstacles,
    double current_s,
    const rclcpp::Clock::SharedPtr& clock) {
    
    if (candidates.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("path_selector"), "[DEBUG] No candidates to select from");
        return nullptr;
    }
    
    // 실제 안전한 경로가 있는지 확인 (has_obstacles보다 정확한 판단)
    int safe_path_count = 0;
    int total_paths = candidates.size();
    for (const auto& c : candidates) {
        if (!c.collided && !c.out_of_track) {
            safe_path_count++;
        }
    }
    
    bool actual_obstacle_situation = (safe_path_count == 0);  // 안전한 경로가 하나도 없으면 장애물 상황
    
    RCLCPP_ERROR(rclcpp::get_logger("path_selector"), 
        "[CRITICAL] PathSelector: %d/%d safe paths, has_obstacles=%s, actual_situation=%s", 
        safe_path_count, total_paths, has_obstacles ? "true" : "false",
        actual_obstacle_situation ? "DANGER" : "SAFE");
    
    // Primary path 선택 (실제 상황 기반)
    CandidateResult* primary = selectPrimaryPath(candidates, actual_obstacle_situation);
    CandidateResult* reference_candidate = findReferenceCandidate(candidates);
    
    // Detour 상태 업데이트
    updateDetourState(reference_candidate);
    
    // 현재 커밋된 경로가 후보 리스트에 있는지 확인
    CandidateResult* committed_in_set = nullptr;
    if (commit_state_.has_commit) {
        for (auto& c : candidates) {
            if (std::abs(c.d_offset - commit_state_.committed_offset) < 1e-3) {
                committed_in_set = &c;
                break;
            }
        }
    }
    
    // 스위칭 허용 여부 판단
    bool allow_switch = shouldAllowSwitch(committed_in_set, primary, has_obstacles, current_s, clock);
    
    CandidateResult* chosen = nullptr;
    
    // 경로 길이 기반 커밋이 최우선
    if (config_.path_length_commit_mode && commit_state_.has_commit && committed_in_set && !allow_switch) {
        chosen = committed_in_set;
    } else if (detour_state_.detour_active && !canReturnFromDetour(reference_candidate)) {
        // Detour 유지 강제
        if (commit_state_.has_commit && committed_in_set && 
            !(committed_in_set->collided || committed_in_set->out_of_track)) {
            chosen = committed_in_set;
        } else {
            chosen = primary;
        }
    } else {
        // 정상 선택 로직
        if (!has_obstacles) {
            if (!detour_state_.detour_active) {
                // 원래부터 중앙선
                if (reference_candidate && !reference_candidate->collided && !reference_candidate->out_of_track) {
                    chosen = reference_candidate;
                }
            } else {
                // Detour에서 복귀 가능한 상황
                if (canReturnFromDetour(reference_candidate)) {
                    chosen = reference_candidate;
                    commit_state_.has_commit = false; // 복귀 시 커밋 초기화
                } else {
                    // 아직 복귀 조건 미충족
                    if (commit_state_.has_commit && committed_in_set && 
                        !(committed_in_set->collided || committed_in_set->out_of_track)) {
                        chosen = committed_in_set;
                    } else {
                        chosen = primary;
                    }
                }
            }
        } else if (commit_state_.has_commit && committed_in_set && !allow_switch) {
            // 이전 커밋 유지 - 단, 안전성 재확인
            if (!(committed_in_set->collided || committed_in_set->out_of_track)) {
                chosen = committed_in_set;
                RCLCPP_INFO(rclcpp::get_logger("path_selector"), 
                    "[SAFE COMMIT] Maintaining safe committed path: offset=%.3f", committed_in_set->d_offset);
            } else {
                // 커밋된 경로가 위험해졌으면 primary로 대체
                chosen = primary;
                RCLCPP_WARN(rclcpp::get_logger("path_selector"), 
                    "[UNSAFE COMMIT] Committed path became unsafe, switching to primary");
            }
        } else {
            // 새 선택 - 안전성 우선 검증
            if (primary && !(primary->collided || primary->out_of_track)) {
                chosen = primary;
                RCLCPP_INFO(rclcpp::get_logger("path_selector"), 
                    "[NEW SAFE] Selected safe primary path: offset=%.3f", primary->d_offset);
            } else if (primary) {
                RCLCPP_ERROR(rclcpp::get_logger("path_selector"), 
                    "[WARNING] Primary path is unsafe but selected: offset=%.3f, collided=%s, out_of_track=%s", 
                    primary->d_offset, primary->collided ? "true" : "false", 
                    primary->out_of_track ? "true" : "false");
                chosen = primary;
            }
        }
    }
    
    // 🚨 최종 안전 검증: 충돌 경로는 절대 반환하지 않음
    if (chosen) {
        if (chosen->collided || chosen->out_of_track) {
            RCLCPP_ERROR(rclcpp::get_logger("path_selector"), 
                "🚨🚨🚨 [SAFETY OVERRIDE] REJECTING DANGEROUS PATH: offset=%.3f, collided=%s, out_of_track=%s", 
                chosen->d_offset, chosen->collided ? "true" : "false", chosen->out_of_track ? "true" : "false");
            
            // 안전한 대안 찾기
            CandidateResult* safe_alternative = nullptr;
            for (auto& c : candidates) {
                if (!c.collided && !c.out_of_track) {
                    safe_alternative = &c;
                    break;  // 첫 번째 안전한 경로 선택
                }
            }
            
            if (safe_alternative) {
                RCLCPP_WARN(rclcpp::get_logger("path_selector"), 
                    "🛡️ [SAFETY RECOVERY] Using safe alternative: offset=%.3f", safe_alternative->d_offset);
                chosen = safe_alternative;
            } else {
                RCLCPP_FATAL(rclcpp::get_logger("path_selector"), 
                    "💀 [CRITICAL] NO SAFE PATHS AVAILABLE - EMERGENCY STOP RECOMMENDED");
                return nullptr;  // 안전한 경로가 없으면 null 반환
            }
        }
        
        RCLCPP_INFO(rclcpp::get_logger("path_selector"), 
            "✅ [FINAL CHOICE] Safe path selected: cost=%.3f, offset=%.3f", 
            chosen->cost, chosen->d_offset);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("path_selector"), "[DEBUG] PathSelector could not choose any path");
    }
    
    return chosen;
}

void PathSelector::updateCommitState(
    const CandidateResult* chosen_path,
    double current_s,
    const rclcpp::Clock::SharedPtr& clock) {
    
    if (!chosen_path) return;
    
    // 현재 장애물 상황 확인 (detour 활성화 여부로 판단)
    bool has_current_obstacles = detour_state_.detour_active;
    
    // 선택된 경로가 커밋과 다르면 커밋 갱신
    if (!commit_state_.has_commit || 
        std::abs(chosen_path->d_offset - commit_state_.committed_offset) > 1e-3) {
        
        commit_state_.has_commit = true;
        commit_state_.committed_offset = chosen_path->d_offset;
        commit_state_.committed_merge_s = chosen_path->merge_s;
        commit_state_.commit_start_s = current_s;
        commit_state_.commit_start_time = clock->now();
        commit_state_.committed_cost = chosen_path->cost;
        commit_state_.committed_ref_alignment = chosen_path->ref_alignment;
        commit_state_.committed_during_obstacle = has_current_obstacles;
        
        // 경로 길이 계산: 장애물 상황에서는 더 긴 커밋
        double base_length = config_.path_length;
        if (has_current_obstacles) {
            commit_state_.obstacle_commit_extra_distance = base_length * (config_.obstacle_path_length_multiplier - 1.0);
            base_length *= config_.obstacle_path_length_multiplier;
            RCLCPP_WARN(rclcpp::get_logger("path_selector"), 
                "[OBSTACLE COMMIT] Extended commit length: %.2fm (extra: %.2fm)", 
                base_length, commit_state_.obstacle_commit_extra_distance);
        } else {
            commit_state_.obstacle_commit_extra_distance = 0.0;
        }
        
        commit_state_.committed_path_end_s = current_s + base_length;
    }
}

CandidateResult* PathSelector::selectPrimaryPath(std::vector<CandidateResult>& candidates, bool has_obstacles) {
    if (candidates.empty()) return nullptr;
    
    RCLCPP_INFO(rclcpp::get_logger("path_selector"), "[DEBUG] selectPrimaryPath: has_obstacles=%s", has_obstacles ? "true" : "false");
    
    // RACELINE RECOVERY: Find raceline candidate first
    CandidateResult* raceline_candidate = nullptr;
    CandidateResult* safe_raceline = nullptr;
    
    for (auto& c : candidates) {
        if (std::abs(c.d_offset) < 0.05) { // raceline candidate
            raceline_candidate = &c;
            if (!c.collided && !c.out_of_track) {
                safe_raceline = &c;
                break; // Found safe raceline, use it immediately
            }
        }
    }
    
    // Priority 1: Safe raceline
    if (safe_raceline) {
        RCLCPP_INFO(rclcpp::get_logger("path_selector"), "[RACELINE RECOVERY] Using safe raceline, cost=%.3f", safe_raceline->cost);
        return safe_raceline;
    }
    
    // Priority 2: REMOVED - 위험한 충돌 raceline 강제 선택 로직 제거
    // 안전성을 최우선으로 하여 충돌하는 raceline은 절대 선택하지 않음
    
    if (has_obstacles) {
        // 장애물 있을 때: 안전성 최우선, 그 다음 비용 최소화
        
        // 1단계: 안전한 경로들만 수집
        std::vector<CandidateResult*> safe_candidates;
        for (auto& c : candidates) {
            if (!c.collided && !c.out_of_track) {
                safe_candidates.push_back(&c);
            }
        }
        
        if (!safe_candidates.empty()) {
            // 안전한 경로 중에서 최적 선택
            CandidateResult* best_safe = nullptr;
            double best_cost = std::numeric_limits<double>::infinity();
            
            // 우선순위: raceline > cost 최소
            for (auto* candidate : safe_candidates) {
                // raceline 우선 검사
                if (std::abs(candidate->d_offset) < 0.05) {
                    RCLCPP_INFO(rclcpp::get_logger("path_selector"), 
                        "[SAFE RACELINE] Found safe raceline in obstacle situation, cost=%.3f", candidate->cost);
                    return candidate;
                }
                
                // 비용 기반 선택
                if (candidate->cost < best_cost) {
                    best_cost = candidate->cost;
                    best_safe = candidate;
                }
            }
            
            if (best_safe) {
                RCLCPP_INFO(rclcpp::get_logger("path_selector"), 
                    "[SAFE PATH] Selected safe path in obstacle situation: offset=%.3f, cost=%.3f", 
                    best_safe->d_offset, best_safe->cost);
                return best_safe;
            }
        }
        
        // 2단계: 안전한 경로가 없을 때만 위험한 경로 고려
        RCLCPP_ERROR(rclcpp::get_logger("path_selector"), 
            "[EMERGENCY] No safe paths available! Selecting least dangerous option");
        
        auto it = std::min_element(candidates.begin(), candidates.end(),
            [](const auto& a, const auto& b) { 
                // 트랙 내부 경로 우선, 그 다음 cost 기준
                if (a.out_of_track != b.out_of_track) {
                    return !a.out_of_track;  // 트랙 내부가 우선
                }
                return a.cost < b.cost; 
            });
        
        return (it != candidates.end() ? &*it : nullptr);
    } else {
        // 장애물이 없을 때: raceline 우선, 없으면 cost 기반 선택
        if (raceline_candidate) {
            RCLCPP_INFO(rclcpp::get_logger("path_selector"), "[RACELINE RECOVERY] No obstacles, using raceline, cost=%.3f", raceline_candidate->cost);
            return raceline_candidate;
        }
        
        auto it = std::min_element(candidates.begin(), candidates.end(),
            [](const auto& a, const auto& b) {
                // out_of_track / collided 우선 제외
                bool a_bad = a.collided || a.out_of_track;
                bool b_bad = b.collided || b.out_of_track;
                if (a_bad != b_bad) return !a_bad; // 좋은 것이 우선
                return a.cost < b.cost; // cost 기반 선택
            });
        
        return (it != candidates.end() ? &*it : nullptr);
    }
}

CandidateResult* PathSelector::findReferenceCandidate(std::vector<CandidateResult>& candidates) {
    for (auto& c : candidates) {
        if (std::abs(c.d_offset - config_.reference_offset_target) < config_.reference_offset_tolerance) {
            return &c;
        }
    }
    return nullptr;
}

bool PathSelector::shouldAllowSwitch(
    const CandidateResult* committed_path,
    const CandidateResult* primary_path,
    bool has_obstacles,
    double current_s,
    const rclcpp::Clock::SharedPtr& clock) {
    
    if (!commit_state_.has_commit || !committed_path) return true;
    
    // 충돌/트랙 이탈 시 즉시 스위치 허용
    if (committed_path->collided || committed_path->out_of_track) {
        return true;
    }
    
    if (config_.path_length_commit_mode) {
        // 경로 길이 기반 커밋: 장애물 상황에서 더 보수적인 해제
        bool basic_length_reached = (current_s >= commit_state_.committed_path_end_s);
        
        // 장애물 상황에서 커밋된 경우, 추가 안전 조건 확인
        if (commit_state_.committed_during_obstacle) {
            // 기본 길이는 도달했지만, 여전히 장애물이 있는 경우 커밋 유지
            if (basic_length_reached && has_obstacles) {
                RCLCPP_WARN(rclcpp::get_logger("path_selector"), 
                    "[OBSTACLE SAFETY] Basic length reached but obstacles still present - maintaining commit");
                return false;  // 커밋 유지
            }
            
            // 장애물이 없어졌을 때만 해제 허용
            if (basic_length_reached && !has_obstacles) {
                RCLCPP_INFO(rclcpp::get_logger("path_selector"), 
                    "[OBSTACLE SAFETY] Length reached and obstacles cleared - allowing switch");
                return true;
            }
        }
        
        return basic_length_reached;
    } else {
        // 기존 시간/거리 기반 커밋 로직
        double progress = current_s - commit_state_.commit_start_s;
        double elapsed = (clock->now() - commit_state_.commit_start_time).seconds();
        
        double min_progress_threshold = has_obstacles ? 
            config_.commit_obstacle_min_progress : config_.commit_min_progress;
        double min_time_threshold = has_obstacles ? 
            config_.commit_obstacle_min_time_sec : config_.commit_min_time_sec;
        double improve_ratio_threshold = has_obstacles ? 
            config_.commit_obstacle_improve_ratio : config_.commit_cost_improve_ratio;
        
        bool min_hold = (progress < min_progress_threshold) || (elapsed < min_time_threshold);
        
        if (primary_path) {
            double current_metric = committed_path->cost;
            double primary_metric = primary_path->cost;
            double improvement = (current_metric - primary_metric) / std::max(1e-6, current_metric);
            bool big_lateral_change = std::abs(primary_path->d_offset - commit_state_.committed_offset) >= 
                config_.commit_lateral_change_min;
            bool significant_improve = improvement > improve_ratio_threshold;
            
            if (min_hold && !significant_improve) {
                return false;
            }
        } else if (min_hold) {
            return false;
        }
        
        return true;
    }
}

void PathSelector::updateDetourState(const CandidateResult* reference_candidate) {
    // Detour 상태 갱신: committed offset이 reference tolerance 밖이면 detour
    if (commit_state_.has_commit) {
        detour_state_.detour_active = std::abs(commit_state_.committed_offset - config_.reference_offset_target) > 
            config_.reference_offset_tolerance;
    }
    
    // Reference candidate가 깨끗한지 평가
    bool reference_clean = false;
    if (reference_candidate && !reference_candidate->collided && !reference_candidate->out_of_track) {
        reference_clean = true;
    }
    
    if (reference_clean) {
        detour_state_.detour_clear_frames++;
    } else {
        detour_state_.detour_clear_frames = 0;
    }
}

bool PathSelector::canReturnFromDetour(const CandidateResult* reference_candidate) const {
    return detour_state_.detour_active && 
           reference_candidate && 
           !reference_candidate->collided && 
           !reference_candidate->out_of_track &&
           detour_state_.detour_clear_frames >= config_.detour_return_clear_frames_threshold;
}

} // namespace advanced
} // namespace lattice_planner_pkg