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
    
    // Primary path 선택 (실제 상황 기반) - 간단한 로직
    CandidateResult* primary = nullptr;
    
    // 안전한 경로 중에서 가장 cost가 낮은 것 선택
    for (auto& c : candidates) {
        if (!c.collided && !c.out_of_track) {
            if (!primary || c.cost < primary->cost) {
                primary = &c;
            }
        }
    }
    
    // 안전한 경로가 없으면 최소 cost 경로라도 선택
    if (!primary && !candidates.empty()) {
        auto it = std::min_element(candidates.begin(), candidates.end(),
            [](const auto& a, const auto& b) { return a.cost < b.cost; });
        primary = &*it;
    }
    
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
    
    CandidateResult* chosen = nullptr;
    
    // 🎯 단순화된 경로 선택 로직: 커밋 우선, 안전성 필수
    if (commit_state_.has_commit && committed_in_set) {
        // 1. 커밋된 경로가 안전하면 계속 사용
        if (!committed_in_set->collided && !committed_in_set->out_of_track) {
            bool should_keep_commit = (current_s < commit_state_.committed_path_end_s);  // 거리 기반 커밋
            
            if (should_keep_commit) {
                chosen = committed_in_set;
                RCLCPP_DEBUG(rclcpp::get_logger("path_selector"), 
                    "[COMMIT KEEP] Safe committed path: offset=%.3f", committed_in_set->d_offset);
            } else {
                // 커밋 만료 - 새로운 경로 선택
                chosen = primary;
                commit_state_.has_commit = false;
                RCLCPP_INFO(rclcpp::get_logger("path_selector"), 
                    "[COMMIT EXPIRED] Switching to new path: %.3f->%.3f", 
                    committed_in_set->d_offset, primary ? primary->d_offset : 0.0);
            }
        } else {
            // 2. 커밋된 경로가 위험해지면 즉시 변경
            chosen = primary;
            commit_state_.has_commit = false;
            RCLCPP_WARN(rclcpp::get_logger("path_selector"), 
                "[COMMIT UNSAFE] Switching to safe alternative");
        }
    } else {
        // 3. 커밋이 없으면 primary 선택
        chosen = primary;
    }
    
    // 🚨 최종 안전 검증: 충돌 경로는 절대 반환하지 않음
    if (chosen) {
        if (chosen->collided || chosen->out_of_track) {
            RCLCPP_ERROR(rclcpp::get_logger("path_selector"), 
                "🚨🚨🚨 [SAFETY OVERRIDE] REJECTING DANGEROUS PATH: offset=%.3f, collided=%s, out_of_track=%s", 
                chosen->d_offset, chosen->collided ? "true" : "false", chosen->out_of_track ? "true" : "false");
            
            // 안전한 대안 찾기 - 비용 기준으로 최적 선택
            CandidateResult* safe_alternative = nullptr;
            double best_safe_cost = std::numeric_limits<double>::max();
            
            for (auto& c : candidates) {
                if (!c.collided && !c.out_of_track) {
                    if (c.cost < best_safe_cost) {
                        best_safe_cost = c.cost;
                        safe_alternative = &c;
                    }
                }
            }
            
            if (safe_alternative) {
                RCLCPP_WARN(rclcpp::get_logger("path_selector"), 
                    "🛡️ [SAFETY RECOVERY] Using BEST safe alternative: offset=%.3f, cost=%.3f", 
                    safe_alternative->d_offset, safe_alternative->cost);
                chosen = safe_alternative;
            } else {
                // 마지막 수단: 최소 비용 경로 (out_of_track이라도)
                RCLCPP_FATAL(rclcpp::get_logger("path_selector"), 
                    "💀 [EMERGENCY] NO COMPLETELY SAFE PATHS! Using minimum cost path as last resort");
                
                CandidateResult* min_cost_path = nullptr;
                double absolute_min_cost = std::numeric_limits<double>::max();
                
                for (auto& c : candidates) {
                    if (c.cost < absolute_min_cost) {
                        absolute_min_cost = c.cost;
                        min_cost_path = &c;
                    }
                }
                
                if (min_cost_path) {
                    RCLCPP_ERROR(rclcpp::get_logger("path_selector"), 
                        "🆘 [LAST RESORT] Using minimum cost path: offset=%.3f, cost=%.3f", 
                        min_cost_path->d_offset, min_cost_path->cost);
                    chosen = min_cost_path;
                } else {
                    return nullptr;  // 정말 아무것도 없으면 null
                }
            }
        }
        
        RCLCPP_DEBUG(rclcpp::get_logger("path_selector"), 
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
    
    // 새로운 경로가 선택되면 커밋 생성
    if (!commit_state_.has_commit || 
        std::abs(chosen_path->d_offset - commit_state_.committed_offset) > 0.1) {  // 10cm 이상 차이
        
        commit_state_.has_commit = true;
        commit_state_.committed_offset = chosen_path->d_offset;
        commit_state_.commit_start_s = current_s;
        commit_state_.committed_cost = chosen_path->cost;
        
        // 간단한 커밋 길이: 기본 길이만 사용
        commit_state_.committed_path_end_s = current_s + config_.path_length;
        
        RCLCPP_INFO(rclcpp::get_logger("path_selector"), 
            "[NEW COMMIT] Path committed: offset=%.3f, length=%.1fm", 
            chosen_path->d_offset, config_.path_length);
    }
}



} // namespace advanced
} // namespace lattice_planner_pkg