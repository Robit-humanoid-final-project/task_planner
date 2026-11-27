#include "task_planner/kick_coordinator.hpp"

#include <algorithm>
#include <cmath>

namespace task_planner
{
namespace
{
// 점과 선분 사이 최단 거리
double distance_to_segment(const FieldPoint &point, const FieldPoint &start, const FieldPoint &end)
{
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    const double projection =
        length_squared > 0.0
            ? std::clamp(((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared, 0.0, 1.0)
            : 0.0;
    const double closest_x = start.x + projection * dx;
    const double closest_y = start.y + projection * dy;
    return std::hypot(point.x - closest_x, point.y - closest_y);
}
// 경기장(안전마진 포함) 내 좌표인지 여부
bool inside_field(const FieldPoint &point, const KickCoordinationInput &in)
{
    const double half_length = in.field_length / 2.0 - in.obstacle_clearance;
    const double half_width = in.field_width / 2.0 - in.obstacle_clearance;
    return half_length > 0.0 && half_width > 0.0 && std::abs(point.x) <= half_length && std::abs(point.y) <= half_width;
}
// 경로에 장애물이 있는지 없는지 여부
bool clear_of_obstacles(const FieldPoint &start, const FieldPoint &end, const KickCoordinationInput &in)
{
    return std::all_of(in.obstacles.begin(), in.obstacles.end(), [&](const FieldPoint &obstacle) {
        return distance_to_segment(obstacle, start, end) >= in.obstacle_clearance;
    });
}
// 패스 목표 탐색
bool choose_pass_target(const KickCoordinationInput &in, double ball_x, double ball_y, double kick_distance,
                        double &heading, double &target_x, double &target_y)
{
    if (!std::isfinite(ball_x) || !std::isfinite(ball_y) || !std::isfinite(kick_distance) ||
        !std::isfinite(in.opponent_goal_x) || !std::isfinite(in.opponent_goal_y) || !std::isfinite(in.field_length) ||
        !std::isfinite(in.field_width) || !std::isfinite(in.obstacle_clearance) ||
        !std::isfinite(in.max_receiver_distance) || !std::isfinite(in.approach_standoff) ||
        !std::isfinite(in.teammate_robot_x) || !std::isfinite(in.teammate_robot_y) || in.field_length <= 0.0 ||
        in.field_width <= 0.0 || in.obstacle_clearance < 0.0 || in.approach_standoff < 0.0)
    {
        return false;
    }

    const double goal_dx = in.opponent_goal_x - ball_x;
    const double goal_dy = in.opponent_goal_y - ball_y;
    const double goal_distance = std::hypot(goal_dx, goal_dy);
    if (goal_distance <= 1e-6 || kick_distance <= 0.0 || in.max_receiver_distance < 0.0)
    {
        return false;
    } // --- 유효성 검사

    const double goal_heading = std::atan2(goal_dy, goal_dx);
    const FieldPoint ball{ball_x, ball_y};
    double best_progress = -1.0;
    double best_receiver_distance = 0.0;
    bool found_target = false;
    const double radians_per_degree = std::acos(-1.0) / 180.0;
    // 전방 각도 중에서 탐색
    for (int offset_degrees = -85; offset_degrees <= 85; offset_degrees += 5)
    {
        const double candidate_heading = goal_heading + offset_degrees * radians_per_degree;
        const FieldPoint approach{ball_x - in.approach_standoff * std::cos(candidate_heading),
                                  ball_y - in.approach_standoff * std::sin(candidate_heading)};
        const FieldPoint target{ball_x + kick_distance * std::cos(candidate_heading),
                                ball_y + kick_distance * std::sin(candidate_heading)};
        const double forward_progress =
            (target.x - ball_x) * (goal_dx / goal_distance) + (target.y - ball_y) * (goal_dy / goal_distance);
        const double receiver_distance = std::hypot(target.x - in.teammate_robot_x, target.y - in.teammate_robot_y);
        if (forward_progress <= 0.0 || receiver_distance > in.max_receiver_distance || !inside_field(target, in) ||
            !inside_field(approach, in) || !clear_of_obstacles(ball, target, in) ||
            !clear_of_obstacles(approach, approach, in))
        {
            continue;
        }

        if (!found_target || forward_progress > best_progress + 1e-6 ||
            (std::abs(forward_progress - best_progress) <= 1e-6 && receiver_distance < best_receiver_distance))
        {
            found_target = true;
            best_progress = forward_progress;
            best_receiver_distance = receiver_distance;
            heading = candidate_heading;
            target_x = target.x;
            target_y = target.y;
        }
    }

    return found_target;
}

} // namespace

KickDecision KickCoordinator::decide(const KickCoordinationInput &in, double kick_distance)
{
    KickDecision out;
    const bool teammate_available = in.teammate_data_fresh && in.teammate_playing;
    const bool local_ball_available = in.ball_detected;
    const bool teammate_ball_available = teammate_available && in.teammate_ball_detected;
    if (!local_ball_available && !teammate_ball_available)
    {
        return out;
    }

    out.ball_available = true;
    // 공 좌표 추정
    if (local_ball_available && teammate_ball_available)
    {
        out.kick_origin_x = (in.ball_x + in.teammate_ball_x) / 2.0;
        out.kick_origin_y = (in.ball_y + in.teammate_ball_y) / 2.0;
    }
    else if (local_ball_available)
    {
        out.kick_origin_x = in.ball_x;
        out.kick_origin_y = in.ball_y;
    }
    else
    {
        out.kick_origin_x = in.teammate_ball_x;
        out.kick_origin_y = in.teammate_ball_y;
    }

    // 로봇과 공 사이 거리를 비교하여 가까운 로봇이 kicker가 되도록 함
    if (!teammate_available)
    {
        out.kicker_id = in.robot_id;
    }
    else
    {
        const double local_distance = std::hypot(out.kick_origin_x - in.robot_x, out.kick_origin_y - in.robot_y);
        const double teammate_distance =
            std::hypot(out.kick_origin_x - in.teammate_robot_x, out.kick_origin_y - in.teammate_robot_y);
        const double distance_delta = local_distance - teammate_distance;
        if (std::abs(distance_delta) <= 0.2) // 거리가 비슷한 경우 id가 낮은(1번) 로봇이 kicker
        {
            out.kicker_id = std::min(in.robot_id, in.teammate_robot_id);
        }
        else
        {
            out.kicker_id = distance_delta < 0.0 ? in.robot_id : in.teammate_robot_id;
        }

        if (last_kicker_id_ == in.robot_id || last_kicker_id_ == in.teammate_robot_id)
        {
            const double previous_distance = last_kicker_id_ == in.robot_id ? local_distance : teammate_distance;
            const double alternative_distance = last_kicker_id_ == in.robot_id ? teammate_distance : local_distance;
            if (previous_distance - alternative_distance <= in.kicker_switch_hysteresis)
            {
                out.kicker_id = last_kicker_id_;
            }
        }
    }

    last_kicker_id_ = out.kicker_id;
    out.is_kicker = in.robot_id == out.kicker_id;
    if (out.is_kicker && teammate_available)
    { // 현재 로봇이 kicker인 경우 kick 경로에 obstacle이 없는 각도를 찾음
        out.receive_target_valid = choose_pass_target(in, out.kick_origin_x, out.kick_origin_y, kick_distance,
                                                      out.pass_heading, out.receive_x, out.receive_y);
    }
    else
    { // receiver인 경우 kicker가 차는 방향으로 kick_distance만큼 떨어진 위치로 이동해 패스를 받음
        out.pass_heading = in.teammate_robot_theta;
        out.receive_x = out.kick_origin_x + kick_distance * std::cos(out.pass_heading);
        out.receive_y = out.kick_origin_y + kick_distance * std::sin(out.pass_heading);
        out.receive_target_valid = inside_field({out.receive_x, out.receive_y}, in);
    }
    return out;
}

} // namespace task_planner
