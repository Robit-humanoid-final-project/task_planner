#include "task_planner/vision_alignment.hpp"
#include <algorithm>
#include <cmath>

namespace task_planner
{

void VisionAlignment::reset()
{
    selected_foot_ = KickFoot::NONE;
    // 추후 D항(미분)이나 저역통과필터(LPF) 추가 시 여기서 내부 상태 초기화
}

KickFoot VisionAlignment::selectFoot(double ball_y)
{
    // 찰 발 선택
    if (selected_foot_ == KickFoot::NONE)
    {
        // TODO: 경기장 좌표, 장애물 위치 고려해서 상위 조건으로
        selected_foot_ = (ball_y >= 0.0) ? KickFoot::LEFT : KickFoot::RIGHT;
        return selected_foot_;
    }

    // 발이 정해진 상태에서는 임계값 이상 넘어갈 때만 전환
    if (selected_foot_ == KickFoot::LEFT && ball_y < -foot_switch_hysteresis_)
    {
        selected_foot_ = KickFoot::RIGHT;
    }
    else if (selected_foot_ == KickFoot::RIGHT && ball_y > foot_switch_hysteresis_)
    {
        selected_foot_ = KickFoot::LEFT;
    }

    return selected_foot_;
}

AlignOutput VisionAlignment::compute(const AlignInput &in)
{
    AlignOutput out;
    out.foot = selectFoot(in.ball_y);

    const FootOffset &target = (out.foot == KickFoot::LEFT) ? left_kick_ : right_kick_;

    const double err_x = in.ball_x - target.x;
    const double err_y = in.ball_y - target.y;

    const bool x_ok = std::abs(err_x) <= tol_x_;
    const bool y_ok = std::abs(err_y) <= tol_y_;
    const bool yaw_ok = std::abs(in.yaw_err) <= tol_yaw_;

    if (x_ok && y_ok && yaw_ok)
    {
        out.is_aligned = true;
        return out;
    }

    // 허용 오차 안에 들어온 축은 속도 0
    const double raw_vx = x_ok ? 0.0 : (kp_x_ * err_x);
    const double raw_vy = y_ok ? 0.0 : (kp_y_ * err_y);
    const double raw_vyaw = yaw_ok ? 0.0 : (kp_yaw_ * in.yaw_err);
    // clamp(v, min, max): min <= v <= max 범위 제한
    out.vx = std::clamp(raw_vx, -max_vx_, max_vx_);
    out.vy = std::clamp(raw_vy, -max_vy_, max_vy_);
    out.vyaw = std::clamp(raw_vyaw, -max_vyaw_, max_vyaw_);
    out.is_aligned = false;

    return out;
}

} // namespace task_planner