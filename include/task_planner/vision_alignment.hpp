#pragma once

namespace task_planner
{

enum class KickFoot
{
    NONE,
    LEFT,
    RIGHT
};

struct FootOffset
{
    double x{0.0};
    double y{0.0};
};

struct AlignInput
{
    double ball_x{0.0};
    double ball_y{0.0};
    double yaw_err{0.0};
};

struct AlignOutput
{
    double vx{0.0};
    double vy{0.0};
    double vyaw{0.0};
    KickFoot foot{KickFoot::NONE}; // 정렬 기준 발
    bool is_aligned{false};
};

class VisionAlignment
{
  public:
    VisionAlignment() = default;

    // ALIGN 상태 진입 시 호출
    void reset();
    // 보행 속도 계산
    AlignOutput compute(const AlignInput &in);

  private:
    KickFoot selectFoot(double ball_y);
    KickFoot selected_foot_{KickFoot::NONE};
    // 킥 목표 오프셋
    FootOffset left_kick_{0.060, 0.065}; // m
    FootOffset right_kick_{0.060, -0.095};
    double foot_switch_hysteresis_{0.04}; // 진동 방지

    // 킥존 허용 오차 (m)
    double tol_x_{0.020};
    double tol_y_{0.025};
    double tol_yaw_{0.06}; // rad

    // Proportional
    double kp_x_{0.8};
    double kp_y_{0.8};
    double kp_yaw_{1.0};

    double max_vx_{2.0};
    double max_vy_{1.0};
    double max_vyaw_{1.0};
};

} // namespace task_planner