#pragma once

#include <vector>

namespace task_planner
{

struct FieldPoint
{
    double x{0.0};
    double y{0.0};
};

struct KickCoordinationInput
{
    int robot_id{1};
    bool ball_detected{false};
    double ball_x{0.0};
    double ball_y{0.0};
    double robot_x{0.0};
    double robot_y{0.0};
    double robot_theta{0.0};

    bool teammate_data_fresh{false};
    bool teammate_playing{false};
    int teammate_robot_id{2};
    double teammate_robot_x{0.0};
    double teammate_robot_y{0.0};
    bool teammate_ball_detected{false};
    double teammate_ball_x{0.0};
    double teammate_ball_y{0.0};
    double teammate_robot_theta{0.0};

    double opponent_goal_x{4.5};
    double opponent_goal_y{0.0};
    double field_length{9.0};
    double field_width{6.0};
    double obstacle_clearance{0.25};
    double max_receiver_distance{3.0};
    double kicker_switch_hysteresis{0.4};
    double approach_standoff{0.30};
    std::vector<FieldPoint> obstacles;
};

struct KickDecision
{
    bool ball_available{false};
    int kicker_id{0};
    bool is_kicker{false};
    bool receive_target_valid{false};
    double kick_origin_x{0.0};
    double kick_origin_y{0.0};
    double pass_heading{0.0};
    double receive_x{0.0};
    double receive_y{0.0};
};

class KickCoordinator
{
  public:
    void reset()
    {
        last_kicker_id_ = 0;
    }

    KickDecision decide(const KickCoordinationInput &in, double kick_distance);

  private:
    int last_kicker_id_{0};
};

} // namespace task_planner
