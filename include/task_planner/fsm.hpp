#pragma once

#include <vector>

#include "task_planner/kick_coordinator.hpp"
#include "task_planner/vision_alignment.hpp"

namespace task_planner
{

enum class MainState : int
{
    INITIAL = 0,
    READY = 1,
    SET = 2,
    PLAYING = 3,
    FINISHED = 4
};

enum class PlaySubState : int
{
    SEARCH = 0,
    APPROACH = 1,
    ALIGN = 2,
    PASS_KICK = 3,
    SHOOT_KICK = 4,
    RECEIVE = 5,
    PASS_WAIT = 6,
    SUPPORT = 7
};

struct FsmParams
{
    double d_enter = 0.45; // APPROACH -> ALIGN 진입 거리 (m) (히스테리시스 적용)
    double d_exit = 0.60;  // ALIGN -> APPROACH 이탈 거리 (m)
    int min_detect_frames = 3;
    int max_lost_frames = 15;
    int min_align_frames = 5;
    double pass_kick_distance = 1.5;   // m
    double pass_receive_timeout = 3.0; // s
    double opponent_goal_x = 4.5;
    double opponent_goal_y = 0.0;
    double field_length = 9.0;
    double field_width = 6.0;
    double pass_obstacle_clearance = 0.25;
    double pass_receiver_max_distance = 3.0;
    double pass_kicker_switch_hysteresis = 0.4;
    double pass_approach_standoff = 0.30;

    // READY 상태 LEFT 진영 내 목표 준비 위치
    float ready_target_x = -2.0f;
    float ready_target_y = 0.0f;
    float ready_target_theta = 0.0f;
};

struct FsmInput
{
    int gc_state = 0; // INITIAL: 0, READY: 1, SET: 2, PLAYING: 3, FINISHED: 4
    bool nav_arrived = false;
    bool motion_done = false;

    bool is_vision_fresh = false;
    bool ball_detected = false;
    double ball_rel_x = 0.0;
    double ball_rel_y = 0.0;

    float robot_global_x = 0.0f;
    float robot_global_y = 0.0f;
    float robot_global_theta = 0.0f;
    float ball_global_x = 0.0f;
    float ball_global_y = 0.0f;
    std::vector<FieldPoint> obstacles;
    float teammate_global_theta = 0.0f;
    float teammate_robot_global_x = 0.0f;
    float teammate_robot_global_y = 0.0f;
    int robot_id = 1;
    bool teammate_data_fresh = false;
    int teammate_main_state = 0;
    int teammate_sub_state = 0;
    int teammate_robot_id = 2;
    int teammate_pass_count = 0;
    bool teammate_ball_detected = false;
    double teammate_ball_global_x = 0.0;
    double teammate_ball_global_y = 0.0;
    double now_seconds = 0.0;
};

struct NavCmdOut
{
    bool enable = false;
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
    float pos_tol = 0.15f;   // m
    float theta_tol = 0.20f; // rad
    bool forbid_right_half = false;
    bool avoid_ball = false;
};

struct FsmOutput
{
    bool send_nav = false;
    NavCmdOut nav;

    bool send_walk = false;
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;

    bool send_kick = false;
    bool is_shoot = false;
    KickFoot kick_foot = KickFoot::RIGHT;
    bool reset_motion_done = false;

    bool send_vision_cmd = false;
    int vision_mode = 0; // 1: vision search, 2: vision align

    bool send_set_yaw = false;

    // 상태 전이 이벤트 플래그 (로깅/디버깅용)
    bool warn_ready_ignored = false;
    bool entered_first_initial = false;
    bool entered_penalty_initial = false;
};

class TaskFsm
{
  public:
    explicit TaskFsm(const FsmParams &params = FsmParams{});

    FsmOutput update(const FsmInput &in);

    MainState main_state() const
    {
        return main_state_;
    }
    PlaySubState play_sub_state() const
    {
        return play_sub_state_;
    }
    int pass_count() const
    {
        return pass_count_;
    }

  private:
    void change_main_state(MainState next_main, FsmOutput &out);
    void change_play_sub_state(PlaySubState next_sub, const FsmInput &in, FsmOutput &out);
    void handle_playing(const FsmInput &in, FsmOutput &out);

    FsmParams params_;
    KickCoordinator kick_coordinator_;
    VisionAlignment vision_aligner_;

    MainState main_state_{MainState::INITIAL};
    PlaySubState play_sub_state_{PlaySubState::SEARCH};

    bool has_game_started_{false};
    int pass_count_{0};
    KickFoot active_kick_foot_{KickFoot::RIGHT};

    int detect_count_{0};
    int lost_count_{0};
    int align_count_{0};
    double receive_wait_started_at_{-1.0};
    double pass_wait_started_at_{-1.0};
    bool receive_timed_out_{false};
    bool receive_target_frozen_{false};
    double receive_target_x_{0.0};
    double receive_target_y_{0.0};
    double receive_origin_x_{0.0};
    double receive_origin_y_{0.0};
};

} // namespace task_planner