#include "task_planner/fsm.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace task_planner
{

TaskFsm::TaskFsm(const FsmParams &params) : params_(params)
{
    if (!std::isfinite(params_.pass_kick_distance) || params_.pass_kick_distance <= 0.0 ||
        !std::isfinite(params_.pass_receive_timeout) || params_.pass_receive_timeout < 0.0 ||
        !std::isfinite(params_.opponent_goal_x) || !std::isfinite(params_.opponent_goal_y) ||
        !std::isfinite(params_.field_length) || !std::isfinite(params_.field_width) ||
        params_.field_length <= 0.0 || params_.field_width <= 0.0 ||
        !std::isfinite(params_.pass_obstacle_clearance) || params_.pass_obstacle_clearance < 0.0 ||
        !std::isfinite(params_.pass_receiver_max_distance) || params_.pass_receiver_max_distance < 0.0 ||
        !std::isfinite(params_.pass_kicker_switch_hysteresis) || params_.pass_kicker_switch_hysteresis < 0.0 ||
        !std::isfinite(params_.pass_approach_standoff) || params_.pass_approach_standoff < 0.0)
    {
        throw std::invalid_argument("invalid pass-planning parameters");
    }
}

FsmOutput TaskFsm::update(const FsmInput &in)
{
    FsmOutput out;

    if (in.gc_state >= 0 && in.gc_state <= 4)
    {
        change_main_state(static_cast<MainState>(in.gc_state), out);
    }

    switch (main_state_)
    {
    case MainState::INITIAL:
    case MainState::SET:
    case MainState::FINISHED: // 정지
        out.send_nav = true;
        out.nav.enable = false;
        out.send_walk = true;
        out.vx = 0.0;
        out.vy = 0.0;
        out.vyaw = 0.0;
        break;

    case MainState::READY:
        if (in.nav_arrived)
        {
            out.send_nav = true;
            out.nav.enable = false;
            out.send_walk = true;
            out.vx = 0.0;
            out.vy = 0.0;
            out.vyaw = 0.0;
        }
        else // left 진영 내 지정된 위치로 이동
        {
            out.send_nav = true;
            out.nav = {true,
                       params_.ready_target_x,
                       params_.ready_target_y,
                       params_.ready_target_theta,
                       0.10f,
                       0.15f,
                       /*forbid_right_half=*/true,
                       /*avoid_ball=*/true};
        }
        break;

    case MainState::PLAYING:
        handle_playing(in, out);
        break;
    }

    return out;
}

void TaskFsm::change_main_state(MainState next_main, FsmOutput &out)
{
    if (main_state_ == next_main)
    {
        return;
    }

    if (next_main == MainState::READY && has_game_started_) // initial 구분용
    {
        out.warn_ready_ignored = true;
        return;
    }

    main_state_ = next_main;
    align_count_ = 0;
    kick_coordinator_.reset();
    receive_wait_started_at_ = -1.0;
    pass_wait_started_at_ = -1.0;
    receive_timed_out_ = false;
    receive_target_frozen_ = false;

    switch (main_state_)
    {
    case MainState::INITIAL:
        pass_count_ = 0;
        out.send_nav = true;
        out.nav.enable = false;
        out.send_walk = true;
        out.vx = 0.0;
        out.vy = 0.0;
        out.vyaw = 0.0;

        if (!has_game_started_)
        {
            out.entered_first_initial = true;
        }
        else
        {
            out.entered_penalty_initial = true;
        }
        break;

    case MainState::READY:
        has_game_started_ = true;
        out.send_set_yaw = true;
        out.send_nav = true;
        out.nav = {true,
                   params_.ready_target_x,
                   params_.ready_target_y,
                   params_.ready_target_theta,
                   0.10f,
                   0.15f,
                   /*forbid_right_half=*/true,
                   /*avoid_ball=*/true};
        break;

    case MainState::SET:
        out.send_nav = true;
        out.nav.enable = false;
        out.send_walk = true;
        out.vx = 0.0;
        out.vy = 0.0;
        out.vyaw = 0.0;
        break;

    case MainState::PLAYING: {
        has_game_started_ = true;
        FsmInput dummy{}; // agument 맞추기
        change_play_sub_state(PlaySubState::SEARCH, dummy, out);
        break;
    }

    case MainState::FINISHED:
        has_game_started_ = false;
        out.send_nav = true;
        out.nav.enable = false;
        out.send_walk = true;
        out.vx = 0.0;
        out.vy = 0.0;
        out.vyaw = 0.0;
        break;
    }
}

void TaskFsm::change_play_sub_state(PlaySubState next_sub, const FsmInput &in, FsmOutput &out)
{
    play_sub_state_ = next_sub;
    align_count_ = 0;

    switch (play_sub_state_)
    {
    case PlaySubState::SEARCH:
        out.send_nav = true;
        out.nav.enable = false;
        out.send_vision_cmd = true;
        out.vision_mode = 1;
        break;

    case PlaySubState::APPROACH: {
        const float target_theta =
            static_cast<float>(std::atan2(params_.opponent_goal_y - in.ball_global_y,
                                          params_.opponent_goal_x - in.ball_global_x));
        out.send_nav = true;
        out.nav = {true,
                   in.ball_global_x,
                   in.ball_global_y,
                   target_theta,
                   0.20f,
                   0.25f,
                   /*forbid_right_half=*/false,
                   /*avoid_ball=*/false};
        break;
    }

    case PlaySubState::ALIGN:
        out.send_nav = true;
        out.nav.enable = false;
        vision_aligner_.reset();
        out.send_vision_cmd = true;
        out.vision_mode = 2;
        break;

    case PlaySubState::PASS_KICK:
        out.send_nav = true;
        out.nav.enable = false;
        out.send_walk = true;
        out.vx = 0.0;
        out.vy = 0.0;
        out.vyaw = 0.0;
        out.reset_motion_done = true;
        out.send_kick = true;
        out.is_shoot = false;
        out.kick_foot = active_kick_foot_;
        break;

    case PlaySubState::SHOOT_KICK:
        out.send_nav = true;
        out.nav.enable = false;
        out.send_walk = true;
        out.vx = 0.0;
        out.vy = 0.0;
        out.vyaw = 0.0;
        out.reset_motion_done = true;
        out.send_kick = true;
        out.is_shoot = true;
        out.kick_foot = active_kick_foot_;
        break;

    case PlaySubState::RECEIVE:
        out.send_nav = true;
        out.nav.enable = false;
        receive_target_frozen_ = false;
        break;

    case PlaySubState::PASS_WAIT:
    case PlaySubState::SUPPORT:
        out.send_nav = true;
        out.nav.enable = false;
        out.send_walk = true;
        out.vx = 0.0;
        out.vy = 0.0;
        out.vyaw = 0.0;
        break;
    }
}

void TaskFsm::handle_playing(const FsmInput &in, FsmOutput &out)
{
    const bool valid_ball = in.is_vision_fresh && in.ball_detected;
    const double ball_dist = std::hypot(in.ball_rel_x, in.ball_rel_y);
    const bool has_valid_rel_ball = (ball_dist > 1e-4);

    KickCoordinationInput kick_input;
    kick_input.robot_id = in.robot_id;
    kick_input.ball_detected = valid_ball;
    kick_input.ball_x = in.ball_global_x;
    kick_input.ball_y = in.ball_global_y;
    kick_input.robot_x = in.robot_global_x;
    kick_input.robot_y = in.robot_global_y;
    kick_input.robot_theta = in.robot_global_theta;
    kick_input.teammate_data_fresh = in.teammate_data_fresh;
    kick_input.teammate_playing = in.teammate_main_state == static_cast<int>(MainState::PLAYING);
    kick_input.teammate_robot_id = in.teammate_robot_id;
    kick_input.teammate_robot_x = in.teammate_robot_global_x;
    kick_input.teammate_robot_y = in.teammate_robot_global_y;
    kick_input.teammate_ball_detected = in.teammate_data_fresh && in.teammate_ball_detected;
    kick_input.teammate_ball_x = in.teammate_ball_global_x;
    kick_input.teammate_ball_y = in.teammate_ball_global_y;
    kick_input.teammate_robot_theta = in.teammate_global_theta;
    kick_input.opponent_goal_x = params_.opponent_goal_x;
    kick_input.opponent_goal_y = params_.opponent_goal_y;
    kick_input.field_length = params_.field_length;
    kick_input.field_width = params_.field_width;
    kick_input.obstacle_clearance = params_.pass_obstacle_clearance;
    kick_input.max_receiver_distance = params_.pass_receiver_max_distance;
    kick_input.kicker_switch_hysteresis = params_.pass_kicker_switch_hysteresis;
    kick_input.approach_standoff = params_.pass_approach_standoff;
    kick_input.obstacles = in.obstacles;
    KickDecision kick_decision = kick_coordinator_.decide(kick_input, params_.pass_kick_distance);
    const bool teammate_is_passing =
        in.teammate_data_fresh && (in.teammate_sub_state == static_cast<int>(PlaySubState::PASS_KICK) ||
                                   in.teammate_sub_state == static_cast<int>(PlaySubState::PASS_WAIT));
    if (teammate_is_passing)
    {
        const double pass_ball_x = kick_decision.ball_available
                                       ? kick_decision.kick_origin_x
                                       : (valid_ball ? in.ball_global_x : in.teammate_ball_global_x);
        const double pass_ball_y = kick_decision.ball_available
                                       ? kick_decision.kick_origin_y
                                       : (valid_ball ? in.ball_global_y : in.teammate_ball_global_y);
        kick_decision.kick_origin_x = pass_ball_x;
        kick_decision.kick_origin_y = pass_ball_y;
        kick_decision.receive_x =
            pass_ball_x + params_.pass_kick_distance * std::cos(in.teammate_global_theta);
        kick_decision.receive_y =
            pass_ball_y + params_.pass_kick_distance * std::sin(in.teammate_global_theta);
        kick_decision.receive_target_valid = true;
    }

    const auto set_approach_command = [&]() {
        const double goal_heading =
            std::atan2(params_.opponent_goal_y - kick_decision.kick_origin_y,
                       params_.opponent_goal_x - kick_decision.kick_origin_x);
        const double target_theta =
            pass_count_ == 0 && kick_decision.receive_target_valid ? kick_decision.pass_heading : goal_heading;
        const double approach_x =
            kick_decision.kick_origin_x - params_.pass_approach_standoff * std::cos(target_theta);
        const double approach_y =
            kick_decision.kick_origin_y - params_.pass_approach_standoff * std::sin(target_theta);
        out.send_nav = true;
        out.nav = {true, static_cast<float>(approach_x), static_cast<float>(approach_y),
                   static_cast<float>(target_theta), 0.20f, 0.25f, /*forbid_right_half=*/false,
                   /*avoid_ball=*/false};
    };

    if (valid_ball)
    {
        detect_count_++;
        lost_count_ = 0;
    }
    else
    {
        lost_count_++;
        detect_count_ = 0;
        align_count_ = 0;
    }

    const bool teammate_pass_active = in.teammate_data_fresh && in.teammate_pass_count > 0;
    const bool pass_in_progress =
        play_sub_state_ == PlaySubState::PASS_KICK || play_sub_state_ == PlaySubState::PASS_WAIT ||
        play_sub_state_ == PlaySubState::SHOOT_KICK || pass_count_ > 0 || teammate_pass_active ||
        teammate_is_passing;
    if (!kick_decision.ball_available && !pass_in_progress)
    {
        receive_wait_started_at_ = -1.0;
        receive_target_frozen_ = false;
        receive_timed_out_ = false;
        if (play_sub_state_ != PlaySubState::SEARCH)
        {
            change_play_sub_state(PlaySubState::SEARCH, in, out);
        }
    }

    if (in.teammate_sub_state != static_cast<int>(PlaySubState::PASS_WAIT))
    {
        receive_timed_out_ = false;
    }

    if (play_sub_state_ == PlaySubState::PASS_WAIT)
    {
        const bool receiver_started_attack = in.teammate_data_fresh && in.teammate_pass_count >= pass_count_ &&
                                             in.teammate_pass_count > 0 &&
                                             in.teammate_sub_state != static_cast<int>(PlaySubState::RECEIVE);
        const bool receiver_aborted = in.teammate_data_fresh &&
                                      in.teammate_sub_state != static_cast<int>(PlaySubState::RECEIVE) &&
                                      in.teammate_pass_count == 0;
        const bool timed_out =
            pass_wait_started_at_ >= 0.0 && in.now_seconds - pass_wait_started_at_ >= params_.pass_receive_timeout;

        if (receiver_started_attack)
        {
            change_play_sub_state(PlaySubState::SUPPORT, in, out);
        }
        else if (receiver_aborted || timed_out)
        {
            pass_count_ = 0;
            pass_wait_started_at_ = -1.0;
            change_play_sub_state(PlaySubState::SEARCH, in, out);
        }
        return;
    }

    if (play_sub_state_ == PlaySubState::SUPPORT)
    {
        const bool teammate_ready_for_new_play = in.teammate_data_fresh && in.teammate_pass_count == 0 &&
                                                 in.teammate_sub_state == static_cast<int>(PlaySubState::SEARCH) &&
                                                 kick_decision.is_kicker;
        if (teammate_ready_for_new_play)
        {
            pass_count_ = 0;
            change_play_sub_state(PlaySubState::SEARCH, in, out);
        }
        else
        {
            out.send_nav = true;
            out.nav.enable = false;
            out.send_walk = true;
            out.vx = 0.0;
            out.vy = 0.0;
            out.vyaw = 0.0;
            return;
        }
    }

    const bool in_kick_motion =
        play_sub_state_ == PlaySubState::PASS_KICK || play_sub_state_ == PlaySubState::SHOOT_KICK;
    if (kick_decision.ball_available && !in_kick_motion && play_sub_state_ != PlaySubState::RECEIVE &&
        !kick_decision.is_kicker &&
        !receive_timed_out_ && pass_count_ == 0 &&
        (!in.teammate_data_fresh || in.teammate_pass_count == 0))
    {
        change_play_sub_state(PlaySubState::RECEIVE, in, out);
    }
    else if (kick_decision.ball_available && !in_kick_motion && play_sub_state_ != PlaySubState::RECEIVE &&
             !kick_decision.is_kicker &&
             !receive_timed_out_)
    {
        change_play_sub_state(PlaySubState::SUPPORT, in, out);
        return;
    }
    else if (play_sub_state_ == PlaySubState::RECEIVE && kick_decision.is_kicker &&
             (!in.teammate_data_fresh ||
              (in.teammate_sub_state != static_cast<int>(PlaySubState::PASS_KICK) &&
               in.teammate_sub_state != static_cast<int>(PlaySubState::PASS_WAIT))))
    {
        receive_wait_started_at_ = -1.0;
        change_play_sub_state(PlaySubState::SEARCH, in, out);
    }

    switch (play_sub_state_)
    {
    case PlaySubState::SEARCH:
        if (kick_decision.ball_available && kick_decision.is_kicker && !valid_ball)
        {
            change_play_sub_state(PlaySubState::APPROACH, in, out);
            set_approach_command();
            return;
        }
        if (valid_ball && has_valid_rel_ball && detect_count_ >= params_.min_detect_frames)
        {
            if (ball_dist <= params_.d_enter)
            {
                change_play_sub_state(PlaySubState::ALIGN, in, out);
            }
            else
            {
                change_play_sub_state(PlaySubState::APPROACH, in, out);
                set_approach_command();
            }
            return;
        }
        out.send_nav = true;
        out.nav.enable = false;
        out.send_walk = true;
        out.vx = 0.0;
        out.vy = 0.0;
        out.vyaw = 0.25;
        break;

    case PlaySubState::APPROACH: {
        if (lost_count_ > params_.max_lost_frames && !kick_decision.ball_available)
        {
            change_play_sub_state(PlaySubState::SEARCH, in, out);
            return;
        }
        if (valid_ball && has_valid_rel_ball && ball_dist <= params_.d_enter)
        {
            change_play_sub_state(PlaySubState::ALIGN, in, out);
            return;
        }

        set_approach_command();
        break;
    }

    case PlaySubState::ALIGN: {
        if (lost_count_ > params_.max_lost_frames)
        {
            change_play_sub_state(PlaySubState::SEARCH, in, out);
            return;
        }
        if (valid_ball && has_valid_rel_ball && ball_dist > params_.d_exit)
        {
            change_play_sub_state(PlaySubState::APPROACH, in, out);
            set_approach_command();
            return;
        }

        out.send_nav = true;
        out.nav.enable = false;

        if (!valid_ball || !has_valid_rel_ball)
        {
            out.send_walk = true;
            out.vx = 0.0;
            out.vy = 0.0;
            out.vyaw = 0.0;
            return;
        }

        AlignInput align_in;
        align_in.ball_x = in.ball_rel_x;
        align_in.ball_y = in.ball_rel_y;
        if (pass_count_ == 0 && !kick_decision.receive_target_valid)
        {
            out.send_walk = true;
            out.vx = 0.0;
            out.vy = 0.0;
            out.vyaw = 0.0;
            return;
        }
        const double target_heading =
            pass_count_ == 0
                ? kick_decision.pass_heading
                : std::atan2(params_.opponent_goal_y - in.ball_global_y,
                             params_.opponent_goal_x - in.ball_global_x);
        align_in.yaw_err = std::remainder(target_heading - in.robot_global_theta, 2.0 * std::acos(-1.0));

        const AlignOutput align_out = vision_aligner_.compute(align_in);

        if (align_out.is_aligned)
        {
            align_count_++;
            active_kick_foot_ = align_out.foot;
            out.send_walk = true;
            out.vx = 0.0;
            out.vy = 0.0;
            out.vyaw = 0.0;

            if (align_count_ >= params_.min_align_frames)
            {
                if (pass_count_ == 0)
                {
                    change_play_sub_state(PlaySubState::PASS_KICK, in, out);
                }
                else
                {
                    change_play_sub_state(PlaySubState::SHOOT_KICK, in, out);
                }
            }
        }
        else
        {
            align_count_ = 0;
            out.send_walk = true;
            out.vx = align_out.vx;
            out.vy = align_out.vy;
            out.vyaw = align_out.vyaw;
        }
        break;
    }

    case PlaySubState::PASS_KICK:
        if (in.motion_done)
        {
            pass_count_ = 1;
            pass_wait_started_at_ = in.now_seconds;
            change_play_sub_state(PlaySubState::PASS_WAIT, in, out);
        }
        break;

    case PlaySubState::SHOOT_KICK:
        if (in.motion_done)
        {
            pass_count_ = 0;
            change_play_sub_state(PlaySubState::SEARCH, in, out);
        }
        break;

    case PlaySubState::RECEIVE: {
        const bool pass_complete =
            in.teammate_data_fresh && in.teammate_sub_state == static_cast<int>(PlaySubState::PASS_WAIT);
        if (pass_complete && receive_wait_started_at_ < 0.0)
        {
            receive_wait_started_at_ = in.now_seconds;
        }

        if (pass_complete && valid_ball && has_valid_rel_ball && in.nav_arrived && ball_dist <= params_.d_enter)
        {
            pass_count_ = std::max(1, in.teammate_pass_count);
            receive_wait_started_at_ = -1.0;
            receive_timed_out_ = false;
            receive_target_frozen_ = false;
            change_play_sub_state(PlaySubState::SEARCH, in, out);
            out.send_nav = true;
            out.nav.enable = false;
            out.send_walk = true;
            out.vx = 0.0;
            out.vy = 0.0;
            out.vyaw = 0.0;
            return;
        }

        const bool receive_timed_out = receive_wait_started_at_ >= 0.0 &&
                                       in.now_seconds - receive_wait_started_at_ >= params_.pass_receive_timeout;
        if (receive_timed_out)
        {
            pass_count_ = 0;
            receive_wait_started_at_ = -1.0;
            receive_timed_out_ = true;
            receive_target_frozen_ = false;
            change_play_sub_state(PlaySubState::SEARCH, in, out);
            out.send_nav = true;
            out.nav.enable = false;
            out.send_walk = true;
            out.vx = 0.0;
            out.vy = 0.0;
            out.vyaw = 0.0;
            return;
        }

        if (teammate_is_passing && !receive_target_frozen_ && kick_decision.receive_target_valid)
        {
            receive_target_x_ = kick_decision.receive_x;
            receive_target_y_ = kick_decision.receive_y;
            receive_origin_x_ = kick_decision.kick_origin_x;
            receive_origin_y_ = kick_decision.kick_origin_y;
            receive_target_frozen_ = true;
        }
        const bool use_frozen_target = teammate_is_passing && receive_target_frozen_;
        const bool receive_target_valid = use_frozen_target || kick_decision.receive_target_valid;
        const double receive_x = use_frozen_target ? receive_target_x_ : kick_decision.receive_x;
        const double receive_y = use_frozen_target ? receive_target_y_ : kick_decision.receive_y;
        const double origin_x = use_frozen_target ? receive_origin_x_ : kick_decision.kick_origin_x;
        const double origin_y = use_frozen_target ? receive_origin_y_ : kick_decision.kick_origin_y;
        const double receive_theta = std::atan2(origin_y - receive_y, origin_x - receive_x);
        if (receive_target_valid)
        {
            out.send_nav = true;
            out.nav = {true,
                       static_cast<float>(receive_x),
                       static_cast<float>(receive_y),
                       static_cast<float>(receive_theta),
                       0.10f,
                       0.20f,
                       /*forbid_right_half=*/false,
                       /*avoid_ball=*/false};
        }
        else
        {
            out.send_nav = true;
            out.nav.enable = false;
            out.send_walk = true;
            out.vx = 0.0;
            out.vy = 0.0;
            out.vyaw = 0.0;
        }
        break;
    }

    case PlaySubState::PASS_WAIT:
    case PlaySubState::SUPPORT:
        break;
    }
}

} // namespace task_planner