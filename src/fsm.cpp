#include "task_planner/fsm.hpp"

#include <cmath>

namespace task_planner
{

TaskFsm::TaskFsm(const FsmParams &params) : params_(params)
{
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
        const float target_theta = (pass_count_ == 0) ? in.teammate_global_theta : in.goal_global_theta;
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
    }
}

void TaskFsm::handle_playing(const FsmInput &in, FsmOutput &out)
{
    const bool valid_ball = in.is_vision_fresh && in.ball_detected;

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

    const double ball_dist = std::hypot(in.ball_rel_x, in.ball_rel_y);
    const bool has_valid_rel_ball = (ball_dist > 1e-4); // 0 거르기

    switch (play_sub_state_)
    {
    case PlaySubState::SEARCH:
        if (has_valid_rel_ball && detect_count_ >= params_.min_detect_frames)
        {
            if (ball_dist <= params_.d_enter)
            {
                change_play_sub_state(PlaySubState::ALIGN, in, out);
            }
            else
            {
                change_play_sub_state(PlaySubState::APPROACH, in, out);
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
        if (lost_count_ > params_.max_lost_frames)
        {
            change_play_sub_state(PlaySubState::SEARCH, in, out);
            return;
        }
        if (valid_ball && has_valid_rel_ball && ball_dist <= params_.d_enter)
        {
            change_play_sub_state(PlaySubState::ALIGN, in, out);
            return;
        }

        const float target_theta = (pass_count_ == 0) ? in.teammate_global_theta : in.goal_global_theta;
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

    case PlaySubState::ALIGN: {
        if (lost_count_ > params_.max_lost_frames)
        {
            change_play_sub_state(PlaySubState::SEARCH, in, out);
            return;
        }
        if (valid_ball && has_valid_rel_ball && ball_dist > params_.d_exit)
        {
            change_play_sub_state(PlaySubState::APPROACH, in, out);
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
        align_in.yaw_err = (pass_count_ == 0) ? in.teammate_yaw_err : in.goal_yaw_err;

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
            pass_count_++;
            change_play_sub_state(PlaySubState::SEARCH, in, out);
        }
        break;

    case PlaySubState::SHOOT_KICK:
        if (in.motion_done)
        {
            change_play_sub_state(PlaySubState::SEARCH, in, out);
        }
        break;
    }
}

} // namespace task_planner