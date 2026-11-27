#include "task_planner/task_planner.hpp"

#include <cmath>

namespace task_planner
{

TaskPlanner::TaskPlanner() : Node("task_planner")
{
    // yaml 받아오기
    gamecontroller2master_topic_name =
        declare_parameter<std::string>("gamecontroller2master_topic_name", "gamecontroldata");
    master2ik_topic_name = declare_parameter<std::string>("master2ik_topic_name", "master2ik");
    navCommand_topic_name = declare_parameter<std::string>("navCommand_topic_name", "nav_command");
    navFeedback_topic_name = declare_parameter<std::string>("navFeedback_topic_name", "nav_feedback");
    visionCommand_topic_name = declare_parameter<std::string>("visionCommand_topic_name", "vision_command");
    // visionFeedback_topic_name = declare_parameter<std::string>("visionFeedback_topic_name", "vision_feedback");
    vision_observ_topic_name = declare_parameter<std::string>("vision_observ_topic_name", "/vision_packages");
    worldModel_topic_name = declare_parameter<std::string>("worldModel_topic_name", "world_model");
    ik_kick_topic_name = declare_parameter<std::string>("ik_kick_topic_name", "kick_flag");
    ik_kick_end_topic_name = declare_parameter<std::string>("ik_kick_end_topic_name", "kick_flag_end");

    set_yaw_zero_topic_name = declare_parameter<std::string>("set_yaw_zero_topic_name", "/imu/set_yaw_0");
    set_yaw_90_topic_name = declare_parameter<std::string>("set_yaw_90_topic_name", "/imu/set_yaw_90");
    set_yaw_m90_topic_name = declare_parameter<std::string>("set_yaw_m90_topic_name", "/imu/set_yaw_m90");

    teamDataTx_topic_name = declare_parameter<std::string>("teamDataTx_topic_name", "/TeamData");
    teamDataRx_topic_name = declare_parameter<std::string>("teamDataRx_topic_name", "/Team2Data");
    robot_id_ = declare_parameter<int>("robot_id", 1);

    FsmParams fsm_params;
    fsm_params.pass_kick_distance = declare_parameter<double>("pass_kick_distance", 1.5);
    fsm_params.pass_receive_timeout = declare_parameter<double>("pass_receive_timeout", 3.0);
    fsm_params.opponent_goal_x = declare_parameter<double>("opponent_goal_x", 4.5);
    fsm_params.opponent_goal_y = declare_parameter<double>("opponent_goal_y", 0.0);
    fsm_params.field_length = declare_parameter<double>("field_length", 9.0);
    fsm_params.field_width = declare_parameter<double>("field_width", 6.0);
    fsm_params.pass_obstacle_clearance = declare_parameter<double>("pass_obstacle_clearance", 0.25);
    fsm_params.pass_receiver_max_distance = declare_parameter<double>("pass_receiver_max_distance", 3.0);
    fsm_params.pass_kicker_switch_hysteresis =
        declare_parameter<double>("pass_kicker_switch_hysteresis", 0.4);
    fsm_params.pass_approach_standoff = declare_parameter<double>("pass_approach_standoff", 0.30);
    fsm_ = TaskFsm(fsm_params);

    // topic pub, sub, QoS 추후 변경
    ControlData_sub = this->create_subscription<controlData>(
        gamecontroller2master_topic_name, 10,
        std::bind(&TaskPlanner::control_data_callback, this, std::placeholders::_1));
    Master2Ik_pub = this->create_publisher<M2Ik>(master2ik_topic_name, 10);
    NavCommand_pub = this->create_publisher<navComm>(navCommand_topic_name, 10);
    NavFeedback_sub = this->create_subscription<navFeedback>(
        navFeedback_topic_name, 10, std::bind(&TaskPlanner::navigation_feedback_callback, this, std::placeholders::_1));
    VisionCommand_pub = this->create_publisher<VSComm>(visionCommand_topic_name, 10);
    // VisionFeedback_sub = this->create_subscription<VSFeedback>(
    //     visionFeedback_topic_name, 10, std::bind(&TaskPlanner::vision_feedback_callback, this,
    //     std::placeholders::_1));
    VisionObserv_sub = this->create_subscription<VSObserv>(
        vision_observ_topic_name, 10, std::bind(&TaskPlanner::vision_observ_callback, this, std::placeholders::_1));
    WorldModel_sub = this->create_subscription<worldModel>(
        worldModel_topic_name, 10, std::bind(&TaskPlanner::world_model_callback, this, std::placeholders::_1));
    IK_Kick_pub = this->create_publisher<std_msgs::msg::String>(ik_kick_topic_name, 10);
    IK_Kick_end_sub = this->create_subscription<std_msgs::msg::Bool>(
        ik_kick_end_topic_name, 10, std::bind(&TaskPlanner::ik_kick_end_callback, this, std::placeholders::_1));

    set_yaw_zero_pub = this->create_publisher<std_msgs::msg::Empty>(set_yaw_zero_topic_name, 10);
    set_yaw_90_pub = this->create_publisher<std_msgs::msg::Empty>(set_yaw_90_topic_name, 10);
    set_yaw_m90_pub = this->create_publisher<std_msgs::msg::Empty>(set_yaw_m90_topic_name, 10);

    teamDataTx_pub = this->create_publisher<teamData>(teamDataTx_topic_name, 10);
    teamDataRx_sub = this->create_subscription<teamData>(
        teamDataRx_topic_name, 10, std::bind(&TaskPlanner::team_data_callback, this, std::placeholders::_1));

    control_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(1000 / 20), std::bind(&TaskPlanner::control_loop, this));
}

TaskPlanner::~TaskPlanner()
{
}

// subscribe callback & publish
void TaskPlanner::control_data_callback(const controlData::SharedPtr msg)
{
    state = msg->state;
    input_.gc_state = msg->state;
}
void TaskPlanner::navigation_feedback_callback(const navFeedback::SharedPtr msg)
{
    input_.nav_arrived = msg->status == 2 ? 1 : 0;
}
// void TaskPlanner::vision_feedback_callback(const VSFeedback::SharedPtr msg)
// {
//     int temp = msg->is_aligned;
// }
void TaskPlanner::vision_observ_callback(const VSObserv::SharedPtr msg)
{
    input_.ball_rel_x = msg->ball[0]; // x
    input_.ball_rel_y = msg->ball[1]; // y
}
void TaskPlanner::world_model_callback(const worldModel::SharedPtr msg)
{
    last_vision_time_ = this->now();

    has_world_model_ = true;
    input_.robot_global_x = msg->robot_pose.x;
    input_.robot_global_y = msg->robot_pose.y;
    input_.robot_global_theta = msg->robot_pose.theta;
    input_.ball_detected = msg->ball_detected;
    input_.ball_global_x = msg->ball_pose.x;
    input_.ball_global_y = msg->ball_pose.y;
    input_.obstacles.clear();
    input_.obstacles.reserve(msg->obstacles.size());
    for (const auto &obstacle : msg->obstacles)
    {
        input_.obstacles.push_back({obstacle.x, obstacle.y});
    }
}
void TaskPlanner::ik_kick_end_callback(const std_msgs::msg::Bool msg)
{
    if (msg.data)
        input_.motion_done = true;
}
void TaskPlanner::team_data_callback(const teamData &msg)
{
    input_.teammate_global_theta = msg.robot_theta;
    input_.teammate_main_state = msg.main_state;
    input_.teammate_sub_state = msg.sub_state;
    input_.teammate_robot_id = msg.robot_id;
    input_.teammate_pass_count = msg.pass_count;
    input_.teammate_robot_global_x = msg.robot_x;
    input_.teammate_robot_global_y = msg.robot_y;
    input_.teammate_ball_detected = msg.ball_detected;
    input_.teammate_ball_global_x = msg.ball_x;
    input_.teammate_ball_global_y = msg.ball_y;
    last_teammate_data_time_ = std::chrono::steady_clock::now();
    has_teammate_pose_ = true;
}
void TaskPlanner::publish_nav_command(const NavCmdOut &nav)
{
    navComm msg;
    msg.nav_enable = nav.enable;
    msg.x = nav.x;
    msg.y = nav.y;
    msg.theta = nav.theta;
    msg.pos_tolerance = nav.pos_tol;
    msg.theta_tolerance = nav.theta_tol;
    msg.forbid_right_half = nav.forbid_right_half;
    msg.avoid_ball = nav.avoid_ball;
    NavCommand_pub->publish(msg);
}
void TaskPlanner::publish_walk(double vx, double vy, double vyaw)
{
    M2Ik msg;
    msg.x_length = vx;
    msg.y_length = vy;
    msg.yaw = vyaw;
    Master2Ik_pub->publish(msg);
}
void TaskPlanner::publish_kick(const KickFoot &foot)
{
    std_msgs::msg::String msg;
    msg.data = (foot == KickFoot::LEFT) ? "left" : "right";
    IK_Kick_pub->publish(msg);
}
void TaskPlanner::publish_vision_command(int comm)
{
    VSComm msg;
    msg.vision_mode = comm;
    VisionCommand_pub->publish(msg);
}
void TaskPlanner::publish_set_yaw(int degree)
{
    std_msgs::msg::Empty msg;
    if (degree == 0)
        set_yaw_zero_pub->publish(msg);
    else if (degree == 90)
        set_yaw_90_pub->publish(msg);
    else if (degree == -90)
        set_yaw_m90_pub->publish(msg);
}
void TaskPlanner::publish_team_data_tx(const teamData &team_data)
{
    teamDataTx_pub->publish(team_data);
}

bool TaskPlanner::is_vision_fresh() const
{
    if (last_vision_time_.nanoseconds() == 0) // 호출되지 않았을 경우
    {
        return false;
    }
    // 너무 delay된 데이터는 쓰지 않음
    return (this->now() - last_vision_time_).seconds() <= vision_timeout_;
}

void TaskPlanner::control_loop()
{
    input_.is_vision_fresh = is_vision_fresh();
    input_.robot_id = robot_id_;
    const auto now = std::chrono::steady_clock::now();
    input_.now_seconds = std::chrono::duration<double>(now.time_since_epoch()).count();
    input_.teammate_data_fresh = has_teammate_pose_ && now - last_teammate_data_time_ <= std::chrono::seconds(1);

    const FsmOutput out = fsm_.update(input_);
    if (has_world_model_)
    {
        teamData team_data;
        team_data.robot_id = static_cast<uint8_t>(robot_id_);
        team_data.main_state = static_cast<uint8_t>(fsm_.main_state());
        team_data.sub_state = static_cast<uint8_t>(fsm_.play_sub_state());
        team_data.pass_count = static_cast<uint8_t>(fsm_.pass_count());
        team_data.robot_x = input_.robot_global_x;
        team_data.robot_y = input_.robot_global_y;
        team_data.robot_theta = input_.robot_global_theta;
        team_data.ball_detected = input_.ball_detected && input_.is_vision_fresh;
        team_data.ball_x = input_.ball_global_x;
        team_data.ball_y = input_.ball_global_y;
        team_data.ball_dist = team_data.ball_detected ? std::hypot(input_.ball_global_x - input_.robot_global_x,
                                                                   input_.ball_global_y - input_.robot_global_y)
                                                      : 0.0;
        publish_team_data_tx(team_data);
    }

    if (out.warn_ready_ignored)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "페널티 후에는 SET 또는 PLAYING으로 전환");
    }
    if (out.entered_first_initial)
    {
        RCLCPP_INFO(this->get_logger(), "경기 시작 전 초기 대기");
    }
    if (out.entered_penalty_initial)
    {
        RCLCPP_WARN(this->get_logger(), "페널티 복귀 (pass_count 리셋)");
    }
    if (out.send_set_yaw)
    {
        switch (robot_id_)
        {
        case 1: // 상단
            publish_set_yaw(90);
            break;

        case 2: // 하단 (id: 2)
            publish_set_yaw(-90);
            break;
        }
        yaw_settle_deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
        yaw_settle_pending_ = true;
    }
    const bool yaw_settle_delay_active = yaw_settle_pending_ && std::chrono::steady_clock::now() < yaw_settle_deadline_;
    if (yaw_settle_pending_ && !yaw_settle_delay_active)
    {
        yaw_settle_pending_ = false;
    }

    if (out.reset_motion_done) // kick end
    {
        input_.motion_done = false;
    }
    if (out.send_nav && out.nav.enable && out.send_walk)
    {
        RCLCPP_ERROR(this->get_logger(), "nav.active와 send_walk가 동시에 활성화됨");
        return;
    }
    if (out.send_nav) // send_nav가 false일 경우 nav에서 보행 topic 발행 X
    {
        if (!out.nav.enable || !yaw_settle_delay_active)
        {
            publish_nav_command(out.nav);
        }
    }
    if (out.send_walk) // 직접 전달 시
    {
        publish_walk(out.vx, out.vy, out.vyaw);
    }
    if (out.send_vision_cmd)
    {
        publish_vision_command(out.vision_mode);
    }
    if (out.send_kick) // kick foot
    {
        publish_kick(out.kick_foot);
    }
}

} // namespace task_planner

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<task_planner::TaskPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
