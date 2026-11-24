#include "task_planner/task_planner.hpp"

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

    input_.ball_detected = msg->ball_detected;
    input_.ball_global_x = msg->ball_pose.x;
    input_.ball_global_y = msg->ball_pose.y;
}
void TaskPlanner::ik_kick_end_callback(const std_msgs::msg::Bool msg)
{
    if (msg.data)
        input_.motion_done = true;
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

    const FsmOutput out = fsm_.update(input_);

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
        publish_nav_command(out.nav);
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
