#pragma once

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "task_planner/fsm.hpp"

#include "gamecontroller/msg/gamecontroldata.hpp"
#include "humanoid_interfaces/msg/master2_ik_msg.hpp"
#include "humanoid_interfaces/msg/nav_command.hpp"
#include "humanoid_interfaces/msg/nav_feedback.hpp"
#include "humanoid_interfaces/msg/vs_command.hpp"
// #include "humanoid_interfaces/msg/vs_feedback.hpp"
#include "humanoid_interfaces/msg/team_data.hpp"
#include "humanoid_interfaces/msg/vision2_local_observ.hpp"
#include "humanoid_interfaces/msg/world_model.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

namespace task_planner
{

class TaskPlanner : public rclcpp::Node
{

  public:
    TaskPlanner();
    ~TaskPlanner();

  private:
    using controlData = gamecontroller::msg::Gamecontroldata;
    using M2Ik = humanoid_interfaces::msg::Master2IkMsg;
    using navComm = humanoid_interfaces::msg::NavCommand;
    using navFeedback = humanoid_interfaces::msg::NavFeedback;
    using VSComm = humanoid_interfaces::msg::VSCommand;
    // using VSFeedback = humanoid_interfaces::msg::VSFeedback;
    using VSObserv = humanoid_interfaces::msg::Vision2LocalObserv;
    using worldModel = humanoid_interfaces::msg::WorldModel;
    using teamData = humanoid_interfaces::msg::TeamData;
    // TODO: 추가할 거
    // ta -> walk(kick, pass 등)
    rclcpp::Subscription<controlData>::SharedPtr ControlData_sub;
    std::string gamecontroller2master_topic_name;
    void control_data_callback(const controlData::SharedPtr msg);
    int state = 0; // INITIAL: 0, READY:1, SET: 2, PLAYING: 3, FINISHED: 4

    rclcpp::Publisher<M2Ik>::SharedPtr Master2Ik_pub;
    std::string master2ik_topic_name;
    void publish_walk(double vx, double vy, double vyaw);

    rclcpp::Publisher<navComm>::SharedPtr NavCommand_pub;
    std::string navCommand_topic_name;
    void publish_nav_command(const NavCmdOut &nav);
    rclcpp::Subscription<navFeedback>::SharedPtr NavFeedback_sub;
    std::string navFeedback_topic_name;
    void navigation_feedback_callback(const navFeedback::SharedPtr msg);

    rclcpp::Publisher<VSComm>::SharedPtr VisionCommand_pub;
    std::string visionCommand_topic_name;
    void publish_vision_command(int comm);
    // rclcpp::Subscription<VSFeedback>::SharedPtr VisionFeedback_sub;
    // std::string visionFeedback_topic_name;
    // void vision_feedback_callback(const VSFeedback::SharedPtr msg);

    rclcpp::Subscription<VSObserv>::SharedPtr VisionObserv_sub;
    std::string vision_observ_topic_name;
    void vision_observ_callback(const VSObserv::SharedPtr msg);

    rclcpp::Subscription<worldModel>::SharedPtr WorldModel_sub;
    std::string worldModel_topic_name;
    void world_model_callback(const worldModel::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr IK_Kick_pub;
    std::string ik_kick_topic_name;
    void publish_kick(const KickFoot &foot);
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr IK_Kick_end_sub;
    std::string ik_kick_end_topic_name;
    void ik_kick_end_callback(const std_msgs::msg::Bool msg);

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr set_yaw_zero_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr set_yaw_90_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr set_yaw_m90_pub;
    std::string set_yaw_zero_topic_name;
    std::string set_yaw_90_topic_name;  // 반시계
    std::string set_yaw_m90_topic_name; // 시계
    void publish_set_yaw(int degree);

    rclcpp::Publisher<teamData>::SharedPtr teamDataTx_pub;
    std::string teamDataTx_topic_name;
    int robot_id_{1};
    void publish_team_data_tx(const teamData &team_data);
    rclcpp::Subscription<teamData>::SharedPtr teamDataRx_sub;
    std::string teamDataRx_topic_name;
    void team_data_callback(const teamData &msg);
    bool has_world_model_{false};
    bool has_teammate_pose_{false};

    std::chrono::steady_clock::time_point last_teammate_data_time_{};

    // ---
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::chrono::steady_clock::time_point yaw_settle_deadline_{};
    bool yaw_settle_pending_{false};
    bool is_vision_fresh() const;
    void control_loop();

    rclcpp::Time last_vision_time_{0, 0, RCL_ROS_TIME};
    const double vision_timeout_{0.12};

    FsmInput input_;
    TaskFsm fsm_;
};

} // namespace task_planner