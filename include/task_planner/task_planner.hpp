#pragma once

#include "rclcpp/rclcpp.hpp"

#include "gamecontroller/msg/gamecontroldata.hpp"
#include "humanoid_interfaces/msg/master2_ik_msg.hpp"
#include "humanoid_interfaces/msg/nav_command.hpp"
#include "humanoid_interfaces/msg/nav_feedback.hpp"
#include "humanoid_interfaces/msg/vs_command.hpp"
#include "humanoid_interfaces/msg/vs_feedback.hpp"
#include "humanoid_interfaces/msg/world_model.hpp"

using controlData = gamecontroller::msg::Gamecontroldata;
using M2Ik = humanoid_interfaces::msg::Master2IkMsg;
using navComm = humanoid_interfaces::msg::NavCommand;
using navFeedback = humanoid_interfaces::msg::NavFeedback;
using VSComm = humanoid_interfaces::msg::VSCommand;
using VSFeedback = humanoid_interfaces::msg::VSFeedback;
using worldModel = humanoid_interfaces::msg::WorldModel;

class TaskPlanner : public rclcpp::Node
{

  public:
    TaskPlanner();
    ~TaskPlanner();

  private:
    // TODO: 추가할 거
    // ta -> gamecontroller(없어도 될지도)
    // ta -> walk(kick, pass 등), 보행은 Master2Ik 또는 vision, navi -> walk
    rclcpp::Subscription<controlData>::SharedPtr ControlData_sub;
    std::string gamecontroller2master_topic_name;
    void control_data_callback(const controlData::SharedPtr msg);
    int state = 0; // INITIAL: 0, READY:1, SET: 2, PLAYING: 3, FINISHED: 4

    rclcpp::Publisher<M2Ik>::SharedPtr Master2Ik_pub;
    std::string master2ik_topic_name;

    rclcpp::Publisher<navComm>::SharedPtr NavCommand_pub;
    std::string navCommand_topic_name;

    rclcpp::Subscription<navFeedback>::SharedPtr NavFeedback_sub;
    std::string navFeedback_topic_name;
    void navigation_feedback_callback(const navFeedback::SharedPtr msg);

    rclcpp::Publisher<VSComm>::SharedPtr VisionCommand_pub;
    std::string visionCommand_topic_name;

    rclcpp::Subscription<VSFeedback>::SharedPtr VisionFeedback_sub;
    std::string visionFeedback_topic_name;
    void vision_feedback_callback(const VSFeedback::SharedPtr msg);

    rclcpp::Subscription<worldModel>::SharedPtr WorldModel_sub;
    std::string worldModel_topic_name;
    void world_model_callback(const worldModel::SharedPtr msg);
};
