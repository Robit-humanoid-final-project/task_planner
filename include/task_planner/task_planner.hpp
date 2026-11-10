#pragma once

#include "rclcpp/rclcpp.hpp"

#include "gamecontroller/msg/gamecontroldata.hpp"
#include "humanoid_interfaces/msg/master2_ik_msg.hpp"

using controlData = gamecontroller::msg::Gamecontroldata;
using M2Ik = humanoid_interfaces::msg::Master2IkMsg;

class TaskPlanner : public rclcpp::Node
{

  public:
    TaskPlanner();
    ~TaskPlanner();

  private:
    rclcpp::Publisher<M2Ik>::SharedPtr Master2Ik_pub;
    std::string master2ik_topic_name;

    rclcpp::Subscription<controlData>::SharedPtr ControlData_sub;
    std::string gamecontroller2master_topic_name;
    void control_data_callback(const controlData::SharedPtr msg);
    int state = 0; // INITIAL: 0, READY:1, SET: 2, PLAYING: 3, FINISHED: 4
};
