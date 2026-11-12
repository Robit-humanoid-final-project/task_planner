#include "task_planner/task_planner.hpp"

TaskPlanner::TaskPlanner() : Node("task_planner")
{
    gamecontroller2master_topic_name =
        declare_parameter<std::string>("gamecontroller2master_topic_name", "gamecontroldata");
    master2ik_topic_name = declare_parameter<std::string>("master2ik_topic_name", "master2ik");

    ControlData_sub = this->create_subscription<controlData>(
        gamecontroller2master_topic_name, 10,
        std::bind(&TaskPlanner::control_data_callback, this, std::placeholders::_1));
    Master2Ik_pub = this->create_publisher<M2Ik>(master2ik_topic_name, 10);
}

TaskPlanner::~TaskPlanner()
{
}

// game controller data subscribe
void TaskPlanner::control_data_callback(const controlData::SharedPtr msg)
{
    state = msg->state;
    // RCLCPP_INFO(this->get_logger(), "gamecontroller state: %d", state);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}