#include "task_planner/task_planner.hpp"

TaskPlanner::TaskPlanner() : Node("task_planner")
{
    // yaml 받아오기
    gamecontroller2master_topic_name =
        declare_parameter<std::string>("gamecontroller2master_topic_name", "gamecontroldata");
    master2ik_topic_name = declare_parameter<std::string>("master2ik_topic_name", "master2ik");
    navCommand_topic_name = declare_parameter<std::string>("navCommand_topic_name", "nav_command");
    navFeedback_topic_name = declare_parameter<std::string>("navFeedback_topic_name", "nav_feedback");
    visionCommand_topic_name = declare_parameter<std::string>("visionCommand_topic_name", "vision_command");
    visionFeedback_topic_name = declare_parameter<std::string>("visionFeedback_topic_name", "vision_feedback");
    worldModel_topic_name = declare_parameter<std::string>("worldModel_topic_name", "world_model");

    // topic pub, sub, QoS 추후 변경
    ControlData_sub = this->create_subscription<controlData>(
        gamecontroller2master_topic_name, 10,
        std::bind(&TaskPlanner::control_data_callback, this, std::placeholders::_1));
    Master2Ik_pub = this->create_publisher<M2Ik>(master2ik_topic_name, 10);
    NavCommand_pub = this->create_publisher<navComm>(navCommand_topic_name, 10);
    NavFeedback_sub = this->create_subscription<navFeedback>(
        navFeedback_topic_name, 10, std::bind(&TaskPlanner::navigation_feedback_callback, this, std::placeholders::_1));
    VisionCommand_pub = this->create_publisher<VSComm>(visionCommand_topic_name, 10);
    VisionFeedback_sub = this->create_subscription<VSFeedback>(
        visionFeedback_topic_name, 10, std::bind(&TaskPlanner::vision_feedback_callback, this, std::placeholders::_1));
    WorldModel_sub = this->create_subscription<worldModel>(
        worldModel_topic_name, 10, std::bind(&TaskPlanner::world_model_callback, this, std::placeholders::_1));
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
// navigation feedback subscribe
void TaskPlanner::navigation_feedback_callback(const navFeedback::SharedPtr msg)
{
    double temp = msg->eta_seconds;
}
// vision feedback subscribe
void TaskPlanner::vision_feedback_callback(const VSFeedback::SharedPtr msg)
{
    int temp = msg->is_aligned;
}
// world model subscribe
void TaskPlanner::world_model_callback(const worldModel::SharedPtr msg)
{
    int temp = msg->ball_detected;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}