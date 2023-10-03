#include "drone_control_node.hpp"

DroneControlNode::DroneControlNode() : Node("drone_control_node"){
    // Set up ROS publishers, subscribers and service clients
    this->setupSubscribers();
    this->setupPublishers();
    this->setupServices();
    // Wait for MAVROS SITL connection
    while (rclcpp::ok() && !current_state_.connected)
    {
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(100ms);
    }
    this->setMode("GUIDED");



        // TODO: Test if drone state really changed to GUIDED

        // TODO: Arm and Take Off
        RCLCPP_INFO(this->get_logger(), "Sending position command.");

        // TODO: Implement position controller and mission commands here
}



void DroneControlNode::setupSubscribers(){
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&DroneControlNode::stateCallback, this, std::placeholders::_1));
}
void DroneControlNode::setupPublishers(){
    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
}
void DroneControlNode::setupServices(){
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");
}
void DroneControlNode::stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
    current_state_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Current State: %s", current_state_.mode.c_str());
}
void DroneControlNode::setMode(string mode){
        std::Shared<mavros_msgs::srv::SetMode::Request> guided_set_mode_req;
        guided_set_mode_req->custom_mode = mode;
        while (!set_mode_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(
                    this->get_logger(), 
                    "Interrupted while waiting for the set_mode service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
        }
        mavros_msgs::srv::SetMode::Response result = 
        set_mode_client_->async_send_request(guided_set_mode_req);
        if(result.mode_sent){
            RCLCPP_INFO(this->get_logger(), "Mode set succesfully.");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Couldn't set mode.");
        }

}
void DroneControlNode::setArmStatus(bool value){
    std::Shared<mavros_msgs::srv::CommandBool::Request> armValue;
    armValue->value = value;
    while (!set_mode_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(
                this->get_logger(), 
                "Interrupted while waiting for the set_mode service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
    }
    std::Shared<mavros_msgs::srv::CommandBool::Response> result = 
    arming_client_->async_send_request(armValue);
    if(result->success){
        RCLCPP_INFO(this->get_logger(), "Arm status changed succesfully.");
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Couldn't change arm status.");
    }
}
// Takeoff or land command 
void DroneControlNode::setTOL(
        float min_pitch = 0, 
        float yaw = 0, 
        float latitude = 0, 
        float longitude = 0, 
        float altitude = 0)
{
    std::Shared<mavros_msgs::srv::CommandTOL::Request> commandTOL;
    commandTOL->min_pitch = min_pitch;
    commandTOL->yaw = yaw;
    commandTOL->latitude = latitude;
    commandTOL->longitude = longitude;
    commandTOL->altitude = altitude;
    while (!set_mode_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(
                this->get_logger(), 
                "Interrupted while waiting for the set_mode service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
    }
    std::Shared<mavros_msgs::srv::CommandTOL::Response> result = 
    arming_client_->async_send_request(commandTOL);
    if(result->success){
        RCLCPP_INFO(this->get_logger(), "TOL command executed.");
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Couldn't execute TOL command.");
    }
}
// TODO: implement mavros_msgs/msg/PositionTarget publisher
//       explore params of PositionTarget msg, especialy coordinate_frame 
//       and type_mask.
// TODO: implement regulator based on position/ velocity.
// TODO: get to know coordinate systems of SITL, MAVROS/MAVLINK, Ardupilot, drone etc.
// TODO: implement regulator precision soft, normal, hard

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemplateDroneControl>());
    rclcpp::shutdown();
    return 0;
}


