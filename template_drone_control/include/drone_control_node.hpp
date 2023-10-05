#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

using namespace std::chrono_literals;

class DroneControlNode : public rclcpp::Node
{
public:
    DroneControlNode();
    // ~DroneControlNode();
private:
    // ROS Subscribers
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    // ROS Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    // ROS Client services
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    // MAVROS msgs
    mavros_msgs::msg::State current_state_;
    // Fuctions
    void setupSubscribers();
    void setupPublishers();
    void setupServices();
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
    void setMode(std::string mode);
    void setArmStatus(bool value);
void setTOL(
        float min_pitch = 0.0, 
        float yaw = 0.0, 
        float latitude = 0.0, 
        float longitude = 0.0, 
        float altitude = 0.0);
};
