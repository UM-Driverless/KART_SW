#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"


rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
int throttle;
int brake;
int steering;
int enable_button;
float dead_zone;

void
joy2cmd_callback(const std::shared_ptr<sensor_msgs::msg::Joy> msg){
    geometry_msgs::msg::Twist twist;

    if (msg->buttons[enable_button] == 1) {
        // Deadzone
        if (msg->axes[steering] < dead_zone && msg->axes[steering] > -dead_zone) {
            twist.angular.z = 0.0;
        } else {
            // Negate so positive = right
            twist.angular.z = -msg->axes[steering];
        }

        // Triggers: raw range [1, -1] → normalized [0, 1]
        auto throttle_norm = (-msg->axes[throttle] + 1.0) / 2.0;
        auto brake_norm    = (-msg->axes[brake]    + 1.0) / 2.0;
        // linear.x carries (throttle - brake) in [-1, 1]
        twist.linear.x = throttle_norm - brake_norm;
    }
    // else: all fields default to 0

    if (rclcpp::ok()) {
        cmd_pub->publish(twist);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("joy_to_cmd_vel");

    node->declare_parameter("throttle", 4);       // R2
    node->declare_parameter("brake", 3);           // L2
    node->declare_parameter("steering", 0);        // Left stick horizontal
    node->declare_parameter("enable_button", 5);   // R1
    node->declare_parameter("dead_zone", 0.05);

    node->get_parameter("throttle", throttle);
    node->get_parameter("brake", brake);
    node->get_parameter("steering", steering);
    node->get_parameter("enable_button", enable_button);
    node->get_parameter("dead_zone", dead_zone);

    RCLCPP_INFO(node->get_logger(),
        "joy_to_cmd_vel: throttle=%d brake=%d steering=%d enable=%d dz=%.2f",
        throttle, brake, steering, enable_button, dead_zone);

    cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>("/kart/cmd_vel", 10);

    auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, joy2cmd_callback);

    rclcpp::spin(node);
    cmd_pub.reset();

    rclcpp::shutdown();
    return 0;
}
