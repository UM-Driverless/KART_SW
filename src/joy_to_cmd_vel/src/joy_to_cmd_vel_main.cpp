#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;
int throttle;
int brake;
int steering;
int enable_button;
float dead_zone;

void
joy2cmd_callback(const std::shared_ptr<sensor_msgs::msg::Joy> msg){
    rclcpp::Clock clock;
    ackermann_msgs::msg::AckermannDriveStamped drive_info;
    
    // Comprobar que el boton enable este pulsado
    if (msg->buttons[enable_button] == 1) {
        drive_info.header.stamp = clock.now();

        // Deadzone del joystick
        if (msg->axes[steering] < dead_zone && msg->axes[steering] > dead_zone*(-1)) {
            drive_info.drive.steering_angle = 0;
        } else {
            // Se multiplica por -1 para que positivo quede a la derecha
            drive_info.drive.steering_angle = (-1)*msg->axes[steering];
        }
        
        // Acerelacion = Acerelador - freno || Gatillos -> 2==izquierdo  5==Derecho 
        // Direccion = 0/1 Joystick izquierdo || 3/4 Joystick Derecho
        drive_info.drive.acceleration = (-1)*msg->axes[throttle] - (-1)*msg->axes[brake];
        // drive_info.drive.speed = 0;
        // drive_info.drive.steering_angle_velocity = 0;
        // drive_info.drive.jerk = 0;
    } else {
        drive_info.header.stamp = clock.now();

        drive_info.drive.acceleration = 0;
        drive_info.drive.steering_angle = 0;
        drive_info.drive.speed = 0;
        drive_info.drive.steering_angle_velocity = 0;
        drive_info.drive.jerk = 0;
    }

    if (rclcpp::ok()) {
        drive_pub->publish(drive_info);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("joy_to_cmd_vel");

    node->declare_parameter("throttle", 5); // R2
    node->declare_parameter("brake", 2);    // L2
    node->declare_parameter("steering", 0); // Joystick izquierdo horizontal
    node->declare_parameter("enable_button", 5);   // R1
    node->declare_parameter("dead_zone", 0.05);  

    // Cargar parametros de yaml
    if (node->get_parameter("throttle", throttle)) {
        RCLCPP_INFO(node->get_logger(), "Throttle parameter: %d", throttle);
    } else {
        RCLCPP_WARN(node->get_logger(), "Throttle parameter not found!");
    }

    if (node->get_parameter("brake", brake)) {
        RCLCPP_INFO(node->get_logger(), "Brake parameter: %d", brake);
    } else {
        RCLCPP_WARN(node->get_logger(), "Brake parameter not found!");
    }

    if (node->get_parameter("steering", steering)) {
        RCLCPP_INFO(node->get_logger(), "Steering parameter: %d", steering);
    } else {
        RCLCPP_WARN(node->get_logger(), "Steering parameter not found!");
    }

    if (node->get_parameter("enable_button", enable_button)) {
        RCLCPP_INFO(node->get_logger(), "Enable_button parameter: %d", enable_button);
    } else {
        RCLCPP_WARN(node->get_logger(), "Enable_button parameter not found!");
    }

    if (node->get_parameter("dead_zone", dead_zone)) {
        RCLCPP_INFO(node->get_logger(), "Dead_zone parameter: %f", dead_zone);
    } else {
        RCLCPP_WARN(node->get_logger(), "Dead_zone parameter not found!");
    }

    drive_pub = node->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_vel", 10);

    auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, joy2cmd_callback);

    rclcpp::spin(node);
    drive_pub.reset();  // Destruir publicador antes de salir

    rclcpp::shutdown();
    return 0;
}
