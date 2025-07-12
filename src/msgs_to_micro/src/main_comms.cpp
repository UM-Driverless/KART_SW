#include "msgs_to_micro/comms_micro.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Comms_micro>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
