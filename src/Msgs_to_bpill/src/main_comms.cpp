#include "msgs_to_bpill/comms_bpill.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Comms_bpill>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
