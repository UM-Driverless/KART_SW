#ifndef COMMS_MICRO_HPP
#define COMMS_MICRO_HPP

#include <rclcpp/rclcpp.hpp>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class Comms_micro : public rclcpp::Node
{
public:
    Comms_micro();
    ~Comms_micro();

private:
    void callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg);

    int uart_fd_ = -1;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr subscription_;
};

#endif  // COMMS_MICRO_HPP

