#ifndef COMMS_BPILL_HPP
#define COMMS_BPILL_HPP

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>

class Comms_bpill : public rclcpp::Node
{
public:
    Comms_bpill();
    ~Comms_bpill();

private:
    void callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg);
    int i2c_file_;
    int i2c_address_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr subscription_;
    const int BYTES_TO_SEND = 4;
};

#endif  // COMMS_BPILL_HPP
