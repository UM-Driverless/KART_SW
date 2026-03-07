#ifndef KB_COMS_MICRO_HPP_
#define KB_COMS_MICRO_HPP_

#include "kb_coms_serial_driver.hpp"
#include "kb_interfaces/msg/frame.hpp"
#include <rclcpp/rclcpp.hpp>

class KB_coms_micro : public rclcpp::Node {

  public:
    KB_coms_micro();

    ~KB_coms_micro();

    void kb_coms_OrinHeartbeat();

    enum  class message_type_t : uint8_t{
      // ==========================
      // ESP32 --> Orin (0x01 - 0x1F)
      // ==========================
      ESP_ACT_SPEED           = 0x01,
      ESP_ACT_ACCELERATION    = 0x02,
      ESP_ACT_BRAKING         = 0x03,
      ESP_ACT_STEERING        = 0x04,
      ESP_MISION              = 0x05,
      ESP_MACHINE_STATE       = 0x06,
      ESP_ACT_SHUTDOWN        = 0x07,
      ESP_HEARTBEAT           = 0x08,
      ESP_COMPLETE            = 0x09,
      ESP_DIAG_STEERING       = 0x0A,
      ESP_HEALTH_STATUS       = 0x0B,

      // ==========================
      // Orin --> ESP32 (0x20 - 0x3F)
      // ==========================
      ORIN_TARG_THROTTLE      = 0x20,
      ORIN_TARG_BRAKING       = 0x21,
      ORIN_TARG_STEERING      = 0x22,
      ORIN_MISION             = 0x23,
      ORIN_MACHINE_STATE      = 0x24,
      ORIN_HEARTBEAT          = 0x25,
      ORIN_SHUTDOWN           = 0x26,
      ORIN_COMPLETE           = 0x27,
      ORIN_CALIBRATE_STEERING = 0x28,

      // ==========================
      // Others (0x40 - 0xFF)
      // ==========================
    };

  private:
    void kb_coms_RXcallback(const SerialDriver::Frame &frame);

    void kb_coms_TXcallback(const kb_interfaces::msg::Frame::SharedPtr msg);

    std::unique_ptr<SerialDriver> serial_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Declaration of all publishers
    rclcpp::Publisher<kb_interfaces::msg::Frame>::SharedPtr esp_heart_pub_;
    rclcpp::Publisher<kb_interfaces::msg::Frame>::SharedPtr esp_speed_pub_;
    rclcpp::Publisher<kb_interfaces::msg::Frame>::SharedPtr esp_acceleration_pub_;
    rclcpp::Publisher<kb_interfaces::msg::Frame>::SharedPtr esp_braking_pub_;
    rclcpp::Publisher<kb_interfaces::msg::Frame>::SharedPtr esp_steering_pub_;
    rclcpp::Publisher<kb_interfaces::msg::Frame>::SharedPtr esp_mision_pub_;
    rclcpp::Publisher<kb_interfaces::msg::Frame>::SharedPtr esp_machine_state_pub_;
    rclcpp::Publisher<kb_interfaces::msg::Frame>::SharedPtr esp_shutdown_pub_;
    rclcpp::Publisher<kb_interfaces::msg::Frame>::SharedPtr esp_health_pub_;

    // Declaration of all subscribers
    rclcpp::Subscription<kb_interfaces::msg::Frame>::SharedPtr orin_throttle_sub_;
    rclcpp::Subscription<kb_interfaces::msg::Frame>::SharedPtr orin_brake_sub_;
    rclcpp::Subscription<kb_interfaces::msg::Frame>::SharedPtr orin_steering_sub_;
    rclcpp::Subscription<kb_interfaces::msg::Frame>::SharedPtr orin_machine_state_sub_;
    rclcpp::Subscription<kb_interfaces::msg::Frame>::SharedPtr orin_mision_sub_;
    rclcpp::Subscription<kb_interfaces::msg::Frame>::SharedPtr orin_heartbeat_sub_;
    rclcpp::Subscription<kb_interfaces::msg::Frame>::SharedPtr orin_shutdown_sub_;
};

#endif // KB_COMS_MICRO_HPP_
