#include "kb_coms_micro/kb_coms_micro.hpp"

KB_coms_micro::KB_coms_micro() : Node("kb_coms_micro_node") {

    // Declare parameters with defaults
    this->declare_parameter<std::string>("serial_port", "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0");
    this->declare_parameter<int>("baudrate", 115200);

    // Get parameters
    std::string port;
    int baud;
    this->get_parameter("serial_port", port);
    this->get_parameter("baudrate", baud);

    // Create publishers
    esp_heart_pub_ = create_publisher<kb_interfaces::msg::Frame>("/esp32/heartbeat", 10);

    esp_speed_pub_ = create_publisher<kb_interfaces::msg::Frame>("/esp32/speed", 10);

    esp_acceleration_pub_ = create_publisher<kb_interfaces::msg::Frame>("/esp32/acceleration", 10);

    esp_braking_pub_ = create_publisher<kb_interfaces::msg::Frame>("/esp32/braking", 10);

    esp_steering_pub_ = create_publisher<kb_interfaces::msg::Frame>("/esp32/steering", 10);

    esp_mision_pub_ = create_publisher<kb_interfaces::msg::Frame>("/esp32/mision", 10);

    esp_machine_state_pub_ = create_publisher<kb_interfaces::msg::Frame>("/esp32/machine_state", 10);

    esp_shutdown_pub_ = create_publisher<kb_interfaces::msg::Frame>("/esp32/shutdown", 10);


    // Create Subscriptors
    orin_throttle_sub_ = create_subscription<kb_interfaces::msg::Frame>(
        "/orin/throttle", 10, std::bind(&KB_coms_micro::kb_coms_TXcallback, this, std::placeholders::_1));

    orin_brake_sub_ = create_subscription<kb_interfaces::msg::Frame>(
        "/orin/brake", 10, std::bind(&KB_coms_micro::kb_coms_TXcallback, this, std::placeholders::_1));
    
    orin_steering_sub_ = create_subscription<kb_interfaces::msg::Frame>(
        "/orin/steering", 10, std::bind(&KB_coms_micro::kb_coms_TXcallback, this, std::placeholders::_1));

    orin_machine_state_sub_ = create_subscription<kb_interfaces::msg::Frame>(
        "/orin/machine_state", 10, std::bind(&KB_coms_micro::kb_coms_TXcallback, this, std::placeholders::_1));

    orin_mision_sub_ = create_subscription<kb_interfaces::msg::Frame>(
        "/orin/mision", 10, std::bind(&KB_coms_micro::kb_coms_TXcallback, this, std::placeholders::_1));

    orin_heartbeat_sub_ = create_subscription<kb_interfaces::msg::Frame>(
        "/orin/heartbeat", 10, std::bind(&KB_coms_micro::kb_coms_TXcallback, this, std::placeholders::_1));
    
    orin_shutdown_sub_ = create_subscription<kb_interfaces::msg::Frame>(
        "/orin/shutdown", 10, std::bind(&KB_coms_micro::kb_coms_TXcallback, this, std::placeholders::_1));

    // Inicializa la librería serial
    serial_ = std::make_unique<SerialDriver>(
        port, baud, [this](const SerialDriver::Frame &frame) { this->kb_coms_RXcallback(frame); });

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&KB_coms_micro::kb_coms_OrinHeartbeat, this));

    serial_->start();
}

KB_coms_micro::~KB_coms_micro() { serial_->stop(); }

void KB_coms_micro::kb_coms_OrinHeartbeat(void) {

    uint8_t type = static_cast<uint8_t>(message_type_t::ORIN_HEARTBEAT);
    std::vector<uint8_t> payload;

    serial_->send(type, payload);
}

// Callback de mensajes de la ESP
void KB_coms_micro::kb_coms_RXcallback(const SerialDriver::Frame &frame_esp) {
    RCLCPP_INFO(this->get_logger(), "Se ha recibido un msg: %d", frame_esp.type);

    // TODO: Procesar los payloads

    switch(frame_esp.type) {
    case kb_interfaces::msg::Frame::ESP_ACT_SPEED: {
        kb_interfaces::msg::Frame msg_orin1;

        msg_orin1.type = frame_esp.type;
        msg_orin1.payload = frame_esp.payload;

        esp_speed_pub_->publish(msg_orin1);

        break;
    }

    case kb_interfaces::msg::Frame::ESP_ACT_ACCELERATION: {
        kb_interfaces::msg::Frame msg_orin2;

        msg_orin2.type = frame_esp.type;
        msg_orin2.payload = frame_esp.payload;

        esp_acceleration_pub_->publish(msg_orin2);

        break;
    }

    case kb_interfaces::msg::Frame::ESP_ACT_BRAKING: {
        kb_interfaces::msg::Frame msg_orin3;

        msg_orin3.type = frame_esp.type;
        msg_orin3.payload = frame_esp.payload;

        esp_braking_pub_->publish(msg_orin3);

        break;
    }

    case kb_interfaces::msg::Frame::ESP_ACT_STEERING: {
        kb_interfaces::msg::Frame msg_orin4;

        msg_orin4.type = frame_esp.type;
        msg_orin4.payload = frame_esp.payload;

        esp_steering_pub_->publish(msg_orin4);

        break;
    }

    case kb_interfaces::msg::Frame::ESP_MISION: {
        kb_interfaces::msg::Frame msg_orin5;

        msg_orin5.type = frame_esp.type;
        msg_orin5.payload = frame_esp.payload;

        esp_mision_pub_->publish(msg_orin5);

        break;
    }

    case kb_interfaces::msg::Frame::ESP_MACHINE_STATE: {
        kb_interfaces::msg::Frame msg_orin6;

        msg_orin6.type = frame_esp.type;
        msg_orin6.payload = frame_esp.payload;

        esp_machine_state_pub_->publish(msg_orin6);

        break;
    }

    case kb_interfaces::msg::Frame::ESP_ACT_SHUTDOWN: {
        kb_interfaces::msg::Frame msg_orin7;

        msg_orin7.type = frame_esp.type;
        msg_orin7.payload = frame_esp.payload;

        esp_shutdown_pub_->publish(msg_orin7);

        break;
    }

    case kb_interfaces::msg::Frame::ESP_HEARTBEAT: {
        kb_interfaces::msg::Frame msg_orin8;

        msg_orin8.type = frame_esp.type;
        msg_orin8.payload = frame_esp.payload;

        esp_heart_pub_->publish(msg_orin8);

        break;
    }

    case kb_interfaces::msg::Frame::ESP_COMPLETE: {

        // Dividir mensaje en los submensajes que vienen en el payload
        kb_interfaces::msg::Frame msg_orin9;

        msg_orin9.type = frame_esp.type;
        msg_orin9.payload = frame_esp.payload;

        break;
    }

    default:
        break;
    }
}

// Callback de mensajes de la ORIN
void KB_coms_micro::kb_coms_TXcallback(const kb_interfaces::msg::Frame::SharedPtr msg) {

    serial_->send(msg->type, msg->payload);
}
