#include "msgs_to_bpill/comms_bpill.hpp"

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <algorithm>

Comms_bpill::Comms_bpill() : Node("comms_bpill")
{
    i2c_address_ = 0x08;

    const char *i2c_device = "/dev/i2c-1";
    if ((i2c_file_ = open(i2c_device, O_RDWR)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo abrir %s", i2c_device);
        return;
    }

    if (ioctl(i2c_file_, I2C_SLAVE, i2c_address_) < 0) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo establecer dirección I2C");
        return;
    }

    subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        "/ackermann_vel", 10,
        std::bind(&Comms_bpill::callback, this, std::placeholders::_1));
}

Comms_bpill::~Comms_bpill()
{
    if (i2c_file_ >= 0) {
        close(i2c_file_);
    }
}

void Comms_bpill::callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg)
{
    float steering = msg->steering_angle;
    float throtle = 0;
    float brake = 0;

    // Se esta acelerando
    if (msg->acceleration > 0) {
        throtle = msg->acceleration;
        brake = 0;

    // Se esta frenando
    } else {
        throtle = 0;
        // Convertir en positivo
        brake = (-1)*msg->acceleration;
    }

    // Normalizar de [-1.0, 1.0] -> [0, 255]
    int stering_i2c = static_cast<int>((steering + 1.0f) * 127.5f);

    // Normalizar de [0.0, 1.0] -> [0, 255]
    int throtle_i2c = static_cast<int>(throtle * 255);
    int brake_i2c = static_cast<int>(brake * 255);

    // Limitar [0, 255]
    stering_i2c = std::clamp(stering_i2c, 0, 255);
    throtle_i2c = std::clamp(throtle_i2c, 0, 255);
    brake_i2c = std::clamp(brake_i2c, 0, 255);

    uint8_t steering_byte = static_cast<uint8_t>(stering_i2c);
    uint8_t throtle_byte = static_cast<uint8_t>(throtle_i2c);
    uint8_t brake_byte = static_cast<uint8_t>(brake_i2c);

    uint8_t packet[4];
    packet[0] = 0xAA;           // Header
    packet[1] = steering_byte;  // Dirección
    packet[2] = throtle_byte;   // Acelerador
    packet[3] = brake_byte;     // Freno

    ssize_t total_written = 0;
    while (total_written < BYTES_TO_SEND) {
        ssize_t n = write(i2c_file_, packet + total_written, BYTES_TO_SEND - total_written);
        if (n < 0) {
            perror("Error en write");
            break;
        }
        total_written += n;
    }
    if (total_written != BYTES_TO_SEND) {
        RCLCPP_ERROR(this->get_logger(), "Numero de bytes enviados incorrecto");
    }
}
