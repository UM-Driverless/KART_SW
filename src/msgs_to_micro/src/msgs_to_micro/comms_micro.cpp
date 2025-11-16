#include "msgs_to_micro/comms_micro.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <algorithm>

Comms_micro::Comms_micro() : Node("comms_micro")
{
    const char *uart_device = "/dev/ttyTHS1";
    uart_fd_ = open(uart_device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(),
                    "No se pudo abrir %s: (%d) %s",
                    uart_device, errno, strerror(errno));
        return;
    }

    struct termios tty;
    if (tcgetattr(uart_fd_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error al obtener atributos del puerto UART");
        close(uart_fd_);
        uart_fd_ = -1;
        return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error al configurar el puerto UART");
        close(uart_fd_);
        uart_fd_ = -1;
        return;
    }

    subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        "/ackermann_vel", 10,
        std::bind(&Comms_micro::callback, this, std::placeholders::_1));
}

Comms_micro::~Comms_micro()
{
    if (uart_fd_ >= 0) {
        close(uart_fd_);
    }
}

void Comms_micro::callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg)
{
    if (uart_fd_ < 0) {
        RCLCPP_WARN(this->get_logger(), "UART no inicializado, ignorando mensaje");
        return;
    }

    float steering = msg->steering_angle;
    float throttle = 0.0f;
    float brake = 0.0f;

    if (msg->acceleration > 0) {
        throttle = msg->acceleration;
        brake = 0.0f;
    } else {
        throttle = 0.0f;
        brake = -msg->acceleration;
    }

    int steering_uart = static_cast<int>((steering + 1.0f) * 127.5f);
    int throttle_uart = static_cast<int>(throttle * 255.0f);
    int brake_uart = static_cast<int>(brake * 255.0f);

    steering_uart = std::clamp(steering_uart, 0, 255);
    throttle_uart = std::clamp(throttle_uart, 0, 255);
    brake_uart = std::clamp(brake_uart, 0, 255);

    uint8_t packet[4];
    packet[0] = 0xAA;
    packet[1] = static_cast<uint8_t>(steering_uart);
    packet[2] = static_cast<uint8_t>(throttle_uart);
    packet[3] = static_cast<uint8_t>(brake_uart);

    ssize_t total_written = 0;
    constexpr ssize_t bytes_to_send = sizeof(packet);

    while (total_written < bytes_to_send) {
        ssize_t n = write(uart_fd_, packet + total_written, bytes_to_send - total_written);
        if (n < 0) {
            perror("Error en write UART");
            break;
        }
        total_written += n;
    }

    if (total_written != bytes_to_send) {
        RCLCPP_ERROR(this->get_logger(), "Número de bytes enviados incorrecto por UART");
    }
}
