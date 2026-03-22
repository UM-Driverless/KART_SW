#pragma once

/**
 * @file serial_driver.hpp
 * @brief Industrial-grade serial driver with separated RX and TX threads.
 *
 * @details
 * This driver implements a robust binary protocol over a POSIX serial port.
 * It features:
 *
 * - Dedicated RX thread (blocking read)
 * - Dedicated TX thread (condition_variable driven)
 * - Supervisor thread with automatic reconnection
 * - CRC-8 (polynomial 0x07)
 * - Frame timeout detection
 * - Thread-safe transmission
 *
 * Designed for embedded Linux systems such as NVIDIA Jetson platforms.
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include <iostream>
#include <string>

/**
 * @class SerialDriver
 * @brief Industrial serial communication driver.
 *
 * @details
 * This class provides a high-reliability serial communication layer
 * implementing a framed binary protocol:
 *
 * Frame format:
 * @code
 *  +------+-----+------+---------+------+
 *  | SOF  | LEN | TYPE | PAYLOAD | CRC  |
 *  +------+-----+------+---------+------+
 *    1B     1B    1B      N B      1B
 * @endcode
 *
 * CRC uses CRC-8 with polynomial 0x07.
 *
 * Thread architecture:
 * - RX thread (blocking read)
 * - TX thread (condition variable driven)
 * - Supervisor thread (auto-reconnect)
 *
 * @thread_safety
 * - send() is thread-safe.
 * - Callback is executed in RX thread context.
 */
class SerialDriver
{
public:

    /**
     * @struct Frame
     * @brief Represents a fully decoded protocol frame.
     */
    struct Frame
    {
        uint8_t type;                 /**< Message type field */
        std::vector<int32_t> payload; /**< Payload data */
    };

    /**
     * @brief Callback type invoked when a valid frame is received.
     */
    using FrameCallback = std::function<void(const Frame&)>;

    /**
     * @brief Constructor.
     *
     * @param port Serial device path (e.g., "/dev/esp32").
     * @param baudrate Communication baudrate.
     * @param callback Function called on valid frame reception.
     *
     * @note Does not start communication automatically. Call start().
     */
    SerialDriver(const std::string& port,
                 uint32_t baudrate,
                 FrameCallback callback);

    /**
     * @brief Destructor.
     *
     * Stops all threads and closes the serial port safely.
     */
    ~SerialDriver();

    /**
     * @brief Starts the driver threads.
     *
     * Launches the supervisor thread which manages connection
     * and spawns RX/TX threads.
     */
    void start();

    /**
     * @brief Stops all threads and closes the port.
     *
     * Safe to call multiple times.
     */
    void stop();

    /**
     * @brief Sends a frame over the serial port.
     *
     * @param type Message type field.
     * @param payload Data payload.
     *
     * @thread_safety Thread-safe.
     */
    void send(uint8_t type, const std::vector<int32_t>& payload);

    /**
     * @brief Returns number of successfully received frames.
     */
    uint64_t rx_ok() const;

    /**
     * @brief Returns number of CRC errors detected.
     */
    uint64_t rx_crc_error() const;

    /**
     * @brief Returns number of frame timeouts detected.
     */
    uint64_t rx_timeout() const;

private:

    /** Start Of Frame marker */
    static constexpr uint8_t SOF = 0xAA;

    /** Maximum allowed payload size */
    static constexpr size_t MAX_PAYLOAD = 255;

    /** Maximum allowed time between SOF and CRC */
    static constexpr std::chrono::milliseconds FRAME_TIMEOUT{100};

    std::string port_;      /**< Serial device path */
    uint32_t baudrate_;     /**< Configured baudrate */
    int fd_ = -1;           /**< File descriptor */

    std::atomic<bool> running_{false};

    FrameCallback callback_;

    std::thread rx_thread_;
    std::thread tx_thread_;
    std::thread supervisor_thread_;

    std::mutex tx_mutex_;
    std::condition_variable tx_cv_;
    std::queue<std::vector<uint8_t>> tx_queue_;

    std::atomic<uint64_t> rx_ok_{0};
    std::atomic<uint64_t> rx_crc_error_{0};
    std::atomic<uint64_t> rx_timeout_{0};

    enum class State
    {
        WAIT_SOF,
        LEN,
        TYPE,
        PAYLOAD,
        CRC
    };

    State state_{State::WAIT_SOF};

    // Frame parsing variables
    uint8_t len_{0};
    uint8_t type_{0};
    uint8_t index_{0};

    std::vector<uint8_t> payload_;

    // Timeout handling
    std::chrono::steady_clock::time_point frame_start_;

    /** Last time a valid frame was received (for watchdog reconnect) */
    std::atomic<std::chrono::steady_clock::time_point> last_rx_ok_time_{std::chrono::steady_clock::now()};

    /** Watchdog timeout — force reconnect if no valid frames for this long */
    static constexpr std::chrono::seconds WATCHDOG_TIMEOUT{5};

    /**
     * @brief Supervisor thread loop.
     *
     * Handles automatic reconnection and thread spawning.
     */
    void supervisor_loop();

    /**
     * @brief Opens and configures the serial port.
     *
     * @return true on success, false otherwise.
     */
    bool open_port();

    /**
     * @brief Closes the serial port safely.
     */
    void close_port();

    /**
     * @brief RX thread loop.
     *
     * Performs blocking reads and feeds the state machine.
     */
    void rx_loop();

    /**
     * @brief TX thread loop.
     *
     * Waits on condition variable and transmits queued frames.
     */
    void tx_loop();

    /**
     * @brief Processes a single received byte.
     *
     * Implements protocol state machine.
     *
     * @param byte Received byte.
     * @retval 1 for end of msg, 0 for not end of msg
     */
    int process_byte(uint8_t byte);

    /**
     * @brief Builds a serialized frame.
     *
     * @param type Message type.
     * @param payload Payload data.
     * @return Serialized frame ready for transmission.
     */
    std::vector<uint8_t> build_frame(uint8_t type,
                                     const std::vector<int32_t>& payload);

    /**
     * @brief Computes CRC-8 checksum.
     *
     * Polynomial: 0x07
     *
     * @param len Length field.
     * @param type Type field.
     * @param data Payload pointer.
     * @param size Payload size.
     * @return Computed CRC value.
     */
    uint8_t crc8(uint8_t len,
                 uint8_t type,
                 const uint8_t* data,
                 uint8_t size);

    
    speed_t baudrate_to_flag(uint32_t baudrate) const;
};
