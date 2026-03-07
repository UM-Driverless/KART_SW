/**
 * @file serial_driver.cpp
 * @brief Implementation of the industrial SerialDriver.
 */

#include "kb_coms_serial_driver.hpp"

#include <string.h>

/* ============================================================
*  Constructor / Destructor
* ============================================================ */

/**
 * @brief Constructs the SerialDriver.
 *
 * @details
 * Initializes configuration parameters but does NOT start
 * communication threads automatically. Call start() explicitly.
 *
 * @param port Serial device path.
 * @param baudrate Communication speed.
 * @param callback Frame reception callback.
 */
SerialDriver::SerialDriver(const std::string& port,
                        uint32_t baudrate,
                        FrameCallback callback)
    : port_(port),
    baudrate_(baudrate),
    callback_(callback)
{
}

/**
 * @brief Destructor.
 *
 * @details
 * Ensures all threads are stopped and the serial port is closed.
 */
SerialDriver::~SerialDriver()
{
    stop();
    while (!tx_queue_.empty()) tx_queue_.pop();
}

/* ============================================================
*  Public API
* ============================================================ */

/**
 * @brief Starts the driver.
 *
 * @details
 * Launches the supervisor thread. The supervisor will attempt
 * to open the serial port and spawn RX and TX threads.
 *
 * Safe to call only once.
 */
void SerialDriver::start() {

    running_ = true;
    supervisor_thread_ = std::thread(&SerialDriver::supervisor_loop, this);
}

/**
 * @brief Stops the driver.
 *
 * @details
 * Signals all threads to terminate, joins them safely,
 * and closes the serial port.
 *
 * Thread-safe.
 */
void SerialDriver::stop()
{
    running_ = false;

    tx_cv_.notify_all();

    if (rx_thread_.joinable()) rx_thread_.join();
    if (tx_thread_.joinable()) tx_thread_.join();
    if (supervisor_thread_.joinable()) supervisor_thread_.join();

    close_port();
}

/**
 * @brief Queues a frame for transmission.
 *
 * @param type Message type field.
 * @param payload Payload data.
 *
 * @details
 * Frame is serialized immediately and pushed into TX queue.
 * TX thread handles actual transmission.
 *
 * Thread-safe.
 */
void SerialDriver::send(uint8_t type,
                        const std::vector<uint8_t>& payload)
{
    if (payload.size() > 251)
        return;
    
    std::lock_guard<std::mutex> lock(tx_mutex_);
    tx_queue_.push(build_frame(type, payload));
    tx_cv_.notify_one();
}

/**
 * @brief Returns number of successfully received frames.
 *
 * @return Count of frames received with valid CRC.
 *
 * @thread_safety Safe to call from any thread.
 */
uint64_t SerialDriver::rx_ok() const
{
return rx_ok_.load();
}

/**
 * @brief Returns number of CRC errors detected.
 *
 * @return Count of frames received with invalid CRC.
 *
 * @thread_safety Safe to call from any thread.
 */
uint64_t SerialDriver::rx_crc_error() const
{
return rx_crc_error_.load();
}

/**
 * @brief Returns number of frame timeouts detected.
 *
 * @return Count of frames discarded due to timeout.
 *
 * @thread_safety Safe to call from any thread.
 */
uint64_t SerialDriver::rx_timeout() const
{
return rx_timeout_.load();
}


/* ============================================================
*  Supervisor Thread
* ============================================================ */

/**
 * @brief Supervisor loop.
 *
 * @details
 * Responsible for:
 * - Opening serial port
 * - Spawning RX and TX threads
 * - Reconnecting if port fails
 *
 * Runs continuously while driver is active.
 */
void SerialDriver::supervisor_loop()
{
    while (running_) {
        if (fd_ < 0 && !rx_thread_.joinable() && !tx_thread_.joinable()) {
            if (open_port()) {
                rx_thread_ = std::thread(&SerialDriver::rx_loop, this);
                tx_thread_ = std::thread(&SerialDriver::tx_loop, this);
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

/* ============================================================
*  Serial Port Handling
* ============================================================ */

/**
 * @brief Opens and configures the serial port.
 *
 * @return true if successfully opened and configured.
 *
 * @details
 * Configures port in raw mode with:
 * - 8N1
 * - No flow control
 * - Selected baudrate
 */
bool SerialDriver::open_port()
{
    
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0)
        return false;

    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0)
    {
        close_port();
        return false;
    }

    // Limpia todos los bytes que ya están en el buffer
    tcflush(fd_, TCIFLUSH);
    
    cfmakeraw(&tty);
    cfsetispeed(&tty, baudrate_to_flag(baudrate_));
    cfsetospeed(&tty, baudrate_to_flag(baudrate_));
    
    tty.c_cflag |= (CLOCAL | CREAD);
    
    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        std::cerr << "Serial: failed to open " << port_ << std::endl;
        close_port();
        return false;
    }
        
    std::cerr << "Serial: opened " << port_ << " fd=" << fd_ << std::endl;
    return true;
}

/**
 * @brief Closes the serial port.
 */
void SerialDriver::close_port()
{
    if (fd_ >= 0)
    {
        // Limpia todos los bytes que ya están en el buffer
        tcflush(fd_, TCIFLUSH);
        close(fd_);
        fd_ = -1;
    }
}

/* ============================================================
*  RX Thread
* ============================================================ */

/**
 * @brief RX loop.
 *
 * @details
 * Performs blocking read() calls.
 * Each received byte is processed by the protocol state machine.
 *
 * If read() fails, the port is closed and supervisor
 * will attempt reconnection.
 */
void SerialDriver::rx_loop()
{
    uint8_t buffer[256];
    uint16_t bytes_in_buffer = 0;

    // Reset parser state on (re)entry to avoid stale data from previous thread
    state_ = State::WAIT_SOF;
    len_ = 0;
    type_ = 0;
    index_ = 0;
    payload_.clear();

    while (running_ && fd_ >= 0)
    {
        ssize_t n = read(fd_, buffer + bytes_in_buffer, sizeof(buffer) - bytes_in_buffer);

        if (n <= 0) {
            std::cerr << "Serial: read error" << std::endl;
            close_port();
            return;
        }
        bytes_in_buffer += static_cast<uint16_t>(n);

        // Process all received bytes
        uint16_t processed = 0;
        for (uint16_t i = 0; i < bytes_in_buffer; ++i) {
            process_byte(buffer[i]);
            processed++;
        }

        // Move unprocessed bytes to start of buffer
        if (processed < bytes_in_buffer) {
            memmove(buffer, buffer + processed, bytes_in_buffer - processed);
            bytes_in_buffer -= processed;
        } else {
            bytes_in_buffer = 0;
        }
    }
}

/* ============================================================
*  TX Thread
* ============================================================ */

/**
 * @brief TX loop.
 *
 * @details
 * Waits on condition_variable until:
 * - Data available in queue
 * - Driver stops
 *
 * Sends frames sequentially.
 */
void SerialDriver::tx_loop()
{
    while (running_)
    {
        std::unique_lock<std::mutex> lock(tx_mutex_);

        tx_cv_.wait(lock, [&]()
        {
            return !tx_queue_.empty() || !running_ || fd_ < 0;
        });

        if (!running_)
            return;

        auto frame = tx_queue_.front();
        tx_queue_.pop();
        lock.unlock();

        if (fd_ >= 0)
        {
            ssize_t written = write(fd_, frame.data(), frame.size());
            if (written < 0)
            {
                close_port();
                return;
            }
        }
    }
}

/* ============================================================
*  Protocol State Machine
* ============================================================ */

/**
 * @brief Processes a single byte through protocol state machine.
 *
 * @param byte Incoming byte.
 *
 * @details
 * Implements frame parsing with timeout protection.
 * If CRC is valid, callback is executed in RX thread context.
 */
int SerialDriver::process_byte(uint8_t byte)
{
    auto now = std::chrono::steady_clock::now();

    switch (state_)
    {
        case State::WAIT_SOF:
            if (byte == SOF)
            {
                frame_start_ = now;
                state_ = State::LEN;
                payload_.clear();
                len_ = 0;
                index_ = 0;
            }
            break;

        case State::LEN:
            len_ = byte;
            if (len_ > MAX_PAYLOAD)
            {
                state_ = State::WAIT_SOF;
                break;
            }
            state_ = State::TYPE;
            break;

        case State::TYPE:
            type_ = byte;
            index_ = 0;
            payload_.clear();
            state_ = (len_ == 0) ? State::CRC : State::PAYLOAD;
            break;

        case State::PAYLOAD:
            payload_.push_back(byte); // agrega el byte al final
            if (payload_.size() == len_)
                state_ = State::CRC;
            break;

        case State::CRC:
        {
            uint8_t crc = crc8(len_, type_, payload_.data(), len_);
            if (crc == byte) {
                Frame frame;
                frame.type = type_;
                frame.payload.assign(payload_.begin(),
                     payload_.begin() + len_);
                rx_ok_++;
                callback_(frame);
                state_ = State::WAIT_SOF;

                // End of msg
                return 1;

            } else {
                std::cerr << "Serial: CRC mismatch" << std::endl;
                rx_crc_error_++;
            }

            state_ = State::WAIT_SOF;
            break;
        }
    }

    return 0;
}

/* ============================================================
*  Frame Serialization
* ============================================================ */

/**
 * @brief Builds a serialized frame.
 *
 * @param type Message type.
 * @param payload Payload.
 * @return Serialized byte vector.
 */
std::vector<uint8_t> SerialDriver::build_frame(
        uint8_t type,
        const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> frame;
    uint8_t len = payload.size();

    frame.reserve(len + 4);
    frame.push_back(SOF);
    frame.push_back(len);
    frame.push_back(type);
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(crc8(len, type, payload.data(), len));

    return frame;
}

/* ============================================================
*  CRC-8 Implementation
* ============================================================ */

/**
 * @brief Computes CRC-8 (polynomial 0x07).
 *
 * @param len Length field.
 * @param type Type field.
 * @param data Payload data.
 * @param size Payload size.
 * @return Calculated CRC value.
 */
uint8_t SerialDriver::crc8(uint8_t len,
                        uint8_t type,
                        const uint8_t* data,
                        uint8_t size)
{
    uint8_t crc = 0;

    auto update = [&](uint8_t b)
    {
        crc ^= b;
        for (int i = 0; i < 8; ++i)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    };

    update(len);
    update(type);

    for (uint8_t i = 0; i < size; ++i)
        update(data[i]);

    return crc;
}

speed_t SerialDriver::baudrate_to_flag(uint32_t baudrate) const
{
    switch (baudrate)
    {
        case 9600: return B9600;
        case 115200: return B115200;
        case 460800: return B460800;
        case 1000000: return B1000000;
        default: return B460800;
    }
}
