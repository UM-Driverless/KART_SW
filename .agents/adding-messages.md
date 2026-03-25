<!-- reference — read when relevant -->
# How to Add New Message Types (Orin ↔ ESP32)

This guide describes how to add a new message type to the serial communication between the Orin (ROS2) and the ESP32 (kart_medulla firmware). Both sides must be updated to keep message tags in sync.

---

## Part 1: Orin Side (kart_brain)

### Step 1 — Add the tag to the ROS custom message

Add the new tag constant to `kb_interfaces/msg/Frame.msg`. The tag number **must match** across all files described below.

```
# ==========================
# ESP32 --> Orin (0x01 - 0x1F)
# ==========================
uint8 ESP_ACT_SPEED         = 0x01
uint8 ESP_ACT_ACCELERATION  = 0x02
uint8 ESP_ACT_BRAKING       = 0x03
uint8 ESP_ACT_STEERING      = 0x04
uint8 ESP_MISION            = 0x05
uint8 ESP_MACHINE_STATE     = 0x06
uint8 ESP_ACT_SHUTDOWN      = 0x07
uint8 ESP_HEARTBEAT         = 0x08
uint8 ESP_COMPLETE          = 0x09
uint8 ESP_DIAG_STEERING     = 0x0A
uint8 ESP_HEALTH_STATUS     = 0x0B

# ==========================
# Orin --> ESP32 (0x20 - 0x3F)
# ==========================
uint8 ORIN_TARG_THROTTLE    = 0x20
uint8 ORIN_TARG_BRAKING     = 0x21
uint8 ORIN_TARG_STEERING    = 0x22
uint8 ORIN_MISION           = 0x23
uint8 ORIN_MACHINE_STATE    = 0x24
uint8 ORIN_HEARTBEAT        = 0x25
uint8 ORIN_SHUTDOWN         = 0x26
uint8 ORIN_COMPLETE         = 0x27
```

### Step 2 — Update `kb_coms_micro.hpp`

Add the new tag to the `message_type_t` enum. It **must match** the tag added to `Frame.msg`.

```cpp
enum class message_type_t : uint8_t {
    // ESP32 --> Orin (0x01 - 0x1F)
    ESP_ACT_SPEED           = 0x01,
    ESP_ACT_ACCELERATION    = 0x02,
    // ... (same tags as Frame.msg)

    // Orin --> ESP32 (0x20 - 0x3F)
    ORIN_TARG_THROTTLE      = 0x20,
    // ...
};
```

Then declare the publisher or subscriber:
- If the **Orin sends** the message → declare a **subscriber** (Orin subscribes to the topic, serializes, and sends over UART).
- If the **Orin receives** the message → declare a **publisher** (Orin deserializes from UART and publishes to a ROS topic).

```cpp
// Publishers (ESP32 → Orin: received from serial, published to ROS)
rclcpp::Publisher<kb_interfaces::msg::Frame>::SharedPtr esp_heart_pub_;

// Subscribers (Orin → ESP32: received from ROS, sent over serial)
rclcpp::Subscription<kb_interfaces::msg::Frame>::SharedPtr orin_throttle_sub_;
```

### Step 3 — Update `kb_coms_micro.cpp`

Initialize the publisher or subscriber declared above:

```cpp
// Publisher (ESP32 → Orin)
esp_health_pub_ = create_publisher<kb_interfaces::msg::Frame>("/esp32/health", 10);

// Subscriber (Orin → ESP32) — always use kb_coms_TXcallback
orin_throttle_sub_ = create_subscription<kb_interfaces::msg::Frame>(
    "/orin/throttle", 10,
    std::bind(&KB_coms_micro::kb_coms_TXcallback, this, std::placeholders::_1));
```

- **Subscribers** must use `kb_coms_TXcallback` as the callback (it serializes and sends over UART).
- **Publishers** require a new case in `kb_coms_RXcallback` (which is called when data arrives from UART):

```cpp
switch (frame_esp.type) {
case kb_interfaces::msg::Frame::ESP_ACT_SPEED: {
    kb_interfaces::msg::Frame msg_orin1;
    msg_orin1.type = frame_esp.type;
    msg_orin1.payload = frame_esp.payload;
    esp_speed_pub_->publish(msg_orin1);
    break;
}
}
```

---

## Part 2: ESP32 Side (kart_medulla)

### Step 1 — Update `km_coms.h`

Add the new tag to the `message_type_t` enum. The tag **must match** the one added on the Orin side.

```c
typedef enum {
    // ESP32 --> Orin (0x01 - 0x1F)
    ESP_ACT_SPEED           = 0x01,
    ESP_ACT_THROTTLE        = 0x02,
    // ...

    // Orin --> ESP32 (0x20 - 0x3F)
    ORIN_TARG_THROTTLE      = 0x20,
    // ...
} message_type_t;
```

### Step 2 — Update `km_coms.c`

Add a new case to `KM_COMS_ProccessPayload` for the new message tag:

```c
switch (msg.type) {
case ORIN_TARG_THROTTLE:
    if (msg.len != 1) return; // Invalid payload
    object_value = (int64_t)msg.payload[0];
    KM_OBJ_SetObjectValue(TARGET_THROTTLE, object_value);
    break;
}
```

---

## Important Notes

### Payload ordering
If a message contains multiple values, the payload order must be consistent between sender and receiver. Example:

```c
// Sender
int32_t msg2send[] = {value1, value2, value3};
```

The receiver must parse them in the **same order**: `payload[0]` = value1, `payload[1]` = value2, etc.

### Sending a message
Both sides provide functions to send data. Create an `int32_t` array and pass it to the send function. For composite messages (multiple values), respect the order expected by the receiving side.

### Receiving a message
- **Orin**: data is published to the ROS topic declared for that message's publisher.
- **ESP32**: data is stored in the `mb_objects` library. Use `KM_OBJ_GetValue(TAG)` with the appropriate object tag to retrieve it.

---

## Checklist for adding a new message

1. [ ] Add tag to `Frame.msg` (kart_brain)
2. [ ] Add tag to `message_type_t` enum in `kb_coms_micro.hpp` (kart_brain)
3. [ ] Declare publisher/subscriber in `kb_coms_micro.hpp` (kart_brain)
4. [ ] Initialize publisher/subscriber in `kb_coms_micro.cpp` (kart_brain)
5. [ ] Add RX switch case or use TX callback in `kb_coms_micro.cpp` (kart_brain)
6. [ ] Add tag to `message_type_t` enum in `km_coms.h` (kart_medulla)
7. [ ] Add case to `KM_COMS_ProccessPayload` in `km_coms.c` (kart_medulla)
8. [ ] `colcon build` on Orin + flash ESP32
9. [ ] Test round-trip
