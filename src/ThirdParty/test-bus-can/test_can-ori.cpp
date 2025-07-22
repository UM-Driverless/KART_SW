#include <Arduino.h>
#include <Wire.h>
#include "BluetoothSerial.h"

#include <driver/twai.h>

// ESP Pin 17  => CTX
// ESP PIN 16 => CRX 


void setup() {
  Serial.begin(115200);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_17, GPIO_NUM_16, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();

  Serial.println("CAN iniciado");
}

void loop() {
  twai_message_t message;
  message.identifier = 0x123;
  message.data_length_code = 2;
  message.data[0] = 0xAB;
  message.data[1] = 0xCD;

  if (twai_transmit(&message, pdMS_TO_TICKS(50000)) == ESP_OK) {
    Serial.println("Mensaje CAN enviado");
  } else {
    Serial.println("Fallo al enviar");
  }

  delay(1000);
}