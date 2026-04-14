#include "driver/twai.h"

#define CAN_TX 5
#define CAN_RX 4

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting CAN Receiver...");

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK &&
      twai_start() == ESP_OK) {
    Serial.println("CAN Receiver started");
  } else {
    Serial.println("CAN init failed");
    while (1);
  }
}

void loop() {
  twai_message_t msg;
  if (twai_receive(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("Received CAN Frame!");
    Serial.print("ID: 0x"); Serial.println(msg.identifier, HEX);
    Serial.print("Length: "); Serial.println(msg.data_length_code);
    Serial.print("Data: ");
    for (int i = 0; i < msg.data_length_code; i++) {
      Serial.print(msg.data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}


