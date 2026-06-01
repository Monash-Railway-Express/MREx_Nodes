
#include "driver/twai.h"
#define TX_GPIO_NUM GPIO_NUM_17
#define RX_GPIO_NUM GPIO_NUM_16
String myMessage = "Hello from Node B!";  //You can change this
bool messageSent = false;
unsigned long lastSent = 0;
// Message assembly buffer
char incomingBuffer[100];
int bufferIndex = 0;
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("CAN Sentence Transceiver (TWAI)");
  twai_general_config_t g_config = {
    .mode = TWAI_MODE_NORMAL,
    .tx_io = TX_GPIO_NUM,
    .rx_io = RX_GPIO_NUM,
    .clkout_io = TWAI_IO_UNUSED,
    .bus_off_io = TWAI_IO_UNUSED,
    .tx_queue_len = 5,
    .rx_queue_len = 5,
    .alerts_enabled = TWAI_ALERT_NONE,
    .clkout_divider = 0,
    .intr_flags = ESP_INTR_FLAG_LEVEL1
  };
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Driver install failed");
    while (1);
  }
  if (twai_start() != ESP_OK) {
    Serial.println("Driver start failed");
    while (1);
  }
  Serial.println("TWAI driver started");
}
void sendSentence(String msg) {
  Serial.println("Sending message: " + msg);
  int msgLen = msg.length();
  for (int i = 0; i <= msgLen; i += 8) {
    twai_message_t tx_msg = {};
    tx_msg.identifier = 0x10;
    tx_msg.extd = 0;
    tx_msg.rtr = 0;
    tx_msg.data_length_code = 8;
    for (int j = 0; j < 8; j++) {
      if (i + j < msgLen) {
        tx_msg.data[j] = msg[i + j];
      } else if (i + j == msgLen) {
        tx_msg.data[j] = '\0';  // Null terminator signals end of sentence
      } else {
        tx_msg.data[j] = ' ';
      }
    }
    if (twai_transmit(&tx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
      Serial.print("Sent chunk: ");
      for (int k = 0; k < 8; k++) Serial.print((char)tx_msg.data[k]);
      Serial.println();
    } else {
      Serial.println("Send failed");
    }
    delay(10);  // Small delay between chunks
  }
}
void loop() {
  // 1. Receive incoming message
  twai_message_t rx_msg;
  if (twai_receive(&rx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
    for (int i = 0; i < rx_msg.data_length_code; i++) {
      char c = (char)rx_msg.data[i];
      if (c == '\0') {
        // End of sentence
        incomingBuffer[bufferIndex] = '\0';
        Serial.print("Full message received: ");
        Serial.println(incomingBuffer);
        bufferIndex = 0;
        // Send response
        delay(500);
        sendSentence(myMessage);
        messageSent = true;
        break;
      } else {
        incomingBuffer[bufferIndex++] = c;
        if (bufferIndex >= sizeof(incomingBuffer) - 1) bufferIndex = 0;
      }
    }
  }
  // 2. If nothing has been sent yet, send your message after 2s
  if (!messageSent && millis() - lastSent > 2000) {
    sendSentence(myMessage);
    messageSent = true;
    lastSent = millis();
  }
}
