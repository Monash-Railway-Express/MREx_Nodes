

// void setup() {
//   // initialize serial communication at 115200 bits per second:
//   Serial.begin(9600);

//   //set the resolution to 12 bits (0-4095)
//   analogReadResolution(12);
// }

// void loop() {
//   // read the analog / millivolts value for pin 12:
//   int sensorval = analogRead(36);
//   float analogValue = sensorval * 3.3/4095.0;
//   int LEDoutput = map(sensorval, 0, 4095, 0, 255);
  

//   // print out the values you read:
//   Serial.printf("ADC analog value = %f\n", analogValue);
//   //Serial.printf("ADC millivolts value = %d\n", analogVolts);
//   Serial.println(analogValue);

//   dacWrite(25, LEDoutput);
  
//   delay(100);  // delay in between reads for clear read from serial
// }





#include "driver/twai.h"

#define CAN_TX 17
#define CAN_RX 16
#define POT_PIN 36       // ADC1 channel 0
#define DAC_PIN 25       // Local DAC out for testing (optional)

void setup() {
  Serial.begin(9600);
  analogReadResolution(12);  // 0–4095
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH); // Enable the CAN transceiver (if required)



  // Start CAN driver
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
    Serial.println("CAN started");
  } else {
    Serial.println("CAN failed to start");
    while (1);
  }
}

void loop() {
  int sensorval = analogRead(POT_PIN);
  float analogValue = sensorval * 3.3 / 4095.0;              // voltage
  int LEDoutput = map(sensorval, 0, 4095, 0, 255);           // scale to 8-bit

  // Debug to Serial
  Serial.printf("Analog voltage: %.2f V | Output: %d\n", analogValue, LEDoutput);

  // Optional DAC out on same board (for debugging)
  dacWrite(DAC_PIN, LEDoutput);

  // Construct CAN frame
  twai_message_t msg;
  msg.identifier = 0x101;                  // Custom ID for analog voltage
  msg.data_length_code = 1;                // 1 byte
  msg.data[0] = LEDoutput;                 // Scaled analog value
  msg.flags = 0;

  // Transmit over CAN
  if (twai_transmit(&msg, pdMS_TO_TICKS(100))) {
    Serial.println("Sent over CAN");
  } else {
    Serial.println("Failed to send CAN frame");
  }

  delay(100);
}

