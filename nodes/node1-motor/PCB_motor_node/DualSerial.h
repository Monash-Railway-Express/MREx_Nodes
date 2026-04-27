/**
 * @file DualSerial.cpp
 * @brief Print to both Serial and remote WiFi connection.
 *
 * @details
 * Provides DualSerial class inheriting from Arduino's Print. Messages printed using DualSerial are pushed to both Serial and as Server-Sent Events.
 *
 * @author Nhan Nguyen
 *
 * @date 24/04/2026
 *
 * @version 1.0.0
 *
 * @organisation MREX
 *
 * @see https://github.com/ayushsharma82/WebSerial/tree/master/src
 * @see /modules/DualSerial
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h> // ESP Async WebServer by ESP32Async
#include <AsyncTCP.h> // Async TCP by ESP32Async

#ifndef DUALSERIAL_CPP
#define DUALSERIAL_CPP

AsyncEventSource serial("/serial");

class DualSerialClass : public Print {
  public:
    void begin(unsigned long baud, wifi_mode_t mode, const char *ssid, const char *passphrase, IPAddress local_ip, IPAddress gateway, IPAddress subnet, AsyncWebServer *server);
    size_t write(uint8_t) override;
    size_t write(const uint8_t *buffer, size_t size) override;
};

void DualSerialClass::begin(unsigned long baud, wifi_mode_t mode, const char *ssid, const char *passphrase, IPAddress localIP, IPAddress gateway, IPAddress subnet, AsyncWebServer *serverPointer) {
  Serial.begin(baud);
  WiFi.mode(mode);

  if (mode == WIFI_AP) {
    WiFi.softAP(ssid, passphrase);
    WiFi.softAPConfig(localIP, gateway, subnet);
    delay(100);
  }

  if (mode == WIFI_STA) {
    WiFi.begin(ssid, passphrase);
    delay(100);
    WiFi.config(localIP, gateway, subnet);
  }

  serverPointer->addHandler(&serial);
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
}

size_t DualSerialClass::write(uint8_t c) {
  return write(&c, 1);
}

size_t DualSerialClass::write(const uint8_t *buffer, size_t size) {
  serial.send((const char*) buffer);
  return Serial.write(buffer, size);
}

DualSerialClass DualSerial;

#endif // DUALSERIAL_CPP