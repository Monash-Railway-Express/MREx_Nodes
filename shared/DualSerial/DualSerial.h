/**
 * @file DualSerial.h
 * @brief Print to both Serial and remote WiFi connection.
 *
 * @details
 * Provides DualSerial class inheriting from Arduino's Print. Drop-in replacement for Serial. Messages printed using DualSerial are pushed to both Serial Monitor and as Server-Sent Events.
 *
 * @author Nhan Nguyen
 *
 * @date 30/04/2026
 *
 * @version 1.0.0
 *
 * @organisation MREX
 *
 * @see DualSerial.cpp
 * @see https://github.com/ayushsharma82/WebSerial/tree/master/src
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h> // ESP Async WebServer by ESP32Async
#include <AsyncTCP.h> // Async TCP by ESP32Async

#ifndef DUALSERIAL_H
#define DUALSERIAL_H

const uint8_t loggerID = 8;
const IPAddress DEFAULT_LOCAL_IP(10, 0, 0, NODE_ID);
const IPAddress DEFAULT_GATEWAY(10, 0, 0, loggerID); // logger assigned as gateway
const IPAddress DEFAULT_SUBNET(255, 255, 255, 0);

AsyncWebServer server(80);
AsyncEventSource serial("/serial");

class DualSerialClass : public Print {
  public:
    void begin(unsigned long baud, wifi_mode_t mode, char *ssid, char *passphrase, IPAddress local_ip, IPAddress gateway, IPAddress subnet);
    size_t write(uint8_t) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    wifi_mode_t WIFI_MODE;
    char *SSID;
    char *PASSPHRASE;
    IPAddress LOCAL_IP;
    IPAddress GATEWAY;
    IPAddress SUBNET;
};

#endif // DUALSERIAL_H