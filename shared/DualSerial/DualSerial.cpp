/**
 * @file DualSerial.cpp
 * @brief Print to both Serial and remote WiFi connection.
 *
 * @details
 * Implements DualSerial class. Drop-in replacement for Serial. Messages printed using DualSerial are pushed to both Serial Monitor and as Server-Sent Events.
 *
 * @author Nhan Nguyen
 *
 * @date 30/04/2026
 *
 * @version 1.0.0
 *
 * @organisation MREX
 */

#include "DualSerial.h"

/**
 * @brief Configure and set up serial connections. Includes Serial.begin() and web server setup.
 *
 * @param baud        Baud rate.
 * @param mode        WiFi connection mode, either WIFI_AP (access point) or WIFI_STA (station).
 * @param ssid        WiFi network SSID.
 * @param passphrase  WiFi network passphrase.
 * @param localIP     IP address of the node.
 * @param gateway     IP address of the gateway, normally the WiFi access point.
 * @param subnet      IP subnet mask.
 */
void DualSerialClass::begin(
  unsigned long baud,
  wifi_mode_t mode=WIFI_MODE_NULL,
  char *ssid="MREx CAN Logger",
  char *passphrase="YesWeCAN",
  IPAddress localIP=DEFAULT_LOCAL_IP,
  IPAddress gateway=DEFAULT_GATEWAY,
  IPAddress subnet=DEFAULT_SUBNET
) {
  if (mode == WIFI_MODE_NULL) {
    if (NODE_ID == loggerID) {
      mode = WIFI_AP;
    } else {
      mode = WIFI_STA;
    }
  }

  WIFI_MODE = mode;
  SSID = ssid;
  PASSPHRASE = passphrase;
  LOCAL_IP = localIP;
  GATEWAY = gateway;
  SUBNET = subnet;

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

  server.addHandler(&serial);
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  server.begin();
}

size_t DualSerialClass::write(uint8_t c) {
  return write(&c, 1);
}

size_t DualSerialClass::write(const uint8_t *buffer, size_t size) {
  serial.send((const char*) buffer);
  return Serial.write(buffer, size);
}

DualSerialClass DualSerial;