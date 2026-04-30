#include "DualSerial.h"

void DualSerialClass::begin(unsigned long baud, wifi_mode_t mode, char *ssid, char *passphrase, IPAddress localIP, IPAddress gateway, IPAddress subnet) {
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

void DualSerialClass::begin(unsigned long baud) {
  wifi_mode_t mode;
  if (NODE_ID == loggerID) {
    mode = WIFI_AP;
  } else {
    mode = WIFI_STA;
  }

  IPAddress localIP(10, 0, 0, NODE_ID);
  IPAddress gateway(10, 0, 0, loggerID); // logger assigned as gateway
  IPAddress subnet(255, 255, 255, 0);

  begin(115200, mode, "MREx CAN Logger", "YesWeCAN", localIP, gateway, subnet);
}

size_t DualSerialClass::write(uint8_t c) {
  return write(&c, 1);
}

size_t DualSerialClass::write(const uint8_t *buffer, size_t size) {
  serial.send((const char*) buffer);
  return Serial.write(buffer, size);
}

DualSerialClass DualSerial;