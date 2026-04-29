void DualSerialClass::begin(unsigned long baud, wifi_mode_t mode, const char *ssid, const char *passphrase, IPAddress localIP, IPAddress gateway, IPAddress subnet) {
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
  if (NODE_ID == loggerID) {
    DEFAULT_WIFI_MODE = WIFI_AP;
  } else {
    DEFAULT_WIFI_MODE = WIFI_STA;
  }

  begin(115200, DEFAULT_WIFI_MODE, DEFAULT_SSID, DEFAULT_PASSPHRASE, DEFAULT_LOCAL_IP, DEFAULT_GATEWAY, DEFAULT_SUBNET);
}

size_t DualSerialClass::write(uint8_t c) {
  return write(&c, 1);
}

size_t DualSerialClass::write(const uint8_t *buffer, size_t size) {
  serial.send((const char*) buffer);
  return Serial.write(buffer, size);
}

DualSerialClass DualSerial;