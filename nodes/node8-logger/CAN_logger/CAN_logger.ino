/**
 * CAN Logger file 
 *
 * File:            CAN_logger.ino
 * Organisation:    MREX
 * Author:          Chiara Gillam
 * Date Created:    12/10/2025
 * Last Modified:   12/10/2025
 * Version:         1.10.1
 *
 */



// Libraries used:
// https://wiki.dfrobot.com/DS3231M%20MEMS%20Precise%20RTC%20SKU:%20DFR0641



#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "driver/twai.h"
#include "DFRobot_DS3231M.h" // https://github.com/DFRobot/DFRobot_DS3231M
#include <WiFi.h>
#include <ESPAsyncWebServer.h> // ESP Async WebServer by ESP32Async
#include <AsyncTCP.h> // Async TCP by ESP32Async
#include "FS.h"
#include <ArduinoJson.h> // ArduinoJson by Benoit Blanchon
#include "wshtml.h"
#include <CAN_MREx.h>

const uint8_t nodeID = 8;  // Change this to set your device's node ID

// --- Pin Definitions ---
#define TX_GPIO_NUM GPIO_NUM_5 // Set GPIO pin for CAN Transmit
#define RX_GPIO_NUM GPIO_NUM_4 // Set GPIO pins for CAN Receive

const char* SSID = "MREx CAN Logger";
const char* PASSWORD = "YesWeCAN";
const char* URL = "10.0.0.1";
IPAddress LOCAL(10, 0, 0, 1);
IPAddress GATEWAY(10, 0, 0, 1);
IPAddress SUBNET(255, 255, 255, 0);
const char* FILEPATH = "/files";
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// RTC
DFRobot_DS3231M rtc;

// SD card
const int SD_CS = 26;
File logFile;
String logFilename;
String indexHTML = "";
const char* indexHTMLc;

// Ring buffer config
const int BUFFER_SIZE = 32;
struct CANFrame {
  String timestamp;
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
};
CANFrame buffer[BUFFER_SIZE];
int bufferIndex = 0;
unsigned long lastFlush = 0;
unsigned long lastUpdate = 0;

struct Parameter {
  String key;
  bool sign;
  uint8_t targetNode;
  uint16_t index;
  uint8_t subindex;
  size_t size;
};
struct Parameter parameters[] = {
  {"kProportional1", 0, 14, 0x60F6, 0x00, sizeof(uint8_t)},
  {"kIntegral1", 0, 14, 0x60F6, 0x01, sizeof(uint8_t)},
  {"kDerivative1", 0, 14, 0x60F6, 0x02, sizeof(uint8_t)},
  {"kProportional2", 0, 14, 0x60F6, 0x03, sizeof(uint8_t)},
  {"kIntegral2", 0, 14, 0x60F6, 0x04, sizeof(uint8_t)},
  {"kDerivative2", 0, 14, 0x60F6, 0x05, sizeof(uint8_t)},
  {"kProportional3", 0, 14, 0x60F6, 0x06, sizeof(uint8_t)},
  {"kIntegral3", 0, 14, 0x60F6, 0x07, sizeof(uint8_t)},
  {"kDerivative3", 0, 14, 0x60F6, 0x08, sizeof(uint8_t)},
  {"kProportional4", 0, 14, 0x60F6, 0x09, sizeof(uint8_t)},
  {"kIntegral4", 0, 14, 0x60F6, 0x0A, sizeof(uint8_t)},
  {"kDerivative4", 0, 14, 0x60F6, 0x0B, sizeof(uint8_t)},
  {"kProportional5", 0, 14, 0x60F6, 0x0C, sizeof(uint8_t)},
  {"kIntegral5", 0, 14, 0x60F6, 0x0D, sizeof(uint8_t)},
  {"kDerivative5", 0, 14, 0x60F6, 0x0E, sizeof(uint8_t)},
};
const int num_parameters = 15;

int getParameterIdx(String key) {
  for (int i = 0; i < num_parameters; i++) {
    if (parameters[i].key == key) {
      return i;
    }
  }
  return -1;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // RTC init
  while(rtc.begin() != true){
    Serial.println("Failed to init chip, please check if the chip connection is fine. ");
    delay(1000);
  }

    // SD init
  while (!SD.begin(SD_CS)) {
    Serial.println("SD init failed");
  }
  Serial.print(SD.usedBytes());
  Serial.print(" bytes out of ");
  Serial.println(SD.totalBytes());
  Serial.println(" used on SD card.");

  // File name init
  int testNumber = 0;
  char filename[13]; // 8.3 format: "/YYMMDDNN.CSV"
  bool fileExists = true;

  rtc.getNowTime();
  do {
    sprintf(filename, "/%02d%02d%02d%02d.CSV", rtc.year() % 100, rtc.month(), rtc.day(), testNumber);
    fileExists = SD.exists(filename);
    if (fileExists) testNumber++;
  } while (fileExists && testNumber < 100); // Limit to 00–99

  logFilename = String(filename);

  // Open log file and write header
  logFile = SD.open(logFilename, FILE_WRITE);
  if (logFile) {
    logFile.println("Timestamp,ID,DLC,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7");
  } else {
    Serial.println("Failed to open log file");
    while (1);
  }

  indexHTML += "<!DOCTYPE html><html lang=\"en\"><head><title>MREx CAN Logger</title><head><body>";
  indexHTML += "<h1>Hello from the MREx CAN logger.</h1><p>Live feed: <a href=\"http://10.0.0.1/feed\">http://10.0.0.1/feed</a> (WebSocket: ws://10.0.0.1/ws)</p><h2>Directory listing</h2><ul>";
  indexHTML += listDir(SD, "/", 1000);
  indexHTML += "</ul></body></html>";
  indexHTMLc = indexHTML.c_str();
  
  //Initialize CANMREX protocol
  initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID); // this or below

  // // CAN init
  // twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
  // twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  // twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK ||
  //     twai_start() != ESP_OK) {
  //   Serial.println("CAN init failed");
  //   while (1);
  // }
  
  Serial.println("CAN logging started");
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID, PASSWORD);
  WiFi.softAPConfig(LOCAL, GATEWAY, SUBNET);
  delay(100);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->sendChunked("text/html", [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
      if (indexHTML.length() <= index) {
        return 0;
      }

      const size_t chunkSize = min((size_t)32768, min(maxLen, indexHTML.length()-index));
      memcpy(buffer, indexHTMLc+index, chunkSize);
      return chunkSize;
    });
  });

  server.on(AsyncURIMatcher::exact(FILEPATH), HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", listDir(SD, "/", 1000));
  });

  server.on("/feed", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", wshtml);
  });

  server.serveStatic(FILEPATH, SD, "/");

  server.on("/munt", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument muntDoc;
    for (int i = 0; i < num_parameters; i++) {
      struct Parameter parameter = parameters[i];
      if (parameter.sign) {
        int32_t value = executeSDORead(nodeID, parameter.targetNode, parameter.index, parameter.subindex);
        muntDoc[parameter.key] = value;
      } else {
        uint32_t value = executeSDORead(nodeID, parameter.targetNode, parameter.index, parameter.subindex);
        muntDoc[parameter.key] = value;
      }
    }

    String response;
    serializeJson(muntDoc, response);
    request->send(200, "application/json", response);
  });

  server.on("/munt", HTTP_PATCH, [](AsyncWebServerRequest *request) {
    JsonDocument responseDoc;
    JsonArray responseArray = responseDoc.to<JsonArray>();

    if (request->hasParam("body", true)) {
      JsonDocument requestDoc;
      deserializeJson(requestDoc, request->getParam("body", true)->value());
      for (JsonPair pair : requestDoc.as<JsonObject>()) {
        String key = pair.key().c_str();
        for (int i = 0; i < num_parameters; i++) {
          struct Parameter parameter = parameters[i];
          if (parameter.key == key) {
            responseArray.add(key);
            if (parameter.sign) {
              int32_t requestValue = pair.value(); // implicit cast
              executeSDOWrite(nodeID, parameter.targetNode, parameter.index, parameter.subindex, parameter.size, &requestValue);
            } else {
              uint32_t requestValue = pair.value(); // implicit cast
              executeSDOWrite(nodeID, parameter.targetNode, parameter.index, parameter.subindex, parameter.size, &requestValue);
            }
          }
        }
      }
    }

    String response;
    serializeJson(responseDoc, response);
    request->send(202, "application/json", response);
  });

  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      Serial.printf("Client connected: %u\n", client->id());
    } else if (type == WS_EVT_DISCONNECT) {
      Serial.printf("Client disconnected: %u\n", client->id());
    } else if (type == WS_EVT_DATA) {
      Serial.printf("Received data from client %u\n", client->id());
      data[len] = 0;
      Serial.println((char *) data);
    }
  });

  server.addHandler(&ws);
  server.begin();
  
  Serial.println("Webserver started.");
  Serial.print("SSID: ");
  Serial.println(SSID);
  Serial.print("Password: ");
  Serial.println(PASSWORD);
  Serial.print("HTTP: http://");
  Serial.println(URL);
  Serial.print("WebSocket: ws://");
  Serial.print(URL);
  Serial.println("/ws");
  Serial.print("MUNT: http://");
  Serial.print(URL);
  Serial.println("/munt");
}

void loop() {
  // handleCAN(nodeID); // causes unnecessary logging delays
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
    rtc.getNowTime();

    CANFrame frame;
    frame.timestamp = String(rtc.year()) + "-" +
                      String(rtc.month()) + "-" +
                      String(rtc.day()) + " " +
                      String(rtc.hour()) + ":" +
                      String(rtc.minute()) + ":" +
                      String(rtc.second());

    // // Fallback timestamp using millis
    // frame.timestamp = String(millis());

    frame.id = message.identifier;
    frame.dlc = message.data_length_code;
    for (int i = 0; i < 8; i++) {
      frame.data[i] = (i < message.data_length_code) ? message.data[i] : 0;
    }

    buffer[bufferIndex++] = frame;

    // Flush if buffer full
    if (bufferIndex >= BUFFER_SIZE) {
      flushBuffer();
    }
  }

  // Periodic flush
  if (millis() - lastFlush > 500 && bufferIndex > 0) {
    flushBuffer();
  }

//   // Simulate a CAN frame every second
//   static unsigned long lastSend = 0;
//   if (millis() - lastSend > 1000) {
//     lastSend = millis();

//     // Create JSON message
//     DynamicJsonDocument doc(128);
//     doc["ts"] = millis();
//     doc["id"] = "0x123";
//     JsonArray data = doc.createNestedArray("data");
//     data.add(0x01);
//     data.add(0x02);
//     data.add(0x03);
//     data.add(0x04);

//     String json;
//     serializeJson(doc, json);

//     // Send to all connected clients
//     ws.textAll(json);
//     Serial.println("Sent: " + json);
//   }
}

void flushBuffer() {
  for (int i = 0; i < bufferIndex; i++) {
    char id[20];
    sprintf(id, "%lX", buffer[i].id);

    String row;
    row += buffer[i].timestamp;
    row += ",0x";
    row += id;
    row += ",";
    row += buffer[i].dlc;
    for (int j = 0; j < 8; j++) {
      row += ",";
      if (j < buffer[i].dlc) {
        row += "0x";
        char str[5];
        sprintf(str, "%02lX", buffer[i].data[j]);
        row += str;
      }
    }
    logFile.println(row);
    ws.textAll(row);
  }
  logFile.flush();
  bufferIndex = 0;
  lastFlush = millis();
}

String listDir(fs::FS &fs, String dirname, uint8_t levels) {
  String listing = "";

  File root = fs.open(dirname);
  if (!root) {
    return "";
  }
  if (!root.isDirectory()) {
    return "";
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      if (levels) {
        String newDirname = file.path();
        newDirname += "/";
        listing += listDir(fs, newDirname, levels - 1);
      }
    } else {
      String path = "http://";
      path += URL;
      path += FILEPATH;
      path += dirname;
      path += file.name();

      listing += "<li><a href=\"";
      listing += path;
      listing += "\">";
      listing += path;
      listing += "</a> ";
      listing += file.size();
      listing += "</li>";
    }
    file = root.openNextFile();
  }

  return listing;
}