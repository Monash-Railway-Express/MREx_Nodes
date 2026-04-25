# DualSerial - serial plus remote serial over WiFi

## Code

Copy `DualSerial.h` to your node folder.

Example code:
```c
#include <WiFi.h>
#include <ESPAsyncWebServer.h> // ESP Async WebServer by ESP32Async
#include <AsyncTCP.h> // Async TCP by ESP32Async
#include "DualSerial.h"

const wifi_mode_t WIFI_MODE = WIFI_STA;
const char *SSID = "MREx CAN Logger";
const char *PASSPHRASE = "YesWeCAN";
const IPAddress LOCAL_IP(10, 0, 0, NODE_ID);
const IPAddress GATEWAY(10, 0, 0, 8); // logger assigned as gateway
const IPAddress SUBNET(255, 255, 255, 0);

AsyncWebServer server(80);

void setup() {
	DualSerial.begin(115200, WIFI_MODE, SSID, PASSPHRASE, LOCAL_IP, GATEWAY, SUBNET, &server);
	server.begin();
}
```

`mode` should be `WIFI_STA`, except for gateway nodes (WiFi access points) which will use `WIFI_AP`. The `gateway` variable should match the `localIP` of a gateway node.

Anywhere you want to send to remote serial in addition to wired serial, replace `Serial` with `DualSerial`. For example:
```c
DualSerial.println("MREx");
```

## Connecting to remote serial

Connect to the MREx CAN Logger WiFi from your device.

You can use the [Live Viewer](https://monash-railway-express.github.io/Live_Viewer/), and connect to `http://10.0.0.<NODE_ID>/serial`.