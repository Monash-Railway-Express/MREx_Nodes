# DualSerial - serial plus remote serial over WiFi

## Usage

```c
const uint8_t NODE_ID = 99;
#include "../../../shared/DualSerial/DualSerial.cpp"
```

`..` represents a parent directory, up one level in the directory tree.

Anywhere you want to send to remote serial in addition to wired serial, replace `Serial` with `DualSerial`. For example:
```c
DualSerial.begin(115200);
DualSerial.print(DualSerial.LOCAL_IP);
DualSerial.println("/serial");
```

## Connecting to remote serial

Connect to the MREx CAN Logger WiFi from your device.

You can use the [Live Viewer](https://monash-railway-express.github.io/Live_Viewer/), and connect to `http://10.0.0.<NODE_ID>/serial`.