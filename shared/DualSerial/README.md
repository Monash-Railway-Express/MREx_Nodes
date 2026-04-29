# DualSerial - serial plus remote serial over WiFi

## Usage

```c
#include "../../../shared/DualSerial/DualSerial.h"
```

Anywhere you want to send to remote serial in addition to wired serial, replace `Serial` with `DualSerial`. For example:
```c
DualSerial.begin(115200);
DualSerial.println("MREx");
```

## Connecting to remote serial

Connect to the MREx CAN Logger WiFi from your device.

You can use the [Live Viewer](https://monash-railway-express.github.io/Live_Viewer/), and connect to `http://10.0.0.<NODE_ID>/serial`.