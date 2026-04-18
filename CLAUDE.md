# MREx Nodes — Claude Context

## Project
ESP32 Arduino firmware for the MREx (Monash Railway Express) locomotive. All nodes communicate over a CAN bus using the CAN_MREx library. Safety is critical — the locomotive competes in the Railway Challenge.

## Mandatory Standards
All code must comply with **Software Standards and Guidelines v1.0.1** (PDF in repo root).  
Key rules in order of importance:

1. **File header** — every `.ino` and `.h` must start with a full Doxygen block: `@file`, `@brief`, `@details`, `@author`, `@date`, `@version`, `@organisation MREX`, `@see` (if applicable).
2. **OD variable naming** — all Object Dictionary variables must be prefixed `od_` in snake_case, e.g. `uint16_t od_regen_brake = 0;`. Comment above every OD var: index, subindex, description, units, direction, PDO mapping.
3. **Local variables** — lowerCamelCase. **Constants** — `UPPER_CASE_WITH_UNDERSCORES`. **Global variables** — snake_case (avoid). **Functions** — UpperCamelCase, verb phrases.
4. **No magic numbers** — all operating modes, pin numbers, OD addresses, thresholds must be `#define` or `static const`.
5. **No `delay()` in logic** — use `millis()` non-blocking patterns everywhere except init.
6. **No dynamic allocation** — no `malloc`, `new`, `delete`.
7. **loop() structure** — MUST use the `OperatingMode` enum switch pattern (see below).
8. **No `handleCAN()` calls** — deprecated since v1.13.0. CAN is handled by the FreeRTOS task.
9. **K&R braces** — opening brace on same line, closing on its own line.
10. **Functions ≤ 50 lines**, files ≤ 500 lines, nesting ≤ 3 levels.

### Mandatory loop() pattern
```cpp
void loop() {
    OperatingMode mode = static_cast<OperatingMode>(nodeOperatingMode);
    switch (mode) {
        case MODE_STOPPED:      StoppedMode();      break;
        case MODE_PREOP:        PreOpMode();        break;
        case MODE_OPERATIONAL:  OperationalMode();  break;
        default:                StoppedMode();      break;  // fail-safe
    }
}
```

### Mandatory enum in header
```cpp
enum OperatingMode : uint8_t {
    MODE_STOPPED     = 0x02,
    MODE_PREOP       = 0x80,
    MODE_OPERATIONAL = 0x01
};
```

### Mandatory setup() pattern
```cpp
initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, nodeID);  // nodeID must NOT be const
xTaskCreatePinnedToCore(CAN_Task, "CAN Task", 4096, &nodeID, 3, NULL, 0);
```

## CAN_MREx Library (v1.13.0)
- CAN is a FreeRTOS task. Never call `handleCAN()`.
- `nodeOperatingMode` (global `uint8_t`) reflects the current NMT state.
- `registerODEntry(index, subindex, access, size, &var)` — access: 0=RO, 1=WO, 2=RW.
- `configureTPDO(pdoNum, cobId, transType=255, inhibitMs, eventMs)` / `mapTPDO(pdoNum, entries, count)`
- `configureRPDO(pdoNum, cobId, transType=255, inhibitMs)` / `mapRPDO(pdoNum, entries, count)`
- `executeSDOWrite(srcNodeId, destNodeId, index, subindex, size, &value)`
- `executeSDORead(srcNodeId, destNodeId, index, subindex)` → returns `uint32_t`
- `sendNMT(command, destNodeId)` — commands: 0x01 start, 0x02 stop, 0x80 preop
- `sendEMCY(priority, nodeId, errorCode)` — priority: 0=major, 1=minor
- `checkMajorEMCY()` / `checkMinorEMCY()` → bool (true if buffer changed)
- `getMajorByIndex(idx, &node, &code)` / `getMinorByIndex(idx, &node, &code)`
- `enableHeartbeatMonitoring(true)` in setup() to monitor other nodes
- `markTpdoDirty(pdoNum)` to trigger immediate TPDO send

## Node Map
| ID | Folder | Purpose | TX GPIO | RX GPIO |
|----|--------|---------|---------|---------|
| 1 | node1-motor | Motor PWM + regen brake | 4 | 5 |
| 2 | node2-brakes | Service brake relay (fail-safe: brakes ON by default) | 18 | 19 |
| 3 | node3-driver-inputs | NMT master, all driver inputs, pots/switches/buttons | 41 | 42 |
| 4 | node4-lights | FWD/REV/PREOP lights + smoke/temp sensors | 4 | 5 |
| 5 | node5-audio | DFPlayer Mini horn | 35 | 36 |
| 6 | node6-autostop | Autostop detector (QS18VP6LP) | TBD | TBD |
| 8 | node8-logger | WiFi/SD CAN logger | — | — |
| 9 | node9-driver-screen | LVGL TFT driver display | TBD | TBD |

## Key OD Index Map (from CAN MREX Nodes Registry.xlsx)
| Index:Sub | Alias | Type | States / Notes |
|-----------|-------|------|----------------|
| 0x3012:00 | od_regen_brake | uint16_t | 0–1023; Node3 TPDO1 → Node1 RPDO1 |
| 0x3012:01 | od_service_brake_mc | uint8_t | 1=on,0=off; Node1 SDO→Node2; Node3 SDO→Node2 (parking) |
| 0x3012:02 | od_service_brake_dc | uint8_t | 1=on,0=off; from driver controls |
| 0x3012:03 | od_brake_threshold | — | km/h threshold for service brakes |
| 0x606A:00 | od_motor_command | uint16_t | Speed/throttle target; Node3 TPDO1 → Node1 RPDO1 |
| 0x606C:00 | od_true_speed | uint32_t | Measured speed km/h; Node1 TPDO1 → Node9 RPDO4 |
| 0x606E:00 | od_tractive_effort | uint32_t | Node1 TPDO1 → Node9 RPDO4 |
| 0x60F6:00–0E | od_kp/ki/kd_1–5 | uint8_t | PID constants per condition mode |
| 0x6060:00 | od_direction_mode | uint8_t | 1=reverse,2=neutral,3=forward; Node3 SDO→Node1 |
| 0x6061:00 | od_condition_mode | uint8_t | 1–5 traction condition; Node3 |
| 0x6062:00 | od_challenge_mode | uint8_t | **1=throttle,2=speed,3=autostop,4=regen,5=traction**; Node3 SDO→Node9 (0x609); Node6 SDO reads from Node3 |
| 0x6064:00 | od_parking_brake | — | Node3 SDO→Node2 (parking switch) |
| 0x6065:00 | od_horn_toggle | uint8_t | 1=play,0=default; Node3 SDO→Node5 |
| 0x1004:00 | od_temperature_front | uint16_t | Node4 TPDO1 |
| 0x1004:01 | od_temperature_rear | uint16_t | Node4 TPDO2 |
| 0x2000:00 | od_recovered_energy | uint32_t | Battery node |
| 0x3015:00 | od_autostop_detection | uint8_t | **TBD** — autostop trigger (Node6 → Node1) |

**Code vs Registry discrepancy:** Node1 and Node3 code uses `0x60FF:00` for motor command, but the registry defines `0x606A:00` as `od_motor_command`. Both are registered in Node1's OD. Use `0x60FF:00` when targeting the actual running code; use `0x606A:00` per the registry spec.

## COB-ID Assignments
Formula: function_code + nodeID. Key ones in use:
- TPDO1 node1: 0x181, TPDO1 node3: 0x183, TPDO1 node4: 0x184, TPDO1 node7: 0x187
- TPDO2 node7: 0x287, TPDO3 node7: 0x387
- RPDO1 node1: listens on 0x183 (node3's TPDO1)
- RPDO1 node9: 0x183, RPDO2 node9: 0x187, RPDO3 node9: 0x287, RPDO4 node9: 0x181

## EMCY Error Code Registry (from Emergency sheet)
| Code | Priority | Description |
|------|----------|-------------|
| 0x00000001 | Minor | OD entry not found after SDO |
| 0x00000008 | Major | SDO response not received (timeout) |
| 0x00000101 | Major | Heartbeat not received in time |
| 0x00000201 | Major | NMT command failure |
| 0x00000301 | Major | Max minor EMCY count reached |
| 0x00000500 | Minor | Audio SD card fault |
| 0x00000505 | Major | Smoke detected |
| 0x00000506 | Major | Temperature front too high |
| 0x00000507 | Major | Temperature rear too high |
| 0x02000010 | Minor | Brake fault |
| 0x02000011 | Minor | Error reading speed for brake fault check |
| **Node 6 codes TBD** | | Assign in 0x060xxx range |

## Node 9 (LCDscreen) — Best Standards Reference
This is the most standards-compliant node in the repo. When unsure about coding style, refer to `nodes/node9-driver-screen/LCDscreen/LCDscreen.ino` and `LCDscreen.h`.

## Things to Note When Making Changes
- Adding a new node: also update `sendAllNMT()` in `nodes/node3-driver-inputs/Controller/Controller.ino`.
- Any OD variable written by both its own node AND another node must be protected with a mutex.
- All OD entries must be registered in the CAN MREX Nodes Registry (Google Sheets).
- All CHANGELOG.md files follow https://keepachangelog.com/en/1.1.0/ format.
- Branch naming: `NodeName/feature/short-description` from `dev`; hotfixes from `main`.
