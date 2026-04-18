# Node 6 — Autostop Detector

## Purpose
Detects the autostop marker on the track using a QS18VP6LP photoelectric sensor and overrides the motor command to stop the train when `od_challenge_mode == 3` (autostop mode).

## Hardware
**Sensor:** Banner QS18VP6LP, powered at 12 V, discrete outputs.
- Has **two complementary outputs** (A and B are always binary opposites of each other).
- Both outputs are voltage-divided down to ESP32-safe levels (3.3 V logic).
- **Both signals are read every poll cycle.** If A and B are NOT opposites → circuit fault → send minor EMCY, do NOT trigger autostop.

**ESP32 pins (confirm with hardware team before finalising):**
- `SENSOR_A_PIN` — voltage-divided output A (HIGH = object detected)
- `SENSOR_B_PIN` — voltage-divided output B (must always equal `!A`)
- `TX_GPIO_NUM` / `RX_GPIO_NUM` — CAN transceiver

## Node ID: 6

## Challenge Mode Values (from OD Registry)
`od_challenge_mode` at 0x6062:00 (uint8_t):
- 1 = throttle control, **2 = speed control**, **3 = autostop**, 4 = regen, 5 = traction

`AUTOSTOP_CHALLENGE_VALUE = 3`

## Safety Logic (in priority order)
1. Always poll sensor every `SENSOR_POLL_MS` ms (non-blocking, use `millis()`).
2. On every poll, read both A and B. If `A == B` (not opposite) → **circuit fault**: send minor EMCY `0x00060601`, reset confirmation counter, do NOT trigger autostop.
3. Check `od_challenge_mode`. If not 3 → reset confirmation counter, no action.
4. If A=HIGH (object detected) AND B=LOW AND challenge_mode==3: increment `autostopConfirmCount`.
5. If `autostopConfirmCount >= AUTOSTOP_CONFIRM_COUNT`: trigger autostop. SDO write to Node 1 (motor) to stop. Send minor EMCY `0x00060602` (informational: autostop triggered).
6. When sensor clears (A=LOW) OR challenge_mode != 3: reset `autostopConfirmCount`.

Fail-safe: On stopped/preop modes, do nothing except keep monitoring sensor (so faults are caught in all states).

## CAN Communication

### How challenge mode is obtained
Node 6 SDO-reads `od_challenge_mode` (0x6062:00) from **Node 3** periodically (every `CHALLENGE_POLL_MS` ms).
- Per registry: Node 3 sheet shows "Transmit write request - Change challenge state - 0x609 - 0x6062" (Node 3 also SDO writes this to Node 9 LCD).
- Node 6 uses `executeSDORead(nodeID, 3, 0x6062, 0x00)` to poll the challenge mode.

### When autostop triggers — SDO write to Node 1
- `executeSDOWrite(nodeID, 1, 0x60FF, 0x00, sizeof(uint16_t), &zero)` — set motor command to 0 (using `0x60FF` per actual running code; registry has `0x606A`)
- Optionally also SDO write service brake to Node 2: `executeSDOWrite(nodeID, 2, 0x3012, 0x01, sizeof(uint8_t), &one)`

### EMCY codes for Node 6
| Code | Priority | Description |
|------|----------|-------------|
| 0x00060601 | Minor (1) | Sensor circuit fault — outputs A and B are not opposite |
| 0x00060602 | Minor (1) | Autostop triggered (informational) |

### Node 6 OD entries
| Index:Sub | Alias | Type | Direction | Description |
|-----------|-------|------|-----------|-------------|
| 0x3015:00 | od_autostop_detection | uint8_t | RW | 1=triggered, 0=clear. Register locally if needed. |

## Dependencies — Changes Required in Other Nodes
- **Node 3** (`Controller.ino`): Add Node 6 to `sendAllNMT()`.
- **Node 1** (`MotorVeroboard.ino`): No code changes required — autostop Node 6 directly SDO writes the motor command to 0. But Node 1 may benefit from registering `0x3015:00` to display/log autostop state.

## Constants (all must be `#define` in header)
```cpp
#define AUTOSTOP_CHALLENGE_VALUE  3    // od_challenge_mode value for autostop
#define AUTOSTOP_CONFIRM_COUNT    3    // consecutive positive reads required
#define SENSOR_POLL_MS            20   // ms between sensor polls
#define CHALLENGE_POLL_MS         200  // ms between SDO reads of challenge mode
```

## File Structure
```
nodes/node6-autostop/
    CLAUDE.md                            ← this file
    autostop_detector/
        autostop_detector.ino            ← top-level module (setup + loop + mode functions)
        autostop_detector.h              ← all #defines, enums, function prototypes, OD var declarations
        CHANGELOG.md
```

## Style Reference
Follow `nodes/node9-driver-screen/LCDscreen/` — it is the most standards-compliant node in the repo. Key things it does right:
- Full Doxygen file headers
- All constants as `static const` or `#define` in header
- `od_` prefixed OD variables with comment blocks
- `switch(OperatingMode)` loop structure
- `_UnderscorePrefixed` static helper functions
- No `handleCAN()` calls
