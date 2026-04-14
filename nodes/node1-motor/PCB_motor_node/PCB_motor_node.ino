/**
 * @file PCB_motor_node.ino
 * @brief PCB motor controller node.
 *
 * @details
 * Motor controller node for the PCB implementation.
 * Includes implementation of different challenge modes.
 *
 * @author Chiara Gillam
 * @author Sean Larkin
 *
 * @date 13/03/2026
 *
 * @version 2.0.0
 *
 * @organisation MREX
 *
 * @see motor_PCB.h
 * @see CHANGELOG.md
 */

#include <CAN_MREx.h>
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "motor_PCB.h"
#include "driver/pcnt.h"

// =============================================================================
// OD Variable Definitions
// =============================================================================

// OD 0x606C:00 – Actual vehicle speed in km/h (0–UINT32_MAX). RW. Mapped to TPDO0.
uint32_t od_true_speed = 0;

// OD 0x3012:00 – Regen brake request (0–1023). RW. Mapped to RPDO0.
uint16_t od_regen_brake = 0;

// OD 0x3012:01 – Service brake active flag (0 = "Braking", 1 = "Not Braking"). RW. Mapped to SDO.
uint8_t od_service_brake_mc = 0;

// OD 0x606A:00 – Raw motor command (0–255). RW. Mapped to RPDO0.
uint16_t od_motor_command = 0;

// OD 0x6061:00 – Condition, 1-5 1 being best track conditions 5 being worst track conditions (Not implemented yet)
uint8_t od_condition_mode = 0;

// OD 0x6060:00 – Direction mode mirror. RW. Mapped to RPDO0.
uint8_t od_direction_mode = 0;

// OD 0x6062:00 – Challenge mode selector (1=Normal, 2=Speed, 3=AutoStop,
//                4=EnergyRecovery, 5=Traction). RW. Mapped to RPDO0.
uint8_t od_challenge_mode = 0;

// OD 0x2000:00 – Recovered energy from battery node (Wh). Read-only. Mapped to RPDO1.
uint32_t od_recovered_energy = 0;

// 

// =============================================================================
// Global Variables
// =============================================================================

// Cumulative pulse count for distance tracking — updated each loop before counter clear
int32_t total_pulse_accum = 0;

// PI controller integrator accumulator
float integrator = 0.0f;

// Anti-windup clamp for integrator
float integrator_limit = 300.0f;

// Timing for 10 Hz control loop
unsigned long previous_millis = 0;

// Preferences API for non-volatile memory
Preferences preferences;

const int numFloatPairs = 15;

struct FloatPair floatPairs[numFloatPairs];

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Serial Coms started at 115200 baud");

    preferences.begin("od_params", false);
    GetPreferences();                           // obtains existing PID parameters

    initCANMREX(TX_GPIO_NUM, RX_GPIO_NUM, NODE_ID);

    xTaskCreatePinnedToCore(
        CAN_Task,
        "CAN Task",
        6144,
        &NODE_ID,
        3,
        NULL,
        0
    );

    // Setting up DAC
    Wire.begin(SDA_PIN, SCL_PIN);
    CommonConfig();

    // Setting up Rotary Encoder
    SetupPCNT();

    pinMode(REVERSING_CONTACTOR, OUTPUT);
    pinMode(ISOLATING_RELAY, OUTPUT);

    // --- Register OD entries ---
    registerODEntry(0x606C, 0x00, 2, sizeof(od_true_speed), &od_true_speed);                // TPDO - 32bit
    registerODEntry(0x3012, 0x00, 2, sizeof(od_regen_brake), &od_regen_brake);              // RPDO - 16bit
    registerODEntry(0x3012, 0x01, 2, sizeof(od_service_brake_mc), &od_service_brake_mc);    // SDO - 8bit
    registerODEntry(0x606A, 0x00, 2, sizeof(od_motor_command), &od_motor_command);          // RPDO - 8bit
    registerODEntry(0x6061, 0x00, 2, sizeof(od_condition_mode), &od_condition_mode);        // RPDO - 8bit
    registerODEntry(0x6060, 0x00, 2, sizeof(od_direction_mode), &od_direction_mode);        // RPDO - 8bit
    registerODEntry(0x6062, 0x00, 2, sizeof(od_challenge_mode), &od_challenge_mode);        // RPDO - 8bit
    registerODEntry(0x2000, 0x00, 2, sizeof(od_recovered_energy), &od_recovered_energy);    // RPDO - 32bit
    
    
    // When adding a new pair, ensure to update numFloatPairs
    // Assign values here rather than statically above as floats do not initialise properly otherwise
    floatPairs[0] = {"od_kp_1", 0.1f, 0x60F6, 0x00};
    floatPairs[1] = {"od_ki_1", 0.2f, 0x60F6, 0x01};
    floatPairs[2] = {"od_kd_1", 0.3f, 0x60F6, 0x02};
    floatPairs[3] = {"od_kp_2", 0.4f, 0x60F6, 0x03};
    floatPairs[4] = {"od_ki_2", 0.5f, 0x60F6, 0x04};
    floatPairs[5] = {"od_kd_2", 0.6f, 0x60F6, 0x05};
    floatPairs[6] = {"od_kp_3", 0.7f, 0x60F6, 0x06};
    floatPairs[7] = {"od_ki_3", 0.8f, 0x60F6, 0x07};
    floatPairs[8] = {"od_kd_3", 0.9f, 0x60F6, 0x08};
    floatPairs[9] = {"od_kp_4", 0.0f, 0x60F6, 0x09};
    floatPairs[10] = {"od_ki_4", 0.1f, 0x60F6, 0x0A};
    floatPairs[11] = {"od_kd_4", 0.2f, 0x60F6, 0x0B};
    floatPairs[12] = {"od_kp_5", 0.3f, 0x60F6, 0x0C};
    floatPairs[13] = {"od_ki_5", 0.4f, 0x60F6, 0x0D};
    floatPairs[14] = {"od_kd_5", 0.5f, 0x60F6, 0x0E};
    for (int i = 0; i < numFloatPairs; i++) {
        // Assigning floatPairs[i] to a variable first does not work to update floatPairs[i].value on SDO writes
        registerODEntry(floatPairs[i].index, floatPairs[i].subindex, 2, sizeof(floatPairs[i].value), &(floatPairs[i].value)); // SDO
    }

    // --- Register TPDOs ---
    configureTPDO(0, 0x180 + NODE_ID, 255, 100, 100);
    PdoMapEntry tpdo_entries[] = {
        {0x606C, 0x00, 32}  // od_true_speed
    };
    mapTPDO(0, tpdo_entries, 1);

    // --- Register RPDOs ---
    configureRPDO(0, 0x180 + 3, 255, 0);
    PdoMapEntry rpdo_entries[] = {
        {0x3012, 0x00, 16},  // od_regen_brake
        {0x606A, 0x00, 16}   // od_motor_command
    };
    mapRPDO(0, rpdo_entries, 2);

    configureRPDO(1, 0x280 + 7, 255, 0);  // COB-ID matches battery node TPDO2 (0x280 + nodeID 7)
    PdoMapEntry rpdo1_entries[] = {
        {0x2000, 0x04, 32},  // recovered_energy_can
    };
    mapRPDO(1, rpdo1_entries, 1);
}


// =============================================================================
// LOOP
// =============================================================================

void loop() {
    OperatingMode mode = static_cast<OperatingMode>(nodeOperatingMode);

    

    switch (mode) {
        case MODE_STOPPED:     StoppedMode();     break;
        case MODE_PREOP:       PreOpMode();       break;
        case MODE_OPERATIONAL: OperationalMode(); break;
        default:               StoppedMode();     break;  // fail-safe
    }
}


// =============================================================================
// OPERATING MODE HANDLERS
// =============================================================================

/**
 * @brief Stopped state — zero all outputs, reset integrator and put preferences to NVM.
 */
void StoppedMode() {
    WriteDAC(0, 0);
    WriteDAC(1, 0);
    integrator = 0.0f;
    PutPreferences();
}

/**
 * @brief Pre-operational state — zero outputs, reset integrator, set direction contactor.
 *
 * @details
 * Zeroes all drive outputs and sets the reversing contactor based on od_direction_mode.
 * Mode 1 = reverse, 2 = neutral (contactor off), 3 = forward.
 */
void PreOpMode() {
    WriteDAC(0, 0);
    WriteDAC(1, 0);
    integrator = 0.0f;

    switch (od_direction_mode) {
        case 1:  // Reverse
            digitalWrite(REVERSING_CONTACTOR, HIGH);
            Serial.println("[PreOp] Direction: Reverse");
            break;
        case 2:  // Neutral — open contactor
            digitalWrite(REVERSING_CONTACTOR, LOW);
            Serial.println("[PreOp] Direction: Neutral");
            break;
        case 3:  // Forward
            digitalWrite(REVERSING_CONTACTOR, LOW);
            Serial.println("[PreOp] Direction: Forward");
            break;
        default:  // Unknown — fail safe to neutral
            digitalWrite(REVERSING_CONTACTOR, LOW);
            Serial.print("[PreOp] Unknown direction mode: ");
            Serial.println(od_direction_mode);
            break;
    }
}

void OperationalMode() {
    unsigned long current_millis = millis();
    if (current_millis - previous_millis < LOOP_INTERVAL_MS) return;

    // --- Read pulse count BEFORE ReadSpeedKMH() clears the counter ---
    int16_t pulse_count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &pulse_count);

    // Accumulate total pulses for distance tracking
    total_pulse_accum += (int32_t)pulse_count;

    float speed_kmh = ReadSpeedKMH(previous_millis);  // clears counter here
    previous_millis = current_millis;
    od_true_speed   = (uint32_t)speed_kmh;

    switch (od_challenge_mode) {
        case CHALLENGE_THROTTLE:        ThrottleControl(speed_kmh);                   break;
        case CHALLENGE_SPEED_CONTROL:   SpeedControl(speed_kmh);                      break;
        case CHALLENGE_AUTO_STOP:       AutoStopChallenge(speed_kmh, total_pulse_accum); break;
        case CHALLENGE_ENERGY_RECOVERY: EnergyRecoveryChallenge(speed_kmh);           break;
        case CHALLENGE_TRACTION:        TractionChallenge(speed_kmh);                 break;
        default:                        ThrottleControl(speed_kmh);                   break;
    }

    // Hard speed cap
    if (speed_kmh > MAX_SPEED_KMH) {
        WriteDAC(0, 0);
        integrator = 0.0f;
        Serial.println("[OperationalMode] Speed cap exceeded — throttle cut.");
        return;
    }

}

// =============================================================================
// CONTROL MODES
// =============================================================================

/**
 * @brief PI speed controller — drives to od_motor_command setpoint.
 *
 * @details
 * Uses a PI loop to compute motor DAC output. Regen brake overrides
 * throttle when active. Service brake applied below SERVICE_BRAKE_SPEED_KMH.
 * Runs every LOOP_INTERVAL_MS (0.1s).
 *
 * @param speed_kmh Current measured speed in km/h.
 */
void SpeedControl(float speed_kmh) {
    uint16_t motor_dac = 0;
    uint16_t brake_dac = 0;

    if (od_regen_brake > REGEN_BRAKE_THRESHOLD) {
        motor_dac  = 0;
        brake_dac  = od_regen_brake;
        integrator = 0.0f;
       //od_service_brake_mc = (speed_kmh <= SERVICE_BRAKE_SPEED_KMH) ? 1 : 0;
    } else {
       //od_service_brake_mc = 0;

        // Scale od_motor_command (0–1023) to speed setpoint (0–MAX_SPEED_KMH)
        float speed_setpoint = ((float)od_motor_command / 1023.0f) * MAX_SPEED_KMH;
        float error = speed_setpoint - speed_kmh;

        // CONSTANT TRACTION MODE 1
        float_t ki = floatPairs[1].value;
        float_t kp = floatPairs[0].value;

        integrator += error * ki * LOOP_INTERVAL_MS/1000;

        if (integrator > integrator_limit) integrator = integrator_limit;
        if (integrator < 0.0f)            integrator = 0.0f;

        float control = (error * kp) + integrator;
        if (control < 0.0f)    control = 0.0f;
        if (control > DAC_MAX) control = (float)DAC_MAX;

        motor_dac = (uint16_t)control;
        brake_dac = 0;
    }

    WriteDAC(0, motor_dac);
    WriteDAC(1, brake_dac);
    // executeSDOWrite(NODE_ID, 2, 0x3012, 0x01, sizeof(od_service_brake_mc), od_service_brake_mc) {

    Serial.print("[SpeedControl] Setpoint: "); Serial.print(((float)od_motor_command / 1023.0f) * MAX_SPEED_KMH);
    Serial.print(" | Speed: ");                Serial.print(speed_kmh);
    Serial.print(" | Motor DAC: ");            Serial.print(motor_dac);
    Serial.print(" | Brake DAC: ");            Serial.println(brake_dac);
}

// spare speed control function in case other does not work

// /**
//  * @brief Simple speed controller — accelerates below target speed and brakes above it.
//  *
//  * @details
//  * No PI loop. The motor command is treated as a target speed request:
//  * - below target speed  -> drive motor
//  * - above target speed  -> apply brake
//  * - within a small deadband -> do nothing
//  *
//  * Regen brake still overrides throttle.
//  *
//  * @param speed_kmh Current measured speed in km/h.
//  */
// void SpeedControl(float speed_kmh) {
//     uint16_t motor_dac = 0;
//     uint16_t brake_dac = 0;

//     // Convert raw command (0–1023) to speed setpoint (0–MAX_SPEED_KMH)
//     float speed_setpoint = ((float)od_motor_command / 1023.0f) * MAX_SPEED_KMH;
//     float error = speed_setpoint - speed_kmh;

//     const float deadband = 0.5f;     // km/h
//     const float motor_gain = 80.0f;  // tune to suit
//     const float brake_gain = 80.0f;  // tune to suit

//     // Regen brake override
//     if (od_regen_brake > REGEN_BRAKE_THRESHOLD) {
//         motor_dac = 0;
//         brake_dac = od_regen_brake;
//         od_service_brake = (speed_kmh <= SERVICE_BRAKE_SPEED_KMH) ? 1 : 0;
//     }
//     else {
//         od_service_brake = 0;

//         if (error > deadband) {
//             // Below target speed -> accelerate
//             float cmd = error * motor_gain;
//             if (cmd > DAC_MAX) cmd = DAC_MAX;
//             motor_dac = (uint16_t)cmd;
//             brake_dac = 0;
//         }
//         else if (error < -deadband) {
//             // Above target speed -> decelerate
//             float cmd = (-error) * brake_gain;
//             if (cmd > DAC_MAX) cmd = DAC_MAX;
//             motor_dac = 0;
//             brake_dac = (uint16_t)cmd;
//         }
//         else {
//             // Close enough to target
//             motor_dac = 0;
//             brake_dac = 0;
//         }
//     }

//     WriteDAC(0, motor_dac);
//     WriteDAC(1, brake_dac);

//     Serial.print("[SpeedControl] Setpoint: ");
//     Serial.print(speed_setpoint);
//     Serial.print(" | Speed: ");
//     Serial.print(speed_kmh);
//     Serial.print(" | Motor DAC: ");
//     Serial.print(motor_dac);
//     Serial.print(" | Brake DAC: ");
//     Serial.println(brake_dac);
// }

/**
 * @brief Throttle control using motor_command.
 *
 * @details
 * Uses od_motor_command directly (like your snippet):
 * - high command + no regen  -> motor on
 * - high command + regen     -> brake
 * - low command + regen      -> brake + service brake
 * - otherwise                -> stop + service brake
 *
 * @param speed_kmh Current measured speed in km/h.
 */
void ThrottleControl(float speed_kmh) {
    uint16_t motor_dac = 0;
    uint16_t brake_dac = 0;

    if (od_motor_command > 10 && od_regen_brake <= 10) {
        motor_dac = od_motor_command;   // full 10-bit range
        brake_dac = 0;
       //od_service_brake_mc= 0;
    }
    else if (od_motor_command > 10 && od_regen_brake > 10) {
        motor_dac = 0;
        brake_dac = od_regen_brake;
       //od_service_brake_mc= 0;
    }
    else if (od_motor_command <= 10 && od_regen_brake > 10) {
        motor_dac = 0;
        brake_dac = od_regen_brake;
       //od_service_brake_mc= (speed_kmh <= SERVICE_BRAKE_SPEED_KMH) ? 1 : 0;
    }
    else {
        motor_dac = 0;
        brake_dac = 0;
       //od_service_brake_mc= 1;
    }

    WriteDAC(0, motor_dac);
    WriteDAC(1, brake_dac);

    Serial.print("[SpeedControl] CMD: ");
    Serial.print(od_motor_command);
    Serial.print(" | Speed: ");
    Serial.print(speed_kmh);
    Serial.print(" | Motor DAC: ");
    Serial.print(motor_dac);
    Serial.print(" | Brake DAC: ");
    Serial.print(brake_dac);
    // Serial.print(" | Service Brake: ");
    // Serial.println(od_service_brake_mc);
}


/**
 * @brief Auto-stop challenge — throttle tapers to zero at 25m then braking ramp to stop.
 *
 * @details
 * On first call, snapshots total_pulse_accum as the start reference.
 * Distance is computed from pulses accumulated in OperationalMode() before
 * each counter clear, avoiding interference with ReadSpeedKMH().
 * Throttle scales proportionally from full at 0m to zero at 25m, naturally
 * decelerating the train. Once stopped, service brake is applied.
 * Non-blocking — uses static state variables across calls.
 *
 * @param speed_kmh   Current measured speed in km/h.
 * @param pulse_accum Total accumulated pulse count from OperationalMode().
 */
void AutoStopChallenge(float speed_kmh, int32_t pulse_accum) {
    static bool    entered_auto_stop     = false;
    static int32_t pulse_start = 0;

    uint16_t motor_dac = 0;
    uint16_t brake_dac = 0;

    // --- Initialise on first entry ---
    if (!entered_auto_stop) {
        pulse_start = pulse_accum;
        entered_auto_stop = true;
        Serial.println("[AutoStop] Challenge started — throttle tapering over 25m.");
    }

    // --- Compute distance from accumulated pulses ---
    // 200 pulses = 1 rev = 0.3m  →  25m ≈ 16,667 pulses
    int32_t pulses_since_start = pulse_accum - pulse_start;
    float distance_m = ((float)pulses_since_start / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE_M;

    // --- Zone 1: Fully stopped ---
    if (speed_kmh <= 0.1f) {
        WriteDAC(0, 0);
        WriteDAC(1, 0);
       //od_service_brake_mc = 1;
        integrator       = 0.0f;
        entered_auto_stop = false;  // Reset for next run
        Serial.println("[AutoStop] Stopped — service brake applied.");
        return;
    }

    // --- Zone 2: 25m reached — throttle zero, regen brake holds until stopped ---
    if (distance_m >= AUTO_STOP_DISTANCE_M) {
        motor_dac        = 0;
       //od_service_brake_mc = 0;
        integrator       = 0.0f;

        brake_dac = (uint16_t)(DAC_MAX);

        WriteDAC(0, motor_dac);
        WriteDAC(1, brake_dac);

        Serial.print("[AutoStop] Coasting to stop — Distance: "); Serial.print(distance_m);
        Serial.print("m | Speed: ");                              Serial.print(speed_kmh);
        Serial.print(" | Brake DAC: ");                           Serial.println(brake_dac);
        return;
    }

    // --- Zone 3: Under 25m — throttle tapers proportionally to distance remaining ---
    // At 0m  → distance_fraction = 1.0 → full od_motor_command
    // At 25m → distance_fraction = 0.0 → zero throttle
    float distance_fraction = 1.0f - (distance_m / AUTO_STOP_DISTANCE_M);
    motor_dac = (uint16_t)(od_motor_command * distance_fraction);
    brake_dac = 0;
   //od_service_brake_mc = 0;

    WriteDAC(0, motor_dac);
    WriteDAC(1, brake_dac);

    Serial.print("[AutoStop] Tapering — Distance: "); Serial.print(distance_m);
    Serial.print("m | Fraction: ");                   Serial.print(distance_fraction);
    Serial.print(" | Motor DAC: ");                   Serial.println(motor_dac);
}

/**
 * @brief Traction challenge — speed control with wheel slip detection.
 *
 * @details
 * Runs PI speed control. If measured speed exceeds desired speed by more than
 * a fixed slip margin, throttle is cut to prevent wheel spin.
 *
 * @param speed_kmh Current measured speed in km/h.
 *
 * TODO(Sean): Tune TRACTION_SLIP_MARGIN_KMH once hardware testing is complete.
 */
void TractionChallenge(float speed_kmh) {
    const float TRACTION_SLIP_MARGIN_KMH = 2.0f;

    float speed_setpoint = ((float)od_motor_command / 1023.0f) * MAX_SPEED_KMH;

    if (speed_kmh > speed_setpoint + TRACTION_SLIP_MARGIN_KMH) {
        WriteDAC(0, 0);
        WriteDAC(1, 0);
       //od_service_brake_mc = 0;
        integrator = 0.0f;

        Serial.print("[Traction] Slip detected. Speed: "); Serial.println(speed_kmh);
        return;
    }

    SpeedControl(speed_kmh);
}


/**
 * @brief Energy recovery challenge — measure regen energy then limit motoring to recovered amount.
 *
 * @details
 * Phase 1: Waits for regen brake input. On first brake, snapshots od_recovered_energy
 *          as energy_brake_start. Applies regen braking, suppressing service brake.
 * Phase 2: Train comes to stop. Snapshots od_recovered_energy as energy_brake_end.
 *          Recovered energy = energy_brake_end - energy_brake_start.
 *          Holds stationary, waiting for throttle input.
 * Phase 3: On first throttle input, snapshots od_recovered_energy as energy_motor_start.
 *          Allows motoring only while energy used < energy recovered.
 *          Cuts throttle once budget is exhausted.
 * Non-blocking — uses static state variables across calls.
 *
 * @param speed_kmh Current measured speed in km/h.
 */
void EnergyRecoveryChallenge(float speed_kmh) {

    // Challenge phases
    enum EnergyPhase : uint8_t {
        PHASE_WAITING_FOR_BRAKE  = 0,
        PHASE_BRAKING            = 1,
        PHASE_STOPPED_IDLE       = 2,
        PHASE_MOTORING           = 3,
        PHASE_BUDGET_EXHAUSTED   = 4
    };

    static EnergyPhase phase             = PHASE_WAITING_FOR_BRAKE;
    static uint32_t    energy_brake_start  = 0;
    static uint32_t    energy_brake_end    = 0;
    static uint32_t    energy_motor_start  = 0;
    static uint32_t    energy_recovered    = 0;

    uint16_t motor_dac = 0;
    uint16_t brake_dac = 0;

    switch (phase) {

        // ----------------------------------------------------------------
        case PHASE_WAITING_FOR_BRAKE:
        // Throttle control active — waiting for driver to initiate braking
        // ----------------------------------------------------------------
           //od_service_brake_mc = 0;

            if (od_motor_command > 10 && od_regen_brake <= 10) {
                motor_dac = od_motor_command;
                brake_dac = 0;
            } else if (od_regen_brake > 10) {
                motor_dac = 0;
                brake_dac = od_regen_brake;
            } else {
                motor_dac = 0;
                brake_dac = 0;
            }

            WriteDAC(0, motor_dac);
            WriteDAC(1, brake_dac);

            // Transition to braking phase on first regen brake input
            if (od_regen_brake > REGEN_BRAKE_THRESHOLD) {
                energy_brake_start = od_recovered_energy;
                phase              = PHASE_BRAKING;
                Serial.print("[EnergyRecovery] Braking started — energy snapshot: ");
                Serial.println(energy_brake_start);
            }
            break;

        // ----------------------------------------------------------------
        case PHASE_BRAKING:
        // Regen braking active — suppress service brake to maximise recovery
        // ----------------------------------------------------------------
            motor_dac        = 0;
            brake_dac        = od_regen_brake;
           //od_service_brake_mc = 0;  // Suppress — keep energy in regen circuit

            WriteDAC(0, motor_dac);
            WriteDAC(1, brake_dac);

            digitalWrite(ISOLATING_RELAY, HIGH);

            // Transition to idle once train is stopped
            if (speed_kmh <= 0.1f) {
                energy_brake_end  = od_recovered_energy;
                energy_recovered  = energy_brake_end - energy_brake_start;
               //od_service_brake_mc = 1;
                WriteDAC(0, 0);
                WriteDAC(1, 0);
                phase = PHASE_STOPPED_IDLE;

                Serial.print("[EnergyRecovery] Stopped — energy recovered: ");
                Serial.print(energy_recovered);
                Serial.println(" (waiting for throttle)");
            }

            Serial.print("[EnergyRecovery] Braking — Brake DAC: "); Serial.println(brake_dac);
            break;

        // ----------------------------------------------------------------
        case PHASE_STOPPED_IDLE:
        // Stationary — holding service brake, waiting for first throttle input
        // ----------------------------------------------------------------
            WriteDAC(0, 0);
            WriteDAC(1, 0);
           //od_service_brake_mc = 1;

            if (od_motor_command > 10) {
                // First throttle received — snapshot energy at this moment
                energy_motor_start = od_recovered_energy;
               //od_service_brake_mc  = 0;
                phase              = PHASE_MOTORING;
                Serial.print("[EnergyRecovery] Throttle received — motor energy start: ");
                Serial.println(energy_motor_start);
            }
            break;

        // ----------------------------------------------------------------
        case PHASE_MOTORING:
        // Motoring — allow throttle only while energy used < energy recovered
        // ----------------------------------------------------------------
        {
            uint32_t energy_used = (energy_motor_start > od_recovered_energy)
                                 ? (energy_motor_start - od_recovered_energy)
                                 : 0;

            if (energy_used >= energy_recovered) {
                WriteDAC(0, 0);
                WriteDAC(1, 0);
               //od_service_brake_mc = 0;
                phase            = PHASE_BUDGET_EXHAUSTED;
                Serial.println("[EnergyRecovery] Energy budget exhausted — throttle cut.");
                break;
            }

            // Budget remaining — raw throttle pass-through
           //od_service_brake_mc = 0;

            if (od_motor_command > 10 && od_regen_brake <= 10) {
                motor_dac = od_motor_command;
                brake_dac = 0;
            } else if (od_regen_brake > 10) {
                motor_dac = 0;
                brake_dac = od_regen_brake;
            } else {
                motor_dac = 0;
                brake_dac = 0;
            }

            WriteDAC(0, motor_dac);
            WriteDAC(1, brake_dac);

            Serial.print("[EnergyRecovery] Used: "); Serial.print(energy_used);
            Serial.print(" / ");                     Serial.print(energy_recovered);
            Serial.print(" | Motor DAC: ");          Serial.print(motor_dac);
            Serial.print(" | Brake DAC: ");          Serial.println(brake_dac);
            break;
        }

        // ----------------------------------------------------------------
        case PHASE_BUDGET_EXHAUSTED:
        // Throttle permanently cut — challenge complete
        // ----------------------------------------------------------------
            digitalWrite(ISOLATING_RELAY, HIGH);
            WriteDAC(0, 0);
            WriteDAC(1, 0);
           //od_service_brake_mc = 0;
            integrator       = 0.0f;

            // Reset if challenge mode is switched away and back
            // — handled by entered flag resetting on mode change if needed
            Serial.println("[EnergyRecovery] Challenge complete — budget exhausted.");
            break;

        default:
            phase = PHASE_WAITING_FOR_BRAKE;
            break;
    }
}


// =============================================================================
// HARDWARE ABSTRACTION
// =============================================================================

/**
 * @brief Reads the quadrature encoder and returns vehicle speed in km/h.
 *
 * @details
 * Reads and clears the PCNT counter, converts pulse count to revolutions,
 * then to linear speed using the elapsed time since prev_time.
 *
 * @param prev_time Timestamp of the last speed reading in milliseconds (from millis()).
 *
 * @return Measured speed in km/h.
 */
float ReadSpeedKMH(unsigned long prev_time) {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
    pcnt_counter_clear(PCNT_UNIT_0);

    float elapsed_s = (millis() - prev_time) / 1000.0f;  // Elapsed time in seconds

    if (elapsed_s <= 0.0f) return 0.0f;  // Guard against divide-by-zero

    float rev       = count / PULSES_PER_REV;
    float meters    = rev * WHEEL_CIRCUMFERENCE_M;
    float speed_mps = meters / elapsed_s;
    return speed_mps * 3.6f;
}


/**
 * @brief Powers up both DAC output channels with internal voltage reference.
 */
void CommonConfig() {
    Wire.beginTransmission(DAC_ADDR);
    Wire.write(0x1F);  // COMMON-CONFIG register
    Wire.write(0x02);  // VOUT0 ON
    Wire.write(0x01);  // VOUT1 ON
    Wire.endTransmission();
}


/**
 * @brief Writes a 10-bit value to the specified DAC channel over I2C.
 *
 * @param channel DAC channel (0 = motor, 1 = regen brake).
 * @param value   10-bit output value (0–1023). Clamped if exceeded.
 */
void WriteDAC(uint8_t channel, uint16_t value) {
    if (value > DAC_MAX) value = DAC_MAX;

    uint16_t reg_val = value << 6;  // Left-align 10-bit value into 16-bit register
    uint8_t  reg     = (channel == 0) ? 0x1C : 0x19;  // DAC-0-DATA / DAC-1-DATA

    Wire.beginTransmission(DAC_ADDR);
    Wire.write(reg);
    Wire.write(reg_val >> 8);
    Wire.write(reg_val & 0xFF);
    Wire.endTransmission();
}


/**
 * @brief Configures the PCNT peripheral for quadrature encoder input.
 *
 * @details
 * ENCODER_A is the pulse input. ENCODER_B controls count direction.
 * Filter set to 1000 APB clock cycles to debounce encoder signals.
 */
void SetupPCNT() {
    pcnt_config_t pcnt_config = {};
    pcnt_config.pulse_gpio_num = ENCODER_A;
    pcnt_config.ctrl_gpio_num  = ENCODER_B;
    pcnt_config.channel        = PCNT_CHANNEL_0;
    pcnt_config.unit           = PCNT_UNIT_0;
    pcnt_config.pos_mode       = PCNT_COUNT_INC;     // Count on rising edge
    pcnt_config.neg_mode       = PCNT_COUNT_DIS;     // Ignore falling edge
    pcnt_config.lctrl_mode     = PCNT_MODE_REVERSE;  // B low = reverse direction
    pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;     // B high = forward direction
    pcnt_config.counter_h_lim  = PCNT_HIGH_LIMIT;
    pcnt_config.counter_l_lim  = PCNT_LOW_LIMIT;

    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(PCNT_UNIT_0, 1000);
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
}

/**
 * @brief Gets variable values from non-volatile memory.
 */
void GetPreferences() {
    // for (int i = 0; i < numFloatPairs; i++) {
    //     floatPairs[i].value = preferences.getFloat(floatPairs[i].key.c_str(), 0.f);
    // }
    
    floatPairs[0].value = 0.0021616f;
    floatPairs[1].value = 0.20276f;
}

/**
 * @brief Puts variable values into non-volatile memory.
 */
void PutPreferences() {
    for (int i = 0; i < numFloatPairs; i++) {
        preferences.putFloat(floatPairs[i].key.c_str(), floatPairs[i].value);
    }
}