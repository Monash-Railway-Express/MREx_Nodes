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
 * @author Nhan Nguyen
 *
 * @date 28/04/2026
 *
 * @version 2.1.0
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
#include "../../../shared/DualSerial/DualSerial.cpp"



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
uint8_t od_challenge_mode = 1;
uint8_t prev_challenge_mode = 0;

// OD 0x2000:04 – Recovered energy from battery node (Wh). Read-only. Mapped to RPDO1.
uint32_t od_recovered_energy = 0;

// OD 0x1050:00 - Autostop lineside marker detection. unsigned 0 or 1, edge=detected
uint8_t od_autostop_detection = 0;

uint32_t od_current_power = 0;

// =============================================================================
// Global Variables
// =============================================================================

bool    entered_auto_stop = false;
int32_t pulse_start       = 0;

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

bool new_autostop_instance = true;


uint16_t motor_dac = 0;
uint16_t brake_dac = 0;
float over_speed_damping = 1;


// =============================================================================
// SETUP
// =============================================================================



void setup() {
	DualSerial.begin(115200);
    delay(1000);
    DualSerial.println("DualSerial Coms started at 115200 baud");

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
    pinMode(BRAKE_SWITCH, OUTPUT);

    // --- Register OD entries ---
    registerODEntry(0x606C, 0x00, 2, sizeof(od_true_speed), &od_true_speed);                // TPDO - 32bit
    registerODEntry(0x3012, 0x00, 2, sizeof(od_regen_brake), &od_regen_brake);              // RPDO - 16bit
    registerODEntry(0x3012, 0x01, 2, sizeof(od_service_brake_mc), &od_service_brake_mc);    // SDO - 8bit
    registerODEntry(0x606A, 0x00, 2, sizeof(od_motor_command), &od_motor_command);          // RPDO - 16bit
    registerODEntry(0x6061, 0x00, 2, sizeof(od_condition_mode), &od_condition_mode);        // SDO - 8bit
    registerODEntry(0x6060, 0x00, 2, sizeof(od_direction_mode), &od_direction_mode);        // SDO - 8bit
    registerODEntry(0x6062, 0x00, 2, sizeof(od_challenge_mode), &od_challenge_mode);        // SDO - 8bit
    registerODEntry(0x2000, 0x03, 2, sizeof(od_current_power), &od_current_power);    // RPDO - 32bit
    registerODEntry(0x2000, 0x04, 2, sizeof(od_recovered_energy), &od_recovered_energy);    // RPDO - 32bit
    registerODEntry(0x1050, 0x00, 2, sizeof(od_autostop_detection), &od_autostop_detection);    // SDO - 8bit
    
    
    // When adding a new pair, ensure to update numFloatPairs
    // Assign values here rather than statically above as floats do not initialise properly otherwise
    floatPairs[0] = {"od_kp_1", 0.016442f, 0x60F6, 0x00};
    floatPairs[1] = {"od_ki_1", 1.7177f, 0x60F6, 0x01};
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
    configureTPDO(0, 0x180 + MOTOR_ID, 255, 100, 100);
    PdoMapEntry tpdo_entries[] = {
        {0x606C, 0x00, 32}  // od_true_speed
    };
    mapTPDO(0, tpdo_entries, 1);

    // --- Register RPDOs ---
    configureRPDO(0, 0x180 + DRIVER_ID, 255, 0);
    PdoMapEntry rpdo_entries[] = {
        {0x3012, 0x00, 16},  // od_regen_brake
        {0x606A, 0x00, 16}   // od_motor_command
    };
    mapRPDO(0, rpdo_entries, 2);

    /**
    * TODO
    */
    configureRPDO(1, 0x280 + BATTERY_ID, 255, 0);

    PdoMapEntry rpdo1_entries[] = {
        {0x2000, 0x03, 32},  // power_can       — aligns bytes 0-3
        {0x2000, 0x04, 32}   // recovered_energy_can — now correctly at bytes 4-7
    };
    mapRPDO(1, rpdo1_entries, 2);
}



// =============================================================================
// LOOP
// =============================================================================



void loop() {
    // completing operation mode functionality
    switch (nodeOperatingMode) {
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
    WriteDAC(THROTTLE_CHANNEL, uint16_t(motor_dac * over_speed_damping));
    WriteDAC(REGEN_CHANNEL, 0);
    digitalWrite(BRAKE_SWITCH, LOW);
    integrator = 0.0f;
    PutPreferences(); // we need to add a params dirty od entry so we only update these when they change
}

/**
 * @brief Pre-operational state — zero outputs, reset integrator, set direction contactor.
 *
 * @details
 * Zeroes all drive outputs and sets the reversing contactor based on od_direction_mode.
 * Mode 1 = reverse, 2 = neutral (contactor off), 3 = forward.
 */
void PreOpMode() {
    WriteDAC(THROTTLE_CHANNEL, uint16_t(motor_dac * over_speed_damping));
    WriteDAC(REGEN_CHANNEL, 0);
    digitalWrite(BRAKE_SWITCH, LOW);


    integrator = 0.0f;

    switch (od_direction_mode) {
        case REVERSE_MODE:  // Reverse
            digitalWrite(REVERSING_CONTACTOR, HIGH);
            DualSerial.println("[PreOp] Direction: Reverse");
            break;
        case NEUTRAL_MODE:  // Neutral — open contactor
            digitalWrite(REVERSING_CONTACTOR, LOW);
            DualSerial.println("[PreOp] Direction: Neutral");
            break;
        case FORWARD_MODE:  // Forward
            digitalWrite(REVERSING_CONTACTOR, LOW);
            DualSerial.println("[PreOp] Direction: Forward");
            break;
        default:  // Unknown — fail safe to neutral
            digitalWrite(REVERSING_CONTACTOR, LOW);
            DualSerial.print("[PreOp] Unknown direction mode: ");
            DualSerial.println(od_direction_mode);
            break;
    }
}

void OperationalMode() {
    unsigned long current_millis = millis();
    if (current_millis - previous_millis < LOOP_INTERVAL_MS) return;

    // --- Read pulse count BEFORE ReadSpeedKMH() clears the counter ---
    int16_t pulse_count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &pulse_count);      // grabs count value from rotary encoder

    // Accumulate total pulses for distance tracking
    total_pulse_accum += (int32_t)pulse_count;              // adds value to accumulated pulses

    float speed_kmh = ReadSpeedKMH(previous_millis);        // clears counter here
    previous_millis = current_millis;
    od_true_speed   = (uint32_t)speed_kmh;


    // Detect mode transitions
    if (od_challenge_mode != prev_challenge_mode) {
        OnChallengeModeExit(prev_challenge_mode);
        OnChallengeModeEnter(od_challenge_mode);
    }


    switch (od_challenge_mode) {
        case CHALLENGE_THROTTLE:        
            entered_auto_stop = false;
            ThrottleControl(speed_kmh);   
            break;

        case CHALLENGE_SPEED_CONTROL:   
            entered_auto_stop = false;
            SpeedControl(speed_kmh);                            
            break;

        case CHALLENGE_AUTO_STOP:       
            AutoStopChallenge(speed_kmh, total_pulse_accum);    
            break;

        case CHALLENGE_ENERGY_RECOVERY: 
            entered_auto_stop = false;
            EnergyRecoveryChallenge(speed_kmh);                 
            break;

        case CHALLENGE_TRACTION:        
            entered_auto_stop = false;
            TractionChallenge(speed_kmh);                       
            break;

        default:                        
            entered_auto_stop = false;
            ThrottleControl(speed_kmh);                         
            break;
    }

    prev_challenge_mode = od_challenge_mode;

    // // Hard speed cap
    // if (speed_kmh > MAX_SPEED_KMH) {
    //     WriteDAC(THROTTLE_CHANNEL, uint16_t(motor_dac * over_speed_damping));
    //     integrator = 0.0f;
    //     // sendEMCY(1, MOTOR_ID, 0x00510);     // send minor emergency - TODO: DECIDE ON PROPER ERROR CODE
    //     DualSerial.println("[OperationalMode] Speed cap exceeded — throttle cut.");
    //     return;
    // }

    limit_speed(speed_kmh, MAX_SPEED_KMH);

}

void limit_speed(float current_speed, int max_speed){
    
    //accesses global variables, motor_dac and brake_dac
    float damping_factor = 0.9;
    if(current_speed > max_speed){
        over_speed_damping = over_speed_damping * damping_factor;
    } else {
        over_speed_damping = over_speed_damping / damping_factor;
    }
    
    if(over_speed_damping > 1){
        over_speed_damping = 1;
    }

    if(over_speed_damping < 0.1){
        over_speed_damping = 0.1;
    }
    return;
}

/**
 * @brief When exiting a challenge mode it will change these 
 */
void OnChallengeModeExit(uint8_t old_mode) {
    if (old_mode == CHALLENGE_AUTO_STOP) {
        SetServiceBrake(false);   // release
        new_autostop_instance = true;
    }
    if (old_mode == CHALLENGE_ENERGY_RECOVERY) {
        digitalWrite(ISOLATING_RELAY, LOW);
        SetServiceBrake(false);   // release
    }
}


/**
 * @brief Gets variable values from non-volatile memory.
 */
void OnChallengeModeEnter(uint8_t new_mode) {
    // entry setup per mode, if needed
}


// =============================================================================
// CONTROL MODES
// =============================================================================
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
    motor_dac = 0;
    brake_dac = 0;

    // if (od_motor_command > 0 && od_regen_brake <= 0) {
    //     motor_dac = od_motor_command;   // full 10-bit range
    //     brake_dac = 0;
    // }
    // else if (od_motor_command > 0 && od_regen_brake > 0) {
    //     motor_dac = 0;
    //     brake_dac = od_regen_brake;
    // }
    // else if (od_motor_command <= 0 && od_regen_brake > 0) {
    //     motor_dac = 0;
    //     brake_dac = od_regen_brake;
    // }
    // else {
    //     motor_dac = 0;
    //     brake_dac = 0;
    // }

    // Write to Throttle and Regen DACs
    WriteDAC(THROTTLE_CHANNEL, uint16_t(od_motor_command*over_speed_damping));
    WriteDAC(REGEN_CHANNEL, od_regen_brake);
    if(od_regen_brake > 0){
        digitalWrite(BRAKE_SWITCH, HIGH);
    } else {
        digitalWrite(BRAKE_SWITCH, LOW);
    }

    DualSerial.print("[SpeedControl] CMD: ");
    DualSerial.print(od_motor_command);
    DualSerial.print(" | Speed: ");
    DualSerial.print(speed_kmh);
    DualSerial.print(" | Motor DAC: ");
    DualSerial.print(motor_dac);
    DualSerial.print(" | Brake DAC: ");
    DualSerial.print(brake_dac);
}

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
    motor_dac = 0;
    brake_dac = 0;

    if (od_regen_brake > REGEN_BRAKE_THRESHOLD) {
        motor_dac  = 0;
        brake_dac  = od_regen_brake;
        integrator = 0.0f;
    } else {
        // Scale od_motor_command (0–1023) to speed setpoint (0–MAX_SPEED_KMH)
        // float speed_setpoint = ((float)od_motor_command / 1023.0f) * MAX_SPEED_KMH;
        float speed_setpoint = ((float)od_motor_command / (float)DAC_MAX) * MAX_SPEED_KMH;
        float error = speed_setpoint - speed_kmh;

        // CONSTANT TRACTION MODE 1
        float_t ki = floatPairs[1].value;
        float_t kp = floatPairs[0].value;

        integrator += error * ki * LOOP_INTERVAL_MS/1000;

        if (integrator > integrator_limit) integrator = integrator_limit;
        if (integrator < 0.0f)            integrator = 0.0f;

        float control = (error * kp) + integrator;
        if (control < 0.0f)    control = 0.0f;
        // if (control > DAC_MAX) control = (float)DAC_MAX;
        
        //multiplying control by 1024;
        //I think PI params were caculated for 0 to 1 output signal not 0 to 1024
        //just multiplying by 100
        control = control * 25;
        if (control > DAC_MAX) control = (float)DAC_MAX;

        motor_dac = (uint16_t)control;
        brake_dac = 0;
    }

    WriteDAC(THROTTLE_CHANNEL, uint16_t(over_speed_damping * motor_dac));
    WriteDAC(REGEN_CHANNEL, brake_dac);
    if(brake_dac > 0){
        digitalWrite(BRAKE_SWITCH, HIGH);
    } else {
        digitalWrite(BRAKE_SWITCH, LOW);
    }
    
    DualSerial.print("[SpeedControl] Setpoint: "); DualSerial.print(((float)od_motor_command / 1023.0f) * MAX_SPEED_KMH);
    DualSerial.print(" | Speed: ");                DualSerial.print(speed_kmh);
    DualSerial.print(" | Motor DAC: ");            DualSerial.print(motor_dac);
    DualSerial.print(" | Brake DAC: ");            DualSerial.println(brake_dac);
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



    motor_dac = 0;
    brake_dac = 0;


    if (!entered_auto_stop && new_autostop_instance) {
        if (od_autostop_detection == 0) {
            // Waiting for lineside marker — normal throttle control
            ThrottleControl(speed_kmh);
            return;
        }

        pulse_start                = pulse_accum;
        entered_auto_stop          = true;
        DualSerial.println("[AutoStop] Challenge started");
    }

    if (entered_auto_stop) {
    int32_t pulses_since_start = pulse_accum - pulse_start;
    float distance_m = ((float)pulses_since_start / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE_M;


    // --- Zone 2: Stopped ---
    if (speed_kmh <= 0.1f) {
        WriteDAC(THROTTLE_CHANNEL, uint16_t(motor_dac * over_speed_damping));
        WriteDAC(REGEN_CHANNEL, 0);
        digitalWrite(BRAKE_SWITCH, LOW);
        integrator = 0.0f;
        SetServiceBrake(true);
        od_autostop_detection = 0;
        entered_auto_stop     = false;
        new_autostop_instance = false;
        DualSerial.println("[AutoStop] Stopped — service brake applied.");
        return;
    }

    // --- Zone 3: 25m reached — full regen brake ---
    if (distance_m >= AUTO_STOP_DISTANCE_M) {
        integrator = 0.0f;
        WriteDAC(THROTTLE_CHANNEL, uint16_t(motor_dac * over_speed_damping));
        WriteDAC(REGEN_CHANNEL, DAC_MAX);
        digitalWrite(BRAKE_SWITCH, HIGH);
        DualSerial.print("[AutoStop] Coasting to stop — Distance: "); DualSerial.print(distance_m);
        DualSerial.print("m | Speed: "); DualSerial.print(speed_kmh);
        DualSerial.print(" | Brake DAC: "); DualSerial.println(DAC_MAX);
        return;
    }

    // --- Zone 1: Throttle taper (under 25m, still moving) ---
    float distance_fraction = 1.0f - (distance_m / AUTO_STOP_DISTANCE_M);
    if (distance_fraction < 0.0f) distance_fraction = 0.0f;
    motor_dac = (uint16_t)(od_motor_command * distance_fraction);
    WriteDAC(THROTTLE_CHANNEL, motor_dac);
    WriteDAC(REGEN_CHANNEL, 0);
    digitalWrite(BRAKE_SWITCH, LOW);
    DualSerial.print("[AutoStop] Tapering — Distance: "); DualSerial.print(distance_m);
    DualSerial.print(" | Fraction: ");                    DualSerial.print(distance_fraction);
    DualSerial.print(" | Motor DAC: ");                   DualSerial.print(motor_dac);
    DualSerial.print(" | Speed: ");                       DualSerial.println(speed_kmh);
    }
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
    // const float TRACTION_SLIP_MARGIN_KMH = 2.0f;

    // float speed_setpoint = ((float)od_motor_command / 1023.0f) * MAX_SPEED_KMH;

    // if (speed_kmh > speed_setpoint + TRACTION_SLIP_MARGIN_KMH) {
    //     WriteDAC(THROTTLE_CHANNEL, uint16_t(motor_dac * over_speed_damping));
    //     WriteDAC(REGEN_CHANNEL, 0);
    //    //od_service_brake_mc = 0;
    //     integrator = 0.0f;

    //     DualSerial.print("[Traction] Slip detected. Speed: "); DualSerial.println(speed_kmh);
    //     return;
    // }

    // SpeedControl(speed_kmh);
    ThrottleControl(speed_kmh);
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
    digitalWrite(ISOLATING_RELAY, HIGH);

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

    motor_dac = 0;
    brake_dac = 0;

    switch (phase) {

        // ----------------------------------------------------------------
        case PHASE_WAITING_FOR_BRAKE:
        // Throttle control active — waiting for driver to initiate braking
        // ----------------------------------------------------------------
            ThrottleControl(speed_kmh);

            // Transition to braking phase on first regen brake input
            if (od_regen_brake > REGEN_BRAKE_THRESHOLD) {
                energy_brake_start = od_recovered_energy;
                phase              = PHASE_BRAKING;
                DualSerial.print("[EnergyRecovery] Braking started — energy snapshot: ");
                DualSerial.println(energy_brake_start);
            }
            break;

        // ----------------------------------------------------------------
        case PHASE_BRAKING:
        // Regen braking active — suppress service brake to maximise recovery
        // ----------------------------------------------------------------
            motor_dac        = 0;
            brake_dac        = od_regen_brake;
           //od_service_brake_mc = 0;  // Suppress — keep energy in regen circuit

            WriteDAC(THROTTLE_CHANNEL, uint16_t(motor_dac * over_speed_damping));
            WriteDAC(REGEN_CHANNEL, brake_dac);
            if(od_regen_brake > 0){
                digitalWrite(BRAKE_SWITCH, HIGH);
            } else {
                digitalWrite(BRAKE_SWITCH, LOW);
            }
 


            // Transition to idle once train is stopped
            if (speed_kmh <= 0.1f) {
                energy_brake_end  = od_recovered_energy;
                energy_recovered  = energy_brake_end - energy_brake_start;
               //od_service_brake_mc = 1;
                WriteDAC(THROTTLE_CHANNEL, uint16_t(motor_dac * over_speed_damping));
                WriteDAC(REGEN_CHANNEL, 0);
                digitalWrite(BRAKE_SWITCH, LOW);
                phase = PHASE_STOPPED_IDLE;

                DualSerial.print("[EnergyRecovery] Stopped — energy recovered: ");
                DualSerial.print(energy_recovered);
                DualSerial.println(" (waiting for throttle)");
            }

            DualSerial.print("[EnergyRecovery] Braking — Brake DAC: "); DualSerial.println(brake_dac);
            break;

        // ----------------------------------------------------------------
        case PHASE_STOPPED_IDLE:
        // Stationary — holding service brake, waiting for first throttle input
        // ----------------------------------------------------------------
            WriteDAC(THROTTLE_CHANNEL, uint16_t(motor_dac * over_speed_damping));
            WriteDAC(REGEN_CHANNEL, 0);
            digitalWrite(BRAKE_SWITCH, LOW);
           //od_service_brake_mc = 1;

            if (od_motor_command > 10) {
                // First throttle received — snapshot energy at this moment
                energy_motor_start = od_recovered_energy;
               //od_service_brake_mc  = 0;
                phase              = PHASE_MOTORING;
                DualSerial.print("[EnergyRecovery] Throttle received — motor energy start: ");
                DualSerial.println(energy_motor_start);
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
                WriteDAC(THROTTLE_CHANNEL, uint16_t(motor_dac * over_speed_damping));
                WriteDAC(REGEN_CHANNEL, 0);
                digitalWrite(BRAKE_SWITCH, LOW);
               //od_service_brake_mc = 0;
                phase            = PHASE_BUDGET_EXHAUSTED;
                DualSerial.println("[EnergyRecovery] Energy budget exhausted — throttle cut.");
                break;
            }

            // Budget remaining — raw throttle pass-through
           //od_service_brake_mc = 0;
            ThrottleControl(speed_kmh);

            DualSerial.print("[EnergyRecovery] Used: "); DualSerial.print(energy_used);
            DualSerial.print(" / ");                     DualSerial.print(energy_recovered);
            DualSerial.print(" | Motor DAC: ");          DualSerial.print(motor_dac);
            DualSerial.print(" | Brake DAC: ");          DualSerial.println(brake_dac);
            break;
        }

        // ----------------------------------------------------------------
        case PHASE_BUDGET_EXHAUSTED:
        // Throttle permanently cut — challenge complete
        // ----------------------------------------------------------------
            digitalWrite(ISOLATING_RELAY, HIGH);
            WriteDAC(THROTTLE_CHANNEL, uint16_t(motor_dac * over_speed_damping));
            WriteDAC(REGEN_CHANNEL, 0);
            digitalWrite(BRAKE_SWITCH, LOW);
           //od_service_brake_mc = 0;
            integrator       = 0.0f;

            // Reset if challenge mode is switched away and back
            // — handled by entered flag resetting on mode change if needed
            DualSerial.println("[EnergyRecovery] Challenge complete — budget exhausted.");
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
 * @brief Sets od_service_brake_mc variable as brakes engaged or unengaged
 */
void SetServiceBrake(bool engaged) {
    od_service_brake_mc = engaged ? 0 : 1;   // 0 = braking, 1 = not braking
    uint8_t value = od_service_brake_mc;
    executeSDOWrite(NODE_ID, 2, 0x3012, 0x01, sizeof(value), &value);
    // ^ check your CAN_MREx signature — adjust args to match
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