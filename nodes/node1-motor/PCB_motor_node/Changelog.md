# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.0] - 02-04-26 - Sean Larkin
### Changed
- Renamed all OD variables to comply with `od_` prefix and snake_case standards
- Renamed all global variables to snake_case
- Renamed all functions to UpperCamelCase
- Renamed all constants to UPPER_CASE and moved to header file
- Replaced LEDC PWM output with I2C DAC (WriteDAC)
- ReadSpeedKMH now takes prev_time argument for accurate elapsed-time speed calculation

### Added
- SpeedControl: PI closed-loop speed controller
- ThrottleControl: Raw motor command pass-through to DAC
- AutoStopChallenge: PI speed control with automatic stop below threshold
- TractionChallenge: Speed control with wheel slip detection and throttle cut
- EnergyRecoveryChallenge: Regen-first braking strategy suppressing service brake
- Challenge mode dispatch in OperationalMode via od_challenge_mode
- Hard speed cap at 15 km/h cutting throttle universally across all modes
- Function prototypes and all pin/constant definitions moved to motor_PCB.h

## [1.0.1] 14-03-26
### Changed
- Changed the common config to align with what is in the datasheet. 
void commonConfig() {
  Wire.beginTransmission(DAC_ADDR);
  Wire.write(0x1F);   // COMMON-CONFIG register
  Wire.write(0x02);   // EN-INT-REF=0, VOUT0 ON, IOUT0 OFF
  Wire.write(0x01);   // VOUT1 ON, IOUT1 OFF
  Wire.endTransmission();
}

## [1.0.0] 14-03-26
### Added
- Created the first version of PCB_motor_node