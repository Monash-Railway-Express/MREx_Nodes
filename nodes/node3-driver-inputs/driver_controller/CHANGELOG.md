# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

unreleased
## [1.3.0] - 2026-05-14
### Changed
- Use DualSerial module for wireless serial monitoring

unreleased
## [1.2.2] - 2026-05-07

## Added
Doxygen style doc comments added to previously undocumented buffer and helper functions:
  - `updateADCBuffer()`
  - `getAverage()`
  - `readStable3PosBuffered()`
  - `readStable5PosBuffered()`
  - `InputTask()`
  - `initBuffer()`

### Fixed
- Compliance with Software Standards and Guidelines V1.0.1 for new function names
---
unreleased
## [1.2.1] - 2026-04-12

### Added 
- Doxygen style headings have been added to remaining functions
- HandleChallenge function has been created which will read form the challenge pin
  - If there is a change in the state of this pin there will be an SDO write to the Motor node. 


---
unreleased
## [1.2.0] - 2026-04-09
### Changed
- Replaced basic `analogRead()` with high-impedance ADC sampling (`readADC_HighZ`)
  - Includes settling time and multi-sampling
  - Improves performance for high-resistance (100kΩ) sources
- Added multi-sample validation for selector switches:
  - `readStable3Pos()`
  - `readStable5Pos()`
- Replaced threshold-based decoding with nearest-value matching:
  - `decodeNearest3()`
  - `decodeNearest5()`
- Updated `UpdateOpMode()`:
  - Now uses `readStable3Pos(OP_MODE_PIN)`
  - Cleaner enum mapping logic
  - Added serial debug output
- Fixed incorrect variable usage:
  - `nodeOperatingMode = opMode` → `nodeOperatingMode = enumOpMode`
-Behaviour changes:
  - PreOp Mode:
    - Handles direction and challenge selection
  - Operational Mode:
    - Handles challenge and inputs
- Refactored logic into dedicated functions:
  - `HandleDirection()`
  - `HandleChallenge()`
  - `HandleInputs()`
  - `HandleHorn()`
  - `HandleParking()`
- Removed reliance on previous state variables
- Now uses direct OD comparison for change detection


unreleased
## [1.1.0] - 2026-04-02
### Changed
- Updated file names in compliance with Software Standards and Guidelines V1.0.1 (SSG V1.0.1)
- Updating variable names to be in compliance with SSG V1.0.1 
- Refactored operating mode with enum-based mapping inside `UpdateOpMode()` for clearer state handling


- Updated main loop structure:
  - Replaced direct mode switching logic with structured `switch(mode)` statement
  - Introduced dedicated mode functions: `StoppedMode()`, `PreOpMode()`, `OperationalMode()`


### Added
- Mode handler functions:
  - `StoppedMode()`
  - `PreOpMode()`
  - `OperationalMode()`
- `UpdateOpMode()` function:
  - Handles validation of switch input (including invalid state `101`)
  - Maps 5-position switch input to enum-based operating modes
  - Sends NMT only on state change

### Removed
- Removed `SendToNewOpMode()` function
- Removed inline mode transition logic from `loop()`

unreleased
## [1.0.3] - 2026-03-31
### Changed 
- Updating file and function documentation in compliance with Software Standards and Guidelines V1.0.1 (SSG V1.0.1)
- Updating variable names to be in compliance with SSG V1.0.1 
- Adding function prototypes and pin definitions to header file in compliance with SSG V1.0.1
- Adding OperatingMode enum to header file, replacing 0x02,0x01,0x80 with MODE_STOPPED, MODE_OPERATIONAL,MODE_PREOP respectively
- in function SendAllNMT(), NODE ID's have been updated to use variable names to aid readability


## [1.0.2] - 2026-03-24
### Changed 
- Cleaned main loop for changing op mode
- Fixed reliablity in main loop with op mode input with timing non blocking function and deadbanding
- Added new parking brake SDO function call   

## [1.0.1] - 2026-03-
### Changed 
- Changed to suit CM 1.13.0

## [1.0.0] - 2026-03-22
### Added 
- added first instance of ControllerV2.ino