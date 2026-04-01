# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---
unreleased
## [1.0.3] - 2026-03-31
- Updating file and function documentation in compliance with Software Standards and Guidelines V1.0.1 (SSG V1.0.1)
- Updating variable names to be in compliance with SSG V1.0.1 
- Adding function prototypes and pin definitions to header file in compliance with SSG V1.0.1
- Adding OperatingMode enum to header file, replacing 0x02,0x01,0x80 with MODE_STOPPED, MODE_OPERATIONAL,MODE_PREOP respectively
- in function SendAllNMT(), NODE ID's have been updated to use variable names to aid readability

unreleased
## [1.0.2] - 2026-03-24
### Changed 
- Cleaned main loop for changing op mode
- Fixed reliablity in main loop with op mode input with timing non blocking function and deadbanding
- Added new parking brake SDO function call   

unreleased
## [1.0.1] - 2026-03-
### Changed 
- Changed to suit CM 1.13.0

## [1.0.0] - 2026-03-22
### Added 
- added first instance of ControllerV2.ino