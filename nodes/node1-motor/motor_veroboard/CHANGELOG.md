# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [1.0.2] - 2026-03-29
### Changed 
- Changed filename
- moved pin definitions
- turned operating modes into state machine


### Added 
- Added motor_controller.h
- Added fucntions for operating modes
- Added header to motor_veroboard


## [1.0.1] - 2026-03-22
### Changed 
- Changed to suit CM 1.13.0

### Added
- Added motor lockout so the train doesnt jerk when both throttle and brake input are given and brake is taken away
- Added safety states to preop and stopped


## [1.0.0] - 2026-03-22
### Added 
- added first instance of MotorVeroboard.ino