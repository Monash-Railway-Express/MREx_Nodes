# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1]  - 2026-03-26

### Added

- Lights self test. Lights will flash in order White->Yellow->Red->All on power on.

### Changed

- Updated to use CAN 1.13.0

### Removed
- None

## [1.2]  - 2026-04-16

### Added

- None

### Changed

- Pre-op (yellow) lights now stay on as long as power is supplied to ESP32. 
- Fixed pin numbering for CAN TX/RX

### Removed
- None


## [1.2.1]  - 2026-05-11

### Added
- lights_and_sensors.ino restored from previous dev commits

### Changed
- Lights.ino was changed to lights_and_sensors.ino to meet Software Standards
- Header file and Function documentation was updated to meet Software Standards
- Variables & Functions unrelated to CAN_MREx were updated to meet Software Standards

### Removed
- Lights.ino, now replaced by lights_and_sensors.ino


## [1.2.1]  - 2026-05-11

### Added
- lights_n_sensors.h as a header file for the node

### Changed
- Pin definitions, constants, enums all moved to the header file
