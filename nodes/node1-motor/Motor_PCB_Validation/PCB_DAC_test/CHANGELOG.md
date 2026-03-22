# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
- Created the first version of PCB_DAC_test