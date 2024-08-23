# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

### Changed

### Removed

## [1.0.4] - 2024-08-23

### Fixed

- Problem when entering service mode the log stopped working

## [1.0.3] - 2024-08-23

### Added
- Changelog

### Changed

- Unified device I2C frequency under single compile-time variable

### Fixed

- Fixed bug with T/RH sensor not properly measuring
- Fixed bug with sensor 4x not properly being initialized into continuous measuring state
- Fixed configuration verification to keep memory-based variables when configuration mismatch detected

## [1.0.2] - 2024-08-22

### Fixed

- Switched nibbles for power vector bytes while in service communication

## [1.0.1] - 2024-08-21

### Added

- Added auxiliary message sending option

### Fixed

- Bug with sending two messages to the server sometimes sending only one message

## [1.0.0] - 2024-08-21

### Added

- Basic code structure
- Added UART service communication support
- Added configuration from EEPROM parsing
- Added version numbering
- Unified all ABC period units to hours
- Added option to boot in UART service mode

### Changed

- All configurations are now stored in the EEPROM

### Removed

- General configuration file with definitions