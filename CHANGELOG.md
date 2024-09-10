# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

### Changed

### Removed

### Fixed

## [1.1.0] - 2024-09-10

### Added

- Function to invert display color in a row
- Full display support
- Status section on the display
- Exception handlers for both core 0 and core 1
- Added variable for minimum sensor power up time
- Added delay for CDM7162 between status read and measured value read

### Changed

- TCP server polling interval is changed to 30 seconds to avoid excessive TCP socket opening
- SW version variables to FW version
- Cleaned CMakeLists.txt files
- SunRise & SunLight - removed condition for sw reset after configuration change - sensors will always reset on init
- Changed CozIR-LP3 get_value function to use only one read for data & status

### Removed

- Closing TCP client on poll

### Fixed

## [1.0.7] - 2024-08-28

### Added

- Option to retry sending message if previous attempt resulted in client error callback

### Changed

- Closing connection on tcp server/client poll
- Changed tcp server/client poll interval to 3 seconds
- Changed tcp client timing to match exact sending interval

### Removed

- Removed pressure checking in measurement loop for SCD41
- Removed option to close TCP client after data was sent

### Fixed

- SUNRISE and SUNLIGHT sensors meter control register not being set properly
- Removed option to set CO2 offset for Sensor 0 via configuration
- Added support for reinit on error toggle
- fixed SUNRISE and SUNLIGHT sensor reset only on writing to EEPROM
- fixed SUNRISE and SUNLIGHT sensor pressure configuration to be written after reset
- Fixed bug causing no data to database being sent

## [1.0.6] - 2024-08-23

### Added

- Configuration verification after sensor reinit

### Changed

- Timer after sensor power-up

### Fixed

- Fixed bug counting no measurement taken as an error
- Fixed bug with sensor not zeroing its iteration error counter causing the sensor to stop measurement

## [1.0.5] - 2024-08-23

### Changed

- Added extra delay for EEPROM reading

### Fixed

- Reverted power vector nibble switch
- Fixed bug with pressure sensor going to SUCCESS state while not connected
- Fixed reading loop taking too long causing service communication to pause

## [1.0.4] - 2024-08-23

### Fixed

- Problem when entering service mode using tester program the log stopped working

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