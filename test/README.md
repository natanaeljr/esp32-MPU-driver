# MPU Unit Test

**Structure:**

+ `mpu-tests` contains the tests themselves.
+ `unit-test-app` is the project to flash and run the tests. Command: `make flash monitor`. Works the same as the default unit-test-app from _esp-idf_.

There is a test configuration menu \[MPU test config\] in menuconfig -> component config, to setup the connection pins, speed, etc.

**Note:**

Before trying to build the unit-test-app, add the I2Cbus/SPIbus library path to `EXTRA_COMPONENT_DIRS` in the unit-test-app/Makefile.

The mpu tests also work with the default unit-test-app from _esp-idf_. If you \'unit-test\' from there, do not forget to add the MPU component path as well as MPU/test/mpu-tests directory to `EXTRA_COMPONENT_DIRS` in the Makefile. (and the above dependencies).

See [Unit Testing in ESP32] for more information.

**Current tests:**

1. basic test
1. sample rate measurement
1. max sample rate test
1. low power accelerometer mode
1. interrupt configuration
1. basic auxiliary I2C configuration
1. slave 4 transfers
1. external frame synchronization (FSYNC pin)
1. sensor data test
1. standby mode
1. FIFO buffer
1. offset test
1. self-test check
1. motion detection and wake-on-motion mode
1. free-fall detection
1. zero-motion detection
1. compass configuration

---

[Unit Testing in ESP32]: https://esp-idf.readthedocs.io/en/latest/api-guides/unit-tests.html