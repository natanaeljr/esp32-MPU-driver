# MPU Driver

Note: Under middle development.

This repository is a driver/library for the following _TDK Invensense_ MPU's parts:

    + MPU6000 - Gyro/Accel (SPI and I2C)
    + MPU6050 - Gyro/Accel (I2C only)
    + MPU6500 - Gyro/Accel (SPI and I2C)
    + MPU6555 - Gyro/Accel (SPI and I2C)
    + MPU9150 - Gyro/Accel/Mag (I2C only)
    + MPU9250 - Gyro/Accel/Mag (SPI and I2C)
    + MPU9255 - Gyro/Accel/Mag (SPI and I2C)

Yes! Supports all these models and both SPI and I2C protocol (_=D_).
The objective here is to have a very complete library for these awesome chips and unlock all of their features.

The library is written in C++ and designed for working with **Espressif ESP32 IoT Development Framework _(esp-idf)_**.
The library is based on Invensense's Embedded Motion Driver 5.1.3 and 6.12 and inherits a bit of style from Jeff Rowberg's MPU6050 library for Arduino.

## Features

Already implemented:

+ Basic configurations (sample rate <4Hz~32KHz>, clock source, full-scale, standby mode, interrupts, DLPF, etc..)
+ Support to SPI and I2C protocol
+ Burst reading for all sensors
+ Low Power Accelerometer mode
+ FIFO buffer for all internal and external sensors
+ Complete Magnetometer interface (even when MPU connected by SPI protocol) [MPU9150 and MPU9250]
+ Complete Auxiliary I2C support for external sensors (up to 4)
+ External Frame Synchronization (FSYNC) as pass-through interrupt
+ ...

To be implemented:

+ Wake-on-motion interrupt for low power operation
+ Free-Fall and Zero-motion detection [MPU6000, MPU6050 and MPU9150]
+ Self-Calibration
+ Self-Test
+ Interface with DMP (Digital Motion Processor)
+ ...

### DMP Features

(in phase of implementation)

+ Low Power Quaternion (3-axis Gyroscope)
+ Low Power Quaternion (6-axis Gyroscope and Accelerometer)
+ Screen Orientation (Android's screen rotation algorithm)
+ Pedometer (Invensense implementation)
+ Tap detection
+ ...

## Instalation

You can clone it directly into your project components directory or in a specific library path.

```git

git clone https://github.com/natanaeljr/MPU-esp32-driver.git MPU

```

### Dependencies

There is a C++ style I2C/SPI library which the MPU driver depends on to communicate with the chip a lot more easily. Either one (according to the protocol you'll use) of these libraries must be installed as components in your project as well.

+ [I2Cbus]
+ [SPIbus]

NOTE: It won't work if neither of these libraries are provided as the components.

## Usage

TODO..  
Example:

```C++
#include "MPU.hpp"
#include "I2Cbus.hpp"

// initialize the bus
i2c0.begin(SDA, SCL, CLOCK);

// MPU object
emd::mpu::MPU_t MPU;
// set bus, address and initialize
MPU.setBus(i2c0);
MPU.setAddr(emd::mpu::MPU_I2CADDRESS_AD0_LOW);
MPU.initialize();
// configure
MPU.setSampleRate(250);
MPU.setGyroFullScaleRange(emd::mpu::GYRO_FSR_500DPS);
MPU.setAccelFullScaleRange(emd::mpu::ACCEL_FSR_4G);
MPU.setDigitalLowPassFilter(emd::mpu::DLPF_42HZ);
MPU.setInterruptEnabled(emd::mpu::INT_EN_RAWDATA_READY);
// start using
emd::mpu::raw_axes_t accel, gyro;
MPU.motion(&accel, &gyro);

// ...
```

### Menuconfig

TODO..

## Struture

TODO..

## Tests

TODO..
Almost all methods and features already implemented have been unit-tested.

---

Copyright © 2017 Natanael Josue Rabello [_natanael.rabello@outlook.com_]

[I2Cbus]: https://github.com/natanaeljr/I2Cbus-esp32
[SPIbus]: https://github.com/natanaeljr/SPIbus-esp32
