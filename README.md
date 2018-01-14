![MPU Driver][Banner]

Note: Under middle development.

This repository is a driver/library for the following _TDK Invensense_ MPU's parts:

+ **MPU6000** - Gyro/Accel - [_SPI_ & _I2C_]
+ **MPU6050** - Gyro/Accel - [_I2C_]
+ **MPU6500** - Gyro/Accel - [_SPI_ & _I2C_]
+ **MPU6555** - Gyro/Accel - [_SPI_ & _I2C_]
+ **MPU9150** - Gyro/Accel/Compass - [_I2C_]
+ **MPU9250** - Gyro/Accel/Compass - [_SPI_ & _I2C_]
+ **MPU9255** - Gyro/Accel/Compass - [_SPI_ & _I2C_]

Yes! Supports all these models and both SPI and I2C protocols.
The objective here is to make a very complete library for these awesome chips and unlock all of their features.

The library is written in C++ and designed for working with **ESP32 IoT Development Framework _(esp-idf)_**.
The library is based on Invensense's Embedded Motion Driver 5.1.3 and 6.12 and inherits a bit of style from Jeff Rowberg's MPU6050 library for Arduino.

## Features

##### Already implemented:

+ Support to SPI and I2C protocol
+ Basic configurations (sample rate _(4Hz~32KHz)_, clock source, full-scale, standby mode, offsets, interrupts, DLPF, etc..)
+ Burst reading for all sensors
+ Low Power Accelerometer mode _(various rates, e.g. 8.4μA at 0.98Hz)_
+ Low Power Wake-on-motion mode _(with motion detection interrupt)_
+ FIFO buffer access for all internal and external sensors
+ Complete Auxiliary I2C support for external sensors _(up to 4)_
+ External Frame Synchronization _(FSYNC)_ pass-through interrupt
+ Motion, Zero-motion and Free-Fall detection _(as motion detection interrupt)_
+ Total access to the Magnetometer _(even when MPU connected by SPI protocol)_
+ Calibration for Gyro and Accel
+ Self-Test _(true implementation from MotionApps)_

##### To be implemented:

+ DMP (Digital Motion Processor) interface

#### DMP Features

##### Future implementation:

+ Quaternion (3-axis Gyroscope)
+ Quaternion (6-axis Gyroscope and Accelerometer)
+ Screen Orientation (Android's screen rotation algorithm)
+ Tap Detection
+ Pedometer
+ Gyroscope calibrated data

## Instalation

You can download the repository, or clone it right into your project components directory.

```git
git clone https://github.com/natanaeljr/MPU-esp32-driver.git MPU
```

**Dependencies:** MPU driver depends on the following protocol libraries to communicate with the chip with ease. Download the one according to the protocol you'll use and place within your components directory as well.

+ Communication libs:  [I2Cbus] | [SPIbus]

_NOTE:_ Either one of these libraries must be installed as components in your project for the MPU library to work. It won't work otherwise.

## Usage

Example:

```C++
#include "I2Cbus.hpp"
#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"

using namespace emd;  // embedded motion driver, contains mpu
// ...
// initialize communication bus
i2c0.begin(SDA, SCL, CLOCK);
// MPU object
MPU_t MPU;
// setup and initialize
MPU.setBus(i2c0).setAddr(mpu::MPU_I2CADDRESS_AD0_LOW);
MPU.initialize();
// configure as needed
MPU.setSampleRate(250);  // Hz
MPU.setAccelFullScale(mpu::ACCEL_FS_4G);
MPU.setGyroFullScale(mpu::GYRO_FS_500DPS);
MPU.setDigitalLowPassFilter(mpu::DLPF_42HZ);  // smother data
MPU.setInterruptEnabled(mpu::INT_EN_RAWDATA_READY);  // enable INT pin
// read sensors data
mpu::raw_axes_t accelRaw;
mpu::raw_axes_t gyroRaw;
MPU.motion(&accelRaw, &gyroRaw);
printf("accel: %+d %+d %+d\n", accelRaw.x, accelRaw.y, accelRaw.z);
printf("gyro: %+d %+d %+d\n", gyroRaw[0], gyroRaw[1], gyroRaw[2]);
// convert data
mpu::float_axes_t accelG = mpu::accelGravity(accelRaw, mpu::ACCEL_FS_4G);
mpu::float_axes_t gyroDPS = mpu::gyroDecPerSec(gyroRaw, mpu::GYRO_FS_500DPS);
printf("accel: %+.2f %+.2f %+.2f\n", accelG[0], accelG[1], accelG[2]);
printf("gyro: %+.2f %+.2f %+.2f\n", gyroDPS.x, gyroDPS.y, gyroDPS.z);
// so on..
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

[Banner]: MPUdriver.jpg
[I2Cbus]: https://github.com/natanaeljr/I2Cbus-esp32
[SPIbus]: https://github.com/natanaeljr/SPIbus-esp32
