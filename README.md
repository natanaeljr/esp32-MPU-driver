# MPU Driver

Note: This version has been replaced for a new better version on [master][NewDesign].

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

---

Copyright © 2017 Natanael Josue Rabello [_natanael.rabello@outlook.com_]

[NewDesign]: https://github.com/natanaeljr/MPU-esp32-driver/tree/master
