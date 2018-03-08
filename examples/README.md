# Examples

To run this examples, do not forget to add `I2Cbus/SPIbus` library path in the Makefile of each example.

+ **MPU-I2C**:  
    Setup MPU through I2C for basic usage.

+ **MPU-SPI**:  
    Setup MPU through SPI for basic usage.

+ **MPU-Real**:  
  A more 'elaborated' example, shows how to:  
  \- Use either SPI or I2C in the same code  
  \- Use the MPU with interrupt signal  
  \- Read sensor data from FIFO  
  \- Perform Self-Test check  
  \- Calibrate sensor data output using offset registers  
  \- Calculate Tilt Angles
