cmake_minimum_required(VERSION 3.5)

# Path to required components
get_filename_component(MPUDRIVER_PATH ../.. ABSOLUTE)
set(I2CBUS_PATH $ENV{HOME}/esp/libraries/I2Cbus)

set(EXTRA_COMPONENT_DIRS
    ${MPUDRIVER_PATH}
    ${I2CBUS_PATH}
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mpu-i2c)
