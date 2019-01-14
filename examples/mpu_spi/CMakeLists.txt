cmake_minimum_required(VERSION 3.5)

# Path to required components
get_filename_component(MPUDRIVER_PATH ../.. ABSOLUTE)
set(SPIBUS_PATH $ENV{HOME}/esp/libraries/SPIbus)

set(EXTRA_COMPONENT_DIRS
    ${MPUDRIVER_PATH}
    ${SPIBUS_PATH}
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mpu-spi)
