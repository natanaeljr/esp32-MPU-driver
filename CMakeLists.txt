set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENT_SRCS
    src/MPU.cpp
    src/MPUdmp.cpp
    src/math.cpp
)

# Change here to the corresponding protocol component name you are using
set(COMPONENT_REQUIRES
    I2Cbus
    SPIbus
)

# Register
register_component()

# Export MPUdriver flag
add_definitions(-DMPU_COMPONENT_TRUE=1)

## Defines needed for library code implementation
# MPU9250 is the same as MPU6500 + AK8963
# MPU9150 is the same as MPU6050 + AK8975
# MPU6000 code is the same as MPU6050 code
# MPU6555 equals MPU6500, MPU9255 equals MPU9250
if(CONFIG_MPU_CHIP_MODEL STREQUAL "MPU9250")
    add_definitions(-DCONFIG_MPU6500)
elseif(CONFIG_MPU_CHIP_MODEL STREQUAL "MPU9150")
    add_definitions(-DCONFIG_MPU6050)
elseif(CONFIG_MPU_CHIP_MODEL STREQUAL "MPU6000")
    add_definitions(-DCONFIG_MPU6050)
elseif(CONFIG_MPU_CHIP_MODEL STREQUAL "MPU6555")
    add_definitions(-DCONFIG_MPU6500)
elseif(CONFIG_MPU_CHIP_MODEL STREQUAL "MPU9255")
    add_definitions(-DCONFIG_MPU9250 -DCONFIG_MPU6500)
endif()
