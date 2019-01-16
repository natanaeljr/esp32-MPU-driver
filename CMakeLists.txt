set(COMPONENT_SRCS
    "src/MPU.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENT_REQUIRES "I2Cbus")

register_component()

component_compile_options(PUBLIC -DMPU_COMPONENT_TRUE=1)

## Defines needed for library code implementation
# MPU9250 is the same as MPU6500 + AK8963
# MPU9150 is the same as MPU6050 + AK8975
# MPU6000 code is the same as MPU6050 code
# MPU6555 equals MPU6500, MPU9255 equals MPU9250
if(${CONFIG_MPU_CHIP_MODEL} STREQUAL "MPU9250")
    component_compile_options(PUBLIC -DCONFIG_MPU6500)
elseif (${CONFIG_MPU_CHIP_MODEL} STREQUAL "MPU9150")
    component_compile_options(PUBLIC -DCONFIG_MPU6050)
elseif (${CONFIG_MPU_CHIP_MODEL} STREQUAL "MPU6000")
    component_compile_options(PUBLIC -DCONFIG_MPU6050)
elseif (${CONFIG_MPU_CHIP_MODEL} STREQUAL "MPU6555")
    component_compile_options(PUBLIC -DCONFIG_MPU6500)
elseif (${CONFIG_MPU_CHIP_MODEL} STREQUAL "MPU9255")
    component_compile_options(PUBLIC -DCONFIG_MPU9250 -DCONFIG_MPU6500)
endif()
