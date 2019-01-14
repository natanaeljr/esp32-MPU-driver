set(COMPONENT_ADD_INCLUDEDIRS)
set(COMPONENT_SRCS "mpu_real.cpp")

set(COMPONENT_REQUIRES)
set(COMPONENT_PRIV_REQUIRES
    driver
    MPUdriver
    I2Cbus
    SPIbus
)

register_component()
