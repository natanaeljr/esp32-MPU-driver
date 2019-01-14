set(COMPONENT_ADD_INCLUDEDIRS)
set(COMPONENT_SRCS "mpu_spi.cpp")

set(COMPONENT_REQUIRES)
set(COMPONENT_PRIV_REQUIRES
    driver
    MPUdriver
    SPIbus
)

register_component()
