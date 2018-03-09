#
# Main component makefile.
#

COMPONENT_SRCDIRS := src .


## Compile DMP stuff only if enabled in menuconfig
$(call compile_only_if,$(CONFIG_MPU_ENABLE_DMP),src/MPUdmp.o)
