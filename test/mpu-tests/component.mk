#
# Component makefile.
#

COMPONENT_ADD_LDFLAGS = -Wl,--whole-archive -l$(COMPONENT_NAME) -Wl,--no-whole-archive

## Compile DMP Test file only if DMP enabled in menuconfig
$(call compile_only_if,$(CONFIG_MPU_ENABLE_DMP),test_dmp.o)
