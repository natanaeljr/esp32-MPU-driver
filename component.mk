#
# Main component makefile.
#
# This Makefile can be left empty. By default, it will take the sources in the 
# src/ directory, compile them and link them into lib(subdirectory_name).a 
# in the build directory. This behaviour is entirely configurable,
# please read the ESP-IDF documents if you need to do this.
#

COMPONENT_SRCDIRS := src .

MPU_COMPONENT_NAME := $(COMPONENT_NAME)
export MPU_COMPONENT_NAME

# check definition of CONFIG_MPU_BUS before
ifeq ("$(findstring I2Cbus, $(COMPONENTS))","")
$(error $(COMPONENT_NAME) component dependency error: I2Cbus component not found. \
Make sure the library is included in your components directory or added to EXTRA_COMPONENTS_DIRS. \
See $(COMPONENT_PATH)/README.md for more information.)
endif