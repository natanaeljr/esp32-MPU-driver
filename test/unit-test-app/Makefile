#
# Project Makefile.
#

PROJECT_NAME := unit-test-app

# ########### NOTE ###############
# Must add here the I2Cbus / SPIbus library path for the TESTS TO WORK
EXTRA_COMPONENT_DIRS += ${HOME}/esp/libraries/I2Cbus
EXTRA_COMPONENT_DIRS += ${HOME}/esp/libraries/SPIbus


MPU_COMPONENT_PATH := $(abspath ../..)
# Add MPU library path
EXTRA_COMPONENT_DIRS += $(MPU_COMPONENT_PATH)
# Add mpu-tests directory as component
EXTRA_COMPONENT_DIRS += $(MPU_COMPONENT_PATH)/test/mpu-tests
# Add ESP-IDF unity-test-app path
EXTRA_COMPONENT_DIRS += $(IDF_PATH)/tools/unit-test-app/components

# Enable MPU testing
# Note: should be mpu-tests directory
override TEST_COMPONENTS += $(notdir $(MPU_COMPONENT_PATH)/test/mpu-tests)

# Follow the same sdkconfig.defaults from esp-idf unit-test-app
SDKCONFIG_DEFAULTS := $(IDF_PATH)/tools/unit-test-app/sdkconfig.defaults

# ESP-IDF project makefile
include $(IDF_PATH)/make/project.mk

# Target to copy the partition table file from esp-idf/tools/unit-test-app
# Note: must be defined after the "include $(IDF_PATH)/make/project.mk", so it's not the first target
$(PROJECT_PATH)/partition_table_unit_test_app.csv: $(IDF_PATH)/tools/unit-test-app/partition_table_unit_test_app.csv
	@cp -u $(IDF_PATH)/tools/unit-test-app/partition_table_unit_test_app.csv ./
