#!/bin/bash
# Script for Building Unit-Tests and Examples

set -e # exit with nonzero exit code if anything fails

# Variables
directory=
chips=
protocols=
valid_chips="MPU6000 MPU6050 MPU6500 MPU6555 MPU9150 MPU9250 MPU9255"
valid_protocols="I2C SPI"

# Functions

abort() {
	[ "$1" ] && echo -e "Error: $1" >&2
	exit 1
}

print_help() {
	echo "Script for Building Unit-Tests and Examples."
	echo "Usage: sh build_unit-test.sh <directory> [--chips=a,b..] [--protocols=a,b]"
	echo "- Communication protocols = ${valid_protocols// /,}"
	echo "- Chip models = ${valid_chips// /,}"
}

check_flag_value() {
	if [ "$2" == "" ] || [ "${2:0:1}" == "-" ]; then
		abort "Missing input in flag '$1'."
	fi
}

parse_directory() {
	if [ "$directory" ]; then abort "Multiple directories input."; fi
	if ! [ -d "$1" ]; then
		abort "Directory '$1' does not exist."
	else
		directory="$1"
	fi
}

parse_chips() {
	if [ "$1" == "all" ]; then
		chips="$valid_chips"
		return
	fi
	temp_chips="${1//,/ }"
	for chip in $temp_chips; do
		check=0
		for vc in $valid_chips; do
			if [ "$chip" == "$vc" ]; then
				check=1
				break
			fi
		done
		if [ $check == 0 ]; then abort "Invalid chip model '$chip'"; fi
		for this in $chips; do
			if [ "$this" == "$chip" ]; then abort "Chip '$chip' is duplicate"; fi
		done
		chips="$chips$chip "
	done
}

parse_protocols() {
	if [ "$1" == "all" ]; then
		protocols="$valid_protocols"
		return
	fi
	temp_protocols="${1//,/ }"
	for pro in $temp_protocols; do
		if [ "$pro" != "I2C" ] && [ "$pro" != "SPI" ]; then abort "Invalid protocol '$pro'"; fi
		for this in $protocols; do
			if [ "$this" == "$pro" ]; then abort "Protocol '$pro' is duplicate"; fi
		done
		protocols="$protocols$pro "
	done
}

parse_arguments() {
	while [ "$1" != "" ]; do
		case "$1" in
		"-h" | "--help")
			print_help
			exit
			;;
		"--chips="*)
			check_flag_value "${1:0:8}" "${1:8}"
			parse_chips "${1:8}"
			;;
		"--protocols="*)
			check_flag_value "${1:0:12}" "${1:12}"
			parse_protocols "${1:12}"
			;;
		*)
			if [ "${1:0:1}" == "-" ]; then abort "Unknown option '$1'."; fi
			parse_directory "$1"
			;;
		esac
		shift
	done
}

validate_config() {
	if [ -z "$directory" ]; then abort "Missing directory input"; fi
	if [ -z "$chips" ]; then abort "Missing chip models input"; fi
	if [ -z "$protocols" ]; then abort "Missing protocols input"; fi
}

print_config() {
	echo "Directory: $directory"
	echo "Chip models: $chips"
	echo "Comm Protocols: $protocols"
}

build() {
	chip="$1"
	pro="$2"
	echo "################################################################################"
	echo ">>> Building $directory: chip '$chip' and protocol '$pro'"
	echo "################################################################################"
	prev_pwd=$(pwd)
	cd "$directory"
	echo "Adjusting Makefile..."
	sed -i "s/\${HOME}/\${TRAVIS_BUILD_DIR}/g" Makefile
	if grep -q "MPU_COMPONENT_PATH" Makefile; then
		sed -i "s/MPU_COMPONENT_PATH :=.*\$/MPU_COMPONENT_PATH := \${TRAVIS_BUILD_DIR}\\n/" Makefile
	fi
	if ! grep -q "EXTRA_COMPONENT_DIRS+=\${TRAVIS_BUILD_DIR}" Makefile; then
		sed -i "1s/^/EXTRA_COMPONENT_DIRS+=\${TRAVIS_BUILD_DIR}\\n/" Makefile
	fi
	# if ! grep -q "EXTRA_COMPONENT_DIRS+=\${HOME}/esp/libraries/MPUdriver" Makefile; then
	# 	sed -i "1s/^/EXTRA_COMPONENT_DIRS+=\${HOME}\\/esp\\/libraries\\/MPUdriver\\n/" Makefile
	# fi
	echo "Creating default 'sdkconfig'..."
	make defconfig >>build.log.txt 2>&1
	# sed -i "s/\"python\"/\"python2\"/" sdkconfig
	echo "Changing chip model to $chip..."
	sed -i "s/CONFIG_MPU6050=y/CONFIG_MPU6050=/" sdkconfig
	sed -i "s/CONFIG_$chip=/CONFIG_$chip=y/" sdkconfig
	echo "Changing comm protocol to $pro..."
	sed -i "s/CONFIG_MPU_I2C=/CONFIG_MPU_$pro=/" sdkconfig
	echo "Running 'defconfig' to adjust to changes..."
	make defconfig >>build.log.txt 2>&1
	echo "Final 'skdconfig':"
	grep "CONFIG_MPU" sdkconfig
	echo "Building..."
	make all -j3 1>>build.log.txt
	echo "Success."
	echo "Cleaning..."
	rm -f sdkconfig sdkconfig.old
	cd "$prev_pwd"
	echo "Done."
}

build_all() {
	for chip in $chips; do
		for pro in $protocols; do
			if ([ "$chip" == "MPU6050" ] || [ "$chip" == "MPU9150" ]) && [ "$pro" == "SPI" ]; then continue; fi
			build "$chip" "$pro"
		done
	done
	echo ">>> Built All successfully !!!"
}

# Main
parse_arguments "$@"
validate_config
print_config
build_all
