#!/bin/bash
# Script for Setting up External Libraries
echo 'Setting up External Libraries...'

# Exit with nonzero exit code if anything fails
set -e

# Download libs to esp/libraries
cd "$TRAVIS_BUILD_DIR"
mkdir -p esp/libraries
cd esp/libraries

echo "Setting up I2Cbus..."
if [ -d I2Cbus ]; then
    (
    cd I2Cbus
    git pull
    cd ..
    )
else
    git clone --recursive https://github.com/natanaeljr/esp32-I2Cbus I2Cbus
fi

echo "Setting up SPIbus.."
if [ -d SPIbus ]; then
    (
    cd SPIbus
    git pull
    cd ..
    )
else
    git clone --recursive https://github.com/natanaeljr/esp32-SPIbus SPIbus
fi

# Debug
# echo ''
# echo 'Libraries:' && ls

echo ''
echo 'Done.'