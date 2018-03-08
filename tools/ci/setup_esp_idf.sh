#!/bin/bash
# Script for Seting up ESP-IDF

# Exit with nonzero exit code if anything fails
set -e

# Create dir to work
cd "$TRAVIS_BUILD_DIR"
mkdir -p esp
cd esp

echo "Setting up Toolchain..."
if [ -d xtensa-esp32-elf ]; then
    echo "Already up to date."
else
    echo 'Downloading Toolchain...'
    wget -c "https://dl.espressif.com/dl/xtensa-esp32-elf-linux64-1.22.0-80-g6c4433a-5.2.0.tar.gz" --tries=20 --no-verbose
    echo 'Uncompressing Toolchain...'
    tar -xzf xtensa-esp32-elf-linux64-1.22.0-80-g6c4433a-5.2.0.tar.gz
    if [ -d xtensa-esp32-elf ] && [ -f xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc ]; then
        echo 'Toolchain successfully setup!'
    else
        echo 'Failed to setup Toolchain!'
        exit 1
    fi
fi

echo "Setting up ESP-IDF..."
if [ -d esp-idf ]; then
    (
    cd esp-idf
    git pull
    git submodule update --recursive
    cd ..
    )
else
    git clone --recursive --depth=50 https://github.com/espressif/esp-idf.git
fi

# Debug
# echo ''
# echo 'Current Directory' && pwd
# echo 'Directory Contents' && ls
# echo 'IDF_PATH is: ' $IDF_PATH

echo ''
echo 'Done.'