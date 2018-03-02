### Setup ESP-IDF
echo 'Setting up esp-idf...'

# Exit with nonzero exit code if anything fails
set -e

# create dir to work
cd $TRAVIS_BUILD_DIR
mkdir esp
cd esp

# Download Toolchain
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

# Clone esp-idf
git clone --recursive --depth=50 https://github.com/espressif/esp-idf.git

# Debug
echo ''
echo 'Current Directory' && pwd
echo 'Directory Contents' && ls
echo 'IDF_PATH is: ' $IDF_PATH

echo ''
echo 'Done.'