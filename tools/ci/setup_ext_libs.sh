# Setup up External Libraries
echo 'Setting up External Libraries...'

# Exit with nonzero exit code if anything fails
set -e

# Download libs to esp/libraries
cd $TRAVIS_BUILD_DIR
mkdir -p esp/libraries
cd esp/libraries

# I2Cbus
git clone --recursive https://github.com/natanaeljr/esp32-I2Cbus I2Cbus

# SPIbus
git clone --recursive https://github.com/natanaeljr/esp32-SPIbus SPIbus


# debug
echo ''
echo 'Libraries:' && ls

echo ''
echo 'Done.'