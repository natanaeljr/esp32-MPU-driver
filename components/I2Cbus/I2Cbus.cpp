#include "I2Cbus.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"


static const char* TAG = "I2Cbus";

/*******************************************************************************
 * INSTANCES
 ******************************************************************************/
I2Cbus I2Cbus0(I2C_NUM_0);
I2Cbus I2Cbus1(I2C_NUM_1);


/*******************************************************************************
 * SETUP
 ******************************************************************************/
I2Cbus::I2Cbus(i2c_port_t port) : port(port), ticksToWait(pdMS_TO_TICKS(TIMEOUT_DEFAULT)) {
}

esp_err_t I2Cbus::begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_speed) {
    return begin(sda_io_num, scl_io_num, GPIO_PULLUP_ENABLE, GPIO_PULLUP_ENABLE, clk_speed);
}

esp_err_t I2Cbus::begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num, gpio_pullup_t sda_pullup_en, gpio_pullup_t scl_pullup_en, uint32_t clk_speed) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_io_num;
    conf.sda_pullup_en = sda_pullup_en;
    conf.scl_io_num = scl_io_num;
    conf.scl_pullup_en = scl_pullup_en;
    conf.master.clk_speed = clk_speed;
    esp_err_t err = i2c_param_config(port, &conf);
    if (!err) err = i2c_driver_install(port, conf.mode, 0, 0, 0);
    return err;
}

esp_err_t I2Cbus::close() {
    return i2c_driver_delete(port);
}

void I2Cbus::setTimeout(uint32_t ms) {
    ticksToWait = pdMS_TO_TICKS(ms);
}



/*******************************************************************************
 * WRITING
 ******************************************************************************/
esp_err_t I2Cbus::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data, int32_t timeout) {
    uint8_t buffer;
    esp_err_t err = readByte(devAddr, regAddr, &buffer, timeout);
    if (err) return err;
    buffer = data ? (buffer | (1 << bitNum)) : (buffer & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, buffer, timeout);
}

esp_err_t I2Cbus::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data, int32_t timeout) {
    uint8_t buffer;
    esp_err_t err = readByte(devAddr, regAddr, &buffer, timeout);
    if (err) return err;
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1);
    data &= mask;
    buffer &= ~mask;
    buffer |= data;
    return writeByte(devAddr, regAddr, buffer, timeout);
}

esp_err_t I2Cbus::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data, int32_t timeout) {
    return writeBytes(devAddr, regAddr, 1, &data, timeout);
}

esp_err_t I2Cbus::writeBytes(uint8_t devAddr, uint8_t regAddr, size_t length, const uint8_t *data, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_ENABLE);
    i2c_master_write_byte(cmd, regAddr, ACK_CHECK_ENABLE);
    i2c_master_write(cmd, (uint8_t*) data, length, ACK_CHECK_ENABLE);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, (timeout < 0 ? ticksToWait : pdMS_TO_TICKS(timeout)));
    i2c_cmd_link_delete(cmd);
    #if defined I2C_LOG_READWRITES
        if (!err) { 
            char str[length*5+1]; 
            for(int i = 0; i < length; i++) 
                sprintf(str+i*5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
            I2C_LOG_READWRITES(TAG, "[slave:0x%X] Write %d bytes to register 0x%X, data: %s", devAddr, length, regAddr, str);
        }
    #endif
    #if defined I2C_LOG_ERRORS
        #ifdef I2C_LOG_READWRITES
            else {
        #else
            if (err) {
        #endif
        ESP_LOGE(TAG, "[slave:0x%X] Failed to write %d bytes to register 0x%X, error: 0x%X", devAddr, length, regAddr, err);
        }
    #endif
    return err;
}


/*******************************************************************************
 * READING
 ******************************************************************************/
esp_err_t I2Cbus::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, int32_t timeout) {
    return readBits(devAddr, regAddr, bitNum, 1, data, timeout);
}

esp_err_t I2Cbus::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, int32_t timeout) {
    uint8_t buffer;
    esp_err_t err = readByte(devAddr, regAddr, &buffer, timeout);
    if(!err) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        buffer &= mask;
        buffer >>= (bitStart - length + 1);
        *data = buffer;
    }
    return err;
}

esp_err_t I2Cbus::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, int32_t timeout) {
    return readBytes(devAddr, regAddr, 1, data, timeout);
}

esp_err_t I2Cbus::readBytes(uint8_t devAddr, uint8_t regAddr, size_t length, uint8_t *data, int32_t timeout) {
    if(length == 0) return ESP_ERR_INVALID_SIZE;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_ENABLE);
    i2c_master_write_byte(cmd, regAddr, ACK_CHECK_ENABLE);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_READ, ACK_CHECK_ENABLE);
    if(length > 1) i2c_master_read(cmd, data, (length - 1), ACK_LEVEL);
    i2c_master_read_byte(cmd, (data + length - 1), NACK_LEVEL);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, (timeout < 0 ? ticksToWait : pdMS_TO_TICKS(timeout)));
    i2c_cmd_link_delete(cmd);
    #if defined I2C_LOG_READWRITES
        if (!err) { 
            char str[length*5+1]; 
            for(int i = 0; i < length; i++) 
            sprintf(str+i*5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
            I2C_LOG_READWRITES(TAG, "[slave:0x%X] Read %d bytes from register 0x%X, data: %s", devAddr, length, regAddr, str);
        }
    #endif
    #if defined I2C_LOG_ERRORS
        #ifdef I2C_LOG_READWRITES
            else {
        #else
            if (err) {
        #endif
        ESP_LOGE(TAG, "[slave:0x%X] Failed to read %d bytes from register 0x%X, error: 0x%X", devAddr, length, regAddr, err);
        }
    #endif
    return err;
}


/*******************************************************************************
 * UTILS
 ******************************************************************************/
esp_err_t I2Cbus::testConnection(uint8_t devAddr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_ENABLE);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, ticksToWait);
    i2c_cmd_link_delete(cmd);
    #if defined I2C_ESP_LOG
        if (!err) I2C_ESP_LOG(TAG, "slave devAddr(0x%X) acknowledged transfer", devAddr);
    #endif
    return err;
}

void I2Cbus::scanner() {
    printf(LOG_COLOR_W "\n>> I2C scanning ..." LOG_RESET_COLOR "\n");
    uint8_t count = 0;
    for (size_t i = 0x3; i < 0x78; i++) {
        if(testConnection(i) == ESP_OK){
            printf(LOG_COLOR_W "- Device found at address 0x%X%s", i, LOG_RESET_COLOR "\n");
            count++;
        }
    }
    if(count == 0)
        printf(LOG_COLOR_E "- No I2C devices found!" LOG_RESET_COLOR "\n");
    printf("\n");
}













