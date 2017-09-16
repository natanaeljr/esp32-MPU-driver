#include "i2cbus.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"


static const char* TAG = "I2Cbus";

/*******************************************************************************
 * INSTANCES
 ******************************************************************************/
i2cbus_t i2c0(I2C_NUM_0);
i2cbus_t i2c1(I2C_NUM_1);


/*******************************************************************************
 * SETUP
 ******************************************************************************/
esp_err_t i2cbus_t::begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_speed) {
    return begin(sda_io_num, scl_io_num, GPIO_PULLUP_ENABLE, GPIO_PULLUP_ENABLE, clk_speed);
}

esp_err_t i2cbus_t::begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num, gpio_pullup_t sda_pullup_en, gpio_pullup_t scl_pullup_en, uint32_t clk_speed) {
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

void i2cbus_t::set_timeout(uint32_t ms) {
    ticks_to_wait = pdMS_TO_TICKS(ms);
}

esp_err_t i2cbus_t::close() {
    return i2c_driver_delete(port);
}


/*******************************************************************************
 * WRITING
 ******************************************************************************/
esp_err_t i2cbus_t::write_bit(uint8_t address, uint8_t _register, uint8_t bit, uint8_t data) {
    uint8_t buffer;
    esp_err_t err = read_byte(address, _register, &buffer);
    if (err) return err;
    buffer = data ? (buffer | (1 << bit)) : (buffer & ~(1 << bit));
    return write_byte(address, _register, buffer);
}

esp_err_t i2cbus_t::write_bits(uint8_t address, uint8_t _register, uint8_t bitstart, uint8_t length, uint8_t data) {
    uint8_t buffer;
    esp_err_t err = read_byte(address, _register, &buffer);
    if (err) return err;
    uint8_t mask = ((1 << length) - 1) << (bitstart - length + 1);
    data <<= (bitstart - length + 1);
    data &= mask;
    buffer &= ~mask;
    buffer |= data;
    return write_byte(address, _register, buffer);
}

esp_err_t i2cbus_t::write_byte(uint8_t address, uint8_t _register, uint8_t data) {
    return write_bytes(address, _register, 1, &data);
}

esp_err_t i2cbus_t::write_bytes(uint8_t address, uint8_t _register, uint8_t length, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, ACK_CHECK_ENABLE);
    i2c_master_write_byte(cmd, _register, ACK_CHECK_ENABLE);
    i2c_master_write(cmd, data, length, ACK_CHECK_ENABLE);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, ticks_to_wait);
    i2c_cmd_link_delete(cmd);
    #if defined I2C_ESP_LOG
        if (!err) I2C_ESP_LOG(TAG, "written %d bytes to reg(0x%X) at slave(0x%X)", length, _register, address);
    #endif
    return err;
}


/*******************************************************************************
 * READING
 ******************************************************************************/
esp_err_t i2cbus_t::read_bit(uint8_t address, uint8_t _register, uint8_t bit, uint8_t *data) {
    return read_bits(address, _register, bit, 1, data);
}

esp_err_t i2cbus_t::read_bits(uint8_t address, uint8_t _register, uint8_t bitstart, uint8_t length, uint8_t *data) {
    uint8_t buffer;
    esp_err_t err = read_byte(address, _register, &buffer);
    if(!err) {
        uint8_t mask = ((1 << length) - 1) << (bitstart - length + 1);
        buffer &= mask;
        buffer >>= (bitstart - length + 1);
        *data = buffer;
    }
    return err;
}

esp_err_t i2cbus_t::read_byte(uint8_t address, uint8_t _register, uint8_t *data) {
    return read_bytes(address, _register, 1, data);
}

esp_err_t i2cbus_t::read_bytes(uint8_t address, uint8_t _register, uint8_t length, uint8_t *data) {
    if(length == 0) return ESP_ERR_INVALID_SIZE;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, ACK_CHECK_ENABLE);
    i2c_master_write_byte(cmd, _register, ACK_CHECK_ENABLE);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, ACK_CHECK_ENABLE);
    if(length > 1) i2c_master_read(cmd, data, (length - 1), ACK_LEVEL);
    i2c_master_read_byte(cmd, (data + length - 1), NACK_LEVEL);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, ticks_to_wait);
    i2c_cmd_link_delete(cmd);
    #if defined I2C_ESP_LOG
        if (!err) I2C_ESP_LOG(TAG, "read %d bytes from reg(0x%X) at slave(0x%X)", length, _register, address);
    #endif
    return err;
}


/*******************************************************************************
 * UTILS
 ******************************************************************************/
esp_err_t i2cbus_t::test(uint8_t address) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, ACK_CHECK_ENABLE);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, ticks_to_wait);
    i2c_cmd_link_delete(cmd);
    #if defined I2C_ESP_LOG
        if (!err) I2C_ESP_LOG(TAG, "slave address(0x%X) acknowledged transfer", address);
    #endif
    return err;
}

void i2cbus_t::scanner() {
    printf(LOG_COLOR_W "\n>> I2C scanning ..." LOG_RESET_COLOR "\n");
    uint8_t count = 0;
    for (size_t i = 0x3; i < 0x78; i++) {
        if(test(i) == ESP_OK){
            printf(LOG_COLOR_W "- Device found at address 0x%X%s", i, LOG_RESET_COLOR "\n");
            count++;
        }
    }
    if(count == 0)
        printf(LOG_COLOR_E "- No I2C devices found!" LOG_RESET_COLOR "\n");
    printf("\n");
}









// end
