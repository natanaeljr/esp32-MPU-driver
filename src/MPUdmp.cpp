// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file MPUdmp.cpp
 * Implement MPU class with DMP interface.
 */

#include "MPUdmp.hpp"
#include <stdint.h>
#include <string.h>
#include "MPU.hpp"
#include "dmp/image.hpp"
#include "mpu/registers.hpp"
#include "mpu/types.hpp"

static const char* TAG = CONFIG_MPU_CHIP_MODEL;

#include "mpu/log.hpp"

#define min(a, b) (a < b ? a : b)
#define max(a, b) (a > b ? a : b)

/*! MPU Driver namespace */
namespace mpud
{
/*! DMP namespace */
namespace dmp
{
/**
 * @brief Load DMP firmware into DMP Memory.
 * @note DMP Memory is vollatile, so it has to be reload every power-up.
 */
esp_err_t MPU::loadDMP()
{
    uint16_t addr = 0;  // chunk start address
    while (addr < kDMPCodeSize) {
        // watch out for the last chunk, might be smaller
        const uint16_t codeLeft = kDMPCodeSize - addr;
        const uint8_t length    = min(codeLeft, kMemoryChunkSize);
        // write this chunk
        if (MPU_ERR_CHECK(writeMemory(addr, length, &kDMPMemory[addr]))) return err;
        // read chunk back to verify
        buffer[kMemoryChunkSize] = {0};
        if (MPU_ERR_CHECK(readMemory(addr, length, buffer))) return err;
        // compare read chunk against this DMP code chunk
        if (memcmp(&kDMPMemory[addr], buffer, length)) {
            MPU_LOGEMSG(msgs::DMP_LOAD_FAIL, " @ bank: %d, address: 0x%X, length: %d",  //
                        (addr >> 8), (addr & 0xFF), length);
            return err = ESP_FAIL;
        }
        MPU_LOGVMSG("done", " chunk: %d, length: %d, left: %d", addr, length, codeLeft - length);
        addr += length;
    }
    MPU_LOGI("DMP firmware loaded");
    return err = ESP_OK;
}

/**
 * @brief Write to the DMP memory.
 * This function prevents I2C writes past the bank boundaries. \n
 * The DMP memory is only accessible when the chip is awake.
 *
 * @param memAddr Memory location (bank << 8 | start_address)
 * @param length  Number of bytes to write.
 * @param data    Bytes to write to memory.
 */
esp_err_t MPU::writeMemory(uint16_t memAddr, uint8_t length, const uint8_t* data)
{
    buffer[0] = memAddr >> 8;
    buffer[1] = memAddr & 0xFF;
    // check bank boundaries
    if (int endAddr = (buffer[1] + length) > kMemoryBankSize) {
        MPU_LOGEMSG(msgs::BANK_BOUNDARIES, "mem_addr: 0x%X, length: %d, excess: %d",  //
                    memAddr, length, endAddr - kMemoryBankSize);
        return err = ESP_ERR_INVALID_SIZE;
    }
    // set memory bank & start address
    if (MPU_ERR_CHECK(writeBytes(regs::BANK_SEL, 2, buffer))) return err;
    // write data to memory
    return MPU_ERR_CHECK(writeBytes(regs::MEM_R_W, length, data));
}

/**
 * @brief Read from the DMP memory.
 * This function prevents I2C writes past the bank boundaries. \n
 * The DMP memory is only accessible when the chip is awake.
 *
 * @param memAddr Memory location (bank << 8 | start_address)
 * @param length  Number of bytes to read.
 * @param data    Bytes to read from memory.
 */
esp_err_t MPU::readMemory(uint16_t memAddr, uint8_t length, uint8_t* data)
{
    buffer[0] = memAddr >> 8;
    buffer[1] = memAddr & 0xFF;
    // check bank boundaries
    if (int endAddr = (buffer[1] + length) > kMemoryBankSize) {
        MPU_LOGEMSG(msgs::BANK_BOUNDARIES, "mem_addr: 0x%X, length: %d, excess: %d",  //
                    memAddr, length, endAddr - kMemoryBankSize);
        return err = ESP_ERR_INVALID_SIZE;
    }
    // set memory bank & start address
    if (MPU_ERR_CHECK(writeBytes(regs::BANK_SEL, 2, buffer))) return err;
    // read data from memory
    return MPU_ERR_CHECK(readBytes(regs::MEM_R_W, length, data));
}

}  // namespace dmp

}  // namespace mpud
