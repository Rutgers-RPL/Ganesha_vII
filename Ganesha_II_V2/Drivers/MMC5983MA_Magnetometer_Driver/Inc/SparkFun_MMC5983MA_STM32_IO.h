/*
 This is a modified library for the MMC5983MA High Performance Magnetometer, adapted for STM32 microcontrollers using the HAL library.
 The original library was written by Ricardo Ramos @ SparkFun Electronics, February 2nd, 2022.
 This file declares all functions used in the MMC5983MA High Performance Magnetometer STM32 Library I2C/SPI IO layer.
 */

#ifndef SPARKFUN_MMC5983MA_STM32_IO_H
#define SPARKFUN_MMC5983MA_STM32_IO_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define MMC5983MA_HAL_TIMEOUT_MS 100U

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *csPort;
    uint16_t           csPin;

    I2C_HandleTypeDef *hi2c;
    uint8_t            address;
    bool               useSPI;
} SFE_MMC5983MA_IO;

/* Initialise the struct to safe defaults */
void MMC5983MA_IO_init(SFE_MMC5983MA_IO *io);

/* Configures and starts the I2C I/O layer */
bool MMC5983MA_IO_beginI2C(SFE_MMC5983MA_IO *io, I2C_HandleTypeDef *hi2c);

/* Configures and starts the SPI I/O layer */
bool MMC5983MA_IO_beginSPI(SFE_MMC5983MA_IO *io, GPIO_TypeDef *csPort, uint16_t csPin, SPI_HandleTypeDef *hspi);

/* Returns true if we get the correct product ID from the device */
bool MMC5983MA_IO_isConnected(SFE_MMC5983MA_IO *io);

/* Read a single byte from a register */
bool MMC5983MA_IO_readSingleByte(SFE_MMC5983MA_IO *io, uint8_t registerAddress, uint8_t *buffer);

/* Writes a single byte into a register */
bool MMC5983MA_IO_writeSingleByte(SFE_MMC5983MA_IO *io, uint8_t registerAddress, uint8_t value);

/* Reads multiple bytes from a register into a buffer */
bool MMC5983MA_IO_readMultipleBytes(SFE_MMC5983MA_IO *io, uint8_t registerAddress, uint8_t *buffer, uint8_t packetLength);

/* Writes multiple bytes to a register from a buffer */
bool MMC5983MA_IO_writeMultipleBytes(SFE_MMC5983MA_IO *io, uint8_t registerAddress, uint8_t *buffer, uint8_t packetLength);

/* Sets a single bit in a specific register */
bool MMC5983MA_IO_setRegisterBit(SFE_MMC5983MA_IO *io, uint8_t registerAddress, uint8_t bitMask);

/* Clears a single bit in a specific register */
bool MMC5983MA_IO_clearRegisterBit(SFE_MMC5983MA_IO *io, uint8_t registerAddress, uint8_t bitMask);

/* Returns true if a specific bit is set in a register */
bool MMC5983MA_IO_isBitSet(SFE_MMC5983MA_IO *io, uint8_t registerAddress, uint8_t bitMask);

/* Returns true if the interface in use is SPI */
bool MMC5983MA_IO_spiInUse(SFE_MMC5983MA_IO *io);

#endif /* SPARKFUN_MMC5983MA_STM32_IO_H */
