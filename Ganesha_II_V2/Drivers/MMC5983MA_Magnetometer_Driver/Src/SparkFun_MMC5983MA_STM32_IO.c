/*
  This is a library written for the MMC5983MA High Performance Magnetometer.
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/19034

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, February 2nd, 2022.
  Modified for STM32 HAL compatibility.
  Converted from C++ to C.

  This file implements all functions used in the MMC5983MA High Performance
  Magnetometer STM32 Library IO layer.

  SparkFun code, firmware, and software is released under the MIT License
  (http://opensource.org/licenses/MIT). See LICENSE.md for more information.
*/

#include "SparkFun_MMC5983MA_STM32_Library_Constants.h"
#include "SparkFun_MMC5983MA_STM32_IO.h"

/* Read operations must have the most significant bit set for SPI */
#define READ_REG(x) (0x80u | (x))

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

void MMC5983MA_IO_init(SFE_MMC5983MA_IO *io)
{
    io->hspi    = NULL;
    io->csPort  = NULL;
    io->csPin   = 0;
    io->hi2c    = NULL;
    io->address = 0;
    io->useSPI  = false;
}

bool MMC5983MA_IO_beginI2C(SFE_MMC5983MA_IO *io, I2C_HandleTypeDef *hi2c)
{
    io->useSPI  = false;
    io->hi2c    = hi2c;
    io->address = (uint8_t)(I2C_ADDR << 1); /* STM32 HAL uses 8-bit address */
    return MMC5983MA_IO_isConnected(io);
}

bool MMC5983MA_IO_beginSPI(SFE_MMC5983MA_IO *io, GPIO_TypeDef *csPort, uint16_t csPin, SPI_HandleTypeDef *hspi)
{
    io->useSPI = true;
    io->csPort = csPort;
    io->csPin  = csPin;
    io->hspi   = hspi;

    /* Set CS pin high (inactive) */
    HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_SET);

    return MMC5983MA_IO_isConnected(io);
}

/* ── Connection ──────────────────────────────────────────────────────────── */

bool MMC5983MA_IO_isConnected(SFE_MMC5983MA_IO *io)
{
    bool result = false;

    if (io->useSPI)
    {
        uint8_t txData = READ_REG(PROD_ID_REG);
        uint8_t rxData = 0;

        HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_RESET);

        if (HAL_SPI_Transmit(io->hspi, &txData, 1, MMC5983MA_HAL_TIMEOUT_MS) == HAL_OK)
        {
            uint8_t dummy = DUMMY;
            if (HAL_SPI_TransmitReceive(io->hspi, &dummy, &rxData, 1, MMC5983MA_HAL_TIMEOUT_MS) == HAL_OK)
            {
                result = (rxData == PROD_ID);
            }
        }

        HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_SET);
    }
    else
    {
        if (HAL_I2C_IsDeviceReady(io->hi2c, io->address, 3, MMC5983MA_HAL_TIMEOUT_MS) == HAL_OK)
        {
            uint8_t id = 0;
            result  = MMC5983MA_IO_readSingleByte(io, PROD_ID_REG, &id);
            result &= (id == PROD_ID);
        }
    }

    return result;
}

/* ── Write ───────────────────────────────────────────────────────────────── */

bool MMC5983MA_IO_writeSingleByte(SFE_MMC5983MA_IO *io, uint8_t registerAddress, uint8_t value)
{
    bool success = true;

    if (io->useSPI)
    {
        HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_RESET);

        uint8_t regAddr = registerAddress;
        if (HAL_SPI_Transmit(io->hspi, &regAddr, 1, MMC5983MA_HAL_TIMEOUT_MS) == HAL_OK)
        {
            uint8_t data = value;
            success = (HAL_SPI_Transmit(io->hspi, &data, 1, MMC5983MA_HAL_TIMEOUT_MS) == HAL_OK);
        }
        else
        {
            success = false;
        }

        HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_SET);
    }
    else
    {
        uint8_t data = value;
        success = (HAL_I2C_Mem_Write(io->hi2c, io->address, registerAddress,
                                     I2C_MEMADD_SIZE_8BIT, &data, 1,
                                     MMC5983MA_HAL_TIMEOUT_MS) == HAL_OK);
    }

    return success;
}

bool MMC5983MA_IO_writeMultipleBytes(SFE_MMC5983MA_IO *io, uint8_t registerAddress, uint8_t *buffer, uint8_t packetLength)
{
    bool success = true;

    if (io->useSPI)
    {
        HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_RESET);

        uint8_t regAddr = registerAddress;
        if (HAL_SPI_Transmit(io->hspi, &regAddr, 1, MMC5983MA_HAL_TIMEOUT_MS) == HAL_OK)
        {
            success = (HAL_SPI_Transmit(io->hspi, buffer, packetLength, MMC5983MA_HAL_TIMEOUT_MS) == HAL_OK);
        }
        else
        {
            success = false;
        }

        HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_SET);
    }
    else
    {
        success = (HAL_I2C_Mem_Write(io->hi2c, io->address, registerAddress,
                                     I2C_MEMADD_SIZE_8BIT, buffer, packetLength,
                                     MMC5983MA_HAL_TIMEOUT_MS) == HAL_OK);
    }

    return success;
}

/* ── Read ────────────────────────────────────────────────────────────────── */

bool MMC5983MA_IO_readSingleByte(SFE_MMC5983MA_IO *io, uint8_t registerAddress, uint8_t *buffer)
{
    bool success = true;

    if (io->useSPI)
    {
        uint8_t txData = READ_REG(registerAddress);

        HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_RESET);

        if (HAL_SPI_Transmit(io->hspi, &txData, 1, MMC5983MA_HAL_TIMEOUT_MS) == HAL_OK)
        {
            uint8_t dummy = DUMMY;
            success = (HAL_SPI_TransmitReceive(io->hspi, &dummy, buffer, 1, MMC5983MA_HAL_TIMEOUT_MS) == HAL_OK);
        }
        else
        {
            success = false;
        }

        HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_SET);
    }
    else
    {
        success = (HAL_I2C_Mem_Read(io->hi2c, io->address, registerAddress,
                                    I2C_MEMADD_SIZE_8BIT, buffer, 1,
                                    MMC5983MA_HAL_TIMEOUT_MS) == HAL_OK);
    }

    return success;
}

bool MMC5983MA_IO_readMultipleBytes(SFE_MMC5983MA_IO *io, uint8_t registerAddress, uint8_t *buffer, uint8_t packetLength)
{
    bool success = true;

    if (io->useSPI)
    {
        uint8_t txData = READ_REG(registerAddress);

        HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_RESET);

        if (HAL_SPI_Transmit(io->hspi, &txData, 1, MMC5983MA_HAL_TIMEOUT_MS) == HAL_OK)
        {
            success = (HAL_SPI_Receive(io->hspi, buffer, packetLength, MMC5983MA_HAL_TIMEOUT_MS) == HAL_OK);
        }
        else
        {
            success = false;
        }

        HAL_GPIO_WritePin(io->csPort, io->csPin, GPIO_PIN_SET);
    }
    else
    {
        success = (HAL_I2C_Mem_Read(io->hi2c, io->address, registerAddress,
                                    I2C_MEMADD_SIZE_8BIT, buffer, packetLength,
                                    MMC5983MA_HAL_TIMEOUT_MS) == HAL_OK);
    }

    return success;
}

/* ── Bit helpers ─────────────────────────────────────────────────────────── */

bool MMC5983MA_IO_setRegisterBit(SFE_MMC5983MA_IO *io, uint8_t registerAddress, uint8_t bitMask)
{
    uint8_t value = 0;
    bool success  = MMC5983MA_IO_readSingleByte(io, registerAddress, &value);
    value        |= bitMask;
    success      &= MMC5983MA_IO_writeSingleByte(io, registerAddress, value);
    return success;
}

bool MMC5983MA_IO_clearRegisterBit(SFE_MMC5983MA_IO *io, uint8_t registerAddress, uint8_t bitMask)
{
    uint8_t value = 0;
    bool success  = MMC5983MA_IO_readSingleByte(io, registerAddress, &value);
    value        &= (uint8_t)(~bitMask);
    success      &= MMC5983MA_IO_writeSingleByte(io, registerAddress, value);
    return success;
}

bool MMC5983MA_IO_isBitSet(SFE_MMC5983MA_IO *io, uint8_t registerAddress, uint8_t bitMask)
{
    uint8_t value = 0;
    MMC5983MA_IO_readSingleByte(io, registerAddress, &value);
    return (bool)(value & bitMask);
}

bool MMC5983MA_IO_spiInUse(SFE_MMC5983MA_IO *io)
{
    return io->useSPI;
}
