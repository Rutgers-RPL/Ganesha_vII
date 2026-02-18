/*
  This is a library written for the MMC5983MA High Performance Magnetometer.
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/19034

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, February 2nd, 2022.
  Modified for STM32 HAL compatibility.
  This file implements all functions used in the MMC5983MA High Performance Magnetometer STM32 Library IO layer.

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  See LICENSE.md for more information.
*/
#include "SparkFun_MMC5983MA_STM32_Library_Constants.h"
#include "SparkFun_MMC5983MA_STM32_IO.h"

// Read operations must have the most significant bit set for SPI
#define READ_REG(x) (0x80 | x)

bool SFE_MMC5983MA_IO::begin(I2C_HandleTypeDef *hi2c)
{
    useSPI = false;
    _hi2c = hi2c;
    _address = I2C_ADDR << 1; // STM32 HAL uses 8-bit address (7-bit address shifted left)
    return isConnected();
}

bool SFE_MMC5983MA_IO::begin(GPIO_TypeDef *csPort, uint16_t csPin, SPI_HandleTypeDef *hspi)
{
    useSPI = true;
    _csPort = csPort;
    _csPin = csPin;
    _hspi = hspi;
    
    // Set CS pin high (inactive)
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
    
    return isConnected();
}

bool SFE_MMC5983MA_IO::isConnected()
{
    bool result = false;
    
    if (useSPI)
    {
        uint8_t txData = READ_REG(PROD_ID_REG);
        uint8_t rxData = 0;
        
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
        
        // Send register address
        if (HAL_SPI_Transmit(_hspi, &txData, 1, HAL_TIMEOUT_MS) == HAL_OK)
        {
            // Receive data
            uint8_t dummy = DUMMY;
            if (HAL_SPI_TransmitReceive(_hspi, &dummy, &rxData, 1, HAL_TIMEOUT_MS) == HAL_OK)
            {
                result = (rxData == PROD_ID);
            }
        }
        
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
    }
    else
    {
        // Check if device responds on I2C bus
        if (HAL_I2C_IsDeviceReady(_hi2c, _address, 3, HAL_TIMEOUT_MS) == HAL_OK)
        {
            uint8_t id = 0;
            result = readSingleByte(PROD_ID_REG, &id);
            result &= (id == PROD_ID);
        }
    }
    
    return result;
}

bool SFE_MMC5983MA_IO::writeMultipleBytes(const uint8_t registerAddress, uint8_t *const buffer, uint8_t const packetLength)
{
    bool success = true;
    
    if (useSPI)
    {
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
        
        // Send register address
        uint8_t regAddr = registerAddress;
        if (HAL_SPI_Transmit(_hspi, &regAddr, 1, HAL_TIMEOUT_MS) == HAL_OK)
        {
            // Send data
            success = (HAL_SPI_Transmit(_hspi, buffer, packetLength, HAL_TIMEOUT_MS) == HAL_OK);
        }
        else
        {
            success = false;
        }
        
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
    }
    else
    {
        // Use HAL_I2C_Mem_Write for I2C
        success = (HAL_I2C_Mem_Write(_hi2c, _address, registerAddress, I2C_MEMADD_SIZE_8BIT, 
                                     buffer, packetLength, HAL_TIMEOUT_MS) == HAL_OK);
    }
    
    return success;
}

bool SFE_MMC5983MA_IO::readMultipleBytes(const uint8_t registerAddress, uint8_t *const buffer, const uint8_t packetLength)
{
    bool success = true;
    
    if (useSPI)
    {
        uint8_t txData = READ_REG(registerAddress);
        
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
        
        // Send register address with read bit
        if (HAL_SPI_Transmit(_hspi, &txData, 1, HAL_TIMEOUT_MS) == HAL_OK)
        {
            // Receive data
            success = (HAL_SPI_Receive(_hspi, buffer, packetLength, HAL_TIMEOUT_MS) == HAL_OK);
        }
        else
        {
            success = false;
        }
        
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
    }
    else
    {
        // Use HAL_I2C_Mem_Read for I2C
        success = (HAL_I2C_Mem_Read(_hi2c, _address, registerAddress, I2C_MEMADD_SIZE_8BIT, 
                                    buffer, packetLength, HAL_TIMEOUT_MS) == HAL_OK);
    }
    
    return success;
}

bool SFE_MMC5983MA_IO::readSingleByte(const uint8_t registerAddress, uint8_t *buffer)
{
    bool success = true;
    
    if (useSPI)
    {
        uint8_t txData = READ_REG(registerAddress);
        
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
        
        // Send register address with read bit
        if (HAL_SPI_Transmit(_hspi, &txData, 1, HAL_TIMEOUT_MS) == HAL_OK)
        {
            // Receive single byte
            uint8_t dummy = DUMMY;
            success = (HAL_SPI_TransmitReceive(_hspi, &dummy, buffer, 1, HAL_TIMEOUT_MS) == HAL_OK);
        }
        else
        {
            success = false;
        }
        
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
    }
    else
    {
        // Use HAL_I2C_Mem_Read for I2C
        success = (HAL_I2C_Mem_Read(_hi2c, _address, registerAddress, I2C_MEMADD_SIZE_8BIT, 
                                    buffer, 1, HAL_TIMEOUT_MS) == HAL_OK);
    }
    
    return success;
}

bool SFE_MMC5983MA_IO::writeSingleByte(const uint8_t registerAddress, const uint8_t value)
{
    bool success = true;
    
    if (useSPI)
    {
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
        
        // Send register address
        uint8_t regAddr = registerAddress;
        if (HAL_SPI_Transmit(_hspi, &regAddr, 1, HAL_TIMEOUT_MS) == HAL_OK)
        {
            // Send data
            uint8_t data = value;
            success = (HAL_SPI_Transmit(_hspi, &data, 1, HAL_TIMEOUT_MS) == HAL_OK);
        }
        else
        {
            success = false;
        }
        
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
    }
    else
    {
        // Use HAL_I2C_Mem_Write for I2C
        uint8_t data = value;
        success = (HAL_I2C_Mem_Write(_hi2c, _address, registerAddress, I2C_MEMADD_SIZE_8BIT, 
                                     &data, 1, HAL_TIMEOUT_MS) == HAL_OK);
    }
    
    return success;
}

bool SFE_MMC5983MA_IO::setRegisterBit(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = 0;
    bool success = readSingleByte(registerAddress, &value);
    value |= bitMask;
    success &= writeSingleByte(registerAddress, value);
    return success;
}

bool SFE_MMC5983MA_IO::clearRegisterBit(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = 0;
    bool success = readSingleByte(registerAddress, &value);
    value &= ~bitMask;
    success &= writeSingleByte(registerAddress, value);
    return success;
}

bool SFE_MMC5983MA_IO::isBitSet(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = 0;
    readSingleByte(registerAddress, &value);
    return (value & bitMask);
}

bool SFE_MMC5983MA_IO::spiInUse()
{
    return useSPI;
}