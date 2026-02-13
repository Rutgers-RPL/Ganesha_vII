/*
 This is a modified library for the MMC5983MA High Performance Magnetometer, adapted for STM32 microcontrollers using the HAL library.
 The original library was written by Ricardo Ramos @ SparkFun Electronics, February 2nd, 2022.
 This file declares all functions used in the MMC5983MA High Performance Magnetometer STM32 Library I2C/SPI IO layer.
 */

#ifndef _SPARKFUN_MMC5983MA_STM32_IO_
#define _SPARKFUN_MMC5983MA_STM32_IO_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

class SFE_MMC5983MA_IO {
private:
	SPI_HandleTypeDef *_hspi = nullptr;
	GPIO_TypeDef *_csPort = nullptr;
	uint16_t _csPin = 0;

	I2C_HandleTypeDef *_hi2c = nullptr;
	uint8_t _address = 0;
	bool useSPI = false;

	// HAL timeout value in milliseconds
	static const uint32_t HAL_TIMEOUT_MS = 100;

public:
	// Default empty constructor.
	SFE_MMC5983MA_IO() = default;

	// Default empty destructor
	~SFE_MMC5983MA_IO() = default;

	// Configures and starts the I2C I/O layer.
	bool begin(I2C_HandleTypeDef *hi2c);

	// Configures and starts the SPI I/O layer.
	bool begin(GPIO_TypeDef *csPort, uint16_t csPin, SPI_HandleTypeDef *hspi);

	// Returns true if we get the correct product ID from the device.
	bool isConnected();

	// Read a single uint8_t from a register.
	bool readSingleByte(const uint8_t registerAddress, uint8_t *buffer);

	// Writes a single uint8_t into a register.
	bool writeSingleByte(const uint8_t registerAddress, const uint8_t value);

	// Reads multiple bytes from a register into buffer uint8_t array.
	bool readMultipleBytes(const uint8_t registerAddress, uint8_t *const buffer,
			const uint8_t packetLength);

	// Writes multiple bytes to register from buffer uint8_t array.
	bool writeMultipleBytes(const uint8_t registerAddress,
			uint8_t *const buffer, const uint8_t packetLength);

	// Sets a single bit in a specific register. Bit position ranges from 0 (lsb) to 7 (msb).
	bool setRegisterBit(const uint8_t registerAddress, const uint8_t bitMask);

	// Clears a single bit in a specific register. Bit position ranges from 0 (lsb) to 7 (msb).
	bool clearRegisterBit(const uint8_t registerAddress, const uint8_t bitMask);

	// Returns true if a specific bit is set in a register. Bit position ranges from 0 (lsb) to 7 (msb).
	bool isBitSet(const uint8_t registerAddress, const uint8_t bitMask);

	// Returns true if the interface in use is SPI
	bool spiInUse();
};

#endif
