#include "bmp581.h"

#include "bmp5_defs.h"
#include "stm32h7xx_hal.h"

#include <cstdint>

BMP5_INTF_RET_TYPE read_spi(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	SPI_HandleTypeDef *handle = (SPI_HandleTypeDef*) intf_ptr;
	return HAL_SPI_Receive_IT(handle, reg_data, (uint16_t) length);
}

BMP5_INTF_RET_TYPE write_spi(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	SPI_HandleTypeDef *handle = (SPI_HandleTypeDef*) intf_ptr;
	return HAL_SPI_Transmit_IT(handle, reg_data, (uint16_t) length);
}

int8_t bmp581_init(struct BMP581 *bmp581, SPI_HandleTypeDef *handle)
{
	int8_t result = BMP5_OK;

	// Use SPI by default, since that is wired on our end
	bmp581->hi2c = handle;
	bmp581->device.intf = BMP5_SPI_INTF;
	bmp581->device.read = read_spi;
	bmp581->device.write = write_spi;
	bmp581->device.intf_ptr = hi2c;
	bmp581->odr_config.odr = BMP5_ODR_240_HZ;

	// Initialize the device
	result = bmp5_init(&bmp581->device);
	if (result != BMP5_OK) {
		return result;
	}

	// Set odr frequency
	result = bmp5_set_osr_odr_press_config(&bmp581->odr_config, &bmp581->device);
	if (result != BMP5_OK) {
		return result;
	}

	// Enable interrupt handler
	result = bmp5_configure_interrupt(BMP5_PULSED, BMP5_ACTIVE_HIGH, BMP5_INTR_PUSH_PULL, BMP5_INTR_ENABLE, &bmp581->device);
	if (result != BMP5_OK) {
		return result;
	}
	return result;
}


int8_t bmp581_get_data(struct BMP581 *bmp581, struct bmp5_sensor_data *data)
{
	return bmp5_get_sensor_data(data, &bmp581->odr_config, &bmp581->device);
}
