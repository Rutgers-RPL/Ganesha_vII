/*
 * mmc5983ma.h
 *
 *  Created on: May 1, 2025
 *      Author: Mahir Shah (mahir.shah@rutgers.edu)
 */

#ifndef INC_MMC5983MA_H_
#define INC_MMC5983MA_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>


/* MMC5983MA Register Map */
#define MMC5983MA_REG_XOUT0          0x00
#define MMC5983MA_REG_XOUT1          0x01
#define MMC5983MA_REG_YOUT0          0x02
#define MMC5983MA_REG_YOUT1          0x03
#define MMC5983MA_REG_ZOUT0          0x04
#define MMC5983MA_REG_ZOUT1          0x05
#define MMC5983MA_REG_XYZOUT2        0x06
#define MMC5983MA_REG_TOUT           0x07
#define MMC5983MA_REG_STATUS         0x08
#define MMC5983MA_REG_CTRL0          0x09
#define MMC5983MA_REG_CTRL1          0x0A
#define MMC5983MA_REG_CTRL2          0x0B
#define MMC5983MA_REG_CTRL3          0x0C
#define MMC5983MA_REG_PRODUCT_ID     0x2F

/* Status Register Bits */
#define MMC5983MA_STATUS_MEAS_M_DONE    (1 << 0)
#define MMC5983MA_STATUS_MEAS_T_DONE    (1 << 1)
#define MMC5983MA_STATUS_OTP_READ_DONE  (1 << 4)

/* Control Register 0 Bits */
#define MMC5983MA_CTRL0_TM_M            (1 << 0)  // Take magnetic measurement
#define MMC5983MA_CTRL0_TM_T            (1 << 1)  // Take temperature measurement
#define MMC5983MA_CTRL0_INT_MEAS_DONE   (1 << 2)  // Enable measurement done interrupt
#define MMC5983MA_CTRL0_SET             (1 << 3)  // Perform SET operation
#define MMC5983MA_CTRL0_RESET           (1 << 4)  // Perform RESET operation
#define MMC5983MA_CTRL0_AUTO_SR_EN      (1 << 5)  // Enable automatic SET/RESET
#define MMC5983MA_CTRL0_OTP_READ        (1 << 6)  // Read OTP data

/* Control Register 1 Bits */
#define MMC5983MA_CTRL1_BW0             (1 << 0)
#define MMC5983MA_CTRL1_BW1             (1 << 1)
#define MMC5983MA_CTRL1_X_INHIBIT       (1 << 2)
#define MMC5983MA_CTRL1_YZ_INHIBIT      (3 << 3)
#define MMC5983MA_CTRL1_SW_RST          (1 << 7)

/* Control Register 2 Bits */
#define MMC5983MA_CTRL2_CMM_EN          (1 << 3)  // Continuous measurement mode enable
#define MMC5983MA_CTRL2_EN_PRD_SET      (1 << 7)  // Enable periodic SET

/* Control Register 3 Bits */
#define MMC5983MA_CTRL3_SPI_3WIRE       (1 << 6)  // 3-wire SPI mode

/* Bandwidth Settings */
#define MMC5983MA_BW_100HZ              0x00  // 8ms measurement time
#define MMC5983MA_BW_200HZ              0x01  // 4ms measurement time
#define MMC5983MA_BW_400HZ              0x02  // 2ms measurement time
#define MMC5983MA_BW_800HZ              0x03  // 0.5ms measurement time

/* Continuous Mode Frequencies */
#define MMC5983MA_CMM_FREQ_OFF          0x00
#define MMC5983MA_CMM_FREQ_1HZ          0x01
#define MMC5983MA_CMM_FREQ_10HZ         0x02
#define MMC5983MA_CMM_FREQ_20HZ         0x03
#define MMC5983MA_CMM_FREQ_50HZ         0x04
#define MMC5983MA_CMM_FREQ_100HZ        0x05
#define MMC5983MA_CMM_FREQ_200HZ        0x06  // Requires BW=01
#define MMC5983MA_CMM_FREQ_1000HZ       0x07  // Requires BW=11

/* Product ID */
#define MMC5983MA_PRODUCT_ID            0x30

/* Conversion Constants */
#define MMC5983MA_SENSITIVITY_16BIT     4096.0f   // counts/Gauss
#define MMC5983MA_SENSITIVITY_18BIT     16384.0f  // counts/Gauss
#define MMC5983MA_NULL_FIELD_OUTPUT     32768     // 16-bit mode center value
#define MMC5983MA_TEMP_OFFSET           -75.0f    // Temperature offset in °C
#define MMC5983MA_TEMP_SENSITIVITY      (200.0f / 255.0f)  // °C per LSB

/* SPI Protocol Bits - MMC5983MA SPI read/write encoding
 * Per datasheet: bit 0 = R/W, bit 1 = don't care, bits 2-7 = address
 * However, the working SparkFun library uses bit 7 (0x80) for read operations
 * This appears to be the actual working protocol based on verified hardware
 */
#define MMC5983MA_SPI_READ              0x80  // Set bit 7 for read (verified working)
#define MMC5983MA_SPI_WRITE             0x00  // Clear bit 7 for write

/* Magnetometer Data Structure */
struct mmc5983ma_mag_data {
    int32_t x_raw;      // Raw X-axis data (18-bit)
    int32_t y_raw;      // Raw Y-axis data (18-bit)
    int32_t z_raw;      // Raw Z-axis data (18-bit)
    float x_gauss;      // X-axis in Gauss
    float y_gauss;      // Y-axis in Gauss
    float z_gauss;      // Z-axis in Gauss
};

/* Temperature Data Structure */
struct mmc5983ma_temp_data {
    uint8_t raw;        // Raw temperature data
    float temp_c;       // Temperature in Celsius
};

/* MMC5983MA Device Structure */
struct MMC5983MA {
    SPI_HandleTypeDef* spi_handle;
    uint8_t use_18bit;  // 0 = 16-bit mode, 1 = 18-bit mode
    uint8_t product_id_read;  // Debug: stores actual product ID read during init
    volatile uint8_t data_ready;  // Interrupt flag: 1 when measurement complete
    uint8_t ctrl0_shadow;  // Shadow of write-only CTRL0 register
};

/* Function Prototypes */

/**
 * @brief Initialize the MMC5983MA magnetometer
 * @param mmc Pointer to MMC5983MA device structure
 * @param spi_handle Pointer to SPI handle
 * @param use_18bit 1 for 18-bit mode, 0 for 16-bit mode
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef mmc5983ma_init(struct MMC5983MA* mmc, SPI_HandleTypeDef* spi_handle, uint8_t use_18bit);

/**
 * @brief Read product ID to verify communication
 * @param mmc Pointer to MMC5983MA device structure
 * @param product_id Pointer to store product ID
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef mmc5983ma_read_product_id(struct MMC5983MA* mmc, uint8_t* product_id);

/**
 * @brief Perform SET operation to eliminate offset
 * @param mmc Pointer to MMC5983MA device structure
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef mmc5983ma_set(struct MMC5983MA* mmc);

/**
 * @brief Perform RESET operation to eliminate offset
 * @param mmc Pointer to MMC5983MA device structure
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef mmc5983ma_reset(struct MMC5983MA* mmc);

/**
 * @brief Take a single magnetic field measurement
 * @param mmc Pointer to MMC5983MA device structure
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef mmc5983ma_measure_magnetic(struct MMC5983MA* mmc);

/**
 * @brief Take a single temperature measurement
 * @param mmc Pointer to MMC5983MA device structure
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef mmc5983ma_measure_temperature(struct MMC5983MA* mmc);

/**
 * @brief Read magnetic field data
 * @param mmc Pointer to MMC5983MA device structure
 * @param mag_data Pointer to store magnetic field data
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef mmc5983ma_read_mag_data(struct MMC5983MA* mmc, struct mmc5983ma_mag_data* mag_data);

/**
 * @brief Read temperature data
 * @param mmc Pointer to MMC5983MA device structure
 * @param temp_data Pointer to store temperature data
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef mmc5983ma_read_temp_data(struct MMC5983MA* mmc, struct mmc5983ma_temp_data* temp_data);

/**
 * @brief Read status register
 * @param mmc Pointer to MMC5983MA device structure
 * @param status Pointer to store status byte
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef mmc5983ma_read_status(struct MMC5983MA* mmc, uint8_t* status);

/**
 * @brief Enable continuous measurement mode
 * @param mmc Pointer to MMC5983MA device structure
 * @param frequency Continuous mode frequency (MMC5983MA_CMM_FREQ_xxx)
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef mmc5983ma_enable_continuous_mode(struct MMC5983MA* mmc, uint8_t frequency);

/**
 * @brief Disable continuous measurement mode
 * @param mmc Pointer to MMC5983MA device structure
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef mmc5983ma_disable_continuous_mode(struct MMC5983MA* mmc);

/**
 * @brief Set bandwidth (measurement time)
 * @param mmc Pointer to MMC5983MA device structure
 * @param bandwidth Bandwidth setting (MMC5983MA_BW_xxx)
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef mmc5983ma_set_bandwidth(struct MMC5983MA* mmc, uint8_t bandwidth);

/**
 * @brief Software reset
 * @param mmc Pointer to MMC5983MA device structure
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef mmc5983ma_software_reset(struct MMC5983MA* mmc);

/**
 * @brief Enable measurement done interrupt
 * @param mmc Pointer to MMC5983MA device structure
 * @return HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef mmc5983ma_enable_interrupt(struct MMC5983MA* mmc);

/* Low-level SPI functions */
HAL_StatusTypeDef mmc5983ma_read_register(struct MMC5983MA* mmc, uint8_t reg_addr, uint8_t* data, uint16_t len);
HAL_StatusTypeDef mmc5983ma_write_register(struct MMC5983MA* mmc, uint8_t reg_addr, uint8_t data);

#endif /* INC_MMC5983MA_H_ */
