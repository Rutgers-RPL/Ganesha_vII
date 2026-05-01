/*
 * mmc5983ma.c
 *
 *  Created on: May 1, 2025
 *      Author: Mahir Shah (mahir.shah@rutgers.edu)
 */

#include "mmc5983ma.h"
#include "main.h"
#include <string.h>



HAL_StatusTypeDef mmc5983ma_read_register(struct MMC5983MA* mmc, uint8_t reg_addr, uint8_t* data, uint16_t len) {
    HAL_StatusTypeDef ret;
    uint8_t tx_buffer[9];  // 1 address byte + up to 8 data bytes
    uint8_t rx_buffer[9];

    if (len > 8) return HAL_ERROR;

    // Prepare read command
    tx_buffer[0] = MMC5983MA_SPI_READ | reg_addr;
    // Fill rest with dummy bytes for clocking in data
    for (uint16_t i = 1; i <= len; i++) {
        tx_buffer[i] = 0x00;
    }

    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_RESET);

    ret = HAL_SPI_TransmitReceive(mmc->spi_handle, tx_buffer, rx_buffer, len + 1, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);

    if (ret == HAL_OK) {
        // Copy received data (skip first byte which is during address transmission)
        for (uint16_t i = 0; i < len; i++) {
            data[i] = rx_buffer[i + 1];
        }
    }

    return ret;
}

/**
 * @brief Write to MMC5983MA register via SPI
 */
HAL_StatusTypeDef mmc5983ma_write_register(struct MMC5983MA* mmc, uint8_t reg_addr, uint8_t data) {
    HAL_StatusTypeDef ret;
    uint8_t tx_buffer[2];
    
    // MMC5983MA SPI protocol: write = address without bit 7 set
    // This matches the working SparkFun library implementation
    tx_buffer[0] = MMC5983MA_SPI_WRITE | reg_addr;  // 0x00 | reg_addr
    tx_buffer[1] = data;
    
    // Pull CS low
    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_RESET);
    
    // Send register address and data
    ret = HAL_SPI_Transmit(mmc->spi_handle, tx_buffer, 2, HAL_MAX_DELAY);
    
    // Pull CS high
    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
    
    return ret;
}

/**
 * @brief Initialize the MMC5983MA magnetometer
 */
HAL_StatusTypeDef mmc5983ma_init(struct MMC5983MA* mmc, SPI_HandleTypeDef* spi_handle, uint8_t use_18bit) {
    HAL_StatusTypeDef ret;
    uint8_t product_id;
    uint8_t status;
    
    // Store SPI handle and mode
    mmc->spi_handle = spi_handle;
    mmc->use_18bit = use_18bit;
    mmc->ctrl0_shadow = 0;
    
    // Set CS high initially
    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
    
    // Wait for power-up (10ms typical)
    HAL_Delay(10);
    
    // Verify product ID
    ret = mmc5983ma_read_product_id(mmc, &product_id);
    if (ret != HAL_OK) {
        // SPI communication failed
        return HAL_ERROR;
    }
    
    // Debug: Store product ID for inspection
    // mmc->product_id_read = product_id;
    
    // if (product_id != MMC5983MA_PRODUCT_ID) {
        // Wrong product ID
   //      return HAL_ERROR;
  //   }
    
    // Wait for OTP read to complete
    uint32_t timeout = HAL_GetTick() + 100;
    do {
        ret = mmc5983ma_read_status(mmc, &status);
        if (ret != HAL_OK) {
            return ret;
        }
        if (HAL_GetTick() > timeout) {
            return HAL_TIMEOUT;
        }
    } while (!(status & MMC5983MA_STATUS_OTP_READ_DONE));
    
    // Software reset to ensure clean state
    ret = mmc5983ma_software_reset(mmc);
    if (ret != HAL_OK) {
        return ret;
    }
    
    HAL_Delay(10);


    ret = mmc5983ma_set_bandwidth(mmc, MMC5983MA_BW_800HZ);
    if (ret != HAL_OK) {
        return ret;
    }
    
    ret = mmc5983ma_reset(mmc);
    if (ret != HAL_OK) {
        return ret;
    }

    HAL_Delay(1);

    mmc->ctrl0_shadow |= MMC5983MA_CTRL0_AUTO_SR_EN | MMC5983MA_CTRL0_INT_MEAS_DONE;
    ret = mmc5983ma_write_register(mmc, MMC5983MA_REG_CTRL0, mmc->ctrl0_shadow);
    if (ret != HAL_OK) {
        return ret;
    }
    
    uint8_t ctrl2_val = MMC5983MA_CTRL2_CMM_EN | (MMC5983MA_CMM_FREQ_1000HZ << 0);
    ret = mmc5983ma_write_register(mmc, MMC5983MA_REG_CTRL2, ctrl2_val);
    if (ret != HAL_OK) {
        return ret;
    }
    
    return HAL_OK;
}

/**
 * @brief Read product ID
 */
HAL_StatusTypeDef mmc5983ma_read_product_id(struct MMC5983MA* mmc, uint8_t* product_id) {
    return mmc5983ma_read_register(mmc, MMC5983MA_REG_PRODUCT_ID, product_id, 1);
}

/**
 * @brief Perform SET operation
 */
HAL_StatusTypeDef mmc5983ma_set(struct MMC5983MA* mmc) {
    HAL_StatusTypeDef ret;
    mmc->ctrl0_shadow |= MMC5983MA_CTRL0_SET;
    ret = mmc5983ma_write_register(mmc, MMC5983MA_REG_CTRL0, mmc->ctrl0_shadow);
    mmc->ctrl0_shadow &= ~MMC5983MA_CTRL0_SET;  // self-clearing bit
    return ret;
}

/**
 * @brief Perform RESET operation
 */
HAL_StatusTypeDef mmc5983ma_reset(struct MMC5983MA* mmc) {
    HAL_StatusTypeDef ret;
    mmc->ctrl0_shadow |= MMC5983MA_CTRL0_RESET;
    ret = mmc5983ma_write_register(mmc, MMC5983MA_REG_CTRL0, mmc->ctrl0_shadow);
    mmc->ctrl0_shadow &= ~MMC5983MA_CTRL0_RESET;  // self-clearing bit
    return ret;
}

/**
 * @brief Take a single magnetic field measurement
 */
HAL_StatusTypeDef mmc5983ma_measure_magnetic(struct MMC5983MA* mmc) {
    HAL_StatusTypeDef ret;
    mmc->ctrl0_shadow |= MMC5983MA_CTRL0_TM_M;
    ret = mmc5983ma_write_register(mmc, MMC5983MA_REG_CTRL0, mmc->ctrl0_shadow);
    mmc->ctrl0_shadow &= ~MMC5983MA_CTRL0_TM_M;  // self-clearing bit
    return ret;
}

/**
 * @brief Take a single temperature measurement
 */
HAL_StatusTypeDef mmc5983ma_measure_temperature(struct MMC5983MA* mmc) {
    HAL_StatusTypeDef ret;
    mmc->ctrl0_shadow |= MMC5983MA_CTRL0_TM_T;
    ret = mmc5983ma_write_register(mmc, MMC5983MA_REG_CTRL0, mmc->ctrl0_shadow);
    mmc->ctrl0_shadow &= ~MMC5983MA_CTRL0_TM_T;  // self-clearing bit
    return ret;
}

/**
 * @brief Read status register
 */
HAL_StatusTypeDef mmc5983ma_read_status(struct MMC5983MA* mmc, uint8_t* status) {
    return mmc5983ma_read_register(mmc, MMC5983MA_REG_STATUS, status, 1);
}

/**
 * @brief Read magnetic field data
 */
HAL_StatusTypeDef mmc5983ma_read_mag_data(struct MMC5983MA* mmc, struct mmc5983ma_mag_data* mag_data) {
    HAL_StatusTypeDef ret;
    uint8_t raw_data[7];  // 6 bytes for XYZ + 1 byte for LSBs
    
    // Read all magnetic data registers (0x00 to 0x06)
    ret = mmc5983ma_read_register(mmc, MMC5983MA_REG_XOUT0, raw_data, 7);
    if (ret != HAL_OK) {
        return ret;
    }
    
    // Parse raw data based on mode
    if (mmc->use_18bit) {
        // 18-bit mode: use all bits
        mag_data->x_raw = ((uint32_t)raw_data[0] << 10) | ((uint32_t)raw_data[1] << 2) | ((raw_data[6] >> 6) & 0x03);
        mag_data->y_raw = ((uint32_t)raw_data[2] << 10) | ((uint32_t)raw_data[3] << 2) | ((raw_data[6] >> 4) & 0x03);
        mag_data->z_raw = ((uint32_t)raw_data[4] << 10) | ((uint32_t)raw_data[5] << 2) | ((raw_data[6] >> 2) & 0x03);
        
        // Convert to Gauss (18-bit mode)
        mag_data->x_gauss = ((float)mag_data->x_raw - (MMC5983MA_NULL_FIELD_OUTPUT * 4)) / MMC5983MA_SENSITIVITY_18BIT;
        mag_data->y_gauss = ((float)mag_data->y_raw - (MMC5983MA_NULL_FIELD_OUTPUT * 4)) / MMC5983MA_SENSITIVITY_18BIT;
        mag_data->z_gauss = ((float)mag_data->z_raw - (MMC5983MA_NULL_FIELD_OUTPUT * 4)) / MMC5983MA_SENSITIVITY_18BIT;
    } else {
        // 16-bit mode: use upper 16 bits only
        mag_data->x_raw = ((uint32_t)raw_data[0] << 8) | raw_data[1];
        mag_data->y_raw = ((uint32_t)raw_data[2] << 8) | raw_data[3];
        mag_data->z_raw = ((uint32_t)raw_data[4] << 8) | raw_data[5];
        
        // Convert to Gauss (16-bit mode)
        mag_data->x_gauss = ((float)mag_data->x_raw - MMC5983MA_NULL_FIELD_OUTPUT) / MMC5983MA_SENSITIVITY_16BIT;
        mag_data->y_gauss = ((float)mag_data->y_raw - MMC5983MA_NULL_FIELD_OUTPUT) / MMC5983MA_SENSITIVITY_16BIT;
        mag_data->z_gauss = ((float)mag_data->z_raw - MMC5983MA_NULL_FIELD_OUTPUT) / MMC5983MA_SENSITIVITY_16BIT;
    }
    
    return HAL_OK;
}

/**
 * @brief Read temperature data
 */
HAL_StatusTypeDef mmc5983ma_read_temp_data(struct MMC5983MA* mmc, struct mmc5983ma_temp_data* temp_data) {
    HAL_StatusTypeDef ret;
    
    ret = mmc5983ma_read_register(mmc, MMC5983MA_REG_TOUT, &temp_data->raw, 1);
    if (ret != HAL_OK) {
        return ret;
    }
    
    // Convert to Celsius
    temp_data->temp_c = MMC5983MA_TEMP_OFFSET + (temp_data->raw * MMC5983MA_TEMP_SENSITIVITY);
    
    return HAL_OK;
}

/**
 * @brief Enable continuous measurement mode
 */
HAL_StatusTypeDef mmc5983ma_enable_continuous_mode(struct MMC5983MA* mmc, uint8_t frequency) {
    uint8_t ctrl2_val = MMC5983MA_CTRL2_CMM_EN | (frequency & 0x07);
    return mmc5983ma_write_register(mmc, MMC5983MA_REG_CTRL2, ctrl2_val);
}

/**
 * @brief Disable continuous measurement mode
 */
HAL_StatusTypeDef mmc5983ma_disable_continuous_mode(struct MMC5983MA* mmc) {
    return mmc5983ma_write_register(mmc, MMC5983MA_REG_CTRL2, 0x00);
}

/**
 * @brief Set bandwidth (measurement time)
 */
HAL_StatusTypeDef mmc5983ma_set_bandwidth(struct MMC5983MA* mmc, uint8_t bandwidth) {
    uint8_t ctrl1_val = bandwidth & 0x03;  // BW bits are [1:0]
    return mmc5983ma_write_register(mmc, MMC5983MA_REG_CTRL1, ctrl1_val);
}

/**
 * @brief Software reset
 */
HAL_StatusTypeDef mmc5983ma_software_reset(struct MMC5983MA* mmc) {
    HAL_StatusTypeDef ret;

    ret = mmc5983ma_write_register(mmc, MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_SW_RST);
    if (ret != HAL_OK) {
        return ret;
    }

    mmc->ctrl0_shadow = 0;  // chip clears all registers on reset

    HAL_Delay(10);

    return HAL_OK;
}

/**
 * @brief Enable measurement done interrupt
 */
HAL_StatusTypeDef mmc5983ma_enable_interrupt(struct MMC5983MA* mmc) {
    mmc->ctrl0_shadow |= MMC5983MA_CTRL0_INT_MEAS_DONE;
    return mmc5983ma_write_register(mmc, MMC5983MA_REG_CTRL0, mmc->ctrl0_shadow);
}
