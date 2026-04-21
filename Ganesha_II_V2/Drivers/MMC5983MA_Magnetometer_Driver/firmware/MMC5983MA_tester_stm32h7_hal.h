#ifndef MMC5983MA_TESTER_STM32H7_HAL_H
#define MMC5983MA_TESTER_STM32H7_HAL_H

#include "stm32h7xx_hal.h"
#include "SparkFun_MMC5983MA_STM32_Library.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MMC5983MA_TESTER_BUS_I2C = 0,
    MMC5983MA_TESTER_BUS_SPI = 1,
} MMC5983MA_TesterBus;

typedef struct {
    SFE_MMC5983MA mag;
    MMC5983MA_TesterBus busType;

    I2C_HandleTypeDef *hi2c;

    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *csPort;
    uint16_t csPin;

    UART_HandleTypeDef *huart;
    GPIO_TypeDef *statusLedPort;
    uint16_t statusLedPin;

    uint32_t lastRunMs;
    uint8_t initialized;
} MMC5983MA_Tester;

void MMC5983MA_Tester_InitI2C(MMC5983MA_Tester *tester,
                              I2C_HandleTypeDef *hi2c,
                              UART_HandleTypeDef *huart,
                              GPIO_TypeDef *statusLedPort,
                              uint16_t statusLedPin);

void MMC5983MA_Tester_InitSPI(MMC5983MA_Tester *tester,
                              SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *csPort,
                              uint16_t csPin,
                              UART_HandleTypeDef *huart,
                              GPIO_TypeDef *statusLedPort,
                              uint16_t statusLedPin);

bool MMC5983MA_Tester_Run(MMC5983MA_Tester *tester);

#ifdef __cplusplus
}
#endif

#endif
