#include "MMC5983MA_tester_stm32h7_hal.h"

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define MMC5983MA_TEST_PERIOD_MS 500U
#define MMC5983MA_RAW_MIN        1U
#define MMC5983MA_RAW_MAX        262142U

static void testerPrint(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    if (huart == NULL) {
        return;
    }

    char msg[192];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    if (len <= 0) {
        return;
    }

    if (len >= (int)sizeof(msg)) {
        len = (int)sizeof(msg) - 1;
        msg[len] = '\0';
    }

    (void)HAL_UART_Transmit(huart, (uint8_t *)msg, (uint16_t)len, HAL_MAX_DELAY);
}

static void testerCommonInit(MMC5983MA_Tester *tester,
                             UART_HandleTypeDef *huart,
                             GPIO_TypeDef *statusLedPort,
                             uint16_t statusLedPin)
{
    (void)memset(tester, 0, sizeof(*tester));

    tester->huart = huart;
    tester->statusLedPort = statusLedPort;
    tester->statusLedPin = statusLedPin;
    tester->lastRunMs = 0U;
    tester->initialized = 0U;

    if (tester->statusLedPort != NULL) {
        HAL_GPIO_WritePin(tester->statusLedPort, tester->statusLedPin, GPIO_PIN_RESET);
    }

    MMC5983MA_init(&tester->mag);
    // MMC5983MA_init(&tester->magnetometer);
    testerPrint(tester->huart, "MMC5983MA Tester (STM32H7 HAL)\r\n");
}

void MMC5983MA_Tester_InitI2C(MMC5983MA_Tester *tester,
                              I2C_HandleTypeDef *hi2c,
                              UART_HandleTypeDef *huart,
                              GPIO_TypeDef *statusLedPort,
                              uint16_t statusLedPin)
{
    testerCommonInit(tester, huart, statusLedPort, statusLedPin);
    tester->busType = MMC5983MA_TESTER_BUS_I2C;
    tester->hi2c = hi2c;
}

void MMC5983MA_Tester_InitSPI(MMC5983MA_Tester *tester,
                              SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *csPort,
                              uint16_t csPin,
                              UART_HandleTypeDef *huart,
                              GPIO_TypeDef *statusLedPort,
                              uint16_t statusLedPin)
{
    testerCommonInit(tester, huart, statusLedPort, statusLedPin);
    tester->busType = MMC5983MA_TESTER_BUS_SPI;
    tester->hspi = hspi;
    tester->csPort = csPort;
    tester->csPin = csPin;
}

static bool testerEnsureConnected(MMC5983MA_Tester *tester)
{
    if (!tester->initialized) {
        bool beginOk = false;

        if (tester->busType == MMC5983MA_TESTER_BUS_SPI) {
            beginOk = MMC5983MA_beginSPI(&tester->mag, tester->csPort, tester->csPin, tester->hspi);
        } else {
            beginOk = MMC5983MA_beginI2C(&tester->mag, tester->hi2c);
        }

        if (!beginOk) {
            return false;
        }

        (void)MMC5983MA_softReset(&tester->mag);
        (void)MMC5983MA_setFilterBandwidth(&tester->mag, 400U);
        tester->initialized = 1U;
        return true;
    }

    return MMC5983MA_isConnected(&tester->mag);
}

static double calculateHeadingDegrees(double normalizedX, double normalizedY)
{
    if (normalizedY != 0.0) {
        if (normalizedX < 0.0) {
            if (normalizedY > 0.0) {
                return 57.2958 * atan(-normalizedX / normalizedY);
            }
            return 57.2958 * atan(-normalizedX / normalizedY) + 180.0;
        }

        if (normalizedY < 0.0) {
            return 57.2958 * atan(-normalizedX / normalizedY) + 180.0;
        }

        return 360.0 - (57.2958 * atan(normalizedX / normalizedY));
    }

    return (normalizedX > 0.0) ? 270.0 : 90.0;
}

bool MMC5983MA_Tester_Run(MMC5983MA_Tester *tester)
{
    uint32_t now = HAL_GetTick();
    if ((now - tester->lastRunMs) < MMC5983MA_TEST_PERIOD_MS) {
        return false;
    }
    tester->lastRunMs = now;

    if (!testerEnsureConnected(tester)) {
        tester->initialized = 0U;
        testerPrint(tester->huart, "MMC5983MA not detected.\r\n");
        if (tester->statusLedPort != NULL) {
            HAL_GPIO_WritePin(tester->statusLedPort, tester->statusLedPin, GPIO_PIN_RESET);
        }
        return false;
    }

    int celsius = MMC5983MA_getTemperature(&tester->mag);
    float fahrenheit = (celsius * 9.0f / 5.0f) + 32.0f;

    uint32_t currentX = 0U;
    uint32_t currentY = 0U;
    uint32_t currentZ = 0U;

    if (!MMC5983MA_getMeasurementXYZ(&tester->mag, &currentX, &currentY, &currentZ)) {
        testerPrint(tester->huart, "MMC5983MA measurement failed.\r\n\r\n");
        if (tester->statusLedPort != NULL) {
            HAL_GPIO_WritePin(tester->statusLedPort, tester->statusLedPin, GPIO_PIN_RESET);
        }
        return false;
    }

    bool good = (currentX >= MMC5983MA_RAW_MIN) && (currentX <= MMC5983MA_RAW_MAX) &&
                (currentY >= MMC5983MA_RAW_MIN) && (currentY <= MMC5983MA_RAW_MAX) &&
                (currentZ >= MMC5983MA_RAW_MIN) && (currentZ <= MMC5983MA_RAW_MAX);

    testerPrint(tester->huart,
                "MMC5983MA connected.\r\n"
                "Die temperature: %dC / %dF\r\n",
                celsius,
                (int)fahrenheit);

    if (!good) {
        testerPrint(tester->huart,
                    "X axis raw value: %lu\tY axis raw value: %lu\tZ axis raw value: %lu\r\n"
                    "Values out of range.\r\n\r\n",
                    (unsigned long)currentX,
                    (unsigned long)currentY,
                    (unsigned long)currentZ);

        if (tester->statusLedPort != NULL) {
            HAL_GPIO_WritePin(tester->statusLedPort, tester->statusLedPin, GPIO_PIN_RESET);
        }
        return false;
    }

    double normalizedX = ((double)currentX - 131072.0) / 131072.0;
    double normalizedY = ((double)currentY - 131072.0) / 131072.0;
    double normalizedZ = ((double)currentZ - 131072.0) / 131072.0;
    double heading = calculateHeadingDegrees(normalizedX, normalizedY);

    testerPrint(tester->huart,
                "X axis raw value: %lu\tY axis raw value: %lu\tZ axis raw value: %lu\r\n"
                "Normalized XYZ: %.4f %.4f %.4f\r\n"
                "Heading: %.1f\r\n\r\n",
                (unsigned long)currentX,
                (unsigned long)currentY,
                (unsigned long)currentZ,
                normalizedX,
                normalizedY,
                normalizedZ,
                heading);

    if (tester->statusLedPort != NULL) {
        HAL_GPIO_WritePin(tester->statusLedPort, tester->statusLedPin, GPIO_PIN_SET);
    }
    return true;
}
