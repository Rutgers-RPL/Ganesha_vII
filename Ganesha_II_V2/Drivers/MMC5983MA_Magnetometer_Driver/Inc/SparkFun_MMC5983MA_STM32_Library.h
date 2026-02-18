/*
  This is a modified library for the MMC5983MA High Performance Magnetometer, adapted for STM32 microcontrollers using the HAL library.
  The original library was written by Ricardo Ramos @ SparkFun Electronics, February 2nd, 2022.
  This file declares all functions used in the MMC5983MA High Performance Magnetometer STM32 Library.
*/

#ifndef SPARKFUN_MMC5983MA_H
#define SPARKFUN_MMC5983MA_H

#include "SparkFun_MMC5983MA_STM32_IO.h"
#include "SparkFun_MMC5983MA_STM32_Library_Constants.h"

/* Shadow memory for write-only registers - default reset values are 0x0 */
typedef struct {
    uint8_t internalControl0;
    uint8_t internalControl1;
    uint8_t internalControl2;
    uint8_t internalControl3;
} MMC5983MA_MemoryShadow;

typedef struct {
    /* I2C/SPI communication object */
    SFE_MMC5983MA_IO mmc_io;

    /* Error callback: function must accept an SF_MMC5983MA_ERROR as errorCode */
    void (*errorCallback)(SF_MMC5983MA_ERROR errorCode);

    /* Shadow memory for write-only registers */
    MMC5983MA_MemoryShadow memoryShadow;
} SFE_MMC5983MA;

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

/* Initialise struct to safe defaults (replaces constructor) */
void MMC5983MA_init(SFE_MMC5983MA *dev);

/* Initializes MMC5983MA using I2C */
bool MMC5983MA_beginI2C(SFE_MMC5983MA *dev, I2C_HandleTypeDef *hi2c);

/* Initializes MMC5983MA using SPI */
bool MMC5983MA_beginSPI(SFE_MMC5983MA *dev, GPIO_TypeDef *csPort, uint16_t csPin, SPI_HandleTypeDef *hspi);

/* Sets the error callback function */
void MMC5983MA_setErrorCallback(SFE_MMC5983MA *dev, void (*errorCallback)(SF_MMC5983MA_ERROR errorCode));

/* Convert errorCode to text */
const char *MMC5983MA_errorCodeString(SF_MMC5983MA_ERROR errorCode);

/* ── Connection ──────────────────────────────────────────────────────────── */

/* Polls if MMC5983MA is connected and if chip ID matches MMC5983MA chip id */
bool MMC5983MA_isConnected(SFE_MMC5983MA *dev);

/* ── General ─────────────────────────────────────────────────────────────── */

/* Returns die temperature. Range is -75C to 125C */
int  MMC5983MA_getTemperature(SFE_MMC5983MA *dev);

/* Soft resets the device */
bool MMC5983MA_softReset(SFE_MMC5983MA *dev);

/* ── Interrupt ───────────────────────────────────────────────────────────── */

/* Enables interrupt generation after measurement is completed. Must be re-enabled after each measurement */
bool MMC5983MA_enableInterrupt(SFE_MMC5983MA *dev);

/* Disables interrupt generation */
bool MMC5983MA_disableInterrupt(SFE_MMC5983MA *dev);

/* Checks if interrupt generation is enabled */
bool MMC5983MA_isInterruptEnabled(SFE_MMC5983MA *dev);

/* Clear the Meas_T_Done and/or Meas_M_Done interrupts */
bool MMC5983MA_clearMeasDoneInterrupt(SFE_MMC5983MA *dev, uint8_t measMask);

/* ── SPI ─────────────────────────────────────────────────────────────────── */

/* Enables 3-wire SPI interface */
bool MMC5983MA_enable3WireSPI(SFE_MMC5983MA *dev);

/* Disables 3-wire SPI interface */
bool MMC5983MA_disable3WireSPI(SFE_MMC5983MA *dev);

/* Checks if 3-wire SPI is enabled */
bool MMC5983MA_is3WireSPIEnabled(SFE_MMC5983MA *dev);

/* ── SET / RESET ─────────────────────────────────────────────────────────── */

/* Performs SET operation */
bool MMC5983MA_performSetOperation(SFE_MMC5983MA *dev);

/* Performs RESET operation */
bool MMC5983MA_performResetOperation(SFE_MMC5983MA *dev);

/* Enables automatic SET/RESET */
bool MMC5983MA_enableAutomaticSetReset(SFE_MMC5983MA *dev);

/* Disables automatic SET/RESET */
bool MMC5983MA_disableAutomaticSetReset(SFE_MMC5983MA *dev);

/* Checks if automatic SET/RESET is enabled */
bool MMC5983MA_isAutomaticSetResetEnabled(SFE_MMC5983MA *dev);

/* ── Channel control ─────────────────────────────────────────────────────── */

/* Enables X channel output */
bool MMC5983MA_enableXChannel(SFE_MMC5983MA *dev);

/* Disables X channel output */
bool MMC5983MA_disableXChannel(SFE_MMC5983MA *dev);

/* Returns true when the X channel is inhibited */
bool MMC5983MA_isXChannelEnabled(SFE_MMC5983MA *dev);

/* Enables Y and Z channel outputs */
bool MMC5983MA_enableYZChannels(SFE_MMC5983MA *dev);

/* Disables Y and Z channel outputs */
bool MMC5983MA_disableYZChannels(SFE_MMC5983MA *dev);

/* Returns true when Y and Z channels are inhibited */
bool MMC5983MA_areYZChannelsEnabled(SFE_MMC5983MA *dev);

/* ── Filter bandwidth ────────────────────────────────────────────────────── */

/* Sets decimation filter bandwidth. Allowed values: 800, 400, 200, 100. Defaults to 100 on invalid values */
bool     MMC5983MA_setFilterBandwidth(SFE_MMC5983MA *dev, uint16_t bandwidth);

/* Gets current decimation filter bandwidth in Hz */
uint16_t MMC5983MA_getFilterBandwidth(SFE_MMC5983MA *dev);

/* ── Continuous mode ─────────────────────────────────────────────────────── */

/* Enables continuous mode. Frequency must be greater than 0 */
bool     MMC5983MA_enableContinuousMode(SFE_MMC5983MA *dev);

/* Disables continuous mode */
bool     MMC5983MA_disableContinuousMode(SFE_MMC5983MA *dev);

/* Checks if continuous mode is enabled */
bool     MMC5983MA_isContinuousModeEnabled(SFE_MMC5983MA *dev);

/* Sets continuous mode frequency. Allowed values: 1000, 200, 100, 50, 20, 10, 1, 0 (off) */
bool     MMC5983MA_setContinuousModeFrequency(SFE_MMC5983MA *dev, uint16_t frequency);

/* Gets continuous mode frequency */
uint16_t MMC5983MA_getContinuousModeFrequency(SFE_MMC5983MA *dev);

/* ── Periodic set ────────────────────────────────────────────────────────── */

/* Enables periodic set */
bool     MMC5983MA_enablePeriodicSet(SFE_MMC5983MA *dev);

/* Disables periodic set */
bool     MMC5983MA_disablePeriodicSet(SFE_MMC5983MA *dev);

/* Checks if periodic set is enabled */
bool     MMC5983MA_isPeriodicSetEnabled(SFE_MMC5983MA *dev);

/* Sets how often the chip performs an automatic set operation. Allowed values: 1, 25, 75, 100, 250, 500, 1000, 2000 */
bool     MMC5983MA_setPeriodicSetSamples(SFE_MMC5983MA *dev, uint16_t numberOfSamples);

/* Gets how many times the chip performs an automatic set operation */
uint16_t MMC5983MA_getPeriodicSetSamples(SFE_MMC5983MA *dev);

/* ── Extra coil current ──────────────────────────────────────────────────── */

/* Apply extra current from positive to negative side of coil */
bool MMC5983MA_applyExtraCurrentPosToNeg(SFE_MMC5983MA *dev);

/* Remove extra current from positive to negative side of coil */
bool MMC5983MA_removeExtraCurrentPosToNeg(SFE_MMC5983MA *dev);

/* Checks if extra current is applied from positive to negative side of coil */
bool MMC5983MA_isExtraCurrentAppliedPosToNeg(SFE_MMC5983MA *dev);

/* Apply extra current from negative to positive side of coil */
bool MMC5983MA_applyExtraCurrentNegToPos(SFE_MMC5983MA *dev);

/* Remove extra current from negative to positive side of coil */
bool MMC5983MA_removeExtraCurrentNegToPos(SFE_MMC5983MA *dev);

/* Checks if extra current is applied from negative to positive side of coil */
bool MMC5983MA_isExtraCurrentAppliedNegToPos(SFE_MMC5983MA *dev);

/* ── Measurements ────────────────────────────────────────────────────────── */

/* Get X axis measurement */
uint32_t MMC5983MA_getMeasurementX(SFE_MMC5983MA *dev);

/* Get Y axis measurement */
uint32_t MMC5983MA_getMeasurementY(SFE_MMC5983MA *dev);

/* Get Z axis measurement */
uint32_t MMC5983MA_getMeasurementZ(SFE_MMC5983MA *dev);

/* Get X, Y and Z field strengths in a single measurement */
bool MMC5983MA_getMeasurementXYZ(SFE_MMC5983MA *dev, uint32_t *x, uint32_t *y, uint32_t *z);

/* Read and return the X, Y and Z field strengths */
bool MMC5983MA_readFieldsXYZ(SFE_MMC5983MA *dev, uint32_t *x, uint32_t *y, uint32_t *z);

/* ── Internal (shadow register helpers — used in .c file, not public API) ── */

bool MMC5983MA_setShadowBit(SFE_MMC5983MA *dev, uint8_t registerAddress, uint8_t bitMask, bool doWrite);
bool MMC5983MA_clearShadowBit(SFE_MMC5983MA *dev, uint8_t registerAddress, uint8_t bitMask, bool doWrite);
bool MMC5983MA_isShadowBitSet(SFE_MMC5983MA *dev, uint8_t registerAddress, uint8_t bitMask);
uint16_t MMC5983MA_getTimeout(SFE_MMC5983MA *dev);

#endif /* SPARKFUN_MMC5983MA_H */
