/*
  This is a library written for the MMC5983MA High Performance Magnetometer.
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/19034

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, February 2nd, 2022.
  Modified for STM32 HAL compatibility.
  Converted from C++ to C.

  This file implements all functions used in the MMC5983MA High Performance
  Magnetometer STM32 Library.

  SparkFun code, firmware, and software is released under the MIT License
  (http://opensource.org/licenses/MIT). See LICENSE.md for more information.
*/

#include "SparkFun_MMC5983MA_STM32_Library_Constants.h"
#include "SparkFun_MMC5983MA_STM32_Library.h"

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

void MMC5983MA_init(SFE_MMC5983MA *dev)
{
    MMC5983MA_IO_init(&dev->mmc_io);
    dev->errorCallback                    = NULL;
    dev->memoryShadow.internalControl0    = 0x0;
    dev->memoryShadow.internalControl1    = 0x0;
    dev->memoryShadow.internalControl2    = 0x0;
    dev->memoryShadow.internalControl3    = 0x0;
}

/* ── Shadow register helpers ─────────────────────────────────────────────── */

bool MMC5983MA_setShadowBit(SFE_MMC5983MA *dev, uint8_t registerAddress, uint8_t bitMask, bool doWrite)
{
    uint8_t *shadowRegister = NULL;

    switch (registerAddress)
    {
    case INT_CTRL_0_REG: shadowRegister = &dev->memoryShadow.internalControl0; break;
    case INT_CTRL_1_REG: shadowRegister = &dev->memoryShadow.internalControl1; break;
    case INT_CTRL_2_REG: shadowRegister = &dev->memoryShadow.internalControl2; break;
    case INT_CTRL_3_REG: shadowRegister = &dev->memoryShadow.internalControl3; break;
    default: break;
    }

    if (shadowRegister)
    {
        *shadowRegister |= bitMask;
        if (doWrite)
            return MMC5983MA_IO_writeSingleByte(&dev->mmc_io, registerAddress, *shadowRegister);
        return true;
    }

    return false;
}

bool MMC5983MA_clearShadowBit(SFE_MMC5983MA *dev, uint8_t registerAddress, uint8_t bitMask, bool doWrite)
{
    uint8_t *shadowRegister = NULL;

    switch (registerAddress)
    {
    case INT_CTRL_0_REG: shadowRegister = &dev->memoryShadow.internalControl0; break;
    case INT_CTRL_1_REG: shadowRegister = &dev->memoryShadow.internalControl1; break;
    case INT_CTRL_2_REG: shadowRegister = &dev->memoryShadow.internalControl2; break;
    case INT_CTRL_3_REG: shadowRegister = &dev->memoryShadow.internalControl3; break;
    default: break;
    }

    if (shadowRegister)
    {
        *shadowRegister &= (uint8_t)(~bitMask);
        if (doWrite)
            return MMC5983MA_IO_writeSingleByte(&dev->mmc_io, registerAddress, *shadowRegister);
        return true;
    }

    return false;
}

bool MMC5983MA_isShadowBitSet(SFE_MMC5983MA *dev, uint8_t registerAddress, uint8_t bitMask)
{
    switch (registerAddress)
    {
    case INT_CTRL_0_REG: return (bool)(dev->memoryShadow.internalControl0 & bitMask);
    case INT_CTRL_1_REG: return (bool)(dev->memoryShadow.internalControl1 & bitMask);
    case INT_CTRL_2_REG: return (bool)(dev->memoryShadow.internalControl2 & bitMask);
    case INT_CTRL_3_REG: return (bool)(dev->memoryShadow.internalControl3 & bitMask);
    default:             return false;
    }
}

/* ── Error handling ──────────────────────────────────────────────────────── */

void MMC5983MA_setErrorCallback(SFE_MMC5983MA *dev, void (*errorCallback)(SF_MMC5983MA_ERROR errorCode))
{
    dev->errorCallback = errorCallback;
}

const char *MMC5983MA_errorCodeString(SF_MMC5983MA_ERROR errorCode)
{
    switch (errorCode)
    {
    case SF_MMC5983MA_ERROR_NONE:                     return "NONE";
    case SF_MMC5983MA_ERROR_I2C_INITIALIZATION_ERROR: return "I2C_INITIALIZATION_ERROR";
    case SF_MMC5983MA_ERROR_SPI_INITIALIZATION_ERROR: return "SPI_INITIALIZATION_ERROR";
    case SF_MMC5983MA_ERROR_INVALID_DEVICE:           return "INVALID_DEVICE";
    case SF_MMC5983MA_ERROR_BUS_ERROR:                return "BUS_ERROR";
    case SF_MMC5983MA_ERROR_INVALID_FILTER_BANDWIDTH: return "INVALID_FILTER_BANDWIDTH";
    case SF_MMC5983MA_ERROR_INVALID_CONTINUOUS_FREQUENCY: return "INVALID_CONTINUOUS_FREQUENCY";
    case SF_MMC5983MA_ERROR_INVALID_PERIODIC_SAMPLES: return "INVALID_PERIODIC_SAMPLES";
    default:                                          return "UNDEFINED";
    }
}

/* ── Begin / connection ──────────────────────────────────────────────────── */

bool MMC5983MA_beginI2C(SFE_MMC5983MA *dev, I2C_HandleTypeDef *hi2c)
{
    bool success = MMC5983MA_IO_beginI2C(&dev->mmc_io, hi2c);
    if (!success)
    {
        SAFE_CALLBACK(dev->errorCallback, SF_MMC5983MA_ERROR_I2C_INITIALIZATION_ERROR);
        return false;
    }
    return MMC5983MA_isConnected(dev);
}

bool MMC5983MA_beginSPI(SFE_MMC5983MA *dev, GPIO_TypeDef *csPort, uint16_t csPin, SPI_HandleTypeDef *hspi)
{
    bool success = MMC5983MA_IO_beginSPI(&dev->mmc_io, csPort, csPin, hspi);
    if (!success)
    {
        SAFE_CALLBACK(dev->errorCallback, SF_MMC5983MA_ERROR_SPI_INITIALIZATION_ERROR);
        return false;
    }
    return MMC5983MA_isConnected(dev);
}

bool MMC5983MA_isConnected(SFE_MMC5983MA *dev)
{
    uint8_t response = 0;
    bool success = MMC5983MA_IO_readSingleByte(&dev->mmc_io, PROD_ID_REG, &response);

    if (!success)
    {
        SAFE_CALLBACK(dev->errorCallback, SF_MMC5983MA_ERROR_BUS_ERROR);
        return false;
    }
    if (response != PROD_ID)
    {
        SAFE_CALLBACK(dev->errorCallback, SF_MMC5983MA_ERROR_INVALID_DEVICE);
        return false;
    }
    return true;
}

/* ── Temperature ─────────────────────────────────────────────────────────── */

int MMC5983MA_getTemperature(SFE_MMC5983MA *dev)
{
    /* Set TM_T via shadow register to avoid inadvertently setting Auto_SR_en */
    if (!MMC5983MA_setShadowBit(dev, INT_CTRL_0_REG, TM_T, true))
    {
        MMC5983MA_clearShadowBit(dev, INT_CTRL_0_REG, TM_T, false);
        SAFE_CALLBACK(dev->errorCallback, SF_MMC5983MA_ERROR_BUS_ERROR);
        return -99;
    }

    /* Wait for measurement to complete, timeout after 5ms */
    uint8_t timeOut = 5;
    do
    {
        HAL_Delay(1);
        timeOut--;
    } while ((!MMC5983MA_IO_isBitSet(&dev->mmc_io, STATUS_REG, MEAS_T_DONE)) && (timeOut > 0));

    MMC5983MA_clearShadowBit(dev, INT_CTRL_0_REG, TM_T, false);

    uint8_t result = 0;
    if (!MMC5983MA_IO_readSingleByte(&dev->mmc_io, T_OUT_REG, &result))
    {
        SAFE_CALLBACK(dev->errorCallback, SF_MMC5983MA_ERROR_BUS_ERROR);
        return -99;
    }

    /* Convert using the equation from the datasheet */
    float temperature = -75.0f + ((float)result * (200.0f / 255.0f));
    return (int)temperature;
}

/* ── Soft reset ──────────────────────────────────────────────────────────── */

bool MMC5983MA_softReset(SFE_MMC5983MA *dev)
{
    bool success = MMC5983MA_setShadowBit(dev, INT_CTRL_1_REG, SW_RST, true);
    MMC5983MA_clearShadowBit(dev, INT_CTRL_1_REG, SW_RST, false);

    /* The reset time is 10ms; wait 15ms to be safe */
    HAL_Delay(15);

    return success;
}

/* ── Interrupt ───────────────────────────────────────────────────────────── */

bool MMC5983MA_enableInterrupt(SFE_MMC5983MA *dev)
{
    return MMC5983MA_setShadowBit(dev, INT_CTRL_0_REG, INT_MEAS_DONE_EN, true);
}

bool MMC5983MA_disableInterrupt(SFE_MMC5983MA *dev)
{
    return MMC5983MA_clearShadowBit(dev, INT_CTRL_0_REG, INT_MEAS_DONE_EN, true);
}

bool MMC5983MA_isInterruptEnabled(SFE_MMC5983MA *dev)
{
    return MMC5983MA_isShadowBitSet(dev, INT_CTRL_0_REG, INT_MEAS_DONE_EN);
}

bool MMC5983MA_clearMeasDoneInterrupt(SFE_MMC5983MA *dev, uint8_t measMask)
{
    measMask &= (uint8_t)(MEAS_T_DONE | MEAS_M_DONE);
    return MMC5983MA_IO_setRegisterBit(&dev->mmc_io, STATUS_REG, measMask);
}

/* ── 3-wire SPI ──────────────────────────────────────────────────────────── */

bool MMC5983MA_enable3WireSPI(SFE_MMC5983MA *dev)
{
    return MMC5983MA_setShadowBit(dev, INT_CTRL_3_REG, SPI_3W, true);
}

bool MMC5983MA_disable3WireSPI(SFE_MMC5983MA *dev)
{
    return MMC5983MA_clearShadowBit(dev, INT_CTRL_3_REG, SPI_3W, true);
}

bool MMC5983MA_is3WireSPIEnabled(SFE_MMC5983MA *dev)
{
    return MMC5983MA_isShadowBitSet(dev, INT_CTRL_3_REG, SPI_3W);
}

/* ── SET / RESET ─────────────────────────────────────────────────────────── */

bool MMC5983MA_performSetOperation(SFE_MMC5983MA *dev)
{
    bool success = MMC5983MA_setShadowBit(dev, INT_CTRL_0_REG, SET_OPERATION, true);
    MMC5983MA_clearShadowBit(dev, INT_CTRL_0_REG, SET_OPERATION, false);
    HAL_Delay(1); /* Wait 500ns minimum; 1ms HAL tick is sufficient */
    return success;
}

bool MMC5983MA_performResetOperation(SFE_MMC5983MA *dev)
{
    bool success = MMC5983MA_setShadowBit(dev, INT_CTRL_0_REG, RESET_OPERATION, true);
    MMC5983MA_clearShadowBit(dev, INT_CTRL_0_REG, RESET_OPERATION, false);
    HAL_Delay(1);
    return success;
}

bool MMC5983MA_enableAutomaticSetReset(SFE_MMC5983MA *dev)
{
    return MMC5983MA_setShadowBit(dev, INT_CTRL_0_REG, AUTO_SR_EN, true);
}

bool MMC5983MA_disableAutomaticSetReset(SFE_MMC5983MA *dev)
{
    return MMC5983MA_clearShadowBit(dev, INT_CTRL_0_REG, AUTO_SR_EN, true);
}

bool MMC5983MA_isAutomaticSetResetEnabled(SFE_MMC5983MA *dev)
{
    return MMC5983MA_isShadowBitSet(dev, INT_CTRL_0_REG, AUTO_SR_EN);
}

/* ── Channel control ─────────────────────────────────────────────────────── */

bool MMC5983MA_enableXChannel(SFE_MMC5983MA *dev)
{
    /* X_INHIBIT is an inhibit bit — clear it to enable the channel */
    return MMC5983MA_clearShadowBit(dev, INT_CTRL_1_REG, X_INHIBIT, true);
}

bool MMC5983MA_disableXChannel(SFE_MMC5983MA *dev)
{
    return MMC5983MA_setShadowBit(dev, INT_CTRL_1_REG, X_INHIBIT, true);
}

bool MMC5983MA_isXChannelEnabled(SFE_MMC5983MA *dev)
{
    /* Returns true when the X channel is inhibited */
    return MMC5983MA_isShadowBitSet(dev, INT_CTRL_1_REG, X_INHIBIT);
}

bool MMC5983MA_enableYZChannels(SFE_MMC5983MA *dev)
{
    /* YZ_INHIBIT is an inhibit bit — clear it to enable the channels */
    return MMC5983MA_clearShadowBit(dev, INT_CTRL_1_REG, YZ_INHIBIT, true);
}

bool MMC5983MA_disableYZChannels(SFE_MMC5983MA *dev)
{
    return MMC5983MA_setShadowBit(dev, INT_CTRL_1_REG, YZ_INHIBIT, true);
}

bool MMC5983MA_areYZChannelsEnabled(SFE_MMC5983MA *dev)
{
    /* Returns true when the Y and Z channels are inhibited */
    return MMC5983MA_isShadowBitSet(dev, INT_CTRL_1_REG, YZ_INHIBIT);
}

/* ── Filter bandwidth ────────────────────────────────────────────────────── */

bool MMC5983MA_setFilterBandwidth(SFE_MMC5983MA *dev, uint16_t bandwidth)
{
    bool success;

    switch (bandwidth)
    {
    case 800:
        success  = MMC5983MA_setShadowBit(dev,   INT_CTRL_1_REG, BW0, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_1_REG, BW1, true);
        break;
    case 400:
        success  = MMC5983MA_clearShadowBit(dev, INT_CTRL_1_REG, BW0, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_1_REG, BW1, true);
        break;
    case 200:
        success  = MMC5983MA_setShadowBit(dev,   INT_CTRL_1_REG, BW0, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_1_REG, BW1, true);
        break;
    case 100:
        success  = MMC5983MA_clearShadowBit(dev, INT_CTRL_1_REG, BW0, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_1_REG, BW1, true);
        break;
    default:
        SAFE_CALLBACK(dev->errorCallback, SF_MMC5983MA_ERROR_INVALID_FILTER_BANDWIDTH);
        success = false;
        break;
    }

    return success;
}

uint16_t MMC5983MA_getFilterBandwidth(SFE_MMC5983MA *dev)
{
    bool bw0 = MMC5983MA_isShadowBitSet(dev, INT_CTRL_1_REG, BW0);
    bool bw1 = MMC5983MA_isShadowBitSet(dev, INT_CTRL_1_REG, BW1);
    uint8_t value = (uint8_t)((bw1 ? 2u : 0u) + (bw0 ? 1u : 0u));

    switch (value)
    {
    case 1:  return 200;
    case 2:  return 400;
    case 3:  return 800;
    default: return 100;
    }
}

uint16_t MMC5983MA_getTimeout(SFE_MMC5983MA *dev)
{
    uint16_t timeOut = MMC5983MA_getFilterBandwidth(dev); /* 100/200/400/800 Hz */
    timeOut = (uint16_t)(800u / timeOut);                 /* 8/4/2/1 ms        */
    timeOut = (uint16_t)(timeOut * 4u);                   /* 32/16/8/4 ms      */
    timeOut = (uint16_t)(timeOut + 1u);                   /* +1 for 800Hz edge */
    return timeOut;
}

/* ── Continuous mode ─────────────────────────────────────────────────────── */

bool MMC5983MA_enableContinuousMode(SFE_MMC5983MA *dev)
{
    return MMC5983MA_setShadowBit(dev, INT_CTRL_2_REG, CMM_EN, true);
}

bool MMC5983MA_disableContinuousMode(SFE_MMC5983MA *dev)
{
    return MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, CMM_EN, true);
}

bool MMC5983MA_isContinuousModeEnabled(SFE_MMC5983MA *dev)
{
    return MMC5983MA_isShadowBitSet(dev, INT_CTRL_2_REG, CMM_EN);
}

bool MMC5983MA_setContinuousModeFrequency(SFE_MMC5983MA *dev, uint16_t frequency)
{
    bool success;

    switch (frequency)
    {
    case 1:    /* CM_FREQ[2:0] = 001 */
        success  = MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, CM_FREQ_2, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, CM_FREQ_1, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, CM_FREQ_0, true);
        break;
    case 10:   /* CM_FREQ[2:0] = 010 */
        success  = MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, CM_FREQ_2, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, CM_FREQ_1, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, CM_FREQ_0, true);
        break;
    case 20:   /* CM_FREQ[2:0] = 011 */
        success  = MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, CM_FREQ_2, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, CM_FREQ_1, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, CM_FREQ_0, true);
        break;
    case 50:   /* CM_FREQ[2:0] = 100 */
        success  = MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, CM_FREQ_2, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, CM_FREQ_1, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, CM_FREQ_0, true);
        break;
    case 100:  /* CM_FREQ[2:0] = 101 */
        success  = MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, CM_FREQ_2, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, CM_FREQ_1, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, CM_FREQ_0, true);
        break;
    case 200:  /* CM_FREQ[2:0] = 110 */
        success  = MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, CM_FREQ_2, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, CM_FREQ_1, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, CM_FREQ_0, true);
        break;
    case 1000: /* CM_FREQ[2:0] = 111 */
        success  = MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, CM_FREQ_2, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, CM_FREQ_1, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, CM_FREQ_0, true);
        break;
    case 0:    /* CM_FREQ[2:0] = 000 */
        success  = MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, CM_FREQ_2, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, CM_FREQ_1, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, CM_FREQ_0, true);
        break;
    default:
        SAFE_CALLBACK(dev->errorCallback, SF_MMC5983MA_ERROR_INVALID_CONTINUOUS_FREQUENCY);
        success = false;
        break;
    }

    return success;
}

uint16_t MMC5983MA_getContinuousModeFrequency(SFE_MMC5983MA *dev)
{
    uint8_t registerValue = dev->memoryShadow.internalControl2 & 0x07u;

    switch (registerValue)
    {
    case 0x01: return 1;
    case 0x02: return 10;
    case 0x03: return 20;
    case 0x04: return 50;
    case 0x05: return 100;
    case 0x06: return 200;
    case 0x07: return 1000;
    default:   return 0;
    }
}

/* ── Periodic set ────────────────────────────────────────────────────────── */

bool MMC5983MA_enablePeriodicSet(SFE_MMC5983MA *dev)
{
    return MMC5983MA_setShadowBit(dev, INT_CTRL_2_REG, EN_PRD_SET, true);
}

bool MMC5983MA_disablePeriodicSet(SFE_MMC5983MA *dev)
{
    return MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, EN_PRD_SET, true);
}

bool MMC5983MA_isPeriodicSetEnabled(SFE_MMC5983MA *dev)
{
    return MMC5983MA_isShadowBitSet(dev, INT_CTRL_2_REG, EN_PRD_SET);
}

bool MMC5983MA_setPeriodicSetSamples(SFE_MMC5983MA *dev, uint16_t numberOfSamples)
{
    bool success;

    switch (numberOfSamples)
    {
    case 25:   /* PRD_SET[2:0] = 001 */
        success  = MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, PRD_SET_2, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, PRD_SET_1, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, PRD_SET_0, true);
        break;
    case 75:   /* PRD_SET[2:0] = 010 */
        success  = MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, PRD_SET_2, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, PRD_SET_1, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, PRD_SET_0, true);
        break;
    case 100:  /* PRD_SET[2:0] = 011 */
        success  = MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, PRD_SET_2, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, PRD_SET_1, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, PRD_SET_0, true);
        break;
    case 250:  /* PRD_SET[2:0] = 100 */
        success  = MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, PRD_SET_2, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, PRD_SET_1, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, PRD_SET_0, true);
        break;
    case 500:  /* PRD_SET[2:0] = 101 */
        success  = MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, PRD_SET_2, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, PRD_SET_1, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, PRD_SET_0, true);
        break;
    case 1000: /* PRD_SET[2:0] = 110 */
        success  = MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, PRD_SET_2, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, PRD_SET_1, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, PRD_SET_0, true);
        break;
    case 2000: /* PRD_SET[2:0] = 111 */
        success  = MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, PRD_SET_2, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, PRD_SET_1, false);
        success &= MMC5983MA_setShadowBit(dev,   INT_CTRL_2_REG, PRD_SET_0, true);
        break;
    case 1:    /* PRD_SET[2:0] = 000 */
        success  = MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, PRD_SET_2, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, PRD_SET_1, false);
        success &= MMC5983MA_clearShadowBit(dev, INT_CTRL_2_REG, PRD_SET_0, true);
        break;
    default:
        SAFE_CALLBACK(dev->errorCallback, SF_MMC5983MA_ERROR_INVALID_PERIODIC_SAMPLES);
        success = false;
        break;
    }

    return success;
}

uint16_t MMC5983MA_getPeriodicSetSamples(SFE_MMC5983MA *dev)
{
    uint8_t registerValue = dev->memoryShadow.internalControl2 & 0x70u;

    switch (registerValue)
    {
    case 0x10: return 25;
    case 0x20: return 75;
    case 0x30: return 100;
    case 0x40: return 250;
    case 0x50: return 500;
    case 0x60: return 1000;
    case 0x70: return 2000;
    default:   return 1;
    }
}

/* ── Extra coil current ──────────────────────────────────────────────────── */

bool MMC5983MA_applyExtraCurrentPosToNeg(SFE_MMC5983MA *dev)
{
    return MMC5983MA_setShadowBit(dev, INT_CTRL_3_REG, ST_ENP, true);
}

bool MMC5983MA_removeExtraCurrentPosToNeg(SFE_MMC5983MA *dev)
{
    return MMC5983MA_clearShadowBit(dev, INT_CTRL_3_REG, ST_ENP, true);
}

bool MMC5983MA_isExtraCurrentAppliedPosToNeg(SFE_MMC5983MA *dev)
{
    return MMC5983MA_isShadowBitSet(dev, INT_CTRL_3_REG, ST_ENP);
}

bool MMC5983MA_applyExtraCurrentNegToPos(SFE_MMC5983MA *dev)
{
    return MMC5983MA_setShadowBit(dev, INT_CTRL_3_REG, ST_ENM, true);
}

bool MMC5983MA_removeExtraCurrentNegToPos(SFE_MMC5983MA *dev)
{
    return MMC5983MA_clearShadowBit(dev, INT_CTRL_3_REG, ST_ENM, true);
}

bool MMC5983MA_isExtraCurrentAppliedNegToPos(SFE_MMC5983MA *dev)
{
    return MMC5983MA_isShadowBitSet(dev, INT_CTRL_3_REG, ST_ENM);
}

/* ── Measurements ────────────────────────────────────────────────────────── */

uint32_t MMC5983MA_getMeasurementX(SFE_MMC5983MA *dev)
{
    if (!MMC5983MA_setShadowBit(dev, INT_CTRL_0_REG, TM_M, true))
    {
        MMC5983MA_clearShadowBit(dev, INT_CTRL_0_REG, TM_M, false);
        SAFE_CALLBACK(dev->errorCallback, SF_MMC5983MA_ERROR_BUS_ERROR);
        return 0;
    }

    uint16_t timeOut = MMC5983MA_getTimeout(dev);
    do
    {
        HAL_Delay(1);
        timeOut--;
    } while ((!MMC5983MA_IO_isBitSet(&dev->mmc_io, STATUS_REG, MEAS_M_DONE)) && (timeOut > 0));

    MMC5983MA_clearShadowBit(dev, INT_CTRL_0_REG, TM_M, false);

    uint32_t result = 0;
    uint8_t  buffer[2]  = {0};
    uint8_t  buffer2bit = 0;

    MMC5983MA_IO_readMultipleBytes(&dev->mmc_io, X_OUT_0_REG, buffer, 2);
    MMC5983MA_IO_readSingleByte(&dev->mmc_io, XYZ_OUT_2_REG, &buffer2bit);

    result = buffer[0];
    result = (result << 8) | buffer[1];
    result = (result << 2) | (buffer2bit >> 6);

    return result;
}

uint32_t MMC5983MA_getMeasurementY(SFE_MMC5983MA *dev)
{
    if (!MMC5983MA_setShadowBit(dev, INT_CTRL_0_REG, TM_M, true))
    {
        MMC5983MA_clearShadowBit(dev, INT_CTRL_0_REG, TM_M, false);
        SAFE_CALLBACK(dev->errorCallback, SF_MMC5983MA_ERROR_BUS_ERROR);
        return 0;
    }

    uint16_t timeOut = MMC5983MA_getTimeout(dev);
    do
    {
        HAL_Delay(1);
        timeOut--;
    } while ((!MMC5983MA_IO_isBitSet(&dev->mmc_io, STATUS_REG, MEAS_M_DONE)) && (timeOut > 0));

    MMC5983MA_clearShadowBit(dev, INT_CTRL_0_REG, TM_M, false);

    uint32_t result = 0;
    uint8_t  buffer[2]  = {0};
    uint8_t  buffer2bit = 0;

    MMC5983MA_IO_readMultipleBytes(&dev->mmc_io, Y_OUT_0_REG, buffer, 2);
    MMC5983MA_IO_readSingleByte(&dev->mmc_io, XYZ_OUT_2_REG, &buffer2bit);

    result = buffer[0];
    result = (result << 8) | buffer[1];
    result = (result << 2) | ((buffer2bit >> 4) & 0x03u);

    return result;
}

uint32_t MMC5983MA_getMeasurementZ(SFE_MMC5983MA *dev)
{
    if (!MMC5983MA_setShadowBit(dev, INT_CTRL_0_REG, TM_M, true))
    {
        MMC5983MA_clearShadowBit(dev, INT_CTRL_0_REG, TM_M, false);
        SAFE_CALLBACK(dev->errorCallback, SF_MMC5983MA_ERROR_BUS_ERROR);
        return 0;
    }

    uint16_t timeOut = MMC5983MA_getTimeout(dev);
    do
    {
        HAL_Delay(1);
        timeOut--;
    } while ((!MMC5983MA_IO_isBitSet(&dev->mmc_io, STATUS_REG, MEAS_M_DONE)) && (timeOut > 0));

    MMC5983MA_clearShadowBit(dev, INT_CTRL_0_REG, TM_M, false);

    uint32_t result = 0;
    uint8_t  buffer[3] = {0};

    MMC5983MA_IO_readMultipleBytes(&dev->mmc_io, Z_OUT_0_REG, buffer, 3);

    result = buffer[0];
    result = (result << 8) | buffer[1];
    result = (result << 2) | ((buffer[2] >> 2) & 0x03u);

    return result;
}

bool MMC5983MA_getMeasurementXYZ(SFE_MMC5983MA *dev, uint32_t *x, uint32_t *y, uint32_t *z)
{
    bool success = MMC5983MA_setShadowBit(dev, INT_CTRL_0_REG, TM_M, true);

    if (!success)
    {
        MMC5983MA_clearShadowBit(dev, INT_CTRL_0_REG, TM_M, false);
        SAFE_CALLBACK(dev->errorCallback, SF_MMC5983MA_ERROR_BUS_ERROR);
        return false;
    }

    uint16_t timeOut = MMC5983MA_getTimeout(dev);
    do
    {
        HAL_Delay(1);
        timeOut--;
    } while ((!MMC5983MA_IO_isBitSet(&dev->mmc_io, STATUS_REG, MEAS_M_DONE)) && (timeOut > 0));

    MMC5983MA_clearShadowBit(dev, INT_CTRL_0_REG, TM_M, false);

    return (MMC5983MA_readFieldsXYZ(dev, x, y, z) && (timeOut > 0));
}

bool MMC5983MA_readFieldsXYZ(SFE_MMC5983MA *dev, uint32_t *x, uint32_t *y, uint32_t *z)
{
    uint8_t registerValues[7] = {0};

    bool success = MMC5983MA_IO_readMultipleBytes(&dev->mmc_io, X_OUT_0_REG, registerValues, 7);

    if (success)
    {
        *x = registerValues[0];
        *x = (*x << 8) | registerValues[1];
        *x = (*x << 2) | (registerValues[6] >> 6);

        *y = registerValues[2];
        *y = (*y << 8) | registerValues[3];
        *y = (*y << 2) | ((registerValues[6] >> 4) & 0x03u);

        *z = registerValues[4];
        *z = (*z << 8) | registerValues[5];
        *z = (*z << 2) | ((registerValues[6] >> 2) & 0x03u);
    }
    else
    {
        SAFE_CALLBACK(dev->errorCallback, SF_MMC5983MA_ERROR_BUS_ERROR);
    }

    return success;
}
