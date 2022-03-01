//
// Description: Common config for TCAXXXX I2C IO expanders
// Created on 2020-05-11
// Copyright (c) 2020 Boreas Technologies All rights reserved.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#ifndef DKCORE_IOEXPANDER_H
#define DKCORE_IOEXPANDER_H

#include "HAL/i2c.h"
#include "gpio.h"
#include "main.h"

typedef enum
{
    IoExpanderPolarityInverted = (0x1),
    IoExpanderPolarityNotInverted = (0x0),
} IoExpanderPolarity;

typedef enum
{
    IoExpanderDirectionInput = (0x1),
    IoExpanderDirectionOutput = (0x0),
} IoExpanderDirection;

typedef enum
{
    ThermometerCode_Nothing = 0x0,
    ThermometerCode0 = 0x1,
    ThermometerCode1 = 0x1 << 1,
    ThermometerCode2 = 0x1 << 2,
    ThermometerCode3 = 0x1 << 3,
    ThermometerCode4 = 0x1 << 4,
    ThermometerCode5 = 0x1 << 5,
    ThermometerCode6 = 0x1 << 6,
    ThermometerCode7 = 0x1 << 7,
    ThermometerCode8 = 0x1 << 8,
    ThermometerCode9 = 0x1 << 9,
    ThermometerCode10 = 0x1 << 10,
    ThermometerCode11 = 0x1 << 11,
    ThermometerCode12 = 0x1 << 12,
    ThermometerCode13 = 0x1 << 13,
    ThermometerCode14 = 0x1 << 14,
    ThermometerCode15 = 0x1 << 15,
    ThermometerCode_All = 0xFFFF
} ThermometerCode;

typedef struct
{
    I2cAdrPins a2;
    I2cAdrPins a1;
    I2cAdrPins a0;
} I2cBusAdr;

typedef struct _IoExpanderDriver IoExpanderDriver;

/**
 * @brief Write all selected channels at once
 * @param ctx Driver context
 * @param channels Channels to write as thermometer coded channels
 * @param state GPIOState High or Low
 * @return pass/fail
 */
typedef bool (*IoExpanderWriteAllChannels)(IoExpanderDriver *ctx, ThermometerCode channels, GPIOState on);

/**
 * @brief Read all channels at once. The value is stored in the register structure under INPUT register
 * @param ctx Driver context
 * @param val Return value passed as a parameter of all ports on device
 * @return pass/fail
 */
typedef bool (*IoExpanderReadAllChannels)(IoExpanderDriver *ctx, uint16_t *val);

/**
 * @brief Write one channel with desired state
 * @param ctx Driver context
 * @param channel Channel as a number (0-7)
 * @param state GPIOState High or Low
 * @return pass/fail
 */
typedef bool (*IoExpanderWriteChannel)(IoExpanderDriver *ctx, uint8_t channel, GPIOState state);

/**
 * @brief Read specified channel. Value returned by input parameter *state
 * @param ctx Driver context
 * @param channel Channel as a number (0-7)
 * @param state State reading for channel (acts as an output value)
 * @return pass/fail
 */
typedef bool (*IoExpanderReadChannel)(IoExpanderDriver *ctx, uint8_t channel, GPIOState *state);

/**
 * @brief configure device register according to desired direction of GPIO
 * @param ctx Driver context
 * @param channel Channel number
 * @param direction GPIO direction
 * @return pass/fail
 */
typedef bool (*IoExpanderConfigureChannelDirection)(IoExpanderDriver *ctx, uint8_t channel, GPIODir direction);

/**
 * @brief configure device register according to desired direction of GPIO
 * @param ctx Driver context
 * @param channel Channel number
 * @param polarity TCA polarity
 * @return pass/fail
 */
typedef bool (*IoExpanderConfigChannelPolarity)(IoExpanderDriver *ctx, uint8_t channel, IoExpanderPolarity polarity);

/**
 * @brief Reads the desired register from device
 * @param ctx Driver Context
 * @param regAdr Register address
 * @param reg Value read of the register (acts as an output value)
 * @return pass/fail
 */
typedef bool (*IoExpanderReadReg)(IoExpanderDriver *ctx, uint8_t regAdr, uint8_t *reg);

/**
 * @brief Writes value to desired register
 * @param ctx Driver context
 * @param regAdr Register address
 * @param regValue Value to write
 * @return pass/fail
 */
typedef bool (*IoExpanderSendReg)(IoExpanderDriver *ctx, uint8_t regAdr, uint8_t regVal);

/**
 * @brief Un-initialize the driver
 * @param driver pointer
 * @return pass/fail
 */
typedef bool (*IoExpanderFree)(IoExpanderDriver *driver);

// function pointers of the driver
struct _IoExpanderDriver
{
    IoExpanderWriteAllChannels writeAllChannels;
    IoExpanderReadAllChannels readAllChannels;
    IoExpanderWriteChannel writeChannel;
    IoExpanderReadChannel readChannel;
    IoExpanderConfigureChannelDirection configChDirection;
    IoExpanderConfigChannelPolarity configChPolarity;
    IoExpanderSendReg sendRegister;
    IoExpanderReadReg readRegister;
    IoExpanderFree free;
};

// init structure
typedef struct
{
    I2C_HandleTypeDef* handle;
    I2cBusAdr busAdr;
}IoExpanderDriverConf;

/******************************
 * public prototypes
 *****************************/

bool IoExpandersInit(IoExpanderDriver** drivers, IoExpanderDriverConf* pinsConf);

#endif //DKCORE_IOEXPANDER_H
