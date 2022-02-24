//
// Description: PCA9574 8bit I2C IO Expander
// Created on 2022-02-21
// Copyright (c) 2022 Boreas Technologies All rights reserved.
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

#include <stdbool.h>
#include "utils/data.h"
#include "HAL/i2c.h"
#include "bsp/gpio/pca9574/pca9574Driver.h"

/*************************************************
    Defines
*************************************************/
#define PCA9574_REGISTER_LENGTH_BYTE    (1)
#define PCA9574_TOTAL_CH                (8)
#define PCA9574_TIMEOUT_MS              200

// Address decoding masks
#define PCA9574_BASE_ADR                (0x20)
#define PCA9574_GND_ADR                 (0x0000)
#define PCA9574_VCC_ADR                 (0x0001)

//***********************************************
//		Registers addresses
//***********************************************

#define PCA9574_ADDRESS_EN_INT          (0)
#define PCA9574_ADDRESS_DI_INT          (1)

#define PCA9574_ADDRESS_INPUT_REG           (0x00)
#define PCA9574_ADDRESS_POL_INV_REG         (0x01)
#define PCA9574_ADDRESS_BUS_PULL_REG        (0x02)
#define PCA9574_ADDRESS_RES_REG             (0x03)
#define PCA9574_ADDRESS_CONFIG_REG          (0x04)
#define PCA9574_ADDRESS_OUTPUT_REG          (0x05)
#define PCA9574_ADDRESS_INTERRUPT_REG       (0x06)

#define PCA9574_NBR_OF_REG                  (0x07)

/*************************************************
    Structures
*************************************************/

typedef struct
{
    bool init;
    uint16_t i2cAdr;
    I2C_HandleTypeDef *i2c;
    IoExpanderDriver driver;
} Context;

static Context driverInstances[NBR_OF_PCA9574];

/*************************************************
    Forward declaration Section
*************************************************/

bool pca9574WriteAllChannels(IoExpanderDriver *ctx, const ThermometerCode channels, const GPIOState state);
bool pca9574ReadAllChannels(IoExpanderDriver *ctx, uint16_t *val);
bool pca9574WriteChannel(IoExpanderDriver *ctx, const uint8_t channel, const GPIOState state);
bool pca9574ReadChannel(IoExpanderDriver *ctx, const uint8_t channel, GPIOState *state);
bool pca9574ConfigureChannelDirection(IoExpanderDriver *ctx, const uint8_t channel, const GPIODir direction);
bool pca9574ConfigChannelPolarity(IoExpanderDriver *ctx, const uint8_t channel, const IoExpanderPolarity polarity);
bool pca9574ReadReg(IoExpanderDriver *ctx, const uint8_t regAdr, uint8_t *reg);
bool pca9574SendReg(IoExpanderDriver *ctx, const uint8_t regAdr, const uint8_t regVal);
bool pca9574Free(IoExpanderDriver *driver);

/*************************************************
    Private Section
*************************************************/

static uint8_t dataOut[64];

static void initiateDriver(Context *ctx)
{
    ctx->driver.writeAllChannels = pca9574WriteAllChannels;
    ctx->driver.readAllChannels = pca9574ReadAllChannels;
    ctx->driver.writeChannel = pca9574WriteChannel;
    ctx->driver.readChannel = pca9574ReadChannel;
    ctx->driver.configChDirection = pca9574ConfigureChannelDirection;
    ctx->driver.configChPolarity = pca9574ConfigChannelPolarity;
    ctx->driver.sendRegister = pca9574SendReg;
    ctx->driver.readRegister = pca9574ReadReg;
    ctx->driver.free = pca9574Free;
}

static  Context *getNewInstance()
{
    Context *ctx = NULL;

    for (uint8_t index = 0; index < DATA_ARRAY_LENGTH(driverInstances); index++)
    {
        if (driverInstances[index].init == false)
        {
            ctx = &driverInstances[index];
            driverInstances[index].init = true;
            break;
        }
    }
    return ctx;
}

static bool convertBusAddress(const I2cAdrPins pinValue, uint16_t *i2cAdr)
{
    bool res = false;
    if (i2cAdr != NULL)
    {
        *i2cAdr = I2C_ADR_PIN_INVALID;

        if (pinValue == I2C_ADR_PIN_GND)
            *i2cAdr = PCA9574_GND_ADR;
        else if (pinValue == I2C_ADR_PIN_VCC)
            *i2cAdr = PCA9574_VCC_ADR;

        res = *i2cAdr != I2C_ADR_PIN_INVALID;
    }

    return res;
}

static bool readReg(Context *ctx, const uint8_t regAdr, uint8_t *reg, uint8_t length)
{
    bool res = false;

    if ((ctx != NULL) && ctx->init && (reg != NULL) && (regAdr < PCA9574_NBR_OF_REG) && length != 0)
    {
        uint8_t data = regAdr; // I know :( temp fix
        // Write command byte
        res = HAL_I2C_Master_Transmit(ctx->i2c, ctx->i2cAdr, &data, 1, PCA9574_TIMEOUT_MS) == HAL_OK;

        // Read from -> value is corresponding to content in command byte
        res = res &&
              (HAL_I2C_Master_Receive(ctx->i2c, ctx->i2cAdr, reg, length, PCA9574_TIMEOUT_MS) == HAL_OK);
    }
    return res;
}

static bool writeReg(Context *ctx, const uint8_t regAdr, const uint8_t *reg, const uint8_t length)
{
    bool res = false;

    if ((ctx != NULL) && ctx->init && (reg != NULL) && (regAdr < PCA9574_NBR_OF_REG) && length != 0)
    {
        dataOut[0] = regAdr;

        for (int index = 0; index < length; index++)
        {
            dataOut[index + 1] = reg[index];
        }

        // First byte is always to be command byte
        res = HAL_I2C_Master_Transmit(ctx->i2c, ctx->i2cAdr, dataOut, length + 1, PCA9574_TIMEOUT_MS) == HAL_OK;
    }
    return res;
}

static bool setI2CAddress(Context *ctx, const IoExpanderDriverConf conf)
{
    bool res = false;
    uint16_t i2cTempAdr = I2C_ADR_PIN_INVALID;

    if ((ctx != NULL) && ctx->init)
    {
        // Initialize address with bits [6:1]
        ctx->i2cAdr = PCA9574_BASE_ADR;

        // Set bit [0]
        res = convertBusAddress(conf.busAdr.a0, &i2cTempAdr);
        if (res)
            ctx->i2cAdr |= (i2cTempAdr & MASK_1_BIT);

        // Mask to 7-bits
        if (res)
            ctx->i2cAdr &= I2C_7BIT_ADDRESS_MASK;
    }
    return res;
}

/*************************************************
    Public Section
*************************************************/

IoExpanderDriver *pca9574DriverInit(I2C_HandleTypeDef *i2c, IoExpanderDriverConf conf)
{
    Context *ctx = getNewInstance();
    IoExpanderDriver *driver = NULL;

    if ((ctx != NULL) && (i2c != NULL))
    {
        initiateDriver(ctx);
        ctx->i2cAdr = I2C_ADR_PIN_INVALID;
        ctx->i2c = i2c;

        // Set I2C address
        if (setI2CAddress(ctx, conf))
            driver = &ctx->driver;
    }

    return driver;
}

bool pca9574WriteAllChannels(IoExpanderDriver *ctx, const ThermometerCode channels, const GPIOState state)
{
    bool res = false;

    if (ctx != NULL)
    {
        Context *driver = container_of(ctx, Context, driver);

        if ((driver != NULL) && driver->init && state < GPIOState_Invalid)
        {
            uint8_t regVal = 0;

            if (state == GPIOState_High)
            {
                regVal = (uint8_t) channels;
            }
            else if (state == GPIOState_Low)
            {
                regVal = ~((uint8_t) channels);
            }
            res = writeReg(driver, PCA9574_ADDRESS_OUTPUT_REG, &regVal, 1);
        }
    }

    return res;
}

bool pca9574ReadAllChannels(IoExpanderDriver *ctx, uint16_t *val)
{
    bool res = false;

    if (ctx != NULL)
    {
        Context *driver = container_of(ctx, Context, driver);

        if ((driver != NULL) && driver->init)
        {
            res = readReg(driver, PCA9574_ADDRESS_INPUT_REG, (uint8_t *) val, 1);
        }
    }

    return res;
}

bool pca9574WriteChannel(IoExpanderDriver *ctx, const uint8_t channel, const GPIOState state)
{
    bool res = false;

    if((ctx != NULL) && channel < (PCA9574_TOTAL_CH - 1))
    {
        Context *driver = container_of(ctx, Context, driver);

        if((driver != NULL) && driver->init && state < GPIOState_Invalid)
        {
            uint16_t portState;
            uint8_t regVal;
            uint8_t regAddr;

            res = driver->driver.readAllChannels(&driver->driver, &portState);
            portState &= 0xFF;

            if(state == GPIOState_High)
            {
                regVal = portState | 0x01 << channel;
            }else
                regVal = portState | (~((0x1) << channel)); // Low state

            // Write a single register
            regAddr = PCA9574_ADDRESS_OUTPUT_REG;
            res = res && writeReg(driver, regAddr, &regVal, 1);
        }
    }
    return res;
}

bool pca9574ReadChannel(IoExpanderDriver *ctx, const uint8_t channel, GPIOState *state)
{
    bool res = false;

    if((ctx != NULL) && channel < (PCA9574_TOTAL_CH - 1))
    {
        Context *driver = container_of(ctx, Context, driver);
        uint8_t portState;

        if(driver != NULL && driver->init)
        {
            res = readReg(driver, PCA9574_ADDRESS_INPUT_REG, &portState, 1);
            *state = (portState & ((0x01) << channel)) ? GPIOState_High : GPIOState_Low;
        }
    }
    return res;
}

bool pca9574ConfigureChannelDirection(IoExpanderDriver *ctx, const uint8_t channel, const GPIODir direction)
{
    bool res = false;

    if((ctx != NULL) && channel < (PCA9574_TOTAL_CH - 1))
    {
        Context *driver = container_of(ctx, Context, driver);
        uint8_t portState;
        uint8_t regVal;

        if(driver != NULL && driver->init && direction < GPIODir_Bidirectional)
        {
            res = readReg(driver, PCA9574_ADDRESS_CONFIG_REG, &portState, 1);

            if(direction == GPIODir_Input)
            {
                regVal = portState | ((0x01) << channel);
            }else
                regVal = portState | (~((0x01) << channel)); // Output direction

            res &= writeReg(driver, PCA9574_ADDRESS_CONFIG_REG, &regVal, 1);
        }
    }
    return res;
}

bool pca9574ConfigChannelPolarity(IoExpanderDriver *ctx, const uint8_t channel, const IoExpanderPolarity polarity)
{
    bool res = false;

    if((ctx != NULL) && channel < (PCA9574_TOTAL_CH - 1))
    {
        Context *driver = container_of(ctx, Context, driver);
        uint8_t portState;
        uint8_t regVal;

        if(driver != NULL && driver->init)
        {
            res = readReg(driver, PCA9574_ADDRESS_POL_INV_REG, &portState, 1);

            if(polarity == IoExpanderPolarityInverted)
            {
                regVal = portState | ((0x01) << channel);
            }else
                regVal = portState | (~((0x01) << channel)); // Not inverted

            res &= writeReg(driver, PCA9574_ADDRESS_CONFIG_REG, &regVal, 1);
        }
    }
    return res;
}

bool pca9574ReadReg(IoExpanderDriver *ctx, const uint8_t regAdr, uint8_t *reg)
{
    bool res = false;

    if((ctx != NULL))
    {
        Context *driver = container_of(ctx, Context, driver);
        uint8_t portState;

        if(driver != NULL && driver->init)
        {
            res = readReg(driver, regAdr, &portState, 1);
        }
    }
    return res;
}

bool pca9574SendReg(IoExpanderDriver *ctx, const uint8_t regAdr, const uint8_t regVal)
{
    bool res = false;

    if((ctx != NULL))
    {
        Context *driver = container_of(ctx, Context, driver);

        if(driver != NULL && driver->init)
        {
            res = writeReg(driver, regAdr, &regVal, 1);
        }
    }
    return res;
}

bool pca9574Free(IoExpanderDriver *driver)
{
    bool res = false;

    if (driver != NULL)
    {
        Context *ctx = container_of(driver, Context, driver);
        if(ctx != NULL)
        {
            ctx->init = false;
            res = true;
        }
    }
    return res;
}