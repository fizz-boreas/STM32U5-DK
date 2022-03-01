#include "manager/ledManager.h"
#include "main.h"

/*************************************************
    Structures and enums
*************************************************/

typedef enum
{
    LED_TYPE_GPIO,
    LED_TYPE_I2C
}LedType;

typedef enum
{
    RGB_GPIO_R = 0,
    RGB_GPIO_G = 1,
    RGB_GPIO_B = 2
}RGB_GPIO;

typedef struct
{
    // Common for all context
    bool init;
    LedType type;
    uint16_t GPIOs[3];

    // For I2C type
    IoExpanderDriver* ioExpanderDriver;

    // For GPIO type
    GPIO_TypeDef* port;
} Context;

// Context for the LEDManager
static Context driverInstances[LED_QUANTITY];

/*************************************************
    Forward declaration Section
*************************************************/

bool LedWriteColorForGPIO(Context* context, RGBLEDColor color);
bool LedWriteColorForIoExpander(Context* context, RGBLEDColor color);

/*************************************************
    Private Section
*************************************************/

bool LedWriteColorForGPIO(Context* context, RGBLEDColor color)
{
    if(context != NULL && context->type == LED_TYPE_GPIO)
    {
        HAL_GPIO_WritePin(context->port, context->GPIOs[RGB_GPIO_R], color.r);
        HAL_GPIO_WritePin(context->port, context->GPIOs[RGB_GPIO_R], color.g);
        HAL_GPIO_WritePin(context->port, context->GPIOs[RGB_GPIO_B], color.b);
    }else{
        return false;
    }
    return true;
}

bool LedWriteColorForIoExpander(Context* context, RGBLEDColor color)
{
    bool res = false;

    if(context != NULL && context->type == LED_TYPE_GPIO)
    {
        res = context->ioExpanderDriver->writeChannel(context->ioExpanderDriver, context->GPIOs[RGB_GPIO_R],
                                                      color.r == 1 ? GPIOState_Low : GPIOState_High);
        res &= context->ioExpanderDriver->writeChannel(context->ioExpanderDriver, context->GPIOs[RGB_GPIO_G],
                                                      color.g == 1 ? GPIOState_Low : GPIOState_High);
        res &= context->ioExpanderDriver->writeChannel(context->ioExpanderDriver, context->GPIOs[RGB_GPIO_B],
                                                       color.b == 1 ? GPIOState_Low : GPIOState_High);
    }
    return res;
}

/******************************
 * public section
 *****************************/

bool LedManagerInit()
{

    bool res;

    IoExpanderDriverConf pinsConf[2];
    IoExpanderDriver* drivers[2];

    // I2C1 -> driving the io expander for the leds
    pinsConf[0].handle = &hi2c1;
    pinsConf[1].handle = &hi2c1;

    // Driving the LED O0 and O1
    pinsConf[0].busAdr.a0 = I2C_ADR_PIN_GND;
    // Driving the LED O2 and O3
    pinsConf[0].busAdr.a0 = I2C_ADR_PIN_GND;
    res = IoExpandersInit(drivers, pinsConf);

    driverInstances[LED_DO0].ioExpanderDriver = drivers[0];
    driverInstances[LED_DO0].type = LED_TYPE_I2C;
    driverInstances[LED_DO0].GPIOs[RGB_GPIO_R] = 1;
    driverInstances[LED_DO0].GPIOs[RGB_GPIO_G] = 2;
    driverInstances[LED_DO0].GPIOs[RGB_GPIO_B] = 3;

    driverInstances[LED_DO1].ioExpanderDriver = drivers[0];
    driverInstances[LED_DO1].type = LED_TYPE_I2C;
    driverInstances[LED_DO0].GPIOs[RGB_GPIO_R] = 5;
    driverInstances[LED_DO0].GPIOs[RGB_GPIO_G] = 6;
    driverInstances[LED_DO0].GPIOs[RGB_GPIO_B] = 7;

    driverInstances[LED_DO2].ioExpanderDriver = drivers[1];
    driverInstances[LED_DO2].type = LED_TYPE_I2C;
    driverInstances[LED_DO2].GPIOs[RGB_GPIO_R] = 1;
    driverInstances[LED_DO2].GPIOs[RGB_GPIO_G] = 2;
    driverInstances[LED_DO2].GPIOs[RGB_GPIO_B] = 3;

    driverInstances[LED_DO3].ioExpanderDriver = drivers[1];
    driverInstances[LED_DO3].type = LED_TYPE_I2C;
    driverInstances[LED_DO3].GPIOs[RGB_GPIO_R] = 5;
    driverInstances[LED_DO3].GPIOs[RGB_GPIO_G] = 6;
    driverInstances[LED_DO3].GPIOs[RGB_GPIO_B] = 7;

    driverInstances[LED_D1].type = LED_TYPE_GPIO;
    driverInstances[LED_D1].port = GPIOG;
    driverInstances[LED_D1].GPIOs[RGB_GPIO_R] = D1_r_Pin;
    driverInstances[LED_D1].GPIOs[RGB_GPIO_G] = D1_g_Pin;
    driverInstances[LED_D1].GPIOs[RGB_GPIO_B] = D1_b_Pin;

    driverInstances[LED_D2].type = LED_TYPE_GPIO;
    driverInstances[LED_D1].port = GPIOD;
    driverInstances[LED_D2].GPIOs[RGB_GPIO_R] = D1_r_Pin;
    driverInstances[LED_D2].GPIOs[RGB_GPIO_G] = D1_g_Pin;
    driverInstances[LED_D2].GPIOs[RGB_GPIO_B] = D1_b_Pin;
    return res;
}

bool LedManagerSetColor(LED led, RGBLEDColor color)
{
    bool res = false;

    if(led < LED_QUANTITY)
    {
        if(driverInstances[led].type == LED_TYPE_I2C)
        {
            res = LedWriteColorForIoExpander(&driverInstances[led], color);
        }else
            res = LedWriteColorForGPIO(&driverInstances[led], color);
    }
    return res;
}