#include "manager/flashManager.h"

/*************************************************
    Defines
*************************************************/
#define FLASH_TIMEOUT_MS              200

//***********************************************
//		Instructions
//***********************************************

#define FLASH_READ_MANUFACTURER_DEVICE_ID 0x90

/*************************************************
    Structures and enums
*************************************************/

static struct
{
    bool init;

    SPI_HandleTypeDef* handle;

    // HOLD data
    uint16_t GPIOHOLD;
    GPIO_TypeDef* portHOLD;
} instance;

static uint8_t dataOut[64];

/*************************************************
    Forward declaration Section
*************************************************/

bool flashWriteInstruction(uint8_t instruction, uint8_t* data, uint8_t length);
bool flashReadData(uint8_t* data, uint8_t length);

/*************************************************
    Private Section
*************************************************/

bool flashWriteInstruction(uint8_t instruction, uint8_t* data, uint8_t length)
{
    dataOut[0] = instruction;

    for(uint8_t i = 0; i < length; i++)
    {
        dataOut[i + 1] = data[i];
    }

    return HAL_SPI_Transmit(instance.handle, dataOut, length + 1, FLASH_TIMEOUT_MS) == HAL_OK;
}

bool flashReadData(uint8_t* data, uint8_t length)
{
    return HAL_SPI_Receive(instance.handle, data, length, FLASH_TIMEOUT_MS) == HAL_OK;
}

/******************************
 * public section
 *****************************/

bool flashManagerInit()
{
    instance.handle = &hspi2;
    instance.GPIOHOLD = HOLD_FLASH_Pin;
    instance.portHOLD = HOLD_FLASH_GPIO_Port;

    // /HOLD flash
    HAL_GPIO_WritePin(instance.portHOLD, instance.GPIOHOLD, 0);
    HAL_Delay(50);

    // Get the manufacturer ID and the Device ID
    uint8_t payload[3] = {0, 0, 0};
    if(flashWriteInstruction(FLASH_READ_MANUFACTURER_DEVICE_ID, payload, 3))
    {
        if(flashReadData(payload, 3))
        {
            instance.init = payload[1] == 0xEF;
            return instance.init;
        }
    }
    return false;
}