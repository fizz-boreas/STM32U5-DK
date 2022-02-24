#include "bsp/gpio/ioExpander.h"
#include <stdbool.h>
#include "utils/data.h"
#include "bsp/gpio/pca9574/pca9574Driver.h"


/******************************
 * public section
 *****************************/
bool IoExpandersInit(IoExpanderDriver** drivers, I2C_HandleTypeDef** handles, IoExpanderDriverConf* pinsConf)
{
    bool res = false;
    uint8_t index = 0;

    if(drivers != NULL && handles != NULL && pinsConf != NULL)
    {
        for(uint8_t index = 0; index < NBR_OF_IO_EXPANDER; index++)
        {
            drivers[index]  = pca9574DriverInit(handles[index], pinsConf[index]);
        }
    }
    return res;
}
