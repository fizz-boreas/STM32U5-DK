#include "bsp/gpio/ioExpander.h"
#include <stdbool.h>
#include "utils/data.h"
#include "bsp/gpio/pca9574/pca9574Driver.h"


/******************************
 * public section
 *****************************/
bool IoExpandersInit(IoExpanderDriver** drivers, IoExpanderDriverConf* pinsConf)
{
    bool res = false;
    uint8_t index = 0;

    if(drivers != NULL && pinsConf != NULL)
    {
        while(index < NBR_OF_IO_EXPANDER && pinsConf[index].handle != NULL)
        {
            drivers[index]  = pca9574DriverInit(pinsConf[index].handle, pinsConf[index].busAdr);
        }
    }
    return res;
}
