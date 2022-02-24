#include "bsp/gpio/ioExpander.h"
#include <stdbool.h>
#include "utils/data.h"

/******************************
 * private section
 *****************************/

typedef struct
{
    bool init;
    IoExpanderDriver *ioExpanderDriver;
} Context;

static Context driverInstances[NBR_OF_IO_EXPANDER];

static Context *getNewInstance()
{
    Context *ctx = NULL;

    for (uint16_t index = 0; index < DATA_ARRAY_LENGTH(driverInstances); index++)
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

