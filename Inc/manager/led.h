#include <stdbool.h>

typedef enum
{
    LED_DO0 = 0,
    LED_DO1 = 1,
    LED_DO2 = 2,
    LED_DO3 = 3,
    LED_D1 = 4,
    LED_D2 = 5,
    LED_QUANTITY = 6
}LED;

typedef union
{
    struct
    {
        uint8_t r : 1;
        uint8_t g : 1;
        uint8_t b: 1;
    };
    uint8_t raw;
}RGBLEDColor;
