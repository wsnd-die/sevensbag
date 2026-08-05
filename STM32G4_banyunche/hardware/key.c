#include "Common_used.h"

uint8_t KEY_Read(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
}

uint8_t KEY_IsPressed(void)
{
    return KEY_Read() == GPIO_PIN_RESET;
}
