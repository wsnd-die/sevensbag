#include "HWT101.h"
#include "i2c.h"

#define HWT101_TIMEOUT_MS 100U

HAL_StatusTypeDef HWT101_IsReady(void)
{
    return HAL_I2C_IsDeviceReady(&hi2c1, HWT101_I2C_ADDR, 2, HWT101_TIMEOUT_MS);
}

HAL_StatusTypeDef HWT101_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, HWT101_I2C_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, data, len, HWT101_TIMEOUT_MS);
}

HAL_StatusTypeDef HWT101_WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, HWT101_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &value, 1, HWT101_TIMEOUT_MS);
}
