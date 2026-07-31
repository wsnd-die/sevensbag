/**
 * @file    color.c
 * @brief   GY-33 颜色传感器封装：I2C 读写、RGB 读取、颜色判断和投票。
 * @note    硬件接口使用 I2C3，PA8=SCL，PB5=SDA。
 */
#include "color.h"
#include "i2c.h"

/* GY-33 连续数据寄存器起始地址和配置寄存器地址。 */
#define COLOR_REG_START          0x00U
#define COLOR_REG_CONFIG         0x10U
#define COLOR_READ_LEN           16U
#define COLOR_I2C_TIMEOUT_MS     100U

/* RGB 判断阈值，后续可根据现场物块颜色和光照重新标定。 */
#define COLOR_WHITE_THRESHOLD    150U // 白色阈值
#define COLOR_PRIMARY_THRESHOLD  50U  // 主颜色阈值
#define COLOR_LED_LEVEL_MAX      10U  // LED 亮度最大等级

/* 稳定颜色投票参数。 */
#define COLOR_WARMUP_COUNT       3U   // 预热次数，丢弃刚启动时可能不稳定的读数。
#define COLOR_SAMPLE_COUNT       15U  // 总采样次数，前 COLOR_SKIP_COUNT 次不计票。
#define COLOR_SKIP_COUNT         7U   // 跳过前几次采样，避免刚启动时的异常读数。
#define COLOR_ACCEPT_COUNT       6U   // 票数达到阈值才认为颜色稳定。

/**
 * @brief  从 GY-33 指定寄存器开始连续读取数据。
 * @param  reg     起始寄存器地址。
 * @param  buffer  接收缓冲区。
 * @param  length  读取长度，单位 byte。
 * @retval HAL_OK / HAL_ERROR
 */
static HAL_StatusTypeDef Color_ReadRegisters(uint8_t reg, uint8_t *buffer, uint16_t length)
{
    if (buffer == NULL || length == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(
        &hi2c3,
        COLOR_SENSOR_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buffer,
        length,
        COLOR_I2C_TIMEOUT_MS);
}

/**
 * @brief  向 GY-33 单个寄存器写入 1 字节。
 * @param  reg    目标寄存器地址。
 * @param  value  写入值。
 * @retval HAL_OK / HAL_ERROR
 */
static HAL_StatusTypeDef Color_WriteRegister(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(
        &hi2c3,
        COLOR_SENSOR_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1U,
        COLOR_I2C_TIMEOUT_MS);
}

/**
 * @brief  检查 GY-33 是否在线。
 * @retval HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef Color_Init(void)
{
    return HAL_I2C_IsDeviceReady(
        &hi2c3,
        COLOR_SENSOR_I2C_ADDR,
        3U,
        COLOR_I2C_TIMEOUT_MS);
}

/**
 * @brief  读取并解析 GY-33 的 16 字节颜色数据。
 * @param  data  输出数据结构体指针。
 * @retval HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef Color_ReadData(Color_DataTypeDef *data)
{
    uint8_t buffer[COLOR_READ_LEN];

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    if (Color_ReadRegisters(COLOR_REG_START, buffer, sizeof(buffer)) != HAL_OK)
    {
        data->online = 0U;
        return HAL_ERROR;
    }

    /* 0x00~0x0F：原始 RGB/Clear、Lux、色温、8 位 RGB、模块颜色位。 */
    data->raw_red = ((uint16_t)buffer[0] << 8) | buffer[1];
    data->raw_green = ((uint16_t)buffer[2] << 8) | buffer[3];
    data->raw_blue = ((uint16_t)buffer[4] << 8) | buffer[5];
    data->raw_clear = ((uint16_t)buffer[6] << 8) | buffer[7];
    data->lux = ((uint16_t)buffer[8] << 8) | buffer[9];
    data->color_temperature = ((uint16_t)buffer[10] << 8) | buffer[11];
    data->red = buffer[12];
    data->green = buffer[13];
    data->blue = buffer[14];
    data->sensor_color = buffer[15];
    data->online = 1U;

    return HAL_OK;
}

/**
 * @brief  设置 GY-33 板载 LED 亮度等级。
 * @param  level  亮度等级，合法范围 0~10。
 * @retval HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef Color_SetLedLevel(uint8_t level)
{
    uint8_t config;

    if (level > COLOR_LED_LEVEL_MAX)
    {
        return HAL_ERROR;
    }

    if (Color_ReadRegisters(COLOR_REG_CONFIG, &config, 1U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 配置寄存器 bit7~bit4 为 LED 等级，bit0 为白平衡控制位。 */
    config = (uint8_t)((level << 4) | (config & 0x01U));
    return Color_WriteRegister(COLOR_REG_CONFIG, config);
}

/**
 * @brief  触发一次 GY-33 白平衡。
 * @retval HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef Color_WhiteBalance(void)
{
    uint8_t config;
    uint8_t led_config;

    if (Color_ReadRegisters(COLOR_REG_CONFIG, &config, 1U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    led_config = config & 0xF0U;

    /* bit0 置 1 开始白平衡，等待后清零，同时保留 LED 等级。 */
    if (Color_WriteRegister(COLOR_REG_CONFIG, led_config | 0x01U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(500U);
    return Color_WriteRegister(COLOR_REG_CONFIG, led_config);
}

/**
 * @brief  根据单帧 RGB 数据判断当前颜色。
 * @param  data  已成功读取的颜色数据。
 * @return Color_TypeDef  颜色枚举。
 * @note   判断顺序参考 Arduino 版本：白色优先，其次按最大 RGB 分量判断蓝/红/绿，否则黑色。
 */
Color_TypeDef Color_Judge(const Color_DataTypeDef *data)
{
    uint8_t r;
    uint8_t g;
    uint8_t b;

    if (data == NULL || data->online == 0U)
    {
        return COLOR_UNKNOWN;
    }

    r = data->red;
    g = data->green;
    b = data->blue;

    /* 三通道都足够亮时优先判定为白色。 */
    if (r >= COLOR_WHITE_THRESHOLD &&
        g >= COLOR_WHITE_THRESHOLD &&
        b >= COLOR_WHITE_THRESHOLD)
    {
        return COLOR_WHITE;
    }

    /* 主分量超过阈值且不小于其他分量时，判定为对应颜色。 */
    if (b >= r && b >= g && b >= COLOR_PRIMARY_THRESHOLD)
    {
        return COLOR_BLUE;
    }

    if (r >= g && r >= b && r >= COLOR_PRIMARY_THRESHOLD)
    {
        return COLOR_RED;
    }

    if (g >= r && g >= b && g >= COLOR_PRIMARY_THRESHOLD)
    {
        return COLOR_GREEN;
    }

    return COLOR_BLACK;
}

/**
 * @brief  多次采样并统计出现次数最多的颜色。
 * @return Color_TypeDef  稳定颜色；最高票数不足时返回 COLOR_UNKNOWN。
 */
Color_TypeDef Color_DetectDominant(void)
{
    uint8_t counts[COLOR_COUNT] = {0U};
    uint8_t max_count = 0U;
    Color_TypeDef dominant = COLOR_UNKNOWN;

    /* 预热传感器，丢弃刚启动时可能不稳定的读数。 */
    for (uint8_t i = 0U; i < COLOR_WARMUP_COUNT; i++)
    {
        (void)Color_Init();
        HAL_Delay(3U);
    }

    /* 前 COLOR_SKIP_COUNT 次不计票，后续有效颜色进入统计。 */
    for (uint8_t i = 0U; i < COLOR_SAMPLE_COUNT; i++)
    {
        Color_DataTypeDef data;
        Color_TypeDef color;

        if (Color_ReadData(&data) == HAL_OK)
        {
            color = Color_Judge(&data);
            if (i >= COLOR_SKIP_COUNT && color > COLOR_UNKNOWN && color < COLOR_COUNT)
            {
                counts[color]++;
            }
        }

        HAL_Delay(5U);
    }

    /* 找出得票最多的颜色。 */
    for (Color_TypeDef color = COLOR_RED; color < COLOR_COUNT; color++)
    {
        if (counts[color] > max_count)
        {
            max_count = counts[color];
            dominant = color;
        }
    }

    /* 票数达到阈值才认为颜色稳定。 */
    if (max_count >= COLOR_ACCEPT_COUNT)
    {
        return dominant;
    }

    return COLOR_UNKNOWN;
}

/**
 * @brief  将颜色枚举转换成便于串口打印的字符串。
 * @param  color  颜色枚举。
 * @return const char*  颜色名称。
 */
const char *Color_ToString(Color_TypeDef color)
{
    switch (color)
    {
    case COLOR_RED:
        return "RED";
    case COLOR_GREEN:
        return "GREEN";
    case COLOR_BLUE:
        return "BLUE";
    case COLOR_WHITE:
        return "WHITE";
    case COLOR_BLACK:
        return "BLACK";
    default:
        return "UNKNOWN";
    }
}
