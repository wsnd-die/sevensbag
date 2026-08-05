/**
 * @file    color.c
 * @brief   GY-33 颜色传感器封装：I2C 读写、RGB 读取、颜色判断和投票。
 * @note    硬件接口使用 I2C3，PA8=SCL，PB5=SDA。
 */
#include "Common_used.h"

#if SW_UART_ENABLE
/* 软件串口模式：GY-33 通过 PB6/PB7 UART 通信 */
#else
/* I2C 模式：GY-33 通过 PB6/PB7 软件 I2C 通信 */
#endif

/* RGB 判断阈值 */
#define COLOR_WHITE_THRESHOLD    150U
#define COLOR_PRIMARY_THRESHOLD  50U
#define COLOR_LED_LEVEL_MAX      10U

/* 投票参数 */
#define COLOR_WARMUP_COUNT       1U   /* 预热次数 */
#define COLOR_SAMPLE_COUNT       8U   /* 采样次数 */
#define COLOR_SKIP_COUNT         2U   /* 跳过前 N 次 */
#define COLOR_ACCEPT_COUNT       3U   /* 票数阈值 */

/* ================================================================
 * I2C 模式（SW_UART_ENABLE == 0）
 * ================================================================ */
#if !SW_UART_ENABLE

#if COLOR_SENSOR_CHIP == 0
  #define COLOR_REG_START          0x00U
  #define COLOR_REG_CONFIG         0x10U
  #define COLOR_READ_LEN           16U
  #define COLOR_DO_INIT            0U
#else
  #define TCS34725_ENABLE          0x00U
  #define TCS34725_ATIME           0x01U
  #define TCS34725_WTIME           0x03U
  #define TCS34725_AILTL           0x04U
  #define TCS34725_AILTH           0x05U
  #define TCS34725_AIHTL           0x06U
  #define TCS34725_AIHTH           0x07U
  #define TCS34725_PERS            0x0CU
  #define TCS34725_CONFIG          0x0DU
  #define TCS34725_CONTROL         0x0FU
  #define TCS34725_ID              0x12U
  #define TCS34725_STATUS          0x13U
  #define TCS34725_CDATAL          0x14U
  #define TCS34725_CDATAH          0x15U
  #define TCS34725_RDATAL          0x16U
  #define TCS34725_RDATAH          0x17U
  #define TCS34725_GDATAL          0x18U
  #define TCS34725_GDATAH          0x19U
  #define TCS34725_BDATAL          0x1AU
  #define TCS34725_BDATAH          0x1BU
  #define TCS34725_ENABLE_PON      0x01U
  #define TCS34725_ENABLE_AEN      0x02U
  #define TCS34725_GAIN_1X         0x00U
  #define TCS34725_GAIN_4X         0x01U
  #define TCS34725_GAIN_16X        0x02U
  #define TCS34725_GAIN_60X        0x03U
  #define TCS34725_ATIME_154MS     0x62U
  #define TCS34725_ATIME_24MS      0xE7U
  #define COLOR_READ_LEN           8U
  #define COLOR_DO_INIT            1U
#endif

static HAL_StatusTypeDef Color_ReadRegisters(uint8_t reg, uint8_t *buffer, uint16_t length)
{
    if (buffer == NULL || length == 0U) { return HAL_ERROR; }
    return OLED_SW_I2C_ReadRegs(COLOR_SENSOR_I2C_ADDR, reg, buffer, length);
}

static HAL_StatusTypeDef Color_WriteRegister(uint8_t reg, uint8_t value)
{
    return OLED_SW_I2C_WriteReg(COLOR_SENSOR_I2C_ADDR, reg, value);
}

HAL_StatusTypeDef Color_Init(void)
{
    if (OLED_SW_I2C_IsDeviceReady(COLOR_SENSOR_I2C_ADDR, 3U) != HAL_OK)
        return HAL_ERROR;
#if COLOR_SENSOR_CHIP == 1
    {
        uint8_t id;
        if (Color_ReadRegisters(TCS34725_ID, &id, 1U) != HAL_OK) return HAL_ERROR;
        if (id != 0x44U && id != 0x4DU) return HAL_ERROR;
    }
    if (Color_WriteRegister(TCS34725_ENABLE, TCS34725_ENABLE_PON) != HAL_OK) return HAL_ERROR;
    HAL_Delay(3U);
    if (Color_WriteRegister(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN) != HAL_OK) return HAL_ERROR;
    if (Color_WriteRegister(TCS34725_ATIME, TCS34725_ATIME_154MS) != HAL_OK) return HAL_ERROR;
    if (Color_WriteRegister(TCS34725_CONTROL, TCS34725_GAIN_4X) != HAL_OK) return HAL_ERROR;
#endif
    return HAL_OK;
}

HAL_StatusTypeDef Color_ReadData(Color_DataTypeDef *data)
{
#if COLOR_SENSOR_CHIP == 0
    uint8_t buffer[16U];
    if (data == NULL) { return HAL_ERROR; }
    if (Color_ReadRegisters(0x00U, buffer, sizeof(buffer)) != HAL_OK)
    { data->online = 0U; return HAL_ERROR; }
    data->raw_red   = ((uint16_t)buffer[0] << 8) | buffer[1];
    data->raw_green = ((uint16_t)buffer[2] << 8) | buffer[3];
    data->raw_blue  = ((uint16_t)buffer[4] << 8) | buffer[5];
    data->raw_clear = ((uint16_t)buffer[6] << 8) | buffer[7];
    data->lux       = ((uint16_t)buffer[8] << 8) | buffer[9];
    data->color_temperature = ((uint16_t)buffer[10] << 8) | buffer[11];
    data->red   = buffer[12];
    data->green = buffer[13];
    data->blue  = buffer[14];
    data->sensor_color = buffer[15];
    data->online = 1U;
#else
    uint8_t buffer[COLOR_READ_LEN];
    if (data == NULL) { return HAL_ERROR; }
    if (Color_ReadRegisters(TCS34725_CDATAL, buffer, sizeof(buffer)) != HAL_OK)
    { data->online = 0U; return HAL_ERROR; }
    data->raw_clear = ((uint16_t)buffer[1] << 8) | buffer[0];
    data->raw_red   = ((uint16_t)buffer[3] << 8) | buffer[2];
    data->raw_green = ((uint16_t)buffer[5] << 8) | buffer[4];
    data->raw_blue  = ((uint16_t)buffer[7] << 8) | buffer[6];
    data->red   = (uint8_t)(data->raw_red   >> 8);
    data->green = (uint8_t)(data->raw_green >> 8);
    data->blue  = (uint8_t)(data->raw_blue  >> 8);
    data->lux   = 0U;
    data->color_temperature = 0U;
    data->sensor_color = 0U;
    data->online = 1U;
#endif
    return HAL_OK;
}

HAL_StatusTypeDef Color_SetLedLevel(uint8_t level)
{
#if COLOR_SENSOR_CHIP == 0
    uint8_t config;
    if (level > COLOR_LED_LEVEL_MAX) { return HAL_ERROR; }
    if (Color_ReadRegisters(COLOR_REG_CONFIG, &config, 1U) != HAL_OK) return HAL_ERROR;
    config = (uint8_t)((level << 4) | (config & 0x01U));
    return Color_WriteRegister(COLOR_REG_CONFIG, config);
#else
    (void)level;
    return HAL_OK;
#endif
}

HAL_StatusTypeDef Color_WhiteBalance(void)
{
#if COLOR_SENSOR_CHIP == 0
    uint8_t config, led_config;
    if (Color_ReadRegisters(COLOR_REG_CONFIG, &config, 1U) != HAL_OK) return HAL_ERROR;
    led_config = config & 0xF0U;
    if (Color_WriteRegister(COLOR_REG_CONFIG, led_config | 0x01U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(500U);
    return Color_WriteRegister(COLOR_REG_CONFIG, led_config);
#else
    return HAL_OK;
#endif
}

/* ================================================================
 * UART 模式（SW_UART_ENABLE == 1）
 * GY-33 帧格式: 5A 5A type qty data[qty] chk
 * chk = (5A+5A+type+qty+data[0..qty-1]) & 0xFF
 * ================================================================ */
#else

/* 发 GY-33 命令: A5 CMD SUM */
static void Color_SendCmd(uint8_t cmd)
{
    SW_UART_SendByte(0xA5U);
    SW_UART_SendByte(cmd);
    SW_UART_SendByte((uint8_t)((0xA5U + cmd) & 0xFFU));
}

/* 读一帧: 同步 5A 5A → type+qty → data → chk */
static bool Color_ReadFrame(uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t buf[10], chk, sum;

    /* 同步到双帧头 */
    uint8_t last = 0, cur = 0;
    while (!(last == 0x5A && cur == 0x5A)) {
        last = cur;
        cur = SW_UART_ReadByte();
    }

    uint8_t dtype = SW_UART_ReadByte();
    uint8_t qty   = SW_UART_ReadByte();

    for (uint8_t i = 0; i < qty && i < 8; i++) buf[i] = SW_UART_ReadByte();
    chk = SW_UART_ReadByte();

    /* checksum */
    sum = 0x5A + 0x5A + dtype + qty;
    for (uint8_t i = 0; i < qty; i++) sum += buf[i];
    if ((sum & 0xFF) != chk) return false;

    if (dtype == 0x45 && qty == 3) {
        *r = buf[0]; *g = buf[1]; *b = buf[2];
        return true;
    }
    return false;
}

HAL_StatusTypeDef Color_Init(void)
{
    Color_SendCmd(0x81U);  /* 连续输出处理后 8bit RGB */
    HAL_Delay(50U);
    return HAL_OK;
}

HAL_StatusTypeDef Color_ReadData(Color_DataTypeDef *data)
{
    uint8_t r = 0, g = 0, b = 0;

    if (data == NULL) return HAL_ERROR;

    if (!Color_ReadFrame(&r, &g, &b)) {
        data->online = 0U;
        return HAL_ERROR;
    }

    data->raw_red   = (uint16_t)r;
    data->raw_green = (uint16_t)g;
    data->raw_blue  = (uint16_t)b;
    data->red   = r;
    data->green = g;
    data->blue  = b;
    data->online = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef Color_SetLedLevel(uint8_t level)
{
    if (level > COLOR_LED_LEVEL_MAX) return HAL_ERROR;
    /* A5 6X SUM: X=0(亮)~10(暗) */
    SW_UART_SendByte(0xA5U);
    SW_UART_SendByte(0x60U | level);
    SW_UART_SendByte((uint8_t)((0xA5U + (0x60U | level)) & 0xFFU));
    return HAL_OK;
}

HAL_StatusTypeDef Color_WhiteBalance(void)
{
    return HAL_OK;
}

#endif /* !SW_UART_ENABLE */

Color_TypeDef Color_Judge(const Color_DataTypeDef *data)
{
    if (data == NULL || data->online == 0U) return COLOR_UNKNOWN;

    uint8_t r = data->red, g = data->green, b = data->blue;

    if (r >= COLOR_WHITE_THRESHOLD && g >= COLOR_WHITE_THRESHOLD && b >= COLOR_WHITE_THRESHOLD)
        return COLOR_WHITE;
    if (b >= r && b >= g && b >= COLOR_PRIMARY_THRESHOLD) return COLOR_BLUE;
    if (r >= g && r >= b && r >= COLOR_PRIMARY_THRESHOLD) return COLOR_RED;
    if (g >= r && g >= b && g >= COLOR_PRIMARY_THRESHOLD) return COLOR_GREEN;
    return COLOR_BLACK;
}

Color_TypeDef Color_DetectDominant(void)
{
    uint8_t counts[COLOR_COUNT] = {0U};
    uint8_t max_count = 0U;
    Color_TypeDef dominant = COLOR_UNKNOWN;

    for (uint8_t i = 0U; i < COLOR_WARMUP_COUNT; i++) {
        (void)Color_Init();
        HAL_Delay(3U);
    }

    for (uint8_t i = 0U; i < COLOR_SAMPLE_COUNT; i++) {
        Color_DataTypeDef data;
        if (Color_ReadData(&data) == HAL_OK) {
            Color_TypeDef color = Color_Judge(&data);
            if (i >= COLOR_SKIP_COUNT && color > COLOR_UNKNOWN && color < COLOR_COUNT)
                counts[color]++;
        }
        HAL_Delay(5U);
    }

    for (Color_TypeDef c = COLOR_RED; c < COLOR_COUNT; c++) {
        if (counts[c] > max_count) { max_count = counts[c]; dominant = c; }
    }

    return (max_count >= COLOR_ACCEPT_COUNT) ? dominant : COLOR_UNKNOWN;
}

const char *Color_ToString(Color_TypeDef color)
{
    switch (color) {
    case COLOR_RED:   return "RED";
    case COLOR_GREEN: return "GREEN";
    case COLOR_BLUE:  return "BLUE";
    case COLOR_WHITE: return "WHITE";
    case COLOR_BLACK: return "BLACK";
    default:          return "UNKNOWN";
    }
}
