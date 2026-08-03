/**
 * @file    color.c
 * @brief   GY-33 颜色传感器封装：I2C 读写、RGB 读取、颜色判断和投票。
 * @note    硬件接口使用 I2C3，PA8=SCL，PB5=SDA。
 */
#include "color.h"
#include "sw_uart.h"

#if SW_UART_ENABLE
/* 软件串口模式：GY-33 通过 PB6/PB7 UART 通信 */
#else
#include "oled.h"
/* I2C 模式：GY-33 通过 PB6/PB7 软件 I2C 通信 */
#endif

/* RGB 判断阈值，后续可根据现场物块颜色和光照重新标定。 */
#define COLOR_WHITE_THRESHOLD    150U // 白色阈值
#define COLOR_PRIMARY_THRESHOLD  50U  // 主颜色阈值
#define COLOR_LED_LEVEL_MAX      10U  // LED 亮度最大等级

/* 稳定颜色投票参数。 */
#define COLOR_WARMUP_COUNT       3U   // 预热次数，丢弃刚启动时可能不稳定的读数。
#define COLOR_SAMPLE_COUNT       15U  // 总采样次数，前 COLOR_SKIP_COUNT 次不计票。
#define COLOR_SKIP_COUNT         7U   // 跳过前几次采样，避免刚启动时的异常读数。
#define COLOR_ACCEPT_COUNT       6U   // 票数达到阈值才认为颜色稳定。

/* ================================================================
 * I2C 模式（SW_UART_ENABLE == 0）
 * 支持两种芯片：GY-33（内置 MCU）和 TCS34725（裸芯片）
 * ================================================================ */
#if !SW_UART_ENABLE

/*
 * GY-33 寄存器（COLOR_SENSOR_CHIP == 0）
 */
#if COLOR_SENSOR_CHIP == 0
  #define COLOR_REG_START          0x00U
  #define COLOR_REG_CONFIG         0x10U
  #define COLOR_READ_LEN           16U
  #define COLOR_DO_INIT            0U    /* GY-33 无需寄存器初始化 */

/*
 * TCS34725 寄存器（COLOR_SENSOR_CHIP == 1）
 */
#else
  /* TCS34725 寄存器 */
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
  #define TCS34725_ID              0x12U    /* 应读出 0x44 */
  #define TCS34725_STATUS          0x13U
  #define TCS34725_CDATAL          0x14U    /* Clear 低字节 */
  #define TCS34725_CDATAH          0x15U
  #define TCS34725_RDATAL          0x16U    /* Red   低字节 */
  #define TCS34725_RDATAH          0x17U
  #define TCS34725_GDATAL          0x18U    /* Green 低字节 */
  #define TCS34725_GDATAH          0x19U
  #define TCS34725_BDATAL          0x1AU    /* Blue  低字节 */
  #define TCS34725_BDATAH          0x1BU

  #define TCS34725_ENABLE_PON      0x01U    /* Power ON */
  #define TCS34725_ENABLE_AEN      0x02U    /* ADC Enable */
  #define TCS34725_GAIN_1X         0x00U
  #define TCS34725_GAIN_4X         0x01U
  #define TCS34725_GAIN_16X        0x02U
  #define TCS34725_GAIN_60X        0x03U

  /* 积分时间：ATIME = 256 - (time_ms * 1024 / 1000) */
  /* 约 154ms: 256 - 154*1024/1000 = 98 = 0x62 */
  /* 约 24ms:  256 - 24*1024/1000  = 231 = 0xE7 */
  #define TCS34725_ATIME_154MS     0x62U    /* ~154ms，推荐默认值 */
  #define TCS34725_ATIME_24MS      0xE7U    /* ~24ms，快速采样 */

  #define COLOR_READ_LEN           8U       /* CDATA + RDATA + GDATA + BDATA */
  #define COLOR_DO_INIT            1U       /* TCS34725 需要初始化 */
#endif

/* ---- 底层 I2C 读写 ---- */

static HAL_StatusTypeDef Color_ReadRegisters(uint8_t reg, uint8_t *buffer, uint16_t length)
{
    if (buffer == NULL || length == 0U) { return HAL_ERROR; }
    return OLED_SW_I2C_ReadRegs(COLOR_SENSOR_I2C_ADDR, reg, buffer, length);
}

static HAL_StatusTypeDef Color_WriteRegister(uint8_t reg, uint8_t value)
{
    return OLED_SW_I2C_WriteReg(COLOR_SENSOR_I2C_ADDR, reg, value);
}

/* ---- Color_Init ---- */

HAL_StatusTypeDef Color_Init(void)
{
    /* 1. 检查传感器是否在线 */
    if (OLED_SW_I2C_IsDeviceReady(COLOR_SENSOR_I2C_ADDR, 3U) != HAL_OK) {
        return HAL_ERROR;
    }

#if COLOR_SENSOR_CHIP == 1
    /* 2. TCS34725：验证 ID 寄存器 */
    {
        uint8_t id;
        if (Color_ReadRegisters(TCS34725_ID, &id, 1U) != HAL_OK) {
            return HAL_ERROR;
        }
        /* TCS34725 ID 应为 0x44，TCS34721/TCS34723 等变体可能不同 */
        if (id != 0x44U && id != 0x4DU) {
            return HAL_ERROR;
        }
    }

    /* 3. 上电 + 使能 ADC（PON=0x01 | AEN=0x02 = 0x03） */
    if (Color_WriteRegister(TCS34725_ENABLE, TCS34725_ENABLE_PON) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(3U);  /* PON 后需等待 2.4ms */

    if (Color_WriteRegister(TCS34725_ENABLE,
                            TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 4. 设置积分时间（~154ms，颜色精度最高） */
    if (Color_WriteRegister(TCS34725_ATIME, TCS34725_ATIME_154MS) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 5. 设置增益（4x，适合一般环境光） */
    if (Color_WriteRegister(TCS34725_CONTROL, TCS34725_GAIN_4X) != HAL_OK) {
        return HAL_ERROR;
    }
#endif

    return HAL_OK;
}

/* ---- Color_ReadData ---- */

HAL_StatusTypeDef Color_ReadData(Color_DataTypeDef *data)
{
#if COLOR_SENSOR_CHIP == 0
    /* ======== GY-33：读取 16 字节 ======== */
    uint8_t buffer[16U];

    if (data == NULL) { return HAL_ERROR; }

    if (Color_ReadRegisters(0x00U, buffer, sizeof(buffer)) != HAL_OK)
    {
        data->online = 0U;
        return HAL_ERROR;
    }

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
    /* ======== TCS34725：读取 8 字节 RGBC ======== */
    uint8_t buffer[COLOR_READ_LEN];

    if (data == NULL) { return HAL_ERROR; }

    if (Color_ReadRegisters(TCS34725_CDATAL, buffer, sizeof(buffer)) != HAL_OK)
    {
        data->online = 0U;
        return HAL_ERROR;
    }

    data->raw_clear = ((uint16_t)buffer[1] << 8) | buffer[0];  /* C */
    data->raw_red   = ((uint16_t)buffer[3] << 8) | buffer[2];  /* R */
    data->raw_green = ((uint16_t)buffer[5] << 8) | buffer[4];  /* G */
    data->raw_blue  = ((uint16_t)buffer[7] << 8) | buffer[6];  /* B */

    /* 缩放到 8 位（保留高 8 位，与原 GY-33 接口兼容） */
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

/* ---- Color_SetLedLevel（仅 GY-33 支持）---- */

HAL_StatusTypeDef Color_SetLedLevel(uint8_t level)
{
#if COLOR_SENSOR_CHIP == 0
    uint8_t config;

    if (level > COLOR_LED_LEVEL_MAX) { return HAL_ERROR; }

    if (Color_ReadRegisters(COLOR_REG_CONFIG, &config, 1U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    config = (uint8_t)((level << 4) | (config & 0x01U));
    return Color_WriteRegister(COLOR_REG_CONFIG, config);
#else
    /* TCS34725 无补光 LED 控制，忽略 */
    (void)level;
    return HAL_OK;
#endif
}

/* ---- Color_WhiteBalance（仅 GY-33 支持）---- */

HAL_StatusTypeDef Color_WhiteBalance(void)
{
#if COLOR_SENSOR_CHIP == 0
    uint8_t config;
    uint8_t led_config;

    if (Color_ReadRegisters(COLOR_REG_CONFIG, &config, 1U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    led_config = config & 0xF0U;

    if (Color_WriteRegister(COLOR_REG_CONFIG, led_config | 0x01U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(500U);
    return Color_WriteRegister(COLOR_REG_CONFIG, led_config);
#else
    /* TCS34725 无白平衡命令，忽略 */
    return HAL_OK;
#endif
}

/* ================================================================
 * UART 模式（SW_UART_ENABLE == 1）— GY-33 正确协议
 * ================================================================ */
#else

/*
 * GY-33 UART 协议：115200-8N1
 *
 * 命令格式：0xA5 + CMD + CS
 *   CS = (0xA5 + CMD) & 0xFF（低字节）
 *
 * 初始化需发 3 条命令：
 *   0xA5 0xAF CS=0x54 → 连续输出模式
 *   0xA5 0x81 CS=0x26 → MCU 处理后的 RGB 值输出
 *   0xA5 0x68 CS=0x0D → LED 亮度配置
 *
 * 响应帧格式（8 字节，帧头 0x5A）：
 *   [0]=0x5A  [1]=sensor_id  [2]=data_type  [3]=reserved
 *   [4]=R     [5]=G          [6]=B          [7]=checksum
 *   checksum: 所有 8 字节求和，低字节 == 0
 */
#define COLOR_UART_FRAME_LEN     8U
#define COLOR_UART_FRAME_HEADER  0x5AU
#define COLOR_UART_TIMEOUT_MS    300U

/* ---- 辅助：发 GY-33 命令（3 字节帧），CS = (0xA5 + CMD) & 0xFF ---- */
static void Color_SendCmd(uint8_t cmd)
{
    SW_UART_SendByte(0xA5U);
    SW_UART_SendByte(cmd);
    SW_UART_SendByte((uint8_t)((0xA5U + cmd) & 0xFFU));
}

/*
 * 假设帧头对齐，读一帧 8 字节，校验 checksum。
 * 返回 true 表示数据有效。
 */
static bool Color_ReadFrame(uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t buf[COLOR_UART_FRAME_LEN];
    uint8_t sum;

    for (uint8_t i = 0U; i < COLOR_UART_FRAME_LEN; i++) {
        buf[i] = SW_UART_ReadByte();
    }

    /* 校验帧头 */
    if (buf[0] != COLOR_UART_FRAME_HEADER) {
        return false;
    }

    /* 校验 checksum：前 7 字节求和，低字节应等于第 8 字节 */
    sum = 0U;
    for (uint8_t i = 0U; i < COLOR_UART_FRAME_LEN - 1U; i++) {
        sum += buf[i];
    }
    if (sum != buf[COLOR_UART_FRAME_LEN - 1U]) {
        return false;
    }

    *r = buf[4];
    *g = buf[5];
    *b = buf[6];
    return true;
}

/* ---- Color_Init ---- */

HAL_StatusTypeDef Color_Init(void)
{
    uint32_t start;
    uint8_t r, g, b;

    /* 清空旧数据 */
    while (SW_UART_Available() > 0U) {
        (void)SW_UART_ReadByte();
    }

    // SW_UART_SendString("Init GY-33...\r\n");

    HAL_Delay(10U);
    // SW_UART_SendString("  -> continuous mode\r\n");
    Color_SendCmd(0xAFU);
    HAL_Delay(10U);
    // SW_UART_SendString("  -> RGB processed mode\r\n");
    Color_SendCmd(0x81U);
    HAL_Delay(10U);
    // SW_UART_SendString("  -> LED brightness\r\n");
    Color_SendCmd(0x68U);
    HAL_Delay(10U);

    /* 等待第一帧数据 */
    {
        char buf[40];
        start = HAL_GetTick();
        while (SW_UART_Available() < COLOR_UART_FRAME_LEN) {
            if ((HAL_GetTick() - start) > COLOR_UART_TIMEOUT_MS) {
                SW_UART_Printf("RX timeout, avail=%u\r\n", SW_UART_Available());
                return HAL_ERROR;
            }
        }
        SW_UART_Printf("RX got %u bytes\r\n", SW_UART_Available());
    }

    /* 尝试读取并校验一帧 */
    if (!Color_ReadFrame(&r, &g, &b)) {
        SW_UART_SendString("Bad frame\r\n");
        return HAL_ERROR;
    }

    SW_UART_Printf("GY-33 OK! R=%u G=%u B=%u\r\n", r, g, b);
    (void)r; (void)g; (void)b;
    return HAL_OK;
}

/* ---- Color_ReadData ---- */

HAL_StatusTypeDef Color_ReadData(Color_DataTypeDef *data)
{
    uint32_t start;
    uint8_t r = 0U, g = 0U, b = 0U;

    if (data == NULL) { return HAL_ERROR; }

    /*
     * 跳过积压的旧帧，只保留最新一帧。
     * 如果缓冲区里有多帧数据，丢弃前面的，读最后一帧。
     */
    while (SW_UART_Available() >= COLOR_UART_FRAME_LEN * 2U) {
        for (uint8_t i = 0U; i < COLOR_UART_FRAME_LEN; i++) {
            (void)SW_UART_ReadByte();
        }
    }

    /* 寻找帧头 0x5A */
    start = HAL_GetTick();
    for (;;) {
        /* 等至少 1 字节 */
        while (SW_UART_Available() < 1U) {
            if ((HAL_GetTick() - start) > COLOR_UART_TIMEOUT_MS) {
                data->online = 0U;
                return HAL_ERROR;
            }
        }

        /* 读一个字节，找帧头 */
        uint8_t byte = SW_UART_ReadByte();
        if (byte == COLOR_UART_FRAME_HEADER) {
            /* 帧头找到，等剩余 7 字节 */
            /* 把已读的帧头"还回去"的逻辑：Color_ReadFrame 期望从 buf[0] 开始读。
             * 改为：把帧头放到 buf[0]，再读 7 字节 */
            uint8_t buf[COLOR_UART_FRAME_LEN];
            buf[0] = COLOR_UART_FRAME_HEADER;

            start = HAL_GetTick();
            for (uint8_t i = 1U; i < COLOR_UART_FRAME_LEN; i++) {
                while (SW_UART_Available() < 1U) {
                    if ((HAL_GetTick() - start) > COLOR_UART_TIMEOUT_MS) {
                        data->online = 0U;
                        return HAL_ERROR;
                    }
                }
                buf[i] = SW_UART_ReadByte();
            }

            /* 校验 checksum：前 7 字节求和 == 第 8 字节 */
            uint8_t cs = 0U;
            for (uint8_t i = 0U; i < COLOR_UART_FRAME_LEN - 1U; i++) {
                cs += buf[i];
            }
            if (cs != buf[COLOR_UART_FRAME_LEN - 1U]) {
                continue;   /* checksum 错，继续找下一个帧头 */
            }

            r = buf[4];
            g = buf[5];
            b = buf[6];
            break;   /* 有效帧 */
        }
    }

    data->raw_red   = (uint16_t)r;
    data->raw_green = (uint16_t)g;
    data->raw_blue  = (uint16_t)b;
    data->raw_clear = 0U;
    data->red   = r;
    data->green = g;
    data->blue  = b;
    data->lux   = 0U;
    data->color_temperature = 0U;
    data->sensor_color = 0U;
    data->online = 1U;

    return HAL_OK;
}

/* ---- 辅助函数 ---- */

HAL_StatusTypeDef Color_SetLedLevel(uint8_t level)
{
    if (level > COLOR_LED_LEVEL_MAX) { return HAL_ERROR; }
    (void)level;
    /* GY-33 LED 已在 Init 中配置，这里不需要额外操作 */
    return HAL_OK;
}

HAL_StatusTypeDef Color_WhiteBalance(void)
{
    /* GY-33 UART 模式无白平衡命令 */
    return HAL_OK;
}

#endif /* !SW_UART_ENABLE */

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
