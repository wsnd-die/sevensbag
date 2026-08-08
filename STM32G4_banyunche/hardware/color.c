/**
 * @file color.c
 * @brief 颜色传感器 — GY-33 / OpenMV 双驱动
 *   - USE_OPENMV_COLOR 0: GY-33,  帧: 5A 5A type qty data[qty] chk
 *   - USE_OPENMV_COLOR 1: OpenMV, 帧: AA CC [color] BB DD  (color=Color_TypeDef枚举)
 */
#include "Common_used.h"
#include <stdlib.h>  /* abs() */

/* ---- 内部参数 ---- */
#define CALIB_TOLERANCE   30U
#define WARMUP_COUNT      1U
#define SAMPLE_COUNT      5U
#define SKIP_COUNT        1U
#define ACCEPT_COUNT      2U

/* ---- 校准数据 (全局) ---- */
Color_Calib_t  g_color_calib[COLOR_COUNT];
Color_Ambient_t g_color_ambient;

/* ================================================================
 * 协议层 — GY-33
 * ================================================================ */
#if USE_OPENMV_COLOR == 1

static void Color_SendCmd(uint8_t cmd)
{
    SW_UART_SendByte(0xA5U);
    SW_UART_SendByte(cmd);
    SW_UART_SendByte((uint8_t)((0xA5U + cmd) & 0xFFU));
}

static bool Color_ReadFrame(uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t buf[10], chk, sum;
    uint8_t last = 0, cur = 0;
    int retry = 500;
    while (!(last == 0x5A && cur == 0x5A) && --retry > 0) {
        last = cur;
        cur = SW_UART_ReadByte();
    }
    if (retry <= 0) return false;

    uint8_t dtype = SW_UART_ReadByte();
    uint8_t qty   = SW_UART_ReadByte();
    for (uint8_t i = 0; i < qty && i < 8; i++) buf[i] = SW_UART_ReadByte();
    chk = SW_UART_ReadByte();

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
    Color_SendCmd(0x81U);
    HAL_Delay(50U);
    return HAL_OK;
}

HAL_StatusTypeDef Color_SetLedLevel(uint8_t level)
{
    if (level > 10U) return HAL_ERROR;
    Color_SendCmd(0x60U | level);
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
    data->red = r; data->green = g; data->blue = b;
    data->online = 1U;
    return HAL_OK;
}

Color_TypeDef Color_Judge(const Color_DataTypeDef *data)
{
    if (data == NULL || data->online == 0U) return COLOR_UNKNOWN;
    uint8_t r = data->red, g = data->green, b = data->blue;

    if (g_color_ambient.enabled) {
        int dr = abs((int)r - g_color_ambient.r);
        int dg = abs((int)g - g_color_ambient.g);
        int db = abs((int)b - g_color_ambient.b);
        if (dr <= g_color_ambient.tolerance &&
            dg <= g_color_ambient.tolerance &&
            db <= g_color_ambient.tolerance)
            return COLOR_UNKNOWN;
    }

    for (int i = COLOR_RED; i < COLOR_COUNT; i++) {
        Color_Calib_t *c = &g_color_calib[i];
        if (!c->enabled) continue;
        int dr = abs((int)r - c->r), dg = abs((int)g - c->g), db = abs((int)b - c->b);
        if (dr <= (int)c->tolerance &&
            dg <= (int)c->tolerance &&
            db <= (int)c->tolerance)
            return (Color_TypeDef)i;
    }

    if (r >= 150 && g >= 150 && b >= 150) return COLOR_WHITE;
    if (b >= r && b >= g && b >= 50)      return COLOR_BLUE;
    if (r >= g && r >= b && r >= 50)      return COLOR_RED;
    if (g >= r && g >= b && g >= 50)      return COLOR_GREEN;
    return COLOR_BLACK;
}

/* ================================================================
 * 协议层 — OpenMV: AA CC [color] BB DD, 直接返回颜色枚举值
 * ================================================================ */
#else

static bool Color_ReadFrame(uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t last = 0, cur = 0;
    int retry = 1000;
    while (!(last == 0xAA && cur == 0xCC) && --retry > 0) {
        last = cur;
        cur = SW_UART_ReadByte();
    }
    if (retry <= 0) return false;

    *r = SW_UART_ReadByte();  /* 颜色枚举值 */
    *g = SW_UART_ReadByte();  /* 应为 0xBB */
    *b = SW_UART_ReadByte();  /* 应为 0xDD */
    return (*g == 0xBB && *b == 0xDD);
}

HAL_StatusTypeDef Color_Init(void)
{
    return HAL_OK;  /* OpenMV 上电自启 */
}

HAL_StatusTypeDef Color_SetLedLevel(uint8_t level)
{
    (void)level;
    return HAL_OK;  /* OpenMV 自带补光 */
}

HAL_StatusTypeDef Color_ReadData(Color_DataTypeDef *data)
{
    uint8_t r = 0, g = 0, b = 0;
    if (data == NULL) return HAL_ERROR;
    if (!Color_ReadFrame(&r, &g, &b)) {
        data->online = 0U;
        return HAL_ERROR;
    }
    data->sensor_color = r;  /* 直接存颜色枚举 */
    data->online = 1U;
    return HAL_OK;
}

Color_TypeDef Color_Judge(const Color_DataTypeDef *data)
{
    if (data == NULL || data->online == 0U) return COLOR_UNKNOWN;
    Color_TypeDef c = (Color_TypeDef)data->sensor_color;
    return (c > COLOR_UNKNOWN && c < COLOR_COUNT) ? c : COLOR_UNKNOWN;
}

#endif /* USE_OPENMV_COLOR */

/* ================================================================
 * 共用 — 不受宏影响
 * ================================================================ */

Color_TypeDef Color_DetectDominant(void)
{
    uint8_t counts[COLOR_COUNT] = {0U};
    Color_TypeDef dominant = COLOR_UNKNOWN;
    uint8_t max_count = 0U;

    for (uint8_t w = 0; w < WARMUP_COUNT; w++) { Color_Init(); HAL_Delay(5U); }

    for (uint8_t i = 0; i < SAMPLE_COUNT; i++) {
        Color_DataTypeDef d;
        if (Color_ReadData(&d) == HAL_OK) {
            Color_TypeDef c = Color_Judge(&d);
            if (i >= SKIP_COUNT && c > COLOR_UNKNOWN && c < COLOR_COUNT)
                counts[c]++;
        }
        HAL_Delay(5U);
    }

    for (int i = COLOR_RED; i < COLOR_COUNT; i++)
        if (counts[i] > max_count) { max_count = counts[i]; dominant = (Color_TypeDef)i; }

    return (max_count >= ACCEPT_COUNT) ? dominant : COLOR_UNKNOWN;
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

/* ================================================================
 * 校准 — 仅 GY-33
 * ================================================================ */
#if USE_OPENMV_COLOR == 0

void Color_Calibrate(Color_TypeDef color)
{
    if (color <= COLOR_UNKNOWN || color >= COLOR_COUNT) return;

    Color_DataTypeDef d;
    if (Color_ReadData(&d) == HAL_OK) {
        g_color_calib[color].r = d.red;
        g_color_calib[color].g = d.green;
        g_color_calib[color].b = d.blue;
        g_color_calib[color].tolerance = CALIB_TOLERANCE;
        g_color_calib[color].enabled = 1U;
    }
}

void Color_CalibAmbient(void)
{
    Color_DataTypeDef d;
    if (Color_ReadData(&d) == HAL_OK) {
        g_color_ambient.r = d.red;
        g_color_ambient.g = d.green;
        g_color_ambient.b = d.blue;
        g_color_ambient.tolerance = 20U;
        g_color_ambient.enabled = 1U;
    }
}

/* ---- Flash 存储 ---- */
#define FLASH_CALIB_PAGE   255U
#define FLASH_CALIB_ADDR   0x0807F800U
#define FLASH_CALIB_MAGIC  0x434F4C52U
#define FLASH_PAGE_SIZE    0x800U

static uint32_t CalibChecksum(void)
{
    uint32_t sum = 0;
    uint8_t *p = (uint8_t *)g_color_calib;
    for (size_t i = 0; i < sizeof(g_color_calib); i++) sum += p[i];
    p = (uint8_t *)&g_color_ambient;
    for (size_t i = 0; i < sizeof(g_color_ambient); i++) sum += p[i];
    return sum;
}

void Color_CalibSave(void)
{
    uint32_t header[2] = { FLASH_CALIB_MAGIC, CalibChecksum() };
    uint32_t page_start = FLASH_CALIB_ADDR;
    uint32_t addr;

    __disable_irq();
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks     = FLASH_BANK_1,
        .Page      = FLASH_CALIB_PAGE,
        .NbPages   = 1U,
    };
    uint32_t err;
    HAL_FLASHEx_Erase(&erase, &err);

    addr = page_start;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, *(uint64_t *)header);
    addr += 8;

    uint32_t blob[12];
    memcpy(&blob[0], g_color_calib, sizeof(g_color_calib));
    memcpy(&blob[9], &g_color_ambient, sizeof(g_color_ambient));
    for (int i = 0; i < 12; i += 2) {
        uint64_t d = (uint64_t)blob[i] | ((uint64_t)blob[i+1] << 32);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, d);
        addr += 8;
    }

    HAL_FLASH_Lock();
    __enable_irq();
}

void Color_CalibLoad(void)
{
    for (int i = 0; i < COLOR_COUNT; i++) {
        g_color_calib[i].enabled = 0U;
        g_color_calib[i].tolerance = CALIB_TOLERANCE;
    }
    g_color_ambient.enabled = 0U;

    uint32_t *flash = (uint32_t *)FLASH_CALIB_ADDR;
    if (flash[0] != FLASH_CALIB_MAGIC) return;

    uint32_t blob[12];
    for (int i = 0; i < 12; i++) blob[i] = flash[2 + i];

    memcpy(g_color_calib, &blob[0], sizeof(g_color_calib));
    memcpy(&g_color_ambient, &blob[9], sizeof(g_color_ambient));

    if (flash[1] != CalibChecksum()) {
        for (int i = 0; i < COLOR_COUNT; i++) g_color_calib[i].enabled = 0U;
        g_color_ambient.enabled = 0U;
    }
}

#else
/* OpenMV 不需要校准 */
void Color_Calibrate(Color_TypeDef color) { (void)color; }
void Color_CalibAmbient(void)             {}
void Color_CalibSave(void)                {}
void Color_CalibLoad(void)                {}
#endif /* USE_OPENMV_COLOR */
