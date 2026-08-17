//
// Created by 35037 on 2026/8/12.
//
#ifndef STM32G4_TEST_NAV_POSITION_H
#define STM32G4_TEST_NAV_POSITION_H

#include "Common_used.h"

/* 当前里程计位姿） */
extern World_Dir_t World_position;

/**
 * @brief 增量式编码器里程计，周期性调用（每 10~20ms）
 * @return 当前世界位姿（同时更新全局 World_position）
 */
World_Dir_t World_position_get(void);

/**
 * @brief 清零编码器里程计并重新起算
 * @note  放完 5 物块 / 奖杯段分界时调用: 只清 World_position;
 *        下次 World_position_get 以当前编码器为基准重设起点, 不跨清零边界累积。
 */
void World_Reset(void);

/* ============================================================
 * 惯导 (INS) — 纯 IMU 加速度积分推位
 *
 * 原理: 车体系加速度(IMU) → Mahony 姿态四元数旋转到世界系
 *       → 减重力 → 积分得世界速度 → 再积分得世界位置
 *
 * 注意: 纯积分对加速度零偏极敏感, 位置会随时间漂移。
 *       调用方应周期性用 ZUPT(静止检测)/编码器做速度校正。
 * ============================================================ */
typedef struct {
    float x, y;            /* 世界位置 (m) */
    float vx, vy;          /* 世界速度 (m/s) */
    uint8_t inited;
    uint8_t stationary;    /* 最近是否判定为静止 (ZUPT) */
    uint32_t last_tick;
    float bias_x, bias_y;  /* 水平加速度零偏估计 (m/s², 静止时在线学习) */
} Ins_t;

extern Ins_t g_ins;

/**
 * @brief 惯导初始化（上电/复位后调用一次，清零状态）
 */
void Ins_Init(void);

/**
 * @brief 惯导更新（周期性调用，如 200Hz 与 IMU 对齐）
 * @note  调用前需先 Ins_Init(); 内部自算 dt, 无需外部传参
 */
void Ins_Update(void);

#endif //STM32G4_TEST_NAV_POSITION_H