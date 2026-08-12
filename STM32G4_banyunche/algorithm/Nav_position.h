//
// Created by 35037 on 2026/8/12.
//
#ifndef STM32G4_TEST_NAV_POSITION_H
#define STM32G4_TEST_NAV_POSITION_H

#include "Common_used.h"

/* 当前里程计位姿 */
extern World_Dir_t World_position;

/**
 * @brief 增量式编码器里程计
 * @return 当前世界位姿（同时更新全局 World_position）
 */
World_Dir_t World_position_get(void);

#endif //STM32G4_TEST_NAV_POSITION_H