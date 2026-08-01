#ifndef __NAVIGATION_MECANUM_H
#define __NAVIGATION_MECANUM_H

#include <stdbool.h>
#include <stdint.h>
#include "Mecanum_Move.h"
#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 类型定义
 * ============================================================ */


typedef struct {
    float x;
    float y;
    float yaw;
    
} World_Dir;






/* ============================================================
 * 函数声明
 * ============================================================ */

void Chassis_WorldMoveTest(void);





#ifdef __cplusplus
}
#endif

#endif /* __NAVIGATION_MECANUM_H */