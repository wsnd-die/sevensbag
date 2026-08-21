#ifndef _Can_H_
#define _Can_H_
#include <stdbool.h>


uint8_t can_SendCmd(__IO uint8_t *cmd, uint8_t len);
void FDCAN1_UserInit(void);

/* CAN 发送错误诊断 (bus-off 恢复用) */
extern volatile uint32_t can_error_count;
extern volatile uint32_t can_error_code;
extern volatile uint32_t can_error_step;

#endif
