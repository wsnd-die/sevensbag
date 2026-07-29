#ifndef _Can_H_
#define _Can_H_
#include <stdbool.h>


uint8_t can_SendCmd(__IO uint8_t *cmd, uint8_t len);
void FDCAN1_UserInit(void);

#endif
