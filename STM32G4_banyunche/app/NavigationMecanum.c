#include "NavigationMecanum.h"
World_Dir Self_Dir={0.0f,0.0f,0.0f};

void Chassis_WorldMoveTest(void)
{
    MecanumMove_t move;
    bool success;

    success = Mecanum_CalculateWorldMove(
        &g_mecanum_config,

        0.50f,                         /* 世界X：前进50 cm */
       -0.50f,                         /* 世界Y：向右50 cm */
        0.0f * MECANUM_DEG_TO_RAD,    /* 开始航向0° */
        50.0f * MECANUM_DEG_TO_RAD,   /* 逆时针旋转50° */

        &move
    );

    if (success) {
        Mecanum_ExecuteMove(
            &g_mecanum_config,
            &move
        );
    }
}











