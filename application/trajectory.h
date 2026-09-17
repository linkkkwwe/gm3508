#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include "struct_typedef.h"

/*
 * ============ 水平轴轨迹生成器（单电机工程，无 Yaw 轴） ============
 * Yaw/GM6020 的代码在 Guide_rail 工程里，见那边。
 * ========================================================================
 */

typedef enum
{
    TRAJ_STOP = 0,      /**< 停止，目标角度恒为 0°. */
    TRAJ_SINE,          /**< 正弦摆动：速度平滑，两端自然减速. */
    TRAJ_TRIANGLE       /**< 三角波：匀速来回扫，从 0° 相位启动. */
} traj_mode_t;

void trajectory_init(void);
void trajectory_set_horizontal_mode(traj_mode_t mode);
fp32 trajectory_get_horizontal(void);

#endif
