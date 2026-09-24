#include "trajectory.h"
#include "main.h"
#include <math.h>

#define PI_F                    3.1415926f

/* 摆幅（输出轴角度，度）。⚠️ 占位值，必须按实际传动比换算：
 * Motor.c 里 current_angle 已除过 gear_ratio(19.2)，所以这里单位是输出轴角度。
 * 量法：让电机转 360°，量滑块走多少 mm → 得 mm/度；再按目标半行程反算。
 * 暂用 180°（= 半圈输出轴），比原值 500° 保守得多——机构未知时先用小行程
 * 确认方向和对齐，再逐级放大。 */
#define TRAJECTORY_AMPLITUDE   720.0f

/* 一个完整周期（ms）。原值是 1U，会让 `elapsed_ms % 1U` 恒等于 0 →
 * 相位永远是 0 → 两个波形都恒输出 0.0f，看起来像"停在 0°"而不是在走波形。 */
#define TRAJECTORY_PERIOD_MS   2000U

/*
 * 轨迹生成器：给定"从模式启动到现在经过的时间"，返回一个目标角度。
 * 主循环每 1ms 调用一次，把目标角度喂给水平电机角度环 PID。
 *
 * 时间用 HAL_GetTick() 计算（真实毫秒），而不是循环计数——这样即使
 * 循环节奏被中断打乱，轨迹相位也不会漂移。
 */

static traj_mode_t horizontal_mode = TRAJ_STOP;
static uint32_t horizontal_start_tick;

/**
 * @brief 三角波：0→+A 匀速上升，+A→-A 匀速下降，-A→0 匀速回升。
 *
 * 按相位分段（phase 在 0~1 内循环）：
 *   0.00~0.25  上升段：角度 = 4*A*phase            (0°  → +30°)
 *   0.25~0.75  下降段：角度 = 2*A - 4*A*phase      (+30° → -30°)
 *   0.75~1.00  回升段：角度 = -4*A + 4*A*phase     (-30° → 0°)
 * 起点选在 0°，避免启动瞬间从 ±30° 跳变。
 */
static fp32 triangle_angle(uint32_t elapsed_ms)
{
    fp32 phase = (elapsed_ms % TRAJECTORY_PERIOD_MS) /
                 (fp32)TRAJECTORY_PERIOD_MS;

    if (phase < 0.25f)
        return 4.0f * TRAJECTORY_AMPLITUDE * phase;
    if (phase < 0.75f)
        return 2.0f * TRAJECTORY_AMPLITUDE -
               4.0f * TRAJECTORY_AMPLITUDE * phase;
    return -4.0f * TRAJECTORY_AMPLITUDE +
           4.0f * TRAJECTORY_AMPLITUDE * phase;
}

/**
 * @brief 按模式把经过时间换算成目标角度。
 * @param mode       轨迹模式（正弦/三角/停止）。
 * @param elapsed_ms 从启动模式起经过的毫秒数。
 * @return 目标角度（度）。
 */
static fp32 calc_angle(traj_mode_t mode, uint32_t elapsed_ms)
{
    fp32 phase;

    switch (mode)
    {
    case TRAJ_SINE:
        /* 正弦：A*sin(2π*phase)，相位随时间连续变化，速度平滑无突变 */
        phase = (elapsed_ms % TRAJECTORY_PERIOD_MS) /
                (fp32)TRAJECTORY_PERIOD_MS;
        return TRAJECTORY_AMPLITUDE * sinf(2.0f * PI_F * phase);

    case TRAJ_TRIANGLE:
        return triangle_angle(elapsed_ms);

    case TRAJ_STOP:
    default:
        return 0.0f;
    }
}

void trajectory_init(void)
{
    horizontal_mode = TRAJ_STOP;
    horizontal_start_tick = HAL_GetTick();
}

void trajectory_set_horizontal_mode(traj_mode_t mode)
{
    horizontal_mode = mode;
    horizontal_start_tick = HAL_GetTick();   /* 记下启动时刻，轨迹从 0 相位开始 */
}

fp32 trajectory_get_horizontal(void)
{
    return calc_angle(horizontal_mode,
                      HAL_GetTick() - horizontal_start_tick);
}
