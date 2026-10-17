#include "trajectory.h"
#include "main.h"
#include <math.h>

#define PI_F                    3.1415926f

/* 摆幅（输出轴角度，度）。Motor.c 里 current_angle 已除过 gear_ratio(19.2)，
 * 所以这里单位是输出轴角度。当前 900°，限位 ±1080°，留 180° 余量。
 * ⚠️ 改这个值要同时看两处：
 *   1. Motor.h 的 ±1080 限位——摆幅超过它会被夹紧削顶，等于白改；
 *   2. 下面 TRAJECTORY_CORNER_MS 注释里的速度/加速度估算。 */
#define TRAJECTORY_AMPLITUDE   900.0f

/* 一个完整周期（ms）。原值是 1U，会让 `elapsed_ms % 1U` 恒等于 0 →
 * 相位永远是 0 → 两个波形都恒输出 0.0f，看起来像"停在 0°"而不是在走波形。 */
#define TRAJECTORY_PERIOD_MS   3000U

/* 三角波顶点倒角半宽（ms）：顶点前后各这么多毫秒内，用抛物线代替尖角，
 * 换向在 2×这个值的时间内完成，而不是在 1 ms 内瞬间反向。
 *
 * 峰值加速度 = V / t_b，V = 4A/T 是匀速段速度。当前 900°/3s → V = 1200 °/s：
 *   200ms → 6000 °/s² → 电流约 2500（限幅 16384，余量充足）
 *   100ms → 12000 °/s² → 电流约 5000
 * 还跳齿就往上加（每次 +50），想更贴近理想三角就往下减。
 *
 * 代价：峰值被削掉 V·t_b/2 = 120°，所以实际摆幅是 ±780° 而不是 ±900°。
 * 想保住 ±900°，把 AMPLITUDE 改成 900+120 = 1020——这个值仍在 ±1080 限位内
 * （余量 60°），不用再动 Motor.h。 */
#define TRAJECTORY_CORNER_MS   200.0f

/* 起步渐入时长（ms）：模式刚启动的这段时间内，把"真实时间"映射成
 * "轨迹时间"，让轨迹推进速率从 0 平滑升到 1，之后再恢复正常速率。
 *
 * 为什么需要：波形都从相位 0（0°）起步，而 0° 恰恰是速度最大的位置——
 *   三角波起步就要 1200°/s，正弦起步就要 1885°/s（A·2π/T，峰值速度）。
 *   从 HOMING 的 200°/s 切过去是 Δv = 1000 / 1685 °/s 的阶跃，
 *   需求加速度 1.0e6 / 1.69e6 °/s² → 撞满 16384 硬冲 25 / 43 ms → 皮带跳齿。
 *   这和三角波顶点是同一类问题（速度不连续），只是位置不同。
 *
 * 加了渐入后起步峰值加速度降到约 6300 / 9900 °/s²，电流约 2600 / 4100。
 * 代价：轨迹相位整体落后 T_ramp/2（150ms），对周期性轨迹无影响。
 * 起步还跳齿就把这个值往上加（每次 +100）。 */
#define TRAJECTORY_START_RAMP_MS 300.0f

/**
 * @brief 起步渐入：把"真实经过时间"映射成"轨迹时间"（毫秒）。
 *        dτ/dt = 0.5·(1 − cos(π·t/T_ramp))：t=0 时为 0（速度为 0），
 *        t=T_ramp 时为 1（正常速率），两端位置与速率都连续。
 */
static uint32_t ramp_elapsed(uint32_t elapsed_ms)
{
    fp32 t  = (fp32)elapsed_ms * 0.001f;
    fp32 ta = TRAJECTORY_START_RAMP_MS * 0.001f;
    fp32 tau;

    if (t >= ta)
        tau = t - 0.5f * ta;                                 /* 之后斜率恒为 1 */
    else
        tau = 0.5f * t - (ta / (2.0f * PI_F)) * sinf(PI_F * t / ta);

    return (uint32_t)(tau * 1000.0f);
}

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
    fp32 period_s  = (fp32)TRAJECTORY_PERIOD_MS * 0.001f;
    fp32 phase     = (elapsed_ms % TRAJECTORY_PERIOD_MS) /
                     (fp32)TRAJECTORY_PERIOD_MS;
    fp32 v         = 4.0f * TRAJECTORY_AMPLITUDE / period_s;  /* 匀速段速度 °/s */
    fp32 tb_s      = TRAJECTORY_CORNER_MS * 0.001f;
    fp32 tb_phase  = tb_s / period_s;
    fp32 accel     = v / tb_s;                                /* 抛物线内恒定加速度 */
    fp32 d1        = phase - 0.25f;                           /* 距 +A 顶点的偏移 */
    fp32 d2        = phase - 0.75f;                           /* 距 -A 顶点的偏移 */
    fp32 t1        = d1 * period_s;                           /* 换算成秒 */
    fp32 t2        = d2 * period_s;

    /* 顶点倒角：抛物线在两端位置和速度都与直线段接上，只把加速度从
     * 无穷大压到 accel。+A 峰向下弯，-A 谷向上弯。 */
    if (d1 > -tb_phase && d1 < tb_phase)
        return TRAJECTORY_AMPLITUDE + v * t1 -
               0.5f * accel * (t1 + tb_s) * (t1 + tb_s);

    if (d2 > -tb_phase && d2 < tb_phase)
        return -TRAJECTORY_AMPLITUDE - v * t2 +
               0.5f * accel * (t2 + tb_s) * (t2 + tb_s);

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
    uint32_t traj_ms = ramp_elapsed(elapsed_ms);   /* 起步渐入后的轨迹时间 */

    switch (mode)
    {
    case TRAJ_SINE:
        /* 正弦：A*sin(2π*phase)，相位随时间连续变化，速度平滑无突变 */
        phase = (traj_ms % TRAJECTORY_PERIOD_MS) /
                (fp32)TRAJECTORY_PERIOD_MS;
        return TRAJECTORY_AMPLITUDE * sinf(2.0f * PI_F * phase);

    case TRAJ_TRIANGLE:
        return triangle_angle(traj_ms);

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
