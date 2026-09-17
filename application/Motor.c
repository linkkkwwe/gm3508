#include "Motor.h"
#include <stddef.h>

#define ECD_RANGE       8192              /* 编码器一圈 = 8192 计数 */
#define ECD_HALF_RANGE  4096              /* 半圈阈值：ecd 跳变超过它即判定过零 */
#define DEG_PER_ECD     (360.0f / 8192.0f) /* 每计数对应的角度 */
#define M3508_GEAR_RATIO   19.2032  /* 3591/187，转子转19.2圈=输出轴1圈 */

/* ============ 水平电机 PID 调参变量（Keil Debug 实时修改） ============
 * 在 Debug 模式的 Watch 窗口里输入变量名即可查看和修改，
 * 修改后下一帧控制立即生效，无需重新编译烧录。
 * 默认值：Kp=500, Ki=0, Kd=200, MaxOut=5000, MaxIout=0
 * ======================================================================== */
fp32 g_horizontal_kp      = 300.0f;
fp32 g_horizontal_ki      =   0.0f;
fp32 g_horizontal_kd      = 1700.0f;
fp32 g_horizontal_max_out = 16384.0f;
fp32 g_horizontal_max_iout =5000.0f;

/* 观测变量：每帧更新，Keil Debug Watch / Logic Analyzer 直接看
 * 加 volatile 防止编译器优化掉（否则 Debug 里报"未知信号"） */
volatile int16_t g_dbg_speed_rpm     = 0;
volatile fp32    g_dbg_current_angle = 0.0f;
volatile fp32    g_dbg_target_angle  = 0.0f;
volatile int16_t g_dbg_pid_output    = 0;

/**
 * @brief 限幅：把 value 限制在 [min_value, max_value]。
 */
static fp32 constrain_float(fp32 value, fp32 min_value, fp32 max_value)
{
    if (value > max_value)
        return max_value;
    if (value < min_value)
        return min_value;
    return value;
}

void motor_ctrl_init(motor_ctrl_t *motor, motor_axis_e axis)
{
    fp32 angle_pid[3];

    if (motor == NULL)
        return;

    /* 本工程只有水平轴：角度环单环，输出直接当电流 */
    (void)axis;
    angle_pid[0] = g_horizontal_kp;
    angle_pid[1] = g_horizontal_ki;
    angle_pid[2] = g_horizontal_kd;
    Pid_init(&motor->pid_angle, angle_pid,
             g_horizontal_max_out, g_horizontal_max_iout);
    motor->use_cascade = 0U; /* 单环（角度环）控制：输出直接当电流，靠轨迹实现匀速往返 */
    motor->min_angle = HORIZONTAL_MIN_ANGLE_DEG;
    motor->max_angle = HORIZONTAL_MAX_ANGLE_DEG;
    motor->gear_ratio = M3508_GEAR_RATIO;

    motor_ctrl_clear(motor);
}

void motor_ctrl_clear(motor_ctrl_t *motor)
{
    if (motor == NULL)
        return;

    Pid_clear(&motor->pid_angle);

    /* gear_ratio 不在这里重置：init 已按轴设好（水平=19.2），
     * 这里覆盖会把减速比冲回 1.0，角度变成转子角度 */
    motor->total_rounds = 0;
    motor->offset_ecd = 0U;
    motor->last_ecd = 0U;
    motor->current_angle = 0.0f;
    motor->initialized = 0U;
}

int16_t motor_ctrl_update(motor_ctrl_t *motor, fp32 target_angle,
                          uint16_t ecd, int16_t speed_rpm)
{
    int32_t ecd_diff;
    int32_t relative_ecd;
    fp32 voltage;

    if (motor == NULL)
        return 0;

    /* 每帧同步最新调参变量到 PID 结构体：
     * Keil Debug 里改 g_horizontal_kp/kd/...，下一帧立即生效。
     * 注意：只同步增益和限幅，不清误差状态（否则每次改参数都会归零抖动）。 */
    motor->pid_angle.Kp = g_horizontal_kp;
    motor->pid_angle.Ki = g_horizontal_ki;
    motor->pid_angle.Kd = g_horizontal_kd;
    motor->pid_angle.max_out = g_horizontal_max_out;
    motor->pid_angle.max_iout = g_horizontal_max_iout;

    /* 首帧反馈：把当前 ecd 记为 0° 参考点（上电位置），
     * 并跳过本次过零判断（还没有 last_ecd 可比） */
    if (!motor->initialized)
    {
        motor->offset_ecd = ecd;
        motor->last_ecd = ecd;
        motor->initialized = 1U;
        return 0;
    }

    /* 过零处理：ecd 是 0~8191 的环形量，跨过 8191↔0 时差值会
     * 突变成 ±8000 量级。差值 > 半圈(4096) 说明反向跨零 → 圈数-1；
     * 差值 < -4096 说明正向跨零 → 圈数+1。用 total_rounds 记录圈数，
     * 把环形 ecd 展开成单调递增的连续角度。 */
    ecd_diff = (int32_t)ecd - (int32_t)motor->last_ecd;
    motor->last_ecd = ecd;

    if (ecd_diff > ECD_HALF_RANGE)
        motor->total_rounds--;
    else if (ecd_diff < -ECD_HALF_RANGE)
        motor->total_rounds++;

    /* 连续角度 = (累计圈数 × 一圈 + 当前ecd - 上电ecd) × 每计数角度 ÷ 减速比 */
    relative_ecd = motor->total_rounds * ECD_RANGE +
                   (int32_t)ecd - (int32_t)motor->offset_ecd;
    motor->current_angle = relative_ecd * DEG_PER_ECD / motor->gear_ratio;

    /* 目标角度限幅，防止轨迹超出机械行程 */
    target_angle = constrain_float(target_angle, motor->min_angle, motor->max_angle);

    /* 单环：角度环输出直接当电流 */
    voltage = Pid_calc(&motor->pid_angle,
                       motor->current_angle, target_angle);

    (void)speed_rpm; /* 单环角度控制不用转速反馈，参数保留兼容 Guide_rail 接口 */

    /* 记录观测变量：Keil Debug Watch 里实时看 */
    g_dbg_speed_rpm     = speed_rpm;
    g_dbg_current_angle = motor->current_angle;
    g_dbg_target_angle  = target_angle;
    g_dbg_pid_output    = (int16_t)voltage;

    return (int16_t)voltage;
}
