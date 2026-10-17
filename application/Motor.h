#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"

/*
 * ==================== 电机控制结构（水平轴单电机工程） ====================
 * 本工程只控制水平轴 M3508（Yaw/GM6020 在 Guide_rail 工程里）。
 * 水平轴：角度环单环，角度环 PID → 电流 → CAN
 * 无 IMU：位置反馈用电机自带编码器（上电位置为 0° 参考）。
 * ========================================================================
 */

/* ===== 水平电机 PID 参数（动态变量，Keil Debug 里可直接修改） =====
 * 运行时修改这些变量，下一帧控制立即生效，不用重新编译烧录。
 * Kp/Kd 是主要调节旋钮，Ki 默认 0（角度环一般不用积分）。 */
extern fp32 g_horizontal_kp;       /* 比例增益：推不动就加大，振荡就减小 */
extern fp32 g_horizontal_ki;       /* 积分增益：默认 0，角度环一般不用 */
extern fp32 g_horizontal_kd;       /* 微分增益：抑制超调，太大会嗡嗡抖 */
extern fp32 g_horizontal_max_out;  /* 输出限幅：M3508 最大 ±16384，先从小的试 */
extern fp32 g_horizontal_max_iout; /* 积分限幅：Ki=0 时无效 */

/* ===== 摩擦前馈 =====
 * 低速顿挫（爬行）是静摩擦造成的：PD 输出过不了静摩擦就卡住，误差攒够了一跳，
 * 跳动后误差塌掉又被抓住 —— 循环往复。这是机械特性，调 PID 治不了根。
 * 这里按运动方向补一个固定偏置电流，直接跨过静摩擦死区。
 *
 * g_horizontal_ff_gain  标定幅值（Keil Watch 里从 0 往上加，加到低速不顿为止，
 *                       典型几百到一千多）；0 = 关闭，不加任何影响。
 * g_horizontal_ff       带符号的当前值，由 rail_control 按模式/方向写入，
 *                       只有手动档低速运动时非零，波形/归位/SAFE 恒为 0。 */
extern volatile fp32 g_horizontal_ff_gain;
extern volatile fp32 g_horizontal_ff;

/* ===== 零点重设 / 限位开关 =====
 * g_horizontal_rezero_req  置 1 后，motor_ctrl_update() 在下一帧把"当前编码器
 *                           位置"记为新的 0° 参考。物理位置不动，只是把坐标系
 *                           平移过去，所以电机不会跳。
 *                           ⚠️ 请求方必须同时把自己持有的目标角度归零，否则
 *                           误差会突然变成"旧目标"那么大，电机会猛冲回去。
 *                           ⚠️ 当前 rail_control 已不再置位（零点固定为上电位置），
 *                           这段逻辑保留备用，别当成还在生效的功能。
 *                           典型用法：手动档摇杆回中停下时，把停下的位置设成原点。
 * g_horizontal_limit_off    1 = 跳过目标角度的限位夹紧（当前恒为 0，全模式都夹）；
 *                           0 = 按 min_angle/max_angle 夹紧（默认）。
 *                           ⚠️ 置 1 期间软件限位完全失效，撞到机械端点不会自己停。
 *                           注意：光靠这里夹还不够——手动档的 manual_target 是
 *                           rail_control 内部的累加器，那边也要同步夹紧，否则
 *                           顶到限位后累加器继续虚涨，往回推会有一段空行程。 */
extern volatile uint8_t g_horizontal_rezero_req;
extern volatile uint8_t g_horizontal_limit_off;

/* ===== 调参观测变量（Keil Debug Watch / Logic Analyzer 直接看） =====
 * 每帧控制后更新，实时反映电机运行状态，方便边看边调。
 * 加 volatile 防止编译器优化掉（否则 Debug 里会报"未知信号"）。 */
extern volatile int16_t g_dbg_speed_rpm;       /**< 电机当前转速（转子 rpm），来自编码器反馈 */
extern volatile fp32    g_dbg_current_angle;   /**< 当前角度（输出轴，度，相对上电位置） */
extern volatile fp32    g_dbg_target_angle;    /**< 目标角度（输出轴，度，来自轨迹生成器） */
extern volatile int16_t g_dbg_pid_output;      /**< PID 输出电流（±16384），发给 CAN */

/* ===== 角度限位（相对上电位置） =====
 * 水平轴：±1080° 限位，靠 constrain_float 限制目标角度（Motor.c 里夹的是
 * 目标角度，不是实际角度）。
 * ⚠️ 改数值就把这行注释一起改，别留下过期说明误导排查。
 * ✅ 这是绝对行程保护：零点固定为上电位置，rail_control 已不再请求重设零点，
 *    所以 ±1080 的起点不会平移。
 * ⚠️ 唯一例外：反馈掉线触发 motor_ctrl_clear() 时，零点会重设为重连那一刻的
 *    位置（offset_ecd 重取、total_rounds 清零）。要连这个也固定掉得另改。 */
#define HORIZONTAL_MIN_ANGLE_DEG   (-1080.0f)
#define HORIZONTAL_MAX_ANGLE_DEG     1080.0f

/* 轴类型：motor_ctrl_init 用它选择对应轴的 PID 参数与限位
 * （本工程只有水平轴，保留枚举形式与 Guide_rail 工程接口一致） */
typedef enum
{
    MOTOR_AXIS_HORIZONTAL = 0
} motor_axis_e;

/* 单轴电机控制器：PID + 编码器连续角度换算状态 */
typedef struct
{
    pid_type_def pid_angle;   /**< 角度环 PID. */

    uint8_t  use_cascade;      /**< 兼容字段：本工程恒为 0（单环角度控制）. */

    fp32     gear_ratio;      /**< 减速比：M3508=19.2（有减速箱）. */
    int32_t  total_rounds;    /**< 累计圈数（过零时 ±1）. */
    uint16_t offset_ecd;      /**< 上电时记录的 ecd，作为 0° 参考点. */
    uint16_t last_ecd;        /**< 上一次 ecd，用于计算过零跳变. */
    fp32     current_angle;   /**< 当前连续角度（度，相对上电位置）. */
    fp32     min_angle;       /**< 角度下限. */
    fp32     max_angle;       /**< 角度上限. */
    uint8_t  initialized;     /**< 首帧反馈是否已捕获（用于记录 offset_ecd）. */
} motor_ctrl_t;
/**
 * @brief 初始化电机控制器：按轴类型选 PID 参数与限位，并清零状态。
 * @param motor 控制器实例。
 * @param axis  轴类型：本工程只有 MOTOR_AXIS_HORIZONTAL。
 */
void motor_ctrl_init(motor_ctrl_t *motor, motor_axis_e axis);

/**
 * @brief 执行一次 PID 控制（每个控制周期调用一次）。
 * @param motor      控制器实例。
 * @param target_angle 目标角度（度，来自轨迹生成器）。
 * @param ecd        电机编码器反馈（0~8191）。
 * @param speed_rpm  电机转速反馈（rpm）。
 * @return 输出电流（±16384），给 CAN_cmd_horizontal。
 */
int16_t motor_ctrl_update(motor_ctrl_t *motor, fp32 target_angle,
                          uint16_t ecd, int16_t speed_rpm);

/**
 * @brief 清零控制器状态（掉线重连/重新上电时调用）。
 *        下次 update 会把当前 ecd 记为新 0° 参考。
 */
void motor_ctrl_clear(motor_ctrl_t *motor);

#endif
