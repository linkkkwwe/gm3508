#ifndef PID_H
#define PID_H

#include "struct_typedef.h"

/**
 * @brief Position-form PID controller.
 *
 * 位置式 PID：out = Kp*e + Ki*Σe + Kd*(e_k - e_{k-1})
 *
 * 角度环（外环）与速度环（内环）共用此结构体，只靠 Pid_init 时传入的
 * 参数不同来区分：外环输入角度误差、输出目标转速；内环输入转速误差、
 * 输出电机电压。
 */
typedef struct
{
    fp32 Kp;        /**< Proportional gain 比例增益. */
    fp32 Ki;        /**< Integral gain 积分增益. */
    fp32 Kd;        /**< Derivative gain 微分增益. */

    fp32 max_out;   /**< 输出饱和限幅，如 GM6020 电压上限 30000. */
    fp32 max_iout;  /**< 积分饱和限幅，防止积分饱和 (anti-windup). */

    fp32 set;       /**< Setpoint 目标值. */
    fp32 fdb;       /**< Feedback 实际测量值. */

    fp32 out;       /**< 最终输出. */
    fp32 Pout;      /**< 比例项输出. */
    fp32 Iout;      /**< 积分项输出（累加后限幅）. */
    fp32 Dout;      /**< 微分项输出. */
    fp32 Dbuf[3];   /**< 微分历史：0=最新，1=上一次，2=上上次. */
    fp32 error[3];  /**< 误差历史：0=最新，1=上一次，2=上上次. */
} pid_type_def;

/**
 * @brief 初始化 PID：写入 Kp/Ki/Kd 与限幅，清零内部状态。
 * @param pid       PID 实例指针。
 * @param PID       增益数组 [Kp, Ki, Kd]。
 * @param max_out   输出限幅（绝对值上限）。
 * @param max_iout  积分限幅（绝对值上限）。
 */
void Pid_init(pid_type_def *pid, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
 * @brief 执行一次 PID 计算（位置式）。
 * @param pid  PID 实例指针。
 * @param ref  反馈值（实际测量，如当前角度/转速）。
 * @param set  目标值（期望角度/转速）。
 * @return     限幅后的输出 out。
 */
fp32 Pid_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
 * @brief 清零 PID 全部内部状态（误差、微分、积分、输出）。
 *        电机离线/重新上电后调用，避免残留积分导致突跳。
 */
void Pid_clear(pid_type_def *pid);

#endif
