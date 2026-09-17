#include "pid.h"
#include <stddef.h>

/**
 * @brief 限幅宏：把 input 限制在 [-max, +max]。
 *        do{...}while(0) 包装使宏可以安全地用在 if/else 之后。
 */
#define LimitMax(input, max)      \
    do {                          \
        if ((input) > (max))      \
            (input) = (max);      \
        else if ((input) < -(max)) \
            (input) = -(max);     \
    } while(0)

void Pid_init(pid_type_def *pid, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
        return;

    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;

    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

fp32 Pid_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
        return 0.0f;

    /* 误差历史平移：e(k-1)→e(k-2)，e(k)→e(k-1)，然后记录新误差 e(k) */
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

    /* P 项：当前误差 × Kp */
    pid->Pout = pid->Kp * pid->error[0];

    /* I 项：历史误差累加 × Ki，累加后做饱和限幅（防积分饱和） */
    pid->Iout += pid->Ki * pid->error[0];

    /* D 项：本次误差与上次误差之差 × Kd（即误差的变化率，无滤波） */
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);

    /* 总输出 = P + I + D，再整体限幅 */
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);

    return pid->out;
}

void Pid_clear(pid_type_def *pid)
{
    if (pid == NULL)
        return;

    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
