#include "serialplot.h"
#include "usart.h"
#include "Motor.h"
#include <stdio.h>

/*
 * 每 10ms 调用一次，通过 USART1 发送 3 个整数（逗号分隔 + 换行）。
 * 用整数格式化（角度×10）避免浮点 sprintf 导致栈溢出。
 * 115200 波特率下 ~15 字节约 1.3ms，在 10ms 间隔下不影响控制周期。
 */
void serialplot_output(void)
{
    char buf[32];
    int len;

    len = snprintf(buf, sizeof(buf), "%d,%d,%d\r\n",
                   (int)(g_dbg_target_angle  * 10.0f),
                   (int)(g_dbg_current_angle * 10.0f),
                   (int)g_dbg_pid_output);

    if (len > 0)
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 10);
}
