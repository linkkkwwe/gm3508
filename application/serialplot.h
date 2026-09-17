#ifndef SERIALPLOT_H
#define SERIALPLOT_H

/*
 * SerialPlot 调试输出（USART1, 115200, PA9 TX）
 *
 * 格式：ASCII 逗号分隔，3 通道
 *   target_angle_x10, current_angle_x10, pid_output\n
 * SerialPlot 设置：ASCII / Comma / 3 channels
 * 角度值 = 实际角度 × 10（避免浮点格式化导致栈溢出）
 */

void serialplot_output(void);

#endif
