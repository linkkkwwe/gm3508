#ifndef RAIL_CONTROL_H
#define RAIL_CONTROL_H

#include "struct_typedef.h"

/*
 * ============ 水平轴（导轨滑块）遥控行为状态机 ============
 *
 * 数据来源：板A（Yaw 板）通过串口发来的 ASCII 帧，25Hz：
 *     $B,<sw>,<stick>,<en>\r\n
 *       sw    拨杆档位 1/2/3（1=波形1，2=波形2，3=摇杆手动）
 *       stick 摇杆值   -1000~1000
 *       en    使能位   1=遥控器在线，0=离线
 *
 * 接法：板B 的 USART1 RX（PB7）。不需要改 CubeMX——这个口已经配好，
 *       NVIC 和 USART1_IRQHandler 都在。PC 串口助手也能直接灌帧测试。
 *
 * ⚠️ USART1 TX（PA9）上还有 serialplot 的遥测数据，和本模块共用一条串口。
 *    接收不受影响（全双工），但上位机会同时看到两种数据。
 *
 * 行为：
 *   sw=1/2  → 先进 HOMING（平滑回到 0°），到位后再启动对应波形
 *   sw=3    → MANUAL，摇杆按速度控制，回中停在原地
 *   离线    → SAFE，目标 = 当前位置（停止移动）
 *
 * 设计原则：模式解释和限位全部在板B 本地；板A 只做数据搬运。
 */

/* 收到一帧时调用（在 USART1 中断里），传入刚收到的一个字节 */
void rail_control_rx_byte(uint8_t byte);

/* 初始化：清状态、复位到 SAFE */
void rail_control_init(void);

/**
 * @brief 每个控制周期（1ms）调用一次，产出水平轴的目标角度。
 * @param current_angle 当前角度（度，相对上电位置），传 motor->current_angle
 * @return 目标角度（度）
 */
fp32 rail_control_update(fp32 current_angle);

#endif
