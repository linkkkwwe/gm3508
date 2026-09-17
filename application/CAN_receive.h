#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
#include "struct_typedef.h"

/*
 * ============ CAN 协议：水平轴 = M3508（C620 电调，单电机工程） ============
 * 本工程只控制水平电机（Yaw/GM6020 在 Guide_rail 工程里，见那边的代码）。
 * M3508 ：控制帧 0x200（管 ID 1~4），反馈帧 0x200+ID，电流 ±16384，减速比 19.2
 * 反馈帧 8 字节 = ecd(2B) + speed_rpm(2B) + given_current(2B)
 *               + temperature(1B) + 保留(1B)，全部大端。
 * ========================================================================
 */

#define C620_COMMAND_ID        0x200U            /* C620 控制帧（管 ID 1~4） */
#define HORIZONTAL_ESC_ID       4U               /* 水平电调拨码 ID（C620/M3508，当前电调实测=4） */
#define CAN_HORIZONTAL_FEEDBACK_ID (0x200U + HORIZONTAL_ESC_ID) /* = 0x204 */

/* 电机索引：与 motor_measure[] 数组下标对应（本工程只有一个电机） */
typedef enum
{
    HORIZONTAL_MOTOR = 0,  /**< 水平轴（移动） */
    GUIDE_MOTOR_COUNT      /**< 电机数量（数组大小） */
} guide_motor_index_e;

/* 一帧反馈解析后的数据（由 CAN 接收中断在后台更新） */
typedef struct
{
    uint16_t ecd;          /**< 编码器原始值 0~8191，对应机械角度 0~360°. */
    int16_t  speed_rpm;    /**< 当前转速 rpm，正负表示方向. */
    int16_t  given_current;/**< 实际电流，有符号. */
    uint8_t  temperature;  /**< 电机温度 °C. */
    uint8_t  received;     /**< 是否收到过反馈（在线标志）. */
    uint32_t last_rx_tick; /**< 最近一次收到反馈的时间戳，用于离线判断. */
} motor_measure_t;

extern volatile motor_measure_t motor_measure[GUIDE_MOTOR_COUNT];

/**
 * @brief 下发水平电机指令：走 0x200（C620）。
 * @param horizontal_voltage 水平电机电流（±16384，C620/M3508）。
 * @return HAL_OK 发送成功；否则失败。
 */
HAL_StatusTypeDef CAN_cmd_horizontal(int16_t horizontal_voltage);

/**
 * @brief 判断水平电机反馈是否在线（100ms 内有新反馈）。
 * @return 1 = 在线可闭环，0 = 掉线需下电。
 */
uint8_t CAN_motor_feedback_ready(void);

#endif
