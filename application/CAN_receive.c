#include "CAN_receive.h"
#include "can.h"
#include <string.h>

/* 电机反馈数组：中断里写，主循环读，必须 volatile 防止编译器缓存 */
volatile motor_measure_t motor_measure[GUIDE_MOTOR_COUNT];

/**
 * @brief 把 8 字节反馈帧解析进电机结构体（大端拼回 16 位）。
 * @param motor 目标电机结构体（volatile，因为是中断上下文）。
 * @param data  CAN 帧 8 字节数据。
 */
static void motor_measure_update(volatile motor_measure_t *motor, const uint8_t data[8])
{
    motor->ecd = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    motor->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    motor->given_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    motor->temperature = data[6];
    motor->received = 1U;                     /* 标记在线 */
    motor->last_rx_tick = HAL_GetTick();      /* 记录本次收到的时间 */
}

/**
 * @brief CAN 接收中断回调（FIFO0 有消息时由 HAL 自动调用）。
 *        先做协议校验（标准帧、数据帧、8 字节），再按反馈 ID 分发。
 *        注意：中断上下文里不能做耗时操作。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (hcan == NULL || hcan->Instance != CAN1)
        return;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        return;
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC != 8U)
        return;

    if (header.StdId == CAN_HORIZONTAL_FEEDBACK_ID)
        motor_measure_update(&motor_measure[HORIZONTAL_MOTOR], data);
}

/**
 * @brief 下发水平电机指令（一帧 CAN）：
 *        帧: 0x200（C620）通道 HORIZONTAL_ESC_ID = 水平电流，其余通道填 0
 *        通道位置 = (ESC_ID - 1) × 2 字节偏移。
 */
HAL_StatusTypeDef CAN_cmd_horizontal(int16_t horizontal_voltage)
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8];
    uint32_t mailbox;

    /* 帧: C620 控制帧 0x200，水平在通道 HORIZONTAL_ESC_ID */
    header.StdId = C620_COMMAND_ID;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;

    memset(data, 0, sizeof(data));
    data[(HORIZONTAL_ESC_ID - 1U) * 2U]     = (uint8_t)((uint16_t)horizontal_voltage >> 8);
    data[(HORIZONTAL_ESC_ID - 1U) * 2U + 1U] = (uint8_t)horizontal_voltage;

    return HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
}

/**
 * @brief 水平电机在线检查：收到过反馈，且最近 100ms 内仍在更新。
 *        电机反馈周期约 1ms，100ms 无新帧即认为掉线。
 */
uint8_t CAN_motor_feedback_ready(void)
{
    uint32_t now = HAL_GetTick();

    return (uint8_t)(motor_measure[HORIZONTAL_MOTOR].received &&
                     (now - motor_measure[HORIZONTAL_MOTOR].last_rx_tick < 100U));
}
