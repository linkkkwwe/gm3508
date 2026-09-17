#include "can_comm.h"
#include "can.h"

/**
 * @brief CAN1 初始化：配滤波器 → 启动 → 开接收中断。
 *
 * 滤波器配置说明（掩码模式，32 位尺度）：
 *   硬件滤波器只做"粗筛"——决定哪些帧有资格进 FIFO0。
 *   MaskIdLow = 0x0006 的含义：对标准帧 11 位 ID 里的 bit1(RTR)、
 *   bit2(IDE) 做检查且必须为 0（即只放行标准数据帧），
 *   而 ID 本身不检查（全放行）。真正的 ID 筛选由
 *   HAL_CAN_RxFifo0MsgPendingCallback 里的 switch 完成（软件精筛）。
 */
HAL_StatusTypeDef can_comm_init(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterActivation = ENABLE;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0006;   /* 只放行标准数据帧，ID 全收 */
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;

    /* 三步都检查返回值：任一失败立即上报，避免"以为配好其实没配" */
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
        return HAL_ERROR;
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
        return HAL_ERROR;
    return HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}
