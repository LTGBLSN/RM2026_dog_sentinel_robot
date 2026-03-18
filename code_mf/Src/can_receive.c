//
// Created by 21481 on 2026/1/30.
//


#include "main.h"
#include "DJI_motors.h"
#include "dm_motor.h"
#include "can_receive.h"

//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/**
  * @brief          HAL库FDCAN接收回调函数
  * @param[in]      hfdcan: FDCAN句柄指针
  * @param[in]      RxFifo0ITs: 中断类型标识
  * @retval         none
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // 检查是否是“收到新消息”中断
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];

        // 1. 从 FIFO0 获取报文
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            return;
        }

        // 2. 判断是哪一路 CAN
        if (hfdcan->Instance == FDCAN1)
        {
            // FDCAN1 逻辑
            switch (rx_header.Identifier) // F4是StdId，H7是Identifier
            {
                case 0x201:
                case 0x202:
                case 0x203:
                case 0x204:
                case 0x205:
                case 0x206:
                case 0x207:
                {
                    uint8_t i = rx_header.Identifier - 0x201;
                    get_motor_measure(&motor_can1_data[i], rx_data);
                    break;
                }

                case 0x11:
                {
                    DM_motors_parse(&XIAOMI_01_right, rx_data);
                }
                default:
                    break;
            }
        }
        else if (hfdcan->Instance == FDCAN2)
        {
            // FDCAN2 逻辑
            switch (rx_header.Identifier)
            {
                case 0x201:
                case 0x202:
                case 0x203:
                case 0x204:
                case 0x205:
                case 0x206:
                case 0x207:
                {
                    uint8_t i = rx_header.Identifier - 0x201;
                    get_motor_measure(&motor_can2_data[i], rx_data);
                    break;
                }
                default:
                    break;
            }
        }
        else if (hfdcan->Instance == FDCAN3)
        {
            // FDCAN3 逻辑
            switch (rx_header.Identifier)
            {
                case 0x201:
                case 0x202:
                case 0x203:
                case 0x204:
                case 0x205:
                case 0x206:
                case 0x207:
                {
                    uint8_t i = rx_header.Identifier - 0x201;
                    get_motor_measure(&motor_can3_data[i], rx_data);
                    break;
                }





                case 0xB:
                {
                    DM_motors_parse(&XIAOMI_01_right, rx_data);
                    XIAOMI_01_right.last_online_time = HAL_GetTick() ;
                    break;
                }

                case 0xC:
                {
                    DM_motors_parse(&XIAOMI_02_left, rx_data);
                    XIAOMI_02_left.last_online_time = HAL_GetTick() ;
                    break;
                }










                default:
                    break;
            }
        }
    }
}


/**
 * @brief  解析达妙电机反馈数据并更新到指定结构体
 * @param  motor_ptr: 指向对应电机结构体的指针 (例如 &DM4340_01)
 * @param  rx_DM_data: 接收到的 8 字节原始数据
 */
void DM_motors_parse(struct dm_motor *motor_ptr, const uint8_t rx_DM_data[8])
{
    if (motor_ptr == NULL) return;

    int p_int;
    int v_int;
    int t_int;

    // 1. 解析原始数据
    motor_ptr->state = (rx_DM_data[0]) >> 4;
    p_int = (rx_DM_data[1] << 8) | rx_DM_data[2];
    v_int = (rx_DM_data[3] << 4) | (rx_DM_data[4] >> 4);
    t_int = ((rx_DM_data[4] & 0xF) << 8) | rx_DM_data[5];

    // 2. 转换并仅更新反馈数成员，不触碰其他成员
    motor_ptr->return_angle = uint_to_float(p_int, DM4340_P_MIN, DM4340_P_MAX, 16);
    motor_ptr->return_speed = uint_to_float(v_int, DM4340_V_MIN, DM4340_V_MAX, 12);
    motor_ptr->return_tor   = uint_to_float(t_int, DM4340_T_MIN, DM4340_T_MAX, 12);
    motor_ptr->Tmos         = (float)(rx_DM_data[6]);
    motor_ptr->Tcoil        = (float)(rx_DM_data[7]);

    // 此时，motor_ptr->target_angle 等其他成员完全不会被改变
}