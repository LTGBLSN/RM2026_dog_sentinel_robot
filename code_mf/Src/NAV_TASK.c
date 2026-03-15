//
// Created by 21481 on 2026/3/14.
//


#include <string.h>
#include "NAV_TASK.h"



// 2. 定义变量（导航）
struct NavReceivePacket nav_rx_packet;      // 最终解析出来的速度数据
uint8_t nav_rx_buffer[sizeof(struct NavReceivePacket)]; // 接收缓冲区
static uint8_t nav_rx_state = 0;            // 状态机状态
static uint8_t nav_rx_index = 0;            // 索引
uint8_t nav_uart_rx_data;                   // 串口单字节接收变量



LioOdom_t g_lio_odom;
uint8_t uart_rx_buf[128]; // 缓冲区大一点，防止溢出





/**
 * @brief 导航数据解析函数，由串口回调函数调用
 * @param rx_data 串口接收到的单个字节
 */
void nav_communication_data_parse(uint8_t rx_data)
{
    switch (nav_rx_state)
    {
        case RX_STATE_WAIT_HEADER:
            // 等待包头 0x9A
            if (rx_data == 0x9A)
            {
                nav_rx_buffer[0] = rx_data;
                nav_rx_index = 1;
                nav_rx_state = RX_STATE_RECEIVING; // 进入接收状态
            }
            break;

        case RX_STATE_RECEIVING:
            // 填充缓冲区
            nav_rx_buffer[nav_rx_index++] = rx_data;

            // 检查是否接收够了 14 个字节 (1+4+4+4+1)
            if (nav_rx_index == sizeof(struct NavReceivePacket))
            {
                // 校验帧尾是否为 0x1B
                if (nav_rx_buffer[sizeof(struct NavReceivePacket) - 1] == 0x1B)
                {
                    // 校验成功，拷贝数据到结构体
                    // 此时 nav_rx_packet.vx, .vy, .wz 就可以直接用了
                    memcpy(&nav_rx_packet, nav_rx_buffer, sizeof(struct NavReceivePacket));

                    // 【在此处添加你的逻辑】
                    // 例如：update_motor_speed(nav_rx_packet.vx, nav_rx_packet.vy, nav_rx_packet.wz);
                }
                else
                {
                    // 帧尾不对，说明数据错位了，丢弃
                }

                // 重置状态机
                nav_rx_state = RX_STATE_WAIT_HEADER;
                nav_rx_index = 0;
            }
            break;

        default:
            nav_rx_state = RX_STATE_WAIT_HEADER;
            nav_rx_index = 0;
            break;
    }
}



