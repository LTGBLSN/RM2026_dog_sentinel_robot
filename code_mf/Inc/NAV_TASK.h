//
// Created by 21481 on 2026/3/14.
//

#ifndef DM_H723_LIB_NAV_TASK_H
#define DM_H723_LIB_NAV_TASK_H

#include "main.h"

extern uint8_t nav_uart_rx_data;                   // 串口单字节接收变量
extern struct NavReceivePacket nav_rx_packet;      // 最终解析出来的速度数据



#pragma pack(1)
struct NavReceivePacket {
    uint8_t header;   // 0x9A
    float vx;         // 线速度 x
    float vy;         // 线速度 y
    float wz;         // 角速度 z
    uint8_t footer;   // 0x1B
};
#pragma pack()


// 状态定义
#define RX_STATE_WAIT_HEADER 0
#define RX_STATE_RECEIVING   1


void nav_communication_data_parse(uint8_t rx_data);








typedef struct __attribute__((packed)) {
    uint8_t header;
    float x;
    float y;
    float yaw;
} LioOdom_t;


extern LioOdom_t g_lio_odom;
extern uint8_t uart_rx_buf[128]; // 缓冲区大一点，防止溢出








#endif //DM_H723_LIB_NAV_TASK_H
