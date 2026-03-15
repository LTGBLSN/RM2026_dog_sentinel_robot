//
// Created by 21481 on 2026/2/12.
//

#include <string.h>
#include "main.h"
#include "AUTO_AIM_TASK.h"
#include "usart.h"
#include "NAV_TASK.h"


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart == &huart7)
//    {
//
//        auto_aim_communication_data_parse(uart7_receive_data);//自瞄数据解析
//
//
//        HAL_UART_Receive_DMA(&huart7, &uart7_receive_data, 1);//继续进行中断接收
//
//    }
//    else if (huart == &huart10)
//    {
//        nav_communication_data_parse(nav_uart_rx_data);
//
//        HAL_UART_Receive_DMA(&huart10, &nav_uart_rx_data, 1);//继续进行中断接收
//    }
//
//
//}



