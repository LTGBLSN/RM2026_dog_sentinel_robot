#include "dm_motor.h"
#include "ERROR_DETECTION.h"
#include "cmsis_os.h"

//
// Created by 21481 on 2026/3/14.
//
void ERROR_DETECTION()
{
    while (1)
    {

        DM_motor_status();//达妙电机离线判断
        //暂无自瞄通讯判断




        osDelay(1);
    }


}





void DM_motor_status()
{
    //第一个电机
    if((HAL_GetTick() - XIAOMI_01_right.last_online_time) > DM_MOTOR_CHECK_TIME )
    {
        XIAOMI_01_right.online_state = DM_MOTOR_DIE ;//离线
    }
    else
    {
        XIAOMI_01_right.online_state = DM_MOTOR_SAFE;//在线

    }


    //第二个电机
    if((HAL_GetTick() - XIAOMI_02_left.last_online_time) > DM_MOTOR_CHECK_TIME )
    {
        XIAOMI_02_left.online_state = DM_MOTOR_DIE ;//离线
    }
    else
    {
        XIAOMI_02_left.online_state = DM_MOTOR_SAFE;//在线

    }


}



