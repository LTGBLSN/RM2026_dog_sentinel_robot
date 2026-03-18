//
// Created by 21481 on 2026/1/30.
//



#include "cmsis_os.h"
#include "DJI_motors.h"
#include "remote_control.h"
#include "can_receive.h"
#include "dm_motor.h"
#include "GET_RC_TASK.h"
#include "CAN_SENT_TASK.h"
#include "ERROR_DETECTION.h"

void CAN_SENT_TASK()
{


    while (1)
    {
    if(rcData.rc.s[0] == 2 )
    {
        FDCAN_DJI_motors(0, 0, 0, 0, 0x200, CAN_CHANNEL_1);
        FDCAN_DJI_motors(0, 0, 0, 0, 0x1FF, CAN_CHANNEL_1);

        FDCAN_DJI_motors(0, 0, 0, 0, 0x200, CAN_CHANNEL_2);
        FDCAN_DJI_motors(0, 0, 0, 0, 0x1FF, CAN_CHANNEL_2);

        FDCAN_DJI_motors(0, 0, 0, 0, 0x200, CAN_CHANNEL_3);
        FDCAN_DJI_motors(0, 0, 0, 0, 0x1FF, CAN_CHANNEL_3);


        if(rcData.rc.s[1] == 1)
        {
            dm_motor_mode_set(CMD_ENABLE_MODE, XIAOMI_02_left.can_channel,XIAOMI_02_left.can_id);
            dm_motor_mode_set(CMD_ENABLE_MODE, XIAOMI_01_right.can_channel,XIAOMI_01_right.can_id);
        }
        else
        {
            DM_CAN_SENT(DM_NO_CURRENT);
        }



    }
    else if (rcData.rc.s[0] == 3 | rcData.rc.s[0] == 1)
    {
        FDCAN_DJI_motors(CHASSIS_3508_ID1_GIVEN_CURRENT,
                         CHASSIS_3508_ID2_GIVEN_CURRENT,
                         CHASSIS_3508_ID3_GIVEN_CURRENT,
                         CHASSIS_3508_ID4_GIVEN_CURRENT, 0x200, CAN_CHANNEL_1);

        FDCAN_DJI_motors(YAW_6020_ID1_GIVEN_CURRENT,
                         SHOOT_2006_ID6_GIVEN_CURRENT,
                         0,
                         0, 0x1FF, CAN_CHANNEL_1);




        FDCAN_DJI_motors(0,
                         PITCH_6020_ID2_GIVEN_CURRENT,
                         0,
                         0, 0x1FF, CAN_CHANNEL_2);


        FDCAN_DJI_motors(CHASSIS_3508_ID5_GIVEN_CURRENT,
                         CHASSIS_3508_ID6_GIVEN_CURRENT,
                         0,
                         0,
                         0x1FF, CAN_CHANNEL_3);

        DM_CAN_SENT(DM_GIVE_CURRENT);



    }



        osDelay(1);
    }
}




void DM_CAN_SENT(uint8_t DM_can_sent_state)
{
    //第一个电机
    if(XIAOMI_01_right.online_state == DM_MOTOR_SAFE)
    {
        switch (DM_can_sent_state)
        {
            case DM_GIVE_CURRENT:
            {
                Dm_Can_Send(XIAOMI_01_right.can_channel, XIAOMI_01_right.can_id, XIAOMI_01_right.motor_type, XIAOMI_01_right.give_tor);
                break;
            }
            case DM_NO_CURRENT:
            {
                Dm_Can_Send(XIAOMI_01_right.can_channel, XIAOMI_01_right.can_id, XIAOMI_01_right.motor_type, 0.0f);
                break;
            }
            default:
            {
                break;
            }
        }
    }
    else//电机保活
    {
        dm_motor_mode_set(CMD_ENABLE_MODE, XIAOMI_01_right.can_channel,XIAOMI_01_right.can_id);
    }


    //第二个电机
    if(XIAOMI_02_left.online_state == DM_MOTOR_SAFE)
    {
        switch (DM_can_sent_state)
        {
            case DM_GIVE_CURRENT:
            {
                Dm_Can_Send(XIAOMI_02_left.can_channel, XIAOMI_02_left.can_id, XIAOMI_02_left.motor_type, XIAOMI_02_left.give_tor);
                break;
            }
            case DM_NO_CURRENT:
            {
                Dm_Can_Send(XIAOMI_02_left.can_channel, XIAOMI_02_left.can_id, XIAOMI_02_left.motor_type, 0.0f);
                break;
            }
            default:
            {
                break;
            }
        }
    }
    else//电机保活
    {
        dm_motor_mode_set(CMD_ENABLE_MODE, XIAOMI_02_left.can_channel,XIAOMI_02_left.can_id);
    }




}




