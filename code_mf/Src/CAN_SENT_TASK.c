//
// Created by 21481 on 2026/1/30.
//



#include "cmsis_os.h"
#include "DJI_motors.h"
#include "remote_control.h"
#include "can_receive.h"
#include "dm_motor.h"
#include "GET_RC_TASK.h"

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




        FDCAN_DJI_motors(FRICTION_WHEEL_3510_ID1_GIVEN_CURRENT,
                         FRICTION_WHEEL_3510_ID2_GIVEN_CURRENT,
                         0,
                         0, 0x200, CAN_CHANNEL_2);

        FDCAN_DJI_motors(0,
                         PITCH_6020_ID2_GIVEN_CURRENT,
                         0,
                         0, 0x1FF, CAN_CHANNEL_2);


    }



        osDelay(1);
    }
}



