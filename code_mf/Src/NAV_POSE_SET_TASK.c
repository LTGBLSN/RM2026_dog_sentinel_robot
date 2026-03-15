//
// Created by 21481 on 2026/3/15.
//


#include "cmsis_os.h"
#include "NAV_TASK.h"

int16_t pose_key = 0;
float pose_set_xy[2][9] =
        {
        {0.0f, 3.0f, 1.0f, 3.8f, 2.4f, 4.9f, 3.0f, 5.0f, 5.0f},//x
        {7.5f, 7.5f, -1.0f, -1.0f, 7.5f, 7.5f, -1.0f, -1.0f, 6.6f},//y
};


void NAV_POSE_SET_TASK()
{
    while(1)
    {
        if(pose_key < 9 )
        {
            if( fabsf((g_lio_odom.x - pose_set_xy[0][pose_key])) < 0.6f && fabsf((g_lio_odom.y - pose_set_xy[1][pose_key])) < 0.6f)
            {
                pose_key++;
            }
        }

        osDelay(1);
    }
}

