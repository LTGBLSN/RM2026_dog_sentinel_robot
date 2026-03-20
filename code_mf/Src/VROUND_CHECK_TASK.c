//
// Created by 21481 on 2026/3/20.
//

#include "main.h"
#include "cmsis_os.h"
#include "GET_RC_TASK.h"

void VROUND_CHECK_TASK()
{
    while (1)
    {
        if(rcData.key.v & KEY_PRESSED_OFFSET_Q)
        {
            if(vround_always_speed == 0)
            {
                vround_always_speed = 1 ;
            }
            else
            {
                vround_always_speed = 0;
            }
            osDelay(200);
        }

    osDelay(1);
    }
}

