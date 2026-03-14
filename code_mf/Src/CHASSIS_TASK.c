//
// Created by 21481 on 2026/3/14.
//
#include "cmsis_os.h"
#include "CHASSIS_TASK.h"
#include "pid.h"
#include "DJI_motors.h"
#include "GET_RC_TASK.h"

pid_type_def chassis_3508_ID1_speed_pid;
pid_type_def chassis_3508_ID2_speed_pid;
pid_type_def chassis_3508_ID3_speed_pid;
pid_type_def chassis_3508_ID4_speed_pid;


void CHASSIS_TASK()
{

    chassis_3508_id1_speed_pid_init();
    chassis_3508_id2_speed_pid_init();
    chassis_3508_id3_speed_pid_init();
    chassis_3508_id4_speed_pid_init();

    while (1)
    {

        //云台速度获取
        gimbal_speed_get();

        //云台转换到底盘
        gimbal_to_chassis_speed_compute();

        //轮子速度解算
        chassis_settlement();
        //pid计算
        motor_chassis_pid_compute();

        osDelay(1);
    }
}

void gimbal_speed_get()
{



    if(rcData.rc.s[0] == 1)
    {
        //在这里写进入导航
//        gimbal_vx =
//        gimbal_vy =

    }
    else
    {
        gimbal_vx = ( (float)rcData.rc.ch[0]/660.0f) * CHASSIS_MAX_VX_SPEED;
        gimbal_vy = ( (float)rcData.rc.ch[1]/660.0f) * CHASSIS_MAX_VY_SPEED;
    }



}




void gimbal_to_chassis_speed_compute()
{
    yaw_angle_difference = (float)(motor_can1_data[4].ecd - YAW_MID_ECD) / (float)(8192 / 360.0f) ;
    yaw_radian_difference = (float)yaw_angle_difference * (float)(M_PI / 180);

    chassis_vx =GIMBAL_2CHASSIS_VX_KP * ( gimbal_vx * (float)cos((double)yaw_radian_difference) - gimbal_vy * (float)sin((double)yaw_radian_difference) );
    chassis_vy =GIMBAL_2CHASSIS_VY_KP * ( gimbal_vx * (float)sin((double)yaw_radian_difference) + gimbal_vy * (float)cos((double)yaw_radian_difference));



}

void chassis_settlement()
{
    CHASSIS_3508_ID1_GIVEN_SPEED = (int16_t)(-chassis_vy + chassis_vx + chassis_vround ) ;
    CHASSIS_3508_ID2_GIVEN_SPEED = (int16_t)(chassis_vy + chassis_vx + chassis_vround) ;
    CHASSIS_3508_ID3_GIVEN_SPEED = (int16_t)(chassis_vy - chassis_vx + chassis_vround) ;
    CHASSIS_3508_ID4_GIVEN_SPEED = (int16_t)(-chassis_vy - chassis_vx + chassis_vround ) ;




}


void motor_chassis_pid_compute()
{
    CHASSIS_3508_ID1_GIVEN_CURRENT = chassis_3508_id1_speed_pid_loop(CHASSIS_3508_ID1_GIVEN_SPEED);
    CHASSIS_3508_ID2_GIVEN_CURRENT = chassis_3508_id2_speed_pid_loop(CHASSIS_3508_ID2_GIVEN_SPEED);
    CHASSIS_3508_ID3_GIVEN_CURRENT = chassis_3508_id3_speed_pid_loop(CHASSIS_3508_ID3_GIVEN_SPEED);
    CHASSIS_3508_ID4_GIVEN_CURRENT = chassis_3508_id4_speed_pid_loop(CHASSIS_3508_ID4_GIVEN_SPEED);

}





//1号电机
void chassis_3508_id1_speed_pid_init(void)
{
    static fp32 chassis_3508_id1_speed_kpkikd[3] = {CHASSIS_3508_ID1_SPEED_PID_KP,CHASSIS_3508_ID1_SPEED_PID_KI,CHASSIS_3508_ID1_SPEED_PID_KD};
    PID_init(&chassis_3508_ID1_speed_pid,PID_POSITION,chassis_3508_id1_speed_kpkikd,CHASSIS_3508_SPEED_PID_OUT_MAX,CHASSIS_3508_SPEED_PID_KI_MAX);

}

int16_t chassis_3508_id1_speed_pid_loop(float chassis_3508_ID1_speed_set_loop)
{
    PID_calc(&chassis_3508_ID1_speed_pid, motor_can1_data[0].speed_rpm, chassis_3508_ID1_speed_set_loop);
    int16_t chassis_3508_ID1_given_current_loop = (int16_t)(chassis_3508_ID1_speed_pid.out);

    return chassis_3508_ID1_given_current_loop ;

}



//2号电机
void chassis_3508_id2_speed_pid_init(void)
{
    static fp32 chassis_3508_id2_speed_kpkikd[3] = {CHASSIS_3508_ID2_SPEED_PID_KP,CHASSIS_3508_ID2_SPEED_PID_KI,CHASSIS_3508_ID2_SPEED_PID_KD};
    PID_init(&chassis_3508_ID2_speed_pid,PID_POSITION,chassis_3508_id2_speed_kpkikd,CHASSIS_3508_SPEED_PID_OUT_MAX,CHASSIS_3508_SPEED_PID_KI_MAX);

}

int16_t chassis_3508_id2_speed_pid_loop(float chassis_3508_ID2_speed_set_loop)
{
    PID_calc(&chassis_3508_ID2_speed_pid, motor_can1_data[1].speed_rpm, chassis_3508_ID2_speed_set_loop);
    int16_t chassis_3508_ID2_given_current_loop = (int16_t)(chassis_3508_ID2_speed_pid.out);

    return chassis_3508_ID2_given_current_loop ;

}



//3号电机
void chassis_3508_id3_speed_pid_init(void)
{
    static fp32 chassis_3508_id3_speed_kpkikd[3] = {CHASSIS_3508_ID3_SPEED_PID_KP,CHASSIS_3508_ID3_SPEED_PID_KI,CHASSIS_3508_ID3_SPEED_PID_KD};
    PID_init(&chassis_3508_ID3_speed_pid,PID_POSITION,chassis_3508_id3_speed_kpkikd,CHASSIS_3508_SPEED_PID_OUT_MAX,CHASSIS_3508_SPEED_PID_KI_MAX);

}

int16_t chassis_3508_id3_speed_pid_loop(float chassis_3508_ID3_speed_set_loop)
{
    PID_calc(&chassis_3508_ID3_speed_pid, motor_can1_data[2].speed_rpm , chassis_3508_ID3_speed_set_loop);
    int16_t chassis_3508_ID3_given_current_loop = (int16_t)(chassis_3508_ID3_speed_pid.out);

    return chassis_3508_ID3_given_current_loop ;

}



//4号电机
void chassis_3508_id4_speed_pid_init(void)
{
    static fp32 chassis_3508_id4_speed_kpkikd[3] = {CHASSIS_3508_ID4_SPEED_PID_KP,CHASSIS_3508_ID4_SPEED_PID_KI,CHASSIS_3508_ID4_SPEED_PID_KD};
    PID_init(&chassis_3508_ID4_speed_pid,PID_POSITION,chassis_3508_id4_speed_kpkikd,CHASSIS_3508_SPEED_PID_OUT_MAX,CHASSIS_3508_SPEED_PID_KI_MAX);

}

int16_t chassis_3508_id4_speed_pid_loop(float chassis_3508_ID4_speed_set_loop)
{
    PID_calc(&chassis_3508_ID4_speed_pid, motor_can1_data[3].speed_rpm , chassis_3508_ID4_speed_set_loop);
    int16_t chassis_3508_ID4_given_current_loop = (int16_t)(chassis_3508_ID4_speed_pid.out);

    return chassis_3508_ID4_given_current_loop ;

}



