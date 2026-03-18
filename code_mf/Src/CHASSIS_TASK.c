//
// Created by 21481 on 2026/3/14.
//
#include "cmsis_os.h"
#include "CHASSIS_TASK.h"
#include "pid.h"
#include "DJI_motors.h"
#include "GET_RC_TASK.h"
#include "dm_motor.h"
#include "NAV_TASK.h"
#include "NAV_POSE_SET_TASK.h"

pid_type_def chassis_3508_ID1_speed_pid;
pid_type_def chassis_3508_ID2_speed_pid;
pid_type_def chassis_3508_ID3_speed_pid;
pid_type_def chassis_3508_ID4_speed_pid;
pid_type_def chassis_3508_ID5_speed_pid;
pid_type_def chassis_3508_ID6_speed_pid;

pid_type_def chassis_3508_ID1_nav_vx_pose_pid;
pid_type_def chassis_3508_ID1_nav_vy_pose_pid;

pid_type_def chassis_follow_gimbal_pid;

pid_type_def xiaomi_01_speed_pid;
pid_type_def xiaomi_02_speed_pid;



void CHASSIS_TASK()
{

    chassis_3508_id1_speed_pid_init();
    chassis_3508_id2_speed_pid_init();
    chassis_3508_id3_speed_pid_init();
    chassis_3508_id4_speed_pid_init();
    chassis_3508_id5_speed_pid_init();
    chassis_3508_id6_speed_pid_init();

    chassis_nav_vx_pose_pid_init();
    chassis_nav_vy_pose_pid_init();

    chassis_follow_gimbal_pid_init();

    dm_motor_init();

    xiaomi_01_pid_init();
    xiaomi_02_pid_init();



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



        xiaomi_given_angle_compute();

        xiaomi_motor_pid_compute();



        osDelay(1);
    }
}

void gimbal_speed_get()
{



    if(rcData.rc.s[0] == 1)
    {
        //在这里写进入导航
        if(pose_key < 9)
        {
            gimbal_vx = chassis_3508_id1_nav_vx_pose_pid_loop(pose_set_xy[0][pose_key]) ;
            gimbal_vy = -chassis_3508_id1_nav_vy_pose_pid_loop(pose_set_xy[1][pose_key]) ;
        }
        else
        {
            gimbal_vx = 0.0f ;
            gimbal_vy = 0.0f ;
        }




    }
    else
    {
        gimbal_vx = ( (float)rcData.rc.ch[1]/660.0f) * CHASSIS_MAX_VX_SPEED;
        gimbal_vy = ( (float)rcData.rc.ch[0]/660.0f) * CHASSIS_MAX_VY_SPEED;
    }



}




//void gimbal_to_chassis_speed_compute()
//{
//    yaw_angle_difference = (float)(motor_can1_data[4].ecd - YAW_MID_ECD) / (float)(8192 / 360.0f) ;
//    yaw_radian_difference = (float)yaw_angle_difference * (float)(M_PI / 180);
//
//    chassis_vx = GIMBAL_2CHASSIS_VX_KP * ( gimbal_vx * (float)cos((double)yaw_radian_difference) - gimbal_vy * (float)sin((double)yaw_radian_difference) );
//    chassis_vy = GIMBAL_2CHASSIS_VY_KP * ( gimbal_vx * (float)sin((double)yaw_radian_difference) + gimbal_vy * (float)cos((double)yaw_radian_difference));
//
//
//
//}


void gimbal_to_chassis_speed_compute()
{
    // 1. 确保角度范围和方向：左转为正，右转为负
    // 假设 8192 是电机一圈的数值，YAW_MID_ECD 是云台正对底盘前方的编码器值
    float relative_angle_rad = ((float )motor_can1_data[4].ecd - YAW_MID_ECD) * (2.0f * (float )M_PI / 8192.0f);

    // 如果发现方向反了（比如左转角度反而减小），就加个负号：
    // relative_angle_rad = -relative_angle_rad;

    // 2. 旋转矩阵解耦 (坐标系变换版)
    // gimbal_vx: 遥控器前进方向
    // gimbal_vy: 遥控器左右方向（左为正）
    float cos_theta = cosf(relative_angle_rad);
    float sin_theta = sinf(relative_angle_rad);

    // 标准解耦公式
    chassis_vx = GIMBAL_2CHASSIS_VX_KP * ( gimbal_vx * cos_theta + gimbal_vy * sin_theta) ;
    chassis_vy = GIMBAL_2CHASSIS_VY_KP * (-gimbal_vx * sin_theta + gimbal_vy * cos_theta);
    chassis_vround = chassis_follow_gimbal_pid_loop(YAW_MID_ECD);

    // 3. 加上小陀螺转速 (Wz)
    // chassis_vw = spinning_speed;
}

void chassis_settlement()
{
    CHASSIS_3508_ID1_GIVEN_SPEED = (int16_t)( chassis_vy - chassis_vx  + chassis_vround) ;
    CHASSIS_3508_ID2_GIVEN_SPEED = (int16_t)( chassis_vy + chassis_vx  + chassis_vround) ;
    CHASSIS_3508_ID3_GIVEN_SPEED = (int16_t)(-chassis_vy + chassis_vx  + chassis_vround) ;
    CHASSIS_3508_ID4_GIVEN_SPEED = (int16_t)(-chassis_vy - chassis_vx  + chassis_vround) ;
    CHASSIS_3508_ID5_GIVEN_SPEED = (int16_t)(- chassis_vx  - chassis_vround) ;
    CHASSIS_3508_ID6_GIVEN_SPEED = (int16_t)(+ chassis_vx  - chassis_vround) ;




}

void xiaomi_given_angle_compute()
{
    if(rcData.rc.ch[4] > 200)
    {
//        XIAOMI_02_left.give_angle = 0.8f ;
//        XIAOMI_01_right.give_angle = -0.8f ;
        XIAOMI_01_right.give_angle = -0.8f ;
        XIAOMI_02_left.give_angle  = 0.8f ;


    }
    else
    {
        XIAOMI_01_right.give_angle = -0.2f ;
        XIAOMI_02_left.give_angle  = 0.2f ;


    }
}

void xiaomi_motor_pid_compute()
{
    XIAOMI_01_right.give_tor = xiaomi_01_pid_loop(XIAOMI_01_right.give_angle);
    XIAOMI_02_left.give_tor = xiaomi_02_pid_loop(XIAOMI_02_left.give_angle);

}


void motor_chassis_pid_compute()
{
    CHASSIS_3508_ID1_GIVEN_CURRENT = chassis_3508_id1_speed_pid_loop(CHASSIS_3508_ID1_GIVEN_SPEED);
    CHASSIS_3508_ID2_GIVEN_CURRENT = chassis_3508_id2_speed_pid_loop(CHASSIS_3508_ID2_GIVEN_SPEED);
    CHASSIS_3508_ID3_GIVEN_CURRENT = chassis_3508_id3_speed_pid_loop(CHASSIS_3508_ID3_GIVEN_SPEED);
    CHASSIS_3508_ID4_GIVEN_CURRENT = chassis_3508_id4_speed_pid_loop(CHASSIS_3508_ID4_GIVEN_SPEED);
    CHASSIS_3508_ID5_GIVEN_CURRENT = chassis_3508_id5_speed_pid_loop(CHASSIS_3508_ID5_GIVEN_SPEED);
    CHASSIS_3508_ID6_GIVEN_CURRENT = chassis_3508_id6_speed_pid_loop(CHASSIS_3508_ID6_GIVEN_SPEED);


}


void xiaomi_01_pid_init(void)
{
    static fp32 xiaomi_01_speed_kpkikd[3] = {XIAOMI_ID1_PID_KP, XIAOMI_ID1_PID_KI, XIAOMI_ID1_PID_KD};
    PID_init(&xiaomi_01_speed_pid,PID_POSITION,xiaomi_01_speed_kpkikd,XIAOMI_PID_OUT_MAX,XIAOMI_PID_KI_MAX);

}

float xiaomi_01_pid_loop(float xiaomi_01_speed_set_loop)
{
    PID_calc(&xiaomi_01_speed_pid, XIAOMI_01_right.return_angle, xiaomi_01_speed_set_loop);
    float xiaomi_01_given_current_loop = (float )(xiaomi_01_speed_pid.out);

    return xiaomi_01_given_current_loop ;

}


void xiaomi_02_pid_init(void)
{
    static fp32 xiaomi_02_speed_kpkikd[3] = {XIAOMI_ID2_PID_KP, XIAOMI_ID2_PID_KI, XIAOMI_ID2_PID_KD};
    PID_init(&xiaomi_02_speed_pid,PID_POSITION,xiaomi_02_speed_kpkikd,XIAOMI_PID_OUT_MAX,XIAOMI_PID_KI_MAX);

}

float xiaomi_02_pid_loop(float xiaomi_02_speed_set_loop)
{
    PID_calc(&xiaomi_02_speed_pid, XIAOMI_02_left.return_angle, xiaomi_02_speed_set_loop);
    float xiaomi_02_given_current_loop = (float )(xiaomi_02_speed_pid.out);

    return xiaomi_02_given_current_loop ;

}








void chassis_nav_vx_pose_pid_init(void)
{
    static fp32 chassis_nav_vx_pose_kpkikd[3] = {CHASSIS_3508_NAV_VX_POSE_PID_KP,CHASSIS_3508_NAV_VX_POSE_PID_KI,CHASSIS_3508_NAV_VX_POSE_PID_KD};
    PID_init(&chassis_3508_ID1_nav_vx_pose_pid,PID_POSITION,chassis_nav_vx_pose_kpkikd,CHASSIS_3508_NAV_VX_POSE_PID_OUT_MAX,CHASSIS_3508_NAV_VX_POSE_PID_KI_MAX);

}

float chassis_3508_id1_nav_vx_pose_pid_loop(float chassis_3508_ID1_nav_vx_pose_set_loop)
{
    PID_calc(&chassis_3508_ID1_nav_vx_pose_pid, g_lio_odom.x, chassis_3508_ID1_nav_vx_pose_set_loop);
    float chassis_3508_ID1_nav_vx_pose_given_current_loop = (float)(chassis_3508_ID1_nav_vx_pose_pid.out);

    return chassis_3508_ID1_nav_vx_pose_given_current_loop ;

}


void chassis_nav_vy_pose_pid_init(void)
{
    static fp32 chassis_nav_vy_pose_kpkikd[3] = {CHASSIS_3508_NAV_VY_POSE_PID_KP,CHASSIS_3508_NAV_VY_POSE_PID_KI,CHASSIS_3508_NAV_VY_POSE_PID_KD};
    PID_init(&chassis_3508_ID1_nav_vy_pose_pid,PID_POSITION,chassis_nav_vy_pose_kpkikd,CHASSIS_3508_NAV_VY_POSE_PID_OUT_MAX,CHASSIS_3508_NAV_VY_POSE_PID_KI_MAX);

}

float chassis_3508_id1_nav_vy_pose_pid_loop(float chassis_3508_ID1_nav_vy_pose_set_loop)
{
    PID_calc(&chassis_3508_ID1_nav_vy_pose_pid, g_lio_odom.y, chassis_3508_ID1_nav_vy_pose_set_loop);
    float chassis_3508_ID1_nav_vy_pose_given_current_loop = (float)(chassis_3508_ID1_nav_vy_pose_pid.out);

    return chassis_3508_ID1_nav_vy_pose_given_current_loop ;

}


void chassis_follow_gimbal_pid_init(void)
{
    static fp32 chassis_follow_gimbal_kpkikd[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP,CHASSIS_FOLLOW_GIMBAL_PID_KI,CHASSIS_FOLLOW_GIMBAL_PID_KD};
    PID_init(&chassis_follow_gimbal_pid, PID_POSITION, chassis_follow_gimbal_kpkikd, CHASSIS_FOLLOW_GIMBAL_PID_OUT_MAX, CHASSIS_FOLLOW_GIMBAL_PID_KI_MAX);

}

float chassis_follow_gimbal_pid_loop(float chassis_follow_gimbal_set_loop)
{
    PID_calc(&chassis_follow_gimbal_pid, motor_can1_data[4].ecd, chassis_follow_gimbal_set_loop);
    float chassis_follow_gimbal_given_current_loop = (float)(chassis_follow_gimbal_pid.out);

    return chassis_follow_gimbal_given_current_loop ;

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


//5号电机
void chassis_3508_id5_speed_pid_init(void)
{
    static fp32 chassis_3508_id5_speed_kpkikd[3] = {CHASSIS_3508_ID5_SPEED_PID_KP,CHASSIS_3508_ID5_SPEED_PID_KI,CHASSIS_3508_ID5_SPEED_PID_KD};
    PID_init(&chassis_3508_ID5_speed_pid,PID_POSITION,chassis_3508_id5_speed_kpkikd,CHASSIS_3508_SPEED_PID_OUT_MAX,CHASSIS_3508_SPEED_PID_KI_MAX);

}

int16_t chassis_3508_id5_speed_pid_loop(float chassis_3508_ID5_speed_set_loop)
{
    PID_calc(&chassis_3508_ID5_speed_pid, motor_can3_data[4].speed_rpm , chassis_3508_ID5_speed_set_loop);
    int16_t chassis_3508_ID5_given_current_loop = (int16_t)(chassis_3508_ID5_speed_pid.out);

    return chassis_3508_ID5_given_current_loop ;

}



//6号电机
void chassis_3508_id6_speed_pid_init(void)
{
    static fp32 chassis_3508_id6_speed_kpkikd[3] = {CHASSIS_3508_ID6_SPEED_PID_KP,CHASSIS_3508_ID6_SPEED_PID_KI,CHASSIS_3508_ID6_SPEED_PID_KD};
    PID_init(&chassis_3508_ID6_speed_pid,PID_POSITION,chassis_3508_id6_speed_kpkikd,CHASSIS_3508_SPEED_PID_OUT_MAX,CHASSIS_3508_SPEED_PID_KI_MAX);

}

int16_t chassis_3508_id6_speed_pid_loop(float chassis_3508_ID6_speed_set_loop)
{
    PID_calc(&chassis_3508_ID6_speed_pid, motor_can3_data[5].speed_rpm , chassis_3508_ID6_speed_set_loop);
    int16_t chassis_3508_ID6_given_current_loop = (int16_t)(chassis_3508_ID6_speed_pid.out);

    return chassis_3508_ID6_given_current_loop ;

}


