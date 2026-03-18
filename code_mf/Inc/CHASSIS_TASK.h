//
// Created by 21481 on 2026/3/14.
//

#ifndef DM_H723_LIB_CHASSIS_TASK_H
#define DM_H723_LIB_CHASSIS_TASK_H


#include "pid.h"



#define CHASSIS_3508_SPEED_PID_OUT_MAX   16384.0f   //max+_16384,for all chassis 3508 motors form id1 to id4
#define CHASSIS_3508_SPEED_PID_KI_MAX   1000.0f


#define CHASSIS_3508_ID1_SPEED_PID_KP   5.0f
#define CHASSIS_3508_ID1_SPEED_PID_KI   0.10f
#define CHASSIS_3508_ID1_SPEED_PID_KD   0.0f

#define CHASSIS_3508_ID2_SPEED_PID_KP   5.0f
#define CHASSIS_3508_ID2_SPEED_PID_KI   0.10f
#define CHASSIS_3508_ID2_SPEED_PID_KD   0.0f

#define CHASSIS_3508_ID3_SPEED_PID_KP   5.0f
#define CHASSIS_3508_ID3_SPEED_PID_KI   0.10f
#define CHASSIS_3508_ID3_SPEED_PID_KD   0.0f

#define CHASSIS_3508_ID4_SPEED_PID_KP   5.0f
#define CHASSIS_3508_ID4_SPEED_PID_KI   0.10f
#define CHASSIS_3508_ID4_SPEED_PID_KD   0.0f

#define CHASSIS_3508_ID5_SPEED_PID_KP   5.0f
#define CHASSIS_3508_ID5_SPEED_PID_KI   0.10f
#define CHASSIS_3508_ID5_SPEED_PID_KD   0.0f

#define CHASSIS_3508_ID6_SPEED_PID_KP   5.0f
#define CHASSIS_3508_ID6_SPEED_PID_KI   0.10f
#define CHASSIS_3508_ID6_SPEED_PID_KD   0.0f

#define XIAOMI_ID1_PID_KP   300.0f
#define XIAOMI_ID1_PID_KI   0.2f
#define XIAOMI_ID1_PID_KD   2500.0f

#define XIAOMI_ID2_PID_KP   300.0f
#define XIAOMI_ID2_PID_KI   0.2f
#define XIAOMI_ID2_PID_KD   2500.0f

#define XIAOMI_PID_OUT_MAX   18.0f
#define XIAOMI_PID_KI_MAX   8.0f


#define CHASSIS_3508_NAV_VX_POSE_PID_KP   0.4f
#define CHASSIS_3508_NAV_VX_POSE_PID_KI   0.0f
#define CHASSIS_3508_NAV_VX_POSE_PID_KD   0.0f
#define CHASSIS_3508_NAV_VX_POSE_PID_OUT_MAX   2.0f
#define CHASSIS_3508_NAV_VX_POSE_PID_KI_MAX   0.0f

#define CHASSIS_3508_NAV_VY_POSE_PID_KP   0.3f
#define CHASSIS_3508_NAV_VY_POSE_PID_KI   0.0f
#define CHASSIS_3508_NAV_VY_POSE_PID_KD   0.0f
#define CHASSIS_3508_NAV_VY_POSE_PID_OUT_MAX   1.0f
#define CHASSIS_3508_NAV_VY_POSE_PID_KI_MAX   0.0f

#define CHASSIS_FOLLOW_GIMBAL_PID_KP   5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI   0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD   0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_OUT_MAX   6000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI_MAX   0.0f



#define YAW_MID_ECD 5103.0f

#define CHASSIS_MAX_VX_SPEED 3.0f //底盘最大移动速度，单位米每秒
#define CHASSIS_MAX_VY_SPEED 3.0f //底盘最大移动速度，单位米每秒

#define GIMBAL_2CHASSIS_VX_KP 3000.0f //需要换算
#define GIMBAL_2CHASSIS_VY_KP 3000.0f //需要换算




extern pid_type_def chassis_3508_ID1_speed_pid;
extern pid_type_def chassis_3508_ID2_speed_pid;
extern pid_type_def chassis_3508_ID3_speed_pid;
extern pid_type_def chassis_3508_ID4_speed_pid;
extern pid_type_def chassis_3508_ID5_speed_pid;
extern pid_type_def chassis_3508_ID6_speed_pid;


extern pid_type_def chassis_3508_ID1_nav_vx_pose_pid;
extern pid_type_def chassis_3508_ID1_nav_vy_pose_pid;
extern pid_type_def chassis_follow_gimbal_pid;



void gimbal_speed_get();

void xiaomi_given_angle_compute();

void xiaomi_motor_pid_compute();

void gimbal_to_chassis_speed_compute();

void chassis_settlement();

void motor_chassis_pid_compute();

void xiaomi_01_pid_init(void);
void xiaomi_02_pid_init(void);

float xiaomi_01_pid_loop(float xiaomi_01_speed_set_loop);
float xiaomi_02_pid_loop(float xiaomi_02_speed_set_loop);


void chassis_3508_id1_speed_pid_init(void);
int16_t chassis_3508_id1_speed_pid_loop(float chassis_3508_ID1_speed_set_loop);
void chassis_3508_id2_speed_pid_init(void);
int16_t chassis_3508_id2_speed_pid_loop(float chassis_3508_ID2_speed_set_loop);
void chassis_3508_id3_speed_pid_init(void);
int16_t chassis_3508_id3_speed_pid_loop(float chassis_3508_ID3_speed_set_loop);
void chassis_3508_id4_speed_pid_init(void);
int16_t chassis_3508_id4_speed_pid_loop(float chassis_3508_ID4_speed_set_loop);
void chassis_3508_id5_speed_pid_init(void);
int16_t chassis_3508_id5_speed_pid_loop(float chassis_3508_ID5_speed_set_loop);
void chassis_3508_id6_speed_pid_init(void);
int16_t chassis_3508_id6_speed_pid_loop(float chassis_3508_ID6_speed_set_loop);



void chassis_nav_vx_pose_pid_init(void);
float chassis_3508_id1_nav_vx_pose_pid_loop(float chassis_3508_ID1_nav_vx_pose_set_loop);
void chassis_nav_vy_pose_pid_init(void);
float chassis_3508_id1_nav_vy_pose_pid_loop(float chassis_3508_ID1_nav_vy_pose_set_loop);

void chassis_follow_gimbal_pid_init(void);
float chassis_follow_gimbal_pid_loop(float chassis_follow_gimbal_set_loop);





#endif //DM_H723_LIB_CHASSIS_TASK_H
