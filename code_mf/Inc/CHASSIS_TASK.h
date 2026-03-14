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





#define YAW_MID_ECD 5103

#define CHASSIS_MAX_VX_SPEED 3.0f //底盘最大移动速度，单位米每秒
#define CHASSIS_MAX_VY_SPEED 3.0f //底盘最大移动速度，单位米每秒

#define GIMBAL_2CHASSIS_VX_KP 1500.0f //需要换算
#define GIMBAL_2CHASSIS_VY_KP 1500.0f //需要换算

extern pid_type_def chassis_3508_ID1_speed_pid;
extern pid_type_def chassis_3508_ID2_speed_pid;
extern pid_type_def chassis_3508_ID3_speed_pid;
extern pid_type_def chassis_3508_ID4_speed_pid;



void gimbal_speed_get();


void gimbal_to_chassis_speed_compute();

void chassis_settlement();

void motor_chassis_pid_compute();


void chassis_3508_id1_speed_pid_init(void);
int16_t chassis_3508_id1_speed_pid_loop(float chassis_3508_ID1_speed_set_loop);
void chassis_3508_id2_speed_pid_init(void);
int16_t chassis_3508_id2_speed_pid_loop(float chassis_3508_ID2_speed_set_loop);
void chassis_3508_id3_speed_pid_init(void);
int16_t chassis_3508_id3_speed_pid_loop(float chassis_3508_ID3_speed_set_loop);
void chassis_3508_id4_speed_pid_init(void);
int16_t chassis_3508_id4_speed_pid_loop(float chassis_3508_ID4_speed_set_loop);


#endif //DM_H723_LIB_CHASSIS_TASK_H
