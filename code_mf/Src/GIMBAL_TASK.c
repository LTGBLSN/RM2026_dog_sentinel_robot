//
// Created by 21481 on 2026/3/14.
//

#include "cmsis_os.h"
#include "GET_RC_TASK.h"
#include "GIMBAL_TASK.h"
#include "pid.h"
#include "DJI_motors.h"
#include "IMU_DATA_GET.h"
#include "AUTO_AIM_TASK.h"


pid_type_def yaw_6020_ID1_speed_pid;
pid_type_def yaw_6020_ID1_angle_pid;

pid_type_def pitch_6020_ID2_speed_pid;
pid_type_def pitch_6020_ID2_angle_pid;

pid_type_def friction_wheel_3510_ID1_speed_pid;
pid_type_def friction_wheel_3510_ID2_speed_pid;

void GIMBAL_TASK()
{

    pitch_speed_from_bmi88_pid_init();
    pitch_angle_pid_init();
    yaw_speed_pid_init();
    yaw_angle_pid_init();

    friction_wheel_3510_id1_speed_pid_init();
    friction_wheel_3510_id2_speed_pid_init();

    while (1)
    {

        friction_wheel_speed_control();
        friction_wheel_pid_control();

        motor_gimbal_angle_compute();
        pid_preprocess();
        motor_gimbal_pid_compute();



        osDelay(1);
    }
}

void motor_gimbal_angle_compute()
{
    //缺鼠标键盘部分
    rc_pitch_input_normalization();
    rc_yaw_input_normalization();



}

void rc_pitch_input_normalization()
{
    // 计算遥控器给出的目标值
    float PITCH_GIVEN_ANGLE_COMPUTE = PITCH_6020_ID2_GIVEN_ANGLE + (PITCH_RC_IN_KP * (float)rcData.rc.ch[3]);


    // 如果开启自瞄且数据有效，覆盖目标值
    if(rcData.rc.s[1] == 1 && auto_aim_rx_packet.distance != -1.0f && auto_aim_rx_packet.distance != 0.0f)
    {
        PITCH_GIVEN_ANGLE_COMPUTE = auto_aim_rx_packet.pitch;
    }

    // 统一限幅 (Clamp)

    if (PITCH_GIVEN_ANGLE_COMPUTE > PITCH_ANGLE_MIN)
    {
        PITCH_GIVEN_ANGLE_COMPUTE = PITCH_ANGLE_MIN;
    }
    else if (PITCH_GIVEN_ANGLE_COMPUTE < PITCH_ANGLE_MAX)
    {
        PITCH_GIVEN_ANGLE_COMPUTE = PITCH_ANGLE_MAX;
    }

    PITCH_6020_ID2_GIVEN_ANGLE = PITCH_GIVEN_ANGLE_COMPUTE;
}


void rc_yaw_input_normalization()
{
    float YAW_GIVEN_ANGLE_COMPUTE = YAW_6020_ID1_GIVEN_ANGLE + (YAW_RC_IN_KP * (float)rcData.rc.ch[2]) ;

    if(rcData.rc.s[1] == 1 && auto_aim_rx_packet.distance != -1.0f && auto_aim_rx_packet.distance != 0.0f)
    {
        YAW_GIVEN_ANGLE_COMPUTE = auto_aim_rx_packet.yaw ;
    }


    if(YAW_GIVEN_ANGLE_COMPUTE > 180.0f)
    {
        YAW_6020_ID1_GIVEN_ANGLE =  YAW_GIVEN_ANGLE_COMPUTE - 360.0f ;
    }
    else if(YAW_GIVEN_ANGLE_COMPUTE < -180.0f)
    {
        YAW_6020_ID1_GIVEN_ANGLE =  YAW_GIVEN_ANGLE_COMPUTE + 360.0f ;
    } else
    {
        YAW_6020_ID1_GIVEN_ANGLE =  YAW_GIVEN_ANGLE_COMPUTE ;
    }
}


void pid_preprocess()
{
    if((YAW_6020_ID1_GIVEN_ANGLE - imu_data_from_board_BMI088_mahony.yaw_degree_angle) < -180.0f )
    {
        yaw_imu_preprocess = imu_data_from_board_BMI088_mahony.yaw_degree_angle - 360.0f ;
    }
    else if((YAW_6020_ID1_GIVEN_ANGLE - imu_data_from_board_BMI088_mahony.yaw_degree_angle) > 180.0f )
    {
        yaw_imu_preprocess = imu_data_from_board_BMI088_mahony.yaw_degree_angle + 360.0f ;
    }
    else
    {
        yaw_imu_preprocess = imu_data_from_board_BMI088_mahony.yaw_degree_angle ;
    }
}





void friction_wheel_speed_control()
{
    if( rcData.rc.s[1] == 2 )
    {
        FRICTION_WHEEL_3510_ID1_GIVEN_SPEED = 0 ;
        FRICTION_WHEEL_3510_ID2_GIVEN_SPEED = 0 ;
    } else
    {
        FRICTION_WHEEL_3510_ID1_GIVEN_SPEED = -FRICTION_WHEEL_SHOOT_SPEED ;
        FRICTION_WHEEL_3510_ID2_GIVEN_SPEED = FRICTION_WHEEL_SHOOT_SPEED ;
    }

}


void friction_wheel_pid_control()
{
    FRICTION_WHEEL_3510_ID1_GIVEN_CURRENT = friction_wheel_3510_id1_speed_pid_loop(FRICTION_WHEEL_3510_ID1_GIVEN_SPEED);
    FRICTION_WHEEL_3510_ID2_GIVEN_CURRENT = friction_wheel_3510_id2_speed_pid_loop(FRICTION_WHEEL_3510_ID2_GIVEN_SPEED);
}


void motor_gimbal_pid_compute()
{
    YAW_6020_ID1_GIVEN_SPEED = yaw_angle_pid_loop(YAW_6020_ID1_GIVEN_ANGLE) ;
    YAW_6020_ID1_GIVEN_CURRENT = (int16_t)yaw_speed_pid_loop(YAW_6020_ID1_GIVEN_SPEED);//速度环

    PITCH_6020_ID2_GIVEN_SPEED = pitch_angle_from_bmi088_pid_loop(PITCH_6020_ID2_GIVEN_ANGLE);//角度环
    PITCH_6020_ID2_GIVEN_CURRENT = (int16_t) (-pitch_speed_from_bmi088_pid_loop(PITCH_6020_ID2_GIVEN_SPEED)); //速度环

}



void yaw_speed_pid_init(void)
{
    static fp32 yaw_6020_id1_speed_kpkikd[3] = {YAW_6020_ID1_SPEED_PID_KP, YAW_6020_ID1_SPEED_PID_KI, YAW_6020_ID1_SPEED_PID_KD};
    PID_init(&yaw_6020_ID1_speed_pid, PID_POSITION, yaw_6020_id1_speed_kpkikd, YAW_6020_ID1_SPEED_PID_OUT_MAX, YAW_6020_ID1_SPEED_PID_KI_MAX);

}

float yaw_speed_pid_loop(float YAW_6020_ID1_speed_set_loop)
{
    PID_calc(&yaw_6020_ID1_speed_pid, imu_data_from_board_BMI088_mahony.yaw_radian_vel , YAW_6020_ID1_speed_set_loop);
    int16_t yaw_6020_ID1_given_current_loop = (int16_t)(yaw_6020_ID1_speed_pid.out);

    return yaw_6020_ID1_given_current_loop ;

}


void yaw_angle_pid_init(void)
{
    static fp32 yaw_6020_id1_angle_kpkikd[3] = {YAW_6020_ID1_ANGLE_PID_KP, YAW_6020_ID1_ANGLE_PID_KI, YAW_6020_ID1_ANGLE_PID_KD};
    PID_init(&yaw_6020_ID1_angle_pid, PID_POSITION, yaw_6020_id1_angle_kpkikd, YAW_6020_ID1_ANGLE_PID_OUT_MAX, YAW_6020_ID1_ANGLE_PID_KI_MAX);

}

float yaw_angle_pid_loop(float YAW_6020_ID1_angle_set_loop)
{
    PID_calc(&yaw_6020_ID1_angle_pid, yaw_imu_preprocess , YAW_6020_ID1_angle_set_loop);
    float yaw_6020_ID1_given_speed_loop = (float)(yaw_6020_ID1_angle_pid.out);

    return yaw_6020_ID1_given_speed_loop ;

}






void pitch_speed_from_bmi88_pid_init(void)
{
    static fp32 pitch_6020_id2_speed_kpkikd[3] = {PITCH_6020_ID2_SPEED_PID_KP,PITCH_6020_ID2_SPEED_PID_KI,PITCH_6020_ID2_SPEED_PID_KD};
    PID_init(&pitch_6020_ID2_speed_pid,PID_POSITION,pitch_6020_id2_speed_kpkikd,PITCH_6020_ID2_SPEED_PID_OUT_MAX,PITCH_6020_ID2_SPEED_PID_KI_MAX);

}

float pitch_speed_from_bmi088_pid_loop(float PITCH_6020_ID2_speed_set_loop)
{
    PID_calc(&pitch_6020_ID2_speed_pid, imu_data_from_board_BMI088_mahony.pitch_radian_vel , PITCH_6020_ID2_speed_set_loop);
    int16_t pitch_6020_ID2_given_current_loop = (int16_t)(pitch_6020_ID2_speed_pid.out);

    return pitch_6020_ID2_given_current_loop ;

}


void pitch_angle_pid_init(void)
{
    static fp32 pitch_6020_id2_angle_kpkikd[3] = {PITCH_6020_ID2_ANGLE_PID_KP,PITCH_6020_ID2_ANGLE_PID_KI,PITCH_6020_ID2_ANGLE_PID_KD};
    PID_init(&pitch_6020_ID2_angle_pid,PID_POSITION,pitch_6020_id2_angle_kpkikd,PITCH_6020_ID2_ANGLE_PID_OUT_MAX,PITCH_6020_ID2_ANGLE_PID_KI_MAX);

}

float pitch_angle_from_bmi088_pid_loop(float PITCH_6020_ID2_angle_set_loop)
{
    PID_calc(&pitch_6020_ID2_angle_pid, imu_data_from_board_BMI088_mahony.pitch_degree_angle , PITCH_6020_ID2_angle_set_loop);
    float pitch_6020_ID2_given_speed_loop = (float)(pitch_6020_ID2_angle_pid.out);

    return pitch_6020_ID2_given_speed_loop ;

}







//friction wheel
void friction_wheel_3510_id1_speed_pid_init(void)
{
    static fp32 friction_wheel_3510_id1_speed_kpkikd[3] = {FRICTION_WHEEL_3510_ID1_SPEED_PID_KP,FRICTION_WHEEL_3510_ID1_SPEED_PID_KI,FRICTION_WHEEL_3510_ID1_SPEED_PID_KD};
    PID_init(&friction_wheel_3510_ID1_speed_pid,PID_POSITION,friction_wheel_3510_id1_speed_kpkikd,FRICTION_WHEEL_3510_ID1_SPEED_PID_OUT_MAX,FRICTION_WHEEL_3510_ID1_SPEED_PID_KI_MAX);

}

int16_t friction_wheel_3510_id1_speed_pid_loop(int16_t friction_wheel_3510_id1_speed_set_loop)
{
    PID_calc(&friction_wheel_3510_ID1_speed_pid, motor_can2_data[0].speed_rpm , friction_wheel_3510_id1_speed_set_loop);
    int16_t friction_wheel_3510_id1_given_current_loop = (int16_t)(friction_wheel_3510_ID1_speed_pid.out);

    return friction_wheel_3510_id1_given_current_loop ;

}



void friction_wheel_3510_id2_speed_pid_init(void)
{
    static fp32 friction_wheel_3510_id2_speed_kpkikd[3] = {FRICTION_WHEEL_3510_ID2_SPEED_PID_KP,FRICTION_WHEEL_3510_ID2_SPEED_PID_KI,FRICTION_WHEEL_3510_ID2_SPEED_PID_KD};
    PID_init(&friction_wheel_3510_ID2_speed_pid,PID_POSITION,friction_wheel_3510_id2_speed_kpkikd,FRICTION_WHEEL_3510_ID2_SPEED_PID_OUT_MAX,FRICTION_WHEEL_3510_ID2_SPEED_PID_KI_MAX);

}

int16_t friction_wheel_3510_id2_speed_pid_loop(int16_t friction_wheel_3510_id2_speed_set_loop)
{
    PID_calc(&friction_wheel_3510_ID2_speed_pid, motor_can2_data[1].speed_rpm , friction_wheel_3510_id2_speed_set_loop);
    int16_t friction_wheel_3510_id2_given_current_loop = (int16_t)(friction_wheel_3510_ID2_speed_pid.out);

    return friction_wheel_3510_id2_given_current_loop ;

}



