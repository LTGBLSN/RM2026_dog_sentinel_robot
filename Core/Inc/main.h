/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern float gyro[3], accel[3], temp;
extern float INS_quat[4] ;
extern float INS_angle[3] ;
extern float INS_degree[3] ;

//shoot
extern int16_t SHOOT_TURN_ON_SPEED ;


extern float SHOOT_2006_ID6_GIVEN_SPEED ;
extern int16_t SHOOT_2006_ID6_GIVEN_CURRENT ;

extern float CHASSIS_3508_ID1_GIVEN_SPEED ;
extern int16_t CHASSIS_3508_ID1_GIVEN_CURRENT ;

extern float CHASSIS_3508_ID2_GIVEN_SPEED ;
extern int16_t CHASSIS_3508_ID2_GIVEN_CURRENT ;

extern float CHASSIS_3508_ID3_GIVEN_SPEED ;
extern int16_t CHASSIS_3508_ID3_GIVEN_CURRENT ;

extern float CHASSIS_3508_ID4_GIVEN_SPEED ;
extern int16_t CHASSIS_3508_ID4_GIVEN_CURRENT ;

extern float CHASSIS_3508_ID5_GIVEN_SPEED ;
extern int16_t CHASSIS_3508_ID5_GIVEN_CURRENT ;

extern float CHASSIS_3508_ID6_GIVEN_SPEED ;
extern int16_t CHASSIS_3508_ID6_GIVEN_CURRENT ;

extern float gimbal_vx ;
extern float gimbal_vy ;

extern float chassis_vx ;
extern float chassis_vy ;
extern float chassis_vround ;

extern float yaw_angle_difference ;
extern float yaw_radian_difference ;

//friction wheel
extern float FRICTION_WHEEL_3510_ID1_GIVEN_SPEED ;
extern float FRICTION_WHEEL_3510_ID2_GIVEN_SPEED ;

extern int16_t FRICTION_WHEEL_3510_ID1_GIVEN_CURRENT ;
extern int16_t FRICTION_WHEEL_3510_ID2_GIVEN_CURRENT ;

extern float YAW_6020_ID1_GIVEN_SPEED ;
extern int16_t YAW_6020_ID1_GIVEN_CURRENT ;
extern float YAW_6020_ID1_GIVEN_ANGLE ;

extern float PITCH_6020_ID2_GIVEN_ANGLE ;
extern float PITCH_6020_ID2_GIVEN_SPEED ;
extern int16_t PITCH_6020_ID2_GIVEN_CURRENT ;

extern float yaw_imu_preprocess ;//yaw轴imu预处理

extern uint8_t uart7_receive_data ;//串口当前接收字节

extern uint32_t odom_rx_time ;


extern int16_t vround_always_speed ;




/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER_OUT2_Pin GPIO_PIN_13
#define POWER_OUT2_GPIO_Port GPIOC
#define POWER_OUT1_Pin GPIO_PIN_14
#define POWER_OUT1_GPIO_Port GPIOC
#define POWER_OUT5V_Pin GPIO_PIN_15
#define POWER_OUT5V_GPIO_Port GPIOC
#define ACC_CS_Pin GPIO_PIN_0
#define ACC_CS_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_3
#define GYRO_CS_GPIO_Port GPIOC
#define ACC_INT_Pin GPIO_PIN_10
#define ACC_INT_GPIO_Port GPIOE
#define GYRO_INT_Pin GPIO_PIN_12
#define GYRO_INT_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
