#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>
typedef struct
{
    float m1;
    float m2;
    float m3;
    float m4;
}MotorOutput_t;
void Motor_Init(void);
void Motor_Stop(void);
/* 输入为占空比百分数，范围 0～100 */
void Motor_SetOutput(float a1,float a2,float a3,float a4);
void Motor_Mix(float throttle,float roll,float pitch,float yaw,MotorOutput_t *output);
uint8_t Motor_IsOutputAllZero(void);
#endif