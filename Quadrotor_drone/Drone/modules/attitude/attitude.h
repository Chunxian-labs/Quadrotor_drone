#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdint.h>

/* 姿态四元数 */
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} AttitudeQuat;

/* 欧拉角，单位：度 */
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} AttitudeEuler;

/*
 * 水平静止时 az 约为 +1 g。
 * 横滚、俯仰由加速度确定，偏航初始化为0。
 * 成功返回1，输入无效返回0。
 */
uint8_t Attitude_Init(AttitudeQuat *quat,float ax, float ay, float az);

/*
 * 六轴互补融合。
 * 加速度单位：g，水平静止时 az 约为 +1。
 * 角速度单位：°/s；dt单位：秒。
 * 成功返回1，失败返回0。
 */
uint8_t Attitude_Update6Axis(AttitudeQuat *quat,float gx, float gy, float gz,float ax, float ay, float az, float dt);

/* 将单位四元数转换成欧拉角 */
void Attitude_QuatToEuler(const AttitudeQuat *quat, AttitudeEuler *euler);

#endif