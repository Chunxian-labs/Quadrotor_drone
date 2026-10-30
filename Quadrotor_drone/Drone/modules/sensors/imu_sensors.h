#ifndef _IMU_SENSORS_H
#define _IMU_SENSORS_H
#include <stdint.h>
/*
 * ax = -芯片AY，ay = -芯片AX，az = +芯片AZ；
 * gx = +芯片GY，gy = +芯片GX，gz = -芯片GZ。
 *
 * 加速度单位：g，水平静止时 az 约为 +1。
 * 角速度单位：°/s。
 */
typedef struct 
{
    float ax, ay, az;  /* 单位：g */
    float gx, gy, gz;  /* 单位：°/s */
}IMU_Data;
/* 初始化芯片，并清除本模块的校准参数 */
void IMU_Sensor_Init(void);
/* 读取芯片标识 */
uint8_t IMU_Sensor_GetID(void);
/* 阻塞校准：仅在停止状态调用 */
uint8_t IMU_Sensor_CalibrateAcc(void);   /* 必须水平且静止 */
uint8_t IMU_Sensor_CalibrateGyro(void);  /* 必须静止 */
/*
 * 读取、扣除零偏、转换到机体坐标系和实际单位。
 */
uint8_t IMU_Sensor_Read(IMU_Data *data);
#endif