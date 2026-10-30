#include "imu_sensors.h"
#include "bsp_timer.h"
#include "QMI8658.h"
/* 根据QMI8658 驱动里的量程配置，计算出实际单位 */
#define ACC_COUNTS_PER_G        2048.0f
#define GYRO_COUNTS_PER_DPS     16.0f
#define CALIBRATION_SAMPLES     1000U
/* 芯片坐标系下的原始数据 */
typedef struct
{
    int16_t ax,ay,az;
    int16_t gx,gy,gz;
}IMU_RawData;
/* 校准参数也是芯片坐标系下的原始计数值 */
typedef struct
{
    float ax,ay,az;
    float gx,gy,gz;
}IMU_Offset;
static IMU_Offset offset;
/* 读取原始数据 */
static uint8_t ReadRaw(IMU_RawData *raw)
{
    return QMI8658_GetData(
        &raw->ax, &raw->ay, &raw->az,
        &raw->gx, &raw->gy, &raw->gz);
}
/* 初始化IMU传感器 */
void IMU_Sensor_Init(void)
{
    offset = (IMU_Offset){0};
    QMI8658_Init();
}
/* 获取IMU传感器ID */
uint8_t IMU_Sensor_GetID(void)
{
    return QMI8658_GetID();
}
/* 校准加速度计 */
uint8_t IMU_Sensor_CalibrateAcc(void)
{
    IMU_RawData raw;
    if (!ReadRaw(&raw))
    {
        return 0;
    }
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;

    for(uint16_t i=0;i<CALIBRATION_SAMPLES;i++)
    {
        if(!ReadRaw(&raw))
            return 0;
        sum_ax += raw.ax;
        sum_ay += raw.ay;
        sum_az += raw.az;
        System_Delay_ms(2);
    }
     /* 水平时，芯片理想读数为 0、0、+2048 */
    offset.ax = (float)sum_ax / CALIBRATION_SAMPLES;
    offset.ay = (float)sum_ay / CALIBRATION_SAMPLES;
    offset.az = (float)sum_az / CALIBRATION_SAMPLES - ACC_COUNTS_PER_G;
    return 1;
}
/* 校准陀螺仪 */
uint8_t IMU_Sensor_CalibrateGyro(void)
{
    IMU_RawData raw;
    if (!ReadRaw(&raw))
    {
        return 0;
    }
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for(uint16_t i=0;i<CALIBRATION_SAMPLES;i++)
    {
        if(!ReadRaw(&raw))
            return 0;
        sum_gx += raw.gx;
        sum_gy += raw.gy;
        sum_gz += raw.gz;
        System_Delay_ms(2);
    }
    /* 静止时角速度理想值为零，因此平均值就是零偏 */
    offset.gx = (float)sum_gx / CALIBRATION_SAMPLES;
    offset.gy = (float)sum_gy / CALIBRATION_SAMPLES;
    offset.gz = (float)sum_gz / CALIBRATION_SAMPLES;
    return 1;
}
/* 读取IMU传感器数据,此时的数据为校准之后的 */
uint8_t IMU_Sensor_Read(IMU_Data *data)
{
    IMU_RawData raw;
    if(!ReadRaw(&raw))
    {
        return 0;
    }
    /*
     * 安装方向转换：
     * 先减对应芯片轴的零偏，再转换方向和单位。
     */
    data->ax = -((float)raw.ay - offset.ay) / ACC_COUNTS_PER_G;
    data->ay = -((float)raw.ax - offset.ax) / ACC_COUNTS_PER_G;
    data->az =  ((float)raw.az - offset.az) / ACC_COUNTS_PER_G;
    data->gx =  ((float)raw.gy - offset.gy) / GYRO_COUNTS_PER_DPS;
    data->gy =  ((float)raw.gx - offset.gx) / GYRO_COUNTS_PER_DPS;
    data->gz = -((float)raw.gz - offset.gz) / GYRO_COUNTS_PER_DPS;

    return 1;
}