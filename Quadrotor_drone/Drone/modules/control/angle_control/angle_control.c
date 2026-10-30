#include "angle_control.h"
void AngleControl_Reset(AngleControl_t *AngleControl)
{
    PID_Init(&AngleControl->Roll);
    PID_Init(&AngleControl->Pitch);
    PID_Init(&AngleControl->Yaw);
}
//初始化横滚角和俯仰角的PID成员参数
void AngleControl_Init(AngleControl_t *AngleControl)
{
    /* 横滚角度PID参数 */
    AngleControl -> Roll = (PID_t){
        .kp = 3.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .OutMax = 1000.0f,
        .OutMin = -1000.0f
    };  
    /* 俯仰角度PID参数 */
    AngleControl -> Pitch = (PID_t){
        .kp = 3.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .OutMax = 1000.0f,
        .OutMin = -1000.0f
    };
    /*偏航角度PID参数 */
    AngleControl -> Yaw = (PID_t){
        .kp = 6.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .OutMax = 1000.0f,
        .OutMin = -1000.0f
    };
    AngleControl_Reset(AngleControl);
}
       
//内环PID为三个轴的角速度PID环，外环为三个轴的角度PID环，通过判断角度误差来输出角度调制量，再把角度调制量当作内环角速度环的目标量
void AngleControl_Update(AngleControl_t *AngleControl,const AngleTarget_t *Target,const AttitudeEuler *Actual,
                            RateVector_t *Target_rate,float dt)
{
    AngleControl -> Roll.Target = Target -> Roll;
    AngleControl -> Roll.Actual = Actual -> roll;
    AngleControl -> Pitch.Target = Target -> Pitch;
    AngleControl -> Pitch.Actual = Actual -> pitch;
    AngleControl -> Yaw.Target = Target -> Yaw;
    AngleControl -> Yaw.Actual = Actual -> yaw;

    /* 偏航跨越±180°时，选择较短的旋转方向 */
    if (AngleControl->Yaw.Target - AngleControl->Yaw.Actual < -180.0f)
    {
        AngleControl->Yaw.Actual -= 360.0f;
    }
    if (AngleControl->Yaw.Target - AngleControl->Yaw.Actual > 180.0f)
    {
        AngleControl->Yaw.Actual += 360.0f;
    }
    PID_Update(&AngleControl->Roll,dt);
    PID_Update(&AngleControl->Pitch,dt);
    PID_Update(&AngleControl->Yaw,dt);

    /* 外环输出成为内环的目标角速度 这里外环的输出单位是 °/s*/
    Target_rate -> Roll = AngleControl -> Roll.Out;
    Target_rate -> Pitch = AngleControl -> Pitch.Out;
    Target_rate -> Yaw = AngleControl -> Yaw.Out;
}