#include "flight_app.h"
#include "bsp_timer.h"

/* 硬件及传感器接口 */
#include "motor.h"
#include "NRF24L01.h"
#include "imu_sensors.h"
#include "SPA06.h"
#include "VL53L1X.h"
#include "battery_monitor.h"

/* 算法模块 */
#include "attitude.h"
#include "angle_control.h"
#include "rate_control.h"
#include "height_observation.h"
#include "height_estimator.h"
#include "vertical_control.h"

/* 操作系统 */
#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
/* ==================== 配置 ==================== */

#define REMOTE_TIMEOUT_MS 3000U /* 有效遥控包超时间隔，ms，3秒 */
#define HEIGHT_CORRECTION_START_CM 4.0f /* 气压相对高度接入门槛 */
#define LASER_GROUND_CLEARANCE_CM  1.0f /* 落地时激光安装高度，约10mm */
#define BARO_CORRECTION_MAX_AGE_MS 100U /* 气压当前配置32Hz，允许使用100ms内的最近高度 */
#define LASER_CORRECTION_MAX_GAP_MS 300U /* 激光当前周期100ms，间隔达到300ms则重新建立校正计时*/

/* ==================== 类型定义 ==================== */

/* 运行状态 */
typedef enum
{
    STATE_STOP = 0,  /* 停止，电机停转 */
    STATE_READY,     /* 就绪，等待起飞条件，电机停转 */
    STATE_RUNNING    /* 运行，执行控制及电机输出 */
} FlightRunState_t;

/* 模式编号，实现定高和定点 */
typedef enum
{
    MODE_STAB = 0, /* 自稳：手动油门、目标倾角 */
    MODE_ALT = 1   /* 定高：自动油门、目标倾角 */
} FlightMode_t;


/* ==================== 调度与运行状态 ==================== */

static uint8_t TaskCount; /* 1ms时间槽计数，循环0~7，错开执行2/4/8ms任务 */

static volatile FlightRunState_t RunState; /* 停止、就绪或运行状态；主循环与定时任务共享 */
static volatile FlightMode_t FlyMode; /* 飞行模式：0自稳、1定高；上电默认定高 */

/* ==================== 遥控数据 ==================== */

static RemoteData_t Remote;    /* 最近一次通过包头及摇杆范围检查的遥控包 */
static TickType_t LastRemoteTick;  /* 最近一次有效遥控包接收时刻，tick */
static uint8_t RemoteReceived; /* 1：收到过有效遥控包；0：尚未收到 */
static uint8_t RemoteOnline;   /* 1：收到过有效包且间隔不足3秒；失联动作尚未接入 */
/* 已交给控制任务使用的偏航摇杆输入 */
static int8_t LH; /* 供4ms任务累加偏航目标的左摇杆横向值，-100~100 */

/* ==================== 运行状态切换 ==================== */

static void FlightApp_Start(void);

/* ==================== IMU与姿态估计 ==================== */

static IMU_Data Imu;      /* 最近一次转换后的六轴数据：加速度g、角速度度/s */
static uint8_t ImuValid;  /* 最近一次2ms任务的六轴读取是否成功，1成功、0无效 */

static AttitudeQuat Quat;           /* 姿态四元数，q0为标量部分，q1~q3为向量部分 */
static AttitudeEuler Euler;         /* 四元数转换的横滚、俯仰、偏航角，度 */
static uint8_t AttitudeInitPending; /* 1：等待下一次有效IMU采样后用加速度初始化姿态 */
static uint8_t AttitudeValid;       /* 最近一次姿态初始化或融合是否成功；IMU读取失败也清零 */

/* ==================== 姿态控制 ==================== */

static AngleControl_t AngleControl; /* 三轴角度外环PID参数和运行状态 */
static RateControl_t RateControl;   /* 三轴角速度内环PID参数和运行状态 */

static AngleTarget_t TargetAngle; /* 独立三轴目标角度，度；启动时须与角度PID同步清零 */
static RateVector_t TargetRate;   /* 外环输出的目标角速度，度/s；启动时须与内环同步清零 */

static float RollTrim;  /* 横滚微调，度，加在角度环实际值上 */
static float PitchTrim; /* 俯仰微调，度，加在角度环实际值上 */

/* ==================== 运动速度目标 ==================== */

/* 单位均为cm/s；进入运行时需与对应PID同步重置 */
static float TargetHeightSpeed;  /* 垂直速度目标，cm/s，由LV/2得到 */

/* ==================== 高度观测 ==================== */

static HeightObservation_t HeightObservation; /* 气压基准、原始相对高度及激光慢校正状态 */

/* 气压计 */
static int8_t BaroInitStatus;        /* SPA06初始化返回值：0成功，非0失败 */
static uint8_t BaroReferencePending; /* 1：用下一份有效气压海拔重新建立零点 */

static float BaroPressurePa;   /* 最近一组新温压数据中的补偿气压，Pa */
static float BaroTemperatureC; /* 最近一组温压数据中的芯片温度，摄氏度，并非环境温度 */
static float BaroAltitudeM;    /* 由气压换算的海拔，m，尚未减起飞基准 */

static uint8_t BaroHeightNew; /* 1：本次8ms任务更新了气压高度；0：本轮没有新可用高度 */
static TickType_t BaroLastTick;   /* 最近一次成功建立基准或更新气压高度的接收时刻，tick */

/* 激光 */
static int8_t LaserInitStatus; /* VL53L1X初始化返回值：0成功，非0失败 */

static uint16_t LaserDistanceMm; /* 驱动最近一次报告有效的斜距，mm；不代表始终新鲜 */
static uint8_t LaserRangeStatus; /* 最近读到的测距状态：0有效，255为初始化占位值 */
static float LaserHeightCm;      /* 最近换算的垂直离地高度，cm，不减40mm、不以首帧归零 */

static uint8_t LaserHeightNew; /* 1：本次8ms任务得到新可用激光高度；0：本轮没有 */
static TickType_t LaserLastTick;   /* 最近一次成功换算激光高度的接收时刻，tick */

/* 激光慢校正计时，不与激光采集时间混用 */
static TickType_t HeightCorrectionLastTick;   /* 上一份校正候选帧的接收时刻，tick */
static uint8_t HeightCorrectionTimeReady; /* 1：已建立连续校正的时间起点 */

/* 最近一次激光查询结果，沿用驱动返回值含义 */
static int8_t LaserReadResult; /* 1新有效帧、0暂无新帧、负数异常 */

/* ==================== 垂直运动估计 ==================== */

static HeightEstimator_t HeightEstimate; /* 三阶互补估计的高度、速度及加速度状态 */
static uint8_t HeightEstimateValid;      /* 本轮8ms任务是否完成有效估计 */
static uint8_t HeightRedefinePending; /* 1：下次有效高度估计前，将估计高度对齐观测 */

/* ==================== 垂直速度控制 ==================== */

static VerticalControl_t VerticalControl; /* 垂直速度PID，输入cm/s，输出集体油门 */

/* ==================== 电机输出 ==================== */

static int16_t Throttle;          /* 自稳取LV，定高取垂直PID输出；供2ms混控 */
static MotorOutput_t MotorOutput; /* 四路浮点混控结果，在Motor_SetOutput中限幅并转整数 */
/* 4ms任务中四路PWM连续全为0的次数，达到100后保持 */
static uint8_t MotorZeroCount;

/* ==================== 初始化函数 ==================== */

static void FlightApp_MotorInit(void);
static void FlightApp_AttitudeInit(void);
static void FlightApp_HeightInit(void);
static void FlightApp_ControlInit(void);
static void FlightApp_RemoteInit(void);
static uint8_t FlightApp_ImuCalibrate(void);

/* ==================== 周期任务 ==================== */

static void FlightApp_Task2ms(void);
static void FlightApp_Task4ms(void);
static void FlightApp_HeightTask8ms(void);

/* ==================== 遥控处理 ==================== */

static void FlightApp_RemoteKey(void);
static void FlightApp_RemoteTargetUpdate(void);
/* 检查遥控在线状态，并执行失联目标覆盖 */
static void FlightApp_RemoteFailsafeUpdate(void);

/* ==================== 高度声明 ==================== */
//采集
static void FlightApp_BaroUpdate(void);
static void FlightApp_LaserUpdate(void);
//校正
static void FlightApp_HeightCorrectionUpdate(void);
//估计
static void FlightApp_HeightEstimateUpdate(void);
//垂直控制
static void FlightApp_VerticalControlUpdate(void);
//降落检查
static void FlightApp_LandingCheck(void);

/* ==================== 应用入口 ==================== */

/* 上电总初始化；硬件顺序为电机、IMU、气压、激光、无线。 */
uint8_t FlightApp_Init(void)
{
    TaskCount = 0;

    FlightApp_MotorInit();
    FlightApp_AttitudeInit();
    if (!FlightApp_ImuCalibrate())
    {
        return 0U;
    }
    FlightApp_HeightInit();
    FlightApp_ControlInit();
    BatteryMonitor_Init();
    FlightApp_RemoteInit();
    return 1U;
}

/* 飞控任务调用：检查遥控失联 */
void FlightApp_ProcessInputs(void)
{
    FlightApp_RemoteFailsafeUpdate();//如果读取失败，也必须执行失联判断
}

/* 1ms中断入口：按原厂时间槽错开周期任务。 */
void FlightApp_Tick1ms(void)
{
    /* 先自增，再对8取余 */
    TaskCount++;
    TaskCount %= 8U;

    if (TaskCount % 2U == 0U)             //TaskCount为0/2/4/6时进入if，即2ms, 500Hz
    {
        FlightApp_Task2ms();
    }

    if (TaskCount % 4U == 1U)             //TaskCount为1/5时进入if，即4ms, 250Hz
    {
        FlightApp_Task4ms();
    }

    if (TaskCount == 3U)                  //TaskCount为3时进入if，即8ms, 125Hz
    {
        FlightApp_HeightTask8ms();
    }
}
/*
 * 读取并校验遥控包。
 * 返回1：received中是一份完整有效指令。
 * 返回0：没有新包或包内容无效。
 *
 * 本函数不修改PID、控制目标、飞行状态，也不操作电机。
 */
 uint8_t FlightApp_RemoteRead(RemoteData_t *received)
{
    if (received == NULL)
    {
        return 0U;
    }

    if (NRF24L01_Receive() != 1)
    {
        return 0U;
    }

    RemoteData_t packet = {
        .ID  = NRF24L01_RxPacket[0],
        .LH  = (int8_t)NRF24L01_RxPacket[1],
        .LV  = (int8_t)NRF24L01_RxPacket[2],
        .RH  = (int8_t)NRF24L01_RxPacket[3],
        .RV  = (int8_t)NRF24L01_RxPacket[4],
        .KEY = NRF24L01_RxPacket[5],
        .received_tick = xTaskGetTickCount()
    };

    if (packet.ID != 0x10U && packet.ID != 0x11U)
    {
        return 0U;
    }

    if (packet.LH < -100 || packet.LH > 100 ||
        packet.LV < -100 || packet.LV > 100 ||
        packet.RH < -100 || packet.RH > 100 ||
        packet.RV < -100 || packet.RV > 100)
    {
        return 0U;
    }

    /* 整包校验通过，才向调用者提交 */
    *received = packet;

    return 1U;
}
/*
 * 应用一份已经校验通过的遥控指令。
 * 只能由飞控所在任务调用。
 */
void FlightApp_RemoteApply(const RemoteData_t *received)
{
    if (received == NULL)
    {
        return;
    }

    Remote = *received;

    /* 使用实际收包时间，避免排队等待延长失联判断 */
    LastRemoteTick = received->received_tick;
    RemoteReceived = 1U;

    /* 保留原来的顺序：先更新摇杆目标 */
    FlightApp_RemoteTargetUpdate();

    /* 再判断是否满足起飞条件 */
    if (Remote.LV > 10 && RunState == STATE_READY)
    {
        FlightApp_Start();
    }
    /* 处理按键 */
    FlightApp_RemoteKey();
}
/* ==================== 上电初始化 ==================== */

/* 初始化PWM，清除基础油门和混控数据。 */
static void FlightApp_MotorInit(void)
{
    Motor_Init();

    Throttle = 0;
    MotorOutput = (MotorOutput_t){0};

    MotorZeroCount = 0U;
}

/* 初始化六轴传感器及姿态状态，等待有效加速度初始化姿态。 */
static void FlightApp_AttitudeInit(void)
{
    Imu = (IMU_Data){0};
    ImuValid = 0;

    /* 临时单位四元数，等待有效加速度建立初始姿态 */
    /* 四元数q0为标量，剩下的三个为向量，给(1,0,0,0)表示机体坐标系与世界坐标系对齐，没有任何旋转 */
    Quat = (AttitudeQuat){.q0 = 1.0f};
    Euler = (AttitudeEuler){0};

    AttitudeInitPending = 1;
    AttitudeValid = 0;

    IMU_Sensor_Init();
}
/* 上电阻塞校准：必须水平、静止 */
static uint8_t FlightApp_ImuCalibrate(void)
{
    if (!IMU_Sensor_CalibrateAcc())
    {
        return 0U;
    }

    if (!IMU_Sensor_CalibrateGyro())
    {
        return 0U;
    }

    return 1U;
}
/* 初始化高度观测、气压与激光，并请求建立气压基准。 */
static void FlightApp_HeightInit(void)
{
    HeightObservation_Init(&HeightObservation);
    HeightEstimator_Init(&HeightEstimate);
    HeightEstimateValid = 0;
    HeightRedefinePending = 0;
    /* 下一份有效气压测量用于建立零点 */
    BaroReferencePending = 1;

    BaroPressurePa = 0.0f;
    BaroTemperatureC = 0.0f;
    BaroAltitudeM = 0.0f;

    BaroHeightNew = 0;
    BaroLastTick = 0;
    
    LaserDistanceMm = 0;
    LaserRangeStatus = 255;
    LaserHeightCm = 0.0f;

    LaserHeightNew = 0;
    LaserLastTick = 0;

    HeightCorrectionLastTick = 0;
    HeightCorrectionTimeReady = 0;
    LaserReadResult = -1;

    BaroInitStatus = SPA06_Init();
    LaserInitStatus = VL53L1X_Init();
}

/* 初始化运行状态、模式、PID参数及控制目标。 */
static void FlightApp_ControlInit(void)
{
    RunState = STATE_STOP;
    FlyMode = MODE_ALT;

    AngleControl_Init(&AngleControl);
    RateControl_Init(&RateControl);
    VerticalControl_Init(&VerticalControl);

    TargetAngle = (AngleTarget_t){0};
    TargetRate = (RateVector_t){0};

    TargetHeightSpeed = 0.0f;

    RollTrim = 0.0f;
    PitchTrim = 0.0f;
}

/* 初始化无线接口、遥控数据及在线判断状态。 */
static void FlightApp_RemoteInit(void)
{
    Remote = (RemoteData_t){0};
    LH = 0;

    NRF24L01_Init();

    LastRemoteTick = 0;
    RemoteReceived = 0;
    RemoteOnline = 0;
}

/* ==================== 遥控处理：主循环执行 ==================== */

/* 按摇杆系数更新倾角、速度目标及偏航输入。 */
static void FlightApp_RemoteTargetUpdate(void)
{
    LH = Remote.LH;

    /* 自稳、定高都用右摇杆给目标倾角 */
    TargetAngle.Roll = Remote.RH / 5.0f;
    TargetAngle.Pitch = -Remote.RV / 5.0f;

    if (FlyMode == MODE_STAB)
    {
        /* 自稳：手动油门 */
        Throttle = Remote.LV;
    }
    else if (FlyMode == MODE_ALT)
    {
        if (BatteryMonitor_IsUnderVoltage())
        {
            /* 定高欠压时请求下降 */
            TargetHeightSpeed = -50.0f;
        }
        else
        {
            TargetHeightSpeed = Remote.LV / 2.0f;
        }
    }
}

/* K1就绪、K2停止、K3自稳、K4定高 */
static void FlightApp_RemoteKey(void)
{
    if(Remote.KEY == 1)            //如果遥控按键为1，进入就绪状态，停止电机，等待摇杆信号
    {
        TargetHeightSpeed = 0.0f;
        VerticalControl.velocity.Target = 0.0f;
        BatteryMonitor_ClearUnderVoltage();
        RunState = STATE_READY;
        Motor_Stop();
    }
    else if(Remote.KEY == 2)       //如果遥控按键为2，进入紧急停止状态，停止电机
    {
        RunState = STATE_STOP;
        Motor_Stop();
    }
    else if (Remote.KEY == 3)
    {
        /* 定高切到自稳 */
        if (FlyMode == MODE_ALT)
        {
            FlyMode = MODE_STAB;
        }

        HeightRedefinePending = 1;
    }
    else if (Remote.KEY == 4)
    {
        /* 自稳切到定高 */
        if (FlyMode == MODE_STAB)
        {
            FlyMode = MODE_ALT;
        }

        HeightRedefinePending = 1;
    }
}
/* 遥控失联处理 */
static void FlightApp_RemoteFailsafeUpdate(void)
{
    TickType_t now_tick = xTaskGetTickCount();

    RemoteOnline =
        RemoteReceived &&
        ((TickType_t)(now_tick - LastRemoteTick) < pdMS_TO_TICKS(REMOTE_TIMEOUT_MS));

    if (RemoteOnline)
    {
        return;
    }

    /* 失联后取消横滚、俯仰倾角指令 */
    TargetAngle.Roll = 0.0f;
    TargetAngle.Pitch = 0.0f;

    if (FlyMode == MODE_STAB)
    {
        /* 自稳失联：基础油门固定为30 */
        Throttle = 30;
    }
    else if (FlyMode == MODE_ALT)
    {
        /* 定高失联：垂直速度目标固定为-50cm/s */
        TargetHeightSpeed = -50.0f;
    }
}
/*
 * 只由飞控任务调用，只复制状态。
 * 无线发送由无线任务负责。
 */
void FlightApp_GetTelemetry(TelemetryData_t *data)
{
    if (data == NULL)
    {
        return;
    }

    *data = (TelemetryData_t){
        .flight_mode = (uint8_t)FlyMode,
        .throttle = Throttle,
        .battery_v = BatteryMonitor_GetVoltage(),

        .roll_deg = Euler.roll,
        .pitch_deg = Euler.pitch,
        .yaw_deg = Euler.yaw,
        .height_cm = HeightObservation.corrected_height_cm,

        .under_voltage = BatteryMonitor_IsUnderVoltage(),
        .run_state = (uint8_t)RunState
    };
}

/* ==================== 运行状态切换 ==================== */

/* 起飞前重置算法状态，不重新初始化硬件 */
static void FlightApp_Start(void)
{
    /* 清除各控制器的运行状态 */
    AngleControl_Reset(&AngleControl);
    RateControl_Reset(&RateControl);
    VerticalControl_Reset(&VerticalControl);

    /* 独立目标与PID内部目标同步清零 */
    TargetAngle = (AngleTarget_t){0};
    TargetRate = (RateVector_t){0};

    TargetHeightSpeed = 0.0f;

    /* 重新建立垂直运动估计 */
    HeightEstimator_Init(&HeightEstimate);
    HeightEstimateValid = 0;
    HeightRedefinePending = 0;

    /* 重新初始化姿态和气压高度零点 */
    AttitudeInitPending = 1;
    AttitudeValid = 0;

    /* 清除上次起飞的修正状态 */
    HeightObservation_Init(&HeightObservation);
    BaroReferencePending = 1;

    HeightCorrectionLastTick = 0;
    HeightCorrectionTimeReady = 0;
    BaroHeightNew = 0;
    LaserHeightNew = 0;

    /* 不沿用上次的基础油门和混控结果 */
    Throttle = 0;
    MotorOutput = (MotorOutput_t){0};

    RunState = STATE_RUNNING;
}

/* ==================== 周期任务,由调度器统一执行 ==================== */

/* 2ms：IMU采样、按需姿态初始化、角速度内环、前馈及混控。 */
static void FlightApp_Task2ms(void)
{
    /*
     * IMU采样、零偏和坐标转换
     * 必要时初始化姿态四元数
     * 运行时计算角速度PID、前馈及电机混控
     */
    ImuValid = IMU_Sensor_Read(&Imu);
    if(!ImuValid)
    {
        AttitudeValid = 0;
        Motor_Stop();
        return;
    }
    /* 在IMU采样后，按标志初始化一次四元数 */
    if(AttitudeInitPending)
    {
        AttitudeValid = Attitude_Init(&Quat,Imu.ax,Imu.ay,Imu.az);
        if(!AttitudeValid)
        {
            Motor_Stop();
            return;
        }
        Attitude_QuatToEuler(&Quat,&Euler);
        /* 只有初始化成功才清除标志 */
        AttitudeInitPending = 0;
    }
    /* 两种模式都需要运行状态和有效姿态 */
    if (RunState != STATE_RUNNING || !AttitudeValid)
    {
        Motor_Stop();
        return;
    }

    /* 只有定高模式需要等待高度初始化和估计 */
    if (FlyMode == MODE_ALT &&
        (BaroReferencePending || !HeightEstimateValid))
    {
        Motor_Stop();
        return;
    }
    /* 陀螺仪角速度作为内环实际值 */
    RateVector_t actual_rate = 
    {
        .Roll = Imu.gx,
        .Pitch = Imu.gy,
        .Yaw = Imu.gz
    };
    //目标值(来源于4ms角度外环）与实际值进行PID计算，输出角速度调制量
    RateControl_Update(&RateControl,&TargetRate,&actual_rate,0.002f);
    /* 前馈,目标倾角直接参与内环输出 */
    //前馈直接根据目标倾角，额外加一点修正量，帮助响应目标角速度变化
    //每1°目标倾角，额外增加0.1个百分点的混控修正量
    RateControl.Roll.Out += TargetAngle.Roll * 0.1f;
    RateControl.Pitch.Out += TargetAngle.Pitch * 0.1f;
    if(Throttle > 90)
    {
        Throttle = 90;
    }
    //计算各路电机混动值
    Motor_Mix(Throttle,RateControl.Roll.Out,RateControl.Pitch.Out,RateControl.Yaw.Out,&MotorOutput);
    //将各路电机混动值设置给电机
    Motor_SetOutput(MotorOutput.m1,MotorOutput.m2,MotorOutput.m3,MotorOutput.m4);

}

/* 4ms：姿态融合、倾斜停机、角度外环及偏航目标更新。 */
static void FlightApp_Task4ms(void)
{
    /*
     * 姿态解算
     * 倾斜停机与落地判断
     * 角度PID
     * 将角度PID输出交给角速度PID目标
     * 更新偏航目标角度
     */
    /*
     * 第一次4ms时间槽在上电后第1ms就会进入，
     * 此时2ms采样还没发生，因此需要等待初始化。
     */
    if (!ImuValid || AttitudeInitPending)
    {
        AttitudeValid = 0;
        return;
    }

    AttitudeValid = Attitude_Update6Axis(&Quat, Imu.gx, Imu.gy, Imu.gz, Imu.ax, Imu.ay, Imu.az, 0.004f);
    if (!AttitudeValid)
    {
        Motor_Stop();
        return;
    }

    Attitude_QuatToEuler(&Quat, &Euler);
    //倾斜停机判断
    if(Euler.roll < -70.0f || Euler.roll > 70.0f || Euler.pitch < -70.0f || Euler.pitch > 70.0f)
    {
        RunState = STATE_STOP;
        Motor_Stop();
    }
    //落地自动停机
    FlightApp_LandingCheck();

    if(RunState != STATE_RUNNING)
    {
        return;
    }
    /* 微调加在实际角度上，不改姿态估计结果 */
    AttitudeEuler actual_angle = Euler;
    actual_angle.roll += RollTrim;
    actual_angle.pitch += PitchTrim;

    AngleControl_Update(&AngleControl,&TargetAngle,&actual_angle,&TargetRate,0.004f);
    /* 计算完本轮角度环后，再更新偏航目标 */
    TargetAngle.Yaw += LH / 100.0f * 180.0f * 0.004f;

    /* 偏航目标越界处理 */
    if (TargetAngle.Yaw > 180.0f)
    {
        TargetAngle.Yaw = -180.0f;
    }

    if (TargetAngle.Yaw < -180.0f)
    {
        TargetAngle.Yaw = 180.0f;
    }
}

/* 8ms：先采集气压，再采集激光，尚未接融合与油门控制。 */
static void FlightApp_HeightTask8ms(void)
{
    FlightApp_BaroUpdate();
    FlightApp_LaserUpdate();
    FlightApp_HeightCorrectionUpdate();
    FlightApp_HeightEstimateUpdate();
    FlightApp_VerticalControlUpdate();
    /* 后续接：激光慢校正、原厂高度与速度估计、垂直速度PID。 */
}

/* ==================== 高度传感器采集：8ms高度任务调用 ==================== */

/* 更新气压相对高度，必要时用有效新测量建立基准。 */
static void FlightApp_BaroUpdate(void)
{
    BaroHeightNew = 0;

    if (BaroInitStatus == 0)
    {
        int8_t result = SPA06_GetData(
            &BaroPressurePa, &BaroTemperatureC);

        if (result == 1)
        {
            uint8_t converted =
                HeightObservation_PressureToAltitude(
                    BaroPressurePa, &BaroAltitudeM);

            if (converted)
            {
                uint8_t updated;

                if (BaroReferencePending)
                {
                    /* 用一份有效测量建立零点 */
                    updated =
                        HeightObservation_SetBaroReference(
                            &HeightObservation, BaroAltitudeM);

                    if (updated)
                    {
                        BaroReferencePending = 0;
                        /* 气压零点改变后，校正计时也重新开始 */
                        HeightCorrectionTimeReady = 0;
                    }
                }
                else
                {
                    updated = HeightObservation_UpdateBaro(
                        &HeightObservation, BaroAltitudeM);
                }

                if (updated)
                {
                    BaroHeightNew = 1;
                    BaroLastTick = xTaskGetTickCount();
                }
            }
        }
    }
}

/* 读取有效新激光帧，并用有效姿态换算垂直离地高度。 */
static void FlightApp_LaserUpdate(void)
{
    LaserHeightNew = 0;

    if (LaserInitStatus == 0)
    {
        int8_t result = VL53L1X_ReadDistance(
            &LaserDistanceMm, &LaserRangeStatus);
        LaserReadResult = result;

        /*
         * 必须是驱动报告有效的新帧，
         * 并且当前姿态可用，才做倾斜补偿。
         */
        if (result == 1 &&
            AttitudeValid &&
            !AttitudeInitPending)
        {
            uint8_t converted =
                HeightObservation_LaserToHeight(
                    LaserDistanceMm,
                    Euler.roll,
                    Euler.pitch,
                    &LaserHeightCm);

            if (converted)
            {
                LaserHeightNew = 1;
                LaserLastTick = xTaskGetTickCount();
            }
        }
    }
}

/* 高度观测校正 */
/* 只用新激光帧更新气压修正量，无效，间隔过长或条件不满足时，保留已有修正量 */
static void FlightApp_HeightCorrectionUpdate(void)
{
    TickType_t now_tick = xTaskGetTickCount();
    /* 只在定高模式使用校正 */
    if (FlyMode != MODE_ALT ||
        BaroReferencePending ||
        !HeightObservation.baro_reference_ready)
    {
        HeightCorrectionTimeReady = 0;
        return;
    }
    /* 没有足够新的气压高度，不与激光比较 */
    if ((TickType_t)(now_tick - BaroLastTick) >=
        pdMS_TO_TICKS(BARO_CORRECTION_MAX_AGE_MS))
    {
        HeightCorrectionTimeReady = 0;
        return;
    }
    /* 气压相对高度达到4cm后才尝试校正 */
    if (HeightObservation.baro_relative_cm <
        HEIGHT_CORRECTION_START_CM)
    {
        HeightCorrectionTimeReady = 0;
        return;
    }
    /*
     * 驱动异常、有效帧换算失败或姿态不可用
     * 冻结修正量，撤销连续计时
     */
    if (LaserReadResult < 0 ||
        (LaserReadResult == 1 && !LaserHeightNew) ||
        !AttitudeValid ||
        AttitudeInitPending)
    {
        HeightCorrectionTimeReady = 0;
        return;
    }
    /* 长时间没有有效新帧，恢复时不得累计这段失效时间 */
    if (HeightCorrectionTimeReady &&
        ((TickType_t)(now_tick - HeightCorrectionLastTick) >=
            pdMS_TO_TICKS(LASER_CORRECTION_MAX_GAP_MS)))
    {
        HeightCorrectionTimeReady = 0;
    }
    /* 暂无新帧时，既不重复校正，也不按8ms重置计时 */
    if (!LaserHeightNew)
    {
        return;
    }
    /* 首次接入或恢复：只记录时间，不瞬间改变修正量 */
    if (!HeightCorrectionTimeReady)
    {
        HeightCorrectionLastTick = LaserLastTick;
        HeightCorrectionTimeReady = 1;
        return;
    }
    TickType_t elapsed_ticks =
        (TickType_t)(LaserLastTick - HeightCorrectionLastTick);

    HeightCorrectionLastTick = LaserLastTick;

    if (elapsed_ticks == 0U)
    {
        return;
    }
    /* 激光离地高度减落地安装高度，对齐气压的起飞零点 */ 
    float laser_reference_cm = LaserHeightCm - LASER_GROUND_CLEARANCE_CM;
    float elapsed_s = (float)elapsed_ticks / (float)configTICK_RATE_HZ;
    uint8_t updated = HeightObservation_CorrectBaro(
        &HeightObservation,
        laser_reference_cm,
        elapsed_s);

    if (!updated)
    {
        HeightCorrectionTimeReady = 0;
    }
}

/* 按8ms周期融合高度观测和垂直加速度 */
static void FlightApp_HeightEstimateUpdate(void)
{
    HeightEstimateValid = 0;
    if(FlyMode != MODE_ALT)
    {
        return;
    }
    /* 必须已有姿态、IMU数据及气压高度基准 */
    if (!ImuValid ||
        !AttitudeValid ||
        AttitudeInitPending ||
        BaroReferencePending ||
        !HeightObservation.baro_reference_ready)
    {
        return;
    }

    float vertical_accel_cm_s2;//去重力后的垂直加速度
    uint8_t calculated = 
        HeightEstimator_CalculateVerticalAccel(
            Imu.ax,
            Imu.ay,
            Imu.az,
            Euler.roll,
            Euler.pitch,
            &vertical_accel_cm_s2);
    if(!calculated)
    {
        return;
    }
    if (HeightRedefinePending)
    {
        HeightEstimate.height_cm = HeightObservation.corrected_height_cm;
        HeightRedefinePending = 0;
    }
    HeightEstimateValid = HeightEstimator_Update(&HeightEstimate, vertical_accel_cm_s2, HeightObservation.corrected_height_cm);
}
/* 按8ms周期计算垂直速度PID，输出集体油门 */
static void FlightApp_VerticalControlUpdate(void)
{
    if(RunState != STATE_RUNNING)
    {
        return;
    }
    if(FlyMode != MODE_ALT)
    {
        return;
    }
    /* 刚启动时等待初始化完成，不把等待首帧当作计算故障 */
    if (AttitudeInitPending ||
        BaroReferencePending ||
        !HeightObservation.baro_reference_ready)
    {
        return;
    }
    /*
     * 补充异常处理：
     * 本轮不能完成高度估计时，停止运行，
     * 避免继续使用旧垂直状态和旧油门。
     */
    if(!HeightEstimateValid)
    {
        RunState = STATE_STOP;
        Throttle = 0;
        Motor_Stop();
        return;
    }
    VerticalControl_Update(&VerticalControl, TargetHeightSpeed, HeightEstimate.vertical_velocity_cm_s, 0.008f);
    Throttle = (int16_t)VerticalControl.velocity.Out;

}

/* 落地停机条件：PWM持续为0，并且垂直PID仍有下降目标 */
static void FlightApp_LandingCheck(void)
{
    if (Motor_IsOutputAllZero())
    {
        if (MotorZeroCount < 100U)
        {
            MotorZeroCount++;
        }
    }
    else
    {
        MotorZeroCount = 0U;
    }

    if (VerticalControl.velocity.Target < -20.0f &&
        MotorZeroCount == 100U)
    {
        RunState = STATE_STOP;
        Motor_Stop();
    }
}