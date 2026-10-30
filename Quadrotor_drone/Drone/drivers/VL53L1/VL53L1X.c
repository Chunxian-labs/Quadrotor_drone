#include <stdint.h>
#include "bsp_i2c.h"
#include "VL53L1X_api.h"
#include "vl53l1_platform.h"

/* 默认的 7 位设备地址为 0x29，左移一位后得到写地址 0x52。
 * 读操作的最低位由 I2C 底层置 1，此处无需再次左移。
 */
static const uint8_t VL53L1X_dev = 0x52;
static uint8_t SensorReady = 0;

/**
 * @brief 配置传感器并启动测距，上电调用一次。
 * @note 使用平台层延时前，需要完成 System_timer_Init()。
 *       返回 0 表示初始化成功，非零表示失败。
 */
int8_t VL53L1X_Init(void)
{
	int8_t status;//保存每一步的执行结果
	SensorReady = 0;//软件里的状态变量，不是传感器寄存器，0表示初始化未成功，1表示初始化成功
	/* 初始化软件 I2C2 引脚：PB10 为时钟线 SCL，PB11 为数据线 SDA。 */
	BSP_I2C_Init(&BSP_I2C2_Bus);
	/*
     * 初始化内部存在等待和延时，
     * 必须在 System_timer_Init() 之后调用。
     */
	/* 调用厂商初始化接口，写入默认配置并完成内部初始化流程。 */
	status = VL53L1X_SensorInit(VL53L1X_dev);
	if(status != 0)
	{
		return status;
	}

    /* 距离模式：1 为短距离，2 为长距离，当前选择长距离模式。
     * 实际可测距离还受环境光、目标反射率和测量时间预算等因素影响。
     */
    status = VL53L1X_SetDistanceMode(VL53L1X_dev, 2);
    if(status != 0)
    {
        return status;
    }
    /* 单次测距的时间预算，单位为毫秒，当前设置为 100 ms。
     * 本项目驱动在长距离模式下支持 20、33、50、100、200、500 ms；
     * 短距离模式还支持 15 ms。预算用于配置单次测量的时间分配。
     */
    status = VL53L1X_SetTimingBudgetInMs(VL53L1X_dev, 100);
    if(status != 0)
    {
        return status;
    }
    /* 相邻两次测量的周期，单位为毫秒，应不小于单次测距时间预算。
     * 当前周期为 100 ms，对应名义频率 10 Hz；实际更新速度需以上板结果为准。
     */
    status = VL53L1X_SetInterMeasurementInMs(VL53L1X_dev, 100);
    if(status != 0)
    {
        return status;
    }
    /* 启动测距，后续通过读取结果接口获取距离。 */
    status = VL53L1X_StartRanging(VL53L1X_dev);
    if(status != 0)
    {
        return status;
    }
	SensorReady = 1;
	return 0;
}

/**
 * @brief 读取传感器的 16 位标识，用于辅助确认设备连接及型号。
 * @return 厂商接口读取到的传感器标识值。
 * @note 当前没有检查读取接口返回的通信状态。
 */
uint16_t VL53L1X_GetID(void)
{
	uint16_t ID;
	VL53L1X_GetSensorId(VL53L1X_dev, &ID);
	return ID;
}

/**
 * @brief 检查有没有新结果，有就取出来，运行中反复调用。
 * @return 本次有没有取到新数据、有没有错误。
 * @param distance_mm 指向存储距离的指针，单位为毫米，即测得的距离。
 * @param range_status 指向存储测距状态的指针，即传感器对本次测量质量的判断。
 * @return 
 *     1  有新数据，且有效,距离变量更新
 *     0  无新数据，距离变量没有更新
 *     -1  未初始化、参数错误或接口报告错误，距离变量没有更新
 *     -2  有新数据，但测距结果无效，距离变量没有更新
 *       
 */
int8_t VL53L1X_ReadDistance(uint16_t *distance_mm,uint8_t *range_status)
{
	uint8_t data_ready = 0;         //新的一阵测距结果是否准备好
	uint8_t result_status = 255;
	uint16_t distance = 0;
	int8_t status;                  //查询操作是否报告错误
	int8_t clear_status;
	//第一步、检查能不能调用接口，是否初始化成功，是否传入了有效指针，指针为零表示没有有效的输出地址，不能通过它写数据
	if(!SensorReady || distance_mm == 0 || range_status == 0)
	{
		return -1;
	}
	//第二步、检查有没有新数据
	/* 不循环等待：没有新数据就立即返回 */
	status = VL53L1X_CheckForDataReady(VL53L1X_dev, &data_ready);
	if(status != 0)
	{
		return -1;     /* 查询出错 */
	}
	if(!data_ready)
	{
		return 0;      /* 查询正常，只是结果还没出来 */
	}
	//第三步、检查这帧测量是否有效
	//“有新数据”不代表“距离可信”。目标反光、环境等情况可能导致测量无效
	status = VL53L1X_GetRangeStatus(VL53L1X_dev, &result_status);
	if(status == 0)       //读取接口没报告错误
	{
		*range_status = result_status;
		if(result_status == 0)   //这帧测距结果有效
		{
			status = VL53L1X_GetDistance(VL53L1X_dev, &distance);   //只有两者都满足，才读取距离
		}
	}
	/*
     * 已就绪的一帧，无论测距是否有效，都要确认处理完成。
     * 这是清传感器内部标志，不是清 STM32 中断。
     */
	//第四步、清除传感器的数据就绪标志。
	clear_status = VL53L1X_ClearInterrupt(VL53L1X_dev);
	//第五步、决定返回什么。
	if(clear_status != 0 || status != 0)
	{
		return -1;
	}
	if(result_status != 0)
	{
		return -2;
	}
	*distance_mm = distance;
	return 1;
}