#include "SPA06.h"
#include "SPA06_Config.h"
#include "bsp_i2c.h"
#include "bsp_timer.h"
#include <math.h>
#define SPA06_ADDR_7BIT		     0x76
#define SPA06_ADDR_WRITE		(SPA06_ADDR_7BIT << 1)
//76 -> SPA06      气压温度传感器
int16_t c0,c1;//12位校准系数c0,c1
int32_t c00,c10;//20位校准系数c00,c10
int16_t c01,c11,c20,c21,c30;//16位校准系数c01,c11,c20,c21,c30
int16_t c31,c40;//12位校准系数c31,c40
static uint8_t SPA06_Initialized = 0;
static void SPA06_ReadCaliCoefficients(void)
{
    uint8_t Coefficients[21];//COEF寄存器有21个字节，地址从0X10-0X24
    BSP_I2C_ReadRegs(&BSP_I2C1_Bus,SPA06_ADDR_WRITE,SPA06_COEF,BSP_I2C_REG_ADDR_8BIT,Coefficients,21);
    // 12-bit
    c0  = (int16_t)(((uint16_t)Coefficients[0] << 4)  | (Coefficients[1] >> 4));
    if(c0 & 0x0800) c0 -= 0x1000; //需要做符号扩展，下面类似
    c1  = (int16_t)(((Coefficients[1] & 0x0F) << 8)  |  Coefficients[2]);
    if(c1 & 0x0800) c1 -= 0x1000;
    // 20-bit (注意：c00, c10 的变量类型必须是 int32_t)
    c00 = (int32_t)(((uint32_t)Coefficients[3] << 12) | ((uint32_t)Coefficients[4] << 4) | (Coefficients[5] >> 4));
    if(c00 & 0x00080000) c00 -= 0x00100000;
    c10 = (int32_t)(((uint32_t)(Coefficients[5] & 0x0F) << 16) | ((uint32_t)Coefficients[6] << 8) | Coefficients[7]);
    if(c10 & 0x00080000) c10 -= 0x00100000;
    
    // 16-bit
    c01 = (int16_t)((Coefficients[8] << 8)  | Coefficients[9]);
    c11 = (int16_t)((Coefficients[10] << 8) | Coefficients[11]);
    c20 = (int16_t)((Coefficients[12] << 8) | Coefficients[13]);
    c21 = (int16_t)((Coefficients[14] << 8) | Coefficients[15]);
    c30 = (int16_t)((Coefficients[16] << 8) | Coefficients[17]);
    
    // 12-bit
    c31 = (int16_t)(((uint16_t)Coefficients[18] << 4) | (Coefficients[19] >> 4));
    if(c31 & 0x0800) c31 -= 0x1000;
    c40 = (int16_t)(((Coefficients[19] & 0x0F) << 8) | Coefficients[20]);
    if(c40 & 0x0800) c40 -= 0x1000;
}
int8_t SPA06_Init(void)
{
    uint8_t ready = 0;
    SPA06_Initialized = 0;
    BSP_I2C_Init(&BSP_I2C1_Bus);
    /* 最多等待约 100 ms，只在初始化阶段阻塞 */
    for(uint16_t i = 0;i < 100U;i++)
    {
        //反复读取MEAS_CFG寄存器，判断是否就绪
        uint8_t status = BSP_I2C_ReadReg(&BSP_I2C1_Bus,SPA06_ADDR_WRITE,SPA06_MEAS_CFG,BSP_I2C_REG_ADDR_8BIT);
        //判断第八位和第七位是否置一，第八位置一代表- Coefficients are available.，第七位置一代表- Sensor initialization complete.
        if((status & 0XC0U) == 0XC0U)
        {
            ready = 1;
            break;
        }
        System_Delay_ms(1);
    }
    if(!ready)
    {
        return -1;
    }
    if(SPA06_GetID() != 0x11U)
    {
        return -2;
    }
    /* 先停止连续测量，再配置 */
    /*这里写 0x00 是让传感器进入待机，停止可能已经在运行的连续测量，方便随后修改采样配置，不是为了读取校准系数。*/
    BSP_I2C_WriteReg(&BSP_I2C1_Bus, SPA06_ADDR_WRITE,SPA06_MEAS_CFG, BSP_I2C_REG_ADDR_8BIT, 0x00);
    //读取校准系数
    SPA06_ReadCaliCoefficients();
    //压力计：过采样16倍，每秒测32次
    BSP_I2C_WriteReg(&BSP_I2C1_Bus,SPA06_ADDR_WRITE,SPA06_PRS_CFG,BSP_I2C_REG_ADDR_8BIT,0x54);
    //温度计：过采样单倍。每秒测32次
    BSP_I2C_WriteReg(&BSP_I2C1_Bus,SPA06_ADDR_WRITE,SPA06_TMP_CFG,BSP_I2C_REG_ADDR_8BIT,0x50);
    //P_SHIFT置1，T_SHIFT置0；
    BSP_I2C_WriteReg(&BSP_I2C1_Bus,SPA06_ADDR_WRITE,SPA06_CFG_REG,BSP_I2C_REG_ADDR_8BIT,0x04);
    if(BSP_I2C_ReadReg(&BSP_I2C1_Bus,SPA06_ADDR_WRITE,SPA06_PRS_CFG,BSP_I2C_REG_ADDR_8BIT) != 0x54 ||
        BSP_I2C_ReadReg(&BSP_I2C1_Bus,SPA06_ADDR_WRITE,SPA06_TMP_CFG,BSP_I2C_REG_ADDR_8BIT) != 0x50 ||
        BSP_I2C_ReadReg(&BSP_I2C1_Bus,SPA06_ADDR_WRITE,SPA06_CFG_REG,BSP_I2C_REG_ADDR_8BIT) != 0x04)
    {
        return -3;
    }
    //开始连续压力和温度测量
    BSP_I2C_WriteReg(&BSP_I2C1_Bus,SPA06_ADDR_WRITE,SPA06_MEAS_CFG,BSP_I2C_REG_ADDR_8BIT,0x07);
    if((BSP_I2C_ReadReg(&BSP_I2C1_Bus,SPA06_ADDR_WRITE,SPA06_MEAS_CFG,BSP_I2C_REG_ADDR_8BIT) & 0x0FU) != 0x07U)
    {
        return -3;
    }
    SPA06_Initialized = 1;
    return 0;
}
uint8_t SPA06_GetID(void)  
{
   return BSP_I2C_ReadReg(&BSP_I2C1_Bus,SPA06_ADDR_WRITE,SPA06_ID,BSP_I2C_REG_ADDR_8BIT);
}
/*
 * 功能：非阻塞读取一组新的气压、温度数据，并完成校准补偿。
 *
 * 参数：
 *   Pcomp：气压输出指针，单位 Pa。
 *   Tcomp：温度输出指针，单位 ℃。
 *
 * 返回值：
 *    1：成功取得新数据，更新两个输出变量。
 *    0：新的温压数据尚未齐全，本次不更新输出。
 *   -1：驱动未初始化、指针为空或传感器运行状态异常。
 *   -2：补偿结果不是有限数，或超出传感器量程。
 *
 * 注意：
 *   1. 返回值不是 1 时，调用者的输出变量保持原值，
 *      不能将这些旧值当成本次的新测量。
 */
int8_t SPA06_GetData(float *Pcomp, float *Tcomp)
{
    uint8_t status;       /* MEAS_CFG 寄存器值：包含就绪标志和测量模式 */
    uint8_t data[6];      /* 原始数据：前 3 字节为压力，后 3 字节为温度 */

    int32_t praw, traw;   /* 符号扩展后的压力、温度原始计数，尚未补偿 */

    float p;             /* 补偿后的气压暂存值，单位 Pa */
    float t;             /* 补偿后的温度暂存值，单位 ℃ */

    float ps;            /* 缩放后的压力原始值，无量纲 */
    float ts;            /* 缩放后的温度原始值，无量纲 */
    float ps2, ps3, ps4;  /* ps 的二、三、四次方，用于压力补偿公式 */
    /* 初始化成功且两个输出地址有效，才允许读取 */
    if(!SPA06_Initialized || Pcomp == 0 || Tcomp == 0)
    {
        return -1;
    }
    /* 读取传感器当前状态；这里只读一次，不循环等待新数据 */
    status = BSP_I2C_ReadReg(&BSP_I2C1_Bus, SPA06_ADDR_WRITE,SPA06_MEAS_CFG, BSP_I2C_REG_ADDR_8BIT);
    /*
     * 0xCF = 1100 1111，检查以下位，忽略 bit5、bit4：
     *   bit7   = 1：校准系数已就绪。
     *   bit6   = 1：传感器自初始化已完成。
     *   bit3   = 0：使用内部温度传感器。
     *   bit2:0 = 111：连续测量压力和温度。
     *
     * 这些位的预期组合为 0xC7 = 1100 0111。
     */
    if ((status & 0xCFU) != 0xC7U)
    {
        return -1;
    }
    /*
     * bit5：新温度数据就绪。
     * bit4：新压力数据就绪。
     *
     * 两者都就绪后才读取；否则立即返回，交给主循环下次检查。
     * 两者各有新结果，不代表它们在同一时刻采样。
     */
    if ((status & 0x30U) != 0x30U)
    {
        return 0;
    }
    /*
     * 从压力最高字节开始连续读取 6 字节：
     *   data[0:2]：压力，高字节在前。
     *   data[3:5]：温度，高字节在前。
     * 读取对应测量数据后，传感器清除相应的数据就绪标志。
     */
    BSP_I2C_ReadRegs(&BSP_I2C1_Bus,SPA06_ADDR_WRITE,SPA06_PRS_B2,BSP_I2C_REG_ADDR_8BIT,data,6);
    praw = (int32_t)(data[0] << 16 | data[1] << 8 | data[2]);
    traw = (int32_t)(data[3] << 16 | data[4] << 8 | data[5]);
    /*
     * 原始数据采用 24 位补码，bit23 是符号位。
     * 符号位为 1 时减去 2^24，将其转换为正确的 int32_t 负数。
     */
    if(praw & 0X00800000) praw -= 0x01000000;//判断第24位是否为1，是则需要做符号扩展
    if(traw & 0X00800000) traw -= 0x01000000;//判断第24位是否为1，是则需要做符号扩展

    ts = traw/524288.0f;   //温度计单倍采样，对应的kP为524288.0f
    ps = praw/253952.0f;   //压力计16倍采样，对应的kP为253952.0f

    ps2 = ps * ps;   //二次方
	ps3 = ps2 * ps;  //三次方
	ps4 = ps3 * ps;  //四次方
	
	p = c00 + c10 * ps + c20 * ps2 + c30 * ps3 + c40 * ps4 +
				ts * (c01 + c11 * ps + c21 * ps2 + c31 * ps3);  //单位：压力：Pa，温度：℃
	t = c0 * 0.5f + c1 * ts;

    /*
     * 检查计算结果：
     *   isfinite() 排除 NaN 和正负无穷。
     *   气压量程：30000～110000 Pa。
     *   温度量程：-40～85 ℃。
     * 这只是基本有效性检查，不能保证量程内的数据一定准确。
     */
    if (!isfinite(p) || !isfinite(t) ||
        p < 30000.0f || p > 110000.0f ||
        t < -40.0f || t > 85.0f)
    {
        return -2;
    }

    /* 全部检查通过后，再更新调用者的输出变量 */
    *Pcomp = p;
    *Tcomp = t;

    return 1;
}