/* QMI8658 六轴驱动：配置、命令握手、同步锁定读取。
 * 初始化前先调用 System_timer_Init()，提供 TIM3 微秒延时。
 * 输出为芯片坐标系原始计数；校准、单位转换由 imu_sensors 负责。
 */
#include "QMI8658.h"
#include "QMI8658_Config.h"
#include "bsp_i2c.h"
#include "bsp_timer.h"
#include <stdint.h>
#define QMI8658_ADDR_7BIT          0x6B
#define QMI8658_ADDR_WRITE         (QMI8658_ADDR_7BIT << 1)
/* 运行状态：1 表示时钟门控命令握手成功；GetData 依赖此状态。 */
volatile uint8_t QmiClockGatingOK = 0;
/* 初始化后的寄存器快照，供 Watch 检查，不是实时值。 */
volatile uint8_t QmiCtrl2Read;
volatile uint8_t QmiCtrl3Read;
volatile uint8_t QmiCtrl5Read;
volatile uint8_t QmiCtrl7Read;

/* 单寄存器读写；BSP 接口使用左移一位的设备地址。 */
static uint8_t QMI_ReadReg(uint8_t reg)
{
    return BSP_I2C_ReadReg(
        &BSP_I2C1_Bus,
        QMI8658_ADDR_WRITE,
        reg,
        BSP_I2C_REG_ADDR_8BIT);
}

static void QMI_WriteReg(uint8_t reg, uint8_t value)
{
    BSP_I2C_WriteReg(
        &BSP_I2C1_Bus,
        QMI8658_ADDR_WRITE,
        reg,
        BSP_I2C_REG_ADDR_8BIT,
        value);
}

/* 初始化专用：最多轮询 1000 次，显式延时至多 100 ms，另有 I2C 开销。
 * expected=1 等待命令完成；expected=0 等待 ACK 后完成标志清除。
 */
static uint8_t QMI_WaitCmdDone(uint8_t expected)
{
    for (uint16_t i = 0; i < 1000U; i++)
    {
        uint8_t status = QMI_ReadReg(QMI8658_STATUSINT);

        /* 0xFF 可能是总线未正常应答，不能当作完成 */
        if (status != 0xFF)
        {
            uint8_t done = (status >> 7) & 1U;

            if (done == expected)
            {
                return 1;
            }
        }
        System_Delay_us(100);
    }

    return 0;
}

/* 同步读取前置配置：关闭芯片内部 AHB 时钟门控，避免数据错位。
 * 流程：清旧状态 -> CAL 参数与 CTRL9 命令 -> 等完成 -> ACK -> 等清除。
 * 与 STM32 外设时钟、中断无关；等待失败返回 0。
 */
static uint8_t QMI_DisableClockGating(void)
{
    /* 清理命令握手状态 */
    QMI_WriteReg(QMI8658_CTRL9, 0x00);

    if (!QMI_WaitCmdDone(0))
    {
        return 0;
    }

    /* CAL1_L = 1：请求关闭内部 AHB 时钟门控 */
    QMI_WriteReg(QMI8658_CAL1_L, 0x01);
    QMI_WriteReg(QMI8658_CTRL9, 0x12);

    if (!QMI_WaitCmdDone(1))
    {
        QMI_WriteReg(QMI8658_CTRL9, 0x00);
        return 0;
    }

    /* 确认命令完成 */
    QMI_WriteReg(QMI8658_CTRL9, 0x00);

    return QMI_WaitCmdDone(0);
}

/* 初始化：保持已经实测通过的寄存器值与写入顺序。 */
void QMI8658_Init(void)
{
    BSP_I2C_Init(&BSP_I2C1_Bus);
    /* 串行接口配置，启用地址自增。 */
    QMI_WriteReg(QMI8658_CTRL1, 0x60);
    /* 加速度量程 ±16 g；六轴模式名义 ODR 896.8 Hz。 */
    QMI_WriteReg(QMI8658_CTRL2, 0x33);
    /* 陀螺仪量程 ±2048 °/s；名义 ODR 896.8 Hz。 */
    QMI_WriteReg(QMI8658_CTRL3, 0x73);
    /* 两种低通均开启：加速度模式 00，陀螺仪模式 10（约 48.3 Hz）。 */
    QMI_WriteReg(QMI8658_CTRL5, 0x51);
    /* 先使能内部传感器，等待芯片处理命令 */
    QMI_WriteReg(QMI8658_CTRL7, 0x03);
    System_Delay_ms(10);

    /* 通过 STATUSINT.bit7 轮询命令完成状态 */
    uint8_t ctrl8 = QMI_ReadReg(QMI8658_CTRL8);
    QMI_WriteReg(QMI8658_CTRL8, ctrl8 | 0x80);

    QmiClockGatingOK = QMI_DisableClockGating();
    /* 成功开启六轴 SyncSample；失败则关闭传感器。 */
    QMI_WriteReg(QMI8658_CTRL7,
                 QmiClockGatingOK ? 0x83 : 0x00);


    /* 成功时预期为 0x33、0x73、0x51、0x83。 */
    QmiCtrl2Read = QMI_ReadReg(QMI8658_CTRL2);
    QmiCtrl3Read = QMI_ReadReg(QMI8658_CTRL3);
    QmiCtrl5Read = QMI_ReadReg(QMI8658_CTRL5);
    QmiCtrl7Read = QMI_ReadReg(QMI8658_CTRL7);
}

uint8_t QMI8658_GetID(void)
{
    return QMI_ReadReg(QMI8658_WHO_AM_I);
}

/* 成功返回 1；锁定等待失败返回 0，输出保持原值。
 * 连续读不等于原子快照：先锁定同一帧，读完 GZ_H 自动解锁。
 * BSP 尚不反馈 ACK/传输错误，本函数无法检测全部总线故障。
 */
uint8_t QMI8658_GetData(
    int16_t *AccX, int16_t *AccY, int16_t *AccZ,
    int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t Data[12];
    uint8_t locked = 0;

    if (!QmiClockGatingOK)
    {
        return 0;
    }

    /*
     * 有限次数轮询，不无限等待。
     * STATUSINT bit0：有数据；读取状态会触发锁定
     * STATUSINT bit1：数据已锁定
     */
    for (uint16_t i = 0; i < 100U; i++)
    {
        uint8_t status = QMI_ReadReg(QMI8658_STATUSINT);

        if (status == 0xFF)
        {
            break;
        }

        if (status & 0x01)
        {
            /*
             * 当前 ODR 对应的锁定等待时间为 6 us，
             * 留少量余量后再确认。
             */
            if (!(status & 0x02))
            {
                System_Delay_us(10);
                status = QMI_ReadReg(QMI8658_STATUSINT);
            }

            if (status != 0xFF && (status & 0x02))
            {
                locked = 1;
                break;
            }
        }

        System_Delay_us(10);
    }

    if (!locked)
    {
        /* 尝试释放可能残留的锁定，本次不输出数据 */
        (void)QMI_ReadReg(QMI8658_GZ_H);
        return 0;
    }

    /*
     * 从 AX_L 连续读到 GZ_H；
     * 最后一个字节读完后，芯片自动释放锁定。
     */
    BSP_I2C_ReadRegs(
        &BSP_I2C1_Bus,
        QMI8658_ADDR_WRITE,
        QMI8658_AX_L,
        BSP_I2C_REG_ADDR_8BIT,
        Data, 12);

    *AccX  = (int16_t)((uint16_t)Data[1]  << 8 | Data[0]);
    *AccY  = (int16_t)((uint16_t)Data[3]  << 8 | Data[2]);
    *AccZ  = (int16_t)((uint16_t)Data[5]  << 8 | Data[4]);
    *GyroX = (int16_t)((uint16_t)Data[7]  << 8 | Data[6]);
    *GyroY = (int16_t)((uint16_t)Data[9]  << 8 | Data[8]);
    *GyroZ = (int16_t)((uint16_t)Data[11] << 8 | Data[10]);

    return 1;
}
