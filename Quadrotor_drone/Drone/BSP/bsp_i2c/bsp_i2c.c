#include "stm32f10x.h"
#include "bsp_i2c.h"
/*
Ack Address
6B -> QMI8658    六轴姿态传感器  7E是它的广播地址
76 -> SPA06      气压温度传感器
7C -> QMC6309    三轴地磁传感器

*/
const BSP_I2C_Bus BSP_I2C1_Bus = {
    GPIOA,GPIO_Pin_11,GPIO_Pin_12,RCC_APB2Periph_GPIOA};
const BSP_I2C_Bus BSP_I2C2_Bus = {
    GPIOB,GPIO_Pin_10,GPIO_Pin_11,RCC_APB2Periph_GPIOB};
void BSP_I2C_Init(const BSP_I2C_Bus *bus)
{
    RCC_APB2PeriphClockCmd(bus -> GPIO_Clock,ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = bus -> SCL_Pin | bus -> SDA_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_SetBits(bus->GPIOx, bus->SCL_Pin | bus->SDA_Pin);
    GPIO_Init(bus -> GPIOx,&GPIO_InitStructure);
}

static void BSP_I2C_W_SCL(const BSP_I2C_Bus *bus,uint8_t BitValue)
{
    GPIO_WriteBit(bus -> GPIOx,bus -> SCL_Pin,(BitAction)BitValue);
}
static void BSP_I2C_W_SDA(const BSP_I2C_Bus *bus,uint8_t BitValue)
{
    GPIO_WriteBit(bus -> GPIOx,bus -> SDA_Pin,(BitAction)BitValue);
}
static uint8_t BSP_I2C_R_SDA(const BSP_I2C_Bus *bus)
{
    uint8_t BitValue = 0;
    BitValue = GPIO_ReadInputDataBit(bus -> GPIOx,bus -> SDA_Pin);
    return BitValue;
}
/**
 * @brief 发送I2C起始信号
 * @param bus I2C总线配置
 * @return 无
 */
static void BSP_I2C_Start(const BSP_I2C_Bus *bus)
{   
    BSP_I2C_W_SDA(bus,1);
    BSP_I2C_W_SCL(bus,1);
    BSP_I2C_W_SDA(bus,0);
    BSP_I2C_W_SCL(bus,0);
}
/**
 * @brief 发送I2C停止信号
 * @param bus I2C总线配置
 * @return 无
 */
static void BSP_I2C_Stop(const BSP_I2C_Bus *bus)
{
    BSP_I2C_W_SDA(bus,0);
    //BSP_I2C_W_SCL(0);
    BSP_I2C_W_SCL(bus,1);
    BSP_I2C_W_SDA(bus,1);
}
static void BSP_I2C_SendByte(const BSP_I2C_Bus *bus,uint8_t Byte)
{
    for(uint8_t i=0;i<8;i++)
    {
        BSP_I2C_W_SDA(bus,!!(Byte & (0X80>>i)));//通过逻辑与操作，结果可能是0x80,0x10,通过双！！转换为0或1，再发送到I2C1
        BSP_I2C_W_SCL(bus,1);
        BSP_I2C_W_SCL(bus,0);
        //I2C在SCL高电平期间，SDA将数据逐位放到数据线上
    }
}

static uint8_t BSP_I2C_ReceiveByte(const BSP_I2C_Bus *bus)
{
    uint8_t Byte=0x00;
    BSP_I2C_W_SDA(bus,1);
    for(uint8_t i=0;i<8;i++)
    {
        BSP_I2C_W_SCL(bus,1);//先释放SDA主线，让从机有机会把数据放到数据线上
        if(BSP_I2C_R_SDA(bus) == 1)
        {
            Byte |= (0x80 >> i);
        }
        BSP_I2C_W_SCL(bus,0);
    }
    return Byte;
}

static void BSP_I2C_SendACK(const BSP_I2C_Bus *bus,uint8_t AckBit)
{
    BSP_I2C_W_SDA(bus,AckBit);
    BSP_I2C_W_SCL(bus,1);
    BSP_I2C_W_SCL(bus,0);
}

static uint8_t BSP_I2C_ReceiveACK(const BSP_I2C_Bus *bus)
{
    uint8_t AckBit = 0;
    BSP_I2C_W_SDA(bus,1);
    BSP_I2C_W_SCL(bus,1);
    AckBit = BSP_I2C_R_SDA(bus);
    BSP_I2C_W_SCL(bus,0);
    return AckBit;
}
/**
 * @brief 发送I2C寄存器地址
 * @param bus I2C总线配置
 * @param RegAddress 寄存器地址
 * @param RegAddrSize 寄存器地址大小
 * @return 无
 */
static void BSP_I2C_SendRegAddress(const BSP_I2C_Bus *bus,uint16_t RegAddress,BSP_I2C_RegAddrSize RegAddrSize)
{
    if(RegAddrSize == BSP_I2C_REG_ADDR_16BIT)
    {
        BSP_I2C_SendByte(bus,(uint8_t)(RegAddress >> 8));
        BSP_I2C_ReceiveACK(bus);
    }
    BSP_I2C_SendByte(bus,(uint8_t)RegAddress);
    BSP_I2C_ReceiveACK(bus);

}
/**
 * @brief 向I2C1写入一个字节
 * @param bus I2C总线配置
 * @param SlaveAddress 从机地址
 * @param RegAddress 寄存器地址
 * @param RegAddrSize 寄存器地址大小
 * @param Byte 要写入的字节
 * @return 无
 */
void BSP_I2C_WriteReg(const BSP_I2C_Bus *bus,uint8_t SlaveAddress,uint16_t RegAddress,BSP_I2C_RegAddrSize RegAddrSize,uint8_t Byte)
{
    BSP_I2C_Start(bus);
    BSP_I2C_SendByte(bus,SlaveAddress);
    BSP_I2C_ReceiveACK(bus);
    BSP_I2C_SendRegAddress(bus,RegAddress,RegAddrSize);
    BSP_I2C_SendByte(bus,Byte);
    BSP_I2C_ReceiveACK(bus);
    BSP_I2C_Stop(bus);
}
/**
 * @brief 向I2C写入多个字节
 * @param SlaveAddress 从机地址
 * @param RegAddress 寄存器地址
 * @param RegAddrSize 寄存器地址大小
 * @param Bytes 要写入的字节数组
 * @param ByteCount 要写入的字节数
 * @return 无
 */
void BSP_I2C_WriteRegs(const BSP_I2C_Bus *bus,uint8_t SlaveAddress,uint16_t RegAddress,BSP_I2C_RegAddrSize RegAddrSize,uint8_t *Bytes,uint8_t ByteCount)
{
    BSP_I2C_Start(bus);
    BSP_I2C_SendByte(bus,SlaveAddress);
    BSP_I2C_ReceiveACK(bus);
    BSP_I2C_SendRegAddress(bus,RegAddress,RegAddrSize);
    for(uint8_t i=0;i<ByteCount;i++)
    {
        BSP_I2C_SendByte(bus,Bytes[i]);
        BSP_I2C_ReceiveACK(bus);
    }
    BSP_I2C_Stop(bus);
}
/**
 * @brief 从I2C读取一个字节
 * @param bus I2C总线配置
 * @param SlaveAddress 从机地址
 * @param RegAddress 寄存器地址
 * @param RegAddrSize 寄存器地址大小
 * @return 读取到的字节
 */
uint8_t BSP_I2C_ReadReg(const BSP_I2C_Bus *bus,uint8_t SlaveAddress,uint16_t RegAddress,BSP_I2C_RegAddrSize RegAddrSize)
{
    uint8_t Byte = 0x00;
    BSP_I2C_Start(bus);
    BSP_I2C_SendByte(bus,SlaveAddress);
    BSP_I2C_ReceiveACK(bus);
    BSP_I2C_SendRegAddress(bus,RegAddress,RegAddrSize);

    BSP_I2C_Start(bus);
    BSP_I2C_SendByte(bus,SlaveAddress | 0x01);//读取时，从机地址的第7位为1
    BSP_I2C_ReceiveACK(bus);
    Byte = BSP_I2C_ReceiveByte(bus);
    BSP_I2C_SendACK(bus,1);
    BSP_I2C_Stop(bus);
    return Byte;
}
/**
 * @brief 从I2C读取多个字节
 * @param bus I2C总线配置
 * @param SlaveAddress 从机地址
 * @param RegAddress 寄存器地址
 * @param RegAddrSize 寄存器地址大小
 * @param Bytes 要读取的字节数组
 * @param ByteCount 要读取的字节数
 * @return 无
 */
void BSP_I2C_ReadRegs(const BSP_I2C_Bus *bus,uint8_t SlaveAddress,uint16_t RegAddress,BSP_I2C_RegAddrSize RegAddrSize,uint8_t *Bytes,uint8_t ByteCount)
{
    BSP_I2C_Start(bus);
    BSP_I2C_SendByte(bus,SlaveAddress);
    BSP_I2C_ReceiveACK(bus);
    BSP_I2C_SendRegAddress(bus,RegAddress,RegAddrSize);

    BSP_I2C_Start(bus);
    BSP_I2C_SendByte(bus,SlaveAddress | 0x01);//读取时，从机地址的第7位为1
    BSP_I2C_ReceiveACK(bus);
    for(uint8_t i=0;i<ByteCount;i++)
    {
        Bytes[i] = BSP_I2C_ReceiveByte(bus);
        if(i < ByteCount - 1)
        {
            BSP_I2C_SendACK(bus,0);//如果SDA拉低，表示响应，继续接收
        }
        else
        {
            BSP_I2C_SendACK(bus,1);//如果SDA拉高，表示不响应，结束接收
        }
    }
    BSP_I2C_Stop(bus);
}