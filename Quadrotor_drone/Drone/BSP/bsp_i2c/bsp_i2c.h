#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "stm32f10x.h"
#include <stdint.h>

typedef struct
{
    GPIO_TypeDef *GPIOx;     // SCL和SDA所在的GPIO端口
    uint16_t SCL_Pin;        // SCL引脚
    uint16_t SDA_Pin;        // SDA引脚
    uint32_t GPIO_Clock;     // GPIO端口的APB2时钟
}BSP_I2C_Bus;
typedef enum
{
    BSP_I2C_REG_ADDR_8BIT,
    BSP_I2C_REG_ADDR_16BIT
}BSP_I2C_RegAddrSize;
// 软件I2C总线配置
extern const BSP_I2C_Bus BSP_I2C1_Bus; // SCL: PA11, SDA: PA12
extern const BSP_I2C_Bus BSP_I2C2_Bus; // SCL: PB10, SDA: PB11

// SlaveAddress传入7位从机地址左移1位后的写地址，bit0为0
void BSP_I2C_Init(const BSP_I2C_Bus *bus);
void BSP_I2C_WriteReg(const BSP_I2C_Bus *bus,uint8_t SlaveAddress,uint16_t RegAddress,BSP_I2C_RegAddrSize RegAddrSize,uint8_t Byte);
void BSP_I2C_WriteRegs(const BSP_I2C_Bus *bus,uint8_t SlaveAddress,uint16_t RegAddress,BSP_I2C_RegAddrSize RegAddrSize,uint8_t *Bytes,uint8_t ByteCount);
uint8_t BSP_I2C_ReadReg(const BSP_I2C_Bus *bus,uint8_t SlaveAddress,uint16_t RegAddress,BSP_I2C_RegAddrSize RegAddrSize);
void BSP_I2C_ReadRegs(const BSP_I2C_Bus *bus,uint8_t SlaveAddress,uint16_t RegAddress,BSP_I2C_RegAddrSize RegAddrSize,uint8_t *Bytes,uint8_t ByteCount);

#endif
