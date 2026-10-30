
/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "vl53l1_platform.h"
#include "bsp_i2c.h"
#include "bsp_timer.h"

void VL53L1_I2C_Init(void)
{
    BSP_I2C_Init(&BSP_I2C2_Bus);
}

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    if (dev > 0xFEU || (dev & 1U) != 0U ||
        (count != 0U && pdata == NULL) ||
        count > (0x10000UL - (uint32_t)index))
    {
        return -1;
    }
    while (count != 0U)
    {
        uint8_t chunk = count > 255U ? 255U : (uint8_t)count;
        BSP_I2C_WriteRegs(&BSP_I2C2_Bus, (uint8_t)dev, index,
                       BSP_I2C_REG_ADDR_16BIT, pdata, chunk);
        pdata += chunk;
        index = (uint16_t)(index + chunk);
        count -= chunk;
    }
    return 0;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    if (dev > 0xFEU || (dev & 1U) != 0U ||
        (count != 0U && pdata == NULL) ||
        count > (0x10000UL - (uint32_t)index))
    {
        return -1;
    }
    while (count != 0U)
    {
        uint8_t chunk = count > 255U ? 255U : (uint8_t)count;
        BSP_I2C_ReadRegs(&BSP_I2C2_Bus, (uint8_t)dev, index,
                       BSP_I2C_REG_ADDR_16BIT, pdata, chunk);
        pdata += chunk;
        index = (uint16_t)(index + chunk);
        count -= chunk;
    }
    return 0;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
		
	return VL53L1_WriteMulti(dev, index, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
	
	uint8_t buf[2];
	
	buf[0] = data >> 8;
	buf[1] = data & 0xFF;
	
	return VL53L1_WriteMulti(dev, index, buf, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
	uint8_t buf[4];
	
	buf[0] = (data >> 24) & 0xFF;
	buf[1] = (data >> 16) & 0xFF;
	buf[2] = (data >> 8) & 0xFF;
	buf[3] = data & 0xFF;
	
	return VL53L1_WriteMulti(dev, index, buf, 4);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	
	return VL53L1_ReadMulti(dev, index, data, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	
	uint8_t buf[2];
    int8_t status;
    if (data == NULL) return -1;
    status = VL53L1_ReadMulti(dev, index, buf, 2);
    if (status != 0) return status;
	
	*data = (buf[0] << 8) | buf[1];
	
	return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	uint8_t buf[4];
    int8_t status;
    if (data == NULL) return -1;
    status = VL53L1_ReadMulti(dev, index, buf, 4);
    if (status != 0) return status;
	
	*data = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
	
	return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	
	(void)dev;
    if (wait_ms < 0) return -1;
    System_Delay_ms((uint32_t)wait_ms);
	
	return 0;
}
