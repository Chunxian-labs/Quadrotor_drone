#ifndef SPA06_CONFIG_H
#define SPA06_CONFIG_H
// === 测量数据输出寄存器 (只读，24位补码) ===
#define SPA06_PRS_B2            0x00  // 气压数据 [23:16] (最高字节 MSB)
#define SPA06_PRS_B1            0x01  // 气压数据 [15:8]  (中间字节)
#define SPA06_PRS_B0            0x02  // 气压数据 [7:0]   (最低字节 XLSB)

#define SPA06_TMP_B2            0x03  // 温度数据 [23:16] (最高字节 MSB)
#define SPA06_TMP_B1            0x04  // 温度数据 [15:8]  (中间字节)
#define SPA06_TMP_B0            0x05  // 温度数据 [7:0]   (最低字节 XLSB)

// === 配置寄存器 ===
#define SPA06_PRS_CFG           0x06  // 气压配置寄存器：设置气压测量速率(PM_RATE)和过采样精度(PM_PRC)
#define SPA06_TMP_CFG           0x07  // 温度配置寄存器：设置温度测量速率(TMP_RATE)和过采样精度(TMP_PRC)
#define SPA06_MEAS_CFG          0x08  // 测量配置寄存器：设置工作模式(待机/单次/后台连续)及读取数据就绪状态标志
#define SPA06_CFG_REG           0x09  // 中断与FIFO配置寄存器：设置中断极性、FIFO使能、SPI模式及数据移位

// === 状态标志寄存器 (只读) ===
#define SPA06_INT_STS           0x0A  // 中断状态寄存器：指示气压/温度数据就绪或FIFO满中断 (读取后自动清零)
#define SPA06_FIFO_STS          0x0B  // FIFO状态寄存器：指示FIFO是满(FULL)还是空(EMPTY)

// === 复位与ID ===
#define SPA06_RESET             0x0C  // 复位寄存器：写入 0x09 执行软复位；写 0x80 清空FIFO
#define SPA06_ID                0x0D  // 产品及版本ID寄存器：用于识别芯片

// === 校准系数寄存器 ===
#define SPA06_COEF              0x10  // 校准系数起始地址 (0x10 ~ 0x24)：存储用于计算真实气压和温度的补偿系数

#endif