#ifndef QMI8658_CONFIG_H
#define QMI8658_CONFIG_H
/* ================= QMI8658A 寄存器地址定义 ================= */

// 芯片信息寄存器
#define QMI8658_WHO_AM_I            0x00    // 设备标识寄存器，正常读出 0x05
#define QMI8658_REVISION_ID         0x01    // 设备版本ID，默认 0x7C

// 设置与控制寄存器
#define QMI8658_CTRL1               0x02    // 串行接口与传感器使能（bit6 置1开启地址自增）
#define QMI8658_CTRL2               0x03    // 加速度计配置（满量程、ODR、自检）
#define QMI8658_CTRL3               0x04    // 陀螺仪配置（满量程、ODR、自检）
//#define QMI8658_Reserved          0x05    // 保留
#define QMI8658_CTRL5               0x06    // 传感器低通滤波器(LPF)配置
//#define QMI8658_Reserved          0x07    // 保留
#define QMI8658_CTRL7               0x08    // 使能传感器（bit1使能陀螺仪，bit0使能加速度计）
#define QMI8658_CTRL8               0x09    // 运动检测控制（Tap/Any/No/Sig-Motion）
#define QMI8658_CTRL9               0x0A    // 主机命令寄存器（特殊握手协议）
#define QMI8658_CAL1_L               0x0B
#define QMI8658_STATUSINT            0x2D
// 传感器数据输出寄存器（16位二进制补码，低字节在前）
#define QMI8658_AX_L                0x35    // X轴加速度低8位
#define QMI8658_AX_H                0x36    // X轴加速度高8位
#define QMI8658_AY_L                0x37    // Y轴加速度低8位
#define QMI8658_AY_H                0x38    // Y轴加速度高8位
#define QMI8658_AZ_L                0x39    // Z轴加速度低8位
#define QMI8658_AZ_H                0x3A    // Z轴加速度高8位
#define QMI8658_GX_L                0x3B    // X轴角速度低8位
#define QMI8658_GX_H                0x3C    // X轴角速度高8位
#define QMI8658_GY_L                0x3D    // Y轴角速度低8位
#define QMI8658_GY_H                0x3E    // Y轴角速度高8位
#define QMI8658_GZ_L                0x3F    // Z轴角速度低8位
#define QMI8658_GZ_H                0x40    // Z轴角速度高8位
#endif