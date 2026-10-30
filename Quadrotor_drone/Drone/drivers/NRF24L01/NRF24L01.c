#include "NRF24L01.h"
#include "stm32f10x.h"
#define NRF24L01_TX_PACKET_WIDTH 32  //发送数据包宽度，范围为1~32字节
#define NRF24L01_RX_PACKET_WIDTH 32  //接收通道0数据包宽度，范围为1~32字节
uint8_t NRF24L01_TxAddress[5] = {0x11,0x22,0x33,0x44,0x55};  //发送地址，固定为5字节
uint8_t NRF24L01_RxAddress[5] = {0x11,0x22,0x33,0x44,0x55};  //接收通道0地址，固定为5字节
uint8_t NRF24L01_TxPacket[NRF24L01_TX_PACKET_WIDTH] = {0};  //发送数据包
uint8_t NRF24L01_RxPacket[NRF24L01_RX_PACKET_WIDTH] = {0};  //接收数据包
/**
  * 函    数：NRF24L01写CE高低电平
  * 参    数：要写入CE的电平值，范围：0/1
  * 返 回 值：无
  * 说    明：当上层函数需要写CE时，此函数会被调用
  *           用户需要根据参数传入的值，将CE置为高电平或者低电平
  *           当参数传入0时，置CE为低电平，当参数传入1时，置CE为高电平
  */
 static void NRF24L01_W_CE(uint8_t Bitvalue)
 {
    GPIO_WriteBit(NRF24L01_W_CE_PORT,NRF24L01_W_CE_PIN,(BitAction)Bitvalue);
 }
 /**
  * 函    数：NRF24L01写CSN高低电平
  * 参    数：要写入CSN的电平值，范围：0/1
  * 返 回 值：无
  * 说    明：当上层函数需要写CSN时，此函数会被调用
  *           用户需要根据参数传入的值，将CSN置为高电平或者低电平
  *           当参数传入0时，置CSN为低电平，当参数传入1时，置CSN为高电平
  */
 static void NRF24L01_W_CSN(uint8_t Bitvalue)
 {
    GPIO_WriteBit(NRF24L01_W_CSN_PORT,NRF24L01_W_CSN_PIN,(BitAction)Bitvalue);
 }
 /**
  * 函    数：NRF24L01写SCK高低电平
  * 参    数：要写入SCK的电平值，范围：0/1
  * 返 回 值：无
  * 说    明：当上层函数需要写SCK时，此函数会被调用
  *           用户需要根据参数传入的值，将SCK置为高电平或者低电平
  *           当参数传入0时，置SCK为低电平，当参数传入1时，置SCK为高电平
  */
 static void NRF24L01_W_SCK(uint8_t Bitvalue)
 {
    GPIO_WriteBit(NRF24L01_W_SCK_PORT,NRF24L01_W_SCK_PIN,(BitAction)Bitvalue);
 }
/**
  * 函    数：NRF24L01写MOSI高低电平
  * 参    数：要写入MOSI的电平值，范围：0/1
  * 返 回 值：无
  * 说    明：当上层函数需要写MOSI时，此函数会被调用
  *           用户需要根据参数传入的值，将MOSI置为高电平或者低电平
  *           当参数传入0时，置MOSI为低电平，当参数传入1时，置MOSI为高电平
  */
 static void NRF24L01_W_MOSI(uint8_t Bitvalue)
 {
    GPIO_WriteBit(NRF24L01_W_MOSI_PORT,NRF24L01_W_MOSI_PIN,(BitAction)Bitvalue);
 }
/**
  * 函    数：NRF24L01读MISO高低电平
  * 参    数：无
  * 返 回 值：读取得到MISO的电平值，范围：0/1
  * 说    明：当上层函数需要读MISO时，此函数会被调用
  *           用户需要读取MISO引脚，返回此引脚的高低电平状态
  *           当MISO为高电平时，返回1，当MISO为低电平时，返回0
  */
 static uint8_t NRF24L01_R_MISO(void)
 {
    return GPIO_ReadInputDataBit(NRF24L01_W_MISO_PORT,NRF24L01_W_MISO_PIN);
 }
/**
  * 函    数：NRF24L01引脚初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：当上层函数需要初始化时，此函数会被调用
  *           用户需要将CSN、CE、MISO、SCK引脚初始化为推挽输出模式，MISO引脚初始化为上拉输入模式
  */
 static void NRF24L01_GPIO_Init(void)
 {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    /*解除JTAG复用*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
    /*将CE、CSN、SCK、MOSI引脚初始化为推挽输出模式*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = NRF24L01_W_CE_PIN | NRF24L01_W_CSN_PIN | NRF24L01_W_SCK_PIN | NRF24L01_W_MOSI_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /*将MISO引脚初始化为上拉输入模式*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = NRF24L01_W_MISO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*置引脚初始化后的默认电平*/
	NRF24L01_W_CE(0);		//CE默认为0，退出收发模式
	NRF24L01_W_CSN(1);		//CSN默认为1，不选中从机
	NRF24L01_W_SCK(0);		//SCK默认为0，对应SPI模式0
	NRF24L01_W_MOSI(0);		//MOSI默认电平随意，1和0均可
 }

/**
  * 函    数：SPI交换一个字节
  * 参    数：Byte 要发送的一个字节数据，范围：0x00~0xFF
  * 返 回 值：接收得到的一个字节数据，范围：0x00~0xFF
  */ 
 static uint8_t NRF24L01_SPI_SwapByte(uint8_t Byte)
{
    for(int i = 0;i < 8; i++)
    {
        if((Byte & 0x80) == 0x80)
        {
            /*SPI为高位先行，因此移出高位至MOSI引脚*/
            NRF24L01_W_MOSI(1);
        }
        else
        {
            NRF24L01_W_MOSI(0);
        }
        NRF24L01_W_SCK(1);
        Byte <<= 1;
        if(NRF24L01_R_MISO() == 1)
        {
            Byte |= 0x01;
        }
        NRF24L01_W_SCK(0);
    }
    return Byte;
}
/**
  * 函    数：NRF24L01读取寄存器（一个字节）
  * 参    数：RegAddress 指定寄存器地址，范围：0x00~0x1F
  * 返 回 值：指定寄存器的数据，范围：0x00~0xFF
  */
 static uint8_t NRF24L01_ReadRegister(uint8_t RegAddress)
 {
    NRF24L01_W_CSN(0);
    NRF24L01_SPI_SwapByte(NRF24L01_R_REGISTER | RegAddress); //发送读取寄存器指令,获得了当前寄存器的状态
    uint8_t Data = NRF24L01_SPI_SwapByte(NRF24L01_NOP);      //后续交换，读取寄存器的值
    NRF24L01_W_CSN(1);
    return Data;
 }
 /**
  * 函    数：NRF24L01读取寄存器（多个字节）
  * 参    数：RegAddress 指定寄存器的地址，范围：0x00~0x1F
  * 参    数：DataArray 读取得到的数据数组，输出参数
  * 参    数：Datalength 指定读取的数量，范围：0~5
  * 返 回 值：无
  */
//  static void NRF24L01_ReadRegisters(uint8_t RegAddress,uint8_t *DataArray,uint8_t Datalength)
//  {
//     NRF24L01_W_CSN(0);
//     NRF24L01_SPI_SwapByte(NRF24L01_R_REGISTER | RegAddress); //发送读取寄存器指令,获得了当前寄存器的状态
//     for(int i = 0;i < Datalength;i++)
//     {
//         DataArray[i] = NRF24L01_SPI_SwapByte(NRF24L01_NOP);      //后续交换，读取寄存器的值
//     }
//     NRF24L01_W_CSN(1);
//  }
 /**
  * 函    数：NRF24L01写入寄存器（一个字节）
  * 参    数：RegAddress 指定寄存器地址，范围：0x00~0x1F
  * 参    数：Data 要写入的一个字节数据，范围：0x00~0xFF
  * 返 回 值：无
  */
 static void NRF24L01_WriteRegister(uint8_t RegAddress,uint8_t Data)
 {
    NRF24L01_W_CSN(0);
    NRF24L01_SPI_SwapByte(NRF24L01_W_REGISTER | RegAddress); //发送写入寄存器指令,获得了当前寄存器的状态
    NRF24L01_SPI_SwapByte(Data); //发送要写入的数据
    NRF24L01_W_CSN(1);
 }
 /**
  * 函    数：NRF24L01写入寄存器（多个字节）
  * 参    数：RegAddress 指定寄存器地址，范围：0x00~0x1F
  * 参    数：DataArray 要写入的数据数组，输入参数
  * 参    数：Datalength 指定写入的数量，范围：0~5
  * 返 回 值：无
  */
 static void NRF24L01_WriteRegisters(uint8_t RegAddress,uint8_t *DataArray,uint8_t Datalength)
 {
    NRF24L01_W_CSN(0);
    NRF24L01_SPI_SwapByte(NRF24L01_W_REGISTER | RegAddress); //发送写入寄存器指令,获得了当前寄存器的状态
    for(int i = 0;i < Datalength;i++)
    {
        NRF24L01_SPI_SwapByte(DataArray[i]); //发送要写入的数据
    }
    NRF24L01_W_CSN(1);
 }
 /**
  * 函    数：NRF24L01读取Rx有效载荷
  * 参    数：DataArray 读取得到的数据数组，输出参数
  * 参    数：Datalength 指定读取的数量，范围：0~32
  * 返 回 值：无
  */
 static void NRF24L01_ReadRxPayload(uint8_t *DataArray,uint8_t Datalength)
 {
    NRF24L01_W_CSN(0);
    NRF24L01_SPI_SwapByte(NRF24L01_R_RX_PAYLOAD); //发送读取Rx有效载荷指令,获得了当前寄存器的状态
    for(int i = 0;i < Datalength;i++)
    {
        DataArray[i] = NRF24L01_SPI_SwapByte(NRF24L01_NOP);      //后续交换，读取Rx有效载荷的值
    }
    NRF24L01_W_CSN(1);
 }
 /**
  * 函    数：NRF24L01写入Tx有效载荷
  * 参    数：DataArray 要写入的数据数组，输入参数
  * 参    数：Count 指定写入的数量，范围：0~5
  * 返 回 值：无
  */
 static void NRF24L01_WriteTxPayload(uint8_t *DataArray,uint8_t Datalength)
 {
    NRF24L01_W_CSN(0);
    NRF24L01_SPI_SwapByte(NRF24L01_W_TX_PAYLOAD); //发送写入Tx有效载荷指令,获得了当前寄存器的状态
    for(int i = 0;i < Datalength;i++)
    {
        NRF24L01_SPI_SwapByte(DataArray[i]); //发送要写入的数据
    }
    NRF24L01_W_CSN(1);
 }
 /**
  * 函    数：NRF24L01清空Tx FIFO的所有数据
  * 参    数：无
  * 返 回 值：无
  */
 static void NRF24L01_FlushTx(void)
 {
    NRF24L01_W_CSN(0);
    NRF24L01_SPI_SwapByte(NRF24L01_FLUSH_TX); //发送清空Tx FIFO指令,获得了当前寄存器的状态
    NRF24L01_W_CSN(1);
 }
 /**
  * 函    数：NRF24L01清空Rx FIFO的所有数据
  * 参    数：无
  * 返 回 值：无
  */
 static void NRF24L01_FlushRx(void)
 {
    NRF24L01_W_CSN(0);
    NRF24L01_SPI_SwapByte(NRF24L01_FLUSH_RX); //发送清空Rx FIFO指令,获得了当前寄存器的状态
    NRF24L01_W_CSN(1);
 }
 /**
  * 函    数：NRF24L01读取状态寄存器
  * 参    数：无
  * 返 回 值：状态寄存器的值，范围：0x00~0xFF
  */
 static uint8_t NRF24L01_ReadStatus(void)
 {
    NRF24L01_W_CSN(0);
    uint8_t Data = NRF24L01_SPI_SwapByte(NRF24L01_NOP); //发送读取状态寄存器指令,获得了当前寄存器的状态
    NRF24L01_W_CSN(1);
    return Data;
 }
 /**
  * 函    数：NRF24L01进入掉电模式（CE = 0，PWR_UP = 0）
  * 参    数：无
  * 返 回 值：无
  */
//  static void NRF24L01_PowerDown(void)
//  {
//    /*CE置0，退出收发模式*/
//    NRF24L01_W_CE(0);
//    /*PWR_UP置0，进入掉电模式*/
//    uint8_t config = NRF24L01_ReadRegister(NRF24L01_CONFIG);
//    if(config == 0xFF){return;}
//    config &= ~0x02; //把配置寄存器的第二位变成0，即pwr_up = 0
//    NRF24L01_WriteRegister(NRF24L01_CONFIG,config);
//  }
 /**
  * 函    数：NRF24L01进入待机模式1（CE = 0，PWR_UP = 1）
  * 参    数：无
  * 返 回 值：无
  */
//  static void NRF24L01_StandbyI(void)
//  {
//    /*CE置0，退出收发模式*/
//    NRF24L01_W_CE(0);
//    uint8_t config = NRF24L01_ReadRegister(NRF24L01_CONFIG);
//    if(config == 0xFF){return;}
//    config |= 0x02; //把配置寄存器的第二位变成1，即pwr_up = 1
//    NRF24L01_WriteRegister(NRF24L01_CONFIG,config);
//  }
 /**
  * 函    数：NRF24L01进入接收模式（CE = 1，PWR_UP = 1，PRIM_RX = 1）
  * 参    数：无
  * 返 回 值：无
  */
 static void NRF24L01_RxModel(void)
 {
   /*CE置0，退出收发模式*/
   NRF24L01_W_CE(0);
   uint8_t config = NRF24L01_ReadRegister(NRF24L01_CONFIG);
   if(config == 0xFF){return;}
   config |= 0x02; //把配置寄存器的第二位变成1，即pwr_up = 1
   config |= 0x01; //把配置寄存器的第一位变成1，即prim_rx = 1
   NRF24L01_WriteRegister(NRF24L01_CONFIG,config);
   /*CE置1，进入接收模式*/
   NRF24L01_W_CE(1);
 }
 /**
  * 函    数：NRF24L01进入发送模式（CE = 1，PWR_UP = 1，PRIM_RX = 0）
  * 参    数：无
  * 返 回 值：无
  */
 static void NRF24L01_TxModel(void)
 {
   /*CE置0，退出收发模式*/
   NRF24L01_W_CE(0);
   uint8_t config = NRF24L01_ReadRegister(NRF24L01_CONFIG);
   if(config == 0xFF){return;}
   config |= 0x02; //把配置寄存器的第二位变成1，即pwr_up = 1
   config &= ~0x01; //把配置寄存器的第一位变成0，即prim_rx = 0
   NRF24L01_WriteRegister(NRF24L01_CONFIG,config);
   /*CE置1，进入发送模式*/
   NRF24L01_W_CE(1);
 }
 /**
  * 函    数：NRF24L01初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：使用前，需要调用此初始化函数
  */
 void NRF24L01_Init(void)
 {
   NRF24L01_GPIO_Init();
   NRF24L01_WriteRegister(NRF24L01_CONFIG,0x08);  //配置寄存器，不屏蔽中断，使能CRC，CRC为1字节，PWR_UP = 0，PRIM_RX = 0
   NRF24L01_WriteRegister(NRF24L01_EN_AA,0x3F);   //使能自动应答功能，使能所有通道的自动应答
   NRF24L01_WriteRegister(NRF24L01_EN_RXADDR,0x01); //使能接收通道，只开启接收通道0
   NRF24L01_WriteRegister(NRF24L01_SETUP_AW,0x03); //设置地址宽度，宽度为5字节
   NRF24L01_WriteRegister(NRF24L01_SETUP_RETR,0x03); //设置自动重传，间隔250us，重传三次
   NRF24L01_WriteRegister(NRF24L01_RF_CH,0X02);    //射频通道，频率为(2400 + 2)MHz = 2.402GHz
   NRF24L01_WriteRegister(NRF24L01_RF_SETUP,0x0E); //设置RF配置寄存器，设置RF功率为0dBm，数据速率为2Mbps
   /*接收通道0的数据包宽度，设置为宏定义NRF24L01_RX_PACKET_WIDTH指定的值*/
   NRF24L01_WriteRegister(NRF24L01_RX_PW_P0,NRF24L01_RX_PACKET_WIDTH);
   /*接收通道0地址，设置为全局数组NRF24L01_RxAddress指定的地址，地址宽度固定为5字节*/
   NRF24L01_WriteRegisters(NRF24L01_RX_ADDR_P0,NRF24L01_RxAddress,5);

   NRF24L01_FlushTx(); //清空Tx FIFO的所有数据
   NRF24L01_FlushRx(); //清空Rx FIFO的所有数据
   /*给状态寄存器的位4（MAX_RT）、位5（TX_DS）和位6（RX_DR）写1，清标志位*/
   NRF24L01_WriteRegister(NRF24L01_STATUS,0x70); 

   NRF24L01_RxModel(); //初始化完成，默认进入接收模式
   }
/**
  * 函    数：NRF24L01发送数据包
  * 参    数：无
  * 返 回 值：发送标志位，方便用户了解发送状态
  * 			1：发送成功，无错误
  * 			2：达到了最大重发次数仍未收到应答，可能是收发双方配置不一致、接收方不存在、接收FIFO已满或者多个发送数据包碰撞
  * 			3：状态寄存器的值不合法，可能是设备不存在、断路、短路或者引脚配置不正确
  * 			4：发送超时，可能是设备未初始化、断路、短路或者引脚配置不正确
  * 说    明：调用此函数前，直接修改全局数组NRF24L01_TxAddress和NRF24L01_TxPacket来设置发送的地址和数据
  */
 uint8_t NRF24L01_Send(void)
 {
   uint8_t Status;
   uint8_t SendFlag;
   uint16_t Timeout;
   /*发送地址，设置为全局数组NRF24L01_TxAddress指定的地址，地址宽度固定为5字节*/
   NRF24L01_WriteRegisters(NRF24L01_TX_ADDR,NRF24L01_TxAddress,5);
   /*接收通道0地址，此处必须也设置为发送地址，用于接收应答*/
   NRF24L01_WriteRegisters(NRF24L01_RX_ADDR_P0,NRF24L01_TxAddress,5);
   /*写发送有效载荷，写入全局数组NRF24L01_TxPacket指定的数据，数据宽度为NRF24L01_TX_PACKET_WIDTH*/
   NRF24L01_WriteTxPayload(NRF24L01_TxPacket,NRF24L01_TX_PACKET_WIDTH);
   /*发送的地址和有效载荷写入完成，进入发送模式，开始发送数据*/
   NRF24L01_TxModel();
   /*指定超时时间*/
   Timeout = 10000;
   /*循环读取状态寄存器*/
   while(1)
   {
       Status = NRF24L01_ReadRegister(NRF24L01_STATUS);
       Timeout--;
       if(Timeout == 0)
       {
         SendFlag = 4;      //发送超时，置标志位为4
         NRF24L01_Init();   //发送出错，重新初始化一次设备
         break;
       }
       if((Status & 0x30) == 0x30)//状态寄存器位4（MAX_RT）和位5（TX_DS）同时为1
       {
         //根据数据手册，当TX_DS为1的时候，说明已经收到了接受方的ACK应答，这时候不应该再进行重传，即MAX_RT在这种情况下不可能为1
         SendFlag = 3;      //状态寄存器的值不合法，置标志位为3
         NRF24L01_Init();   //发送出错，重新初始化一次设备
         break;
       }
       else if((Status & 0x10) == 0x10)
       {
         SendFlag = 2;      //达到了最大重发次数仍未收到应答，置标志位为2
         NRF24L01_Init();   //发送出错，重新初始化一次设备
         break;
       }
       else if((Status & 0x20) == 0x20)
       {
         SendFlag = 1;      //发送成功，置标志位为1
         break;
       }
   }
   /*给状态寄存器的位4（MAX_RT）和位5（TX_DS）写1，清标志位*/
      NRF24L01_WriteRegister(NRF24L01_STATUS,0x30); //清标志位
      NRF24L01_FlushTx(); //清空Tx FIFO的所有数据
      NRF24L01_WriteRegisters(NRF24L01_RX_ADDR_P0,NRF24L01_RxAddress,5);
      /*接收通道0地址，设置为全局数组NRF24L01_RxAddress指定的地址，地址宽度固定为5字节*/
      NRF24L01_RxModel(); //进入接收模式

      return SendFlag;
}
/**
  * 函    数：NRF24L01接收数据包
  * 参    数：无
  * 返 回 值：接收标志位，方便用户了解接收状态
  * 			0：未接收到数据包
  * 			1：成功接收到一个数据包
  * 			2：状态寄存器的值不合法，可能是设备不存在、断路、短路或者引脚配置不正确
  * 			3：设备仍处于掉电模式，可能是设备未初始化、曾经断电过、断路、短路或者引脚配置不正确
  * 说    明：如果收到了数据包，则可直接从全局数组NRF24L01_RxPacket取数据
  */
uint8_t NRF24L01_Receive(void)
{
   uint8_t Status;
   uint8_t Config;
   uint8_t ReceiveFlag;
   Status = NRF24L01_ReadStatus();
   Config = NRF24L01_ReadRegister(NRF24L01_CONFIG);
   if((Config & 0x02) == 0x00)     //配置寄存器位1（PWR_UP）为0
   {
      ReceiveFlag = 3;      //设备仍处于掉电模式，置标志位为3
      NRF24L01_Init();   //接收出错，重新初始化一次设备
   }
   else if((Status & 0x30) == 0x30)
   {
      ReceiveFlag = 2;      //状态寄存器的值不合法，置标志位为2
      NRF24L01_Init();   //接收出错，重新初始化一次设备
   }
   else if((Status & 0x40) == 0x40) //状态寄存器位6（RX_DR）为1,说明有数据到来
   {
      ReceiveFlag = 1;      //成功接收到一个数据包，置标志位为1
      NRF24L01_ReadRxPayload(NRF24L01_RxPacket,NRF24L01_RX_PACKET_WIDTH);
      NRF24L01_WriteRegister(NRF24L01_STATUS,0x40); //清标志位
      NRF24L01_FlushRx(); //清空Rx FIFO的所有数据
   }
   else
   {
      ReceiveFlag = 0;      //未接收到数据包，置标志位为0
   }
   return ReceiveFlag;
}