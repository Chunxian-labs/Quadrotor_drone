#ifndef __NRF24L01_H
#define __NRF24L01_H
#define NRF24L01_W_CE_PORT GPIOB
#define NRF24L01_W_CE_PIN GPIO_Pin_11
#define NRF24L01_W_CSN_PORT GPIOB
#define NRF24L01_W_CSN_PIN GPIO_Pin_12
#define NRF24L01_W_SCK_PORT GPIOB
#define NRF24L01_W_SCK_PIN GPIO_Pin_13
#define NRF24L01_W_MISO_PORT GPIOB
#define NRF24L01_W_MISO_PIN GPIO_Pin_14
#define NRF24L01_W_MOSI_PORT GPIOB
#define NRF24L01_W_MOSI_PIN GPIO_Pin_15
#include "NRF24L01_Config.h"
#include <stdint.h>
extern uint8_t NRF24L01_TxPacket[];
extern uint8_t NRF24L01_RxPacket[];
void NRF24L01_Init(void);
uint8_t NRF24L01_Send(void);
uint8_t NRF24L01_Receive(void);
#endif
