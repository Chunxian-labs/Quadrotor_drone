#ifndef NRF24L01_H
#define NRF24L01_H
#define NRF24L01_W_CE_PORT GPIOB
#define NRF24L01_W_CE_PIN GPIO_Pin_7
#define NRF24L01_W_CSN_PORT GPIOB
#define NRF24L01_W_CSN_PIN GPIO_Pin_6
#define NRF24L01_W_SCK_PORT GPIOB
#define NRF24L01_W_SCK_PIN GPIO_Pin_5
#define NRF24L01_W_MISO_PORT GPIOB
#define NRF24L01_W_MISO_PIN GPIO_Pin_3
#define NRF24L01_W_MOSI_PORT GPIOB
#define NRF24L01_W_MOSI_PIN GPIO_Pin_4
#include "NRF24L01_Config.h"
#include <stdint.h>
extern uint8_t NRF24L01_TxPacket[];
extern uint8_t NRF24L01_RxPacket[];
void NRF24L01_Init(void);
uint8_t NRF24L01_Send(void);
uint8_t NRF24L01_Receive(void);
#endif
