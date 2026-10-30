#ifndef __BUTTON_H__
#define __BUTTON_H__
#define BUTTON_NUM  12
#define BUTTON_PRESSED 1
#define BUTTON_RELEASED 0
#include <stdint.h>
typedef enum
{
    BTN_1 = 0,
    BTN_2 = 1,
    BTN_3 = 2,
    BTN_4 = 3,
    BTN_5 = 4,
    BTN_6 = 5,
    BTN_7 = 6,
    BTN_8 = 7,
    BTN_9 = 8,
    BTN_10 = 9,
    BTN_11 = 10,
    BTN_12 = 11
}Button_ID;
void BSP_Button_Init(void);
void BSP_Button_Scan1ms(void);
uint8_t BSP_Button_CheckState(Button_ID id, uint8_t flag);
#endif