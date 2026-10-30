#include "Button.h"
#include "stm32f10x.h"
#define BUTTON_DEBOUNCE_TIME 20 // // 消抖时间，单位：ms
typedef struct
{
    GPIO_TypeDef* GPIO_PORT;
    uint16_t      GPIO_PIN;
    uint8_t       GPIO_MODE;    // GPIO_Mode_IPU / GPIO_Mode_IPD
}Button_cfg;

typedef struct
{
    uint8_t initialized;   //是否初始化
    uint8_t candidate;     //候选状态
    uint8_t stable;        //稳定状态
    uint8_t count;         //消抖时间计数器
}Button_state;
typedef struct
{
    volatile uint8_t Button_pressed_state;
}Button_event; 
typedef struct
{
    Button_cfg cfg;
    Button_state state;
    Button_event event;
}Button_obj;
 
static const Button_cfg Button_cfg_table[BUTTON_NUM] =
{
    {GPIOA, GPIO_Pin_6, GPIO_Mode_IPU},
    {GPIOA, GPIO_Pin_7, GPIO_Mode_IPU},
    {GPIOB, GPIO_Pin_0, GPIO_Mode_IPU},
    {GPIOB, GPIO_Pin_1, GPIO_Mode_IPU},
    {GPIOB, GPIO_Pin_2, GPIO_Mode_IPU},
    {GPIOB, GPIO_Pin_10, GPIO_Mode_IPU},
    {GPIOA, GPIO_Pin_11, GPIO_Mode_IPU},
    {GPIOA, GPIO_Pin_12, GPIO_Mode_IPU},
    {GPIOB, GPIO_Pin_5, GPIO_Mode_IPU},
    {GPIOB, GPIO_Pin_6, GPIO_Mode_IPU},
    {GPIOA, GPIO_Pin_4, GPIO_Mode_IPU},
    {GPIOA, GPIO_Pin_5, GPIO_Mode_IPU},
};
static Button_obj Button[BUTTON_NUM];
void BSP_Button_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    for(uint8_t i=0;i<BUTTON_NUM;i++)
    {
        Button[i].cfg = Button_cfg_table[i];
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_StructInit(&GPIO_InitStruct);
        GPIO_InitStruct.GPIO_Mode = Button[i].cfg.GPIO_MODE;
        GPIO_InitStruct.GPIO_Pin = Button[i].cfg.GPIO_PIN;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(Button[i].cfg.GPIO_PORT, &GPIO_InitStruct);
        Button[i].state.initialized = 1;
    }
}
static void BSP_Button_ScanOnce(Button_obj *Button)
{
    uint8_t pressed;
    if(!Button->state.initialized)
    {
        return;
    }
    pressed = GPIO_ReadInputDataBit(Button->cfg.GPIO_PORT, Button->cfg.GPIO_PIN);
    if(Button->cfg.GPIO_MODE == GPIO_Mode_IPU)
    {
        pressed = !pressed;
    }
    if(pressed != Button->state.candidate)
    {
        Button->state.candidate = pressed;
        Button->state.count = 0;
        return;
    }
    if(Button->state.count < BUTTON_DEBOUNCE_TIME)
    {
        Button->state.count++;
        return;
    }
    if(Button->state.count == BUTTON_DEBOUNCE_TIME && Button->state.stable != pressed)
    {
        Button->state.stable = pressed;
        if(pressed)
        {
            Button->event.Button_pressed_state = 1;
        }
        else
        {
            Button->event.Button_pressed_state = 0;
        }
    }
}

void BSP_Button_Scan1ms(void)
{
    for(uint8_t i=0;i<BUTTON_NUM;i++)
    {
        BSP_Button_ScanOnce(&Button[i]);
    }
}

uint8_t BSP_Button_CheckState(Button_ID id, uint8_t flag)
{
    if(id >= BUTTON_NUM)
    {
        return 0;
    }
    if(Button[id].event.Button_pressed_state == flag)
    {
        return 1;
    }
    return 0;
}