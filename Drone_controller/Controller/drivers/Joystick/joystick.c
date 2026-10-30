#include "joystick.h"
#include "AD.h"
#include <stdint.h>
/*
ADValue (0 ~ 4095)
   ↓ 减去中点
Value (-2048 ~ 2047)
   ↓ 死区处理
Value (-1948 ~ 1947)
   ↓ 缩放
Value (-100 ~ 100)
*/
int8_t DataProcess(uint16_t ADValue)
{
    int16_t Value = ADValue - 2048; //范围 -2048 ~ 2047
    //死区处理
    //摇杆在中间 ±100 以内，直接当 0，避免中位抖动导致控制漂移
    if(Value > 100)
    {
        Value -= 100;
    }
    else if(Value < -100)
    {
        Value += 100;
    }
    else
    {
        Value = 0;
    }

    Value = Value * 101 / 1949;//这是整数运算，通过缩放使正负都尽量到±100
    return Value;
}
