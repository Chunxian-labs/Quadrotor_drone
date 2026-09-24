#ifndef COM_DEBUG_H
#define COM_DEBUG_H
#include <stdio.h>

#define DEBUG_LOG_ENABLE 1
#ifdef DEBUG_LOG_ENABLE
#define debug_printf(format,...) printf("[DEBUG] %s:%d: " format, __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define debug_printf(format...)
#endif

#endif // COM_DEBUG_UART_H