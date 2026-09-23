#ifndef __COM_DEBUG_H__
#define __COM_DEBUG_H__
#include "stdarg.h"

#define DEBUG_LOG_ENABLE 1
#ifdef DEBUG_LOG_ENABLE
/* All the debug_printf shoule be off while the drone is flying 
Otherwise, it may interfere with the drone's operation,cause printf is time-consuming */
#define debug_printf(format,...) printf("[DEBUG] %s:%d: " format, __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define debug_printf(format,...)
#endif

#endif /* __COM_DEBUG_H__ */