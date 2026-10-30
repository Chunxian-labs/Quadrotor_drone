# CMake file for STM32F103C8T6 with SPL
cmake_minimum_required(VERSION 3.15.3)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arm-none-eabi-gcc.cmake)

project(Quadrotor_drone)
set(EXECUTABLE ${PROJECT_NAME}.elf)
set(LINKER_FILE ${CMAKE_SOURCE_DIR}/STM32F103C8TX_FLASH.ld)

enable_language(C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SRC_FILES
    app/main.c
    app/syscalls.c
    app/flight_app.c
    app/application.c
    BSP/bsp_i2c/bsp_i2c.c
    BSP/bsp_pwm/bsp_pwm.c
    BSP/bsp_timer/bsp_timer.c
    BSP/bsp_adc/bsp_ad.c
    drivers/NRF24L01/NRF24L01.c
    drivers/QMI8658/QMI8658.c

    drivers/SPA06/SPA06.c
    drivers/VL53L1/vl53l1_platform.c
    drivers/VL53L1/VL53L1X_api.c
    drivers/VL53L1/VL53L1X_calibration.c
    drivers/VL53L1/VL53L1X.c
    modules/attitude/attitude.c
    modules/sensors/imu_sensors.c
    modules/control/motor/motor.c
    modules/battery/battery_monitor.c
    modules/telemetry/telemetry.c
    modules/control/pid/pid.c
    modules/height/height_observation.c
    modules/height/height_estimator.c
    modules/control/rate_control/rate_control.c
    modules/control/angle_control/angle_control.c
    modules/control/vertical_control/vertical_control.c

    # FreeRTOS kernel (software timers and co-routines are disabled).
    FreeRTOS/source/tasks.c
    FreeRTOS/source/queue.c
    FreeRTOS/source/list.c
    FreeRTOS/source/event_groups.c
    FreeRTOS/source/stream_buffer.c
    # GCC Cortex-M3 port and a single heap implementation.
    FreeRTOS/portable/cotex_m3/port.c
    FreeRTOS/portable/memory/heap_4.c

    Firmware/CMSIS/device/startup_stm32f103x8.s
    Firmware/CMSIS/device/stm32f10x_it.c
    Firmware/CMSIS/device/system_stm32f10x.c
    Firmware/driver/src/misc.c
    Firmware/driver/src/stm32f10x_gpio.c
    Firmware/driver/src/stm32f10x_rcc.c
    Firmware/driver/src/stm32f10x_tim.c
    Firmware/driver/src/stm32f10x_adc.c
    Firmware/driver/src/stm32f10x_usart.c
)

add_executable(${EXECUTABLE} ${SRC_FILES})

target_compile_definitions(${EXECUTABLE} PRIVATE
    STM32F10X_MD
    USE_STDPERIPH_DRIVER
)

target_include_directories(${EXECUTABLE} PRIVATE
    FreeRTOS/include
    FreeRTOS/portable/cotex_m3
    Firmware/CMSIS/core
    Firmware/CMSIS/device
    Firmware/driver/inc
    app
    drivers
    modules/attitude
    modules/sensors
    modules/control/motor
    modules/control/pid
    modules/control/rate_control
    modules/control/angle_control
    modules/control/vertical_control
    modules/height
    modules/battery
    modules/telemetry
    BSP/bsp_i2c
    BSP/bsp_pwm
    BSP/bsp_timer
    BSP/bsp_adc
    drivers/NRF24L01
    drivers/QMI8658
    drivers/SPA06
    drivers/VL53L1
)

target_compile_options(${EXECUTABLE} PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -fdata-sections
    -ffunction-sections
    -Wall
    -O0
    -g3
)
target_link_libraries(${EXECUTABLE} PRIVATE m)
target_link_options(${EXECUTABLE} PRIVATE
    -T${LINKER_FILE}
    -mcpu=cortex-m3
    -mthumb
    -Wl,-u,_printf_float
    -specs=nano.specs
    -specs=nosys.specs
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE_UTIL} ${EXECUTABLE}
    COMMAND ${CMAKE_OBJCOPY} -O srec --srec-len=64 ${EXECUTABLE} ${PROJECT_NAME}.s19
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin
)
