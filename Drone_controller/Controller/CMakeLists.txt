# CMake file for STM32F103C8T6 with SPL
cmake_minimum_required(VERSION 3.15.3)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arm-none-eabi-gcc.cmake)

project(Drone_Controller)
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
    app/Application.c
    drivers/OLED/OLED.c
    drivers/OLED/OLED_Data.c
    drivers/Joystick/joystick.c
    drivers/NRF24L01/NRF24L01.c
    BSP/bsp_tim/timer.c
    BSP/bsp_button/Button.c
    BSP/bsp_ad/AD.c
    Firmware/CMSIS/device/startup_stm32f103x8.s
    Firmware/CMSIS/device/stm32f10x_it.c
    Firmware/CMSIS/device/system_stm32f10x.c
    Firmware/driver/src/misc.c
    Firmware/driver/src/stm32f10x_gpio.c
    Firmware/driver/src/stm32f10x_rcc.c
    Firmware/driver/src/stm32f10x_tim.c
    Firmware/driver/src/stm32f10x_i2c.c
    Firmware/driver/src/stm32f10x_adc.c
)

add_executable(${EXECUTABLE} ${SRC_FILES})

target_compile_definitions(${EXECUTABLE} PRIVATE
    STM32F10X_MD
    USE_STDPERIPH_DRIVER
)

target_include_directories(${EXECUTABLE} PRIVATE
    Firmware/CMSIS/core
    Firmware/CMSIS/device
    Firmware/driver/inc
    app
    drivers/OLED
    drivers/Joystick
    drivers/NRF24L01
    BSP/bsp_tim
    BSP/bsp_button
    BSP/bsp_ad
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

target_link_options(${EXECUTABLE} PRIVATE
    -T${LINKER_FILE}
    -mcpu=cortex-m3
    -mthumb
    # Enable floating-point formatting for OLED_Printf (%f).
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
