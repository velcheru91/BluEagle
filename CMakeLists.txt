cmake_minimum_required(VERSION 3.16)

set(CMAKE_VERBOSE_MAKEFILE ON)

#set(CMAKE_C_COMPILER arm-none-eabi-gcc.exe)
# Path to toolchain file. This one has to be before 'project()' below
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/arm-none-eabi-gcc.cmake)
set(CMAKE_C_COMPILER "C:/Program Files/Arm/GNU Arm Embedded Toolchain/bin/arm-none-eabi-gcc.exe")
set(CMAKE_CXX_COMPILER "C:/Program Files/Arm/GNU Arm Embedded Toolchain/bin/arm-none-eabi-g++.exe")
set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES "C:/Program Files/Arm/GNU Arm Embedded Toolchain/arm-none-eabi/include")
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
# Setup project, output, and linker files
project(BLUEAGLE)
set(EXECUTABLE ${PROJECT_NAME}.elf)
set(LINKER_FILE ${CMAKE_SOURCE_DIR}/app/${PROJECT_NAME}.ld)
ADD_DEFINITIONS(-Dgcc)
enable_language(C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(APP_SRC
    app/source/startup_gcc.c
    app/source/main.c
    app/source/BSP_LCD.c
    app/source/BSP_delay.c
    app/source/BSP.c
)

set(DRVLIB_SRC
    driverlib/interrupt.c
    driverlib/systick.c
    driverlib/cpu.c
    drivers/fpu.c
    drivers/scb.c)

set(FREE_RTOS_SRC
    FreeRTOS/Source/croutine.c
    FreeRTOS/Source/event_groups.c
    FreeRTOS/Source/list.c 
    FreeRTOS/Source/queue.c
    FreeRTOS/Source/stream_buffer.c 
    FreeRTOS/Source/tasks.c 
    FreeRTOS/Source/timers.c 
    FreeRTOS/Source/portable/GCC/tm4c123g/port.c 
    FreeRTOS/Source/portable/MemMang/heap_1.c )

set(APP_H_DIR app/inc/ driverlib/ driverlib/inc drivers/include FreeRTOS/Source/include FreeRTOS/Source/portable/GCC/tm4c123g)
set(APP_H_FILES
    app/inc/BSP.h
    app/inc/tm4c123gh6pm.h
    app/inc/FreeRTOSConfig.h
)

add_executable(${EXECUTABLE} ${APP_SRC} ${FREE_RTOS_SRC} ${DRVLIB_SRC})

# List of include directories
target_include_directories(${EXECUTABLE} PRIVATE
    ${APP_H_DIR}
)

# List of compiler defines, prefix with -D compiler option
target_compile_definitions(${EXECUTABLE} PRIVATE
    -DPART_TM4C123GH6PM
    -DTARGET_IS_TM4C123_RB1
)

target_link_libraries(${EXECUTABLE} PUBLIC
    c
    m
    rdimon
    gcc
)

# Compiler options
target_compile_options(${EXECUTABLE} PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    # Optimizations
    -ffunction-sections
    -fdata-sections
    -O0
    -g3
    # Compile warnings
    -Wall
    -Wextra
    -Wpedantic
)

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
    -T${LINKER_FILE}
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -specs=rdimon.specs
    -Wl,--entry=ResetISR
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
    -Wl,-print-memory-usage
)

# Print executable size as part of the post build process
add_custom_command(TARGET ${EXECUTABLE}
    POST_BUILD
    COMMAND ${CMAKE_SIZE_UTIL} ${EXECUTABLE}
)

# Create hex, bin and S-Record files after the build
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
set(S19_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.s19)
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${BIN_FILE}
    # COMMAND ${CMAKE_OBJCOPY} -O srec --srec-len=64 $<TARGET_FILE:${EXECUTABLE}> ${S19_FILE}
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${HEX_FILE}
    COMMENT "Building ${BIN_FILE}"
)
