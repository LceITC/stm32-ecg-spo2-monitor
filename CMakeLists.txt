set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.22)

# Cross compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(Project1_v3 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(MCU_FLAGS -mcpu=cortex-m3 -mthumb -mthumb-interwork)

add_compile_options(
        ${MCU_FLAGS}
        -ffunction-sections
        -fdata-sections
        -fno-common
        -fmessage-length=0
)

add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

file(GLOB_RECURSE STD_PERIPH_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_SOURCE_DIR}/Drivers/STM32F10x_StdPeriph_Driver/src/*.c"
        "${CMAKE_SOURCE_DIR}/System/Src/*.c"
        "${CMAKE_SOURCE_DIR}/my_lib/Src/*.c"
        "${CMAKE_SOURCE_DIR}/Hardware/Src/*.c"
)

set(PROJECT_SOURCES
        ${STD_PERIPH_SOURCES}
        ${SYSTEM_SOURCES}
        ${HAREWARE_SOURCES}
        start/system_stm32f10x.c
        start/startup_stm32f10x_md.s
        user/main.c
        user/stm32f10x_it.c
        user/syscalls.c
        my_lib/Inc/delay.h
)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103C8TX_FLASH.ld)

add_executable(${PROJECT_NAME}.elf ${PROJECT_SOURCES})

target_include_directories(${PROJECT_NAME}.elf PRIVATE
        start
        user
        System/Inc
        my_lib/Inc
        Hardware/Inc
        Drivers/STM32F10x_StdPeriph_Driver/inc
)

target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
        DEBUG
        USE_STDPERIPH_DRIVER
        STM32F10X_MD
)

target_link_options(${PROJECT_NAME}.elf PRIVATE
        ${MCU_FLAGS}
        -T${LINKER_SCRIPT}
        -Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
