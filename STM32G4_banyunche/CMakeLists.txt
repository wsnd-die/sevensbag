cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Suppress "unused variable" warning when CMAKE_TOOLCHAIN_FILE is already cached
if(DEFINED CMAKE_TOOLCHAIN_FILE)
    message(STATUS "Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
endif()

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME STM32G4_TEST)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for C and ASM languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        hardware/Uart2_tbop10.c
        hardware/uart2_tbop10.h
        hardware/sw_uart.h
        hardware/sw_uart.c
        hardware/k230.c
        hardware/k230.h
        hardware/Trace_base.c
        hardware/Trace_base.h
        hardware/Circle_base.c
        hardware/Circle_base.h
        app/banyuntask.c
        hardware/ColorIdentif.c
        app/BollLocator.c
        app/BollLocator.h
        hardware/angle_ctrl.c
        hardware/angle_ctrl.c
        hardware/imu660ra_config.c
        algorithm/ahrs_mahony.c
        algorithm/ahrs_mahony.h
        algorithm/siyuan_imu.c
        algorithm/siyuan_imu.h
        hardware/collect_ir.c
        algorithm/Nav_position.c
        algorithm/Nav_position.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add user hardware sources to executable
# Note: ackermann.c and spi_imu660rc.c are alternative implementations,
# not used by the Keil project (which uses ackermann1.c and imu_660.c instead)
set(USER_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/app/Mecanum_Move.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/app/NavigationMecanum.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/hardware/block_basic.c"
    "Core/Src/can.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/color.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/emm_v5.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/hardware/HWT101_iic.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/imu_660.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/k230.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/Trace_base.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/hardware/oled.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/oled_data.c"
    "algorithm/mecanum.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/pid.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/angle_ctrl.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/Send_motor.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/hardware/QRcode.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/sw_uart.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/hardware/wit_protocol.c"
        "algorithm/ahrs_mahony.c"
        "algorithm/siyuan_imu.c"
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/app"
    # User hardware include path
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware"
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m
    # Add user defined libraries
)

# Generate .hex and .bin files from ELF (for flashing)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating hex and bin files"
)

# Flash target: 一键编译+烧录到 STM32G4
# 在 CLion 中切换到此 target 然后点 ▶ Run 即可烧录
add_custom_target(flash
    COMMAND E:/Tools/xpack-openocd/xpack-openocd-0.12.0-7/bin/openocd.exe
        -s E:/Tools/xpack-openocd/xpack-openocd-0.12.0-7/openocd/scripts
        -f interface/cmsis-dap.cfg
        -f target/stm32g4x.cfg
        -c "adapter speed 500; program $<TARGET_FILE:STM32G4_TEST> verify reset exit"
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "Flashing STM32G4 via DAPLink..."
)
