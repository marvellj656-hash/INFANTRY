cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME OMNI_jpl)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# LinkScope queries all variable types. GCC -g3 macro debug data made this
# query take ~23 s; -g2 retains line/type/variable debugging without macros.
# Keep this in the user-owned root file, not CubeMX's regenerated toolchain.
add_compile_options("$<$<CONFIG:Debug>:-g2>" "$<$<CONFIG:Debug>:-gdwarf-4>")

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
# 自动收集这些目录及其子目录里的 .c 文件
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Task/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Bsp/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Algorithm/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Modules/*.c"
)

# 头文件也归入目标，CLion 新建 .h 后无需逐个点击“修正”。
file(GLOB_RECURSE USER_HEADERS CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Task/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/Bsp/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/Algorithm/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/Modules/*.h"
)
file(GLOB_RECURSE USER_INCLUDE_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/Algorithm/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/Bsp/*.h")
foreach(header IN LISTS USER_INCLUDE_FILES)
    get_filename_component(header_dir "${header}" DIRECTORY)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE "${header_dir}")
endforeach()

# CubeMX 管 USB PCD、引脚与 IRQ；Bsp/nuc 只接 USB Device/CDC 协议栈。
# HAL 的 pcd/pcd_ex/ll_usb 已由生成的 STM32_Drivers 编译，不能在这里重复。
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${USER_SOURCES}
        ${USER_HEADERS}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Task
    Modules
    Bsp
    Algorithm
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

    # Add user defined libraries
)

# 旧版四元数 EKF 只用六个矩阵函数，不拉入整套 DSP 源码。
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE Drivers/CMSIS/DSP/Include)
foreach(op init add sub mult trans inverse)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_${op}_f32.c)
endforeach()
