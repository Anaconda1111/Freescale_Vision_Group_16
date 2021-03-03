#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.17)

# specify cross compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(RT1064 C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork -specs=nosys.specs)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

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

include_directories("nxp_libraries/CMSIS/Include" "nxp_libraries/cmsis_drivers" "nxp_libraries/components/lists" "nxp_libraries/components/serial_manager"
        "nxp_libraries/components/uart" "nxp_libraries/deceive" "nxp_libraries/drives" "nxp_libraries/middleware/fatfs/source" "nxp_libraries/middleware/sdmmc/inc"
        "nxp_libraries/middleware/sdmmc/port" "nxp_libraries/usb/device" "nxp_libraries/usb/phy" "nxp_libraries/usb/osa" "nxp_libraries/usb/include"
        "nxp_libraries/utilities" "nxp_libraries/utilities/str" "nxp_libraries/utilities/debug_console" "nxp_libraries/xip" "seekfree_libraries" "seekfree_libraries/board"
        "seekfree_libraries/common" "seekfree_peripheral" "nxp_libraries/middleware/fatfs/source/fsl_sd_disk" "CODE" "USER")

add_definitions(-DCPU_MIMXRT1064DVL6A -DSKIP_SYSCLK_INIT -DXIP_EXTERNAL_FLASH=1 -DXIP_BOOT_HEADER_DCD_ENABLE=1
        -DXIP_BOOT_HEADER_ENABLE=1 -DPRINTF_FLOAT_ENABLE=1 -DSCANF_FLOAT_ENABLE=1 -DPRINTF_ADVANCED_ENABLE=1
        -DSCANF_ADVANCED_ENABLE=1 -DFSL_DRIVER_TRANSFER_DOUBLE_WEAK_IRQ=0 -DUSB_STACK_BM)

file(GLOB_RECURSE SOURCES "seekfree_libraries/board/*.c" "nxp_libraries/components/serial_manager/serial_manager.c" "seekfree_libraries/common/*.c"
        "nxp_libraries/components/serial_manager/serial_port_uart.c" "nxp_libraries/components/lists/generic_list.c" "nxp_libraries/components/uart/lpuart_adapter.c"
        "nxp_libraries/deceive/system_MIMXRT1064.c" "nxp_libraries/drives/*.c" "nxp_libraries/middleware/fatfs/source/*.c" "nxp_libraries/middleware/sdmmc/src/*.c"
        "nxp_libraries/middleware/sdmmc/port/usdhc/interrupt/*.c" "seekfree_libraries/*.c" "seekfree_peripheral/*.c" "startup_MIMXRT1064.S"
        "nxp_libraries/usb/osa/*.c" "nxp_libraries/usb/device/usb_device_cdc_acm.c" "nxp_libraries/usb/device/usb_device_ch9.c" "nxp_libraries/usb/device/usb_device_class.c"
        "nxp_libraries/usb/device/usb_device_dci.c" "nxp_libraries/usb/device/usb_device_descriptor.c" "nxp_libraries/usb/device/usb_device_ehci.c"
        "nxp_libraries/usb/phy/usb_phy.c" "nxp_libraries/utilities/*.c" "nxp_libraries/utilities/debug_console/*.c" "nxp_libraries/utilities/str/*.c"
        "nxp_libraries/xip/*.c" "nxp_libraries/middleware/fatfs/source/fsl_sd_disk/fsl_sd_disk.c" "USER/*.c" "CODE/*.c" "CODE/*.h")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/RT1064_flexspi_nor.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

target_link_libraries(${PROJECT_NAME}.elf m)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT " Building ${HEX_FILE}
        Building ${BIN_FILE} ")

