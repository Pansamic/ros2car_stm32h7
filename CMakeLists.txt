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
set(CMAKE_PROJECT_NAME ros2car_stm32h7)

set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings
# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -fdata-sections -ffunction-sections -Wno-unused-parameter")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS}")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32H743XX_FLASH.ld\"")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --specs=nano.specs")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--print-memory-usage")

set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lstdc++ -lsupc++ -Wl,--end-group")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here

    USB_DEVICE/Target/usbd_conf.c
    USB_DEVICE/App/usb_device.c
    USB_DEVICE/App/usbd_desc.c
    USB_DEVICE/App/usbd_cdc_if.c
    LWIP/Target/ethernetif.c
    LWIP/App/lwip.c
    FATFS/Target/bsp_driver_sd.c
    FATFS/Target/sd_diskio.c
    FATFS/App/fatfs.c
    Core/Src/main.c
    Core/Src/freertos.c
    Core/Src/stm32h7xx_it.c
    Core/Src/stm32h7xx_hal_msp.c
    Core/Src/stm32h7xx_hal_timebase_tim.c
    Core/Src/sysmem.c
    Core/Src/syscalls.c
    Core/Src/systime.c
    Core/Src/syslog.c
    startup_stm32h743xx.s

    Core/Src/system_stm32h7xx.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_exti.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_gpio.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_eth.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_eth_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_rcc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_utils.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_adc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_dma.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_crc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_i2c.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_lptim.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_pwr.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_rng.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_rtc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_sdmmc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_delayblock.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sd.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sd_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mmc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mmc_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_spi.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_tim.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usart.c
    Drivers/BSP/Components/lan8742/lan8742.c

    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    # Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc.c
    # Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_bot.c
    # Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_data.c
    # Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_scsi.c
    # Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Src/usbd_dfu_media_template.c
    # Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Src/usbd_dfu.c

    Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    Middlewares/Third_Party/FreeRTOS/Source/list.c
    Middlewares/Third_Party/FreeRTOS/Source/queue.c
    Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    Middlewares/Third_Party/FreeRTOS/Source/timers.c
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    Middlewares/Third_Party/FreeRTOS/Source/FreeRTOS_CLI.c

    Middlewares/Third_Party/LwIP/src/netif/ppp/auth.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/ccp.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/chap_ms.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/chap-md5.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/chap-new.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/demand.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/eap.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/eui64.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/fsm.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/ipcp.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/ipv6cp.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/lcp.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/magic.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/mppe.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/multilink.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/ppp.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/pppapi.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/pppcrypt.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/pppoe.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/pppol2tp.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/pppos.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/upap.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/utils.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/vj.c
    Middlewares/Third_Party/LwIP/src/netif/bridgeif.c
    Middlewares/Third_Party/LwIP/src/netif/bridgeif_fdb.c
    Middlewares/Third_Party/LwIP/src/netif/ethernet.c
    Middlewares/Third_Party/LwIP/src/netif/lowpan6.c
    Middlewares/Third_Party/LwIP/src/netif/lowpan6_ble.c
    Middlewares/Third_Party/LwIP/src/netif/lowpan6_common.c
    Middlewares/Third_Party/LwIP/src/netif/slipif.c
    Middlewares/Third_Party/LwIP/src/netif/zepif.c
    Middlewares/Third_Party/LwIP/src/netif/ppp/ecp.c
    Middlewares/Third_Party/LwIP/src/api/if_api.c
    Middlewares/Third_Party/LwIP/src/api/netbuf.c
    Middlewares/Third_Party/LwIP/src/api/api_msg.c
    Middlewares/Third_Party/LwIP/src/api/api_lib.c
    Middlewares/Third_Party/LwIP/src/api/netdb.c
    Middlewares/Third_Party/LwIP/src/api/err.c
    Middlewares/Third_Party/LwIP/src/api/tcpip.c
    Middlewares/Third_Party/LwIP/src/api/sockets.c
    Middlewares/Third_Party/LwIP/src/api/netifapi.c
    Middlewares/Third_Party/LwIP/src/core/netif.c
    Middlewares/Third_Party/LwIP/src/core/altcp_tcp.c
    Middlewares/Third_Party/LwIP/src/core/stats.c
    Middlewares/Third_Party/LwIP/src/core/def.c
    Middlewares/Third_Party/LwIP/src/core/tcp.c
    Middlewares/Third_Party/LwIP/src/core/pbuf.c
    Middlewares/Third_Party/LwIP/src/core/raw.c
    Middlewares/Third_Party/LwIP/src/core/tcp_out.c
    Middlewares/Third_Party/LwIP/src/core/inet_chksum.c
    Middlewares/Third_Party/LwIP/src/core/tcp_in.c
    Middlewares/Third_Party/LwIP/src/core/ip.c
    Middlewares/Third_Party/LwIP/src/core/altcp.c
    Middlewares/Third_Party/LwIP/src/core/init.c
    Middlewares/Third_Party/LwIP/src/core/udp.c
    Middlewares/Third_Party/LwIP/src/core/memp.c
    Middlewares/Third_Party/LwIP/src/core/altcp_alloc.c
    Middlewares/Third_Party/LwIP/src/core/mem.c
    Middlewares/Third_Party/LwIP/src/core/dns.c
    Middlewares/Third_Party/LwIP/src/core/timeouts.c
    Middlewares/Third_Party/LwIP/src/core/sys.c
    Middlewares/Third_Party/LwIP/src/core/ipv4/igmp.c
    Middlewares/Third_Party/LwIP/src/core/ipv4/ip4.c
    Middlewares/Third_Party/LwIP/src/core/ipv4/autoip.c
    Middlewares/Third_Party/LwIP/src/core/ipv4/ip4_frag.c
    Middlewares/Third_Party/LwIP/src/core/ipv4/etharp.c
    Middlewares/Third_Party/LwIP/src/core/ipv4/icmp.c
    Middlewares/Third_Party/LwIP/src/core/ipv4/ip4_addr.c
    Middlewares/Third_Party/LwIP/src/core/ipv4/dhcp.c
    Middlewares/Third_Party/LwIP/src/core/ipv6/dhcp6.c
    Middlewares/Third_Party/LwIP/src/core/ipv6/icmp6.c
    Middlewares/Third_Party/LwIP/src/core/ipv6/mld6.c
    Middlewares/Third_Party/LwIP/src/core/ipv6/inet6.c
    Middlewares/Third_Party/LwIP/src/core/ipv6/ip6_addr.c
    Middlewares/Third_Party/LwIP/src/core/ipv6/nd6.c
    Middlewares/Third_Party/LwIP/src/core/ipv6/ip6.c
    Middlewares/Third_Party/LwIP/src/core/ipv6/ethip6.c
    Middlewares/Third_Party/LwIP/src/core/ipv6/ip6_frag.c
    Middlewares/Third_Party/LwIP/system/OS/sys_arch.c
    Middlewares/Third_Party/LwIP/src/apps/mqtt/mqtt.c

    Middlewares/Third_Party/FatFs/src/diskio.c
    Middlewares/Third_Party/FatFs/src/ff.c
    Middlewares/Third_Party/FatFs/src/ff_gen_drv.c
    Middlewares/Third_Party/FatFs/src/option/syscall.c

    Drivers/BSP/Components/battery/battery.c
    Drivers/BSP/Components/drv8874/drv8874.c
    Drivers/BSP/Components/icm42688p/icm42688p.c
    Drivers/BSP/Components/oled/oled_data.c
    Drivers/BSP/Components/oled/oled.c
    Drivers/BSP/Components/rx8900ce/rx8900ce.c
    Drivers/BSP/Components/ws2812b/ws2812b.c

    Middlewares/Third_Party/cringbuf/src/cringbuf.c
    Middlewares/Third_Party/lwbtn/src/lwbtn.c
    Middlewares/Third_Party/lwprintf/src/lwprintf.c

    # Middlewares/Third_Party/tinyusb/src/class/cdc/cdc_device.c
    # Middlewares/Third_Party/tinyusb/src/class/dfu/dfu_device.c
    # Middlewares/Third_Party/tinyusb/src/class/msc/msc_device.c
    # Middlewares/Third_Party/tinyusb/src/common/tusb_fifo.c
    # Middlewares/Third_Party/tinyusb/src/device/usbd.c
    # Middlewares/Third_Party/tinyusb/src/device/usbd_control.c
    # Middlewares/Third_Party/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
    # Middlewares/Third_Party/tinyusb/src/typec/usbc.c
    # Middlewares/Third_Party/tinyusb/src/tusb.c

    Middlewares/Third_Party/Micro-CDR/src/c/types/array.c
    Middlewares/Third_Party/Micro-CDR/src/c/types/basic.c
    Middlewares/Third_Party/Micro-CDR/src/c/types/sequence.c
    Middlewares/Third_Party/Micro-CDR/src/c/types/string.c
    Middlewares/Third_Party/Micro-CDR/src/c/common.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/log/log.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/serialization/xrce_header.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/serialization/xrce_subheader.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/serialization/xrce_types.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/stream/input_best_effort_stream.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/stream/input_reliable_stream.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/stream/output_best_effort_stream.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/stream/output_reliable_stream.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/stream/seq_num.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/stream/stream_id.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/stream/stream_storage.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/common_create_entities.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/create_entities_bin.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/create_entities_xml.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/create_entities_ref.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/object_id.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/read_access.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/session_info.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/session.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/submessage.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/core/session/write_access.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/profile/matching/matching.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/profile/transport/ip/ip_lwip.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/profile/transport/ip/udp/udp_transport.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/profile/transport/ip/udp/udp_transport_lwip.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/profile/transport/stream_framing/stream_framing_protocol.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/util/ping.c
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/src/c/util/time.c

    Middlewares/Third_Party/uxrce-ros2-msgs/action_msgs/msg/GoalInfo.c
    Middlewares/Third_Party/uxrce-ros2-msgs/action_msgs/msg/GoalStatus.c
    Middlewares/Third_Party/uxrce-ros2-msgs/action_msgs/msg/GoalStatusArray.c
    Middlewares/Third_Party/uxrce-ros2-msgs/builtin_interfaces/msg/Duration.c
    Middlewares/Third_Party/uxrce-ros2-msgs/builtin_interfaces/msg/Time.c
    Middlewares/Third_Party/uxrce-ros2-msgs/control_msgs/msg/DynamicJointState.c
    Middlewares/Third_Party/uxrce-ros2-msgs/control_msgs/msg/GripperCommand.c
    Middlewares/Third_Party/uxrce-ros2-msgs/control_msgs/msg/InterfaceValue.c
    Middlewares/Third_Party/uxrce-ros2-msgs/control_msgs/msg/JointControllerState.c
    Middlewares/Third_Party/uxrce-ros2-msgs/control_msgs/msg/JointJog.c
    Middlewares/Third_Party/uxrce-ros2-msgs/control_msgs/msg/JointTolerance.c
    Middlewares/Third_Party/uxrce-ros2-msgs/control_msgs/msg/MecanumDriveControllerState.c
    Middlewares/Third_Party/uxrce-ros2-msgs/control_msgs/msg/PidState.c
    Middlewares/Third_Party/uxrce-ros2-msgs/control_msgs/msg/SteeringControllerStatus.c
    Middlewares/Third_Party/uxrce-ros2-msgs/diagnostic_msgs/msg/DiagnosticArray.c
    Middlewares/Third_Party/uxrce-ros2-msgs/diagnostic_msgs/msg/DiagnosticStatus.c
    Middlewares/Third_Party/uxrce-ros2-msgs/diagnostic_msgs/msg/KeyValue.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/Accel.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/AccelStamped.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/AccelWithCovariance.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/AccelWithCovarianceStamped.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/Inertia.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/InertiaStamped.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/Point.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/Point32.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/PointStamped.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/Polygon.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/PolygonStamped.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/Pose.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/Pose2D.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/PoseArray.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/PoseStamped.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/PoseWithCovariance.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/PoseWithCovarianceStamped.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/Quaternion.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/QuaternionStamped.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/Transform.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/TransformStamped.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/Twist.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/TwistStamped.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/TwistWithCovariance.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/TwistWithCovarianceStamped.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/Vector3.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/Vector3Stamped.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/Wrench.c
    Middlewares/Third_Party/uxrce-ros2-msgs/geometry_msgs/msg/WrenchStamped.c
    Middlewares/Third_Party/uxrce-ros2-msgs/lifecycle_msgs/msg/State.c
    Middlewares/Third_Party/uxrce-ros2-msgs/lifecycle_msgs/msg/Transition.c
    Middlewares/Third_Party/uxrce-ros2-msgs/lifecycle_msgs/msg/TransitionDescription.c
    Middlewares/Third_Party/uxrce-ros2-msgs/lifecycle_msgs/msg/TransitionEvent.c
    Middlewares/Third_Party/uxrce-ros2-msgs/map_msgs/msg/OccupancyGridUpdate.c
    Middlewares/Third_Party/uxrce-ros2-msgs/map_msgs/msg/ProjectedMapInfo.c
    Middlewares/Third_Party/uxrce-ros2-msgs/nav_msgs/msg/GridCells.c
    Middlewares/Third_Party/uxrce-ros2-msgs/nav_msgs/msg/MapMetaData.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/BatteryState.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/CameraInfo.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/ChannelFloat32.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/CompressedImage.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/FluidPressure.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/Illuminance.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/Image.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/Imu.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/JointState.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/Joy.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/JoyFeedback.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/JoyFeedbackArray.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/LaserEcho.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/LaserScan.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/MagneticField.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/MultiEchoLaserScan.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/NavSatFix.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/NavSatStatus.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/PointCloud.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/PointCloud2.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/PointField.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/Range.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/RegionOfInterest.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/RelativeHumidity.c
    Middlewares/Third_Party/uxrce-ros2-msgs/sensor_msgs/msg/Temperature.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Bool.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Byte.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/ByteMultiArray.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Char.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/ColorRGBA.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Empty.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Float32.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Float32MultiArray.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Float64.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Float64MultiArray.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Header.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Int8.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Int8MultiArray.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Int16.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Int16MultiArray.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Int32.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Int32MultiArray.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Int64.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/Int64MultiArray.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/MultiArrayDimension.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/MultiArrayLayout.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/String.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/UInt8.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/UInt8MultiArray.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/UInt16.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/UInt16MultiArray.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/UInt32.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/UInt32MultiArray.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/UInt64.c
    Middlewares/Third_Party/uxrce-ros2-msgs/std_msgs/msg/UInt64MultiArray.c

    Middlewares/Third_Party/ptpd/src/ptpd_arith.c
    Middlewares/Third_Party/ptpd/src/ptpd_bmc.c
    Middlewares/Third_Party/ptpd/src/ptpd_main.c
    Middlewares/Third_Party/ptpd/src/ptpd_msg.c
    Middlewares/Third_Party/ptpd/src/ptpd_net.c
    Middlewares/Third_Party/ptpd/src/ptpd_port.c
    Middlewares/Third_Party/ptpd/src/ptpd_protocol.c
    Middlewares/Third_Party/ptpd/src/ptpd_servo.c
    Middlewares/Third_Party/ptpd/src/ptpd_time.c
    Middlewares/Third_Party/ptpd/src/ptpd_timer.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/Inc
    FATFS/Target
    FATFS/App
    LWIP/App
    LWIP/Target
    USB_DEVICE/App
    USB_DEVICE/Target

    Drivers/STM32H7xx_HAL_Driver/Inc
    Drivers/STM32H7xx_HAL_Driver/Inc/Legacy

    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32H7xx/Include

    Drivers/BSP/Components/ws2812b
    Drivers/BSP/Components/rx8900ce
    Drivers/BSP/Components/oled
    Drivers/BSP/Components/lan8742
    Drivers/BSP/Components/icm42688p
    Drivers/BSP/Components/drv8874
    Drivers/BSP/Components/battery

    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    Middlewares/Third_Party/FatFs/src

    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc

    Middlewares/Third_Party/LwIP/src/include
    Middlewares/Third_Party/LwIP/system
    Middlewares/Third_Party/LwIP/src/include/netif/ppp
    Middlewares/Third_Party/LwIP/src/include/lwip
    Middlewares/Third_Party/LwIP/src/include/lwip/apps
    Middlewares/Third_Party/LwIP/src/include/lwip/priv
    Middlewares/Third_Party/LwIP/src/include/lwip/prot
    Middlewares/Third_Party/LwIP/src/include/netif
    Middlewares/Third_Party/LwIP/src/include/compat/posix
    Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa
    Middlewares/Third_Party/LwIP/src/include/compat/posix/net
    Middlewares/Third_Party/LwIP/src/include/compat/posix/sys
    Middlewares/Third_Party/LwIP/src/include/compat/stdc
    Middlewares/Third_Party/LwIP/system/arch

    Middlewares/Third_Party/cringbuf/include
    Middlewares/Third_Party/lwprintf/include
    Middlewares/Third_Party/lwbtn/include
    Middlewares/Third_Party/trice/include
    Middlewares/Third_Party/uxrce-ros2-msgs
    Middlewares/Third_Party/Micro-CDR/include
    Middlewares/Third_Party/Micro-XRCE-DDS-Client/include
    Middlewares/Third_Party/ptpd/include
    Middlewares/Third_Party/ulog/include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    USE_FULL_LL_DRIVER 
	USE_PWR_LDO_SUPPLY 
	USE_HAL_DRIVER 
	STM32H743xx
    $<$<CONFIG:Debug>:DEBUG>
    LWIP_PTPD
    HAL_ETH_USE_PTP
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    # Add user defined libraries
)
