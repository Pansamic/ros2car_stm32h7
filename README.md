# ros2car_stm32h7

This is a project designed for small vehicle with high performance SLAM requirements.

![pcb_top_view](https://github.com/pansamic/ros2car_stm32h7/docs/pcb_top_view.png)

## feature

### peripherals

* Texas Instrument DRV8874 DC motor driver support.
* ICM-42688-P IMU support.
* WS18B20 serial RGB LED support.
* STM32 ethernet and LAN8720-A ethernet PHY support.
* STM32 USB2.0 FS support.
* TF card support.
* RX8900-CE:UA0 external RTC support.
* PWM servo driver x4 support.
* External 0.96/1.3inch 7P OLED support.
* External bluetooth module support.
* External SBUS receiver support.
* External GNSS module support.

### system performance

* DC motor position, velocity, torque control with 6A peak current.
* ROS2 IMU and control message publish and subscribe via 100Mps Ethernet.
* multi-sensor(IMU, Lidar and camera) sync - trigger signals aligned to GMT+0800 second PPS.
* Camera STROBE signal detection and ROS2 message publishing of timestamp of CMOS exposure.
* GNSS time synchronization within 1000 nanoseconds offset.
* Hardware PTPv2 time synchronization within 100 nanoseconds offset.
* Log system with FATFS storage and visualization support.
* USB Mass storage + DFU + VCP.

### pcb electronics

* 8 - 24V input; 5V 8A power supply for small computer; 5V 8A power supply for servos.
* ±30kV ESD per IEC 61000-4-2(Air) protection for external function pins.
* ±25kV ESD per IEC 61000-4-2(Contact) protection for external function pins.

## build

```
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
```

## user guide

## contribute

## license
