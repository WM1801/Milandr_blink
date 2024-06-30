TARGET_MCU    := cortex-m3
OUTPUT_NAME   := MDR_project
FIRMWARE_NAME := mdr_firmware
LSCRIPT       := MDR32F9Qx.ld
VERSION       := _1_0

INCLUDE := \
../CMSIS/MDR32FxQI/CoreSupport/CM3 \
../CMSIS/MDR32FxQI/DeviceSupport/MDR32F9Q2I/inc/ \
../SPL/MDR32FxQI/ \
../CMSIS/MDR32FxQI/DeviceSupport/MDR32F9Q2I/startup/ 


SOURCE := \
../CMSIS/MDR32FxQI/DeviceSupport/MDR32F9Q2I/startup/arm/ \
../CMSIS/MDR32FxQI/DeviceSupport/MDR32F9Q2I/startup/ \
../src

GCC_DEF := \
-D__NO_SYSTEM_INIT -D__STARTUP_CLEAR_BSS \
-D__STACK_SIZE=4096 -D__START=main

OCD_INTERFACE_ := stlink-v2.cfg
OCD_INTERFACE := jlink.cfg
OCD_TARGET    := mdr32f9q2i.cfg

OCD_RESET_CMD := \
-c 'mww 0x4002001C 0x1010' \
-c 'mww 0x40060004 0x00' \
-c 'mww 0x40060000 0xFF' 