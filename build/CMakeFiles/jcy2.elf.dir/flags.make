# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

# compile ASM with C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-gcc.exe
# compile C with C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-gcc.exe
ASM_DEFINES = -DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -DMCU_LIB_SELECT=2 -D__FPU_PRESENT=1

ASM_INCLUDES = -ID:/YL_TECH_CODE/JCY/inc -ID:/YL_TECH_CODE/JCY/config -ID:/YL_TECH_CODE/JCY/lib/driver/n32/inc -ID:/YL_TECH_CODE/JCY/lib/mcu/n32/common -ID:/YL_TECH_CODE/JCY/lib/SEGGER_RTT/RTT -ID:/YL_TECH_CODE/JCY/lib/CMSIS/Include -ID:/YL_TECH_CODE/JCY/stc/atc-master -ID:/YL_TECH_CODE/JCY/freeRTOS/Source/include

ASM_FLAGS = -g -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mcpu=cortex-m4 -mthumb -mthumb-interwork -ffunction-sections -fdata-sections -fno-common -fmessage-length=0 -Og -g

C_DEFINES = -DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -DMCU_LIB_SELECT=2 -D__FPU_PRESENT=1

C_INCLUDES = -ID:/YL_TECH_CODE/JCY/inc -ID:/YL_TECH_CODE/JCY/config -ID:/YL_TECH_CODE/JCY/lib/driver/n32/inc -ID:/YL_TECH_CODE/JCY/lib/mcu/n32/common -ID:/YL_TECH_CODE/JCY/lib/SEGGER_RTT/RTT -ID:/YL_TECH_CODE/JCY/lib/CMSIS/Include -ID:/YL_TECH_CODE/JCY/stc/atc-master -ID:/YL_TECH_CODE/JCY/freeRTOS/Source/include

C_FLAGS = -g -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mcpu=cortex-m4 -mthumb -mthumb-interwork -ffunction-sections -fdata-sections -fno-common -fmessage-length=0 -Og -g -std=gnu11

