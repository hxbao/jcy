#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.21)

# specify cross compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
#project(tn-017-bms-software-stack C ASM)
project(jcy2 C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

#Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_definitions(__FPU_PRESENT=1)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)
if(MCU_SELECT_HC)
    add_compile_options(-mcpu=cortex-m0plus -mthumb -mthumb-interwork)
    add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
else(MCU_SELECT_HC==OFF)
    add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork )
    add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
endif()


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

if(MCU_SELECT_HC)


else(MCU_SELECT_HC==OFF)
    include_directories(
        inc/
        config/
        lib/driver/n32/inc
        lib/mcu/n32/common
        lib/SEGGER_RTT/RTT
        lib/CMSIS/Include
        stc/atc-master
        freeRTOS/Source/include

        )

    add_definitions(-DMCU_LIB_SELECT=2)

    file(GLOB_RECURSE SOURCES  "startup/startup_n32l406x.s"
        "lib/mcu/n32/common/*.c"
        "lib/SEGGER_RTT/RTT/*.c"
        "lib/driver/n32/src/*.c"
        "src/*.c"
        "freeRTOS/Source/*.c"
        )

endif()

#MCU 型号选择        
if(MCU_SELECT_HC)
    set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/hc32l17x.ld)
    set(LINKER_SCRIPT_BOOTLOADER ${CMAKE_SOURCE_DIR}/app/bootloader/hc32l17x.ld)
    add_link_options(-mcpu=cortex-m0plus -mthumb -mthumb-interwork)
else(MCU_SELECT_HC==OFF)
    set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/n32l406x.ld)
    set(LINKER_SCRIPT_BOOTLOADER ${CMAKE_SOURCE_DIR}/app/bootloader/nc32l406.ld)
    add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
endif()

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)

if(BOOT_SELECT)
    if(MCU_SELECT_HC)
    add_link_options(-T ${LINKER_SCRIPT_BOOTLOADER})
    add_executable(${PROJECT_NAME}.elf ${SOURCES} "app/bootloader/bootmain.c" ${LINKER_SCRIPT_BOOTLOADER})
    else(MCU_SELECT_HC==OFF)
    add_link_options(-T ${LINKER_SCRIPT_BOOTLOADER})
    add_executable(${PROJECT_NAME}.elf ${SOURCES} "app/bootloader/bootmain.c" ${LINKER_SCRIPT_BOOTLOADER})
    endif()  
else(BOOT_SELECT == OFF)
    add_link_options(-T ${LINKER_SCRIPT})
    add_executable(${PROJECT_NAME}.elf ${SOURCES} "app/main.c" ${LINKER_SCRIPT})
endif()


set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BOOT_HEX_FILE ${PROJECT_BINARY_DIR}/bootloader_hc.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)


if(BOOT_SELECT)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD        
            COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${BOOT_HEX_FILE}
            COMMENT "Building ${BOOT_HEX_FILE}"
)
else(BOOT_SELECT == OFF)
    add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD  
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
    COMMENT "Building ${HEX_FILE} 
    Building ${BIN_FILE}"
)
endif()


