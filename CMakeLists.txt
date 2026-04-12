cmake_minimum_required(VERSION 3.20)

# ── Toolchain setup (must come before project()) ─────────────────────
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY      arm-none-eabi-objcopy)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(BalancingController C CXX ASM)

# ── Nanopb ────────────────────────────────────────────────────────────
set(NANOPB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../nanopb")

set(NANOPB_CORE
    ${NANOPB_DIR}/pb_encode.c
    ${NANOPB_DIR}/pb_decode.c
    ${NANOPB_DIR}/pb_common.c
)

# Detect nanopb protoc binary vs python generator
if(EXISTS "${NANOPB_DIR}/generator-bin/protoc")
    set(PROTOC "${NANOPB_DIR}/generator-bin/protoc")
else()
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(WIN32)
        set(PROTOC python "${NANOPB_DIR}/generator/protoc")
    else()
        set(PROTOC "${NANOPB_DIR}/generator/protoc")
    endif()
endif()

# ── Protobuf code generation ─────────────────────────────────────────
set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/drv/comms)

add_custom_command(
    OUTPUT  ${PROTO_DIR}/protocol.pb.c ${PROTO_DIR}/protocol.pb.h
    COMMAND ${PROTOC} --nanopb_out=${PROTO_DIR} ${PROTO_DIR}/protocol.proto
    DEPENDS ${PROTO_DIR}/protocol.proto
    COMMENT "Generating nanopb sources for protocol.proto"
)

add_custom_command(
    OUTPUT  ${PROTO_DIR}/config.pb.c ${PROTO_DIR}/config.pb.h
    COMMAND ${PROTOC} --nanopb_out=${PROTO_DIR} ${PROTO_DIR}/config.proto
    DEPENDS ${PROTO_DIR}/config.proto
    COMMENT "Generating nanopb sources for config.proto"
)

# ── Descriptor binary (deflated proto descriptor embedded in .elf) ───
set(DESCRIPTOR_BIN  ${CMAKE_BINARY_DIR}/descriptor.pb.bin)
set(DESCRIPTOR_ZLIB ${CMAKE_BINARY_DIR}/descriptor.pb.bin.deflate)
set(DESCRIPTOR_OBJ  ${CMAKE_BINARY_DIR}/descriptor.pb.deflate.o)

add_custom_command(
    OUTPUT  ${DESCRIPTOR_BIN}
    COMMAND ${PROTOC} ${PROTO_DIR}/config.proto --descriptor_set_out=${DESCRIPTOR_BIN}
    DEPENDS ${PROTO_DIR}/config.proto
    COMMENT "Generating protobuf descriptor set"
)

add_custom_command(
    OUTPUT  ${DESCRIPTOR_ZLIB}
    COMMAND python3 -c
            "import zlib,sys; sys.stdout.buffer.write(zlib.compress(sys.stdin.buffer.read(),9))"
            < ${DESCRIPTOR_BIN} > ${DESCRIPTOR_ZLIB}
    DEPENDS ${DESCRIPTOR_BIN}
    COMMENT "Deflating protobuf descriptor"
)

add_custom_command(
    OUTPUT  ${DESCRIPTOR_OBJ}
    COMMAND ${CMAKE_OBJCOPY}
            -I binary -B arm -O elf32-littlearm
            ${DESCRIPTOR_ZLIB} ${DESCRIPTOR_OBJ}
            --rename-section=.data=.rodata
    DEPENDS ${DESCRIPTOR_ZLIB}
    COMMENT "Converting deflated descriptor to object"
)

# ── Source files ──────────────────────────────────────────────────────
file(GLOB APP_CPP       "*.cpp")
file(GLOB IO_CPP        "io/*.cpp")
file(GLOB IMU_CPP       "imu/*.cpp")
file(GLOB GUARDS_CPP    "guards/*.cpp")
file(GLOB DRV_VESC_CPP  "drv/vesc/*.cpp")
file(GLOB DRV_COMMS_CPP "drv/comms/*.cpp")
file(GLOB DRV_SETTINGS_CPP "drv/settings/*.cpp")
file(GLOB DRV_MPU_CPP   "drv/mpu6050/*.cpp")

file(GLOB STM_LIB_C     "stm_lib/src/*.c")
file(GLOB SYSCALLS_C    "syscalls/*.c")
file(GLOB CMSIS_BOOT_C  "cmsis_boot/*.c")
file(GLOB CMSIS_BOOT_STARTUP_C "cmsis_boot/*/*.c")

set(SRCS
    ${APP_CPP}
    ${IO_CPP}
    ${IMU_CPP}
    ${GUARDS_CPP}
    ${DRV_VESC_CPP}
    ${DRV_COMMS_CPP}
    ${DRV_SETTINGS_CPP}
    ${DRV_MPU_CPP}
    ${STM_LIB_C}
    ${SYSCALLS_C}
    ${CMSIS_BOOT_C}
    ${CMSIS_BOOT_STARTUP_C}
    ${NANOPB_CORE}
    drv/comms/protocol.pb.c
    drv/comms/config.pb.c
)

# ── Include directories ──────────────────────────────────────────────
set(INC_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/drv
    ${CMAKE_CURRENT_SOURCE_DIR}/drv/vesc
    ${CMAKE_CURRENT_SOURCE_DIR}/drv/comms
    ${CMAKE_CURRENT_SOURCE_DIR}/cmsis_boot
    ${CMAKE_CURRENT_SOURCE_DIR}/cmsis
    ${CMAKE_CURRENT_SOURCE_DIR}/stm_lib/inc
    ${NANOPB_DIR}
)

# ── Compile definitions ──────────────────────────────────────────────
set(DEFS
    STM32F103CB
    STM32F10X_MD
    USE_STDPERIPH_DRIVER
    __ASSEMBLY__
    SUPPORT_CPLUSPLUS
)

# ── Architecture / compile flags ─────────────────────────────────────
set(ARCH_FLAGS "-mcpu=cortex-m3 -mthumb")

set(COMMON_FLAGS "${ARCH_FLAGS} -Wall -ffunction-sections -g -O2 -flto -fno-builtin")

set(CMAKE_C_FLAGS   "${COMMON_FLAGS}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=gnu++11" CACHE STRING "" FORCE)

# ── ELF target ────────────────────────────────────────────────────────
add_executable(BalancingController.elf ${SRCS})

target_include_directories(BalancingController.elf PRIVATE ${INC_DIRS})
target_compile_definitions(BalancingController.elf PRIVATE ${DEFS})

set_target_properties(BalancingController.elf PROPERTIES
    LINK_FLAGS "${ARCH_FLAGS} -g -flto -O2 \
-Wl,-Map=${CMAKE_BINARY_DIR}/BalancingController.map \
-Wl,--gc-sections \
-Wl,--entry=main \
-Wl,-T${CMAKE_CURRENT_SOURCE_DIR}/link.ld"
    LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/link.ld"
)

target_link_libraries(BalancingController.elf PRIVATE
    ${DESCRIPTOR_OBJ}
    m gcc c stdc++
)

# ── .bin and .hex targets ─────────────────────────────────────────────
add_custom_command(TARGET BalancingController.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
        $<TARGET_FILE:BalancingController.elf>
        ${CMAKE_BINARY_DIR}/BalancingController.bin
    COMMENT "Generating BalancingController.bin"
)

add_custom_command(TARGET BalancingController.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
        $<TARGET_FILE:BalancingController.elf>
        ${CMAKE_BINARY_DIR}/BalancingController.hex
    COMMENT "Generating BalancingController.hex"
)

# ── Flash targets ─────────────────────────────────────────────────────
add_custom_target(program
    COMMAND "C:/CooCox/CoIDE/bin/coflash.exe" program STM32F103CB
        $<TARGET_FILE:BalancingController.elf>
        --adapter-name=ST-Link --port=SWD --adapter-clk=2000000
        --erase=affected --reset=SYSRESETREQ
        "--driver=C:/CooCox/CoIDE/flash/STM32F10x_MD_128.elf"
        --verify=false
    DEPENDS BalancingController.elf
    COMMENT "Flashing via CoFlash"
)

add_custom_target(program_openocd
    COMMAND openocd
        -f interface/stlink.cfg -f target/stm32f1x.cfg
        -c "init\;targets\;halt\;flash write_image erase $<TARGET_FILE:BalancingController.elf>\;verify_image $<TARGET_FILE:BalancingController.elf>\;reset\;shutdown"
    DEPENDS BalancingController.elf
    COMMENT "Flashing via OpenOCD"
)
