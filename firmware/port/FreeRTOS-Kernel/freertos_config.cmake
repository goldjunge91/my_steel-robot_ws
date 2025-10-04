cmake_minimum_required(VERSION 3.15)
project(freertos_port C CXX)

# TODO: After successfull build can be removed
# add_library(freertos_config STATIC)
# # Sources for the freertos_config static library (implementation)
# target_sources(freertos_config PUBLIC
#     ${CMAKE_CURRENT_LIST_DIR}/IdleMemory.c
# )

# 1) Von FreeRTOS-Kernel erwartetes Target
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}            # enthält FreeRTOSConfig.h
)
target_compile_definitions(freertos_config INTERFACE
    projCOVERAGE_TEST=0
    # configNUMBER_OF_CORES=2
    configNUM_CORES=2
)

# 2) Eigene Port-Objekte separat bauen
add_library(freertos_port_objs STATIC
    ${CMAKE_CURRENT_LIST_DIR}/IdleMemory.c
    ${CMAKE_CURRENT_LIST_DIR}/cppMemory.cpp
)
target_link_libraries(freertos_port_objs
    PUBLIC freertos_config FreeRTOS-Kernel pico_stdlib
)
# Create an INTERFACE target to export include directories and compile definitions
# to any target that links against the freertos configuration. This ensures
# header paths and defines propagate to Pico SDK and other build units.
add_library(freertos_config_headers INTERFACE)

target_include_directories(freertos_config_headers INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/lib
    ${CMAKE_CURRENT_LIST_DIR}/../../../lib/FreeRTOS-Kernel/include
    ${CMAKE_CURRENT_LIST_DIR}/../../../lib/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/include
    # rp2_common provides pico/platform/compiler.h
    ${PICO_SDK_PATH}/src/rp2_common/pico_platform_compiler/include
    $ENV{HOME}/pico-sdk/src/rp2_common/pico_platform_compiler/include
    # hardware headers for platform_defs.h and registers
    ${PICO_SDK_PATH}/src/rp2040/hardware_regs/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_clocks/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_xosc/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_uart/include
    # generated headers produced by pico_sdk_init (pico/version.h etc.)
    ${CMAKE_BINARY_DIR}/generated/pico_base
    ${PROJECT_BINARY_DIR}/generated/pico_base
    # Also accept sibling build locations (when this CMake is configured in a subdir)
    ${CMAKE_BINARY_DIR}/../firmware/generated/pico_base
    ${CMAKE_BINARY_DIR}/../generated/pico_base
    $ENV{HOME}/pico-sdk/src
    $ENV{HOME}/pico-sdk/include
    $ENV{HOME}/pico-sdk/src/common/pico_base_headers/include
    $ENV{HOME}/pico-sdk/src/boards/include
    $ENV{HOME}/pico-sdk/src/rp2040/pico_platform/include
    $ENV{HOME}/pico-sdk/src/common/pico_stdlib_headers/include
    ${PICO_SDK_PATH}/src
    ${PICO_SDK_PATH}/include
    ${PICO_SDK_PATH}/src/rp2040/pico_platform/include
    ${PICO_SDK_PATH}/src/common/pico_base_headers/include
    ${PICO_SDK_PATH}/src/boards/include
    ${PROJECT_BINARY_DIR}
)

target_compile_definitions(freertos_config_headers INTERFACE
    projCOVERAGE_TEST=0
    configNUMBER_OF_CORES=2
)

# Link implementation target against libraries required to build the sources.
# Link the INTERFACE headers target publicly so dependents that link
# against freertos_config also get the headers/defines.
target_link_libraries(freertos_config
    PUBLIC
        freertos_config_headers

    PRIVATE
        pico_stdlib
        FreeRTOS-Kernel
)

# Ensure the implementation target itself can compile its sources: add the
# same include directories and compile definitions privately so the
# implementation object files find pico and rp2040 headers during build.
target_include_directories(freertos_config PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../../../lib/FreeRTOS-Kernel/include
    ${CMAKE_CURRENT_LIST_DIR}/../../../lib/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/include
    # rp2_common provides pico/platform/compiler.h used by pico/platform.h
    ${PICO_SDK_PATH}/src/rp2_common/pico_platform_compiler/include
    $ENV{HOME}/pico-sdk/src/rp2_common/pico_platform_compiler/include
    # hardware headers for platform_defs.h and registers
    ${PICO_SDK_PATH}/src/rp2040/hardware_regs/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_clocks/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_xosc/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_uart/include
    # generated headers produced by pico_sdk_init (pico/version.h etc.)
    ${CMAKE_BINARY_DIR}/generated/pico_base
    ${PROJECT_BINARY_DIR}/generated/pico_base
    # Also accept sibling build locations (when this CMake is configured in a subdir)
    ${CMAKE_BINARY_DIR}/../firmware/generated/pico_base
    ${CMAKE_BINARY_DIR}/../generated/pico_base
    $ENV{HOME}/pico-sdk/src
    $ENV{HOME}/pico-sdk/include
    $ENV{HOME}/pico-sdk/src/common/pico_base_headers/include
    $ENV{HOME}/pico-sdk/src/boards/include
    $ENV{HOME}/pico-sdk/src/rp2040/pico_platform/include
    $ENV{HOME}/pico-sdk/src/common/pico_stdlib_headers/include
    ${PICO_SDK_PATH}/src
    ${PICO_SDK_PATH}/include
    ${PICO_SDK_PATH}/src/rp2040/pico_platform/include
    ${PICO_SDK_PATH}/src/common/pico_base_headers/include
    ${PICO_SDK_PATH}/src/boards/include
    ${PROJECT_BINARY_DIR}
)

target_compile_definitions(freertos_config PRIVATE
    projCOVERAGE_TEST=0
    configNUMBER_OF_CORES=2
)

# Also make it convenient: link freertos_config_headers to freertos_config so
# consumers can link either and obtain includes/defines.
target_link_libraries(freertos_config_headers INTERFACE pico_stdlib)