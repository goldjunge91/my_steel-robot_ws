cmake_minimum_required(VERSION 3.13)

# Change your executable name to something creative!
set(NAME my_firmware) # <-- Name your project/executable here!
set(PICO_SDK_PATH "/home/marco/pico-sdk" CACHE PATH "Path to the PICO SDK" FORCE)
include(pico_sdk_import.cmake)

# Gooey boilerplate
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

set (FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/../lib/FreeRTOS-Kernel" CACHE STRING "Common Lib" FORCE)

# SET(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/port/FreeRTOS-Kernel" CACHE STRING "Local Config")

# include_directories("${FREERTOS_CONFIG_FILE_DIRECTORY}") 
# include(port/FreeRTOS-Kernel/freertos_config.cmake)


# 1. Erstelle ein "INTERFACE"-Target nur für die Konfiguration.
add_library(freertos_config INTERFACE)

# 2. Füge das Verzeichnis mit der FreeRTOSConfig.h NUR zu diesem Target hinzu.
target_include_directories(freertos_config SYSTEM INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/port/FreeRTOS-Kernel"
)

# 3. Füge die notwendigen Compiler-Definitionen für das Target hinzu.
# target_compile_definitions(freertos_config PUBLIC
#     projCOVERAGE_TEST=0
# )
target_compile_definitions(freertos_config INTERFACE
    projCOVERAGE_TEST=0
    # configNUMBER_OF_CORES=2
    configNUM_CORES=2
)
# === ENDE DER FREERTOS-KORREKTUR ===
include(FreeRTOS_Kernel_import.cmake)

set(MICRO_ROS_PATH "${CMAKE_CURRENT_LIST_DIR}/../lib/micro_ros_raspberrypi_pico_sdk/" CACHE STRING "Common Lib")
include(micro_ros.cmake)

set(EIGEN_PATH "${CMAKE_CURRENT_LIST_DIR}/../lib/eigen/" CACHE STRING "Common Lib")
include(eigen_import.cmake)

set(DISTANCE_SENSOR_PATH "${CMAKE_CURRENT_LIST_DIR}/../lib/pico-distance-sensor/" CACHE STRING "Common Lib")
include(distance_sensor.cmake)

add_subdirectory(src)

#set up files for the release packages
install(CODE "execute_process(COMMAND $ENV{HOME}/bin/picoDeploy.sh ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.elf)")

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.uf2
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)
