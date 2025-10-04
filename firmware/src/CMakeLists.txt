add_executable(${NAME}
        main.cpp
        
        GPIOInputMgr.cpp
        GPIOObserver.cpp
        
        MotorMgr.cpp
        MotorPID.cpp
        
        PubEntities.cpp
        MotorsAgent.cpp
        DDD.cpp
        HCSR04Agent.cpp
        
        # Application Layer
        Agent.cpp
        BlinkAgent.cpp
        application/ImuAgent.cpp
        application/vl6180xAgent.cpp

        # Hardware Abstraction Layer
        hal/hardware/Icm20948Simple.cpp
        hal/hardware/vl6180x/vl6180x_driver.cpp

        # Micro-ROS
        uRosBridge.cpp
        uRosEntities.cpp
        pico_usb_transport.c
        freertos_allocators.c
)

target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} PRIVATE
    pico_stdlib
    hardware_pwm
    hardware_i2c
    hardware_spi
    FreeRTOS-Kernel-Heap4
    freertos_config
    micro_ros
    pico_rand
    eigen
    distance_sensor
)

        # Find Eigen3 (system) so <Eigen/Core> resolves when building firmware
        find_package(Eigen3 QUIET)
        if(TARGET Eigen3::Eigen)
            # Preferred modern package provides an imported target
            target_link_libraries(${NAME} PRIVATE Eigen3::Eigen)
        elseif(Eigen3_FOUND)
            # Older FindEigen3 may provide include dirs instead of a target
            if(DEFINED Eigen3_INCLUDE_DIRS)
                target_include_directories(${NAME} PRIVATE ${Eigen3_INCLUDE_DIRS})
            elseif(DEFINED EIGEN3_INCLUDE_DIR)
                target_include_directories(${NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})
            else()
                # As a fallback if variables aren't set, try common include variable
                if(DEFINED Eigen3_DIR)
                    target_include_directories(${NAME} PRIVATE ${Eigen3_DIR})
                endif()
            endif()
        else()
            message(WARNING "Eigen3 not found by CMake find_package; creating imported target Eigen3::Eigen using workspace lib/eigen or /usr/include/eigen3 fallback")
            # Prefer workspace lib/eigen if present
            if(EXISTS "${CMAKE_SOURCE_DIR}/../lib/eigen/Eigen/src/Core/util/Macros.h")
                # Create an IMPORTED INTERFACE target to satisfy consumers linking to Eigen3::Eigen
                add_library(Eigen3::Eigen INTERFACE IMPORTED)
                target_include_directories(Eigen3::Eigen INTERFACE ${CMAKE_SOURCE_DIR}/../lib/eigen)
                target_link_libraries(${NAME} PRIVATE Eigen3::Eigen)
            else()
                message(WARNING "Workspace lib/eigen not found; falling back to /usr/include/eigen3")
                target_include_directories(${NAME} PRIVATE /usr/include/eigen3)
            endif()
        endif()

# Ensure firmware executable sees FreeRTOS headers and config defines

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 1)
add_compile_definitions(PICO_USB_ENABLE_CRLF_SUPPORT=1)
#add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
#add_compile_definitions(PICO_STDIO_
