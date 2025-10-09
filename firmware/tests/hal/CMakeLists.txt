# HAL Mock Library for Tests

add_library(hal_interfaces INTERFACE)
target_include_directories(hal_interfaces INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_compile_features(hal_interfaces INTERFACE cxx_std_17)

add_library(hal_mocks STATIC
    mocks/I2CMock.cpp
    mocks/GPIOMock.cpp
)
target_include_directories(hal_mocks PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(hal_mocks PUBLIC hal_interfaces)
target_compile_features(hal_mocks PUBLIC cxx_std_17)
target_compile_options(hal_mocks PRIVATE -Wall -Wextra)

target_link_libraries(hal_mocks 
    PUBLIC 
        hal_interfaces
)

target_compile_features(hal_mocks PUBLIC cxx_std_17)

# Enable warnings for better code quality
target_compile_options(hal_mocks PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)