cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(belt STATIC
    src/belt.cpp
    src/vesc_can.cpp
    src/fdcan_driver.cpp
    ${CMAKE_SOURCE_DIR}/gn10-can/drivers/stm32_fdcan/driver_stm32_fdcan.cpp

    
)

target_include_directories(belt PUBLIC
    include
    ${CMAKE_SOURCE_DIR}/gn10-can        
)
target_link_libraries(belt PRIVATE stm32cubemx)
target_link_libraries(belt PUBLIC gn10_can)
