cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(belt STATIC
    src/app.cpp
    src/vesc_can.cpp
    src/fdcan_driver.cpp
    src/can_driver.cpp
)

target_include_directories(belt PUBLIC
    include     
)
target_link_libraries(belt PRIVATE stm32cubemx)
target_link_libraries(belt PUBLIC gn10_can)
