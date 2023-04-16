cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(rp_fly_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_subdirectory(deneg-lib)
include_directories(deneg-lib/drivers/ssd1306-driver/include)
include_directories(deneg-lib/math/position-calc/include)

add_executable(RPfly
  main.c
 )

pico_enable_stdio_usb(RPfly 1)
pico_enable_stdio_uart(RPfly 0)
pico_add_extra_outputs(RPfly)
target_link_libraries(RPfly pico_stdlib hardware_i2c ssd1306-driver position-calc)
