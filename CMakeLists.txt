cmake_minimum_required(VERSION 3.12)

set(PICO_SDK_PATH "../../../pico-sdk")
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(pico_sdk_import.cmake)

project(lag_test_controller_ult CXX C)

pico_sdk_init()

add_executable(lag_test_controller_ult
    src/main.cpp
    src/usb/logic.cpp
    src/usb/protocol.cpp
    src/joybus.cpp
)

pico_generate_pio_header(lag_test_controller_ult ${CMAKE_CURRENT_LIST_DIR}/pio/my_pio.pio)

add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/generated/my_pio.pio.h
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/my_pio.pio
        COMMAND Pioasm ${CMAKE_CURRENT_LIST_DIR}/my_pio.pio ${CMAKE_CURRENT_LIST_DIR}/generated/my_pio.pio.h
        )
        
target_include_directories(lag_test_controller_ult PRIVATE include)

pico_set_linker_script(lag_test_controller_ult ../lag_test_controller_ult.ld)

target_link_libraries(lag_test_controller_ult hardware_adc hardware_pio pico_util pico_stdlib pico_multicore hardware_pwm hardware_resets hardware_irq)

pico_add_extra_outputs(lag_test_controller_ult)
