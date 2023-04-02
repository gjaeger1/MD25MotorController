project(MD25MotorController C CXX)
add_library(MD25MotorController INTERFACE)
target_sources(MD25MotorController INTERFACE
        )
target_include_directories(MD25MotorController INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

list(APPEND DEPS foundation-logging)


if(NOT ADMORPH_PLATFORM STREQUAL "posix")
    # we have a RP2040 platform
    list(APPEND DEPS pico_stdlib pico_multicore hardware_i2c hardware_irq i2c_slave)

    if(PICO_BOARD STREQUAL "pico_w")
        # we have pico_w
        list(APPEND DEPS pico_cyw43_arch_none)
    endif()
endif()

# optionally disable logging
if(ADMORPH_DISABLE_LOGGING)
    message("Disabling logging!")
    target_compile_definitions(MD25MotorController PRIVATE FOUNDATION_LOGGING_DISABLED)
endif()

target_link_libraries(MD25MotorController INTERFACE ${DEPS})

