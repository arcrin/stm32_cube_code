cmake_minimum_required(VERSION 3.15.0)

project(mcu1-led-toggle)

enable_language(C ASM)

set(CMAKE_C_STANDARD 99)

set(PROJECT_FILES
    drivers/Inc/stm32f407xx.h
    drivers/Inc/stm32f407xx_gpio_driver.h
    drivers/Inc/stm32f407xx_spi_driver.h
    drivers/Src/stm32f407xx_gpio_driver.c
    drivers/Src/stm32f407xx_spi_driver.c
    
    Src/002led_button.c

    stm32_startup.c
)

set(EXECUTABLE ${PROJECT_NAME}.elf)

add_executable(${EXECUTABLE} ${PROJECT_FILES})

target_include_directories(${EXECUTABLE} PRIVATE
    drivers/Inc
    drivers/Src
    Src
)

set(MACH cortex-m4)

target_compile_options(${EXECUTABLE} PRIVATE
    -mcpu=${MACH}
    -mthumb
    -mfloat-abi=soft
    -Wall
    $<$<CONFIG:Debug>:-Og>
)

target_link_options(${EXECUTABLE} PRIVATE
    -T ${CMAKE_SOURCE_DIR}/stm32_ls.ld
    -mcpu=${MACH}
    -mthumb
    -mfloat-abi=soft
    --specs=nosys.specs
    -Wl,-Map=${PROJECT_NAME}.map
)