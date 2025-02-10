# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(nRF24L01P_PTX C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(nRF24L01P_PTX nRF24L01P_PTX.c nRF24L01P.c)

pico_set_program_name(nRF24L01P_PTX "nRF24L01_PTX")
pico_set_program_version(nRF24L01P_PTX "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(nRF24L01P_PTX 0)
pico_enable_stdio_usb(nRF24L01P_PTX 1)

# Include the nRF24L01 header files
add_library(nRF24L01P INTERFACE) 
target_include_directories(nRF24L01P INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

add_library(nRF24L01P_Data_And_Ctrl INTERFACE) 
target_include_directories(nRF24L01P_Data_And_Ctrl INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

add_library(nRF24L01P_Register_Map INTERFACE) 
target_include_directories(nRF24L01P_Register_Map INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

# Add the standard library to the build
target_link_libraries(nRF24L01P_PTX
        pico_stdlib
        hardware_spi
        nRF24L01P
        nRF24L01P_Data_And_Ctrl
        nRF24L01P_Register_Map)

# Add the standard include files to the build
target_include_directories(nRF24L01P_PTX PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(nRF24L01P_PTX)

