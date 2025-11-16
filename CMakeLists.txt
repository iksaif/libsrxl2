cmake_minimum_required(VERSION 3.10)
project(SRXL2_Experiments C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2")

# Global definitions
add_definitions(-DSRXL_NUM_OF_BUSES=1)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/SRXL2/Source
    ${CMAKE_CURRENT_SOURCE_DIR}/SRXL2/Examples
    ${CMAKE_CURRENT_SOURCE_DIR}/SRXL2_Master
    ${CMAKE_CURRENT_SOURCE_DIR}/fakeuart
    ${CMAKE_CURRENT_SOURCE_DIR}/programs
    ${CMAKE_CURRENT_SOURCE_DIR}/SpektrumDocumentation/Telemetry
)

###############################################################################
# Fake UART Library
###############################################################################

add_library(fakeuart STATIC
    fakeuart/fakeuart.c
    fakeuart/fakeuart.h
)

###############################################################################
# SRXL2 Official Library
###############################################################################

add_library(srxl2 STATIC
    SRXL2/Source/spm_srxl.c
    SRXL2/Source/spm_srxl.h
)

# Enable master code for this library
target_compile_definitions(srxl2 PRIVATE SRXL_INCLUDE_MASTER_CODE)

# Override the config include path to use our simulation config
# We do this by adding our programs directory first in the include path
target_include_directories(srxl2 BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/programs
)

###############################################################################
# SRXL2 Master Library
###############################################################################

add_library(srxl2_master STATIC
    SRXL2_Master/spm_srxl_master.c
    SRXL2_Master/spm_srxl_master.h
    SRXL2_Master/spm_srxl_master_config.h
)

# Enable master code for this library
target_compile_definitions(srxl2_master PRIVATE SRXL_INCLUDE_MASTER_CODE)

# Use our simulation config
target_include_directories(srxl2_master BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/programs
)

target_link_libraries(srxl2_master srxl2)

###############################################################################
# Simulation Programs
###############################################################################

# SRXL2 Master Simulator
add_executable(srxl2_master_sim
    programs/srxl2_master_sim.c
    SRXL2/Source/spm_srxl.c
    SRXL2_Master/spm_srxl_master.c
)

# Enable master code and internal packets for this program
target_compile_definitions(srxl2_master_sim PRIVATE
    SRXL_INCLUDE_MASTER_CODE
    SRXL_ENABLE_INTERNAL
)

target_include_directories(srxl2_master_sim BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/programs
)

target_link_libraries(srxl2_master_sim
    fakeuart
    m
)

# SRXL2 Battery Device Simulator
add_executable(srxl2_battery_sim
    programs/srxl2_battery_sim.c
    SRXL2/Source/spm_srxl.c
)

target_include_directories(srxl2_battery_sim BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/programs
)

target_link_libraries(srxl2_battery_sim
    fakeuart
    m
)

# SRXL2 Bus Sniffer
add_executable(srxl2_sniffer
    programs/srxl2_sniffer.c
)

# Enable internal packets for decoding
target_compile_definitions(srxl2_sniffer PRIVATE
    SRXL_ENABLE_INTERNAL
)

target_include_directories(srxl2_sniffer BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/programs
)

target_link_libraries(srxl2_sniffer
    fakeuart
)

###############################################################################
# SRXL2_Master Examples (optional)
###############################################################################

option(BUILD_MASTER_EXAMPLES "Build SRXL2 Master examples" OFF)

if(BUILD_MASTER_EXAMPLES)
    # Note: We would need to adapt the examples to not depend on SRXL2 library
    message(STATUS "Master examples disabled for simulation build")
endif()

###############################################################################
# Installation
###############################################################################

install(TARGETS
    srxl2_master_sim
    srxl2_battery_sim
    srxl2_sniffer
    DESTINATION bin
)

install(TARGETS
    fakeuart
    srxl2
    srxl2_master
    DESTINATION lib
)

install(FILES
    fakeuart/fakeuart.h
    DESTINATION include/fakeuart
)

install(FILES
    SRXL2/Source/spm_srxl.h
    DESTINATION include/srxl2
)

install(FILES
    SRXL2_Master/spm_srxl_master.h
    SRXL2_Master/spm_srxl_master_config.h
    DESTINATION include/srxl2_master
)

###############################################################################
# Print Configuration
###############################################################################

message(STATUS "")
message(STATUS "═══════════════════════════════════════════════")
message(STATUS "SRXL2 Experiments Configuration")
message(STATUS "═══════════════════════════════════════════════")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "  Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "")
message(STATUS "Targets:")
message(STATUS "  srxl2_master_sim  - SRXL2 Master Simulator")
message(STATUS "  srxl2_battery_sim - SRXL2 Battery Device Simulator")
message(STATUS "  srxl2_sniffer     - SRXL2 Bus Sniffer")
message(STATUS "═══════════════════════════════════════════════")
message(STATUS "")
