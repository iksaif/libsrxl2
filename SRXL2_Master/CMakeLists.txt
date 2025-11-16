cmake_minimum_required(VERSION 3.10)
project(SRXL2_Master C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Add compile definitions
add_definitions(-DSRXL_INCLUDE_MASTER_CODE)
add_definitions(-DSRXL_NUM_OF_BUSES=1)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../SRXL2/Source
    ${CMAKE_CURRENT_SOURCE_DIR}/../SRXL2/Examples
)

# SRXL2 Master Library
add_library(srxl2_master STATIC
    spm_srxl_master.c
    spm_srxl_master.h
    spm_srxl_master_config.h
)

# Optional: Build examples
option(BUILD_MASTER_EXAMPLES "Build SRXL2 Master examples" ON)

if(BUILD_MASTER_EXAMPLES)
    # Example 1: Simple Logger
    add_executable(example1_simple_logger
        examples/example1_simple_logger.c
        spm_srxl_master.c
    )
    target_link_libraries(example1_simple_logger m)

    # Example 2: Telemetry Monitor
    add_executable(example2_telemetry_monitor
        examples/example2_telemetry_monitor.c
        spm_srxl_master.c
    )
    target_link_libraries(example2_telemetry_monitor m)

    # Example 3: Custom Scheduler
    add_executable(example3_custom_scheduler
        examples/example3_custom_scheduler.c
        spm_srxl_master.c
    )
    target_link_libraries(example3_custom_scheduler m)

    # Example 4: Device Manager
    add_executable(example4_device_manager
        examples/example4_device_manager.c
        spm_srxl_master.c
    )
    target_link_libraries(example4_device_manager m)

    # Install examples
    install(TARGETS
        example1_simple_logger
        example2_telemetry_monitor
        example3_custom_scheduler
        example4_device_manager
        DESTINATION bin
    )
endif()

# Install library and headers
install(TARGETS srxl2_master DESTINATION lib)
install(FILES
    spm_srxl_master.h
    spm_srxl_master_config.h
    DESTINATION include/srxl2_master
)

# Install README and documentation
install(FILES README.md DESTINATION share/doc/srxl2_master)
