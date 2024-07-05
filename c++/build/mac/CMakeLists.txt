cmake_minimum_required(VERSION 3.9)
project(dxl_mac_cpp VERSION 1.0.1 DESCRIPTION "libdxl_mac_cpp description")
include(GNUInstallDirs)
add_library(dxl_mac_cpp SHARED
    ../../src/dynamixel_sdk/group_bulk_read.cpp
    ../../src/dynamixel_sdk/group_bulk_read.cpp		
    ../../src/dynamixel_sdk/packet_handler.cpp		
    ../../src/dynamixel_sdk/port_handler_mac.cpp
    ../../src/dynamixel_sdk/group_bulk_write.cpp		
    ../../src/dynamixel_sdk/port_handler.cpp		
    ../../src/dynamixel_sdk/port_handler_windows.cpp
    ../../src/dynamixel_sdk/group_sync_read.cpp		
    ../../src/dynamixel_sdk/port_handler_arduino.cpp	
    ../../src/dynamixel_sdk/protocol1_packet_handler.cpp
    ../../src/dynamixel_sdk/group_sync_write.cpp		
    ../../src/dynamixel_sdk/port_handler_linux.cpp		
    ../../src/dynamixel_sdk/protocol2_packet_handler.cpp

    )
set_target_properties(dxl_mac_cpp PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER  ../../src/DynamixelSDK.h
    
    )
target_include_directories(dxl_mac_cpp PUBLIC ../../include/dynamixel_sdk/)

install(TARGETS dxl_mac_cpp
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

 install(DIRECTORY "../../include/dynamixel_sdk/" # source directory
        DESTINATION "include/dynamixel_sdk/" # target directory
        FILES_MATCHING # install only matched files
        PATTERN "*.h" # select header files
)