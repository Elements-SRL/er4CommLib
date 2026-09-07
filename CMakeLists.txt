cmake_minimum_required(VERSION 3.21)

project(er4commlib
    VERSION 1.21.1
    LANGUAGES CXX
)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

option(BUILD_FOR_LABVIEW "Build the library as a DLL wrapper for LabVIEW" OFF)

# Library

set(LIB_SOURCES
    src/er4commlib.cpp
    src/messagedispatcher.cpp
    src/commandcoder.cpp
    src/devices/e1/messagedispatcher_e1b_el03c_legacyedr3_pcbv02.cpp
    src/devices/e1/messagedispatcher_e1light.cpp
    src/devices/e1/messagedispatcher_e1plus.cpp
    src/devices/e1/messagedispatcher_e1plus_gen2_el03c_pcbv01.cpp
    src/devices/e1/messagedispatcher_e1hc.cpp
    src/devices/e1/messagedispatcher_e1hc_gen2_el09a_pcbv01.cpp
    src/devices/e1/messagedispatcher_e1light_gen2_el03c_pcbv01.cpp
    src/devices/e1/messagedispatcher_e1uln_gen2_el03c_pcbv06.cpp
    src/devices/e1/messagedispatcher_e1uln_gen2_el03c_pcbv067_fwv03.cpp
    src/devices/e1/messagedispatcher_e1uln.cpp
    src/devices/e1/messagedispatcher_e1light_gen2_el03c_pcbulnv067.cpp
    src/devices/e1/messagedispatcher_e1_e4nbased.cpp
    src/devices/eNPR/messagedispatcher_enpr.cpp
    src/devices/eNPR/messagedispatcher_enpr_hc.cpp
    src/devices/eNPR/messagedispatcher_enpr_hc_el06g_pcbv01_fwv01.cpp
    src/devices/e2/messagedispatcher_e2hc.cpp
    src/devices/e2/messagedispatcher_e2uln_v01.cpp
    src/devices/e2/messagedispatcher_e2qc_debug.cpp
    src/devices/e4/messagedispatcher_e4n.cpp
    src/devices/e4/messagedispatcher_e4n_fwv01.cpp
    src/devices/e4/messagedispatcher_e4e.cpp
    src/devices/e4/messagedispatcher_e4n_inputsync.cpp
    src/devices/e4/messagedispatcher_e4qc01a.cpp
    src/devices/e16/messagedispatcher_e16e.cpp
    src/devices/e16/messagedispatcher_e16n.cpp
    src/devices/e16/messagedispatcher_e16n_ramps.cpp
    src/devices/e16/messagedispatcher_e16n_el08a.cpp
    src/devices/e16/messagedispatcher_e16fastpulses.cpp
    src/devices/e16/messagedispatcher_e16hc.cpp
    src/devices/e16/messagedispatcher_e16eth.cpp
    src/devices/testboard/messagedispatcher_el03ce_char.cpp
    src/devices/testboard/messagedispatcher_el06b.cpp
    src/devices/testboard/messagedispatcher_el06c.cpp
    src/devices/testboard/messagedispatcher_el06d_el06e.cpp
    src/devices/testboard/messagedispatcher_el06f.cpp
    src/devices/testboard/messagedispatcher_el09a_tb.cpp
    src/devices/testboard/messagedispatcher_el10a_tb.cpp
    src/devices/fake/messagedispatcher_fake_enpr.cpp
    src/devices/fake/messagedispatcher_fake_enpr_hc.cpp
    src/devices/fake/messagedispatcher_fake_e16n.cpp
    src/devices/fake/messagedispatcher_fake_e16fastpulses.cpp
    src/devices/testboard/messagedispatcher_el08b_tb.cpp
)

if(BUILD_FOR_LABVIEW)
    set(TARGET_NAME "er4commlib_labview")

    list(APPEND LIB_SOURCES src/er4commlib_labview.cpp)
    set(LIB_HEADERS include/er4commlib_labview.h)

    add_library(${TARGET_NAME} SHARED ${LIB_SOURCES} ${LIB_HEADERS})
    add_library(er4commlib::er4commlib ALIAS ${TARGET_NAME})

    target_compile_definitions(${TARGET_NAME} PRIVATE
        ER4COMMLIB_LIBRARY
    )

    target_compile_definitions(${TARGET_NAME} PUBLIC
        ER4COMMLIB_LABVIEW_WRAPPER
    )

    cmake_path(SET LABVIEW_TO_C_PATH_CMAKE "$ENV{LABVIEW_TO_C_PATH}")
    include(${LABVIEW_TO_C_PATH_CMAKE}cmake/Findlabview2c.cmake)

    target_link_libraries(${TARGET_NAME}
        PUBLIC
        labview2c::labview2c
    )

else()
    set(TARGET_NAME "er4commlib")

    add_library(${TARGET_NAME} STATIC ${LIB_SOURCES})
    add_library(er4commlib::er4commlib ALIAS ${TARGET_NAME})

    target_compile_definitions(${TARGET_NAME} PUBLIC ER4COMMLIB_STATIC)

endif()

# C standard

set_target_properties(${TARGET_NAME} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# Include directories

target_include_directories(${TARGET_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/devices/e1>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/devices/eNPR>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/devices/e2>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/devices/e4>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/devices/e16>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/devices/fake>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/devices/testboard>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Defines

target_compile_definitions(${TARGET_NAME}
    PUBLIC
    COMMLIB_MAJOR=${PROJECT_VERSION_MAJOR}
    COMMLIB_MINOR=${PROJECT_VERSION_MINOR}
    COMMLIB_PATCH=${PROJECT_VERSION_PATCH}
)

# Transitive dependency

cmake_path(SET FTDI_UTILS_PATH_CMAKE "$ENV{FTDI_UTILS_PATH}")
list(APPEND CMAKE_PREFIX_PATH "${FTDI_UTILS_PATH_CMAKE}")

find_package(ftdi_utils REQUIRED CONFIG)

target_link_libraries(${TARGET_NAME}
    PUBLIC
    ftdi_utils::ftdi_utils
)

# target name debug/release

set_target_properties(${TARGET_NAME} PROPERTIES
    DEBUG_POSTFIX d
)

# Install

cmake_path(SET ER4COMMLIB_PATH_CMAKE "$ENV{ER4COMMLIB_PATH}")
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX ${ER4COMMLIB_PATH_CMAKE} CACHE PATH "Install path" FORCE)
endif()

install(TARGETS ${TARGET_NAME}
    EXPORT er4commlibTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install header

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
)

# Export target

install(EXPORT er4commlibTargets
    FILE er4commlibTargets.cmake
    NAMESPACE er4commlib::
    DESTINATION "${CMAKE_INSTALL_PREFIX}/cmake/\${CMAKE_INSTALL_CONFIG_NAME}"
)

# Config package

configure_package_config_file(
    cmake/er4commlibConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/er4commlibConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/er4commlibConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/er4commlibConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/er4commlibConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake
)
