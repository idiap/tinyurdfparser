# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
#
# SPDX-License-Identifier: GPL-3.0-only

cmake_minimum_required(VERSION 3.13)

project(TinyURDFParser)

set(CMAKE_CXX_STANDARD 17)
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -fPIC -O3" )

find_package(Eigen3 REQUIRED)

option(USE_KDL "USE_KDL" ON)
configure_file(include/TinyURDFParser/TinyURDFParser_config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/TinyURDFParser/TinyURDFParser_config.h)

find_package(orocos_kdl REQUIRED)

set(LIBRARY_HEADERS
    include/TinyURDFParser/TinyURDFParser.hpp
    include/TinyURDFParser/TinyURDFParser_config.h
    lib/tinyxml2/tinyxml2.h
)

set(LIBRARY_SOURCES
    TinyURDFParser/TinyURDFParser.cpp
    lib/tinyxml2/tinyxml2.cpp
)

if(USE_KDL)
    list(APPEND LIBRARY_HEADERS include/TinyURDFParser/KDLRobot.hpp)
    list(APPEND LIBRARY_SOURCES TinyURDFParser/KDLRobot.cpp)
endif()

add_library(
    ${PROJECT_NAME}

    STATIC

    ${LIBRARY_HEADERS}
    ${LIBRARY_SOURCES}
)

option(BUILD_EXAMPLES "Build examples" ON)
if(BUILD_EXAMPLES)
    if(USE_KDL)
        add_subdirectory(examples)
    endif()
endif()

option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    add_subdirectory(lib/Catch2)
    enable_testing()
    add_subdirectory(tests)
endif()


if(USE_KDL)
    target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # Make headers available to the compilers
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>  # Make headers available to the compilers
        ${Eigen3_INCLUDE_DIRS}
        ${orocos_kdl_INCLUDE_DIRS}
    )

    target_link_libraries(${PROJECT_NAME} # Link used libraries
        PUBLIC
            Eigen3::Eigen
            ${orocos_kdl_LIBRARIES}
    )

    option(BUILD_PYTHON_BINDINGS "Build python bindings" OFF)

    if(BUILD_PYTHON_BINDINGS)
        message(STATUS "Building python bindings")

        SET(PYTHON_VERSION 3 CACHE STRING "Python Version")

        set(PYBIND11_PYTHON_VERSION ${PYTHON_VERSION} CACHE STRING "Python version used by PyBind11")

        find_package(PythonInterp ${PYTHON_VERSION} REQUIRED)
        find_package(PythonLibs ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR} REQUIRED)

        set(LIBRARY_NAME "PyTinyURDFParser")

        set(PYTHON_SITE_PACKAGES_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages")  # This might be overridden below if built with catkin or if conda is detected

        if(DEFINED ENV{CONDA_PREFIX})
        message(STATUS "CONDA INSTALLATION DETECTED")
        set(PYTHON_SITE_PACKAGES_INSTALL_DIR "$ENV{CONDA_PREFIX}/lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages")  # This might be overridden below if built with catkin.
        endif()


        SET(PYTHON_MODULE_EXTENSION ".so")
        add_subdirectory(lib/pybind11)
        pybind11_add_module(${LIBRARY_NAME}
        TinyURDFParser/bindings.cpp)

        message(STATUS "Will install python library in: ${PYTHON_SITE_PACKAGES_INSTALL_DIR}")
        target_link_libraries(${LIBRARY_NAME} PRIVATE TinyURDFParser)

        install(TARGETS ${LIBRARY_NAME} DESTINATION "${PYTHON_SITE_PACKAGES_INSTALL_DIR}")
    endif()
else()
    target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # Make headers available to the compilers
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>  # Make headers available to the compilers
    ${Eigen3_INCLUDE_DIRS}
    )

    target_link_libraries(${PROJECT_NAME} # Link used libraries
    PUBLIC
        Eigen3::Eigen
    )
endif()

export(TARGETS ${PROJECT_NAME} NAMESPACE ${PROJECT_NAME}:: FILE ${PROJECT_NAME}Config.cmake)
set(CMAKE_EXPORT_PACKAGE_REGISTRY ON)
export(PACKAGE ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} # Install library
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include/TinyURDFParser
)

include(CMakePackageConfigHelpers) # Create this shit to be friendly with cmake guidelines
write_basic_package_version_file(
    ${PROJECT_NAME}ConfigVersion.cmake
    VERSION 1
    COMPATIBILITY AnyNewerVersion
)

install( # Install config file (needed to do findPackage(...))
    FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)

install(EXPORT ${PROJECT_NAME}Targets # Install Targets file
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

install(
    FILES
        ${LIBRARY_HEADERS}
    DESTINATION
        include/${PROJECT_NAME}
)
