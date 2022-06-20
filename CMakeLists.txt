cmake_minimum_required(VERSION 3.13)

project(TinyURDFParser)

set(CMAKE_CXX_STANDARD 17)
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC" )

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

set(LIBRARY_HEADERS
    include/TinyURDFParser/TinyURDFParser.hpp
    include/TinyURDFParser/KDLRobot.hpp
    lib/tinyxml2/tinyxml2.h
)

set(LIBRARY_SOURCES
    TinyURDFParser/TinyURDFParser.cpp
    TinyURDFParser/KDLRobot.cpp
    lib/tinyxml2/tinyxml2.cpp
)

add_library(
    ${PROJECT_NAME}

    STATIC

    ${LIBRARY_HEADERS}
    ${LIBRARY_SOURCES}
)

option(BUILD_EXAMPLES "Build examples" ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
    add_subdirectory(lib/Catch2)
    enable_testing()
    add_subdirectory(tests)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC    
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # Make headers available to the compilers
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>  # Make headers available to the compilers
    $<INSTALL_INTERFACE:include>  # Make headers available to the compilers
    $<INSTALL_INTERFACE:lib>  # Make headers available to the compilers
    ${Eigen3_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} # Link used libraries
    PUBLIC
        Eigen3::Eigen
        ${orocos_kdl_LIBRARIES}
)

export(TARGETS ${PROJECT_NAME} NAMESPACE ${PROJECT_NAME}:: FILE ${PROJECT_NAME}Config.cmake)
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
