# Copyright: (C) 2019 EDPR - Istituto Italiano di Tecnologia
# Authors: Arren Glover, Daniele Domenichelli
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 3.5)
project(event-driven
        LANGUAGES CXX C
        VERSION 1.3)

include(GNUInstallDirs)
include(FeatureSummary)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(YCM QUIET)
set_package_properties(YCM PROPERTIES TYPE RECOMMENDED)

if(NOT YCM_FOUND)
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
endif()

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

# option(BUILD_TESTING "Create tests using CMake" OFF)
# if(BUILD_TESTING)
#     enable_testing()
# endif()

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

set(CONTEXT_DIR event-driven)
set(EVENTDRIVEN_LIBRARY event-driven)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fno-strict-aliasing -std=c++11 -fPIC")
#set(EVENTDRIVENLIBS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/libraries/include)

option(ADD_DOCS_TO_IDE "Add apps/documentation to IDE" OFF)

find_package(MetavisionSDK COMPONENTS core driver QUIET)
set_package_properties(MetavisionSDK PROPERTIES TYPE OPTIONAL)

find_package(prophesee_core QUIET)
set_package_properties(prophesee_core PROPERTIES TYPE OPTIONAL)

find_package(OpenCV)
set_package_properties(OpenCV PROPERTIES TYPE OPTIONAL)

if(OpenCV_FOUND)
    set(OpenCV_dep OpenCV)
    find_package(YARP COMPONENTS os sig math dev cv REQUIRED)
else()
    set(OpenCV_dep "")
    find_package(YARP COMPONENTS os sig math dev REQUIRED)
endif()
yarp_configure_external_installation(event-driven)



feature_summary(WHAT ALL INCLUDE_QUIET_PACKAGES)
if(OpenCV_FOUND)
    message(STATUS "Found OpenCV: (found version ${OpenCV_VERSION})")
endif()
add_subdirectory(lib)
add_subdirectory(src)
# add_subdirectory(bindings)
# if(BUILD_TESTING)
#     add_subdirectory(test)
# endif()

if(ADD_DOCS_TO_IDE)
    file(GLOB tutorialfiles documentation/*.md
                            documentation/application_instructions/*.md)
    add_custom_target(project_documentation SOURCES README.md ${tutorialfiles})
endif(ADD_DOCS_TO_IDE)


include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME}
                            EXPORT eventdriven
                            VERSION ${${PROJECT_NAME}_VERSION}
                            COMPATIBILITY SameMajorVersion
                            LOWERCASE_FILENAMES
                            NAMESPACE ev::
                            DEPENDENCIES "YARP COMPONENTS OS sig"
                                         ${OpenCV_dep}
                            INCLUDE_CONTENT "yarp_configure_external_installation(event-driven NO_PATH_D)")

include(AddUninstallTarget)
