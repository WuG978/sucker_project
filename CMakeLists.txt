# ----------------------------------------------------------------------------
#  Root CMake file for OpenCV
#
#    From the off-tree build directory, invoke:
#      $ cmake <PATH_TO_OPENCV_ROOT>
#
# ----------------------------------------------------------------------------

# Disable in-source builds to prevent source tree corruption.
if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
    message(FATAL_ERROR "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
")
endif()

#  minimum required cmake version that defined in cmake/ProjectMinDepVersions.cmake
include(cmake/ProjectMinDepVersions.cmake)
if(CMAKE_SYSTEM_NAME MATCHES WindowsPhone OR CMAKE_SYSTEM_NAME MATCHES WindowsStore)
    cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
    #Required to resolve linker error issues due to incompatibility with CMake v3.0+ policies.
    #CMake fails to find _fseeko() which leads to subsequent linker error.
    #See details here: http://www.cmake.org/Wiki/CMake/Policies
    cmake_policy(VERSION 2.8)
else()
    cmake_minimum_required(VERSION "${MIN_VER_CMAKE}" FATAL_ERROR)
endif()

#
# Configure CMake policies
#
if(POLICY CMP0074)
    # 1. Remove with 3.12.4.
    # 2. Remove search paths with *_ROOT since they will be automatically checked
    cmake_policy(SET CMP0074 NEW)
endif()

if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

# Set target C++ standard and required compiler features
set(CMAKE_CXX_STANDARD 14 CACHE STRING "The target C++ standard. PCL requires C++14 or higher.")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(PCL_CXX_COMPILE_FEATURES cxx_std_14)

set(CMAKE_CUDA_STANDARD 14 CACHE STRING "The target CUDA/C++ standard. PCL requires CUDA/C++ 14 or higher.")
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "possible configurations" FORCE)
# In case the user does not setup CMAKE_BUILD_TYPE, assume it's RelWithDebInfo
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "build type default to RelWithDebInfo, set to Release to improve performance" FORCE)
endif()

#
# Configure OpenCV CMake hooks
#


project(SuckerProject LANGUAGES CXX C)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# Make project require C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()


find_package(PCL REQUIRED)
find_package(realsense2 REQUIRED)
# include(cmake/FindRSSDK2.cmake)
# find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
# if(NOT REALSENSE2_FOUND)
#     SET(REALSENSE2_FOUND "realsense2")
#     message(WARN "Failed to find_library(realsense2)")
# endif()

include_directories(3rdparty)

add_subdirectory(tools)
add_subdirectory(3rdparty)

# getting-started
add_executable(${PROJECT_NAME} main.cpp)
# target_link_libraries(${PROJECT_NAME} ${RSSDK2_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY} ${PCL_LIBRARIES})

# Print helper
include(CMakePrintHelpers)
cmake_print_variables(PCL_LIBRARIES)
cmake_print_variables(realsense2_LIBRARY realsense2_INCLUDE_DIR)


install(
    TARGETS

    ${PROJECT_NAME}

    RUNTIME DESTINATION
    ${CMAKE_INSTALL_PREFIX}/bin
)
