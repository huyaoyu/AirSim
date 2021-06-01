cmake_minimum_required(VERSION 3.5.0)
project(CustomizedInterpolation)


add_compile_options(-fPIC)

find_package(Threads REQUIRED)

# ========== Partially copied from the CMakeLists.txt for AirLib. ==========

if (APPLE)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wstrict-aliasing -D__CLANG__")
else ()
    set(CMAKE_CXX_FLAGS "\
        -std=c++14 -ggdb -Wall -Wextra \
        -Wno-variadic-macros -Wno-parentheses -Wno-unused-function -Wno-unused \
        -pthread -fPIC\
        ${CMAKE_CXX_FLAGS}")

    if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
        set(CMAKE_CXX_FLAGS "-stdlib=libc++ -Wno-documentation -Wno-unknown-warning-option ${CMAKE_CXX_FLAGS}")
        set(CXX_EXP_LIB "-lc++fs -ferror-limit=10")
    else()
        set(CXX_EXP_LIB "-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel")
    endif ()
    message(STATUS ">>> ${CMAKE_CXX_COMPILER_ID}")
endif ()

set(BUILD_PLATFORM "x64")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if (CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
endif ()

## TODO: we are not using Boost any more so below shouldn't be needed
## common boost settings to make sure we are all on the same page
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
#set(Boost_USE_STATIC_RUNTIME ON)

## TODO: probably should set x64 explicitly
## strip x64 from /machine:x64 from CMAKE_STATIC_LINKER_FLAGS and set in BUILD_PLATFORM
if(NOT "${CMAKE_STATIC_LINKER_FLAGS}" STREQUAL "")
  string(SUBSTRING ${CMAKE_STATIC_LINKER_FLAGS} 9 -1 "BUILD_PLATFORM")
endif()

# message(STATUS ">>> CMAKE_STATIC_LINKER_FLAGS = ${CMAKE_STATIC_LINKER_FLAGS}")
# set(CMAKE_STATIC_LINKER_FLAGS "${CMAKE_STATIC_LINKER_FLAGS} -static")

find_path(AIRSIM_ROOT NAMES AirSim.sln PATHS ".." "../.." "../../.." "../../../.." "../../../../.." "../../../../../..")
message(STATUS "found AIRSIM_ROOT=${AIRSIM_ROOT}")

SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/lib)

find_package(Boost)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

set(OpenCV_STATIC ON)
# set(OpenCV_DIR /home/yaoyu/Libraries/OpenCV/install_clang)
set(OpenCV_DIR $ENV{OPENCV_AIRSIM}/lib/cmake/opencv4)
find_package(OpenCV 4.2.0 REQUIRED NO_DEFAULT_PATH)
message(STATUS "OpenCV_VERSION = ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_INSTALL_PATH = ${OpenCV_INSTALL_PATH}")
message(STATUS "OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")

include_directories(
  /usr/lib/llvm-10/include
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/${PROJECT_NAME}/include
)

find_package(Eigen3)

set(LIB_NAME ${PROJECT_NAME})

add_library(${LIB_NAME} STATIC
        ${AIRSIM_ROOT}/${LIB_NAME}/src/planar_2_full_view.cpp )

target_include_directories(${LIB_NAME} 
    PRIVATE 
    ${AIRSIM_ROOT}/AirLib/deps/eigen3
    ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${LIB_NAME}
        PRIVATE ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES} Eigen3::Eigen ${OpenCV_LIBRARIES})
