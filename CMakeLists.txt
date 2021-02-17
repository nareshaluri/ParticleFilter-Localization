project(PARTICLE_FILTER)

cmake_minimum_required (VERSION 3.5)

# set the third party root
set(THIRD_ROOT ${CMAKE_SOURCE_DIR}/third_party)
message(STATUS "Third party ${THIRD_ROOT}")

if(EXISTS ${THIRD_ROOT})
  message(STATUS "Found third party directory")
else()
  message(FATAL_ERROR "Missing ${THIRD_ROOT}")
endif()

# Search for uWebSockets includes & libraries
set(CMAKE_PREFIX_PATH "${THIRD_ROOT}/uWebSockets")
# check that uWebSockets header has been installed
if(NOT EXISTS "${CMAKE_PREFIX_PATH}/include/uWS/uWS.h")
  message(FATAL_ERROR "Could NOT find uWebSockets (missing: include/uWS/uWS.h)")
endif()
# check that uWebSockets lib has been installed (assume it contains the library file to link to)
if(NOT EXISTS "${CMAKE_PREFIX_PATH}/lib")
  message(FATAL_ERROR "Could NOT find uWebSockets (missing: lib)")
endif()
# find the the libraries
find_library(uWS_LIBRARY NAMES libuWS.so PATHS "${THIRD_ROOT}/uWebSockets/lib")
message(STATUS "Found uWebSockets: \n"   "--   " ${uWS_LIBRARY} "\n")
include_directories("${THIRD_ROOT}/uWebSockets/include")


add_definitions(-std=c++11)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/particle_filter.cpp src/main.cpp)

add_executable(particle_filter ${sources})

target_link_libraries(particle_filter z ssl uv ${uWS_LIBRARY})

