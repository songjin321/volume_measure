cmake_minimum_required(VERSION 2.8.3)
project(dense_reconstruction)

#set(CMAKE_CXX_FLAGS "-std=c++11")
list(APPEND CMAKE_MODULE_PATH  ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_BUILD_TYPE Release)
set (CMAKE_CXX_STANDARD 11)


set(LIBELAS_SRC_DIR elas)
set(CMAKE_CXX_FLAGS "-msse3")
set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 

find_package(OpenCV REQUIRED)
find_package(Boost 1.54.0 COMPONENTS system filesystem program_options REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${LIBELAS_SRC_DIR}
)

FILE(GLOB LIBELAS_SRC_FILES "elas/*.cpp")

add_executable(dense ${LIBELAS_SRC_FILES} dense.cpp)
target_link_libraries(dense ${OpenCV_LIBS} ${Boost_LIBRARIES})
