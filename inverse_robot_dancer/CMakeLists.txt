cmake_minimum_required(VERSION 2.8)
project(servo)

#set(CMAKE_CXX_STANDARD 11)
set( CMAKE_CXX_FLAGS "-std=c++11" )
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11") ##？？？？
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread") #需要手动链接	
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
${PROJECT_SOURCE_DIR}/include
)

add_executable(servo src/main.cpp)
