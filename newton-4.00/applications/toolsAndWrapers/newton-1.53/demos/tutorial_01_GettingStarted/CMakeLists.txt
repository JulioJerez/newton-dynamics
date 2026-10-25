cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)

project("tutorial_01_GettingStarted")
set (projectName "tutorial_01_GettingStarted")


file(GLOB CPP_SOURCE *.h *.cpp)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})


include_directories(.)
include_directories(../)
include_directories(../common)

link_directories(${libraryPath})

#set(CMAKE_C_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Zi /Od /MTd")
#set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Zi /Od /MT")
#set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} /Zi /Od /MT")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Zi /Od /MTd")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Zi /Od /MT")
#set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} /Zi /Od /MT")

add_executable(${projectName} ${CPP_SOURCE})

target_link_libraries (${projectName} common newton)
target_link_libraries (${projectName} "${CMAKE_CURRENT_SOURCE_DIR}/../gl/Glut32.lib")

