cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)

project("tutorial_08_HeightFieldCollision")
set (projectName "tutorial_08_HeightFieldCollision")
message ("${projectName}")

file(GLOB CPP_SOURCE *.h *.cpp)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})


include_directories(.)
include_directories(../)
include_directories(../common)

link_directories(${libraryPath})

add_executable(${projectName} ${CPP_SOURCE})

target_link_libraries (${projectName} common newton)
target_link_libraries (${projectName} "${CMAKE_CURRENT_SOURCE_DIR}/../gl/Glut32.lib")

add_custom_command(
	TARGET ${projectName} POST_BUILD 
	COMMAND ${CMAKE_COMMAND}
	ARGS -E copy ${CMAKE_CURRENT_SOURCE_DIR}/../gl/glew32.dll ${PROJECT_BINARY_DIR}/$<CONFIG>)

add_custom_command(
	TARGET ${projectName} POST_BUILD 
	COMMAND ${CMAKE_COMMAND}
	ARGS -E copy ${CMAKE_CURRENT_SOURCE_DIR}/../gl/Glut32.dll ${PROJECT_BINARY_DIR}/$<CONFIG>)

add_custom_command(
	TARGET ${projectName} POST_BUILD 
	COMMAND ${CMAKE_COMMAND}
	ARGS -E copy ${PROJECT_BINARY_DIR}/../../$<CONFIG>/newton.dll ${PROJECT_BINARY_DIR}/$<CONFIG>)
