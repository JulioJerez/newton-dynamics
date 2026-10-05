# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)

option("NEWTON_BUILD_SHARED_LIBS" "build shared library" ON)

set (projectName "newton")
project(${projectName})

message (${projectName})


set (wrapper "${PROJECT_BINARY_DIR}")

if(NEWTON_BUILD_SHARED_LIBS)
	add_definitions(-D_NEWTON_BUILD_DLL)
endif()

# source and header files
file(GLOB CPP_SOURCE 
	*.h
	*.cpp
	newton-4/source/*.h
	newton-4/source/*.c
	newton-4/source/*.cpp
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

include_directories(.)
include_directories(newton-4/source)

add_library(${projectName} SHARED ${CPP_SOURCE})


if (MSVC OR MINGW)
  set (cSuffix ".dll")
else()
  set (cSuffix ".so")
endif()

add_custom_command(
	TARGET ${projectName} POST_BUILD COMMAND ${CMAKE_COMMAND}
	ARGS -E copy $<TARGET_FILE:${projectName}> ${wrapper}/${projectName}/${projectName}${CMAKE_DEBUG_POSTFIX}${cSuffix})
