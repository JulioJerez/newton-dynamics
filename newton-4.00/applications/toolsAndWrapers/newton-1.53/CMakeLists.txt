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

set(CMAKE_CONFIGURATION_TYPES Debug RelWithDebInfo Release)
#set(CMAKE_DEBUG_POSTFIX "_d")

if(NEWTON_BUILD_SHARED_LIBS)
	add_definitions(-D_NEWTON_BUILD_DLL)
endif()

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" PROCESSOR_ARCH)

if (PROCESSOR_ARCH  MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
	message ("x86-64 processor detected")

	option("NEWTON_BUILD_DEMOS" "build demos" ON)
	option("NEWTON_ENABLE_AVX2" "enable AVX2" OFF)

	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  /Zi /Od")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  /Gy /Zi /GS /Ox /Oi /Ot /Ob2 /Oy /Qpar /fp:fast")
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /Zi /O2 /Qpar /fp:fast")

	if (NEWTON_ENABLE_AVX2)
		add_definitions(-DD_NEWTON_USE_AVX2_OPTION)
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /arch:AVX2")
		set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /arch:AVX2")
	else()
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /arch:SSE2")
		set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /arch:SSE2")
	endif()
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang") 
	add_definitions(-DD_SCALAR_VECTOR_CLASS)
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU") 
	add_definitions(-DD_SCALAR_VECTOR_CLASS)
endif()

# source and header files
file(GLOB CPP_SOURCE 
	*.h
	*.cpp
	joints/*.h
	joints/*.cpp
	newton-4/source/*.h
	newton-4/source/*.c
	newton-4/source/*.cpp
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

include_directories(.)
include_directories(newton-4/source)

add_library(${projectName} SHARED ${CPP_SOURCE})

#if (MSVC OR MINGW)
#  set (cSuffix ".dll")
#else()
#  set (cSuffix ".so")
#endif()

if (NEWTON_BUILD_DEMOS)
	add_subdirectory(demos/common)
	add_subdirectory(demos/tutorial_01_GettingStarted)
	add_subdirectory(demos/tutorial_02_UsingCallbacks)
	add_subdirectory(demos/tutorial_03_UsingCollisionTree)
	add_subdirectory(demos/tutorial_04_IntroductionToMaterials)
	add_subdirectory(demos/tutorial_05_UsingJoints)
	add_subdirectory(demos/tutorial_06_UtilityFuntionality)
	#add_subdirectory(demos/tutorial_07_CharaterController)
	
endif()