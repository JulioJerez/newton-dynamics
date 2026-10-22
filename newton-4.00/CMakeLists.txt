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

option("NEWTON_BUILD_ASSET_EDITOR" "build the asset editor" ON)
option("NEWTON_BUILD_SANDBOX_DEMOS" "generates demos projects" ON)
option("NEWTON_BUILD_PROFILER" "build profiler" OFF)
option("NEWTON_BUILD_SINGLE_THREADED" "single threaded" OFF)
option("NEWTON_BUILD_SHARED_LIBS" "build shared library" ON)
option("NEWTON_ENABLE_OPENCL_SDK" "enable opencl compute" OFF)
option("NEWTON_DOUBLE_PRECISION" "generate double precision" OFF)
option("NEWTON_SCALAR_VECTOR_CLASS" "generate simd vector class" OFF)
option("NEWTON_STATIC_RUNTIME_LIBRARIES" "use windows static libraries" OFF)
option("NEWTON_USE_DEFAULT_NEW_AND_DELETE" "overload new and delete when building dll" OFF)
option("NEWTON_BUILD_TESTS" "tutorials test (required newton static libraries option" OFF)

#message ("operation System: " ${CMAKE_SYSTEM_NAME})
message(STATUS "Target System Processor: ${CMAKE_SYSTEM_PROCESSOR}")

set(CMAKE_CONFIGURATION_TYPES Debug RelWithDebInfo Release)
set(CMAKE_DEBUG_POSTFIX "_d")

project(NewtonSDK)

if (MSVC)
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif()

# determine if we are compiling for a 32bit or 64bit system
include(CheckTypeSize)
CHECK_TYPE_SIZE("void*" PTR_SIZE BUILTIN_TYPES_ONLY)

#set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Don't use e.g. GNU extension (like -std=gnu++11) for portability
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
	message ("arm processor detected")
	set (X86 FALSE)
else()
	set (X86 TRUE)
	message ("assume x86 processor detected")
	option("NEWTON_ENABLE_AVX2" "enable AVX2" ON)
endif()

if (NEWTON_SCALAR_VECTOR_CLASS OR (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")) 
	add_definitions(-DD_SCALAR_VECTOR_CLASS)
endif()

if(NEWTON_DOUBLE_PRECISION)
	add_definitions(-DD_NEWTON_USE_DOUBLE)
endif()

if(NEWTON_BUILD_SINGLE_THREADED)
	add_definitions(-DD_USE_THREAD_EMULATION)
endif()

if(NEWTON_ENABLE_OPENCL_SDK AND MSVC)
	add_definitions(-DD_USE_OPENCL_SDK)
endif()

if (X86 AND NEWTON_ENABLE_AVX2)
	add_definitions(-DD_NEWTON_USE_AVX2_OPTION)
endif()

if(NEWTON_BUILD_SHARED_LIBS)
	add_definitions(-D_D_TINY_DLL)
    add_definitions(-D_D_CORE_DLL)
	add_definitions(-D_D_TINY_DLL)
	add_definitions(-D_D_NEWTON_DLL)
	add_definitions(-D_D_COLLISION_DLL)
endif(NEWTON_BUILD_SHARED_LIBS)

#If no build type set, Release as default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
          "Choose the type of build, options are: Debug RelWithDebInfo Release"
          FORCE)
endif(NOT CMAKE_BUILD_TYPE)

#check for UNIX
if(UNIX)
	# ----------------------------------------------------------------------
	# Ensure the RPATH for libndNewton is set correctly both inside the build
	# tree as well as in the installed location (ie after `make install`).
	#
	# The following settings were adapted from here
	# https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling

	set(DEFAULT_INSTALL_PREFIX "/usr/local/${PROJECT_NAME}")
	if (${CMAKE_INSTALL_PREFIX} STREQUAL ${DEFAULT_INSTALL_PREFIX})
		set(CMAKE_INSTALL_PREFIX "../newton-4.00/" CACHE PATH "..." FORCE)
		message ("installPrefixPath = ${CMAKE_INSTALL_PREFIX}")
	endif()

	# Use the full RPATH for the build tree.
	set(CMAKE_SKIP_BUILD_RPATH FALSE)

	# Do not use the install RPATH during building but use later on when we install the library.
	set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

	# Set this RPATH in the installed library.
	set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

	# Add the automatically determined parts of the RPATH which point to directories
	# outside the build tree to the install RPATH.
	set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

	set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)
	if(NOT NEWTON_BUILD_SHARED_LIBS)
		add_compile_options(-fPIC)
	endif()

	if(APPLE) 
		message ("disabling warning: -no_warning_for_no_symbols seems impossble")
		message ("nothing is eassy with xcode, it really sucks")
		#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -no_warning_for_no_symbols ")
		#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -no_warning_for_no_symbols ")
		#set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -no_warning_for_no_symbols ")
	else()
		set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wno-unused-cli ")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wno-unused-cli ")
		set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -Wno-unused-cli ")
	endif()

	if (X86 AND NEWTON_ENABLE_AVX2 AND (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
		set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -march=alderlake ")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=alderlake ")
		set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -march=alderlake ")
	endif()

	include("GNUInstallDirs")

	# uninstall target
	if(NOT TARGET uninstall)
		configure_file(
			"${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
			"${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
			IMMEDIATE @ONLY)

		add_custom_target(uninstall
			COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
	endif()

	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
endif ()

if (MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
	add_definitions(-D_DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR)

	set(DEFAULT_INSTALL_PREFIX "C:/Program Files (x86)/${PROJECT_NAME}")
	if (${CMAKE_INSTALL_PREFIX} STREQUAL ${DEFAULT_INSTALL_PREFIX})
		set(CMAKE_INSTALL_PREFIX "../newton-4.00/" CACHE PATH "..." FORCE)
		message ("installPrefixPath = ${CMAKE_INSTALL_PREFIX}")
	endif()

	if(NEWTON_STATIC_RUNTIME_LIBRARIES)
		message ("link to static runtime libraries")
		set(CMAKE_C_FLAGS_DEBUG "/MTd")
		set(CMAKE_C_FLAGS_RELEASE "/MT")
		set(CMAKE_C_FLAGS_RELWITHDEBINFO "/MT")
		set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
		set(CMAKE_CXX_FLAGS_RELEASE "/MT")
		set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/MT")
	else()
		message ("link to dynamic runtime libraries")
		set(CMAKE_C_FLAGS_DEBUG "/MDd")
		set(CMAKE_C_FLAGS_RELEASE "/MD")
		set(CMAKE_C_FLAGS_RELWITHDEBINFO "/MD")
		set(CMAKE_CXX_FLAGS_DEBUG "/MDd")
		set(CMAKE_CXX_FLAGS_RELEASE "/MD")
		set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/MD")
	endif()

	message("Compiler Id: ${CMAKE_CXX_COMPILER_ID}") 

	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Zi /Od /fp:fast")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi /Gy /GS /Ox /Oi /Ot /Ob2 /Oy /Qpar /fp:fast")
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /Zi /Gy /GS /Ox /Oi /Ot /Ob2 /Oy /Qpar /fp:fast")
	set(CMAKE_WARNINGS "/Wall /wd26495 /wd4061 /wd4191 /wd4201 /wd4263 /wd4264 /wd4266 /wd4324 /wd4505 /wd4514 /wd4582 /wd4587 /wd4625 /wd4626 /wd4710 /wd4711 /wd4371 /wd4571 /wd4623 /wd4668 /wd4774 /wd4820 /wd5026 /wd5027 /wd5039 /wd5045 /wd5264")

	if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
		set(CMAKE_WARNINGS "${CMAKE_WARNINGS} -Wno-unsafe-buffer-usage -Wno-c++98-compat -Wno-c++98-compat-pedantic -Wno-sign-conversion -Wno-extra-semi-stmt -Wno-gnu-anonymous-struct -Wno-nested-anon-types -Wno-covered-switch-default -Wno-float-equal")
	endif()

	if(NEWTON_ENABLE_AVX2)
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /arch:AVX2")
		set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /arch:AVX2")
	else()
		#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /arch:SSE2")
		#set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /arch:SSE2")
	endif()
	
	message("CMAKE_WARNINGS is ${CMAKE_WARNINGS}")
	message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
	message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")
	message("CMAKE_CXX_FLAGS_RELWITHDEBINFO is ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${CMAKE_WARNINGS}")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_WARNINGS}")
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${CMAKE_WARNINGS}")

	set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${CMAKE_WARNINGS} /Zi /Od")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${CMAKE_WARNINGS} /Zi /Od")
	set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} ${CMAKE_WARNINGS} /Zi /Od")

elseif(MINGW)
	if(CMAKE_CL_64)
		message("build 64 bit")
	else()
		message("build 32 bit")
	endif()
endif()

if(NEWTON_BUILD_PROFILER)
	add_definitions(-DD_PROFILER)
endif()

add_subdirectory(sdk)
add_subdirectory(thirdParty)
add_subdirectory(applications)

if(NEWTON_BUILD_TESTS)
	add_subdirectory(tests)
endif()