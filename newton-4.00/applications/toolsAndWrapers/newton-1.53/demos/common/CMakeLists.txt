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

set (projectName "common")
message (${projectName})

include_directories(.)
include_directories(..)
include_directories(../..)

file(GLOB CPP_SOURCE *.h *.cpp)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

add_library(${projectName} STATIC ${CPP_SOURCE})

#install(FILES ${CPP_SOURCE} DESTINATION "install/source")
#install(TARGETS ${projectName} 
#		LIBRARY DESTINATION "install/lib" 
#		ARCHIVE DESTINATION "install/lib" 
#		RUNTIME DESTINATION "install/bin")
#set_target_properties(${projectName} PROPERTIES FOLDER "utils")

#if (MSVC)
#	set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YundModelStdafx.h")
#	set_source_files_properties(ndModelStdafx.cpp PROPERTIES COMPILE_FLAGS "/YcndModelStdafx.h")
#endif()



