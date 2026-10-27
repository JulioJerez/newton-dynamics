/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef ND_VECTOR8_H_
#define ND_VECTOR8_H_

#define ND_SIMD8_WORK_GROUP_SIZE	8 

#if defined (D_SCALAR_VECTOR_CLASS) || defined(__clang__)  || defined(__GNUC__)
	// using scalar version for all none visual studio compilers or none x86/x64 cpus
	#include "ndVector8Scalar.h"
#else
	// using explicit intrinsics only for visual studio compiler
	#include "ndVector8Simd.h"
#endif


#endif
