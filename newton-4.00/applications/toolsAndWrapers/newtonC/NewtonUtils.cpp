/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#include "newtonStdafx.h"
#include "Newton.h"
#include "newtonWorld.h"
#include "newtonMaterial.h"
#include "newtonBodyNotify.h"

static void* NewtonDefaultAlloc(int sizeInBytes)
{
	return malloc(sizeInBytes);
}

static void NewtonDefaultFree(void* const ptr, int sizeInBytes)
{
	free(ptr);
}

NewtonFreeMemory newtonFree = NewtonDefaultFree;
NewtonAllocMemory newtonAlloc = NewtonDefaultAlloc;

static void* ndNewtonAllocator(size_t size)
{
	return newtonAlloc(int(size));
}

static void ndNewtonFree(void* const ptr)
{
	newtonFree(ptr, int(ndMemory::GetSize(ptr)));
}

void* NewtonAlloc(int sizeInBytes)
{
	return ndMemory::Malloc(sizeInBytes);
}

void NewtonFree(void* const ptr)
{
	ndMemory::Free(ptr);
}

#if !defined (_NEWTON_STATIC_LIB) 
void* operator new(std::size_t count)
{
	return ndMemory::Malloc(count);
}

void operator delete(void* ptr) noexcept
{
	ndMemory::Free(ptr);
}
#endif

bool CheckFloat(ndFloat32* ptr, ndInt32 size)
{
	for (ndInt32 i = 0; i < size; ++i)
	{
		if (!_finite(ptr[i]) || _isnan(ptr[i]))
		{
			return false;
		}
	}
	return true;
}

// fixme: needs docu
// @param mallocFnt is a pointer to the memory allocator callback function. If this parameter is NULL the standard *malloc* function is used.
// @param mfreeFnt is a pointer to the memory release callback function. If this parameter is NULL the standard *free* function is used.
//
void NewtonSetMemorySystem(NewtonAllocMemory mallocFnt, NewtonFreeMemory mfreeFnt)
{
	newtonFree = mfreeFnt;
	newtonAlloc = mallocFnt;
	ndMemory::SetMemoryAllocators(ndNewtonAllocator, ndNewtonFree);
}

/*!
  Return the exact amount of memory (in Bytes) use by the engine at any given time.

  @return total memory use by the engine.

  Applications can use this function to ascertain that the memory use by the
  engine is balanced at all times.

  See also: ::NewtonCreate
*/
int NewtonGetMemoryUsed()
{
	TRACE_FUNCTION(__FUNCTION__);
	return int(ndMemory::GetMemoryUsed());
}

/*!
  Return the current library version number.

  @return version number as an integer, eg 314.

  The version number is a three-digit integer.

  First digit:  major version (interface changes among other things)
  Second digit: major patch number (new features, and bug fixes)
  Third Digit:  minor bug fixed patch.
*/
int NewtonWorldGetVersion()
{
	TRACE_FUNCTION(__FUNCTION__);
	return NEWTON_MAJOR_VERSION * 100 + NEWTON_MINOR_VERSION;
}

/*!
  Return the size of a Newton dFloat in bytes.

  @return sizeof(dFloat)
*/
int NewtonWorldFloatSize()
{
	TRACE_FUNCTION(__FUNCTION__);
	return sizeof(ndFloat32);
}

