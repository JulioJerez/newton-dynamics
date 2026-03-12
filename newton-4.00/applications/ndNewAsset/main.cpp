/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndNewAssetStdafx.h"
#include "ndAssetEditor.h"

void* operator new (size_t size)
{
	void* const ptr = ndMemory::Malloc(size);
	return ptr;
}

void operator delete (void* ptr) noexcept
{
	ndMemory::Free(ptr);
}

class ndSetAllocators
{
	public:
	ndSetAllocators()
	{
		ndMemory::SetMemoryAllocators(PhysicsAlloc, PhysicsFree);
	}

	static void* PhysicsAlloc(size_t sizeInBytes)
	{
		void* const ptr = malloc(sizeInBytes);
		return ptr;
	}

	// memory free use by the engine
	static void PhysicsFree(void* ptr)
	{
		free(ptr);
	}
};

int main(int, char**)
{
	ndSetAllocators setAllocators;

	ndDemoEntityManager editor;
	editor.Run();
	return 0;
}

