/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#ifndef NEWTON_UTILS_H_
#define NEWTON_UTILS_H_

#include "newtonStdafx.h"

template<class Object, class Handle>
Object* PointerCast(const Handle* const ptr)
{
	return const_cast<Object*>(reinterpret_cast<const Object*>(ptr));
}

inline bool CheckFloat____(const ndFloat32* ptr, ndInt32 size)
{
	#ifdef _DEBUG
	for (ndInt32 i = 0; i < size; ++i)
	{
		if (!_finite(ptr[i]) || _isnan(ptr[i]))
		{
			return false;
		}
	}
	#endif
	return true;
}

inline ndMatrix GetMatrix(const ndFloat32* const matrix4x4Ptr)
{
	ndMatrix matrix(matrix4x4Ptr);
	if (!CheckFloat____(&matrix[0][0], 16))
	{
		ndExpandTraceMessage(("uninitialized matrix, setting to identity\n"));
		matrix = ndGetIdentityMatrix();
	}
	return matrix;
}


#endif 

