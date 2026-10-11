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

bool CheckFloat(ndFloat32* ptr, ndInt32 size);

#endif 

