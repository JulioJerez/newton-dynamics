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

#include "newtonStdafx.h"
#include "newtonWorld.h"
#include "newtonMaterial.h"

ndNewtonMaterial::ndNewtonMaterial()
	:ndApplicationMaterial()
	,m_onAABBOverlap(nullptr)
	,m_onContactsProcess(nullptr)
	,m_onSubShapeAABBOverlap(nullptr)
{
}

ndNewtonMaterial::ndNewtonMaterial(const ndNewtonMaterial& copy)
	:ndApplicationMaterial(copy)
	,m_onAABBOverlap(copy.m_onAABBOverlap)
	,m_onContactsProcess(copy.m_onContactsProcess)
	,m_onSubShapeAABBOverlap(copy.m_onSubShapeAABBOverlap)
{
}

ndNewtonMaterial::~ndNewtonMaterial()
{
}

bool ndNewtonMaterial::OnAabbOverlap(const ndBodyKinematic* const, const ndBodyKinematic* const) const
{
	if (m_onAABBOverlap)
	{
		ndAssert(0);
	}
	return true;
}

bool ndNewtonMaterial::OnAabbOverlap(const ndContact* const, ndFloat32, const ndShapeInstance&, const ndShapeInstance&) const
{
	if (m_onSubShapeAABBOverlap)
	{
		ndAssert(0);
	}
	return true;
}

void ndNewtonMaterial::OnContactCallback(const ndContact* const, ndFloat32) const
{
	if (m_onContactsProcess)
	{
		ndAssert(0);
	}
}


