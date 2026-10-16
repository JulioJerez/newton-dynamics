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
#include "NewtonUtils.h"
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

bool ndNewtonMaterial::OnAabbOverlap(const ndContact* const contact, ndFloat32 timestep, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const
{
	if (m_onSubShapeAABBOverlap && (instance0.m_subCollisionHandle || instance1.m_subCollisionHandle))
	{
		NewtonBody* const body0 = PointerCast<NewtonBody, ndBodyKinematic>(contact->GetBody0());
		NewtonBody* const body1 = PointerCast<NewtonBody, ndBodyKinematic>(contact->GetBody1());
		return m_onSubShapeAABBOverlap(PointerCast<NewtonJoint, ndContact>(contact), timestep, body0, *instance0.m_subCollisionHandle, body1, *instance1.m_subCollisionHandle, 0);
	}
	else if (m_onAABBOverlap)
	{
		return m_onAABBOverlap(PointerCast<NewtonJoint, ndContact>(contact), timestep, 0);
	}
	return true;
}

void ndNewtonMaterial::OnContactCallback(const ndContact* const contact, ndFloat32 timestep) const
{
	if (m_onContactsProcess)
	{
		m_onContactsProcess(PointerCast<NewtonJoint, ndContact>(contact), timestep, 0);
	}
}


