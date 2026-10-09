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

ndSpinLock ndNewtonWorld::m_globalCriticalSection;

ndNewtonWorld::ndNewtonWorld()
	:ndWorld()
	,m_userData(nullptr)
	,m_bodyMaterialGroup(1)
	,m_onPostUpdate(nullptr)
	,m_onJointSerialize(nullptr)
	,m_onJointDeserialize(nullptr)
	,m_onBodySerialize(nullptr)
	,m_onBodyDeserialize(nullptr)
	,m_onCreateContact(nullptr)
	,m_onDestroyContact(nullptr)
{
	SetSubSteps(2);
	//SetThreadCount(2);
	SelectSolver(ndSimd8Solver);
	SetContactNotify(ndSharedPtr<ndContactNotify>(new ndContactCallback));
}

ndNewtonWorld::~ndNewtonWorld()
{
}

void ndNewtonWorld::OnAddBody(ndBody* const body) const
{
	ndWorld::OnAddBody(body);
}

void ndNewtonWorld::OnRemoveBody(ndBody* const body) const
{
	ndWorld::OnRemoveBody(body);
}


void ndNewtonWorld::ClearMaterials()
{
	SetContactNotify(ndSharedPtr<ndContactNotify>(new ndContactCallback));
}

ndMaterial* ndNewtonWorld::GetMaterial(int id0, int id1) const
{
	ndContactCallback* const notify = static_cast<ndContactCallback*>(*GetContactNotify());
	if (!notify->HasMaterial(id0, id1))
	{
		if ((id0 < m_bodyMaterialGroup) && (id1 < m_bodyMaterialGroup))
		{
			ndNewtonMaterial material;
			notify->RegisterMaterial(material, id0, id1);
		}
	}
	return notify->GetMaterial(id0, id1);
}

void ndNewtonWorld::Update(ndFloat32 timestep)
{
	//ndTrace(("%f\n", timestep));
	ndWorld::Update(timestep);
}

void ndNewtonWorld::PostUpdate(ndFloat32 timestep)
{
	if (m_onPostUpdate)
	{
		ndWeakPtr<ndNewtonWorld> sharedWorld(this);
		m_onPostUpdate(reinterpret_cast<NewtonWorld*>(&sharedWorld), timestep);
	}
}