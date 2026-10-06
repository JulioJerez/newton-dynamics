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

ndNewtonWorld::ndNewtonWorld()
	:ndWorld()
	,m_userData(nullptr)
	,m_bodyMaterialGroup(1)
{
	SetContactNotify(ndSharedPtr<ndContactNotify>(new ndContactCallback));
}

ndNewtonWorld::~ndNewtonWorld()
{
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