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
#include "Newton.h"
#include "NewtonUtils.h"
#include "newtonWorld.h"
#include "newtonMaterial.h"
#include "newtonBodyNotify.h"

class ndNewtonContact : public ndContact
{
	public:
	D_CLASS_REFLECTION(ndNewtonContact, ndContact)

		ndNewtonContact(ndNewtonWorld* const world)
		:ndContact()
		,m_world(world)
	{
	}

	ndNewtonContact(const ndNewtonContact& src)
		:ndContact(src)
		,m_world(src.m_world)
	{
	}

	ndContact* Clone() const override
	{
		return new ndNewtonContact(*this);
	}

	void JacobianDerivative(ndConstraintDescritor& desc) override
	{
		ndBodyKinematic* const kinBody0 = GetBody0();
		ndBodyKinematic* const kinBody1 = GetBody1();
		const ndNewtonBodyNotify* const notify0 = PointerCast<ndNewtonBodyNotify, ndBodyNotify>(*kinBody0->GetNotifyCallback());
		const ndNewtonBodyNotify* const notify1 = PointerCast<ndNewtonBodyNotify, ndBodyNotify>(*kinBody1->GetNotifyCallback());
		const ndNewtonMaterial* const material = PointerCast<ndNewtonMaterial, ndMaterial>(m_world->GetMaterial(notify0->m_materialGoupId, notify1->m_materialGoupId));
		ndAssert(material);
		if (material->m_onContactGeneration)
		{
			//typedef int (*NewtonOnContactGeneration) (
			// const NewtonMaterial* const material, 
			// const NewtonBody* const body0, const NewtonCollision* const collision0, 
			// const NewtonBody* const body1, const NewtonCollision* const collision1, 
			// NewtonUserContactPoint* const contactBuffer, int maxCount, int threadIndex);

			NewtonUserContactPoint contactBuffer[32];
			const NewtonMaterial* const self = PointerCast<NewtonMaterial, ndNewtonMaterial>(material);
			const NewtonBody* const body0 = PointerCast<NewtonBody, ndBodyKinematic>(kinBody0);
			const NewtonBody* const body1 = PointerCast<NewtonBody, ndBodyKinematic>(kinBody1);
			const NewtonCollision* const instance0 = PointerCast<NewtonCollision, ndShapeInstance>(&kinBody0->GetCollisionShape());
			const NewtonCollision* const instance1 = PointerCast<NewtonCollision, ndShapeInstance>(&kinBody1->GetCollisionShape());
		
			ndContactPointList& contacPointsList = m_contacPointsList;
			ndInt32 count = material->m_onContactGeneration(self, body0, instance0, body1, instance1, contactBuffer, contacPointsList.GetCount(), ndInt32(contacPointsList.GetCount()));
			while (contacPointsList.GetCount() > count)
			{
				contacPointsList.Remove(contacPointsList.GetLast());
			}
		}
		else
		{
			//ndAssert(0);
		}
		ndContact::JacobianDerivative(desc);
	}

	ndWeakPtr<ndNewtonWorld> m_world;
};


ndNewtonContactCallback::ndNewtonContactCallback(ndScene* const scene)
	:ndContactCallback(scene)
{
	m_defaultConstructor = ndSharedPtr<ndContact>(new ndNewtonContact(PointerCast<ndNewtonWorld, ndWorld>(scene->GetWorld())));
}

ndNewtonMaterial::ndNewtonMaterial()
	:ndApplicationMaterial()
	,m_usedData(nullptr)
	,m_onAABBOverlap(nullptr)
	,m_onContactsProcess(nullptr)
	,m_onContactGeneration(nullptr)
	,m_onSubShapeAABBOverlap(nullptr)
{
}

ndNewtonMaterial::ndNewtonMaterial(const ndNewtonMaterial& copy)
	:ndApplicationMaterial(copy)
	,m_usedData(copy.m_usedData)
	,m_onAABBOverlap(copy.m_onAABBOverlap)
	,m_onContactsProcess(copy.m_onContactsProcess)
	,m_onContactGeneration(copy.m_onContactGeneration)
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
