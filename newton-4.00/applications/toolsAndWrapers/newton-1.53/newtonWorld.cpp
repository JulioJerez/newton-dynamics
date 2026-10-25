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
#include "NewtonBody.h"
#include "NewtonUtils.h"
#include "newtonWorld.h"
//#include "newtonMaterial.h"

ndSpinLock ndNewtonWorld::m_globalCriticalSection;

class ndNewtonMaterial : public ndApplicationMaterial
{
	public:
	typedef int (*NewtonOnAABBOverlap) (const NewtonJoint* const contact, ndFloat32 timestep, int threadIndex);
	typedef void (*NewtonContactsProcess) (const NewtonJoint* const contact, ndFloat32 timestep, int threadIndex);
	typedef int (*NewtonOnCompoundSubCollisionAABBOverlap) (const NewtonJoint* const contact, ndFloat32 timestep, const NewtonBody* const body0, const void* const collisionNode0, const NewtonBody* const body1, const void* const collisionNode1, int threadIndex);
	typedef int (*NewtonOnContactGeneration) (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonCollision* const collision0, const NewtonBody* const body1, const NewtonCollision* const collision1, NewtonUserContactPoint* const contactBuffer, int maxCount, int threadIndex);

	ndNewtonMaterial()
		:ndApplicationMaterial()
		,m_usedData(nullptr)
		,m_onAABBOverlap(nullptr)
		,m_onContactsProcess(nullptr)
		,m_onContactGeneration(nullptr)
		,m_onSubShapeAABBOverlap(nullptr)
	{
	}

	ndNewtonMaterial(const ndNewtonMaterial& copy)
		:ndApplicationMaterial(copy)
		,m_usedData(copy.m_usedData)
		,m_onAABBOverlap(copy.m_onAABBOverlap)
		,m_onContactsProcess(copy.m_onContactsProcess)
		,m_onContactGeneration(copy.m_onContactGeneration)
		,m_onSubShapeAABBOverlap(copy.m_onSubShapeAABBOverlap)
	{
	}

	virtual ~ndNewtonMaterial() override
	{
	}

	virtual ndApplicationMaterial* Clone() const override
	{
		return new ndNewtonMaterial(*this);
	}

	virtual void OnContactCallback(const ndContact* const contact, ndFloat32 timestep) const override
	{
		if (m_onContactsProcess)
		{
			m_onContactsProcess(PointerCast<NewtonJoint, ndContact>(contact), timestep, 0);
		}
	}

	virtual bool OnAabbOverlap(const ndBodyKinematic* const bodyPtr0, const ndBodyKinematic* const bodyPtr1) const override
	{
		if (m_onAABBOverlap)
		{
			const NewtonBody* const body0 = PointerCast<NewtonBody, ndBodyKinematic>(bodyPtr0);
			const NewtonBody* const body1 = PointerCast<NewtonBody, ndBodyKinematic>(bodyPtr1);
			const ndContact* const contact = bodyPtr0->GetContactMap().FindContact(bodyPtr0, bodyPtr1);
			if (!contact)
			{
				return true;
			}
			return m_onAABBOverlap(PointerCast<NewtonJoint, ndContact>(contact), ndFloat32(0.0f), 0);
		}
		return true;
	}

	virtual bool OnAabbOverlap(const ndContact* const contact, ndFloat32 timestep, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const override
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

	ndWeakPtr<void> m_usedData;
	NewtonOnAABBOverlap m_onAABBOverlap;
	NewtonContactsProcess m_onContactsProcess;
	NewtonOnContactGeneration m_onContactGeneration;
	NewtonOnCompoundSubCollisionAABBOverlap m_onSubShapeAABBOverlap;
};


class ndNewtonContact : public ndContact
{
public:
	D_CLASS_REFLECTION(ndNewtonContact, ndContact)

		ndNewtonContact(ndNewtonWorld* const world)
		:ndContact()
		, m_world(world)
	{
	}

	ndNewtonContact(const ndNewtonContact& src)
		:ndContact(src)
		, m_world(src.m_world)
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

class ndNewtonContactCallback : public ndContactCallback
{
	public:
	ndNewtonContactCallback(ndScene* const scene)
		:ndContactCallback(scene)
	{
		m_defaultConstructor = ndSharedPtr<ndContact>(new ndNewtonContact(PointerCast<ndNewtonWorld, ndWorld>(scene->GetWorld())));
	}
};

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
	SetContactCallback();
}

ndNewtonWorld::~ndNewtonWorld()
{
}

void ndNewtonWorld::SetContactCallback()
{
	ndContactCallback* const callback = new ndNewtonContactCallback(GetScene());
	ndSharedPtr<ndApplicationMaterial> defaultMaterial(new ndNewtonMaterial);
	callback->SetDefualtMaterial(defaultMaterial);
	SetContactNotify(ndSharedPtr<ndContactNotify>(callback));
}

void ndNewtonWorld::OnAddBody(ndBody* const body) const
{
	ndWorld::OnAddBody(body);
}

void ndNewtonWorld::OnRemoveBody(ndBody* const body) const
{
	ndWorld::OnRemoveBody(body);
}

void ndNewtonWorld::OnAddJoint(ndJointBilateralConstraint* const joint) const
{
	ndWorld::OnAddJoint(joint);
}

void ndNewtonWorld::OnRemoveJoint(ndJointBilateralConstraint* const joint) const
{
	ndWorld::OnRemoveJoint(joint);
}

//void ndNewtonWorld::ClearMaterials()
//{
//	SetContactCallback();
//}

void ndNewtonWorld::OnInitSkeleton(ndSkeletonContainer* const skeleton) const 
{
	ndWorld::OnInitSkeleton(skeleton);
	skeleton->SetAutoRegualrization(false, ndFloat32(2.0f) * D_DEFAULT_REGULARIZER);
}

ndMaterial* ndNewtonWorld::GetMaterial(int id0, int id1) const
{
	ndContactCallback* const notify = static_cast<ndContactCallback*>(*GetContactNotify());
	if (!notify->HasMaterial(id0, id1))
	{
		if ((id0 < m_bodyMaterialGroup) && (id1 < m_bodyMaterialGroup))
		{
			if ((id0 != 0) && (id1 != 0))
			{
				ndNewtonMaterial material;
				notify->RegisterMaterial(material, id0, id1);
			}
		}
	}
	return notify->GetMaterial(id0, id1);
}

void ndNewtonWorld::PostUpdate(ndFloat32 timestep)
{
	if (m_onPostUpdate)
	{
		m_onPostUpdate(PointerCast<NewtonWorld, ndNewtonWorld>(this), timestep);
	}
}

void ndNewtonWorld::Update(ndFloat32 timestep)
{
	//ndTrace(("%f\n", timestep));
	ndWorld::Update(timestep);
}


/*!
  Create an instance of the Newton world.

  @return Pointer to new Newton world.

  This function must be called before any of the other API functions.

  See also: ::NewtonDestroy, ::NewtonDestroyAllBodies
*/
NewtonWorld* NewtonCreate(NewtonAllocMemory malloc, NewtonFreeMemory mfree)
{
	TRACE_FUNCTION(__FUNCTION__);

	NewtonSetMemorySystem(malloc, mfree);
	ndNewtonWorld* const world = new ndNewtonWorld();
	return PointerCast<NewtonWorld, ndNewtonWorld>(world);
}

/*!
  Destroy an instance of the Newton world.

  @param *newtonWorld Pointer to the Newton world.
  @return Nothing.

  This function will destroy the entire Newton world.

  See also: ::NewtonCreate, ::NewtonDestroyAllBodies
*/
void NewtonDestroy(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	delete world;
}

/*!
  Return the current library version number.

  @return version number as an integer, eg 314.

  The version number is a three-digit integer.

  First digit:  major version (interface changes among other things)
  Second digit: major patch number (new features, and bug fixes)
  Third Digit:  minor bug fixed patch.
*/
int NewtonWorldGetVersion(const NewtonWorld*)
{
	TRACE_FUNCTION(__FUNCTION__);
	return NEWTON_MAJOR_VERSION * 100 + NEWTON_MINOR_VERSION;
}


// Name: NewtonUpdate 
// Advance the simulation by an amount of time.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *dFloat* timestep - time step in seconds
// 
// Return: Nothing
//
// Remarks: This function will advance the simulation by the amount of time specified by *timestep*. The Newton Engine does 
// not perform sub-steps, and does not need tuning parameters. It is the responsibility of the application to 
// ensure that *timestep* is small enough to guarantee physics stability. 
void NewtonUpdate(const NewtonWorld* newtonWorld, dFloat timestep)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);

	world->Update(timestep);
	world->Sync();
}
