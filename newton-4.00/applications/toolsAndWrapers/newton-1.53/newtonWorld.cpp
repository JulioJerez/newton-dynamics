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
#include "NewtonMaterialGraph.h"

ndSpinLock ndNewtonWorld::m_globalCriticalSection;

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
		ndNewtonBodyNotify* const bodyNotify0 = PointerCast<ndNewtonBodyNotify, ndBodyNotify>(*kinBody0->GetNotifyCallback());
		ndNewtonBodyNotify* const bodyNotify1 = PointerCast<ndNewtonBodyNotify, ndBodyNotify>(*kinBody1->GetNotifyCallback());
		if (bodyNotify0->m_collideWithConnectedBodies || bodyNotify1->m_collideWithConnectedBodies)
		{
			ndAssert(0);
		}

		//const ndNewtonBodyNotify* const notify0 = PointerCast<ndNewtonBodyNotify, ndBodyNotify>(*kinBody0->GetNotifyCallback());
		//const ndNewtonBodyNotify* const notify1 = PointerCast<ndNewtonBodyNotify, ndBodyNotify>(*kinBody1->GetNotifyCallback());
		//const ndNewtonMaterial* const material = PointerCast<ndNewtonMaterial, ndMaterial>(m_world->GetMaterial(notify0->m_materialGoupId, notify1->m_materialGoupId));
		//ndAssert(material);
		//if (material->m_onContactGeneration)
		//{
		//	//typedef int (*NewtonOnContactGeneration) (
		//	// const NewtonMaterial* const material, 
		//	// const NewtonBody* const body0, const NewtonCollision* const collision0, 
		//	// const NewtonBody* const body1, const NewtonCollision* const collision1, 
		//	// NewtonUserContactPoint* const contactBuffer, int maxCount, int threadIndex);
		//
		//	NewtonUserContactPoint contactBuffer[32];
		//	const NewtonMaterial* const self = PointerCast<NewtonMaterial, ndNewtonMaterial>(material);
		//	const NewtonBody* const body0 = PointerCast<NewtonBody, ndBodyKinematic>(kinBody0);
		//	const NewtonBody* const body1 = PointerCast<NewtonBody, ndBodyKinematic>(kinBody1);
		//	const NewtonCollision* const instance0 = PointerCast<NewtonCollision, ndShapeInstance>(&kinBody0->GetCollisionShape());
		//	const NewtonCollision* const instance1 = PointerCast<NewtonCollision, ndShapeInstance>(&kinBody1->GetCollisionShape());
		//
		//	ndContactPointList& contacPointsList = m_contacPointsList;
		//	ndInt32 count = material->m_onContactGeneration(self, body0, instance0, body1, instance1, contactBuffer, contacPointsList.GetCount(), ndInt32(contacPointsList.GetCount()));
		//	while (contacPointsList.GetCount() > count)
		//	{
		//		contacPointsList.Remove(contacPointsList.GetLast());
		//	}
		//}
		//else
		//{
		//	//ndAssert(0);
		//}
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
	SetSubSteps(3);
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


// Name: NewtonWorldFreezeBody 
// Remove the body from the active simulation list.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *const NewtonBody* *body - is the pointer to the body to be frozen
// 
// Return: Nothing.
//
// Remarks: When a body is created it is automatically placed in the active simulation list. As an optimization 
// for large scenes, you may use this function to put background bodies in an inactive equilibrium state.
//
// Remarks: This function tells Newton that this body does not currently need to be simulated.
// However, if the body is part of a larger configuration it may be affected indirectly by the reaction forces
// of objects that it is connected to.
//
// See also: NewtonWorldUnfreezeBody, NewtonBodySetAutoFreeze, NewtonBodyGetAutoFreeze
void NewtonWorldFreezeBody(const NewtonWorld* newtonWorld, const NewtonBody* body)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//Newton* world;
	//world = (Newton*)newtonWorld;
	//world->FreezeBody((dgBody*)body);
}

// Name: NewtonWorldUnfreezeBody 
// Add the body to the active simulation list.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *const NewtonBody* *body - is the pointer to the body to be activated
// 
// Return: Nothing
//
// Remarks: This function tells Newton to simulate the rigid body *body*.
//
// See also: NewtonWorldFreezeBody, NewtonBodySetAutoFreeze, NewtonBodyGetAutoFreeze
void NewtonWorldUnfreezeBody(const NewtonWorld* newtonWorld, const NewtonBody* bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = PointerCast<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();
	body->SetSleepState(false);
}

// Name: NewtonWorldRayCast 
// Shoot a ray from p0 to p1 and call the application callback with each ray intersection.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the world.
// *const dFloat* *p0 - pointer to an array of at least three floats containing the beginning of the ray in global space.
// *const dFloat* *p1 - pointer to an array of at least three floats containing the end of the ray in global space.
// *NewtonWorldRayFilterCallback* filter - user define function to be called for each body hit during the ray scan.
// *void* *userData - user data to be passed to the filter callback.
// *NewtonWorldRayPrefilterCallback* prefilter - user define function to be called for each body before intesection.
//
// Return: nothing
// 
// Remarks: The ray cast function will call the application with each body intersecting the line segment. 
// By writing the callback filter function in different ways the application can implement different flavors of ray casting. 
// For example an all body ray cast can be easily implemented by having the filter function always returning 1.0, and copying each 
// rigid body into an array of pointers; a closest hit ray cast can be implemented by saving the body with the smaller intersection 
// parameter and returning the parameter t; and a report the first body hit can be implemented by having the filter function returning 
// zero after the first call and saving the pointer to the rigid body.
//
// Remarks: The most common use for the ray cast function is the closest body hit, In this case it is important, for performance reasons, 
// that the filter function returns the intersection parameter. If the filter function returns a value of zero the ray cast will terminate 
// immediately.
//
// Remarks: if prefilter is not NULL, Newton will call the application right before executing the intersections between the ray and the primitive.
// if the function returns zero the Newton will not raycast the primitive. passing a NULL pointer will raycast the. 
// The application can use this implement faster or smarter filters when implementing complex logic, otherwise for normal all ray cast
// this parameter could be NULL.
//
// Remarks: The ray cast function is provided as an utility function, this means that even thought the function is very high performance 
// by function standards, it can not by batched and therefore it can not be an incremental function. For example the cost of calling 1000 
// ray cast is 1000 times the cost of calling one ray cast. This is much different than the collision system where the cost of calculating 
// collision for 1000 pairs in much, much less that the 1000 times the cost of one pair. Therefore this function must be used with care, 
// as excessive use of it can degrade performance.
void NewtonWorldRayCast(const NewtonWorld* newtonWorld, const dFloat* p0, const dFloat* p1, NewtonWorldRayFilterCallback filter, void* userData, NewtonWorldRayPrefilterCallback prefilter)
{
	TRACE_FUNCTION(__FUNCTION__);
	if (filter)
	{
		class WorldRayCast : public ndRayCastClosestHitCallback
		{
			public:
			WorldRayCast(ndNewtonWorld* const world, void* const userData, NewtonWorldRayFilterCallback filter, NewtonWorldRayPrefilterCallback prefilter)
				:ndRayCastClosestHitCallback()
				,m_userData(userData)
				,m_world(world)
				,m_filter(filter)
				,m_prefilter(prefilter)
			{
			}

			ndUnsigned32 OnRayPrecastAction(const ndBody* const body, const ndShapeInstance* const instance) override
			{
				if (m_prefilter)
				{
					return m_prefilter(PointerCast<NewtonBody, ndBody>(body), PointerCast<NewtonCollision, ndShapeInstance>(instance), m_userData);
				}
				return true;
			}

			ndFloat32 OnRayCastAction(const ndContactPoint& contact, ndFloat32 intersetParam) override
			{
				if (m_filter)
				{
					intersetParam = m_filter(PointerCast<NewtonBody, ndBody>(contact.m_body0), &contact.m_normal[0], int (contact.m_shapeId0), m_userData, intersetParam);
				}
				return intersetParam;
			}

			void* m_userData;
			ndNewtonWorld* m_world;
			NewtonWorldRayFilterCallback m_filter;
			NewtonWorldRayPrefilterCallback m_prefilter;
		};

		ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
		WorldRayCast rayCaster(world, userData, filter, prefilter);
		const ndVector pp0(p0[0], p0[1], p0[2], ndFloat32(0.0f));
		const ndVector pp1(p1[0], p1[1], p1[2], ndFloat32(0.0f));
		world->RayCast(rayCaster, pp0, pp1);
	}
}

// Name: NewtonWorldForEachBodyDo 
// Iterate thought every body in the world calling the function callback.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *NewtonBodyIterator* callback - application define callback 
//
// Return: nothing
// 
// Remarks: The application can call this function to iterate thought every body in the world. 
// the application should provide the function *NewtonBodyIterator callback* to be called by Newton for every body in the world 
//
// Remarks: this function affect severally the performance of Newton. The application should call this function only for debugging purpose
// See also: NewtonBodyForEachPolygonDo, NewtonWorldForEachBodyInAABBDo
void NewtonWorldForEachBodyDo(const NewtonWorld* newtonWorld, NewtonBodyIterator callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	const ndBodyListView& bodyList = world->GetBodyList();

	for (ndBodyListView::ndNode* node = bodyList.GetFirst()->GetNext(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBody>& body = node->GetInfo();
		callback(PointerCast<NewtonBody, ndBody>(*body));
	}
}

void NewtonWorldSetUserData(const NewtonWorld* newtonWorld, void* userData)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->m_userData = userData;
}

void* NewtonWorldGetUserData(const NewtonWorld* newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	return *world->m_userData;
}

NewtonPostUpdateCallback NewtonGetPostUpdateCallback(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	return world->m_onPostUpdate;
}

void NewtonSetPostUpdateCallback(const NewtonWorld* const newtonWorld, NewtonPostUpdateCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->m_onPostUpdate = callback;
}
