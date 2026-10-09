/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
*
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
*
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "newtonStdafx.h"
#include "Newton.h"
#include "newtonWorld.h"
#include "newtonMaterial.h"
#include "newtonBodyNotify.h"

/*!
  Trigger callback function for each joint in the world.

  @param *newtonWorld Pointer to the Newton world.
  @param callback The callback function to invoke for each joint.
  @param *userData User data to pass into the callback.

  @return nothing

  The application should provide the function *NewtonJointIterator callback* to
  be called by Newton for every joint in the world.

  Note that this function is primarily for debugging. The performance penalty
  for calling it is high.

  See also: ::NewtonWorldForEachBodyInAABBDo, ::NewtonWorldGetFirstBody
*/
void NewtonWorldForEachJointDo(const NewtonWorld* const newtonWorld, NewtonJointIterator callback, void* const userData)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = ObjectFromHandle<ndNewtonWorld, NewtonWorld>(newtonWorld);
	const ndJointList& jointList = world->GetJointList();

	//dgTree<dgConstraint*, dgConstraint*> jointMap(world->dgWorld::GetAllocator());
	//for (dgBodyMasterList::dgListNode* node = masterList.GetFirst()->GetNext(); node; node = node->GetNext()) {
	//	dgBodyMasterListRow& row = node->GetInfo();
	//	for (dgBodyMasterListRow::dgListNode* jointNode = row.GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
	//		const dgBodyMasterListCell& cell = jointNode->GetInfo();
	//		if (cell.m_joint->GetId() != dgConstraint::m_contactConstraint) {
	//			if (!jointMap.Find(cell.m_joint)) {
	//				jointMap.Insert(cell.m_joint, cell.m_joint);
	//				callback((const NewtonJoint*)cell.m_joint, userData);
	//			}
	//		}
	//	}
	//}
	for (ndJointList::ndNode* node = jointList.GetFirst()->GetNext(); node; node = node->GetNext())
	{
		ndSharedPtr<ndJointBilateralConstraint>& joint = node->GetInfo();
		callback(reinterpret_cast<NewtonJoint*>(&joint), userData);
	}
}

void NewtonWorldForEachBodyDo(const NewtonWorld* const newtonWorld, NewtonBodyIterator callback, void* const userData)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = ObjectFromHandle<ndNewtonWorld, NewtonWorld>(newtonWorld);
	const ndBodyListView& jointList = world->GetBodyList();

	for (ndBodyListView::ndNode* node = jointList.GetFirst()->GetNext(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBody>& body = node->GetInfo();
		callback(reinterpret_cast<NewtonBody*>(&body), userData);
	}
}


/*!
  Shoot ray from point p0 to p1 and trigger callback for each body on that line.

  @param *newtonWorld Pointer to the Newton world.
  @param *p0 - pointer to an array of at least three floats containing the beginning of the ray in global space.
  @param *p1 - pointer to an array of at least three floats containing the end of the ray in global space.
  @param filter Callback function for each hit during the ray scan.
  @param *userData user data to pass along to the filter callback.
  @param prefilter user defined function to be called for each body before intersection.
  @param threadIndex Index of thread that called this function (zero if called form outsize a newton update).

  @return nothing

  The ray cast function will trigger the callback for every intersection between
  the line segment (from p0 to p1) and a body in the world.

  By writing the callback filter function in different ways the application can
  implement different flavors of ray casting. For example an all body ray cast
  can be easily implemented by having the filter function always returning 1.0,
  and copying each rigid body into an array of pointers; a closest hit ray cast
  can be implemented by saving the body with the smaller intersection parameter
  and returning the parameter t; and a report the first body hit can be
  implemented by having the filter function returning zero after the first call
  and saving the pointer to the rigid body.

  The most common use for the ray cast function is the closest body hit, In this
  case it is important, for performance reasons, that the filter function
  returns the intersection parameter. If the filter function returns a value of
  zero the ray cast will terminate immediately.

  if prefilter is not NULL, Newton will call the application right before
  executing the intersections between the ray and the primitive. if the function
  returns zero the Newton will not ray cast the primitive. passing a NULL
  pointer will ray cast the. The application can use this implement faster or
  smarter filters when implementing complex logic, otherwise for normal all ray
  cast this parameter could be NULL.

  The ray cast function is provided as an utility function, this means that even
  thought the function is very high performance by function standards, it can
  not by batched and therefore it can not be an incremental function. For
  example the cost of calling 1000 ray cast is 1000 times the cost of calling
  one ray cast. This is much different than the collision system where the cost
  of calculating collision for 1000 pairs in much, much less that the 1000 times
  the cost of one pair. Therefore this function must be used with care, as
  excessive use of it can degrade performance.

  See also: ::NewtonWorldConvexCast
*/
void NewtonWorldRayCast(const NewtonWorld* const newtonWorld, const dFloat* const p0, const dFloat* const p1, NewtonWorldRayFilterCallback filter, void* const userData, NewtonWorldRayPrefilterCallback prefilter, int threadIndex)
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

			ndUnsigned32 OnRayPrecastAction(const ndBody* const body, const ndShapeInstance* const instancePtr) override
			{
				if (m_prefilter)
				{
					ndSharedPtr<ndBody> sharedBody(m_world->GetBody(const_cast<ndBody*>(reinterpret_cast<const ndBody*>(body))));
					ndShapeInstance* const instance = const_cast<ndShapeInstance*>(instancePtr);
					return m_prefilter(reinterpret_cast<NewtonBody*>(&sharedBody), reinterpret_cast<const NewtonCollision*>(instance), m_userData);
				}
				return true;
			}

			ndFloat32 OnRayCastAction(const ndContactPoint& contact, ndFloat32 intersetParam) override
			{
				if (m_filter)
				{
					//typedef dFloat(*NewtonWorldRayFilterCallback)(
					// const NewtonBody* const body, 
					// const NewtonCollision* const shapeHit, 
					// const dFloat* const hitContact, 
					// const dFloat* const hitNormal, 
					// dLong collisionID, 
					// void* const userData, 
					// dFloat intersectParam);
					// 
					//ndWeakPtr<const ndShapeInstance> sharedInstance(contact.m_shapeInstance0);
					ndShapeInstance* const instance = const_cast<ndShapeInstance*>(contact.m_shapeInstance0);
					ndSharedPtr<ndBody> sharedBody(m_world->GetBody(const_cast<ndBody*>(reinterpret_cast<const ndBody*>(contact.m_body0))));
					intersetParam = m_filter(reinterpret_cast<NewtonBody*>(&sharedBody), reinterpret_cast<const NewtonCollision*>(instance), &intersetParam, &contact.m_normal[0], contact.m_shapeId0, m_userData, intersetParam);
				}
				return intersetParam;
			}

			void* m_userData;
			ndNewtonWorld* m_world;
			NewtonWorldRayFilterCallback m_filter;
			NewtonWorldRayPrefilterCallback m_prefilter;
		};

		ndNewtonWorld* const world = ObjectFromHandle<ndNewtonWorld, NewtonWorld>(newtonWorld);
		WorldRayCast rayCaster(world, userData, filter, prefilter);
		const ndVector pp0(p0[0], p0[1], p0[2], ndFloat32(0.0f));
		const ndVector pp1(p1[0], p1[1], p1[2], ndFloat32(0.0f));
		world->RayCast(rayCaster, pp0, pp1);
	}
}


/*!
  Trigger a callback for every body that intersects the specified AABB.

  @param *newtonWorld Pointer to the Newton world.
  @param *p0 - pointer to an array of at least three floats to hold minimum value for the AABB.
  @param *p1 - pointer to an array of at least three floats to hold maximum value for the AABB.
  @param callback application defined callback
  @param *userData pointer to the user defined user data value.

  @return nothing

  The application should provide the function *NewtonBodyIterator callback* to
  be called by Newton for every body in the world.

  For small AABB volumes this function is much more inefficients (fixme: more or
  less efficient?) than NewtonWorldGetFirstBody. However, if the AABB contains
  the majority of objects in the scene, the overhead of scanning the internal
  Broadphase collision plus the AABB test make this function more expensive.

  See also: ::NewtonWorldGetFirstBody
*/
void NewtonWorldForEachBodyInAABBDo(const NewtonWorld* const newtonWorld, const dFloat* const p0, const dFloat* const p1, NewtonBodyIterator callback, void* const userData)
{
	TRACE_FUNCTION(__FUNCTION__);

	ndNewtonWorld* const world = ObjectFromHandle<ndNewtonWorld, NewtonWorld>(newtonWorld);
	const ndVector minBox(ndMin(p0[0], p1[0]), ndMin(p0[1], p1[1]), ndMin(p0[2], p1[2]), ndFloat32(0.0f));
	const ndVector maxBox(ndMax(p0[0], p1[0]), ndMax(p0[1], p1[1]), ndMax(p0[2], p1[2]), ndFloat32(0.0f));

	class WorldBodiesInAabbNotify : public ndBodiesInAabbNotify
	{
		public:
		WorldBodiesInAabbNotify(ndNewtonWorld* const world, NewtonBodyIterator callback, void* const userData)
			:ndBodiesInAabbNotify()
			,m_userData(userData)
			,m_world (world)
			,m_callback(callback)
		{
		}

		virtual void OnOverlap(const ndBody* const body) override
		{
			if (m_callback)
			{
				ndSharedPtr<ndBody> sharedBody(m_world->GetBody(const_cast<ndBody*>(reinterpret_cast<const ndBody*>(body))));
				m_callback(reinterpret_cast<NewtonBody*>(&sharedBody), m_userData);
			}
		}

		void* const m_userData;
		ndNewtonWorld* m_world;
		NewtonBodyIterator m_callback;
	};

	WorldBodiesInAabbNotify notify(world, callback, userData);
	world->BodiesInAabb(notify, minBox, maxBox);
}


/*!
  cast a simple convex shape along the ray that goes for the matrix position to the destination and get the firsts contacts of collision.

  @param *newtonWorld Pointer to the Newton world.
  @param *matrix pointer to an array of at least three floats containing the beginning and orienetaion of the shape in global space.
  @param *target pointer to an array of at least three floats containing the end of the ray in global space.
  @param shape collision shap[e use to cat the ray.
  @param param pointe to a variable the will contart the time to closet aproah to the collision.
  @param *userData user data to be passed to the prefilter callback.
  @param prefilter user define function to be called for each body before intersection.
  @param *info pointer to an array of contacts at the point of intesections.
  @param maxContactsCount maximun number of contacts to be conclaculated, the variable sould be initialized to the capaciaty of *info*
  @param threadIndex thread index from whe thsi function is called, zero if call form outsize a newton update

  @return the number of contact at the intesection point (a value equal o lower than maxContactsCount.
  variable *hitParam* will be set the uintesation parameter an the momen of impact.

  passing and value of NULL in *info* an dzero in maxContactsCount will turn thos function into a spcial Ray cast
  where the function will only calculate the *hitParam* at the momenet of contacts. tshi si one of the most effiecnet way to use thsio function.

  these function is similar to *NewtonWorldRayCast* but instead of casting a point it cast a simple convex shape along a ray for maoprix.m_poit
  to target position. the shape is global orientation and position is set to matrix and then is swept along the segment to target and it will stop at the very first intersession contact.

  for case where the application need to cast solid short to medium rays, it is better to use this function instead of casting and array of parallel rays segments.
  examples of these are: implementation of ray cast cars with cylindrical tires, foot placement of character controllers, kinematic motion of objects, user controlled continuous collision, etc.
  this function may not be as efficient as sampling ray for long segment, for these cases try using parallel ray cast.

  The most common use for the ray cast function is the closest body hit, In this case it is important, for performance reasons,
  that the filter function returns the intersection parameter. If the filter function returns a value of zero the ray cast will terminate
  immediately.

  if prefilter is not NULL, Newton will call the application right before executing the intersections between the ray and the primitive.
  if the function returns zero the Newton will not ray cast the primitive.
  The application can use this callback to implement faster or smarter filters when implementing complex logic, otherwise for normal all ray cast
  this parameter could be NULL.

  See also: ::NewtonWorldRayCast
*/
int NewtonWorldConvexCast(const NewtonWorld* const newtonWorld, const dFloat* const matrix, const dFloat* const target, const NewtonCollision* const shape,
	dFloat* const param, void* const userData, NewtonWorldRayPrefilterCallback prefilter, NewtonWorldConvexCastReturnInfo* const info,
	int maxContactsCount, int threadIndex)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgVector destination(target[0], target[1], target[2], dgFloat32(0.0f));
	//Newton* const world = (Newton*)newtonWorld;
	//return world->GetBroadPhase()->ConvexCast((dgCollisionInstance*)shape, dgMatrix(matrix), destination, param, (OnRayPrecastAction)prefilter, userData, (dgConvexCastReturnInfo*)info, maxContactsCount, threadIndex);
	ndAssert(0);
	return 0;
}

int NewtonWorldCollide(const NewtonWorld* const newtonWorld, const dFloat* const matrix, const NewtonCollision* const shape, void* const userData,
	NewtonWorldRayPrefilterCallback prefilter, NewtonWorldConvexCastReturnInfo* const info, int maxContactsCount, int threadIndex)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return world->GetBroadPhase()->Collide((dgCollisionInstance*)shape, dgMatrix(matrix), (OnRayPrecastAction)prefilter, userData, (dgConvexCastReturnInfo*)info, maxContactsCount, threadIndex);
	ndAssert(0);
	return 0;
}

NewtonJoint* NewtonWorldFindJoint(const NewtonBody* const body0, const NewtonBody* const body1)
{
	TRACE_FUNCTION(__FUNCTION__);
	//for (NewtonJoint* joint = NewtonBodyGetFirstJoint(body0); joint; joint = NewtonBodyGetNextJoint(body0, joint)) {
	//	if (((body0 == NewtonJointGetBody0(joint)) && (body1 == NewtonJointGetBody1(joint))) ||
	//		((body1 == NewtonJointGetBody0(joint)) && (body0 == NewtonJointGetBody1(joint)))) {
	//		return joint;
	//	}
	//}
	//return NULL;
	ndAssert(0);
	return 0;
}

/*!
  Get the first body in the body in the world body list.

  @param *newtonWorld Pointer to the Newton world.

  @return nothing

  The application can call this function to iterate thought every body in the world.

  The application call this function for debugging purpose
  See also: ::NewtonWorldGetNextBody, ::NewtonWorldForEachBodyInAABBDo, ::NewtonWorldForEachJointDo
*/
NewtonBody* NewtonWorldGetFirstBody(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = ObjectFromHandle<ndNewtonWorld, NewtonWorld>(newtonWorld);
	const ndBodyListView& bodyList = world->GetBodyList();
	if (bodyList.GetCount())
	{
		ndSharedPtr<ndBody>& body = bodyList.GetFirst()->GetInfo();
		return reinterpret_cast<NewtonBody*>(&body);
	}
	return nullptr;
}


/*!
  Get the first body in the general body.

  @param *newtonWorld Pointer to the Newton world.
  @param curBody fixme

  @return nothing

  The application can call this function to iterate through every body in the world.

  The application call this function for debugging purpose

  See also: ::NewtonWorldGetFirstBody, ::NewtonWorldForEachBodyInAABBDo, ::NewtonWorldForEachJointDo
*/
NewtonBody* NewtonWorldGetNextBody(const NewtonWorld* const newtonWorld, const NewtonBody* const curBody)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)curBody;
	//
	//dgBodyMasterList::dgListNode* const node = body->GetMasterList()->GetNext();
	//if (node) {
	//	return (NewtonBody*)node->GetInfo().GetBody();
	//}
	//else {
	//	return NULL;
	//}
	ndAssert(0);
	return 0;
}

/*!
  Get the first Material pair from the material array.

  @param *newtonWorld Pointer to the Newton world.

  @return the first material.

  See also: ::NewtonWorldGetNextMaterial
*/
NewtonMaterial* NewtonWorldGetFirstMaterial(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return (NewtonMaterial*)world->GetFirstMaterial();
	ndAssert(0);
	return 0;
}

/*!
  Get the next Material pair from the material array.

  @param *newtonWorld Pointer to the Newton world.
  @param *material corrent material

  @return next material in material array or NULL if material is the last material in the list.

  See also: ::NewtonWorldGetFirstMaterial
*/
NewtonMaterial* NewtonWorldGetNextMaterial(const NewtonWorld* const newtonWorld, const NewtonMaterial* const material)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//
	//return (NewtonMaterial*)world->GetNextMaterial((dgContactMaterial*)material);
	ndAssert(0);
	return 0;
}
