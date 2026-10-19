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
#include "NewtonUtils.h"
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
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
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
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	const ndBodyListView& jointList = world->GetBodyList();

	for (ndBodyListView::ndNode* node = jointList.GetFirst()->GetNext(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBody>& body = node->GetInfo();
		callback(reinterpret_cast<NewtonBody*>(&body), userData);
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
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
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
				m_callback(PointerCast<NewtonBody, ndBody>(body), m_userData);
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
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	const ndBodyListView& bodyList = world->GetBodyList();
	if (bodyList.GetCount())
	{
		ndSharedPtr<ndBody>& body = bodyList.GetFirst()->GetInfo();
		return reinterpret_cast<NewtonBody*>(*body);
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
NewtonBody* NewtonWorldGetNextBody(const NewtonWorld* const, const NewtonBody* const curBody)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const kinBody = PointerCast<ndBody, NewtonBody>(curBody)->GetAsBodyKinematic();
	ndBodyListView::ndNode* const node = kinBody->GetSceneNode();
	ndBodyListView::ndNode* const nextNode = node ? node->GetNext() : nullptr;
	return nextNode ? reinterpret_cast<NewtonBody*>(*nextNode->GetInfo()) : nullptr;
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


/*!
  Iterate thought polygon of the collision geometry of a body calling the function callback.

  @param *collisionPtr is the pointer to the collision objects.
  @param *matrixPtr is the pointer to the collision objects.
  @param callback application define callback
  @param *userDataPtr pointer to the user defined user data value.

  @return nothing

  This function used to be a member of the rigid body, but to making it a member of the collision object provides better
  low lever display capabilities. The application can still call this function to show the collision of a rigid body by
  getting the collision and the transformation matrix from the rigid, and then calling this functions.

  This function can be called by the application in order to show the collision geometry. The application should provide a pointer to the function *NewtonCollisionIterator*,
  Newton will convert the collision geometry into a polygonal mesh, and will call *callback* for every polygon of the mesh

  this function affect severely the performance of Newton. The application should call this function only for debugging purpose

  This function will ignore user define collision mesh
  See also: ::NewtonWorldGetFirstBody, ::NewtonWorldForEachBodyInAABBDo
*/
void NewtonCollisionForEachPolygonDo(const NewtonCollision* const collision, const dFloat* const matrixPtr, NewtonCollisionIterator callback, void* const userData)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);
	ndMatrix matrix(GetMatrix(matrixPtr));

	class ndDrawShape : public ndShapeDebugNotify
	{
		public:
		ndDrawShape(NewtonCollisionIterator callback, void* const userData)
			:ndShapeDebugNotify()
			,m_userData(userData)
			,m_callback(callback)

		{
		}

		virtual void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceVertex, const ndEdgeType* const) override
		{
			ndFixSizeArray<ndFloat32, 256> face;
			for (ndInt32 i = 0; i < vertexCount; ++i)
			{
				face.PushBack(faceVertex[i].m_x);
				face.PushBack(faceVertex[i].m_y);
				face.PushBack(faceVertex[i].m_z);
			}
			m_callback(m_userData, vertexCount, &face[0], 0);
		}

		void* const m_userData;
		NewtonCollisionIterator m_callback;
	};
	ndDrawShape debugShape(callback, userData);
	instance->DebugShape(matrix, debugShape);
}

/*!
  Calculate the most extreme point of a convex collision shape along the given direction.

  @param *collisionPtr pointer to the collision object.
  @param *dir pointer to an array of at least three floats representing the search direction.
  @param *vertex pointer to an array of at least three floats to hold the collision most extreme vertex along the search direction.

  @return nothing.

  the search direction must be in the space of the collision shape.

  See also: ::NewtonCollisionRayCast, ::NewtonCollisionClosestPoint, ::NewtonCollisionPointDistance
*/
void NewtonCollisionSupportVertex(const NewtonCollision* const collision, const dFloat* const dir, dFloat* const vertex)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);

	const ndMatrix& matrix = instance->GetLocalMatrix();

	ndVector searchDir(matrix.UnrotateVector(ndVector(dir[0], dir[1], dir[2], ndFloat32(0.0f))));
	searchDir = searchDir.Normalize();
	ndVector vertexOut(matrix.TransformVector(instance->SupportVertex(searchDir)));

	vertex[0] = vertexOut[0];
	vertex[1] = vertexOut[1];
	vertex[2] = vertexOut[2];
}

/*!
  Calculate the closest points between two disjoint convex collision primitive.

  @param *newtonWorld Pointer to the Newton world.
  @param *collisionA pointer to collision primitive A.
  @param *matrixA pointer to an array of 16 floats containing the offset matrix of collision primitiveA.
  @param *collisionB pointer to collision primitive B.
  @param *matrixB pointer to an array of 16 floats containing the offset matrix of collision primitiveB.
  @param *contactA pointer to and array of a least 3 floats to contain the closest point to collisionA.
  @param *contactB pointer to and array of a least 3 floats to contain the closest point to collisionB.
  @param *normalAB pointer to and array of a least 3 floats to contain the separating vector normal.
  @param  threadIndex -Thread index form where the call is made from, zeor otherwize

  @return one if the tow bodies are disjoint and he closest point could be found,
  zero if the two collision primitives are intersecting.

  This function can be used as a low-level building block for a stand-alone collision system.
  Applications that have already there own physics system, and only want and quick and fast collision solution,
  can use Newton advanced collision engine as the low level collision detection part.
  To do this the application only needs to initialize Newton, create the collision primitives at application discretion,
  and just call this function when the objects are in close proximity. Applications using Newton as a collision system
  only, are responsible for implementing their own broad phase collision determination, based on any high level tree structure.
  Also the application should implement their own trivial aabb test, before calling this function .

  the current implementation of this function does not work on collision trees, or user define collision.

  See also: ::NewtonCollisionCollideContinue, ::NewtonCollisionPointDistance, ::NewtonCollisionCollide, ::NewtonCollisionRayCast, ::NewtonCollisionCalculateAABB
*/
int NewtonCollisionClosestPoint(const NewtonWorld* const newtonWorld,
	const NewtonCollision* const collisionA, const dFloat* const matrixPtrA,
	const NewtonCollision* const collisionB, const dFloat* const matrixPtrB,
	dFloat* const contactA, dFloat* const contactB, dFloat* const normalAB, int threadIndex)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	const ndShapeInstance* const shapeA = PointerCast<ndShapeInstance, NewtonCollision>(collisionA);
	const ndShapeInstance* const shapeB = PointerCast<ndShapeInstance, NewtonCollision>(collisionB);

	ndMatrix matrixA(GetMatrix(matrixPtrA));
	ndMatrix matrixB(GetMatrix(matrixPtrB));
	ndContactSolver solver;
	
	ndContactNotify notification(world->GetScene());
	solver.CalculateClosestPoints(shapeA, matrixA, shapeB, matrixB, &notification);

	const ndVector pointA(solver.GetClosetVector0());
	const ndVector pointB(solver.GetClosetVector1());
	const ndVector normal(solver.GetSeparatingVector());

	for (ndInt32 i = 0; i < 3; ++i)
	{
		contactA[i] = pointA[i];
		contactB[i] = pointB[i];
		normalAB[i] = normal[i];
	}
	return 1;
}

/*!
  Calculate contact points between two collision primitive.

  @param *newtonWorld Pointer to the Newton world.
  @param maxSize size of maximum number of elements in contacts, normals, and penetration.
  @param *collisionA pointer to collision primitive A.
  @param *matrixA pointer to an array of 16 floats containing the offset matrix of collision primitiveA.
  @param *collisionB pointer to collision primitive B.
  @param *matrixB pointer to an array of 16 floats containing the offset matrix of collision primitiveB.
  @param *contacts pointer to and array of a least 3 times maxSize floats to contain the collision contact points.
  @param *normals pointer to and array of a least 3 times maxSize floats to contain the collision contact normals.
  @param *penetration pointer to and array of a least maxSize floats to contain the collision penetration at each contact.
  @param attributeA fixme
  @param attributeB fixme
  @param threadIndex Thread index form where the call is made from, zeor otherwize

  @return the number of contact points.

  This function can be used as a low-level building block for a stand-alone collision system.
  Applications that have already there own physics system, and only want and quick and fast collision solution,
  can use Newton advanced collision engine as the low level collision detection part.
  To do this the application only needs to initialize Newton, create the collision primitives at application discretion,
  and just call this function when the objects are in close proximity. Applications using Newton as a collision system
  only, are responsible for implementing their own broad phase collision determination, based on any high level tree structure.
  Also the application should implement their own trivial aabb test, before calling this function .

  See also: ::NewtonCollisionCollideContinue, ::NewtonCollisionClosestPoint, ::NewtonCollisionPointDistance, ::NewtonCollisionRayCast, ::NewtonCollisionCalculateAABB
*/
int NewtonCollisionCollide(const NewtonWorld* const newtonWorld, int maxSize,
	const NewtonCollision* const collisionA, const dFloat* const matrixPtrA,
	const NewtonCollision* const collisionB, const dFloat* const matrixPtrB,
	dFloat* const contacts, dFloat* const normals, dFloat* const penetration,
	dLong* const attributeA, dLong* const attributeB, int threadIndex)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	const ndShapeInstance* const shapeA = PointerCast<ndShapeInstance, NewtonCollision>(collisionA);
	const ndShapeInstance* const shapeB = PointerCast<ndShapeInstance, NewtonCollision>(collisionB);

	ndMatrix matrixA(GetMatrix(matrixPtrA));
	ndMatrix matrixB(GetMatrix(matrixPtrB));
	ndContactSolver solver;

	ndVector veloc(ndVector::m_zero);
	ndFixSizeArray<ndContactPoint, 16> contactPoints;
	ndContactNotify notification(world->GetScene());
	solver.CalculateContacts(shapeA, matrixA, veloc, shapeB, matrixB, veloc, contactPoints, &notification);
	const ndInt32 contactCount = ndMin(contactPoints.GetCount(), maxSize);
	for (ndInt32 i = 0; i < contactCount; ++i)
	{
		contacts[i * 3 + 0] = contactPoints[i].m_point.m_x;
		contacts[i * 3 + 1] = contactPoints[i].m_point.m_y;
		contacts[i * 3 + 2] = contactPoints[i].m_point.m_z;
		normals[i * 3 + 0] = contactPoints[i].m_normal.m_x;
		normals[i * 3 + 1] = contactPoints[i].m_normal.m_y;
		normals[i * 3 + 2] = contactPoints[i].m_normal.m_z;
		penetration[i] = contactPoints[i].m_penetration;
		attributeA[i] = contactPoints[i].m_shapeId0;
		attributeB[i] = contactPoints[i].m_shapeId1;
	}
	return contactCount;
}

int NewtonCollisionIntersectionTest(const NewtonWorld* const newtonWorld, const NewtonCollision* const collisionA, const dFloat* const matrixPtrA, const NewtonCollision* const collisionB, const dFloat* const matrixPtrB, int threadIndex)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	const ndShapeInstance* const shapeA = PointerCast<ndShapeInstance, NewtonCollision>(collisionA);
	const ndShapeInstance* const shapeB = PointerCast<ndShapeInstance, NewtonCollision>(collisionB);

	ndMatrix matrixA(GetMatrix(matrixPtrA));
	ndMatrix matrixB(GetMatrix(matrixPtrB));
	ndContactSolver solver;

	ndVector veloc(ndVector::m_zero);
	ndFixSizeArray<ndContactPoint, 16> contactPoints;
	ndContactNotify notification(world->GetScene());
	return solver.CollisionTest(shapeA, matrixA, veloc, shapeB, matrixB, veloc, &notification) ? 1 : 0;
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
					intersetParam = m_filter(PointerCast<NewtonBody, ndBody>(contact.m_body0), PointerCast<NewtonCollision, ndShapeInstance>(contact.m_shapeInstance0), &intersetParam, &contact.m_normal[0], contact.m_shapeId0, m_userData, intersetParam);
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
int NewtonWorldConvexCast(const NewtonWorld* const newtonWorld, const dFloat* const matrixPtr, const dFloat* const target, const NewtonCollision* const shape,
	dFloat* const param, void* const userData, NewtonWorldRayPrefilterCallback prefilter, NewtonWorldConvexCastReturnInfo* const info,
	int maxContactsCount, int threadIndex)
{
	TRACE_FUNCTION(__FUNCTION__);
	class ndConvexCast : public ndConvexCastNotify
	{
		public:
		typedef unsigned (*NewtonWorldRayPrefilterCallback)(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData);

		ndConvexCast(ndNewtonWorld* const world, const ndMatrix& origin, const ndVector& dst, const ndShapeInstance& shape,
			NewtonWorldRayPrefilterCallback prefilter, void* const userData)
			:ndConvexCastNotify()
			,m_world(world)
			,m_onFilter(prefilter)
			,m_userData(userData)
		{
			world->ConvexCast(*this, shape, origin, dst);
		}

		virtual ndUnsigned32 OnRayPrecastAction(const ndBody* const bodyPtr, const ndShapeInstance* const collision)
		{
			if (m_onFilter)
			{
				NewtonBody* const body = PointerCast<NewtonBody, ndBody>(bodyPtr);
				NewtonCollision* const instance = PointerCast<NewtonCollision, ndShapeInstance>(collision);
				return m_onFilter(body, instance, *m_userData);
			}
			return 1;
		}

		ndWeakPtr<void> m_userData;
		ndWeakPtr<ndNewtonWorld> m_world;
		NewtonWorldRayPrefilterCallback m_onFilter;
	};

	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	const ndMatrix matrix(matrixPtr);
	const ndVector destination(target[0], target[1], target[2], ndFloat32(0.0f));

	ndAssert(matrix.TestOrthogonal());
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(shape);
	ndAssert(instance->GetShape()->GetAsShapeConvex());
	ndConvexCast caster(PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld), matrix, destination, *instance, prefilter, userData);
	*param = caster.m_param;

	const ndInt32 contactCount = ndMin(caster.m_contacts.GetCount(), maxContactsCount);
	for (ndInt32 i = 0; i < contactCount; ++i)
	{
		info[i].m_contactID = caster.m_contacts[i].m_shapeId0;
		info[i].m_hitBody = PointerCast<NewtonBody, ndBody>(caster.m_contacts[i].m_body0);
		for (ndInt32 j = 0; j < 4; ++j)
		{
			info[i].m_point[j] = caster.m_contacts[i].m_point[j];
			info[i].m_normal[j] = caster.m_contacts[i].m_normal[j];
		}
	}
	return contactCount;
}


/*!
  Ray cast specific collision object.

  @param *collisionPtr pointer to the collision object.
  @param  *p0 - pointer to an array of at least three floats representing the ray origin in the local space of the geometry.
  @param  *p1 - pointer to an array of at least three floats representing the ray end in the local space of the geometry.
  @param *normal pointer to an array of at least three floats to hold the normal at the intersection point.
  @param *attribute pointer to an array of at least one floats to hold the ID of the face hit by the ray.

  @return the parametric value of the intersection, between 0.0 and 1.0, an value larger than 1.0 if the ray miss.

  This function is intended for applications using newton collision system separate from the dynamics system, also for applications
  implementing any king of special purpose logic like sensing distance to another object.

  the ray most be local to the collisions geometry, for example and application ray casting the collision geometry of
  of a rigid body, must first take the points p0, and p1 to the local space of the rigid body by multiplying the points by the
  inverse of he rigid body transformation matrix.

  See also: ::NewtonCollisionClosestPoint, ::NewtonCollisionSupportVertex, ::NewtonCollisionPointDistance, ::NewtonCollisionCollide, ::NewtonCollisionCalculateAABB
*/
dFloat NewtonCollisionRayCast(const NewtonCollision* const collision, const dFloat* const p0, const dFloat* const p1, dFloat* const normal, dLong* const attribute)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);

	class DommyNotify : public ndRayCastNotify
	{
		public:
		virtual ndFloat32 OnRayCastAction(const ndContactPoint& contact, ndFloat32 intersetParam)
		{
			ndAssert(0);
			return intersetParam;
		}
		
		ndFloat32 m_param;
	};
	DommyNotify notify;

	const ndMatrix matrix (instance->GetLocalMatrix());
	const ndVector q0(matrix.UntransformVector(ndVector(p0[0], p0[1], p0[2], ndFloat32(0.0f))));
	const ndVector q1(matrix.UntransformVector(ndVector(p1[0], p1[1], p1[2], ndFloat32(0.0f))));

	ndContactPoint contact;
	ndQuerieBody dommyBody(*instance);
	ndFloat32 param = instance->RayCast(notify, q0 & ndVector::m_triplexMask, q1 & ndVector::m_triplexMask, &dommyBody, contact, ndFloat32(1.0f));

	if (param >= ndFloat32(0.0f) && param <= ndFloat32(1.0f)) 
	{
		attribute[0] = dLong(contact.m_shapeId0);
	
		ndVector n(matrix.RotateVector(contact.m_normal));
		normal[0] = n[0];
		normal[1] = n[1];
		normal[2] = n[2];
	}
	return param;
}


/*!
  Add buoyancy force and torque for bodies immersed in a fluid.

  @param convexCollision fixme
  @param matrix fixme
  @param fluidPlane fixme
  @param centerOfBuoyancy fixme

  @return Nothing.

  This function is only effective when called from *NewtonApplyForceAndTorque callback*

  This function adds buoyancy force and torque to a body when it is immersed in a fluid.
  The force is calculated according to Archimedes Buoyancy Principle. When the parameter *buoyancyPlane* is set to NULL, the body is considered
  to completely immersed in the fluid. This can be used to simulate boats and lighter than air vehicles etc..

  If *buoyancyPlane* return 0 buoyancy calculation for this collision primitive is ignored, this could be used to filter buoyancy calculation
  of compound collision geometry with different IDs.

  See also: ::NewtonConvexCollisionCalculateVolume
*/
dFloat NewtonConvexCollisionCalculateBuoyancyVolume(const NewtonCollision* const convexCollision, const dFloat* const matrixPtr, const dFloat* const fluidPlane, dFloat* const centerOfBuoyancy)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(convexCollision);
	const ndVector plane(fluidPlane[0], fluidPlane[1], fluidPlane[2], fluidPlane[3]);
	const ndMatrix matrix(GetMatrix(matrixPtr));

	ndVector com;
	ndFloat32 volume = instance->CalculateBuoyancyCenterOfPresure(com, matrix, plane);
	centerOfBuoyancy[0] = com[0];
	centerOfBuoyancy[1] = com[1];
	centerOfBuoyancy[2] = com[2];
	com.m_w = ndFloat32(0.0f);
	return volume;
}

/*!
  calculate the total volume defined by a convex collision geometry.

  @param *convexCollision pointer to the collision.

  @return collision geometry volume. This function will return zero if the body collision geometry is no convex.

  The total volume calculated by the function is only an approximation of the ideal volume. This is not an error, it is a fact resulting from the polygonal representation of convex solids.

  This function can be used to assist the application in calibrating features like fluid density weigh factor when calibrating buoyancy forces for more realistic result.
*/
dFloat NewtonConvexCollisionCalculateVolume(const NewtonCollision* const convexCollision)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(convexCollision);
	return instance->GetVolume();
}

NewtonJoint* NewtonWorldFindJoint(const NewtonBody* const body0, const NewtonBody* const body1)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const kinBody0 = PointerCast<ndBodyKinematic, NewtonBody>(body0);
	ndBodyKinematic* const kinBody1 = PointerCast<ndBodyKinematic, NewtonBody>(body1);

	const ndBodyKinematic::ndJointList& jointList = kinBody0->GetJointList();
	for (ndBodyKinematic::ndJointList::ndNode* node = jointList.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo();
		ndBodyKinematic* const jointBody0 = joint->GetBody0();
		ndBodyKinematic* const jointBody1 = joint->GetBody1();
		bool test = (jointBody0 == kinBody0) && (jointBody1 == kinBody1);
		test = test || ((jointBody1 == kinBody0) && (jointBody0 == kinBody1));
		if (test)
		{
			return PointerCast<NewtonJoint, ndJointBilateralConstraint>(joint);
		}
	}
	return nullptr;
}

int NewtonWorldCollide(const NewtonWorld* const newtonWorld, const dFloat* const matrixPtr, const NewtonCollision* const collision, void* const userData,
	NewtonWorldRayPrefilterCallback prefilter, NewtonWorldConvexCastReturnInfo* const info, int maxContactsCount, int threadIndex)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(GetMatrix(matrixPtr));

	matrix.m_front.m_w = ndFloat32(0.0f);
	matrix.m_up.m_w = ndFloat32(0.0f);
	matrix.m_right.m_w = ndFloat32(0.0f);
	matrix.m_posit.m_w = ndFloat32(1.0f);

	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);

	//Newton* const world = (Newton*)newtonWorld;
	//return world->GetBroadPhase()->Collide((dgCollisionInstance*)shape, dgMatrix(matrix), (OnRayPrecastAction)prefilter, userData, (dgConvexCastReturnInfo*)info, maxContactsCount, threadIndex);
	ndAssert(0);
	return 0;
}
