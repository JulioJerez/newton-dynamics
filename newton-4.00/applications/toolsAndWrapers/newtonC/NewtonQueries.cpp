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
				, m_userData(userData)
				, m_world(world)
				, m_filter(filter)
				, m_prefilter(prefilter)
			{
			}

			ndUnsigned32 OnRayPrecastAction(const ndBody* const body, const ndShapeInstance* const instance) override
			{
				if (m_prefilter)
				{
					ndWeakPtr<const ndShapeInstance> sharedInstance(instance);
					ndSharedPtr<ndBody> sharedBody(m_world->GetBody(const_cast<ndBody*>(reinterpret_cast<const ndBody*>(body))));
					return m_prefilter(reinterpret_cast<NewtonBody*>(&sharedBody), reinterpret_cast<const NewtonCollision*>(&sharedInstance), m_userData);
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
					ndWeakPtr<const ndShapeInstance> sharedInstance(contact.m_shapeInstance0);
					ndSharedPtr<ndBody> sharedBody(m_world->GetBody(const_cast<ndBody*>(reinterpret_cast<const ndBody*>(contact.m_body0))));
					intersetParam = m_filter(reinterpret_cast<NewtonBody*>(&sharedBody), reinterpret_cast<const NewtonCollision*>(&sharedInstance), &intersetParam, &contact.m_normal[0], contact.m_shapeId0, m_userData, intersetParam);
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