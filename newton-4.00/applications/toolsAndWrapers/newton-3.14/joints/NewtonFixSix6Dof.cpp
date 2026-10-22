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
#include "newtonUserJointClass.h"


/*!
  fix 6 dof joint

  @param *newtonWorld Pointer to the Newton world.
  @param *pivotPoint is origin of ball and socket in global space.
  @param *childBody is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
  @param *parentBody is the pointer to the parent rigid body, this body can be NULL or any kind of rigid body.
*/
NewtonJoint* NewtonConstraintCreateFix6Dof(const NewtonWorld* const newtonWorld,
	const NewtonBody* const childBody,
	const NewtonBody* const parentBody)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndBodyKinematic* const body0 = PointerCast<ndBody, NewtonBody>(childBody)->GetAsBodyKinematic();
	ndBodyKinematic* const body1 = parentBody ? PointerCast<ndBody, NewtonBody>(parentBody)->GetAsBodyKinematic() : world->GetSentinelBody();
	ndAssert(body0);
	ndAssert(body1);
	ndAssert(body0 != body1);

	ndSharedPtr<ndJointBilateralConstraint> joint(new ndJointFix6dof(body0->GetMatrix(), body0, body1));
	world->AddJoint(joint);
	return PointerCast< NewtonJoint, ndJointBilateralConstraint>(*joint);
}

