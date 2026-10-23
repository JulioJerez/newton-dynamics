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

class NewtonRoller : public ndJointRoller
{
	public:
	D_CLASS_REFLECTION(NewtonRoller, ndJointRoller)

	typedef unsigned (*NewtonCorkscrewCallback) (const NewtonJoint* const joint, NewtonHingeSliderUpdateDesc* const desc);

	NewtonRoller(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
		:ndJointRoller(pinAndPivotFrame, child, parent)
		,m_callback(nullptr)
	{
	}
	NewtonCorkscrewCallback m_callback;
};


// 
// roller joint (new for 3.15 bu not supported in 3.14
//
NewtonJoint* NewtonConstraintCreateRoller(const NewtonWorld* const newtonWorld, const dFloat* pivotPoint, const dFloat* pinDir, const NewtonBody* const childBody, const NewtonBody* const parentBody)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndBodyKinematic* const body0 = PointerCast<ndBody, NewtonBody>(childBody)->GetAsBodyKinematic();
	ndBodyKinematic* const body1 = parentBody ? PointerCast<ndBody, NewtonBody>(parentBody)->GetAsBodyKinematic() : world->GetSentinelBody();
	ndAssert(body0);
	ndAssert(body1);
	ndAssert(body0 != body1);

	const ndVector pivot(pivotPoint[0], pivotPoint[1], pivotPoint[2], ndFloat32(0.0f));
	const ndMatrix& bodyMatrix = body0->GetMatrix();

	const ndVector pin(pinDir[0], pinDir[1], pinDir[2], ndFloat32(0.0f));
	ndMatrix matrix(ndGramSchmidtMatrix(pin));
	matrix.m_posit = ndVector(pivotPoint[0], pivotPoint[1], pivotPoint[2], ndFloat32(1.0f));
	ndSharedPtr<ndJointBilateralConstraint> joint(new NewtonRoller(matrix, body0, body1));
	world->AddJoint(joint);
	return PointerCast<NewtonJoint, ndJointBilateralConstraint>(*joint);
}

void NewtonRollerSetUserCallback(const NewtonJoint* const joint, NewtonCorkscrewCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
}

dFloat NewtonRollerGetJointPosit(const NewtonJoint* const joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
}

dFloat NewtonRollerGetJointAngle(const NewtonJoint* const joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
}

dFloat NewtonRollerGetJointVeloc(const NewtonJoint* const joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
}

dFloat NewtonRollerGetJointOmega(const NewtonJoint* const joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
}

void NewtonRollerGetJointForce(const NewtonJoint* const joint, dFloat* const force)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
}

dFloat NewtonRollerCalculateStopAlpha(const NewtonJoint* const joint, const NewtonHingeSliderUpdateDesc* const desc, dFloat angle)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
}

dFloat NewtonRollerCalculateStopAccel(const NewtonJoint* const joint, const NewtonHingeSliderUpdateDesc* const desc, dFloat position)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
}

void NewtonRollerEnableLinearLimits(const NewtonJoint* const joint, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonRoller* const userJoint = PointerCast<NewtonRoller, NewtonJoint>(joint);
	userJoint->SetLimitStatePosit(state ? true : false);
}

void NewtonRollerSetLinearLimits(const NewtonJoint* const joint, dFloat minPosit, dFloat maxPosit)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonRoller* const userJoint = PointerCast<NewtonRoller, NewtonJoint>(joint);
	userJoint->SetLimitsPosit(minPosit, maxPosit);
}

NEWTON_API void NewtonRollerSetLinearSpringDamper(const NewtonJoint* const joint, dFloat regularizer, dFloat spring, dFloat32 damper)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonRoller* const userJoint = PointerCast<NewtonRoller, NewtonJoint>(joint);
	userJoint->SetAsSpringDamperPosit(regularizer, spring, damper);
}

void NewtonRollerEnableAngularLimits(const NewtonJoint* const joint, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonRoller* const userJoint = PointerCast<NewtonRoller, NewtonJoint>(joint);
	userJoint->SetLimitStateAngle(state ? true : false);
}

void NewtonRollerSetAngularLimits(const NewtonJoint* const joint, dFloat minTwistAngle, dFloat maxTwistAngle)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonRoller* const userJoint = PointerCast<NewtonRoller, NewtonJoint>(joint);
	userJoint->SetLimitsAngle(minTwistAngle, maxTwistAngle);
}
