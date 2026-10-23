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

class NewtonRoller : public ndJointCylinder
{
	public:
	D_CLASS_REFLECTION(NewtonRoller, ndJointCylinder)

	typedef unsigned (*NewtonCorkscrewCallback) (const NewtonJoint* const corkscrew, NewtonHingeSliderUpdateDesc* const desc);

	NewtonRoller(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
		:ndJointCylinder(pinAndPivotFrame, child, parent)
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
	return NewtonConstraintCreateCorkscrew(newtonWorld, pivotPoint, pinDir, childBody, parentBody);
}

void NewtonRollerSetUserCallback(const NewtonJoint* const corkscrew, NewtonCorkscrewCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
}

dFloat NewtonRollerGetJointPosit(const NewtonJoint* const corkscrew)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
}

dFloat NewtonRollerGetJointAngle(const NewtonJoint* const corkscrew)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
}

dFloat NewtonRollerGetJointVeloc(const NewtonJoint* const corkscrew)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
}

dFloat NewtonRollerGetJointOmega(const NewtonJoint* const corkscrew)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
}

void NewtonRollerGetJointForce(const NewtonJoint* const corkscrew, dFloat* const force)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
}

dFloat NewtonRollerCalculateStopAlpha(const NewtonJoint* const corkscrew, const NewtonHingeSliderUpdateDesc* const desc, dFloat angle)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
}

dFloat NewtonRollerCalculateStopAccel(const NewtonJoint* const corkscrew, const NewtonHingeSliderUpdateDesc* const desc, dFloat position)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
}

void NewtonRollerEnableLinearLimits(const NewtonJoint* const corkscrew, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
}

void NewtonRollerSetLinearLimits(const NewtonJoint* const corkscrew, dFloat minPosit, dFloat maxPosit)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
}

void NewtonRollerEnableAngularLimits(const NewtonJoint* const corkscrew, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
}

void NewtonRollerSetAngularLimits(const NewtonJoint* const corkscrew, dFloat minTwistAngle, dFloat maxTwistAngle)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
}

