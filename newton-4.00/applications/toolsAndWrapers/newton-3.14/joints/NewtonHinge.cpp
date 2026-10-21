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

class NewtonHinge: public ndJointHinge
{
	public:
	D_CLASS_REFLECTION(NewtonHinge, ndJointHinge)

	//typedef void (*NewtonBallCallback) (const NewtonJoint* const joint,  dFloat timestep);

	NewtonHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
		:ndJointHinge(pinAndPivotFrame, child, parent)
	{
	}
	//NewtonBallCallback m_callback;
};


//
// Create a hinge joint.
//
NewtonJoint* NewtonConstraintCreateHinge(const NewtonWorld* const newtonWorld, const dFloat* pivotPoint, const dFloat* pinDir, const NewtonBody* const childBody, const NewtonBody* const parentBody)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return nullptr;
	//Newton* const world = (Newton*)newtonWorld;
	//dgBody* const body0 = (dgBody*)childBody;
	//dgBody* const body1 = (dgBody*)parentBody;
	//dgVector pin(pinDir[0], pinDir[1], pinDir[2], dgFloat32(0.0f));
	//dgVector pivot(pivotPoint[0], pivotPoint[1], pivotPoint[2], dgFloat32(0.0f));
	//return (NewtonJoint*)world->CreateHingeConstraint(pivot, pin, body0, body1);
}

void NewtonHingeSetUserCallback(const NewtonJoint* const hinge, NewtonHingeCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//dgHingeConstraint* contraint = (dgHingeConstraint*)hinge;
	//contraint->SetJointParameterCallback((dgHingeJointAcceleration)callback);
}

dFloat NewtonHingeGetJointAngle(const NewtonJoint* const hinge)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
	//dgHingeConstraint* contraint = (dgHingeConstraint*)hinge;
	//return contraint->GetJointAngle();
}

dFloat NewtonHingeGetJointOmega(const NewtonJoint* const hinge)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
	//dgHingeConstraint* contraint = (dgHingeConstraint*)hinge;
	//return contraint->GetJointOmega();
}

void NewtonHingeGetJointForce(const NewtonJoint* const hinge, dFloat* const force)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgHingeConstraint* contraint = (dgHingeConstraint*)hinge;
	//const dgVector forceVector(contraint->GetJointForce());
	//force[0] = forceVector.m_x;
	//force[1] = forceVector.m_y;
	//force[2] = forceVector.m_z;
}

dFloat NewtonHingeCalculateStopAlpha(const NewtonJoint* const hinge, const NewtonHingeSliderUpdateDesc* const desc, dFloat angle)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
	//dgHingeConstraint* contraint = (dgHingeConstraint*)hinge;
	//return contraint->CalculateStopAlpha(angle, (dgJointCallbackParam*)desc);
}
