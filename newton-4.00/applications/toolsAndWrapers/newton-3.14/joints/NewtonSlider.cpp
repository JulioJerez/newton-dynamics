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

class NewtonSlider: public ndJointSlider
{
	public:
	D_CLASS_REFLECTION(NewtonSlider, ndJointSlider)

	typedef unsigned (*NewtonSliderCallback) (const NewtonJoint* const slider, NewtonHingeSliderUpdateDesc* const desc);

	NewtonSlider(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
		:ndJointSlider(pinAndPivotFrame, child, parent)
		,m_callback(nullptr)
	{
	}
	NewtonSliderCallback m_callback;
};


void NewtonSliderSetUserCallback(const NewtonJoint* const hinge, NewtonSliderCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//dgHingeConstraint* contraint = (dgHingeConstraint*)hinge;
	//contraint->SetJointParameterCallback((dgHingeJointAcceleration)callback);
}

dFloat NewtonSliderGetJointAngle(const NewtonJoint* const hinge)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
	//dgHingeConstraint* contraint = (dgHingeConstraint*)hinge;
	//return contraint->GetJointAngle();
}

dFloat NewtonSliderGetJointOmega(const NewtonJoint* const hinge)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
	//dgHingeConstraint* contraint = (dgHingeConstraint*)hinge;
	//return contraint->GetJointOmega();
}

void NewtonSliderGetJointForce(const NewtonJoint* const hinge, dFloat* const force)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgHingeConstraint* contraint = (dgHingeConstraint*)hinge;
	//const dgVector forceVector(contraint->GetJointForce());
	//force[0] = forceVector.m_x;
	//force[1] = forceVector.m_y;
	//force[2] = forceVector.m_z;
}

dFloat NewtonSliderCalculateStopAlpha(const NewtonJoint* const hinge, const NewtonHingeSliderUpdateDesc* const desc, dFloat angle)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
	//dgHingeConstraint* contraint = (dgHingeConstraint*)hinge;
	//return contraint->CalculateStopAlpha(angle, (dgJointCallbackParam*)desc);
}

//
// Create a hinge joint.
//
NewtonJoint* NewtonConstraintCreateSlider(const NewtonWorld* const newtonWorld, const dFloat* pivotPoint, const dFloat* pinDir, const NewtonBody* const childBody, const NewtonBody* const parentBody)
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

	ndVector pin(pinDir[0], pinDir[1], pinDir[2], ndFloat32 (0.0f));
	ndMatrix matrix(ndGramSchmidtMatrix(pin));
	matrix.m_posit = ndVector (pivotPoint[0], pivotPoint[1], pivotPoint[2], ndFloat32(1.0f));
	ndSharedPtr<ndJointBilateralConstraint> joint(new NewtonSlider(matrix, body0, body1));
	world->AddJoint(joint);
	return PointerCast<NewtonJoint, ndJointBilateralConstraint>(*joint);
}

void NewtonSliderEnableLimits(const NewtonJoint* const joint, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonSlider* const userJoint = PointerCast<NewtonSlider, NewtonJoint>(joint);
	userJoint->SetLimitState(state ? true : false);
}

void NewtonSliderSetLimits(const NewtonJoint* const joint, dFloat minPosit, dFloat maxPosit)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonSlider* const userJoint = PointerCast<NewtonSlider, NewtonJoint>(joint);
	userJoint->SetLimits(minPosit, maxPosit);
}

void NewtonSliderSetSpringDamper(const NewtonJoint* const joint, dFloat regularizer, dFloat spring, dFloat32 damper)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonSlider* const userJoint = PointerCast<NewtonSlider, NewtonJoint>(joint);
	userJoint->SetAsSpringDamper(regularizer, spring, damper);
}

void NewtonSliderSetAsMotor(const NewtonJoint* const joint, dFloat speed, dFloat maxForce)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonSlider* const userJoint = PointerCast<NewtonSlider, NewtonJoint>(joint);
	userJoint->SetAsMotor(speed, maxForce);
}

void NewtonSliderSetDryFriction(const NewtonJoint* const joint, dFloat frictionForce)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonSlider* const userJoint = PointerCast<NewtonSlider, NewtonJoint>(joint);
	userJoint->SetAsDryFriction(frictionForce);
}