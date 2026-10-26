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

class NewtonHinge: public ndJointHinge
{
	public:
	D_CLASS_REFLECTION(NewtonHinge, ndJointHinge)

	typedef unsigned (*NewtonHingeCallBack) (const NewtonJoint* hinge, NewtonHingeSliderUpdateDesc* desc);

	NewtonHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
		:ndJointHinge(pinAndPivotFrame, child, parent)
		,m_callback(nullptr)
	{
	}

	void JacobianDerivative(ndConstraintDescritor& desc) override
	{
		if (m_callback && (**GetUserData()))
		{
			NewtonHingeSliderUpdateDesc userDesc;
			unsigned state = 0;
			userDesc.m_accel = 0.0f;
			userDesc.m_timestep = desc.m_timestep;
			userDesc.m_minFriction = -1.0e-20f;
			userDesc.m_maxFriction = 1.0e-20f;
			NewtonJoint* const joint = PointerCast<NewtonJoint, NewtonHinge>(this);
			SetAsMotor(ndFloat32 (0.0f), ndFloat32(0.0f));
			if (m_callback(joint, &userDesc))
			{
				SetAsMotor(GetOmega() + userDesc.m_accel * desc.m_invTimestep, ndMax (-userDesc.m_minFriction, userDesc.m_maxFriction));
			}
		}

		ndJointHinge::JacobianDerivative(desc);
	}

	NewtonHingeCallBack m_callback;
};

#if 0

// ***************************************************************************************************************
//
// Name: Hinge joint interface
//
// ***************************************************************************************************************

// Name: NewtonConstraintCreateHinge
// Create a hinge joint. 
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const NewtonCollision* *pivotPoint - is origin of the hinge in global space.
// *const NewtonCollision* *pinDir - is the line of action of the hinge in global space.
// *const NewtonBody* *childBody - is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
// *const NewtonBody* *parentBody - is the pointer to the parent rigid body, this body can be NULL or any kind of rigid body.
//
// Return: Pointer to the hinge joint.
//
// Remarks: This function creates a hinge and add it to the world. By default joint disables collision with the linked bodies. 
NewtonJoint* NewtonConstraintCreateHinge(const NewtonWorld* newtonWorld, const dFloat* pivotPoint, const dFloat* pinDir, const NewtonBody* childBody, const NewtonBody* parentBody)
{
	dgBody* body0;
	dgBody* body1;
	Newton* world;

	world = (Newton*)newtonWorld;
	body0 = (dgBody*)childBody;
	body1 = (dgBody*)parentBody;
	dgVector pivot(pivotPoint[0], pivotPoint[1], pivotPoint[2]);
	dgVector pin(pinDir[0], pinDir[1], pinDir[2]);
	return (NewtonJoint*)world->CreateHingeConstraint(pivot, pin, body0, body1);
}


// Name: NewtonHingeSetUserCallback
// Set an update call back to be called when either of the two body linked by the joint is active.
//
// Parameters:
// *const NewtonJoint* *Hinge - pointer to the joint.
// *NewtonHingeCallBack* callback - pointer to the joint function call back.
//
// Return: nothing.
//
// Remarks: if the application wants to have some feedback from the joint simulation, the application can register a function
// update callback to be call every time any of the bodies linked by this joint is active. This is useful to provide special
// effects like particles, sound or even to simulate breakable moving parts.
// 
// See also: NewtonJointGetUserData, NewtonJointSetUserData
void NewtonHingeSetUserCallback(const NewtonJoint* Hinge, NewtonHingeCallBack callback)
{
	dgHingeConstraint* contraint;

	contraint = (dgHingeConstraint*)Hinge;
	contraint->SetJointParameterCallBack((dgHingeJointAcceleration)callback);
}


// Name: NewtonHingeGetJointAngle
// Get the relative joint angle between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Hinge - pointer to the joint.
//
// Return: the joint angle relative to the hinge pin.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play a bell sound when the joint angle passes some max value.
// 
// See also: NewtonHingeSetUserCallback
dFloat NewtonHingeGetJointAngle(const NewtonJoint* Hinge)
{
	dgHingeConstraint* contraint;

	contraint = (dgHingeConstraint*)Hinge;
	return contraint->GetJointAngle();

}

// Name: NewtonHingeGetJointOmega
// Get the relative joint angular velocity between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Hinge - pointer to the joint.
//
// Return: the joint angular velocity relative to the pin axis.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play the creaky noise of a hanging lamp.
// 
// See also: NewtonHingeSetUserCallback
dFloat NewtonHingeGetJointOmega(const NewtonJoint* Hinge)
{
	dgHingeConstraint* contraint;

	contraint = (dgHingeConstraint*)Hinge;
	return contraint->GetJointOmega();
}

// Name: NewtonHingeGetJointForce
// Calculate the angular acceleration needed to stop the hinge at the desired angle.
//
// Parameters:
// *const NewtonJoint* *Hinge - pointer to the joint.
// *NewtonHingeSliderUpdateDesc* *desc - is the pointer to and the Hinge or slide structure.
// *dFloat* angle - is the desired hinge stop angle 
//
// Return: the relative angular acceleration needed to stop the hinge.
//
// Remarks: this function can only be called from a *NewtonHingeCallBack* and it can be used by the application to implement hinge limits.
// 
// See also: NewtonHingeSetUserCallback
dFloat NewtonHingeCalculateStopAlpha(const NewtonJoint* hinge, const NewtonHingeSliderUpdateDesc* desc, dFloat angle)
{
	dgHingeConstraint* contraint;

	contraint = (dgHingeConstraint*)hinge;
	return contraint->CalculateStopAlpha(angle, (dgJointCallBackParam*)desc);
}

// Name: NewtonHingeGetJointForce
// Get the total force asserted over the joint pivot point, to maintain the constraint.
//
// Parameters:
// *const NewtonJoint* *Hinge - pointer to the joint.
// *dFloat* *force - pointer to an array of a least three floats to hold the force value of the joint.
//
// Return: nothing.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can destroy the joint if the force exceeds some predefined value.
// 
// See also: NewtonHingeSetUserCallback
void NewtonHingeGetJointForce(const NewtonJoint* Hinge, dFloat* force)
{
	dgHingeConstraint* contraint;

	contraint = (dgHingeConstraint*)Hinge;
	dgVector forceVector(contraint->GetJointForce());
	force[0] = forceVector.m_x;
	force[1] = forceVector.m_y;
	force[2] = forceVector.m_z;
}

dFloat NewtonHingeGetJointAngle(const NewtonJoint* const hinge)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
	//dgHingeConstraint* contraint = (dgHingeConstraint*)hinge;
	//return contraint->GetJointAngle();
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


void NewtonHingeEnableLimits(const NewtonJoint* const joint, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonHinge* const userJoint = PointerCast<NewtonHinge, NewtonJoint>(joint);
	userJoint->SetLimitState(state ? true : false);
}

void NewtonHingeSetLimits(const NewtonJoint* const joint, dFloat minTwistAngle, dFloat maxTwistAngle)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonHinge* const userJoint = PointerCast<NewtonHinge, NewtonJoint>(joint);
	userJoint->SetLimits(minTwistAngle, maxTwistAngle);
}

void NewtonHingeSetSpringDamper(const NewtonJoint* const joint, dFloat regularizer, dFloat spring, dFloat32 damper)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonHinge* const userJoint = PointerCast<NewtonHinge, NewtonJoint>(joint);
	userJoint->SetAsSpringDamper(regularizer, spring, damper);
}

void NewtonHingeSetAsMotor(const NewtonJoint* const joint, dFloat omega, dFloat maxTorque)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonHinge* const userJoint = PointerCast<NewtonHinge, NewtonJoint>(joint);
	userJoint->SetAsMotor(omega, maxTorque);
}

void NewtonHingeSetDryFriction(const NewtonJoint* const joint, dFloat frictionTorque)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonHinge* const userJoint = PointerCast<NewtonHinge, NewtonJoint>(joint);
	userJoint->SetAsDryFriction(frictionTorque);
}
#endif

//
// Create a hinge joint.
//
NewtonJoint* NewtonConstraintCreateHinge(const NewtonWorld* const newtonWorld, const dFloat* pivotPoint, const dFloat* pinDir, const NewtonBody* const childBody, const NewtonBody* const parentBody)
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

	ndVector pin(pinDir[0], pinDir[1], pinDir[2], ndFloat32(0.0f));
	ndMatrix matrix(ndGramSchmidtMatrix(pin));
	matrix.m_posit = ndVector(pivotPoint[0], pivotPoint[1], pivotPoint[2], ndFloat32(1.0f));
	ndSharedPtr<ndJointBilateralConstraint> joint(new NewtonHinge(matrix, body0, body1));
	world->AddJoint(joint);
	return PointerCast<NewtonJoint, ndJointBilateralConstraint>(*joint);
}

// Name: NewtonHingeSetUserCallback
// Set an update call back to be called when either of the two body linked by the joint is active.
//
// Parameters:
// *const NewtonJoint* *Hinge - pointer to the joint.
// *NewtonHingeCallBack* callback - pointer to the joint function call back.
//
// Return: nothing.
//
// Remarks: if the application wants to have some feedback from the joint simulation, the application can register a function
// update callback to be call every time any of the bodies linked by this joint is active. This is useful to provide special
// effects like particles, sound or even to simulate breakable moving parts.
// 
// See also: NewtonJointGetUserData, NewtonJointSetUserData
void NewtonHingeSetUserCallback(const NewtonJoint* const joint, NewtonHingeCallBack callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonHinge* const userJoint = PointerCast<NewtonHinge, NewtonJoint>(joint);
	userJoint->m_callback = callback;
}

dFloat NewtonHingeGetJointOmega(const NewtonJoint* const joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonHinge* const userJoint = PointerCast<NewtonHinge, NewtonJoint>(joint);
	return userJoint->GetOmega();
}
