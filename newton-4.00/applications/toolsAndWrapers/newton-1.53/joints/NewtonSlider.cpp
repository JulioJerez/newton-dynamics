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

class NewtonSlider: public ndJointSlider
{
	public:
	D_CLASS_REFLECTION(NewtonSlider, ndJointSlider)

	typedef unsigned (*NewtonSliderCallBack) (const NewtonJoint* slider, NewtonHingeSliderUpdateDesc* desc);

	NewtonSlider(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
		:ndJointSlider(pinAndPivotFrame, child, parent)
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
			NewtonJoint* const joint = PointerCast<NewtonJoint, NewtonSlider>(this);
			SetAsMotor(ndFloat32 (0.0f), ndFloat32(0.0f));
			if (m_callback(joint, &userDesc))
			{
				SetAsMotor(GetSpeed() + userDesc.m_accel * desc.m_invTimestep, ndMax (-userDesc.m_minFriction, userDesc.m_maxFriction));
			}
		}

		ndJointSlider::JacobianDerivative(desc);
	}

	NewtonSliderCallBack m_callback;
};

#if 0

// ***************************************************************************************************************
//
// Name: Slider joint interface
//
// ***************************************************************************************************************

// Name: NewtonConstraintCreateSlider
// Create a Slider joint. 
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const NewtonCollision* *pivotPoint - is origin of the Slider in global space.
// *const NewtonCollision* *pinDir - is the line of action of the Slider in global space.
// *const NewtonBody* *childBody - is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
// *const NewtonBody* *parentBody - is the pointer to the parent rigid body, this body can be NULL or any kind of rigid body.
//
// Return: Pointer to the Slider joint.
//
// Remarks: This function creates a Slider and add it to the world. By default joint disables collision with the linked bodies. 
NewtonJoint* NewtonConstraintCreateSlider(const NewtonWorld* newtonWorld, const dFloat* pivotPoint, const dFloat* pinDir, const NewtonBody* childBody, const NewtonBody* parentBody)
{
	dgBody* body0;
	dgBody* body1;
	Newton* world;

	world = (Newton*)newtonWorld;
	body0 = (dgBody*)childBody;
	body1 = (dgBody*)parentBody;
	dgVector pivot(pivotPoint[0], pivotPoint[1], pivotPoint[2]);
	dgVector pin(pinDir[0], pinDir[1], pinDir[2]);
	return (NewtonJoint*)world->CreateSliderConstraint(pivot, pin, body0, body1);
}


// Name: NewtonSliderSetUserCallback
// Set an update call back to be called when either of the two body linked by the joint is active.
//
// Parameters:
// *const NewtonJoint* *Slider - pointer to the joint.
// *NewtonSliderCallBack* callback - pointer to the joint function call back.
//
// Return: nothing.
//
// Remarks: if the application wants to have some feedback from the joint simulation, the application can register a function
// update callback to be call every time any of the bodies linked by this joint is active. This is useful to provide special
// effects like particles, sound or even to simulate breakable moving parts.
// 
// See also: NewtonJointGetUserData, NewtonJointSetUserData
void NewtonSliderSetUserCallback(const NewtonJoint* Slider, NewtonSliderCallBack callback)
{
	dgSliderConstraint* contraint;

	contraint = (dgSliderConstraint*)Slider;
	contraint->SetJointParameterCallBack((dgSliderJointAcceleration)callback);
}


// Name: NewtonSliderGetJointAngle
// Get the relative joint angle between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Slider - pointer to the joint.
//
// Return: the joint angle relative to the Slider pin.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play a bell sound when the joint angle passes some max value.
// 
// See also: NewtonSliderSetUserCallback
dFloat NewtonSliderGetJointAngle(const NewtonJoint* Slider)
{
	dgSliderConstraint* contraint;

	contraint = (dgSliderConstraint*)Slider;
	return contraint->GetJointAngle();

}

// Name: NewtonSliderGetJointOmega
// Get the relative joint angular velocity between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Slider - pointer to the joint.
//
// Return: the joint angular velocity relative to the pin axis.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play the creaky noise of a hanging lamp.
// 
// See also: NewtonSliderSetUserCallback
dFloat NewtonSliderGetJointOmega(const NewtonJoint* Slider)
{
	dgSliderConstraint* contraint;

	contraint = (dgSliderConstraint*)Slider;
	return contraint->GetJointOmega();
}

// Name: NewtonSliderGetJointForce
// Calculate the angular acceleration needed to stop the Slider at the desired angle.
//
// Parameters:
// *const NewtonJoint* *Slider - pointer to the joint.
// *NewtonSliderSliderUpdateDesc* *desc - is the pointer to and the Slider or slide structure.
// *dFloat* angle - is the desired Slider stop angle 
//
// Return: the relative angular acceleration needed to stop the Slider.
//
// Remarks: this function can only be called from a *NewtonSliderCallBack* and it can be used by the application to implement Slider limits.
// 
// See also: NewtonSliderSetUserCallback
dFloat NewtonSliderCalculateStopAlpha(const NewtonJoint* Slider, const NewtonSliderSliderUpdateDesc* desc, dFloat angle)
{
	dgSliderConstraint* contraint;

	contraint = (dgSliderConstraint*)Slider;
	return contraint->CalculateStopAlpha(angle, (dgJointCallBackParam*)desc);
}

// Name: NewtonSliderGetJointForce
// Get the total force asserted over the joint pivot point, to maintain the constraint.
//
// Parameters:
// *const NewtonJoint* *Slider - pointer to the joint.
// *dFloat* *force - pointer to an array of a least three floats to hold the force value of the joint.
//
// Return: nothing.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can destroy the joint if the force exceeds some predefined value.
// 
// See also: NewtonSliderSetUserCallback
void NewtonSliderGetJointForce(const NewtonJoint* Slider, dFloat* force)
{
	dgSliderConstraint* contraint;

	contraint = (dgSliderConstraint*)Slider;
	dgVector forceVector(contraint->GetJointForce());
	force[0] = forceVector.m_x;
	force[1] = forceVector.m_y;
	force[2] = forceVector.m_z;
}

dFloat NewtonSliderGetJointAngle(const NewtonJoint* const Slider)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
	//dgSliderConstraint* contraint = (dgSliderConstraint*)Slider;
	//return contraint->GetJointAngle();
}

void NewtonSliderGetJointForce(const NewtonJoint* const Slider, dFloat* const force)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgSliderConstraint* contraint = (dgSliderConstraint*)Slider;
	//const dgVector forceVector(contraint->GetJointForce());
	//force[0] = forceVector.m_x;
	//force[1] = forceVector.m_y;
	//force[2] = forceVector.m_z;
}

dFloat NewtonSliderCalculateStopAlpha(const NewtonJoint* const Slider, const NewtonSliderSliderUpdateDesc* const desc, dFloat angle)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return 0;
	//dgSliderConstraint* contraint = (dgSliderConstraint*)Slider;
	//return contraint->CalculateStopAlpha(angle, (dgJointCallbackParam*)desc);
}


void NewtonSliderEnableLimits(const NewtonJoint* const joint, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonSlider* const userJoint = PointerCast<NewtonSlider, NewtonJoint>(joint);
	userJoint->SetLimitState(state ? true : false);
}

void NewtonSliderSetLimits(const NewtonJoint* const joint, dFloat minTwistAngle, dFloat maxTwistAngle)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonSlider* const userJoint = PointerCast<NewtonSlider, NewtonJoint>(joint);
	userJoint->SetLimits(minTwistAngle, maxTwistAngle);
}

void NewtonSliderSetSpringDamper(const NewtonJoint* const joint, dFloat regularizer, dFloat spring, dFloat32 damper)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonSlider* const userJoint = PointerCast<NewtonSlider, NewtonJoint>(joint);
	userJoint->SetAsSpringDamper(regularizer, spring, damper);
}

void NewtonSliderSetAsMotor(const NewtonJoint* const joint, dFloat omega, dFloat maxTorque)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonSlider* const userJoint = PointerCast<NewtonSlider, NewtonJoint>(joint);
	userJoint->SetAsMotor(omega, maxTorque);
}

void NewtonSliderSetDryFriction(const NewtonJoint* const joint, dFloat frictionTorque)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonSlider* const userJoint = PointerCast<NewtonSlider, NewtonJoint>(joint);
	userJoint->SetAsDryFriction(frictionTorque);
}
#endif

//
// Create a Slider joint.
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

	ndVector pin(pinDir[0], pinDir[1], pinDir[2], ndFloat32(0.0f));
	ndMatrix matrix(ndGramSchmidtMatrix(pin));
	matrix.m_posit = ndVector(pivotPoint[0], pivotPoint[1], pivotPoint[2], ndFloat32(1.0f));
	ndSharedPtr<ndJointBilateralConstraint> joint(new NewtonSlider(matrix, body0, body1));
	world->AddJoint(joint);
	return PointerCast<NewtonJoint, ndJointBilateralConstraint>(*joint);
}

// Name: NewtonSliderSetUserCallback
// Set an update call back to be called when either of the two body linked by the joint is active.
//
// Parameters:
// *const NewtonJoint* *Slider - pointer to the joint.
// *NewtonSliderCallBack* callback - pointer to the joint function call back.
//
// Return: nothing.
//
// Remarks: if the application wants to have some feedback from the joint simulation, the application can register a function
// update callback to be call every time any of the bodies linked by this joint is active. This is useful to provide special
// effects like particles, sound or even to simulate breakable moving parts.
// 
// See also: NewtonJointGetUserData, NewtonJointSetUserData
void NewtonSliderSetUserCallback(const NewtonJoint* const joint, NewtonSliderCallBack callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonSlider* const userJoint = PointerCast<NewtonSlider, NewtonJoint>(joint);
	userJoint->m_callback = callback;
}

dFloat NewtonSliderGetJointVeloc(const NewtonJoint* const joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonSlider* const userJoint = PointerCast<NewtonSlider, NewtonJoint>(joint);
	return userJoint->GetSpeed();
}
