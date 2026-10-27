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

class NewtonUniversal: public ndJointDoubleHinge
{
	public:
	D_CLASS_REFLECTION(NewtonUniversal, ndJointDoubleHinge)

	typedef unsigned (*NewtonUniversalCallBack) (const NewtonJoint* joint, NewtonHingeSliderUpdateDesc* desc);

	NewtonUniversal(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
		:ndJointDoubleHinge(pinAndPivotFrame, child, parent)
		,m_callback(nullptr)
	{
	}

	void JacobianDerivative(ndConstraintDescritor& desc) override
	{
		if (m_callback && (**GetUserData()))
		{
			//NewtonHingeSliderUpdateDesc userDesc[2];
			//unsigned state = 0;
			//userDesc[0].m_accel = 0.0f;
			//userDesc[0].m_timestep = desc.m_timestep;
			//userDesc[0].m_minFriction = -1.0e-20f;
			//userDesc[0].m_maxFriction = 1.0e-20f;
			//userDesc[1].m_accel = 0.0f;
			//userDesc[1].m_timestep = desc.m_timestep;
			//userDesc[1].m_minFriction = -1.0e-20f;
			//userDesc[1].m_maxFriction = 1.0e-20f;
			//
			//NewtonJoint* const joint = PointerCast<NewtonJoint, NewtonUniversal>(this);
			//ndInt32 code = m_callback(joint, userDesc);

			//NewtonHingeSliderUpdateDesc userDesc;
			//unsigned state = 0;
			//userDesc.m_accel = 0.0f;
			//userDesc.m_timestep = desc.m_timestep;
			//userDesc.m_minFriction = -1.0e-20f;
			//userDesc.m_maxFriction = 1.0e-20f;
			//NewtonJoint* const joint = PointerCast<NewtonJoint, NewtonCylinder>(this);
			//SetAsMotor(ndFloat32 (0.0f), ndFloat32(0.0f));
			//if (m_callback(joint, &userDesc))
			//{
			//	SetAsMotor(GetSpeed() + userDesc.m_accel * desc.m_invTimestep, ndMax (-userDesc.m_minFriction, userDesc.m_maxFriction));
			//}
		}

		ndJointDoubleHinge::JacobianDerivative(desc);
	}

	NewtonUniversalCallBack m_callback;
};

#if 0

// Name: NewtonUniversalSetUserCallback
// Set an update call back to be called when either of the two body linked by the joint is active.
//
// Parameters:
// *const NewtonJoint* *Universal - pointer to the joint.
// *NewtonUniversalCallBack* callback - pointer to the joint function call back.
//
// Return: nothing.
//
// Remarks: if the application wants to have some feedback from the joint simulation, the application can register a function
// update callback to be called every time any of the bodies linked by this joint is active. This is useful to provide special
// effects like particles, sound or even to simulate breakable moving parts.
//
// Remarks: the function *NewtonUniversalCallBack callback* should return a bit field code.
// if the application does not want to set the joint acceleration the return code is zero
// if the application only wants to change the joint linear acceleration the return code is 1
// if the application only wants to change the joint angular acceleration the return code is 2
// if the application only wants to change the joint angular and linear acceleration the return code is 3
// 
// See also: NewtonJointGetUserData, NewtonJointSetUserData
void NewtonUniversalSetUserCallback(const NewtonJoint* joint, NewtonUniversalCallBack callback)
{
	dgUniversalConstraint* contraint;

	contraint = (dgUniversalConstraint*)Universal;
	contraint->SetJointParameterCallBack((dgUniversalJointAcceleration)callback);
}


// Name: NewtonUniversalGetJointForce
// Get the total force asserted over the joint pivot point, to maintain the constraint.
//
// Parameters:
// *const NewtonJoint* *Universal - pointer to the joint.
// *dFloat* *force - pointer to an array of a least three floats to hold the force value of the joint.
//
// Return: nothing.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can destroy the joint if the force exceeds some predefined value.
// 
// See also: NewtonUniversalSetUserCallback
void NewtonUniversalGetJointForce(const NewtonJoint* joint, dFloat* force)
{
	dgUniversalConstraint* contraint;

	contraint = (dgUniversalConstraint*)Universal;
	dgVector forceVector(contraint->GetJointForce());
	force[0] = forceVector.m_x;
	force[1] = forceVector.m_y;
	force[2] = forceVector.m_z;
}

#endif

// Name: NewtonConstraintCreateUniversal
// Create a universal joint. 
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const NewtonCollision* *pivotPoint - is origin of the universal joint in global space.
// *const NewtonCollision* *pinDir0 - first axis of rotation fixed on childBody body and perpendicular to pinDir1.
// *const NewtonCollision* *pinDir1 - second axis of rotation fixed on parentBody body and perpendicular to pinDir0.
// *const NewtonBody* *childBody - is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
// *const NewtonBody* *parentBody - is the pointer to the parent rigid body, this body can be NULL or any kind of rigid body.
//
// Return: Pointer to the universal joint.
//
// Remarks: This function creates a universal joint and add it to the world. By default joint disables collision with the linked bodies. 
//
// Remark: a universal joint is a constraint that restricts twp rigid bodies to be connected to a point fixed on both bodies, 
// while and allowing one body to spin around a fix axis in is own frame, and the other body to spin around another axis fixes on 
// it own frame. Both axis must be mutually perpendicular. 
NewtonJoint* NewtonConstraintCreateUniversal(const NewtonWorld* newtonWorld, const dFloat* pivotPoint,
	const dFloat* pinDir0, const dFloat* pinDir1, const NewtonBody* childBody, const NewtonBody* parentBody)
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

	ndVector pin0(pinDir0[0], pinDir0[1], pinDir0[2], ndFloat32(0.0f));
	ndVector pin1(pinDir1[0], pinDir1[1], pinDir1[2], ndFloat32(0.0f));
	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_front = pin0.Normalize();
	matrix.m_right = (pin0.CrossProduct(pin1) & ndVector::m_triplexMask).Normalize();
	matrix.m_up = matrix.m_right.CrossProduct(matrix.m_front);
	matrix.m_posit = ndVector(pivotPoint[0], pivotPoint[1], pivotPoint[2], ndFloat32(1.0f));

	ndSharedPtr<ndJointBilateralConstraint> joint(new NewtonUniversal(matrix, body0, body1));
	world->AddJoint(joint);
	return PointerCast<NewtonJoint, ndJointBilateralConstraint>(*joint);
}

// Name: NewtonUniversalGetJointAngle0
// Get the relative joint angle between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Universal - pointer to the joint.
//
// Return: the joint angle relative to the universal pin0.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play a bell sound when the joint angle passes some max value.
// 
// See also: NewtonUniversalSetUserCallback
dFloat NewtonUniversalGetJointAngle0(const NewtonJoint* joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonUniversal* const userJoint = PointerCast<NewtonUniversal, NewtonJoint>(joint);
	return userJoint->GetAngle0();
}

// Name: NewtonUniversalGetJointAngle1
// Get the relative joint angle between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Universal - pointer to the joint.
//
// Return: the joint angle relative to the universal pin1.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play a bell sound when the joint angle passes some max value.
// 
// See also: NewtonUniversalSetUserCallback
dFloat NewtonUniversalGetJointAngle1(const NewtonJoint* joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonUniversal* const userJoint = PointerCast<NewtonUniversal, NewtonJoint>(joint);
	return userJoint->GetAngle1();
}

// Name: NewtonUniversalGetJointOmega0
// Get the relative joint angular velocity between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Universal - pointer to the joint.
//
// Return: the joint angular velocity relative to the pin0 axis.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play the creaky noise of a hanging lamp.
// 
// See also: NewtonUniversalSetUserCallback
dFloat NewtonUniversalGetJointOmega0(const NewtonJoint* joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonUniversal* const userJoint = PointerCast<NewtonUniversal, NewtonJoint>(joint);
	return userJoint->GetOmega0();
}

// Name: NewtonUniversalGetJointOmega1
// Get the relative joint angular velocity between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Universal - pointer to the joint.
//
// Return: the joint angular velocity relative to the pin1 axis.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play the creaky noise of a hanging lamp.
// 
// See also: NewtonUniversalSetUserCallback
dFloat NewtonUniversalGetJointOmega1(const NewtonJoint* joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonUniversal* const userJoint = PointerCast<NewtonUniversal, NewtonJoint>(joint);
	return userJoint->GetOmega1();
}


// Name: NewtonUniversalCalculateStopAlpha0
// Calculate the angular acceleration needed to stop the universal at the desired angle.
//
// Parameters:
// *const NewtonJoint* *Universal - pointer to the joint.
// *NewtonUniversalSliderUpdateDesc* *desc - is the pointer to the Universal or slide structure.
// *dFloat* angle - is the desired universal stop angle rotation around pin0
//
// Return: the relative angular acceleration needed to stop the universal.
//
// Remarks: this function can only be called from a *NewtonUniversalCallBack* and it can be used by the application to implement universal limits.
// 
// See also: NewtonUniversalSetUserCallback
dFloat NewtonUniversalCalculateStopAlpha0(const NewtonJoint* joint, const NewtonHingeSliderUpdateDesc* desc, dFloat angle)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonUniversal* const userJoint = PointerCast<NewtonUniversal, NewtonJoint>(joint);
	ndAssert(0);
	return 0;
}

// Name: NewtonUniversalCalculateStopAlpha1
// Calculate the angular acceleration needed to stop the universal at the desired angle.
//
// Parameters:
// *const NewtonJoint* *Universal - pointer to the joint.
// *NewtonUniversalSliderUpdateDesc* *desc - is the pointer to and the Universal or slide structure.
// *dFloat* angle - is the desired universal stop angle rotation around pin1
//
// Return: the relative angular acceleration needed to stop the universal.
//
// Remarks: this function can only be called from a *NewtonUniversalCallBack* and it can be used by the application to implement universal limits.
// 
// See also: NewtonUniversalSetUserCallback
dFloat NewtonUniversalCalculateStopAlpha1(const NewtonJoint* joint, const NewtonHingeSliderUpdateDesc* desc, dFloat angle)
{
	//dgUniversalConstraint* contraint;
	//contraint = (dgUniversalConstraint*)universal;
	//return contraint->CalculateStopAlpha1(angle, (dgJointCallBackParam*)desc);
	TRACE_FUNCTION(__FUNCTION__);
	NewtonUniversal* const userJoint = PointerCast<NewtonUniversal, NewtonJoint>(joint);
	ndAssert(0);
	return 0;
}
