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

class NewtonCylinder: public ndJointCylinder
{
	public:
	D_CLASS_REFLECTION(NewtonCylinder, ndJointCylinder)

	typedef unsigned (*NewtonCorkscrewCallBack) (const NewtonJoint* joint, NewtonHingeSliderUpdateDesc* desc);

	NewtonCylinder(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
		:ndJointCylinder(pinAndPivotFrame, child, parent)
		,m_desc(nullptr)
		,m_callback(nullptr)
	{
	}

	void JacobianDerivative(ndConstraintDescritor& desc) override
	{
		m_desc = ndWeakPtr<ndConstraintDescritor>(&desc);
		if (m_callback)
		{
			NewtonHingeSliderUpdateDesc userDesc[2];
			unsigned state = 0;
			userDesc[0].m_accel = 0.0f;
			userDesc[0].m_timestep = desc.m_timestep;
			userDesc[0].m_minFriction = -1.0e-20f;
			userDesc[0].m_maxFriction = 1.0e-20f;
			userDesc[1].m_accel = 0.0f;
			userDesc[1].m_timestep = desc.m_timestep;
			userDesc[1].m_minFriction = -1.0e-20f;
			userDesc[1].m_maxFriction = 1.0e-20f;

			NewtonJoint* const joint = PointerCast<NewtonJoint, NewtonCylinder>(this);
			m_callback(joint, userDesc);
		}

		ndJointCylinder::JacobianDerivative(desc);
	}

	ndWeakPtr<ndConstraintDescritor> m_desc;
	NewtonCorkscrewCallBack m_callback;
};

//
// Create a hinge joint.
//
NewtonJoint* NewtonConstraintCreateCorkscrew(const NewtonWorld* const newtonWorld, const dFloat* pivotPoint, const dFloat* pinDir, const NewtonBody* const childBody, const NewtonBody* const parentBody)
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
	ndSharedPtr<ndJointBilateralConstraint> joint(new NewtonCylinder(matrix, body0, body1));
	world->AddJoint(joint);
	return PointerCast<NewtonJoint, ndJointBilateralConstraint>(*joint);
}

// Name: NewtonCorkscrewSetUserCallback
// Set an update call back to be called when either of the two body linked by the joint is active.
//
// Parameters:
// *const NewtonJoint* *Corkscrew - pointer to the joint.
// *NewtonCorkscrewCallBack* callback - pointer to the joint function call back.
//
// Return: nothing.
//
// Remarks: if the application wants to have some feedback from the joint simulation, the application can register a function
// update callback to be call every time any of the bodies linked by this joint is active. This is useful to provide special
// effects like particles, sound or even to simulate breakable moving parts.
//
// Remarks: the function *NewtonCorkscrewCallBack callback* should return a bit field code.
// if the application does not want to set the joint acceleration the return code is zero
// if the application only wants to change the joint linear acceleration the return code is 1
// if the application only wants to change the joint angular acceleration the return code is 2
// if the application only wants to change the joint angular and linear acceleration the return code is 3
// 
// See also: NewtonJointGetUserData, NewtonJointSetUserData
void NewtonCorkscrewSetUserCallback(const NewtonJoint* joint, NewtonCorkscrewCallBack callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonCylinder* const userJoint = PointerCast<NewtonCylinder, NewtonJoint>(joint);
	userJoint->m_callback = callback;
}

// Name: NewtonCorkscrewGetJointPosit
// Get the relative joint angle between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Corkscrew - pointer to the joint.
//
// Return: the joint angle relative to the hinge pin.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play a bell sound when the joint angle passes some max value.
// 
// See also: NewtonCorkscrewSetUserCallback
dFloat NewtonCorkscrewGetJointPosit(const NewtonJoint* joint)
{
	NewtonCylinder* const userJoint = PointerCast<NewtonCylinder, NewtonJoint>(joint);
	return userJoint->GetPosit();
}

// Name: NewtonCorkscrewGetJointVeloc
// Get the relative joint angular velocity between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Corkscrew - pointer to the joint.
//
// Return: the joint angular velocity relative to the pin axis.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play the creaky noise of a hanging lamp.
// 
// See also: NewtonCorkscrewSetUserCallback
dFloat NewtonCorkscrewGetJointVeloc(const NewtonJoint* joint)
{
	NewtonCylinder* const userJoint = PointerCast<NewtonCylinder, NewtonJoint>(joint);
	return userJoint->GetSpeed();
}

// Name: NewtonCorkscrewGetJointAngle
// Get the relative joint angle between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Corkscrew - pointer to the joint.
//
// Return: the joint angle relative to the corkscrew pin.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play a bell sound when the joint angle passes some max value.
// 
// See also: NewtonCorkscrewSetUserCallback
dFloat NewtonCorkscrewGetJointAngle(const NewtonJoint* joint)
{
	NewtonCylinder* const userJoint = PointerCast<NewtonCylinder, NewtonJoint>(joint);
	return userJoint->GetAngle();
}

// Name: NewtonCorkscrewGetJointOmega
// Get the relative joint angular velocity between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Corkscrew - pointer to the joint.
//
// Return: the joint angular velocity relative to the pin axis.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play the creaky noise of a hanging lamp.
// 
// See also: NewtonCorkscrewSetUserCallback
dFloat NewtonCorkscrewGetJointOmega(const NewtonJoint* joint)
{
	NewtonCylinder* const userJoint = PointerCast<NewtonCylinder, NewtonJoint>(joint);
	return userJoint->GetOmega();
}

void NewtonCorkscrewSetAsMotorAngular(const NewtonJoint* const joint, dFloat speed, dFloat maxTorque)
{
	NewtonCylinder* const userJoint = PointerCast<NewtonCylinder, NewtonJoint>(joint);
	userJoint->SetAsMotorAngle(speed, maxTorque);
}

void NewtonCorkscrewEnableLimits(const NewtonJoint* const joint, int state)
{
	NewtonCylinder* const userJoint = PointerCast<NewtonCylinder, NewtonJoint>(joint);
	userJoint->SetLimitStatePosit(state ? true : false);
}

void NewtonCorkscrewSetLimits(const NewtonJoint* const joint, dFloat minPosit, dFloat maxPosit)
{
	NewtonCylinder* const userJoint = PointerCast<NewtonCylinder, NewtonJoint>(joint);
	userJoint->SetLimitsPosit(minPosit, maxPosit);
}
