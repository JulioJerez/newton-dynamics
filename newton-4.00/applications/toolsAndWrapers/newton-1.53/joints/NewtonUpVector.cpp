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

class NewtonUpVector : public ndJointUpVector
{
	public:
	D_CLASS_REFLECTION(NewtonUpVector, ndJointUpVector)

	NewtonUpVector(const ndVector& normal, ndBodyKinematic* const child, ndBodyKinematic* const parent)
		:ndJointUpVector(normal, child, parent)
	{
	}

	void JacobianDerivative(ndConstraintDescritor& desc) override
	{
		m_desc = ndWeakPtr<ndConstraintDescritor>(&desc);
		//if (m_callback)
		//{
		//	NewtonHingeSliderUpdateDesc userDesc;
		//	unsigned state = 0;
		//	userDesc.m_accel = 0.0f;
		//	userDesc.m_timestep = desc.m_timestep;
		//	userDesc.m_minFriction = -1.0e-20f;
		//	userDesc.m_maxFriction = 1.0e-20f;
		//	NewtonJoint* const joint = PointerCast<NewtonJoint, NewtonHinge>(this);
		//	m_callback(joint, &userDesc);
		//}

		ndJointUpVector::JacobianDerivative(desc);
	}

	ndWeakPtr<ndConstraintDescritor> m_desc;
};

// ***************************************************************************************************************
//
// Name: UpVector joint Interface
//
// ***************************************************************************************************************

// Name: NewtonConstraintCreateUpVector
// Create a UpVector joint. 
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const NewtonCollision* *pinDir - is the aligning vector.
// *const NewtonBody* *body - is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
//
// Return: Pointer to the up vector joint.
//
// Remarks: This function creates an up vector joint. An up vector joint is a constraint that allows a body to translate freely in 3d space,
// but it only allows the body to rotate around the pin direction vector. This could be use by the application to control a character 
// with physics and collision.
//
// Remark: Since the UpVector joint is a unary constraint, there is not need to have user callback or user data assigned to it.
// The application can simple hold to the joint handle and update the pin on the force callback function of the rigid body owning the joint.
NewtonJoint* NewtonConstraintCreateUpVector(const NewtonWorld* newtonWorld, const dFloat* pinDir, const NewtonBody* body)
{
	TRACE_FUNCTION(__FUNCTION__);

	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndBodyKinematic* const body0 = PointerCast<ndBody, NewtonBody>(body)->GetAsBodyKinematic();
	ndBodyKinematic* const body1 = world->GetSentinelBody();
	ndAssert(body0);
	ndAssert(body1);
	ndAssert(body0 != body1);

	const ndVector pin(pinDir[0], pinDir[1], pinDir[2], ndFloat32(0.0f));
	ndSharedPtr<ndJointBilateralConstraint> joint(new NewtonUpVector(pin, body0, body1));
	world->AddJoint(joint);
	return PointerCast< NewtonJoint, ndJointBilateralConstraint>(*joint);
}


// Name: NewtonUpVectorSetPin
// Set the up vector pin of this joint in global space.
//
// Parameters:
// *const NewtonJoint* *upVector - pointer to the joint.
// *dFloat* *pin - pointer to an array of a least three floats containing the up vector direction in global space.
//
// Return: nothing.
//
// Remarks: the application ca call this function to change the joint up vector, this is useful to animate the up vector.
// if the application is going to animated the up vector, it must do so by applying only small rotation, 
// too large rotation can cause vibration of the joint.
// 
// See also: NewtonUpVectorSetUserCallback, NewtonUpVectorGetPin
void NewtonUpVectorSetPin(const NewtonJoint* joint, const dFloat* pin)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonUpVector* const userJoint = PointerCast<NewtonUpVector, NewtonJoint>(joint);
	ndVector pinVector(pin[0], pin[1], pin[2], ndFloat32(0.0f));
	userJoint->SetPinDir(pinVector);
}


// Name: NewtonUpVectorGetPin
// Get the up vector pin of this joint in global space.
//
// Parameters:
// *const NewtonJoint* *upVector - pointer to the joint.
// *dFloat* *pin - pointer to an array of a least three floats to hold the up vector direction in global space.
//
// Return: nothing.
//
// Remarks: the application ca call this function to read the up vector, this is useful to animate the up vector.
// if the application is going to animated the up vector, it must do so by applying only small rotation, 
// too large rotation can cause vibration of the joint.
// 
// See also: NewtonUpVectorSetUserCallback, NewtonUpVectorSetPin
void NewtonUpVectorGetPin(const NewtonJoint* joint, dFloat* pin)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonUpVector* const userJoint = PointerCast<NewtonUpVector, NewtonJoint>(joint);
	const ndMatrix matrix(userJoint->GetLocalMatrix1());
	pin[0] = matrix.m_front.m_x;
	pin[1] = matrix.m_front.m_y;
	pin[2] = matrix.m_front.m_z;
}
