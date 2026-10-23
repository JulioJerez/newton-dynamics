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

class NewtonUpVector: public ndJointUpVector
{
	public:
	D_CLASS_REFLECTION(NewtonUpVector, ndJointUpVector)

	NewtonUpVector(const ndVector& normal, ndBodyKinematic* const child, ndBodyKinematic* const parent)
		:ndJointUpVector(normal, child, parent)
	{
	}
};


/*!
  Create a UpVector joint.

  @param *newtonWorld Pointer to the Newton world.
  @param *pinDir is the aligning vector.
  @param *body is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.

  @return Pointer to the up vector joint.

  This function creates an up vector joint. An up vector joint is a constraint that allows a body to translate freely in 3d space,
  but it only allows the body to rotate around the pin direction vector. This could be use by the application to control a character
  with physics and collision.

  Since the UpVector joint is a unary constraint, there is not need to have user callback or user data assigned to it.
  The application can simple hold to the joint handle and update the pin on the force callback function of the rigid body owning the joint.
*/
NewtonJoint* NewtonConstraintCreateUpVector(const NewtonWorld* const newtonWorld, const dFloat* pinDir, const NewtonBody* const body)
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


/*!
  Get the up vector pin of this joint in global space.

  @param *upVector pointer to the joint.
  @param *pin pointer to an array of a least three floats to hold the up vector direction in global space.

  @return nothing.

  the application ca call this function to read the up vector, this is useful to animate the up vector.
  if the application is going to animated the up vector, it must do so by applying only small rotation,
  too large rotation can cause vibration of the joint.

  See also: ::NewtonUpVectorSetPin
*/
void NewtonUpVectorGetPin(const NewtonJoint* const upVector, dFloat* pin)
{
	ndAssert(0);
	//dgUpVectorConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgUpVectorConstraint*)upVector;
	//
	//dgVector pinVector(contraint->GetPinDir());
	//pin[0] = pinVector.m_x;
	//pin[1] = pinVector.m_y;
	//pin[2] = pinVector.m_z;
}


/*!
  Set the up vector pin of this joint in global space.

  @param *upVector pointer to the joint.
  @param *pin pointer to an array of a least three floats containing the up vector direction in global space.

  @return nothing.

  the application ca call this function to change the joint up vector, this is useful to animate the up vector.
  if the application is going to animated the up vector, it must do so by applying only small rotation,
  too large rotation can cause vibration of the joint.

  See also: ::NewtonUpVectorGetPin
*/
void NewtonUpVectorSetPin(const NewtonJoint* const upVector, const dFloat* pin)
{
	ndAssert(0);
	//dgUpVectorConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgUpVectorConstraint*)upVector;
	//
	//dgVector pinVector(pin[0], pin[1], pin[2], dgFloat32(0.0f));
	//contraint->SetPinDir(pinVector);
}

