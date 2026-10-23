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

class NewtonDoubleHinge : public ndJointDoubleHinge
{
	public:
	D_CLASS_REFLECTION(NewtonDoubleHinge, ndJointDoubleHinge)

	typedef unsigned (*NewtonUniversalCallback) (const NewtonJoint* const joint, NewtonHingeSliderUpdateDesc* const desc);

	NewtonDoubleHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
		:ndJointDoubleHinge(pinAndPivotFrame, child, parent)
		,m_callback(nullptr)
	{
	}
	NewtonUniversalCallback m_callback;
};


/*!
  Set an update call back to be called when either of the two body linked by the joint is active.

  @param *universal pointer to the joint.
  @param callback pointer to the joint function call back.

  @return nothing.

  if the application wants to have some feedback from the joint simulation, the application can register a function
  update callback to be called every time any of the bodies linked by this joint is active. This is useful to provide special
  effects like particles, sound or even to simulate breakable moving parts.

  the function *NewtonUniversalCallback callback* should return a bit field code.
  if the application does not want to set the joint acceleration the return code is zero
  if the application only wants to change the joint linear acceleration the return code is 1
  if the application only wants to change the joint angular acceleration the return code is 2
  if the application only wants to change the joint angular and linear acceleration the return code is 3

  See also: ::NewtonJointGetUserData, ::NewtonJointSetUserData
*/
void NewtonUniversalSetUserCallback(const NewtonJoint* const joint, NewtonUniversalCallback callback)
{
	ndAssert(0);
	//dgUniversalConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgUniversalConstraint*)universal;
	//contraint->SetJointParameterCallback((dgUniversalJointAcceleration)callback);
}


/*!
  Get the relative joint angle between the two bodies.

  @param *universal pointer to the joint.

  @return the joint angle relative to the universal pin0.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can play a bell sound when the joint angle passes some max value.

  See also: ::NewtonUniversalSetUserCallback
*/
dFloat NewtonUniversalGetJointAngle0(const NewtonJoint* const joint)
{
	ndAssert(0);
	return 0;

	//dgUniversalConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgUniversalConstraint*)universal;
	//return contraint->GetJointAngle0();
}

/*!
  Get the relative joint angle between the two bodies.

  @param *universal pointer to the joint.

  @return the joint angle relative to the universal pin1.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can play a bell sound when the joint angle passes some max value.

  See also: ::NewtonUniversalSetUserCallback
*/
dFloat NewtonUniversalGetJointAngle1(const NewtonJoint* const joint)
{
	ndAssert(0);
	return 0;

	//dgUniversalConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgUniversalConstraint*)universal;
	//return contraint->GetJointAngle1();
}


/*!
  Get the relative joint angular velocity between the two bodies.

  @param *universal pointer to the joint.

  @return the joint angular velocity relative to the pin0 axis.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can play the creaky noise of a hanging lamp.

  See also: ::NewtonUniversalSetUserCallback
*/
dFloat NewtonUniversalGetJointOmega0(const NewtonJoint* const joint)
{
	ndAssert(0);
	return 0;

	//dgUniversalConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgUniversalConstraint*)universal;
	//return contraint->GetJointOmega0();
}


/*!
  Get the relative joint angular velocity between the two bodies.

  @param *universal pointer to the joint.

  @return the joint angular velocity relative to the pin1 axis.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can play the creaky noise of a hanging lamp.

  See also: ::NewtonUniversalSetUserCallback
*/
dFloat NewtonUniversalGetJointOmega1(const NewtonJoint* const joint)
{
	ndAssert(0);
	return 0;

	//dgUniversalConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgUniversalConstraint*)universal;
	//return contraint->GetJointOmega1();
}



/*!
  Calculate the angular acceleration needed to stop the universal at the desired angle.

  @param *universal pointer to the joint.
  @param *desc is the pointer to the Universal or slide structure.
  @param angle is the desired universal stop angle rotation around pin0

  @return the relative angular acceleration needed to stop the universal.

  this function can only be called from a *NewtonUniversalCallback* and it can be used by the application to implement universal limits.

  See also: ::NewtonUniversalSetUserCallback
*/
dFloat NewtonUniversalCalculateStopAlpha0(const NewtonJoint* const joint, const NewtonHingeSliderUpdateDesc* const desc, dFloat angle)
{
	ndAssert(0);
	return 0;

	//dgUniversalConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgUniversalConstraint*)universal;
	//return contraint->CalculateStopAlpha0(angle, (dgJointCallbackParam*)desc);
}

/*!
  Calculate the angular acceleration needed to stop the universal at the desired angle.

  @param *universal pointer to the joint.
  @param *desc is the pointer to and the Universal or slide structure.
  @param angle is the desired universal stop angle rotation around pin1

  @return the relative angular acceleration needed to stop the universal.

  this function can only be called from a *NewtonUniversalCallback* and it can be used by the application to implement universal limits.

  See also: ::NewtonUniversalSetUserCallback
*/
dFloat NewtonUniversalCalculateStopAlpha1(const NewtonJoint* const joint, const NewtonHingeSliderUpdateDesc* const desc, dFloat angle)
{
	ndAssert(0);
	return 0;

	//dgUniversalConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgUniversalConstraint*)universal;
	//return contraint->CalculateStopAlpha1(angle, (dgJointCallbackParam*)desc);
}



/*!
  Get the total force asserted over the joint pivot point, to maintain the constraint.

  @param *universal pointer to the joint.
  @param *force pointer to an array of a least three floats to hold the force value of the joint.

  @return nothing.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can destroy the joint if the force exceeds some predefined value.

  See also: ::NewtonUniversalSetUserCallback
*/
void NewtonUniversalGetJointForce(const NewtonJoint* const joint, dFloat* const force)
{
	ndAssert(0);
	//dgUniversalConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgUniversalConstraint*)universal;
	//dgVector forceVector(contraint->GetJointForce());
	//force[0] = forceVector.m_x;
	//force[1] = forceVector.m_y;
	//force[2] = forceVector.m_z;
}



/*!
  Create a universal joint.

  @param *newtonWorld Pointer to the Newton world.
  @param *pivotPoint is origin of the universal joint in global space.
  @param  *pinDir0 - first axis of rotation fixed on childBody body and perpendicular to pinDir1.
  @param  *pinDir1 - second axis of rotation fixed on parentBody body and perpendicular to pinDir0.
  @param *childBody is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
  @param *parentBody is the pointer to the parent rigid body, this body can be NULL or any kind of rigid body.

  @return Pointer to the universal joint.

  This function creates a universal joint and add it to the world. By default joint disables collision with the linked bodies.

  a universal joint is a constraint that restricts twp rigid bodies to be connected to a point fixed on both bodies,
  while and allowing one body to spin around a fix axis in is own frame, and the other body to spin around another axis fixes on
  it own frame. Both axis must be mutually perpendicular.
*/
NewtonJoint* NewtonConstraintCreateUniversal(const NewtonWorld* const newtonWorld, const dFloat* pivotPoint,
	const dFloat* pinDir0, const dFloat* pinDir1, const NewtonBody* const childBody, const NewtonBody* const parentBody)
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

	ndSharedPtr<ndJointBilateralConstraint> joint(new NewtonDoubleHinge(matrix, body0, body1));
	world->AddJoint(joint);
	return PointerCast<NewtonJoint, ndJointBilateralConstraint>(*joint);
}

void NewtonUniversalEnableLimits0(const NewtonJoint* const joint, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonDoubleHinge* const userJoint = PointerCast<NewtonDoubleHinge, NewtonJoint>(joint);
	userJoint->SetLimitState0(state ? true : false);
}

void NewtonUniversalSetLimits0(const NewtonJoint* const joint, dFloat minAngle, dFloat maxAngle)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonDoubleHinge* const userJoint = PointerCast<NewtonDoubleHinge, NewtonJoint>(joint);
	userJoint->SetLimits0(minAngle, maxAngle);
}

void NewtonUniversalEnableLimits1(const NewtonJoint* const joint, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonDoubleHinge* const userJoint = PointerCast<NewtonDoubleHinge, NewtonJoint>(joint);
	userJoint->SetLimitState1(state ? true : false);
}

void NewtonUniversalSetLimits1(const NewtonJoint* const joint, dFloat minAngle, dFloat maxAngle)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonDoubleHinge* const userJoint = PointerCast<NewtonDoubleHinge, NewtonJoint>(joint);
	userJoint->SetLimits1(minAngle, maxAngle);
}

void NewtonRollerSetSpringDamper0(const NewtonJoint* const joint, dFloat regularizer, dFloat spring, dFloat32 damper)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonDoubleHinge* const userJoint = PointerCast<NewtonDoubleHinge, NewtonJoint>(joint);
	userJoint->SetAsSpringDamper0(regularizer, spring, damper);
}

void NewtonRollerSetSpringDamper1(const NewtonJoint* const joint, dFloat regularizer, dFloat spring, dFloat32 damper)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonDoubleHinge* const userJoint = PointerCast<NewtonDoubleHinge, NewtonJoint>(joint);
	userJoint->SetAsSpringDamper1(regularizer, spring, damper);
}
