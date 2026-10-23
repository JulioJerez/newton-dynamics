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

class NewtonCylindrical : public ndJointCylinder
{
	public:
	D_CLASS_REFLECTION(NewtonCylindrical, ndJointCylinder)

	typedef unsigned (*NewtonCorkscrewCallback) (const NewtonJoint* const corkscrew, NewtonHingeSliderUpdateDesc* const desc);

	NewtonCylindrical(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
		:ndJointCylinder(pinAndPivotFrame, child, parent)
		,m_callback(nullptr)
	{
	}
	NewtonCorkscrewCallback m_callback;
};



/*!
  Set an update call back to be called when either of the two body linked by the joint is active.

  @param *corkscrew pointer to the joint.
  @param callback pointer to the joint function call back.

  @return nothing.

  if the application wants to have some feedback from the joint simulation, the application can register a function
  update callback to be call every time any of the bodies linked by this joint is active. This is useful to provide special
  effects like particles, sound or even to simulate breakable moving parts.

  the function *NewtonCorkscrewCallback callback* should return a bit field code.
  if the application does not want to set the joint acceleration the return code is zero
  if the application only wants to change the joint linear acceleration the return code is 1
  if the application only wants to change the joint angular acceleration the return code is 2
  if the application only wants to change the joint angular and linear acceleration the return code is 3

  See also: ::NewtonJointGetUserData, ::NewtonJointSetUserData
*/
void NewtonCorkscrewSetUserCallback(const NewtonJoint* const corkscrew, NewtonCorkscrewCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//dgCorkscrewConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgCorkscrewConstraint*)corkscrew;
	//contraint->SetJointParameterCallback((dgCorkscrewJointAcceleration)callback);
}

/*!
  Get the relative joint angle between the two bodies.

  @param *corkscrew pointer to the joint.

  @return the joint angle relative to the hinge pin.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can play a bell sound when the joint angle passes some max value.

  See also: ::NewtonCorkscrewSetUserCallback
*/
dFloat NewtonCorkscrewGetJointPosit(const NewtonJoint* const corkscrew)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//dgCorkscrewConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgCorkscrewConstraint*)corkscrew;
	//return contraint->GetJointPosit();
	return 0;
}

/*!
  Get the relative joint angular velocity between the two bodies.

  @param *corkscrew pointer to the joint.

  @return the joint angular velocity relative to the pin axis.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can play the creaky noise of a hanging lamp.

  See also: ::NewtonCorkscrewSetUserCallback
*/
dFloat NewtonCorkscrewGetJointVeloc(const NewtonJoint* const corkscrew)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//dgCorkscrewConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgCorkscrewConstraint*)corkscrew;
	//return contraint->GetJointVeloc();
	return 0;
}

/*!
  Get the relative joint angle between the two bodies.

  @param *corkscrew pointer to the joint.

  @return the joint angle relative to the corkscrew pin.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can play a bell sound when the joint angle passes some max value.

  See also: ::NewtonCorkscrewSetUserCallback
*/
dFloat NewtonCorkscrewGetJointAngle(const NewtonJoint* const corkscrew)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//dgCorkscrewConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgCorkscrewConstraint*)corkscrew;
	//return contraint->GetJointAngle();
	return 0;
}

/*!
  Get the relative joint angular velocity between the two bodies.

  @param *corkscrew pointer to the joint.

  @return the joint angular velocity relative to the pin axis.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can play the creaky noise of a hanging lamp.

  See also: ::NewtonCorkscrewSetUserCallback
*/
dFloat NewtonCorkscrewGetJointOmega(const NewtonJoint* const corkscrew)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//dgCorkscrewConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgCorkscrewConstraint*)corkscrew;
	//return contraint->GetJointOmega();
	return 0;
}


/*!
  Calculate the angular acceleration needed to stop the corkscrew at the desired angle.

  @param *corkscrew pointer to the joint.
  @param *desc is the pointer to the Corkscrew or slide structure.
  @param angle is the desired corkscrew stop angle

  @return the relative angular acceleration needed to stop the corkscrew.

  this function can only be called from a *NewtonCorkscrewCallback* and it can be used by the application to implement corkscrew limits.

  See also: ::NewtonCorkscrewSetUserCallback
*/
dFloat NewtonCorkscrewCalculateStopAlpha(const NewtonJoint* const corkscrew, const NewtonHingeSliderUpdateDesc* const desc, dFloat angle)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//dgCorkscrewConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgCorkscrewConstraint*)corkscrew;
	//return contraint->CalculateStopAlpha(angle, (dgJointCallbackParam*)desc);
	return 0;
}


/*!
  Calculate the angular acceleration needed to stop the corkscrew at the desired angle.

  @param *corkscrew pointer to the joint.
  @param *desc is the pointer to the Corkscrew or slide structure.
  @param distance desired stop distance relative to the pivot point

  @return the relative linear acceleration needed to stop the corkscrew.

  this function can only be called from a *NewtonCorkscrewCallback* and it can be used by the application to implement corkscrew limits.

  See also: ::NewtonCorkscrewSetUserCallback
*/
dFloat NewtonCorkscrewCalculateStopAccel(const NewtonJoint* const corkscrew, const NewtonHingeSliderUpdateDesc* const desc, dFloat distance)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//dgCorkscrewConstraint* contraint;
	//contraint = (dgCorkscrewConstraint*)corkscrew;
	//return contraint->CalculateStopAccel(distance, (dgJointCallbackParam*)desc);
	return 0;
}

/*!
  Get the total force asserted over the joint pivot point, to maintain the constraint.

  @param *corkscrew pointer to the joint.
  @param *force pointer to an array of a least three floats to hold the force value of the joint.

  @return nothing.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can destroy the joint if the force exceeds some predefined value.

  See also: ::NewtonCorkscrewSetUserCallback
*/
void NewtonCorkscrewGetJointForce(const NewtonJoint* const corkscrew, dFloat* const force)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//dgCorkscrewConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgCorkscrewConstraint*)corkscrew;
	//dgVector forceVector(contraint->GetJointForce());
	//force[0] = forceVector.m_x;
	//force[1] = forceVector.m_y;
	//force[2] = forceVector.m_z;
}

/*!
  Create a corkscrew joint.

  @param *newtonWorld Pointer to the Newton world.
  @param *pivotPoint is origin of the corkscrew in global space.
  @param *pinDir is the line of action of the corkscrew in global space.
  @param *childBody is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
  @param *parentBody is the pointer to the parent rigid body, this body can be NULL or any kind of rigid body.

  @return Pointer to the corkscrew joint.

  This function creates a corkscrew and add it to the world. By default joint disables collision with the linked bodies.
*/
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
	ndSharedPtr<ndJointBilateralConstraint> joint(new NewtonCylindrical(matrix, body0, body1));
	world->AddJoint(joint);
	return PointerCast<NewtonJoint, ndJointBilateralConstraint>(*joint);
}

void NewtonCorkscrewEnableLinearLimits(const NewtonJoint* const joint, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonCylindrical* const userJoint = PointerCast<NewtonCylindrical, NewtonJoint>(joint);
	userJoint->SetLimitStatePosit(state ? true : false);
}

void NewtonCorkscrewSetLinearLimits(const NewtonJoint* const joint, dFloat minPosit, dFloat maxPosit)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonCylindrical* const userJoint = PointerCast<NewtonCylindrical, NewtonJoint>(joint);
	userJoint->SetLimitsPosit(minPosit, maxPosit);
}

void NewtonCorkscrewEnableAngularLimits(const NewtonJoint* const joint, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonCylindrical* const userJoint = PointerCast<NewtonCylindrical, NewtonJoint>(joint);
	userJoint->SetLimitStateAngle(state ? true : false);
}

void NewtonCorkscrewSetAngularLimits(const NewtonJoint* const joint, dFloat minTwistAngle, dFloat maxTwistAngle)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonCylindrical* const userJoint = PointerCast<NewtonCylindrical, NewtonJoint>(joint);
	userJoint->SetLimitsAngle(minTwistAngle, maxTwistAngle);
}
