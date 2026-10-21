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

class NewtonBallAndSockect: public ndJointSpherical
{
	public:
	D_CLASS_REFLECTION(NewtonBallAndSockect, ndJointSpherical)

	NewtonBallAndSockect()
		:ndJointSpherical()
	{
	}
};

/*!
  Create a ball an socket joint.

  @param *newtonWorld Pointer to the Newton world.
  @param *pivotPoint is origin of ball and socket in global space.
  @param *childBody is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
  @param *parentBody is the pointer to the parent rigid body, this body can be NULL or any kind of rigid body.

  @return Pointer to the ball and socket joint.

  This function creates a ball and socket and add it to the world. By default joint disables collision with the linked bodies.
*/
NewtonJoint* NewtonConstraintCreateBall(const NewtonWorld* const newtonWorld,
	const dFloat* pivotPoint,
	const NewtonBody* const childBody,
	const NewtonBody* const parentBody)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return nullptr;
	//Newton* const world = (Newton*)newtonWorld;
	//dgBody* const body0 = (dgBody*)childBody;
	//dgBody* const body1 = (dgBody*)parentBody;
	//dgVector pivot(pivotPoint[0], pivotPoint[1], pivotPoint[2], dgFloat32(0.0f));
	//return (NewtonJoint*)world->CreateBallConstraint(pivot, body0, body1);
}

/*!
  Set the ball and socket cone and twist limits.

  @param *ball is the pointer to a ball and socket joint.
  @param *pin pointer to a unit vector defining the cone axis in global space.
  @param maxConeAngle max angle in radians the attached body is allow to swing relative to the pin axis, a value of zero will disable this limits.
  @param maxTwistAngle max angle in radians the attached body is allow to twist relative to the pin axis, a value of zero will disable this limits.

  limits are disabled at creation time. A value of zero for *maxConeAngle* disable the cone limit, a value of zero for *maxTwistAngle* disable the twist limit
  all non-zero value for *maxConeAngle* are clamped between 5 degree and 175 degrees

  See also: ::NewtonConstraintCreateBall
*/
void NewtonBallSetConeLimits(const NewtonJoint* const ball, const dFloat* pin, dFloat maxConeAngle, dFloat maxTwistAngle)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//dgBallConstraint* const joint = (dgBallConstraint*)ball;
	//
	//dgVector coneAxis(pin[0], pin[1], pin[2], dgFloat32(0.0f));
	//
	//if (coneAxis.DotProduct(coneAxis).GetScalar() < 1.0e-3f) {
	//	coneAxis.m_x = dgFloat32(1.0f);
	//}
	//dgVector tmp(dgFloat32(1.0f), dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f));
	//if (dgAbs(tmp.DotProduct(coneAxis).GetScalar()) > dgFloat32(0.999f)) {
	//	tmp = dgVector(dgFloat32(0.0f), dgFloat32(1.0f), dgFloat32(0.0f), dgFloat32(0.0f));
	//	if (dgAbs(tmp.DotProduct(coneAxis).GetScalar()) > dgFloat32(0.999f)) {
	//		tmp = dgVector(dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(1.0f), dgFloat32(0.0f));
	//		dgAssert(dgAbs(tmp.DotProduct(coneAxis).GetScalar()) < dgFloat32(0.999f));
	//	}
	//}
	//dgVector lateral(tmp.CrossProduct(coneAxis));
	//dgAssert(lateral.m_w == dgFloat32(0.0f));
	//dgAssert(coneAxis.m_w == dgFloat32(0.0f));
	//lateral = lateral.Normalize();
	//coneAxis = coneAxis.Normalize();
	//
	//maxConeAngle = dgAbs(maxConeAngle);
	//maxTwistAngle = dgAbs(maxTwistAngle);
	//joint->SetConeLimitState((maxConeAngle > dgDegreeToRad) ? true : false);
	//joint->SetTwistLimitState((maxTwistAngle > dgDegreeToRad) ? true : false);
	//joint->SetLatealLimitState(false);
	//joint->SetLimits(coneAxis, -maxConeAngle, maxConeAngle, maxTwistAngle, lateral, 0.0f, 0.0f);
}


/*!
  Set an update call back to be called when either of the two bodies linked by the joint is active.

  @param *ball pointer to the joint.
  @param callback pointer to the joint function call back.

  @return nothing.

  if the application wants to have some feedback from the joint simulation, the application can register a function
  update callback to be called every time any of the bodies linked by this joint is active. This is useful to provide special
  effects like particles, sound or even to simulate breakable moving parts.

  See also: ::NewtonJointSetUserData
*/
void NewtonBallSetUserCallback(const NewtonJoint* const ball, NewtonBallCallback callback)
{
	ndAssert(0);
	//dgBallConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgBallConstraint*)ball;
	//contraint->SetJointParameterCallback((dgBallJointFriction)callback);
}


/*!
  Get the relative joint angle between the two bodies.

  @param *ball pointer to the joint.
  @param *angle pointer to an array of a least three floats to hold the joint relative Euler angles.

  @return nothing.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can play a bell sound when the joint angle passes some max value.

  See also: ::NewtonBallSetUserCallback
*/
void NewtonBallGetJointAngle(const NewtonJoint* const ball, dFloat* angle)
{
	ndAssert(0);
	//dgBallConstraint* contraint;
	//
	//contraint = (dgBallConstraint*)ball;
	//dgVector angleVector(contraint->GetJointAngle());
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//angle[0] = angleVector.m_x;
	//angle[1] = angleVector.m_y;
	//angle[2] = angleVector.m_z;
}

/*!
  Get the relative joint angular velocity between the two bodies.

  @param *ball pointer to the joint.
  @param *omega pointer to an array of a least three floats to hold the joint relative angular velocity.

  @return nothing.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can play the creaky noise of a hanging lamp.

  See also: ::NewtonBallSetUserCallback
*/
void NewtonBallGetJointOmega(const NewtonJoint* const ball, dFloat* omega)
{
	ndAssert(0);
	//dgBallConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgBallConstraint*)ball;
	//dgVector omegaVector(contraint->GetJointOmega());
	//omega[0] = omegaVector.m_x;
	//omega[1] = omegaVector.m_y;
	//omega[2] = omegaVector.m_z;
}

/*!
  Get the total force asserted over the joint pivot point, to maintain the constraint.

  @param *ball pointer to the joint.
  @param *force pointer to an array of a least three floats to hold the force value of the joint.

  @return nothing.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can destroy the joint if the force exceeds some predefined value.

  See also: ::NewtonBallSetUserCallback
*/
void NewtonBallGetJointForce(const NewtonJoint* const ball, dFloat* const force)
{
	ndAssert(0);
	//// fixme: type? "constraint" instead of "contraint"?
	//dgBallConstraint* contraint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//contraint = (dgBallConstraint*)ball;
	//dgVector forceVector(contraint->GetJointForce());
	//force[0] = forceVector.m_x;
	//force[1] = forceVector.m_y;
	//force[2] = forceVector.m_z;
}
