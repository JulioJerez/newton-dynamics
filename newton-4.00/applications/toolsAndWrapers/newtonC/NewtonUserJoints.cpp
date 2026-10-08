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
#include "newtonWorld.h"
#include "newtonMaterial.h"
#include "newtonBodyNotify.h"


/*!
  Create a user define bilateral joint.

  @param *newtonWorld Pointer to the Newton world.
  @param maxDOF is the maximum number of degree of freedom controlled by this joint.
  @param submitConstraints pointer to the joint constraint definition function call back.
  @param getInfo pointer to callback for collecting joint information.
  @param *childBody is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
  @param *parentBody is the pointer to the parent rigid body, this body can be NULL or any kind of rigid body.

  Bilateral joint are constraints that can have up to 6 degree of freedoms, 3 linear and 3 angular.
  By restricting the motion along any number of these degree of freedom a very large number of useful joint between
  two rigid bodies can be accomplished. Some of the degree of freedoms restriction makes no sense, and also some
  combinations are so rare that only make sense to a very specific application, the Newton engine implements the more
  commons combinations like, hinges, ball and socket, etc. However if and application is in the situation that any of
  the provided joints can achieve the desired effect, then the application can design it own joint.

  User defined joint is a very advance feature that should be look at, only for very especial situations.
  The designer must be a person with a very good understanding of constrained dynamics, and it may be the case
  that many trial have to be made before a good result can be accomplished.

  function *submitConstraints* is called before the solver state to get the jacobian derivatives and the righ hand acceleration
  for the definition of the constraint.

  maxDOF is and upper bound as to how many degrees of freedoms the joint can control, usually this value
  can be 6 for bilateral joints, but it can be higher for special joints like vehicles where by the used of friction clamping
  the number of rows can be higher.
  In general the application should determine maxDof correctly, passing an unnecessary excessive value will lead to performance decreased.

  See also: ::NewtonUserJointSetFeedbackCollectorCallback
*/
NewtonJoint* NewtonConstraintCreateUserJoint(const NewtonWorld* const newtonWorld, int maxDOF,
	NewtonUserBilateralCallback submitConstraints,
	const NewtonBody* const childBody, const NewtonBody* const parentBody)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//dgBody* const body0 = (dgBody*)childBody;
	//dgBody* const body1 = (dgBody*)parentBody;
	//dgAssert(body0);
	//dgAssert(body0 != body1);
	//return (NewtonJoint*) new (world->dgWorld::GetAllocator()) NewtonUserJoint(world, maxDOF, submitConstraints, body0, body1);
	ndAssert(0);
	return 0;
}

/*!
	Set the solver algorithm use to calculation the constraint forces.

	@param *joint pointer to the joint.
	@param  *model - solve model to choose.

	model = 0  zero is the default value and tells the solver to use the best possible algorithm
	model = 1 to signal the engine that is two joints form a kinematic loop
	model = 2 to signal the engine this joint can be solved with a less accurate algorithm.
	In case multiple joints form a kinematic loop, joints with a lower model are preffered towards an exact solution.

	See also: NewtonUserJointGetSolverModel
*/
void NewtonUserJointSetSolverModel(const NewtonJoint* const joint, int model)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgConstraint* const contraint = (dgConstraint*)joint;
	//contraint->SetSolverModel(model);

	ndAssert(0);
}

/*!
Get the solver algorithm use to calculation the constraint forces.
@param *joint pointer to the joint.

See also: NewtonUserJointGetSolverModel
*/
int NewtonUserJointGetSolverModel(const NewtonJoint* const joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgConstraint* const contraint = (dgConstraint*)joint;
	//return contraint->GetSolverModel();

	ndAssert(0);
	return 0;
}

void NewtonUserJointMassScale(const NewtonJoint* const joint, dFloat scaleBody0, dFloat scaleBody1)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const contraint = (NewtonUserJoint*)joint;
	//dgAssert(contraint->IsBilateral());
	//contraint->SetMassScale(scaleBody0, scaleBody1);
	ndAssert(0);
}

/*!
  Add a linear restricted degree of freedom.

  @param *joint pointer to the joint.
  @param  *pivot0 - pointer of a vector in global space fixed on body zero.
  @param  *pivot1 - pointer of a vector in global space fixed on body one.
  @param *dir pointer of a unit vector in global space along which the relative position, velocity and acceleration between the bodies will be driven to zero.

  A linear constraint row calculates the Jacobian derivatives and relative acceleration required to enforce the constraint condition at
  the attachment point and the pin direction considered fixed to both bodies.

  The acceleration is calculated such that the relative linear motion between the two points is zero, the application can
  afterward override this value to create motors.

  after this function is call and internal DOF index will point to the current row entry in the constraint matrix.

  This function call only be called from inside a *NewtonUserBilateralCallback* callback.

  See also: ::NewtonUserJointAddAngularRow,
*/
void NewtonUserJointAddLinearRow(const NewtonJoint* const joint, const dFloat* const pivot0, const dFloat* const pivot1, const dFloat* const dir)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const userJoint = (NewtonUserJoint*)joint;
	//
	//TRACE_FUNCTION(__FUNCTION__);
	//dgVector direction(dir[0], dir[1], dir[2], dgFloat32(0.0f));
	//direction = direction.Normalize();
	//dgAssert(dgAbs(direction.DotProduct(direction).GetScalar() - dgFloat32(1.0f)) < dgFloat32(1.0e-4f));
	//dgVector pivotPoint0(pivot0[0], pivot0[1], pivot0[2], dgFloat32(0.0f));
	//dgVector pivotPoint1(pivot1[0], pivot1[1], pivot1[2], dgFloat32(0.0f));
	//
	//userJoint->AddLinearRowJacobian(pivotPoint0, pivotPoint1, direction);
	ndAssert(0);

}


/*!
  Add an angular restricted degree of freedom.

  @param *joint pointer to the joint.
  @param relativeAngleError relative angle error between both bodies around pin axis.
  @param *pin pointer of a unit vector in global space along which the relative position, velocity and acceleration between the bodies will be driven to zero.

  An angular constraint row calculates the Jacobian derivatives and relative acceleration required to enforce the constraint condition at
  pin direction considered fixed to both bodies.

  The acceleration is calculated such that the relative angular motion between the two points is zero, The application can
  afterward override this value to create motors.

  After this function is called and internal DOF index will point to the current row entry in the constraint matrix.

  This function call only be called from inside a *NewtonUserBilateralCallback* callback.

  This function is of not practical to enforce hard constraints, but it is very useful for making angular motors.

  See also: ::NewtonUserJointAddLinearRow
*/
void NewtonUserJointAddAngularRow(const NewtonJoint* const joint, dFloat relativeAngleError, const dFloat* const pin)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const userJoint = (NewtonUserJoint*)joint;
	//dgVector direction(pin[0], pin[1], pin[2], dgFloat32(0.0f));
	//direction = direction.Normalize();
	//dgAssert(dgAbs(direction.DotProduct(direction).GetScalar() - dgFloat32(1.0f)) < dgFloat32(1.0e-3f));
	//
	//userJoint->AddAngularRowJacobian(direction, relativeAngleError);
	ndAssert(0);
}

/*!
  set the general linear and angular Jacobian for the desired degree of freedom

  @param *joint pointer to the joint.
  @param  *jacobian0 - pointer of a set of six values defining the linear and angular Jacobian for body0.
  @param  *jacobian1 - pointer of a set of six values defining the linear and angular Jacobian for body1.

  In general this function must be used for very special effects and in combination with other joints.
  it is expected that the user have a knowledge of Constrained dynamics to make a good used of this function.
  Must typical application of this function are the creation of synchronization or control joints like gears, pulleys,
  worm gear and some other mechanical control.

  this function set the relative acceleration for this degree of freedom to zero. It is the
  application responsibility to set the relative acceleration after a call to this function

  See also: ::NewtonUserJointAddLinearRow, ::NewtonUserJointAddAngularRow
*/
void NewtonUserJointAddGeneralRow(const NewtonJoint* const joint, const dFloat* const jacobian0, const dFloat* const jacobian1)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const userJoint = (NewtonUserJoint*)joint;
	//userJoint->AddGeneralRowJacobian(jacobian0, jacobian1);
	ndAssert(0);
}

int NewtonUserJoinRowsCount(const NewtonJoint* const joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const userJoint = (NewtonUserJoint*)joint;
	//return userJoint->GetJacobianCount();
	ndAssert(0);
	return 0;
}

void NewtonUserJointGetGeneralRow(const NewtonJoint* const joint, int index, dFloat* const jacobian0, dFloat* const jacobian1)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const userJoint = (NewtonUserJoint*)joint;
	//userJoint->GetJacobianAt(index, jacobian0, jacobian1);
	ndAssert(0);
}

/*!
  Set the maximum friction value the solver is allow to apply to the joint row.

  @param *joint pointer to the joint.
  @param friction maximum friction value for this row. It must be a positive value between 0.0 and INFINITY.

  This function will override the default friction values set after a call to NewtonUserJointAddLinearRow or NewtonUserJointAddAngularRow.
  friction value is context sensitive, if for linear constraint friction is a Max friction force, for angular constraint friction is a
  max friction is a Max friction torque.

  See also: ::NewtonUserJointSetRowMinimumFriction, ::NewtonUserJointAddLinearRow, ::NewtonUserJointAddAngularRow
*/
void NewtonUserJointSetRowMaximumFriction(const NewtonJoint* const joint, dFloat friction)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const userJoint = (NewtonUserJoint*)joint;
	//userJoint->SetHighFriction(friction);
	ndAssert(0);
}

/*!
  Set the minimum friction value the solver is allow to apply to the joint row.

  @param *joint pointer to the joint.
  @param friction friction value for this row. It must be a negative value between 0.0 and -INFINITY.

  This function will override the default friction values set after a call to NewtonUserJointAddLinearRow or NewtonUserJointAddAngularRow.
  friction value is context sensitive, if for linear constraint friction is a Min friction force, for angular constraint friction is a
  friction is a Min friction torque.

  See also: ::NewtonUserJointSetRowMaximumFriction, ::NewtonUserJointAddLinearRow, ::NewtonUserJointAddAngularRow
*/
void NewtonUserJointSetRowMinimumFriction(const NewtonJoint* const joint, dFloat friction)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const userJoint = (NewtonUserJoint*)joint;
	//userJoint->SetLowerFriction(friction);
	ndAssert(0);
}

/*!
  Set the value for the desired acceleration for the current constraint row.

  @param *joint pointer to the joint.
  @param acceleration desired acceleration value for this row.

  This function will override the default acceleration values set after a call to NewtonUserJointAddLinearRow or NewtonUserJointAddAngularRow.
  friction value is context sensitive, if for linear constraint acceleration is a linear acceleration, for angular constraint acceleration is an
  angular acceleration.

  See also: ::NewtonUserJointAddLinearRow, ::NewtonUserJointAddAngularRow
*/
void NewtonUserJointSetRowAcceleration(const NewtonJoint* const joint, dFloat acceleration)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const userJoint = (NewtonUserJoint*)joint;
	//userJoint->SetAcceleration(acceleration);
	ndAssert(0);
}

dFloat NewtonUserJointGetRowAcceleration(const NewtonJoint* const joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const userJoint = (NewtonUserJoint*)joint;
	//return userJoint->GetAcceleration();
	ndAssert(0);
	return 0;
}

void NewtonUserJointGetRowJacobian(const NewtonJoint* const joint, dFloat* const linear0, dFloat* const angular0, dFloat* const linear1, dFloat* const angular1)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const userJoint = (NewtonUserJoint*)joint;
	//dgJacobian jacobian0;
	//dgJacobian jacobian1;
	//userJoint->GetJacobian(jacobian0, jacobian1);
	//for (dgInt32 i = 0; i < 3; i++) {
	//	linear0[i] = jacobian0.m_linear[i];
	//	angular0[i] = jacobian0.m_angular[i];
	//	linear1[i] = jacobian1.m_linear[i];
	//	angular1[i] = jacobian1.m_angular[i];
	//}
	ndAssert(0);
}

dFloat NewtonUserJointCalculateRowZeroAcceleration(const NewtonJoint* const joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const userJoint = (NewtonUserJoint*)joint;
	//return userJoint->CalculateZeroMotorAcceleration();
	ndAssert(0);
	return 0;
}

/*!
  Calculates the row acceleration to satisfy the specified the spring damper system.

  @param *joint pointer to the joint.
  @param rowStiffness fraction of the row reaction forces used a sspring damper penalty.
  @param spring desired spring stiffness, it must be a positive value.
  @param damper desired damper coefficient, it must be a positive value.

  This function will override the default acceleration values set after a call to NewtonUserJointAddLinearRow or NewtonUserJointAddAngularRow.
  friction value is context sensitive, if for linear constraint acceleration is a linear acceleration, for angular constraint acceleration is an
  angular acceleration.

  the acceleration calculated by this function represent the mass, spring system of the form
  a = -ks * x - kd * v.

  for this function to take place the joint stiffness must be set to a values lower than 1.0

  See also: ::NewtonUserJointSetRowAcceleration, ::NewtonUserJointSetRowStiffness
*/
void NewtonUserJointSetRowMassIndependentSpringDamperAcceleration(const NewtonJoint* const joint, dFloat rowStiffness, dFloat spring, dFloat damper)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const userJoint = (NewtonUserJoint*)joint;
	//userJoint->SetMassIndependentSpringDamperAcceleration(rowStiffness, spring, damper);
	ndAssert(0);
}

void NewtonUserJointSetRowMassDependentSpringDamperAcceleration(const NewtonJoint* const joint, dFloat spring, dFloat damper)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const userJoint = (NewtonUserJoint*)joint;
	//userJoint->SetMassDependentSpringDamperAcceleration(spring, damper);
	ndAssert(0);
}

/*!
  Set the maximum percentage of the constraint force that will be applied to the constraint row.

  @param *joint pointer to the joint.
  @param stiffness row stiffness, it must be a values between 0.0 and 1.0, the default is 0.9.

  This function will override the default stiffness value set after a call to NewtonUserJointAddLinearRow or NewtonUserJointAddAngularRow.
  the row stiffness is the percentage of the constraint force that will be applied to the rigid bodies. Ideally the value should be
  1.0 (100% stiff) but dues to numerical integration error this could be the joint a little unstable, and lower values are preferred.

  See also: ::NewtonUserJointAddLinearRow, ::NewtonUserJointAddAngularRow, ::NewtonUserJointSetMassIndependentRowSpringDamperAcceleration
*/
void NewtonUserJointSetRowStiffness(const NewtonJoint* const joint, dFloat stiffness)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const userJoint = (NewtonUserJoint*)joint;
	//userJoint->SetRowStiffness(dgFloat32(1.0f) - stiffness);
	ndAssert(0);
}

/*!
  Return the magnitude previews force or torque value calculated by the solver for this constraint row.

  @param *joint pointer to the joint.
  @param row index to the constraint row.

  This function can be call for any of the previews row for this particular joint, The application must keep track of the meaning of the row.

  This function can be used to produce special effects like breakable or malleable joints, fro example a hinge can turn into ball and socket
  after the force in some of the row exceed  certain high value.
*/
dFloat NewtonUserJointGetRowForce(const NewtonJoint* const joint, int row)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const userJoint = (NewtonUserJoint*)joint;
	//return userJoint->GetRowForce(row);
	ndAssert(0);
	return 0;
}


/*!
  Set a constrain callback to collect the force calculated by the solver to enforce this constraint

  @param *joint pointer to the joint.
  @param getFeedback pointer to the joint constraint definition function call back.

  See also: ::NewtonUserJointGetRowForce
*/
void NewtonUserJointSetFeedbackCollectorCallback(const NewtonJoint* const joint, NewtonUserBilateralCallback getFeedback)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonUserJoint* const userJoint = (NewtonUserJoint*)joint;
	//return userJoint->SetUpdateFeedbackFunction(getFeedback);
	ndAssert(0);
}
