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

void NewtonSetWorldSize(const NewtonWorld* newtonWorld, const dFloat* minPoint, const dFloat* maxPoint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//do nothing;
	ndTrace(("deprecated: there not world bounds in newton 4\n"));
}

void NewtonSetMinimumFrameRate(const NewtonWorld* newtonWorld, dFloat frameRate)
{
	TRACE_FUNCTION(__FUNCTION__);
	//do nothing;
	//ndTrace(("deprecated function: %s\n", __FUNCDNAME__));
}

/*!
  Set the default softness coefficients for the material interaction between two physics materials .

  @param *newtonWorld pointer to the Newton world.
  @param  id0 - group id0
  @param  id1 - group id1
  @param softnessCoef softness coefficient

  @return Nothing.

  *softnessCoef* must be a positive value.
  It is recommended that *softnessCoef* be set to value lower or equal to 1.0
  A low value for *softnessCoef* will make the material soft. A typical value for *softnessCoef* is 0.15
*/
void NewtonMaterialSetDefaultSoftness(const NewtonWorld* const newtonWorld, int id0, int id1, dFloat softnessCoef)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndTrace(("deprecated function: %s\n", __FUNCDNAME__));
}

/*!
  Set the strength coefficient to be applied to the joint reaction forces.

  @param *joint pointer to the joint.
  @param stiffness stiffness coefficient, a value between 0, and 1.0, the default value for most joint is 0.9

  @return nothing.

  Constraint keep bodies together by calculating the exact force necessary to cancel the relative acceleration between one or
  more common points fixed in the two bodies. The problem is that when the bodies drift apart due to numerical integration inaccuracies,
  the reaction force work to pull eliminated the error but at the expense of adding extra energy to the system, does violating the rule
  that constraint forces must be work less. This is a inevitable situation and the only think we can do is to minimize the effect of the
  extra energy by dampening the force by some amount. In essence the stiffness coefficient tell Newton calculate the precise reaction force
  by only apply a fraction of it to the joint point. And value of 1.0 will apply the exact force, and a value of zero will apply only
  10 percent.

  The stiffness is set to a all around value that work well for most situation, however the application can play with these
  parameter to make finals adjustment. A high value will make the joint stronger but more prompt to vibration of instability; a low
  value will make the joint more stable but weaker.

  See also: ::NewtonJointGetStiffness
*/
void NewtonJointSetStiffness(const NewtonJoint* const jointPtr, dFloat stiffness)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndTrace(("deprecated function: %s\n", __FUNCDNAME__));
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
dFloat NewtonHingeCalculateStopAlpha(const NewtonJoint* joint, const NewtonHingeSliderUpdateDesc* desc, dFloat angle)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndTrace(("deprecated function: %s\n", __FUNCDNAME__));
	ndTrace(("please use the joint limits or motor\n"));
	ndAssert(0);
	return ndFloat32(0.0f);
}


// Name: NewtonSliderGetJointForce
// Calculate the angular acceleration needed to stop the slider at the desired angle.
//
// Parameters:
// *const NewtonJoint* *slider - pointer to the joint.
// *NewtonSliderSliderUpdateDesc* *desc - is the pointer to the Slider or slide structure.
// *dFloat* distance - desired stop distance relative to the pivot point
//
// Return: the relative linear acceleration needed to stop the slider.
//
// Remarks: this function can only be called from a *NewtonSliderCallBack* and it can be used by the application to implement slider limits.
// 
// See also: NewtonSliderSetUserCallback
dFloat NewtonSliderCalculateStopAccel(const NewtonJoint* joint, const NewtonHingeSliderUpdateDesc* desc, dFloat distance)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndTrace(("deprecated function: %s\n", __FUNCDNAME__));
	ndTrace(("please use the joint limits or motor\n"));
	ndAssert(0);
	return ndFloat32(0.0f);
}

// Name: NewtonCorkscrewCalculateStopAlpha
// Calculate the angular acceleration needed to stop the corkscrew at the desired angle.
//
// Parameters:
// *const NewtonJoint* *Corkscrew - pointer to the joint.
// *NewtonCorkscrewSliderUpdateDesc* *desc - is the pointer to the Corkscrew or slide structure.
// *dFloat* angle - is the desired corkscrew stop angle 
//
// Return: the relative angular acceleration needed to stop the corkscrew.
//
// Remarks: this function can only be called from a *NewtonCorkscrewCallBack* and it can be used by the application to implement corkscrew limits.
// 
// See also: NewtonCorkscrewSetUserCallback
dFloat NewtonCorkscrewCalculateStopAlpha(const NewtonJoint* joint, const NewtonHingeSliderUpdateDesc* desc, dFloat angle)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndTrace(("deprecated function: %s\n", __FUNCDNAME__));
	ndTrace(("please use the joint limits or motor\n"));
	ndAssert(0);
	return ndFloat32(0.0f);
}

// Name: NewtonCorkscrewGetJointForce
// Calculate the angular acceleration needed to stop the corkscrew at the desired angle.
//
// Parameters:
// *const NewtonJoint* *corkscrew - pointer to the joint.
// *NewtonCorkscrewCorkscrewUpdateDesc* *desc - is the pointer to the Corkscrew or slide structure.
// *dFloat* distance - desired stop distance relative to the pivot point
//
// Return: the relative linear acceleration needed to stop the corkscrew.
//
// Remarks: this function can only be called from a *NewtonCorkscrewCallBack* and it can be used by the application to implement corkscrew limits.
// 
// See also: NewtonCorkscrewSetUserCallback
dFloat NewtonCorkscrewCalculateStopAccel(const NewtonJoint* joint, const NewtonHingeSliderUpdateDesc* desc, dFloat distance)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndTrace(("deprecated function: %s\n", __FUNCDNAME__));
	ndTrace(("please use the joint limits or motor\n"));
	ndAssert(0);
	return ndFloat32(0.0f);
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
	ndTrace(("deprecated function: %s\n", __FUNCDNAME__));
	ndTrace(("please use the joint limits or motor\n"));
	ndAssert(0);
	return ndFloat32(0.0f);
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
	ndTrace(("deprecated function: %s\n", __FUNCDNAME__));
	ndTrace(("please use the joint limits or motor\n"));
	ndAssert(0);
	return ndFloat32(0.0f);
}

// Name: NewtonBodySetAutoactiveCallback 
// Assign an activation/deativation callback function.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *NewtonBodyActivationStatem callback - pointer to a function callback is used to be called when the body is about to be activated or deactivated.
//
// Return: Nothing.
//
// Remarks: this function sets an activation and deactivation callback with the body. The application can use this method to implement advanced scene management control.
void NewtonBodySetAutoactiveCallback(const NewtonBody* bodyPtr, NewtonBodyActivationState callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndTrace(("deprecated function: %s\n", __FUNCDNAME__));
}

// Name: NewtonBodySetContinuousCollisionMode 
// Set the continuous collision state mode for this rigid body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *int* state - collision state. 1 indicates this body may tunnel through other objects while moving at high speed. 0 ignore high speed collision checks.
//
// Return: Nothing.
//
// Remarks: continue collision mode enable allow the engine to predict colliding contact on rigid bodies
// Moving at high speed of subject to strong forces.
//
// Remarks: continue collision mode does not prevent rigid bodies from inter penetration instead it prevent bodies from 
// passing trough each others by extrapolating contact points when the bodies normal contact calculation determine the bodies are not colliding. 
//
// Remarks: for performance reason the bodies angular velocities is only use on the broad face of the collision, 
// but not on the contact calculation. 
// 
// Remarks: continue collision does not perform back tracking to determine time of contact, instead it extrapolate contact by incrementally 
// extruding the collision geometries of the two colliding bodies along the linear velocity of the bodies during the time step, 
// if during the extrusion colliding contact are found, a collision is declared and the normal contact resolution is called. 
//
// Remarks: for continue collision to be active the continue collision mode must on the material pair of the colliding bodies as well as on at least one of the two colliding bodies.
//
// Remarks: Because there is penalty of about 40% to 80% depending of the shape complexity of the collision geometry, this feature is set 
// off by default. It is the job of the application to determine what bodies need this feature on. Good guidelines are: very small objects, 
// and bodies that move a height speed.  
// 
// See also: NewtonBodyGetContinuousCollisionMode, NewtonBodySetContinuousCollisionMode
void NewtonBodySetContinuousCollisionMode(const NewtonBody* body, unsigned state)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndTrace(("deprecated function: %s\n", __FUNCDNAME__));
}

// Name: NewtonBodyGetContinuousCollisionMode 
// Get the continuous collision state mode for this rigid body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
//
// Return: Nothing.
//
// Remarks: 
//Remark: Because there is there is penalty of about 3 to 5 depending of the shape complexity of the collision geometry, this feature is set 
//off by default. It is the job of the application to determine what bodies need this feature on. Good guidelines are: very small objects, 
//and bodies that move a height speed.  
//
//Remark: this feature is currently disabled:
//
// See also: NewtonBodySetContinuousCollisionMode, NewtonBodySetContinuousCollisionMode
int NewtonBodyGetContinuousCollisionMode(const NewtonBody* bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndTrace(("deprecated function: %s\n", __FUNCDNAME__));
	return 0;
}
