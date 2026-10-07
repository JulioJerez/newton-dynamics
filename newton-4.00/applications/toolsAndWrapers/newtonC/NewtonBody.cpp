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
  Create a rigid body.

  @param *newtonWorld Pointer to the Newton world.
  @param *collisionPtr pointer to the collision object.
  @param *matrixPtr fixme

  @return Pointer to the rigid body.

  This function creates a Newton rigid body and assigns a *collisionPtr* as the collision geometry representing the rigid body.
  This function increments the reference count of the collision geometry.
  All event functions are set to NULL and the material gruopID of the body is set to the default GroupID.

  See also: ::NewtonDestroyBody
*/
NewtonBody* NewtonCreateDynamicBody(const NewtonWorld* const newtonWorld, const NewtonCollision* const collision, const dFloat* const matrixPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance instance(new ndShapeNull());
	if (collision)
	{
		instance = *ObjectFromHandle<ndShapeInstance, NewtonCollision>(collision);
	}

	ndMatrix matrix(matrixPtr);
	if (!CheckFloat(&matrix[0][0], 16))
	{
		ndExpandTraceMessage(("uninitialized matrix, setting to identity\n"));
		matrix = ndGetIdentityMatrix();
	}

	matrix.m_front.m_w = ndFloat32(0.0f);
	matrix.m_up.m_w = ndFloat32(0.0f);
	matrix.m_right.m_w = ndFloat32(0.0f);
	matrix.m_posit.m_w = ndFloat32(1.0f);

	ndSharedPtr<ndBody>* const body = new ndSharedPtr<ndBody>(new ndBodyDynamic());
	ndBodyDynamic* const dynBody = (*body)->GetAsBodyDynamic();

	ndSharedPtr<ndBodyNotify> bodyNotify(new ndNewtonBodyNotify(reinterpret_cast<NewtonBody*>(body)));

	dynBody->SetMatrix(matrix);
	dynBody->SetCollisionShape(instance);
	dynBody->SetNotifyCallback(bodyNotify);

	ndNewtonWorld* const world = ObjectFromHandle<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->AddBody(*body);
	return reinterpret_cast<NewtonBody*>(body);
}

/*!
  Store a user defined data value with the body.

  @param *bodyPtr pointer to the body.
  @param *userDataPtr pointer to the user defined user data value.

  @return Nothing.

  The application can store a user defined value with the Body. This value can be the pointer to a structure containing some application data for special effect.
  if the application allocate some resource to store the user data, the application can register a joint destructor to get rid of the allocated resource when the body is destroyed

  See also: ::NewtonBodyGetUserData
*/
void  NewtonBodySetUserData(const NewtonBody* const bodyPtr, void* const userDataPtr)
{
	TRACE_FUNCTION(__FUNCTION__);

	ndBody* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr);
	ndNewtonBodyNotify* const bodyNotify = static_cast<ndNewtonBodyNotify*>(*body->GetNotifyCallback());
	bodyNotify->m_userData = ndWeakPtr<void>(userDataPtr);
}

/*!
  Retrieve a user defined data value stored with the body.

  @param *bodyPtr pointer to the body.

  @return The user defined data.

  The application can store a user defined value with a rigid body. This value can be the pointer
  to a structure which is the graphical representation of the rigid body.

  See also: ::NewtonBodySetUserData,
*/
void* NewtonBodyGetUserData(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBody* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr);
	ndNewtonBodyNotify* const bodyNotify = static_cast<ndNewtonBodyNotify*>(*body->GetNotifyCallback());
	return *bodyNotify->m_userData;
}


/*!
  Assign a material group id to the body.

  @param *bodyPtr pointer to the body.
  @param id id of a previously created material group.

  @return Nothing.

  When the application creates a body, the default material group, *defaultGroupId*, is applied by default.

  See also: ::NewtonBodyGetMaterialGroupID, ::NewtonMaterialCreateGroupID, ::NewtonMaterialGetDefaultGroupID
*/
void NewtonBodySetMaterialGroupID(const NewtonBody* const bodyPtr, int id)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBody* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr);
	ndNewtonBodyNotify* const bodyNotify = static_cast<ndNewtonBodyNotify*>(*body->GetNotifyCallback());
	bodyNotify->m_materialGoupId = id;

	ndShapeInstance& instance = body->GetAsBodyKinematic()->GetCollisionShape();
	ndShapeMaterial material = instance.GetMaterial();
	material.m_userId = id;
}

void  NewtonBodySetMassProperties(const NewtonBody* const bodyPtr, dFloat mass, const NewtonCollision* const collisionPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBody* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr);
	ndShapeInstance* const instance = ObjectFromHandle<ndShapeInstance, NewtonCollision>(collisionPtr);
	body->GetAsBodyKinematic()->SetMassMatrix(mass, *instance);
}

/*!
  Return the pointer to the current force and torque call back function.

  @param *bodyPtr pointer to the body.

  @return pointer to the force call back.

  This function can be used to concatenate different force calculation components making more modular the
  design of function components dedicated to apply special effect. For example a body may have a basic force a force that
  only apply the effect of gravity, but that application can place a region in where there can be a fluid volume, or another gravity field.
  we this function the application can read the correct function and save into a local variable, and set a new one.
  this new function will firs call the save function pointer and upon return apply the correct effect.
  this similar to the concept of virtual methods on objected oriented languages.

  The function *NewtonApplyForceAndTorque callback* is called by the Newton Engine every time an active body is going to be simulated.
  The Newton Engine does not call the *NewtonApplyForceAndTorque callback* function for bodies that are inactive or have reached a state of stable equilibrium.

  See also: ::NewtonBodyGetUserData, ::NewtonBodyGetUserData, ::NewtonBodySetForceAndTorqueCallback
*/
NewtonApplyForceAndTorque NewtonBodyGetForceAndTorqueCallback(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBody* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr);
	ndNewtonBodyNotify* const bodyNotify = static_cast<ndNewtonBodyNotify*>(*body->GetNotifyCallback());
	return bodyNotify->m_forceAndTorque;
}

/*!
  Assign an event function for applying external force and torque to a rigid body.

  @param *bodyPtr pointer to the body.
  @param callback pointer to a function callback used to apply force and torque to a rigid body.

  @return Nothing.

  Before the *NewtonApplyForceAndTorque callback* is called for a body, Newton first clears the net force and net torque for the body.

  The function *NewtonApplyForceAndTorque callback* is called by the Newton Engine every time an active body is going to be simulated.
  The Newton Engine does not call the *NewtonApplyForceAndTorque callback* function for bodies that are inactive or have reached a state of stable equilibrium.

  See also: ::NewtonBodyGetUserData, ::NewtonBodyGetUserData, ::NewtonBodyGetForceAndTorqueCallback
*/
void  NewtonBodySetForceAndTorqueCallback(const NewtonBody* const bodyPtr, NewtonApplyForceAndTorque callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBody* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr);
	ndNewtonBodyNotify* const bodyNotify = static_cast<ndNewtonBodyNotify*>(*body->GetNotifyCallback());
	bodyNotify->m_forceAndTorque = callback;
}

/*!
  Set the transformation matrix of a rigid body.

  @param *bodyPtr pointer to the body.
  @param *matrixPtr pointer to an array of 16 floats containing the global matrix of the rigid body.

  @return Nothing.

  The matrix should be arranged in row-major order.
  If you are using OpenGL matrices (column-major) you will need to transpose you matrices into a local array, before
  passing them to Newton.

  That application should make sure the transformation matrix has not scale, otherwise unpredictable result will occur.

  See also: ::NewtonBodyGetMatrix
*/
void NewtonBodySetMatrix(const NewtonBody* const bodyPtr, const dFloat* const matrixPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBody* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr);
	ndMatrix matrix(matrixPtr);
	if (!CheckFloat(&matrix[0][0], 16))
	{
		ndExpandTraceMessage(("uninitialized matrix, setting to identity\n"));
		matrix = ndGetIdentityMatrix();
	}

	matrix.m_front.m_w = ndFloat32(0.0f);
	matrix.m_up.m_w = ndFloat32(0.0f);
	matrix.m_right.m_w = ndFloat32(0.0f);
	matrix.m_posit.m_w = ndFloat32(1.0f);
	body->SetMatrix(matrix);
}

/*!
  Get the transformation matrix of a rigid body.

  @param *bodyPtr pointer to the body.
  @param *matrixPtr pointer to an array of 16 floats that will hold the global matrix of the rigid body.

  @return Nothing.

  The matrix should be arranged in row-major order (this is the way direct x stores matrices).
  If you are using OpenGL matrices (column-major) you will need to transpose you matrices into a local array, before
  passing them to Newton.

  See also: ::NewtonBodySetMatrix, ::NewtonBodyGetRotation
*/
void NewtonBodyGetMatrix(const NewtonBody* const bodyPtr, dFloat* const matrixPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBody* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr);
	const ndMatrix matrix(body->GetMatrix());
	ndMemCpy(matrixPtr, &matrix[0][0], sizeof(ndMatrix) / sizeof(ndFloat32));
}

void NewtonBodyGetPosition(const NewtonBody* const bodyPtr, dFloat* const posPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBody* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr);
	const ndMatrix matrix(body->GetMatrix());
	posPtr[0] = matrix.m_posit.m_x;
	posPtr[1] = matrix.m_posit.m_y;
	posPtr[2] = matrix.m_posit.m_z;
}

/*!
  Add the net force applied to a rigid body.

  @param *bodyPtr pointer to the body to be destroyed.
  @param *vectorPtr pointer to an array of 3 floats containing the net force to be applied to the body.

  @return Nothing.

  This function is only effective when called from *NewtonApplyForceAndTorque callback*

  See also: ::NewtonBodySetForce, ::NewtonBodyGetForce
*/
void  NewtonBodyAddForce(const NewtonBody* const bodyPtr, const dFloat* const vectorPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();
	ndVector vector(vectorPtr[0], vectorPtr[1], vectorPtr[2], ndFloat32(0.0f));
	body->SetForce(body->GetForce() + vector);
}

/*!
  Add the net torque applied to a rigid body.

  @param *bodyPtr pointer to the body.
  @param *vectorPtr pointer to an array of 3 floats containing the net torque to be applied to the body.

  @return Nothing.

  This function is only effective when called from *NewtonApplyForceAndTorque callback*

  See also: ::NewtonBodySetTorque, ::NewtonBodyGetTorque
*/
void  NewtonBodyAddTorque(const NewtonBody* const bodyPtr, const dFloat* const vectorPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();
	ndVector vector(vectorPtr[0], vectorPtr[1], vectorPtr[2], ndFloat32(0.0f));
	body->SetTorque(body->GetTorque() + vector);
}

/*!
  Assign a transformation event function to the body.

  @param *bodyPtr pointer to the body.
  @param callback pointer to a function callback in used to update the transformation matrix of the visual object that represents the rigid body.

  @return Nothing.

  The function *NewtonSetTransform callback* is called by the Newton engine every time a visual object that represents the rigid body has changed.
  The application can obtain the pointer user data value that points to the visual object.
  The Newton engine does not call the *NewtonSetTransform callback* function for bodies that are inactive or have reached a state of stable equilibrium.

  The matrix should be organized in row-major order (this is the way directX and OpenGL stores matrices).

  See also: NewtonBodyGetTransformCallback
*/
void  NewtonBodySetTransformCallback(const NewtonBody* const bodyPtr, NewtonSetTransform callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();
	ndNewtonBodyNotify* const bodyNotify = static_cast<ndNewtonBodyNotify*>(*body->GetNotifyCallback());
	bodyNotify->m_applyTransform = callback;
}


/*!
  Assign a transformation event function to the body.

  @param *bodyPtr pointer to the body.

  @return Nothing.

  The function *NewtonSetTransform callback* is called by the Newton engine every time a visual object that represents the rigid body has changed.
  The application can obtain the pointer user data value that points to the visual object.
  The Newton engine does not call the *NewtonSetTransform callback* function for bodies that are inactive or have reached a state of stable equilibrium.

  The matrix should be organized in row-major order (this is the way directX and OpenGL stores matrices).

  See also: ::NewtonBodySetTransformCallback
*/
NewtonSetTransform NewtonBodyGetTransformCallback(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();
	ndNewtonBodyNotify* const bodyNotify = static_cast<ndNewtonBodyNotify*>(*body->GetNotifyCallback());
	return bodyNotify->m_applyTransform;
}

/*!
  Set the continuous collision state mode for this rigid body.
  continuous collision flag is off by default in when bodies are created.

  @param *bodyPtr pointer to the body.
  @param state collision state. 1 indicates this body may tunnel through other objects while moving at high speed. 0 ignore high speed collision checks.

  @return Nothing.

  continuous collision mode enable allow the engine to predict colliding contact on rigid bodies
  Moving at high speed of subject to strong forces.

  continuous collision mode does not prevent rigid bodies from inter penetration instead it prevent bodies from
  passing trough each others by extrapolating contact points when the bodies normal contact calculation determine the bodies are not colliding.

  for performance reason the bodies angular velocities is only use on the broad face of the collision,
  but not on the contact calculation.

  continuous collision does not perform back tracking to determine time of contact, instead it extrapolate contact by incrementally
  extruding the collision geometries of the two colliding bodies along the linear velocity of the bodies during the time step,
  if during the extrusion colliding contact are found, a collision is declared and the normal contact resolution is called.

  for continuous collision to be active the continuous collision mode must on the material pair of the colliding bodies as well as on at least one of the two colliding bodies.

  Because there is penalty of about 40% to 80% depending of the shape complexity of the collision geometry, this feature is set
  off by default. It is the job of the application to determine what bodies need this feature on. Good guidelines are: very small objects,
  and bodies that move a height speed.

  See also: ::NewtonBodyGetContinuousCollisionMode, ::NewtonBodySetContinuousCollisionMode
*/
void NewtonBodySetContinuousCollisionMode(const NewtonBody* const bodyPtr, unsigned state)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();
	//body->SetContinueCollisionMode(state ? true : false);
}

/*!
  Get the continuous collision state mode for this rigid body.

  @param *bodyPtr pointer to the body.

  @return Nothing.


  Because there is there is penalty of about 3 to 5 depending of the shape complexity of the collision geometry, this feature is set
  off by default. It is the job of the application to determine what bodies need this feature on. Good guidelines are: very small objects,
  and bodies that move a height speed.

  this feature is currently disabled:

  See also: ::NewtonBodySetContinuousCollisionMode, ::NewtonBodySetContinuousCollisionMode
*/
int NewtonBodyGetContinuousCollisionMode(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();
	//return body->GetContinueCollisionMode() ? 1 : false;
	return 0;
}


/*!
  Apply the linear viscous damping coefficient to the body.

  @param *bodyPtr is the pointer to the body.
  @param linearDamp linear damping coefficient.

  the default value of *linearDamp* is clamped to a value between 0.0 and 1.0; the default value is 0.1,
  There is a non zero implicit attenuation value of 0.0001 assume by the integrator.

  The dampening viscous friction force is added to the external force applied to the body every frame before going to the solver-integrator.
  This force is proportional to the square of the magnitude of the velocity to the body in the opposite direction of the velocity of the body.
  An application can set *linearDamp* to zero when the application takes control of the external forces and torque applied to the body, should the application
  desire to have absolute control of the forces over that body. However, it is recommended that the *linearDamp* coefficient is set to a non-zero
  value for the majority of background bodies. This saves the application from having to control these forces and also prevents the integrator from
  adding very large velocities to a body.

  See also: ::NewtonBodyGetLinearDamping
*/
void NewtonBodySetLinearDamping(const NewtonBody* const bodyPtr, dFloat linearDamp)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();

	body->SetLinearDamping(linearDamp);
}

/*!
  Get the linear viscous damping of the body.

  @param *bodyPtr is the pointer to the body.

  @return The linear damping coefficient.

  See also: ::NewtonBodySetLinearDamping
*/
dFloat NewtonBodyGetLinearDamping(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();

	return body->GetLinearDamping();
}


/*!
  Apply the angular viscous damping coefficient to the body.

  @param *bodyPtr is the pointer to the body.
  @param *angularDamp pointer to an array of at least three floats containing the angular damping coefficients for the principal axis of the body.

  the default value of *angularDamp* is clamped to a value between 0.0 and 1.0; the default value is 0.1,
  There is a non zero implicit attenuation value of 0.0001 assumed by the integrator.

  The dampening viscous friction torque is added to the external torque applied to the body every frame before going to the solver-integrator.
  This torque is proportional to the square of the magnitude of the angular velocity to the body in the opposite direction of the angular velocity of the body.
  An application can set *angularDamp* to zero when the to take control of the external forces and torque applied to the body, should the application
  desire to have absolute control of the forces over that body. However, it is recommended that the *linearDamp* coefficient be set to a non-zero
  value for the majority of background bodies. This saves the application from needing to control these forces and also prevents the integrator from
  adding very large velocities to a body.

  See also: ::NewtonBodyGetAngularDamping
*/
void  NewtonBodySetAngularDamping(const NewtonBody* const bodyPtr, const dFloat* angularDamp)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();

	ndVector vector(angularDamp[0], angularDamp[1], angularDamp[2], ndFloat32(0.0f));
	body->SetAngularDamping(vector);
}


/*!
  Get the linear viscous damping of the body.

  @param *bodyPtr is the pointer to the body.
  @param *angularDamp pointer to an array of at least three floats to hold the angular damping coefficient for the principal axis of the body.

  See also: ::NewtonBodySetAngularDamping
*/
void  NewtonBodyGetAngularDamping(const NewtonBody* const bodyPtr, dFloat* angularDamp)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();

	ndVector vector(body->GetAngularDamping());
	angularDamp[0] = vector.m_x;
	angularDamp[1] = vector.m_y;
	angularDamp[2] = vector.m_z;
}

/*!
  Set the global linear velocity of the body.

  @param *bodyPtr is the pointer to the body.
  @param *velocity pointer to an array of at least three floats containing the velocity vector.

  See also: ::NewtonBodyGetVelocity
*/
void NewtonBodySetVelocity(const NewtonBody* const bodyPtr, const dFloat* const velocity)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();

	ndVector vector(velocity[0], velocity[1], velocity[2], ndFloat32(0.0f));
	body->SetVelocity(vector);
}

void NewtonBodySetVelocityNoSleep(const NewtonBody* const bodyPtr, const dFloat* const velocity)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();

	ndVector vector(velocity[0], velocity[1], velocity[2], ndFloat32(0.0f));
	body->SetVelocityNoSleep(vector);
}

/*!
  Get the global linear velocity of the body.

  @param *bodyPtr is the pointer to the body.
  @param *velocity pointer to an array of at least three floats to hold the velocity vector.

  See also: ::NewtonBodySetVelocity
*/
void NewtonBodyGetVelocity(const NewtonBody* const bodyPtr, dFloat* const velocity)
{
	TRACE_FUNCTION(__FUNCTION__);

	const ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();

	ndVector vector(body->GetVelocity());
	velocity[0] = vector.m_x;
	velocity[1] = vector.m_y;
	velocity[2] = vector.m_z;
}

void NewtonBodyGetPointVelocity(const NewtonBody* const bodyPtr, const dFloat* const point, dFloat* const velocOut)
{
	TRACE_FUNCTION(__FUNCTION__);
	const ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();
	const ndVector veloc(body->GetVelocityAtPoint(ndVector(point[0], point[1], point[2], ndFloat32(0.0f))));
	velocOut[0] = veloc[0];
	velocOut[1] = veloc[1];
	velocOut[2] = veloc[2];
}

/*!
  Get the global angular velocity of the body.

  @param *bodyPtr is the pointer to the body
  @param *omega pointer to an array of at least three floats to hold the angular velocity vector.

  See also: ::NewtonBodySetOmega
*/
void NewtonBodyGetOmega(const NewtonBody* const bodyPtr, dFloat* const omega)
{
	TRACE_FUNCTION(__FUNCTION__);

	const ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();
	const ndVector vector(body->GetOmega());
	omega[0] = vector.m_x;
	omega[1] = vector.m_y;
	omega[2] = vector.m_z;
}

/*!
  Set the global angular velocity of the body.

  @param *bodyPtr is the pointer to the body.
  @param *omega pointer to an array of at least three floats containing the angular velocity vector.

  See also: ::NewtonBodyGetOmega
*/
void NewtonBodySetOmega(const NewtonBody* const bodyPtr, const dFloat* const omega)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();

	ndVector vector(omega[0], omega[1], omega[2], ndFloat32(0.0f));
	body->SetOmega(vector);
}

void NewtonBodySetOmegaNoSleep(const NewtonBody* const bodyPtr, const dFloat* const omega)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();

	ndVector vector(omega[0], omega[1], omega[2], ndFloat32(0.0f));
	body->SetOmegaNoSleep(vector);
}

/*!
  Get the mass matrix of a rigid body.

  @param *bodyPtr pointer to the body.
  @param *mass pointer to a variable that will hold the mass value of the body.
  @param *Ixx pointer to a variable that will hold the moment of inertia of the first principal axis of inertia of the body.
  @param *Iyy pointer to a variable that will hold the moment of inertia of the first principal axis of inertia of the body.
  @param *Izz pointer to a variable that will hold the moment of inertia of the first principal axis of inertia of the body.

  @return Nothing.

  See also: ::NewtonBodySetMassMatrix, ::NewtonBodyGetInvMass
*/
void  NewtonBodyGetMass(const NewtonBody* const bodyPtr, dFloat* const mass, dFloat* const Ixx, dFloat* const Iyy, dFloat* const Izz)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();

	//	dgVector vector (body->GetApparentMass());
	ndVector vector(body->GetMassMatrix());
	Ixx[0] = vector.m_x;
	Iyy[0] = vector.m_y;
	Izz[0] = vector.m_z;
	mass[0] = vector.m_w;
	if (vector.m_w > ndFloat32(1.0e12f))
	{
		Ixx[0] = 0.0f;
		Iyy[0] = 0.0f;
		Izz[0] = 0.0f;
		mass[0] = 0.0f;
	}
}

/*!
  Set the mass matrix of a rigid body.

  @param *bodyPtr pointer to the body.
  @param mass mass value.
  @param inertiaMatrix fixme

  @return Nothing.

  Newton algorithms have no restriction on the values for the mass, but due to floating point dynamic
  range (24 bit precision) it is best if the ratio between the heaviest and the lightest body in the scene is limited to 200.
  There are no special utility functions in Newton to calculate the moment of inertia of common primitives.
  The application should specify the inertial values, keeping in mind that realistic inertia values are necessary for
  realistic physics behavior.

  See also: ::NewtonConvexCollisionCalculateInertialMatrix, ::NewtonBodyGetMass, ::NewtonBodyGetInvMass
*/
void NewtonBodySetFullMassMatrix(const NewtonBody* const bodyPtr, dFloat mass, const dFloat* const inertiaMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();
	ndMatrix inertia(inertiaMatrix);
	body->SetMassMatrix(mass, inertia);
}

void NewtonBodySetMassMatrix(const NewtonBody* const bodyPtr, dFloat mass, dFloat Ixx, dFloat Iyy, dFloat Izz)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix inertia(ndGetIdentityMatrix());
	inertia[0][0] = Ixx;
	inertia[1][1] = Iyy;
	inertia[2][2] = Izz;
	NewtonBodySetFullMassMatrix(bodyPtr, mass, &inertia[0][0]);
}

/*!
  Set the relative position of the center of mass of a rigid body.

  @param *bodyPtr pointer to the body.
  @param *comPtr pointer to an array of 3 floats containing the relative offset of the center of mass of the body.

  @return Nothing.

  This function can be used to set the relative offset of the center of mass of a rigid body.
  when a rigid body is created the center of mass is set the the point c(0, 0, 0), and normally this is
  the best setting for a rigid body. However the are situations in which and object does not have symmetry or
  simple some kind of special effect is desired, and this origin need to be changed.

  Care must be taken when offsetting the center of mass of a body.
  The application must make sure that the external torques resulting from forces applied at at point
  relative to the center of mass are calculated appropriately.
  this could be done Transform and Torque callback function as the follow pseudo code fragment shows:

  Matrix matrix;
  Vector center;

  NewtonGetMatrix(body, matrix)
  NewtonGetCentreOfMass(body, center);

  //for global space torque.
  Vector localForce (fx, fy, fz);
  Vector localPosition (x, y, z);
  Vector localTorque (crossproduct ((localPosition - center). localForce);
  Vector globalTorque (matrix.RotateVector (localTorque));

  //for global space torque.
  Vector globalCentre (matrix.TranformVector (center));
  Vector globalPosition (x, y, z);
  Vector globalForce (fx, fy, fz);
  Vector globalTorque (crossproduct ((globalPosition - globalCentre). globalForce);

  See also: ::NewtonConvexCollisionCalculateInertialMatrix, ::NewtonBodyGetCentreOfMass
*/
void NewtonBodySetCentreOfMass(const NewtonBody* const bodyPtr, const dFloat* const comPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();
	ndVector vector(comPtr[0], comPtr[1], comPtr[2], ndFloat32(1.0f));
	body->SetCentreOfMass(vector);
}

/*!
  Get the relative position of the center of mass of a rigid body.

  @param *bodyPtr pointer to the body.
  @param *comPtr pointer to an array of 3 floats to hold the relative offset of the center of mass of the body.

  @return Nothing.

  This function can be used to set the relative offset of the center of mass of a rigid body.
  when a rigid body is created the center of mass is set the the point c(0, 0, 0), and normally this is
  the best setting for a rigid body. However the are situations in which and object does not have symmetry or
  simple some kind of special effect is desired, and this origin need to be changed.

  This function can be used in conjunction with *NewtonConvexCollisionCalculateInertialMatrix*

  See also: ::NewtonConvexCollisionCalculateInertialMatrix, ::NewtonBodySetCentreOfMass
*/
void NewtonBodyGetCentreOfMass(const NewtonBody* const bodyPtr, dFloat* const comPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = ObjectFromHandle<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();
	ndVector vector(body->GetCentreOfMass());
	comPtr[0] = vector.m_x;
	comPtr[1] = vector.m_y;
	comPtr[2] = vector.m_z;
}
