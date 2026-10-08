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

NewtonBody* NewtonCreateAsymetricDynamicBody(const NewtonWorld* const newtonWorld, const NewtonCollision* const collisionPtr, const dFloat* const matrixPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return nullptr;
	//Newton* const world = (Newton*)newtonWorld;
	//dgCollisionInstance* collision = (dgCollisionInstance*)collisionPtr;
	//if (!collisionPtr) {
	//	collision = (dgCollisionInstance*)NewtonCreateNull(newtonWorld);
	//}
	//
	//dgMatrix matrix(matrixPtr);
	//matrix.m_front.m_w = dgFloat32(0.0f);
	//matrix.m_up.m_w = dgFloat32(0.0f);
	//matrix.m_right.m_w = dgFloat32(0.0f);
	//matrix.m_posit.m_w = dgFloat32(1.0f);
	//
	//NewtonBody* const body = (NewtonBody*)world->CreateDynamicBodyAsymetric(collision, matrix);
	//if (!collisionPtr) {
	//	NewtonDestroyCollision((NewtonCollision*)collision);
	//}
	//return body;
}

NewtonBody* NewtonCreateKinematicBody(const NewtonWorld* const newtonWorld, const NewtonCollision* const collisionPtr, const dFloat* const matrixPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	return nullptr;
	//Newton* const world = (Newton*)newtonWorld;
	//dgCollisionInstance* collision = (dgCollisionInstance*)collisionPtr;
	//if (!collisionPtr) {
	//	collision = (dgCollisionInstance*)NewtonCreateNull(newtonWorld);
	//}
	//
	//dgMatrix matrix(matrixPtr);
	//matrix.m_front.m_w = dgFloat32(0.0f);
	//matrix.m_up.m_w = dgFloat32(0.0f);
	//matrix.m_right.m_w = dgFloat32(0.0f);
	//matrix.m_posit.m_w = dgFloat32(1.0f);
	//
	//NewtonBody* const body = (NewtonBody*)world->CreateKinematicBody(collision, matrix);
	//if (!collisionPtr) {
	//	NewtonDestroyCollision((NewtonCollision*)collision);
	//}
	//return body;
}

/*!
  Destroy a rigid body.

  @param *bodyPtr pointer to the body to be destroyed.

  @return Nothing.

  If this function is called from inside a simulation step the destruction of the body will be delayed until end of the time step.
  This function will decrease the reference count of the collision geometry by one. If the reference count reaches zero, then the collision
  geometry will be destroyed. This function will destroy all joints associated with this body.

  See also: ::NewtonCreateDynamicBody
*/
void NewtonDestroyBody(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//dgBody* const body = (dgBody*)bodyPtr;
	//dgWorld* const world = body->GetWorld();
	//world->DestroyBody(body);
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

/*!
  Gets the current simulation state of the specified body.

  @param *bodyPtr pointer to the body to be inspected.

  @return the current simulation state 0: disabled 1: active.

  See also: ::NewtonBodySetSimulationState
*/
int NewtonBodyGetSimulationState(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//dgWorld* const world = body->GetWorld();
	//return world->GetBodyEnableDisableSimulationState(body) ? 1 : 0;
	ndAssert(0);
	return 0;
}

/*!
  Sets the current simulation state of the specified body.

  @param *bodyPtr pointer to the body to be changed.
  @param state the new simulation state 0: disabled 1: active

  @return Nothing.

  See also: ::NewtonBodyGetSimulationState
*/
void NewtonBodySetSimulationState(const NewtonBody* const bodyPtr, const int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//dgWorld* const world = body->GetWorld();
	//
	//if (state) {
	//	world->BodyEnableSimulation(body);
	//}
	//else {
	//	world->BodyDisableSimulation(body);
	//}
	ndAssert(0);
}

int NewtonBodyGetCollidable(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//gBody* const body = (dgBody*)bodyPtr;
	//eturn body->IsCollidable() ? 1 : 0;
	ndAssert(0);
	return 0;
}

void NewtonBodySetCollidable(const NewtonBody* const bodyPtr, int collidable)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//body->SetCollidable(collidable ? true : false);

	ndAssert(0);
}

int NewtonBodyGetType(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//if (body->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
	//	return NEWTON_DYNAMIC_BODY;
	//}
	//else if (body->IsRTTIType(dgBody::m_kinematicBodyRTTI)) {
	//	return NEWTON_KINEMATIC_BODY;
	//}
	//else if (body->IsRTTIType(dgBody::m_dynamicBodyAsymentricRTTI)) {
	//	return NEWTON_DYNAMIC_ASYMETRIC_BODY;
	//	//	} else if (body->IsRTTIType(dgBody::m_deformableBodyRTTI)) {
	//	//		return NEWTON_DEFORMABLE_BODY;
	//}
	//dgAssert(0);
	//return 0;

	ndAssert(0);
	return 0;
}

int NewtonBodyGetID(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//return body->GetUniqueID();

	ndAssert(0);
	return 0;
}


/*!
  Return pointer to the Newton world of the specified body.

  @param *bodyPtr Pointer to the body.

  @return World that owns this body.

  The application can also determine the world from a joint, if it queries one
  of the bodies attached to that joint.
*/
NewtonWorld* NewtonBodyGetWorld(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//return (NewtonWorld*)body->GetWorld();

	ndAssert(0);
	return 0;
}


/*!
  Assign an event function to be called when this body is about to be destroyed.

  @param *bodyPtr pointer to the body to be destroyed.
  @param callback pointer to a function callback.

  @return Nothing.


  This function *NewtonBodyDestructor callback* acts like a destruction function in CPP. This function
  is called when the body and all data joints associated with the body are about to be destroyed.
  The application could use this function to destroy or release any resource associated with this body.
  The application should not make reference to this body after this function returns.


  The destruction of a body will destroy all joints associated with the body.

  See also: ::NewtonBodyGetUserData, ::NewtonBodyGetUserData
*/
void NewtonBodySetDestructorCallback(const NewtonBody* const bodyPtr, NewtonBodyDestructor callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//body->SetDestructorCallback(dgBody::OnBodyDestroy(callback));
	ndAssert(0);
}


NewtonBodyDestructor NewtonBodyGetDestructorCallback(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//return NewtonBodyDestructor(body->GetDestructorCallback());
	ndAssert(0);
	return 0;
}

/*!
  Get the inverse mass matrix of a rigid body.

  @param *bodyPtr pointer to the body.
  @param *invMass pointer to a variable that will hold the mass inverse value of the body.
  @param *invIxx pointer to a variable that will hold the moment of inertia inverse of the first principal axis of inertia of the body.
  @param *invIyy pointer to a variable that will hold the moment of inertia inverse of the first principal axis of inertia of the body.
  @param *invIzz pointer to a variable that will hold the moment of inertia inverse of the first principal axis of inertia of the body.

  @return Nothing.

  See also: ::NewtonBodySetMassMatrix, ::NewtonBodyGetMass
*/
void NewtonBodyGetInvMass(const NewtonBody* const bodyPtr, dFloat* const invMass, dFloat* const invIxx, dFloat* const invIyy, dFloat* const invIzz)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//
	////	dgVector vector1 (body->GetMass());
	////	invIxx[0] = dgFloat32 (1.0f) / (vector1.m_x + dgFloat32 (1.0e-8f));
	////	invIyy[0] = dgFloat32 (1.0f) / (vector1.m_y + dgFloat32 (1.0e-8f)); 
	////	invIzz[0] = dgFloat32 (1.0f) / (vector1.m_z + dgFloat32 (1.0e-8f));
	////	invMass[0] = dgFloat32 (1.0f) / (vector1.m_w + dgFloat32 (1.0e-8f));
	//dgVector inverseMass(body->GetInvMass());
	//invIxx[0] = inverseMass.m_x;
	//invIyy[0] = inverseMass.m_y;
	//invIzz[0] = inverseMass.m_z;
	//invMass[0] = inverseMass.m_w;

	ndAssert(0);
}


void NewtonBodyGetInertiaMatrix(const NewtonBody* const bodyPtr, dFloat* const inertiaMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//
	//dgMatrix matrix(body->CalculateInertiaMatrix());
	//memcpy(inertiaMatrix, &matrix[0][0], sizeof(dgMatrix));

	ndAssert(0);
}

void NewtonBodyGetInvInertiaMatrix(const NewtonBody* const bodyPtr, dFloat* const invInertiaMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//
	//dgMatrix matrix(body->CalculateInvInertiaMatrix());
	//memcpy(invInertiaMatrix, &matrix[0][0], sizeof(dgMatrix));
	ndAssert(0);
}

void NewtonBodySetMatrixNoSleep(const NewtonBody* const bodyPtr, const dFloat* const matrixPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//dgMatrix matrix(matrixPtr);
	//
	//matrix.m_front.m_w = dgFloat32(0.0f);
	//matrix.m_up.m_w = dgFloat32(0.0f);
	//matrix.m_right.m_w = dgFloat32(0.0f);
	//matrix.m_posit.m_w = dgFloat32(1.0f);
	//body->SetMatrixNoSleep(matrix);
	ndAssert(0);
}

/*!
  Apply hierarchical transformation to a body.

  @param *bodyPtr pointer to the body.
  @param *matrixPtr pointer to an array of 16 floats containing the global matrix of the rigid body.

  @return Nothing.

  This function applies the transformation matrix to the *body* and also applies the appropriate transformation matrix to
  set of articulated bodies. If the body is in contact with another body the other body is not transformed.

  this function should not be used to transform set of articulated bodies that are connected to a static body.
  doing so will result in unpredictables results. Think for example moving a chain attached to a ceiling from one place to another,
  to do that in real life a person first need to disconnect the chain (destroy the joint), move the chain (apply the transformation to the
  entire chain), the reconnect it in the new position (recreate the joint again).

  this function will set to zero the linear and angular velocity of all bodies that are part of the set of articulated body array.

  The matrix should be arranged in row-major order (this is the way direct x stores matrices).
  If you are using OpenGL matrices (column-major) you will need to transpose you matrices into a local array, before
  passing them to Newton.

  See also: ::NewtonBodySetMatrix
*/
void NewtonBodySetMatrixRecursive(const NewtonBody* const bodyPtr, const dFloat* const matrixPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//Newton* const world = (Newton*)body->GetWorld();
	//
	//world->BodySetMatrix(body, dgMatrix(matrixPtr));

	ndAssert(0);
}



/*!
  Get the rotation part of the transformation matrix of a body, in form of a unit quaternion.

  @param *bodyPtr pointer to the body.
  @param *rotPtr pointer to an array of 4 floats that will hold the global rotation of the rigid body.

  @return Nothing.

  The rotation matrix is written set in the form of a unit quaternion in the format Rot (q0, q1, q1, q3)

  The rotation quaternion is the same as what the application would get by using at function to extract a quaternion form a matrix.
  however since the rigid body already contained the rotation in it, it is more efficient to just call this function avoiding expensive conversion.

  this function could be very useful for the implementation of pseudo frame rate independent simulation.
  by running the simulation at a fix rate and using linear interpolation between the last two simulation frames.
  to determine the exact fraction of the render step.

  See also: ::NewtonBodySetMatrix, ::NewtonBodyGetMatrix
*/
void NewtonBodyGetRotation(const NewtonBody* const bodyPtr, dFloat* const rotPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//const dgQuaternion& rot = body->GetRotation();
	//rotPtr[0] = rot.m_x;
	//rotPtr[1] = rot.m_y;
	//rotPtr[2] = rot.m_z;
	//rotPtr[3] = rot.m_w;

	ndAssert(0);
}


/*!
  Set the net force applied to a rigid body.

  @param *bodyPtr pointer to the body.
  @param *vectorPtr pointer to an array of 3 floats containing the net force to be applied to the body.

  @return Nothing.

  This function is only effective when called from *NewtonApplyForceAndTorque callback*

  See also: ::NewtonBodyAddForce, ::NewtonBodyGetForce
*/
void  NewtonBodySetForce(const NewtonBody* const bodyPtr, const dFloat* const vectorPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//dgVector vector(vectorPtr[0], vectorPtr[1], vectorPtr[2], dgFloat32(0.0f));
	//body->SetForce(vector);

	ndAssert(0);
}


/*!
  Get the net force applied to a rigid body after the last NewtonUpdate.

  @param *bodyPtr pointer to the body.
  @param *vectorPtr pointer to an array of 3 floats to hold the net force of the body.

  @return Nothing.

  See also: ::NewtonBodyAddForce, ::NewtonBodyGetForce
*/
void NewtonBodyGetForce(const NewtonBody* const bodyPtr, dFloat* const vectorPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//dgVector vector(body->GetForce());
	//vectorPtr[0] = vector.m_x;
	//vectorPtr[1] = vector.m_y;
	//vectorPtr[2] = vector.m_z;
	ndAssert(0);
}

/*!
  Set the net torque applied to a rigid body.

  @param *bodyPtr pointer to the body.
  @param *vectorPtr pointer to an array of 3 floats containing the net torque to be applied to the body.

  @return Nothing.

  This function is only effective when called from *NewtonApplyForceAndTorque callback*

  See also: ::NewtonBodyAddTorque, ::NewtonBodyGetTorque
*/
void  NewtonBodySetTorque(const NewtonBody* const bodyPtr, const dFloat* const vectorPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//dgVector vector(vectorPtr[0], vectorPtr[1], vectorPtr[2], dgFloat32(0.0f));
	//body->SetTorque(vector);
	ndAssert(0);
}


/*!
  Get the net torque applied to a rigid body after the last NewtonUpdate.

  @param *bodyPtr pointer to the body.
  @param *vectorPtr pointer to an array of 3 floats to hold the net torque of the body.

  @return Nothing.

  See also: ::NewtonBodyAddTorque, ::NewtonBodyGetTorque
*/
void NewtonBodyGetTorque(const NewtonBody* const bodyPtr, dFloat* const vectorPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//dgVector vector(body->GetTorque());
	//vectorPtr[0] = vector.m_x;
	//vectorPtr[1] = vector.m_y;
	//vectorPtr[2] = vector.m_z;
	ndAssert(0);
}

/*!
  Return a pointer to the first joint attached to this rigid body.

  @param *bodyPtr pointer to the body.

  @return Joint if at least one is attached to the body, NULL if not joint is attached

  this function will only return the pointer to user defined joints, older build in constraints will be skipped by this function.

  this function can be used to implement recursive walk of complex articulated arrangement of rodid bodies.

  See also: ::NewtonBodyGetNextJoint
*/
NewtonJoint* NewtonBodyGetFirstJoint(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//return (NewtonJoint*)body->GetFirstJoint();

	ndAssert(0);
	return 0;
}

/*!
  Return a pointer to the next joint attached to this body.

  @param *bodyPtr pointer to the body.
  @param *jointPtr pointer to current joint.

  @return Joint is at least one joint is attached to the body, NULL if not joint is attached

  this function will only return the pointer to User defined joint, older build in constraints will be skipped by this function.

  this function can be used to implement recursive walk of complex articulated arrangement of rodid bodies.

  See also: ::NewtonBodyGetFirstJoint
*/
NewtonJoint* NewtonBodyGetNextJoint(const NewtonBody* const bodyPtr, const NewtonJoint* const jointPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//return (NewtonJoint*)body->GetNextJoint((dgConstraint*)jointPtr);
	ndAssert(0);
	return 0;
}


/*!
  Return a pointer to the first contact joint attached to this rigid body.

  @param *bodyPtr pointer to the body.

  @return Contact if the body is colliding with anther body, NULL otherwise

  See also: ::NewtonBodyGetNextContactJoint, ::NewtonContactJointGetFirstContact, ::NewtonContactJointGetNextContact, ::NewtonContactJointRemoveContact
*/
NewtonJoint* NewtonBodyGetFirstContactJoint(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//return (NewtonJoint*)body->GetFirstContact();

	ndAssert(0);
	return 0;
}

/*!
  Return a pointer to the next contact joint attached to this rigid body.

  @param *bodyPtr pointer to the body.
  @param *contactPtr pointer to corrent contact joint.

  @return Contact if the body is colliding with anther body, NULL otherwise

  See also: ::NewtonBodyGetFirstContactJoint, ::NewtonContactJointGetFirstContact, ::NewtonContactJointGetNextContact, ::NewtonContactJointRemoveContact
*/
NewtonJoint* NewtonBodyGetNextContactJoint(const NewtonBody* const bodyPtr, const NewtonJoint* const contactPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//return (NewtonJoint*)body->GetNextContact((dgConstraint*)contactPtr);
	ndAssert(0);
	return 0;
}


/*!
  Return a pointer to the next contact joint the connect these two bodies, if the are colliding

  @param *body0 pointer to one body.
  @param *body1 pointer to secund body.

  @return Contact if the body is colliding with anther body, NULL otherwise

  See also: ::NewtonBodyGetFirstContactJoint, ::NewtonContactJointGetFirstContact, ::NewtonContactJointGetNextContact, ::NewtonContactJointRemoveContact
*/
NewtonJoint* NewtonBodyFindContact(const NewtonBody* const body0, const NewtonBody* const body1)
{
	TRACE_FUNCTION(__FUNCTION__);
	//const dgBody* const bodyPtr0 = (dgBody*)body0;
	//const dgBody* const bodyPtr1 = (dgBody*)body1;
	//return (NewtonJoint*)bodyPtr0->GetWorld()->FindContactJoint(bodyPtr0, bodyPtr1);

	ndAssert(0);
	return 0;
}


int NewtonBodyGetSerializedID(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//return body->GetSerializedID();
	ndAssert(0);
	return 0;
}

/*!
  Assign a collision primitive to the body.

  @param *bodyPtr pointer to the body.
  @param *collisionPtr pointer to the new collision geometry.

  @return Nothing.

  This function replaces a collision geometry of a body with the new collision geometry.
  This function increments the reference count of the collision geometry and decrements the reference count
  of the old collision geometry. If the reference count of the old collision geometry reaches zero, the old collision geometry is destroyed.
  This function can be used to swap the collision geometry of bodies at runtime.

  See also: ::NewtonCreateDynamicBody, ::NewtonBodyGetCollision
*/
void NewtonBodySetCollision(const NewtonBody* const bodyPtr, const NewtonCollision* const collisionPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//dgCollisionInstance* const collision = (dgCollisionInstance*)collisionPtr;
	//body->AttachCollision(collision);
	//body->UpdateCollisionMatrix(dgFloat32(0.0f), 0);

	ndAssert(0);
}

void NewtonBodySetCollisionScale(const NewtonBody* const bodyPtr, dFloat scaleX, dFloat scaleY, dFloat scaleZ)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//dgWorld* const world = body->GetWorld();
	//NewtonCollision* const collision = NewtonBodyGetCollision(bodyPtr);
	//
	//dgFloat32 mass = body->GetInvMass().m_w > dgFloat32(0.0f) ? body->GetMass().m_w : dgFloat32(0.0f);
	//NewtonCollisionSetScale(collision, scaleX, scaleY, scaleZ);
	//
	//NewtonJoint* nextJoint;
	//for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(bodyPtr); joint; joint = nextJoint) {
	//	dgConstraint* const contactJoint = (dgConstraint*)joint;
	//	nextJoint = NewtonBodyGetNextContactJoint(bodyPtr, joint);
	//	//world->DestroyConstraint (contactJoint);
	//	contactJoint->ResetMaxDOF();
	//}
	//NewtonBodySetMassProperties(bodyPtr, mass, collision);
	//body->UpdateCollisionMatrix(dgFloat32(0.0f), 0);
	//world->GetBroadPhase()->ResetEntropy();

	ndAssert(0);
}


/*!
  Get the collision primitive of a body.

  @param *bodyPtr pointer to the body.

  @return Pointer to body collision geometry.

  This function does not increment the reference count of the collision geometry.

  See also: ::NewtonCreateDynamicBody, ::NewtonBodySetCollision
*/
NewtonCollision* NewtonBodyGetCollision(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//return (NewtonCollision*)body->GetCollision();
	ndAssert(0);
	return 0;
}

/*!
  Get the material group id of the body.

  @param *bodyPtr pointer to the body.

  @return Nothing.

  See also: ::NewtonBodySetMaterialGroupID
*/
int NewtonBodyGetMaterialGroupID(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//return int(body->GetGroupID());
	ndAssert(0);
	return 0;

}


/*!
  Set the collision state flag of this body when the body is connected to another body by a hierarchy of joints.

  @param *bodyPtr pointer to the body.
  @param state collision state. 1 indicates this body will collide with any linked body. 0 disable collision with body connected to this one by joints.

  @return Nothing.

  sometimes when making complicated arrangements of linked bodies it is possible the collision geometry of these bodies is in the way of the
  joints work space. This could be a problem for the normal operation of the joints. When this situation happens the application can determine which bodies
  are the problem and disable collision for those bodies while they are linked by joints. For the collision to be disable for a pair of body,
  both bodies must have the collision disabled. If the joints connecting the bodies are destroyed these bodies become collidable automatically.
  This feature can also be achieved by making special material for the whole configuration of jointed bodies, however it is a lot easier just to set collision disable
  for jointed bodies.

  See also: ::NewtonBodySetMaterialGroupID
*/
void NewtonBodySetJointRecursiveCollision(const NewtonBody* const bodyPtr, unsigned state)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//body->SetCollisionWithLinkedBodies(state ? true : false);

	ndAssert(0);
}

/*!
  Get the collision state flag when the body is joint.

  @param *bodyPtr pointer to the body.

  @return return the collision state flag for this body.

  See also: ::NewtonBodySetMaterialGroupID
*/
int NewtonBodyGetJointRecursiveCollision(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//
	//return body->GetCollisionWithLinkedBodies() ? 1 : 0;

	ndAssert(0);
	return 0;

}

/*!
  get the freeze state of this body

  @param *bodyPtr is the pointer to the body to be frozen

  @return 1 id the bode is frozen, 0 if bode is unfrozen.

  When a body is created it is automatically placed in the active simulation list. As an optimization
  for large scenes, you may use this function to put background bodies in an inactive equilibrium state.

  This function tells Newton that this body does not currently need to be simulated.
  However, if the body is part of a larger configuration it may be affected indirectly by the reaction forces
  of objects that it is connected to.

  See also: ::NewtonBodySetAutoSleep, ::NewtonBodyGetAutoSleep
*/
int NewtonBodyGetFreezeState(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//return body->GetFreeze() ? 1 : 0;

	ndAssert(0);
	return 0;

}


/*!
  This function tells Newton to simulate or suspend simulation of this body and all other bodies in contact with it

  @param *bodyPtr is the pointer to the body to be activated
  @param state 1 teels newton to freeze the bode and allconceted bodiesm, 0 to unfreze it

  @return Nothing

  This function to no activate the body, is just lock or unlock the body for physics simulation.

  See also: ::NewtonBodyGetFreezeState, ::NewtonBodySetAutoSleep, ::NewtonBodyGetAutoSleep
*/
void NewtonBodySetFreezeState(const NewtonBody* const bodyPtr, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//body->SetFreeze(state ? true : false);

	ndAssert(0);
}

int NewtonBodyGetGyroscopicTorque(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//return body->GetGyroMode() ? 1 : 0;

	ndAssert(0);
	return 0;

}

void NewtonBodySetGyroscopicTorque(const NewtonBody* const bodyPtr, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//body->SetGyroMode(state ? true : false);
	ndAssert(0);
}

/*!
  Set the auto-activation mode for this body.

  @param *bodyPtr is the pointer to the body.
  @param state active mode: 1 = auto-activation on (controlled by Newton). 0 = auto-activation off and body is active all the time.

  @return Nothing.

  Bodies are created with auto-activation on by default.

  Auto activation enabled is the default state for the majority of bodies in a large scene.
  However, for player control, ai control or some other special circumstance, the application may want to control
  the activation/deactivation of the body.
  In that case, the application may call NewtonBodySetAutoSleep (body, 0) followed by
  NewtonBodySetFreezeState(body), this will make the body active forever.

  See also: ::NewtonBodyGetFreezeState, ::NewtonBodySetFreezeState, ::NewtonBodyGetAutoSleep
*/
void NewtonBodySetAutoSleep(const NewtonBody* const bodyPtr, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//body->SetAutoSleep(state ? true : false);
	ndAssert(0);
}

/*!
  Get the auto-activation state of the body.

  @param *bodyPtr is the pointer to the body.

  @return Auto activation state: 1 = auto-activation on. 0 = auto-activation off.

  See also: ::NewtonBodySetAutoSleep, ::NewtonBodyGetSleepState
*/
int NewtonBodyGetAutoSleep(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//
	//return body->GetAutoSleep() ? 1 : 0;

	ndAssert(0);
	return 0;
}


/*!
  Return the sleep mode of a rigid body.

  @param *bodyPtr is the pointer to the body.

  @return Sleep state: 0 = active. 1 = sleeping.

  See also: ::NewtonBodySetAutoSleep
*/
int NewtonBodyGetSleepState(const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//return body->GetSleepState() ? 1 : 0;
	ndAssert(0);
	return 0;

}

void NewtonBodySetSleepState(const NewtonBody* const bodyPtr, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//body->SetSleepState(state ? true : false);
	ndAssert(0);
}


/*!
  Get the world axis aligned bounding box (AABB) of the body.

  @param *bodyPtr is the pointer to the body.
  @param  *p0 - pointer to an array of at least three floats to hold minimum value for the AABB.
  @param  *p1 - pointer to an array of at least three floats to hold maximum value for the AABB.

*/
void NewtonBodyGetAABB(const NewtonBody* const bodyPtr, dFloat* const p0, dFloat* const p1)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgVector vector0;
	//dgVector vector1;
	//
	//dgBody* const body = (dgBody*)bodyPtr;
	//body->GetAABB(vector0, vector1);
	//
	//p0[0] = vector0.m_x;
	//p0[1] = vector0.m_y;
	//p0[2] = vector0.m_z;
	//
	//p1[0] = vector1.m_x;
	//p1[1] = vector1.m_y;
	//p1[2] = vector1.m_z;
	ndAssert(0);
}


/*!
Get the global angular accelration of the body.

@param *bodyPtr is the pointer to the body
@param *omega pointer to an array of at least three floats to hold the angular acceleration vector.
*/
void NewtonBodyGetAlpha(const NewtonBody* const bodyPtr, dFloat* const alpha)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//
	//const dgVector vector(body->GetAlpha());
	//alpha[0] = vector.m_x;
	//alpha[1] = vector.m_y;
	//alpha[2] = vector.m_z;
	ndAssert(0);
}


/*!
Get the global linear Acceleration of the body.

@param *bodyPtr is the pointer to the body.
@param *acceleration pointer to an array of at least three floats to hold the acceleration vector.

See also: ::NewtonBodySetVelocity
*/
void NewtonBodyGetAcceleration(const NewtonBody* const bodyPtr, dFloat* const acceleration)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//
	//dgVector vector(body->GetAccel());
	//acceleration[0] = vector.m_x;
	//acceleration[1] = vector.m_y;
	//acceleration[2] = vector.m_z;

	ndAssert(0);
}

/*!
  Add an impulse to a specific point on a body.

  @param *bodyPtr is the pointer to the body.
  @param pointDeltaVeloc pointer to an array of at least three floats containing the desired change in velocity to point pointPosit.
  @param  pointPosit	- pointer to an array of at least three floats containing the center of the impulse in global space.
  @param timestep - the update rate time step.

  @return Nothing.

  This function will activate the body.

  *pointPosit* and *pointDeltaVeloc* must be specified in global space.

  *pointDeltaVeloc* represent a change in velocity. For example, a value of *pointDeltaVeloc* of (1, 0, 0) changes the velocity
  of *bodyPtr* in such a way that the velocity of point *pointDeltaVeloc* will increase by (1, 0, 0)

  *the calculate impulse will be applied to the body on next frame update

  Because *pointDeltaVeloc* represents a change in velocity, this function must be used with care. Repeated calls
  to this function will result in an increase of the velocity of the body and may cause to integrator to lose stability.
*/
void NewtonBodyAddImpulse(const NewtonBody* const bodyPtr, const dFloat* const pointDeltaVeloc, const dFloat* const pointPosit, dFloat timestep)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//
	//if (body->GetInvMass().m_w > dgFloat32(0.0f)) {
	//	dgVector p(pointPosit[0], pointPosit[1], pointPosit[2], dgFloat32(0.0f));
	//	dgVector v(pointDeltaVeloc[0], pointDeltaVeloc[1], pointDeltaVeloc[2], dgFloat32(0.0f));
	//	body->AddImpulse(v, p, timestep);
	//}
	ndAssert(0);
}


/*!
  Add an train of impulses to a specific point on a body.

  @param *bodyPtr is the pointer to the body.
  @param  impulseCount	- number of impulses and distances in the array distance
  @param  strideInByte	- sized in bytes of vector impulse and
  @param impulseArray pointer to an array containing the desired impulse to apply ate position point array.
  @param pointArray pointer to an array of at least three floats containing the center of the impulse in global space.
  @param timestep - the update rate time step.

  @return Nothing.

  This function will activate the body.

  *pointPosit* and *pointDeltaVeloc* must be specified in global space.

  *the calculate impulse will be applied to the body on next frame update

  this function apply at general impulse to a body a oppose to a desired change on velocity
  this mean that the body mass, and Inertia will determine the gain on velocity.
*/
void NewtonBodyApplyImpulseArray(const NewtonBody* const bodyPtr, int impulseCount, int strideInByte, const dFloat* const impulseArray, const dFloat* const pointArray, dFloat timestep)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//
	//if (body->GetInvMass().m_w > dgFloat32(0.0f)) {
	//	body->ApplyImpulsesAtPoint(impulseCount, strideInByte, impulseArray, pointArray, timestep);
	//}
	ndAssert(0);
}

void NewtonBodyApplyImpulsePair(const NewtonBody* const bodyPtr, dFloat* const linearImpulse, dFloat* const angularImpulse, dFloat timestep)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//
	//if (body->GetInvMass().m_w > dgFloat32(0.0f)) {
	//	dgVector l(linearImpulse[0], linearImpulse[1], linearImpulse[2], dgFloat32(0.0f));
	//	dgVector a(angularImpulse[0], angularImpulse[1], angularImpulse[2], dgFloat32(0.0f));
	//	body->ApplyImpulsePair(l, a, timestep);
	//}
	ndAssert(0);

}

void NewtonBodyIntegrateVelocity(const NewtonBody* const bodyPtr, dFloat timestep)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBody* const body = (dgBody*)bodyPtr;
	//
	//if (body->IsRTTIType(dgBody::m_kinematicBody) || (body->GetInvMass().m_w > dgFloat32(0.0f))) {
	//	body->IntegrateVelocity(timestep);
	//}
	ndAssert(0);
}