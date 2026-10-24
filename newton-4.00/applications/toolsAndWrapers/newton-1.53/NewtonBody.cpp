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
//#include "newtonMaterial.h"
//#include "newtonBodyNotify.h"

class ndNewtonBodyNotify : public ndModelBodyNotify
{
	public:
	D_CLASS_REFLECTION(ndNewtonBodyNotify, ndModelBodyNotify)

	typedef void (*NewtonBodyDestructor) (const NewtonBody* const body);
	typedef void (*NewtonApplyForceAndTorque) (const NewtonBody* const body, ndFloat32 timestep, int threadIndex);
	typedef void (*NewtonSetTransform) (const NewtonBody* const body, const ndFloat32* const matrix, int threadIndex);

	ndNewtonBodyNotify(ndNewtonWorld* const world, NewtonBody* const owner)
		:ndModelBodyNotify()
		,m_userData(nullptr)
		,m_owner(owner)
		,m_world(world)
		,m_materialGoupId(0)
		,m_capSpeed(ndFloat32(60.0f))
		,m_capOmega(ndFloat32(75.0f))
		,m_bodyIsInWorld(false)
		,m_onDestroy(nullptr)
		,m_applyTransform(nullptr)
		,m_forceAndTorque(nullptr)
	{
	}

	ndNewtonBodyNotify(const ndNewtonBodyNotify& notify)
		:ndModelBodyNotify(notify)
		,m_userData(notify.m_userData)
		,m_owner(notify.m_owner)
		,m_world(notify.m_world)
		,m_materialGoupId(notify.m_materialGoupId)
		,m_capSpeed(notify.m_capSpeed)
		,m_capOmega(notify.m_capOmega)
		,m_bodyIsInWorld(false)
		,m_onDestroy(notify.m_onDestroy)
		,m_applyTransform(notify.m_applyTransform)
		,m_forceAndTorque(notify.m_forceAndTorque)
	{
	}

	virtual ~ndNewtonBodyNotify() override
	{
		if (m_onDestroy)
		{
			ndAssert(m_world);
			m_onDestroy(PointerCast<NewtonBody, ndBody>(GetBody()));
		}
	}

	ndBodyNotify* Clone() const override
	{
		return new ndNewtonBodyNotify(*this);
	}

	virtual void OnBodyAddedToWorld()
	{
		ndAssert(0);
	}

	virtual void OnBodyRemovedFromWorld()
	{
		ndAssert(0);
	}

	virtual void OnTransform(ndFloat32 timestep, const ndMatrix& matrix) override
	{
		if (m_applyTransform)
		{
			ndAssert(0);
		}
	}

	virtual void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep) override
	{
		if (m_forceAndTorque)
		{
			ndAssert(0);
		}
	}

	//bool CheckInWorld(const ndMatrix& matrix) const;

	//ndDemoEntityManager* m_manager;
	//ndSharedPtr<ndRenderSceneNode> m_entity;
	//ndTransform m_transform;
	//ndMatrix m_bindMatrix;

	ndWeakPtr<void> m_userData;
	ndWeakPtr<NewtonBody> m_owner;
	ndWeakPtr<ndNewtonWorld> m_world;
	ndInt32 m_materialGoupId;
	ndFloat32 m_capSpeed;
	ndFloat32 m_capOmega;

	bool m_bodyIsInWorld;

	NewtonBodyDestructor m_onDestroy;
	NewtonSetTransform m_applyTransform;
	NewtonApplyForceAndTorque m_forceAndTorque;
};

#if 0

// Name: NewtonBodyGetWorld
// Retrieve get the pointer to the world from the body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
//
// Return: the world that own this body.
//
// Remarks: The application can use this function to determine what world own this body. If the application 
// have to get the world from a joint, it can do so by getting one of the bodies attached to the joint and getting the world from 
// that body.
// 
NewtonWorld* NewtonBodyGetWorld(const NewtonBody* bodyPtr)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	return (NewtonWorld*)body->GetWorld();
}


// Name: NewtonBodySetTransformCallback 
// Assign a transformation event function to the body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *NewtonSetTransform callback - pointer to a function callback in used to update the transformation matrix of the visual object that represents the rigid body.
//
// Return: Nothing.
//
// Remarks: The function *NewtonSetTransform callback* is called by the Newton engine every time a visual object that represents the rigid body has changed.
// The application can obtain the pointer user data value that points to the visual object. 
// The Newton engine does not call the *NewtonSetTransform callback* function for bodies that are inactive or have reached a state of stable equilibrium.
//
// Remarks: The matrix should be organized in row-major order (this is the way directX stores matrices).
// If you are using OpenGL matrices (column-major) you will need to transpose the matrices into a local array, before
// you pass them to Newton.
// 
// See also: NewtonBodyGetUserData, NewtonBodyGetUserData
void  NewtonBodySetTransformCallback(const NewtonBody* bodyPtr, NewtonSetTransform callback)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	body->SetMatrixUpdateCallback((OnMatrixUpdateCallback)callback);
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
	dgBody* body;
	body = (dgBody*)bodyPtr;
	body->SetAutoactiveNotify((OnActivation)callback);
}


// Name: NewtonBodySetForceAndTorqueCallback 
// Assign an event function for applying external force and torque to a rigid body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *NewtonApplyForceAndTorque callback - pointer to a function callback used to apply force and torque to a rigid body.
//
// Return: Nothing.
//
// Remarks: Before the *NewtonApplyForceAndTorque callback* is called for a body, Newton first clears the net force and net torque for the body. 
//
// Remarks: The function *NewtonApplyForceAndTorque callback* is called by the Newton Engine every time an active body is going to be simulated. 
// The Newton Engine does not call the *NewtonApplyForceAndTorque callback* function for bodies that are inactive or have reached a state of stable equilibrium.
// 
// See also: NewtonBodyGetUserData, NewtonBodyGetUserData, NewtonBodyGetForceAndTorqueCallback
void  NewtonBodySetForceAndTorqueCallback(const NewtonBody* bodyPtr, NewtonApplyForceAndTorque callback)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	body->SetExtForceAndTorqueCallback((OnApplyExtForceAndTorque)callback);
}


// Name: NewtonBodyGetForceAndTorqueCallback 
// Return the pointer to the current force and torque call back function.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
//
// Return: pointer to the force call back.
//
// Remarks: This function can be used to concatenate different force calculation components making more modular the 
// design of function components dedicated to apply special effect. For example a body may have a basic force a force that
// only apply the effect of gravity, but that application can place a region in where there can be a fluid volume, or another gravity field.
// we this function the application can read the correct function and save into a local variable, and set a new one.
// this new function will firs call the save function pointer and upon return apply the correct effect.
// this similar to the concept of virtual methods on objected oriented languages.
//
// Remarks: The function *NewtonApplyForceAndTorque callback* is called by the Newton Engine every time an active body is going to be simulated. 
// The Newton Engine does not call the *NewtonApplyForceAndTorque callback* function for bodies that are inactive or have reached a state of stable equilibrium.
// 
// See also: NewtonBodyGetUserData, NewtonBodyGetUserData, NewtonBodySetForceAndTorqueCallback
NewtonApplyForceAndTorque NewtonBodyGetForceAndTorqueCallback(const NewtonBody* bodyPtr)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	return (NewtonApplyForceAndTorque)body->GetExtForceAndTorqueCallback();
}


// Name: NewtonBodySetDestructorCallback 
// Assign an event function to be called when this body is about to be destroyed.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body to be destroyed.
// *NewtonBodyDestructor* callback - pointer to a function callback. 
//
// Return: Nothing.
//
// Remarks: 
// This function *NewtonBodyDestructor callback* acts like a destruction function in CPP. This function
// is called when the body and all data joints associated with the body are about to be destroyed.
// The application could use this function to destroy or release any resource associated with this body. 
// The application should not make reference to this body after this function returns.
//
// Remarks: 
// The destruction of a body will destroy all joints associated with the body.
// 
// See also: NewtonBodyGetUserData, NewtonBodyGetUserData
void NewtonBodySetDestructorCallback(const NewtonBody* bodyPtr, NewtonBodyDestructor callback)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	body->SetDestructorCallback((OnBodyDestroy)callback);
}

// Name: NewtonBodyGetMassMatrix 
// Get the mass matrix of a rigid body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *dFloat* *mass - pointer to a variable that will hold the mass value of the body.
// *dFloat* *Ixx - pointer to a variable that will hold the moment of inertia of the first principal axis of inertia of the body.
// *dFloat* *Iyy - pointer to a variable that will hold the moment of inertia of the first principal axis of inertia of the body.
// *dFloat* *Izz - pointer to a variable that will hold the moment of inertia of the first principal axis of inertia of the body.
//
// Return: Nothing.
//
// See also: NewtonBodySetMassMatrix, NewtonBodyGetInvMass
void  NewtonBodyGetMassMatrix(const NewtonBody* bodyPtr, dFloat* mass, dFloat* Ixx, dFloat* Iyy, dFloat* Izz)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;

	//	dgVector vector (body->GetMass ());
	dgVector vector(body->GetAparentMass());
	Ixx[0] = vector.m_x;
	Iyy[0] = vector.m_y;
	Izz[0] = vector.m_z;
	mass[0] = vector.m_w;
	if (vector.m_w > DG_INFINITE_MASS * 0.5f) {
		Ixx[0] = 0.0f;
		Iyy[0] = 0.0f;
		Izz[0] = 0.0f;
		mass[0] = 0.0f;
	}
}

// Name: NewtonBodyGetInvMass 
// Get the inverse mass matrix of a rigid body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *dFloat* *invMass - pointer to a variable that will hold the mass inverse value of the body.
// *dFloat* *invIxx - pointer to a variable that will hold the moment of inertia inverse of the first principal axis of inertia of the body.
// *dFloat* *invIyy - pointer to a variable that will hold the moment of inertia inverse of the first principal axis of inertia of the body.
// *dFloat* *invIzz - pointer to a variable that will hold the moment of inertia inverse of the first principal axis of inertia of the body.
//
// Return: Nothing.
//
// See also: NewtonBodySetMassMatrix, NewtonBodyGetMassMatrix
void NewtonBodyGetInvMass(const NewtonBody* bodyPtr, dFloat* invMass, dFloat* invIxx, dFloat* invIyy, dFloat* invIzz)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;

	dgVector vector(body->GetInvMass());
	//	invIxx[0] = vector.m_x;
	//	invIyy[0] = vector.m_y; 
	//	invIzz[0] = vector.m_z;
	//	invMass[0] = vector.m_w;

	dgVector vector1(body->GetAparentMass());
	invIxx[0] = dgFloat32(1.0f) / (vector1.m_x + dgFloat32(1.0e-8f));
	invIyy[0] = dgFloat32(1.0f) / (vector1.m_y + dgFloat32(1.0e-8f));
	invIzz[0] = dgFloat32(1.0f) / (vector1.m_z + dgFloat32(1.0e-8f));
	invMass[0] = dgFloat32(1.0f) / (vector1.m_w + dgFloat32(1.0e-8f));
}

// Name: NewtonBodySetMatrixRecursive 
// Apply hierarchical transformation to a body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *const dFloat* *matrixPtr - pointer to an array of 16 floats containing the global matrix of the rigid body.
//
// Return: Nothing.
//
// Remarks: This function applies the transformation matrix to the *body* and also applies the appropriate transformation matrix to
// set of articulated bodies. If the body is in contact with another body the other body is not transformed.
//
// Remarks: this function should not be used to transform set of articulated bodies that are connected to a static body.
// doing so will result in unpredictables results. Think for example moving a chain attached to a ceiling from one place to another,
// to do that in real life a person first need to disconnect the chain (destroy the joint), move the chain (apply the transformation to the 
// entire chain), the reconnect it in the new position (recreate the joint again).
//
// Remarks: this function will set to zero the linear and angular velocity of all bodies that are part of the set of articulated body array.
//
// Remarks: The matrix should be arranged in row-major order (this is the way direct x stores matrices).
// If you are using OpenGL matrices (column-major) you will need to transpose you matrices into a local array, before
// passing them to Newton.
//
// See also: NewtonBodySetMatrix  
void NewtonBodySetMatrixRecursive(const NewtonBody* bodyPtr, const dFloat* matrixPtr)
{
	dgBody* body;
	Newton* world;

	body = (dgBody*)bodyPtr;
	world = (Newton*)body->GetWorld();
	dgMatrix matrix(*((dgMatrix*)matrixPtr));

	world->BodySetMatrix(body, matrix);
}

// Name: NewtonBodySetForce 
// Set the net force applied to a rigid body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *const dFloat* *forcePtr - pointer to an array of 3 floats containing the net force to be applied to the body.
//
// Return: Nothing.
//
// Remarks: This function is only effective when called from *NewtonApplyForceAndTorque callback*
//
// See also: NewtonBodyAddForce, NewtonBodyGetForce
void  NewtonBodySetForce(const NewtonBody* bodyPtr, const dFloat* forcePtr)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	dgVector vector(forcePtr[0], forcePtr[1], forcePtr[2]);
	body->SetForce(vector);
}

// Name: NewtonBodyAddForce 
// Add the net force applied to a rigid body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body to be destroyed.
// *const dFloat* *forcePtr - pointer to an array of 3 floats containing the net force to be applied to the body.
//
// Return: Nothing.
//
// Remarks: This function is only effective when called from *NewtonApplyForceAndTorque callback*
//
// See also: NewtonBodySetForce, NewtonBodyGetForce
void  NewtonBodyAddForce(const NewtonBody* bodyPtr, const dFloat* forcePtr)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	dgVector vector(forcePtr[0], forcePtr[1], forcePtr[2]);

	body->AddForce(vector);
}

// Name: NewtonBodyGetForce 
// Get the net force applied to a rigid body after the last NewtonUpdate.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *const dFloat* *forcePtr - pointer to an array of 3 floats to hold the net force of the body.
//
// Return: Nothing.
//
// See also: NewtonBodyAddForce, NewtonBodyGetForce
void NewtonBodyGetForce(const NewtonBody* bodyPtr, dFloat* vectorPtr)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;

	dgVector vector(body->GetNetForce());
	vectorPtr[0] = vector.m_x;
	vectorPtr[1] = vector.m_y;
	vectorPtr[2] = vector.m_z;
}

// Name: NewtonBodySetTorque 
// Set the net torque applied to a rigid body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *const dFloat* *torquePtr - pointer to an array of 3 floats containing the net torque to be applied to the body.
//
// Return: Nothing.
//
// Remarks: This function is only effective when called from *NewtonApplyForceAndTorque callback*
//
// See also: NewtonBodyAddTorque, NewtonBodyGetTorque
void  NewtonBodySetTorque(const NewtonBody* bodyPtr, const dFloat* forcePtr)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	dgVector vector(forcePtr[0], forcePtr[1], forcePtr[2]);
	body->SetTorque(vector);
}

// Name: NewtonBodySetCentreOfMass 
// Set the relative position of the center of mass of a rigid body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *const dFloat* *comPtr - pointer to an array of 3 floats containing the relative offset of the center of mass of the body.
//
// Return: Nothing.
//
// Remarks: This function can be used to set the relative offset of the center of mass of a rigid body.
// when a rigid body is created the center of mass is set the the point c(0, 0, 0), and normally this is 
// the best setting for a rigid body. However the are situations in which and object does not have symmetry or
// simple some kind of special effect is desired, and this origin need to be changed.
//
// Remarks: Care must be taken when offsetting the center of mass of a body. 
// The application must make sure that the external torques resulting from forces applied at at point
// relative to the center of mass are calculated appropriately.
// this could be done Transform and Torque callback function as the follow pseudo code fragment shows:
//
// Matrix matrix;
// Vector center;
//
// NewtonGatMetrix(body, matrix)
// NewtonGetCentreOfMass(body, center);
//
// for global space torque.  
// Vector localForce (fx, fy, fz);
// Vector localPosition (x, y, z);
// Vector localTroque (crossproduct ((localPosition - center). localForce);
// Vector globalTroque (matrix.RotateVector (localTroque));
// 
// for global space torque. 
// Vector globalCentre (matrix.TranformVector (center));
// Vector globalPosition (x, y, z);
// Vector globalForce (fx, fy, fz);
// Vector globalTroque (crossproduct ((globalPosition - globalCentre). globalForce);
//
// See also: NewtonConvexCollisionCalculateInertialMatrix, NewtonBodyGetCentreOfMass 
void NewtonBodySetCentreOfMass(const NewtonBody* bodyPtr, const dFloat* comPtr)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	dgVector vector(comPtr[0], comPtr[1], comPtr[2]);
	body->SetCentreOfMass(vector);
}

// Name: NewtonBodyAddTorque 
// Add the net torque applied to a rigid body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *const dFloat* *torquePtr - pointer to an array of 3 floats containing the net torque to be applied to the body.
//
// Return: Nothing.
//
// Remarks: This function is only effective when called from *NewtonApplyForceAndTorque callback*
//
// See also: NewtonBodySetTorque, NewtonBodyGetTorque
void  NewtonBodyAddTorque(const NewtonBody* bodyPtr, const dFloat* forcePtr)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	dgVector vector(forcePtr[0], forcePtr[1], forcePtr[2]);
	body->AddTorque(vector);
}

// Name: NewtonBodyGetTorque 
// Get the net torque applied to a rigid body after the last NewtonUpdate.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *const dFloat* *torquePtr - pointer to an array of 3 floats to hold the net torque of the body.
//
// Return: Nothing.
//
// See also: NewtonBodyAddTorque, NewtonBodyGetTorque
void NewtonBodyGetTorque(const NewtonBody* bodyPtr, dFloat* vectorPtr)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;

	dgVector vector(body->GetNetTorque());
	vectorPtr[0] = vector.m_x;
	vectorPtr[1] = vector.m_y;
	vectorPtr[2] = vector.m_z;
}

// Name: NewtonBodyGetCentreOfMass 
// Get the relative position of the center of mass of a rigid body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *dFloat* *comPtr - pointer to an array of 3 floats to hold the relative offset of the center of mass of the body.
//
// Return: Nothing.
//
// Remarks: This function can be used to set the relative offset of the center of mass of a rigid body.
// when a rigid body is crated the center of mass is set the the point c(0, 0, 0), and normally this is 
// the best setting for a rigid body. However the are situations in which and object does not have symmetry or
// simple some kind of special effect is desired, and this origin need to be changed.
//
// Remarks: This function can be used in conjunction with *NewtonConvexCollisionCalculateInertialMatrix* 
//
// See also: NewtonConvexCollisionCalculateInertialMatrix, NewtonBodySetCentreOfMass 
void NewtonBodyGetCentreOfMass(const NewtonBody* bodyPtr, dFloat* comPtr)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	dgVector vector(body->GetCentreOfMass());
	comPtr[0] = vector.m_x;
	comPtr[1] = vector.m_y;
	comPtr[2] = vector.m_z;
}

// Name: NewtonBodyAddBuoyancyForce 
// Add buoyancy force and torque for bodies immersed in a fluid.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *dFloat* fluidDensity - fluid density.
// *dFloat* fluidLinearViscosity - fluid linear viscosity (resistance to linear translation).
// *dFloat* fluidAngularViscosity - fluid angular viscosity (resistance to rotation).
// *const dFloat* *gravityVector - pointer to an array of floats containing the gravity vector.
// *NewtonGetBuoyancyPlane* *buoyancyPlane - pointer to an array of at least 4 floats containing the plane equation of the surface of the fluid. This parameter can be NULL
//
// Return: Nothing.
//
// Remarks: This function is only effective when called from *NewtonApplyForceAndTorque callback*
//
// Remarks: This function adds buoyancy force and torque to a body when it is immersed in a fluid.
// The force is calculated according to Archimedes’ Buoyancy Principle. When the parameter *buoyancyPlane* is set to NULL, the body is considered
// to completely immersed in the fluid. This can be used to simulate boats and lighter than air vehicles etc..
//
// Remarks: If *buoyancyPlane* return 0 buoyancy calculation for this collision primitive is ignored, this could be used to filter bouyancy calculation 
// of compound collision geometry with different IDs. 
//
// See also: NewtonConvexCollisionCalculateVolume
void  NewtonBodyAddBuoyancyForce(
	const NewtonBody* bodyPtr,
	dFloat fluidDensity,
	dFloat fluidLinearViscosity,
	dFloat fluidAngularViscosity,
	const dFloat* gravityVector,
	NewtonGetBuoyancyPlane buoyancyPlane,
	void* context)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;

	dgVector gravity(gravityVector[0], gravityVector[1], gravityVector[2]);
	body->AddBuoyancyForce(fluidDensity, fluidLinearViscosity, fluidAngularViscosity, gravity, (GetBuoyancyPlane)buoyancyPlane, context);
}


// Name: NewtonBodySetCollision 
// Assign a collision primitive to the body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *const collisionPtr* *collisionPtr - pointer to the new collision geometry.
//
// Return: Nothing.
//
// Remarks: This function replaces a collision geometry of a body with the new collision geometry.
// This function increments the reference count of the collision geometry and decrements the reference count
// of the old collision geometry. If the reference count of the old collision geometry reaches zero, the old collision geometry is destroyed.
// This function can be used to swap the collision geometry of bodies at runtime.
//
// See also: NewtonCreateBody, NewtonBodyGetCollision
void NewtonBodySetCollision(const NewtonBody* bodyPtr, const NewtonCollision* collisionPtr)
{
	dgBody* body;
	dgCollision* collision;

	body = (dgBody*)bodyPtr;
	collision = (dgCollision*)collisionPtr;
	body->AttachCollision(collision);
}


// Name: NewtonBodyCoriolisForcesMode 
// Enable or disable Coriolis and gyroscopic force calculation for this body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *int* mode - force mode zero indicate not gyroscopic force calculation.
//
// Return: Nothing.
//
// Remarks: Gyroscopic forces internal forces generated as a result of an asymmetric tensor. They are a pure mathematical consequence that the physics have to comply in order to agree with the math. As Gyroscopic forces are not real forces but the result of net unbalance of the changing inertia tensor or a rigid body when its angular velocity is measured on a reference frame different than the body’s own.  
// Gyroscopic forces are extremely non linear by nature, therefore a first order implicit integrator will have a extremely hard time at dealing with this kind of forces, however because the fact that they are not real forces they do not make much difference in the outcome of the integration.
// Fortunately due to the fact that the magnitude of gyroscopic forces is proportional to the unbalance of the inertia tensor, it is possible to disregard the effect of this forces by assuming their inertial tensor is symmetric for the purpose of this calculation. For most cases an ordinary person is not capable to distinguish the motion of a body subject to gyroscopic forces and one that is not, especially when the motion is constrained.
// Because of this fact gyroscopic force are turned off by default in Newton, however there are cases when the desire effect is precisely to simulate these forces like a spinning top, or the design of a space navigational system, etc. The most important feature of gyroscopic forces is that they make the rigid body to process. 
void  NewtonBodyCoriolisForcesMode(const NewtonBody* bodyPtr, int mode)
{
	dgBody* body;

	body = (dgBody*)bodyPtr;
	body->SetGiroscopicTorqueMode(mode ? true : false);
}


// Name: NewtonBodyGetCollision 
// Get the collision primitive of a body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
//
// Return: Pointer to body collision geometry.
//
// Remarks: This function does not increment the reference count of the collision geometry. 
//
// See also: NewtonCreateBody, NewtonBodySetCollision
NewtonCollision* NewtonBodyGetCollision(const NewtonBody* bodyPtr)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	return (NewtonCollision*)body->GetCollision();
}


// Name: NewtonBodySetMaterialGroupID 
// Assign a material group id to the body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *int* id - id of a previously created material group.
//
// Return: Nothing.
//
// Remarks: When the application creates a body, the default material group, *defaultGroupId*, is applied by default.
//
// See also: NewtonBodyGetMaterialGroupID, NewtonMaterialCreateGroupID, NewtonMaterialGetDefaultGroupID
void NewtonBodySetMaterialGroupID(const NewtonBody* bodyPtr, int id)
{
	dgBody* body;

	body = (dgBody*)bodyPtr;
	body->SetGroupID(id);
}


// Name: NewtonBodyGetMaterialGroupID 
// Get the material group id of the body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *int* id - id of a previously created material group.
//
// Return: Nothing.
//
// See also: NewtonBodySetMaterialGroupID
int NewtonBodyGetMaterialGroupID(const NewtonBody* bodyPtr)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	return body->GetGroupID();
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
void NewtonBodySetContinuousCollisionMode(const NewtonBody* bodyPtr, unsigned state)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	body->SetContinuesCollisionMode(state ? true : false);
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
	dgBody* body;
	body = (dgBody*)bodyPtr;
	return body->GetContinuesCollisionMode() ? 1 : false;
}



// Name: NewtonBodySetJointRecursiveCollision 
// Set the collision state flag of this body when the body is connected to another body by a hierarchy of joints.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *int* state - collision state. 1 indicates this body will collide with any linked body. 0 disable collision with body connected to this one by joints.
//
// Return: Nothing.
//
// Remarks: sometimes when making complicated arrangements of linked bodies it is possible the collision geometry of these bodies is in the way of the 
// joints work space. This could be a problem for the normal operation of the joints. When this situation happens the application can determine which bodies
// are the problem and disable collision for those bodies while they are linked by joints. For the collision to be disable for a pair of body, 
// both bodies must have the collision disabled. If the joints connecting the bodies are destroyed these bodies become collidable automatically.
// This feature can also be achieved by making special material for the whole configuration of jointed bodies, however it is a lot easier just to set collision disable
// for jointed bodies.
//
// See also: NewtonBodySetMaterialGroupID
void NewtonBodySetJointRecursiveCollision(const NewtonBody* bodyPtr, unsigned state)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	body->SetCollisionWithLinkedBodies(state ? true : false);
}

// Name: NewtonBodyGetJointRecursiveCollision 
// Get the collision state flag when the body is joint.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
//
// Return: return the collision state flag for this body.
//
// See also: NewtonBodySetMaterialGroupID
int NewtonBodyGetJointRecursiveCollision(const NewtonBody* bodyPtr)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	return body->GetCollisionWithLinkedBodies() ? 1 : 0;
}

// Name: NewtonBodySetAutoFreeze 
// Set the auto-activation mode for this body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - is the pointer to the body.
// *int* state - active mode: 1 = auto-activation on (controlled by Newton). 0 = auto-activation off (controlled by the application). 
// 
// Return: Nothing.
//
// Remarks: Bodies are created with auto-activation on by default. 
//
// Remarks: Auto activation enabled is the default state for the majority of bodies in a large scene.
// However, for player control, ai control or some other special circumstance, the application may want to control
// the activation/deactivation of the body. In that case, the application may call NewtonBodySetAutoFreeze (body, 0) followed by
// NewtonWorldUnfreezeBody(world, body), this will make the body active forever. 
//
// See also: NewtonWorldFreezeBody, NewtonWorldUnfreezeBody, NewtonBodyGetAutoFreeze, NewtonBodySetFreezeTreshold
void NewtonBodySetAutoFreeze(const NewtonBody* bodyPtr, int state)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	body->SetAutoFreeze(state ? true : false);
}

// Name: NewtonBodyGetAutoFreeze 
// Get the auto-activation state of the body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - is the pointer to the body.
// 
// Return: Auto activation state: 1 = auto-activation on. 0 = auto-activation off.
//
// See also: NewtonBodySetAutoFreeze, NewtonBodyGetSleepingState
int NewtonBodyGetAutoFreeze(const NewtonBody* bodyPtr)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	return body->GetAutoFreeze() ? 1 : 0;
}


// Name: NewtonBodyGetSleepingState
// Return the sleep mode of a rigid body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - is the pointer to the body.
// 
// Return: Sleep state: 1 = active. 0 = sleeping. 
//
// See also: NewtonBodySetAutoFreeze
int NewtonBodyGetSleepingState(const NewtonBody* bodyPtr)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	return body->GetActive() ? 1 : 0;
}

// Name: NewtonBodySetFreezeTreshold 
// Set the minimum values for velocity of a body that will be considered at rest.
//
// Parameters:
// *const NewtonBody* *bodyPtr - is the pointer to the body.
// *dFloat* freezeSpeedMag2 - magnitude squared of the velocity threshold.
// *dFloat* freezeOmegaMag2 - magnitude squared of angular velocity threshold.
// *int* framesCount - number of frames the body velocity and angular will not exceed freezeSpeedMag and freezeOmegaMag.
// 
// Remarks: Ideally, a body should be deactivated when it reaches a state of stable equilibrium. However, because of floating point 
// inaccuracy, discrete time step simulation and other factors it is virtually impossible for a body to reach that state
// in a real-time simulation. Therefore, in the Newton World, a body is considered to be in stable equilibrium when its 
// velocity and angular velocity fall below some threshold for a consecutive number of frames. 
//
// Remarks: The default and minimum values for the thresholds is 0.01 for speed and 10 for frames count.
// These values are tuned for single objects colliding under the influence of gravity. It is possible that for complex configuration
// of bodies like multiples pendulums, rag dolls, etc. these values may need to be increased. This is because joints have the property that they
// add a small amount of energy to the system in order to reduce the separation error. This may cause the bodies reach a state of unstable
// equilibrium. That is, when a body oscillates between two different positions because the energy added to the body is equal to the energy
// dissipated by the integrator. This is a situation that is hard to predict, and the best solution is to tweak these values for specific cases. 
//
// See also: NewtonBodySetAutoFreeze, NewtonBodyGetFreezeTreshold
void NewtonBodySetFreezeTreshold(const NewtonBody* bodyPtr, dFloat freezeSpeedMag2, dFloat freezeOmegaMag2, int framesCount)
{
	//	dFloat alpha;
	//	dFloat accel;
	dgBody* body;

	body = (dgBody*)bodyPtr;
	//	framesCount =  GetMin (framesCount * 2, 30); 
	//	alpha = 60.0f * dgSqrt (freezeOmegaMag2) / framesCount;
	//	accel = 60.0f * dgSqrt (freezeSpeedMag2) / framesCount;
	//	body->SetFreezeTreshhold (accel * accel, alpha * alpha, freezeSpeedMag2, freezeOmegaMag2);
	//	dFloat scale;
	//	slace = dgFloat32(1.0f) / (framesCount

	body->SetFreezeTreshhold(freezeSpeedMag2, freezeOmegaMag2, freezeSpeedMag2 * dgFloat32(0.1f), freezeOmegaMag2 * dgFloat32(0.1f));
}

// Name: NewtonBodyGetFreezeTreshold 
// Get the minimum values for velocity of a body the will be considered at rest.
//
// Parameters:
// *const NewtonBody* *bodyPtr - is the pointer to the body.
// *dFloat* freezeSpeedMag2 - point the to a dFloat to hold the velocity threshold
// *dFloat* freezeOmegaMag2 - point the to a dFloat to hold the angular velocity threshold 
//
// See also: NewtonBodySetFreezeTreshold
void NewtonBodyGetFreezeTreshold(const NewtonBody* bodyPtr, dFloat* freezeSpeedMag2, dFloat* freezeOmegaMag2)
{
	dgBody* body;
	dFloat alpha;
	dFloat accel;

	body = (dgBody*)bodyPtr;
	body->GetFreezeTreshhold(accel, alpha, *freezeSpeedMag2, *freezeOmegaMag2);

	freezeSpeedMag2[0] *= 10.0f;
	freezeOmegaMag2[0] *= 10.0f;
}


// Name: NewtonBodyGetAABB 
// Get the world axis aligned bounding box (AABB) of the body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - is the pointer to the body.
// *dFloat* *p0 - pointer to an array of at least three floats to hold minimum value for the AABB.
// *dFloat* *p1 - pointer to an array of at least three floats to hold maximum value for the AABB.
//
void NewtonBodyGetAABB(const NewtonBody* bodyPtr, dFloat* p0, dFloat* p1)
{
	dgBody* body;

	dgVector vector0;
	dgVector vector1;

	body = (dgBody*)bodyPtr;
	body->GetAABB(vector0, vector1);

	p0[0] = vector0.m_x;
	p0[1] = vector0.m_y;
	p0[2] = vector0.m_z;

	p1[0] = vector1.m_x;
	p1[1] = vector1.m_y;
	p1[2] = vector1.m_z;

}

// Name: NewtonBodySetVelocity 
// Set the global linear velocity of the body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - is the pointer to the body.
// *const dFloat* *velocity - pointer to an array of at least three floats containing the velocity vector.
//
// See also: NewtonBodyGetVelocity
void NewtonBodySetVelocity(const NewtonBody* bodyPtr, const dFloat* velocity)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	dgVector vector(velocity[0], velocity[1], velocity[2]);
	body->SetVelocity(vector);
}

// Name: NewtonBodyGetVelocity 
// Get the global linear velocity of the body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - is the pointer to the body.
// *const dFloat* *velocity - pointer to an array of at least three floats to hold the velocity vector.
//
// See also: NewtonBodySetVelocity
void NewtonBodyGetVelocity(const NewtonBody* bodyPtr, dFloat* velocity)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;

	//	dgVector& vector = *((dgVector*) velocity);
	//	vector = body->GetVelocity();

	dgVector vector(body->GetVelocity());
	velocity[0] = vector.m_x;
	velocity[1] = vector.m_y;
	velocity[2] = vector.m_z;

}



// Name: NewtonBodyGetOmega 
// Get the global angular velocity of the body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - is the pointer to the body
// *dFloat* *omega - pointer to an array of at least three floats to hold the angular velocity vector.
//
// See also: NewtonBodySetOmega
void NewtonBodyGetOmega(const NewtonBody* bodyPtr, dFloat* omega)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;

	//	dgVector& vector = *((dgVector*) omega);
	//	vector = body->GetOmega();

	dgVector vector(body->GetOmega());
	omega[0] = vector.m_x;
	omega[1] = vector.m_y;
	omega[2] = vector.m_z;
}


// Name: NewtonBodySetLinearDamping 
// Apply the linear viscous damping coefficient to the body. 
//
// Parameters:
// *const NewtonBody* *bodyPtr - is the pointer to the body.
// *dFloat* linearDamp - linear damping coefficient.
//
// Remarks: the default value of *linearDamp* is clamped to a value between 0.0 and 1.0; the default value is 0.1,
// There is a non zero implicit attenuation value of 0.0001 assume by the integrator.
//
// Remarks: The dampening viscous friction force is added to the external force applied to the body every frame before going to the solver-integrator. 
// This force is proportional to the square of the magnitude of the velocity to the body in the opposite direction of the velocity of the body. 
// An application can set *linearDamp* to zero when the application takes control of the external forces and torque applied to the body, should the application
// desire to have absolute control of the forces over that body. However, it is recommended that the *linearDamp* coefficient is set to a non-zero 
// value for the majority of background bodies. This saves the application from having to control these forces and also prevents the integrator from
// adding very large velocities to a body.
//
// See also: NewtonBodyGetLinearDamping
void NewtonBodySetLinearDamping(const NewtonBody* bodyPtr, dFloat linearDamp)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	body->SetLinearDamping(linearDamp);
}

// Name: NewtonBodyGetLinearDamping 
// Get the linear viscous damping of the body. 
//
// Parameters:
// *const NewtonBody* *bodyPtr - is the pointer to the body.
//
// Return: The linear damping coefficient. 
//
// See also: NewtonBodySetLinearDamping
dFloat NewtonBodyGetLinearDamping(const NewtonBody* bodyPtr)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;

	return body->GetLinearDamping();
}


// Name: NewtonBodySetAngularDamping 
// Apply the angular viscous damping coefficient to the body. 
//
// Parameters:
// *const NewtonBody* *bodyPtr - is the pointer to the body.
// *dFloat* *angularDamp - pointer to an array of at least three floats containing the angular damping coefficients for the principal axis of the body.
//
// Remarks: the default value of *angularDamp* is clamped to a value between 0.0 and 1.0; the default value is 0.1,
// There is a non zero implicit attenuation value of 0.0001 assumed by the integrator.
//
// Remarks: The dampening viscous friction torque is added to the external torque applied to the body every frame before going to the solver-integrator. 
// This torque is proportional to the square of the magnitude of the angular velocity to the body in the opposite direction of the angular velocity of the body. 
// An application can set *angularDamp* to zero when the to take control of the external forces and torque applied to the body, should the application
// desire to have absolute control of the forces over that body. However, it is recommended that the *linearDamp* coefficient be set to a non-zero 
// value for the majority of background bodies. This saves the application from needing to control these forces and also prevents the integrator from
// adding very large velocities to a body.
//
// See also: NewtonBodyGetAngularDamping
void  NewtonBodySetAngularDamping(const NewtonBody* bodyPtr, const dFloat* angularDamp)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;
	dgVector vector(angularDamp[0], angularDamp[1], angularDamp[2]);
	body->SetAngularDamping(vector);
}


// Name: NewtonBodyGetAngularDamping 
// Get the linear viscous damping of the body. 
//
// Parameters:
// *const NewtonBody* *bodyPtr - is the pointer to the body.
// *dFloat* *angularDamp - pointer to an array of at least three floats to hold the angular damping coefficient for the principal axis of the body.
//
// See also: NewtonBodySetAngularDamping
void  NewtonBodyGetAngularDamping(const NewtonBody* bodyPtr, dFloat* angularDamp)
{
	dgBody* body;
	body = (dgBody*)bodyPtr;

	//	dgVector& vector = *((dgVector*) angularDamp);
	//	vector = body->GetAngularDamping();

	dgVector vector(body->GetAngularDamping());
	angularDamp[0] = vector.m_x;
	angularDamp[1] = vector.m_y;
	angularDamp[2] = vector.m_z;
}

// Name: NewtonAddBodyImpulse
// Add an impulse to a specific point on a body.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - pointer to the Newton world.
// *const NewtonBody* *bodyPtr - is the pointer to the body.
// *const dFloat* pointDeltaVeloc - pointer to an array of at least three floats containing the desired change in velocity to point pointPosit.
// *const dFloat* pointPosit	- pointer to an array of at least three floats containing the center of the impulse in global space.
//
// Return: Nothing.
// 
// Remarks: This function will activate the body.
//
// Remarks: *pointPosit* and *pointDeltaVeloc* must be specified in global space.
//
// Remarks: *pointDeltaVeloc* represent a change in velocity. For example, a value of *pointDeltaVeloc* of (1, 0, 0) changes the velocity 
// of *bodyPtr* in such a way that the velocity of point *pointDeltaVeloc* will increase by (1, 0, 0)
//
// Remarks: Because *pointDeltaVeloc* represents a change in velocity, this function must be used with care. Repeated calls
// to this function will result in an increase of the velocity of the body and may cause to integrator to lose stability.
void NewtonAddBodyImpulse(const NewtonBody* bodyPtr, const dFloat* pointDeltaVeloc, const dFloat* pointPosit)
{
	dgBody* body;
	dgWorld* world;

	body = (dgBody*)bodyPtr;
	world = body->GetWorld();

	dgVector p(pointPosit);
	dgVector v(pointDeltaVeloc);

	world->AddBodyImpulse(body, v, p);
}

#endif


// Name: NewtonCreateBody 
// Create a rigid body.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const NewtonCollision* *collisionPtr - pointer to the collision object.
//
// Return: Pointer to the rigid body.
//
// Remarks: This function creates a Newton rigid body and assigns a *collisionPtr* as the collision geometry representing the rigid body.
// This function increments the reference count of the collision geometry. 
// All event functions are set to NULL and the material gruopID of the body is set to the default GroupID.
//
// See also: NewtonDestroyBody
NewtonBody* NewtonCreateBody(const NewtonWorld* newtonWorld, const NewtonCollision* collision)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndSharedPtr<ndBody> body(new ndBodyDynamic());
	ndBodyDynamic* const dynBody = body->GetAsBodyDynamic();

	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndSharedPtr<ndBodyNotify> bodyNotify(new ndNewtonBodyNotify(world, reinterpret_cast<NewtonBody*>(*body)));

	ndAssert(collision);
	ndShapeInstance instance(*PointerCast<ndShapeInstance, NewtonCollision>(collision));

	dynBody->SetCollisionShape(instance);
	dynBody->SetNotifyCallback(bodyNotify);
	world->AddBody(body);
	return PointerCast<NewtonBody, ndBodyDynamic>(dynBody);
}


// Name: NewtonDestroyBody 
// Destroy a rigid body.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const NewtonBody* *bodyPtr - pointer to the body to be destroyed.
//
// Return: Nothing.
//
// Remarks: If this function is called from inside a simulation step the destruction of the body will be delayed until end of the time step.
// This function will decrease the reference count of the collision geometry by one. If the reference count reaches zero, then the collision 
// geometry will be destroyed. This function will destroy all joints associated with this body.
//
// See also: NewtonCreateBody
void NewtonDestroyBody(const NewtonWorld* newtonWorld, const NewtonBody* body)
{
	TRACE_FUNCTION(__FUNCTION__);

	ndAssert(0);
	ndBodyKinematic* const kinBody = PointerCast<ndBodyKinematic, NewtonBody>(body);
	ndBodyListView::ndNode* node = kinBody->GetSceneNode();
	if (node)
	{
		ndWorld* const world = kinBody->GetScene()->GetWorld();
		world->RemoveBody(kinBody);
	}
}

// Name: NewtonBodySetUserData 
// Store a user defined data value with the body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *void* *userDataPtr - pointer to the user defined user data value.
//
// Return: Nothing.
//
// Remarks: The application can store a user defined value with the Body. This value can be the pointer to a structure containing some application data for special effect. 
// if the application allocate some resource to store the user data, the application can register a joint destructor to get rid of the allocated resource when the body is destroyed
// 
// See also: NewtonBodyGetUserData, NewtonBodySetDestructorCallback 
void  NewtonBodySetUserData(const NewtonBody* bodyPtr, void* userData)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	ndBody* const body = PointerCast<ndBody, NewtonBody>(bodyPtr);
	ndNewtonBodyNotify* const bodyNotify = static_cast<ndNewtonBodyNotify*>(*body->GetNotifyCallback());
	ndAssert(bodyNotify && !strcmp(bodyNotify->ClassName(), ndNewtonBodyNotify::StaticClassName()));
	bodyNotify->m_userData = ndWeakPtr<void>(userData);
}

// Name: NewtonBodyGetUserData
// Retrieve a user defined data value stored with the body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
//
// Return: The user defined data.
//
// Remarks: The application can store a user defined value with a rigid body. This value can be the pointer
// to a structure which is the graphical representation of the rigid body.
// 
// See also: NewtonBodySetUserData
void* NewtonBodyGetUserData(const NewtonBody* bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);

	ndBody* const body = PointerCast<ndBody, NewtonBody>(bodyPtr);
	ndNewtonBodyNotify* const bodyNotify = static_cast<ndNewtonBodyNotify*>(*body->GetNotifyCallback());
	return *bodyNotify->m_userData;
}

// Name: NewtonBodyForEachPolygonDo 
// Iterate thought polygon of the collision geometry of a body calling the function callback. 
//
// Parameters:
// *const NewtonBody* *bodyPtr - is the pointer to the body.
// *NewtonCollisionIterator* callback - application define callback 
//
// Return: nothing
// 
// Remarks: This function can be called by the application in order to show the collision geometry. The application should provide a pointer to the function *NewtonCollisionIterator*, 
// Newton will convert the collision geometry into a polygonal mesh, and will call *callback* for every polygon of the mesh
//
// Remarks: this function affect severely the performance of Newton. The application should call this function only for debugging purpose
//
// Remarks: This function will ignore user define collision mesh
// See also: NewtonWorldForEachBodyDo, NewtonWorldForEachBodyInAABBDo
void NewtonBodyForEachPolygonDo(const NewtonBody* bodyPtr, NewtonCollisionIterator callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);

	//dgBody* body;
	//void* saveCallBack;
	//dgCollision* collision;
	//dgCollisionPolygonalSoup* treeCollision;
	//
	//body = (dgBody*)bodyPtr;
	//collision = body->GetCollision();
	//if (collision->IsType(dgCollisionPolygonalSoup::GetRttiType())) {
	//	treeCollision = (dgCollisionPolygonalSoup*)collision;
	//	saveCallBack = treeCollision->GetCallBack();
	//	if (saveCallBack == NewtonCollisionTree::GetIntersectingPolygons) {
	//		treeCollision->SetCallBack((void*)NewtonCollisionTree::IteratePolygonMesh);
	//		collision->DebugCollision(*body, (DebugCollisionMeshCallback)callback);
	//		treeCollision->SetCallBack(saveCallBack);
	//	}
	//}
	//else {
	//	collision->DebugCollision(*body, (DebugCollisionMeshCallback)callback);
	//}
}

// Name: NewtonBodySetMatrix
// Set the transformation matrix of a rigid body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *const dFloat* *matrixPtr - pointer to an array of 16 floats containing the global matrix of the rigid body.
//
// Return: Nothing.
//
// Remarks: The matrix should be arranged in row-major order (this is the way directX stores matrices).
// If you are using OpenGL matrices (column-major) you will need to transpose you matrices into a local array, before
// passing them to Newton.
//
// See also: NewtonBodyGetMatrix, NewtonBodySetMatrixRecursive 
void NewtonBodySetMatrix(const NewtonBody* bodyPtr, const dFloat* matrixPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBody* const body = PointerCast<ndBody, NewtonBody>(bodyPtr);
	ndMatrix matrix(GetMatrix(matrixPtr));

	matrix.m_front.m_w = ndFloat32(0.0f);
	matrix.m_up.m_w = ndFloat32(0.0f);
	matrix.m_right.m_w = ndFloat32(0.0f);
	matrix.m_posit.m_w = ndFloat32(1.0f);
	body->SetMatrix(matrix);
}

// Name: NewtonBodyGetMatrix 
// Get the transformation matrix of a rigid body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *const dFloat* *matrixPtr - pointer to an array of 16 floats that will hold the global matrix of the rigid body.
//
// Return: Nothing.
//
// Remarks: The matrix should be arranged in row-major order (this is the way direct x stores matrices).
// If you are using OpenGL matrices (column-major) you will need to transpose you matrices into a local array, before
// passing them to Newton.
//
// See also: NewtonBodySetMatrix
void NewtonBodyGetMatrix(const NewtonBody* bodyPtr, dFloat* matrixPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBody* const body = PointerCast<ndBody, NewtonBody>(bodyPtr);
	const ndMatrix matrix(body->GetMatrix());
	ndMemCpy(matrixPtr, &matrix[0][0], sizeof(ndMatrix) / sizeof(ndFloat32));
}

void NewtonBodySetFullMassMatrix(const NewtonBody* const bodyPtr, dFloat mass, const dFloat* const inertiaMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = PointerCast<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();
	ndMatrix inertia(inertiaMatrix);
	body->SetMassMatrix(mass, inertia);
}

// Name: NewtonBodySetMassMatrix 
// Set the mass matrix of a rigid body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - pointer to the body.
// *dFloat* mass - mass value. 
// *dFloat* Ixx - moment of inertia of the first principal axis of inertia of the body.
// *dFloat* Iyy - moment of inertia of the first principal axis of inertia of the body.
// *dFloat* Izz - moment of inertia of the first principal axis of inertia of the body.
//
// Return: Nothing.
//
// Remarks: Newton algorithms have no restriction on the values for the mass, but due to floating point dynamic
// range (24 bit precision) it is best if the ratio between the heaviest and the lightest body in the scene is limited to 200.
// There are no special utility functions in Newton to calculate the moment of inertia of common primitives. 
// The application should specify the inertial values, keeping in mind that realistic inertia values are necessary for
// realistic physics behavior.
//
// See also: NewtonConvexCollisionCalculateInertialMatrix, NewtonBodyGetMassMatrix, NewtonBodyGetInvMass
void NewtonBodySetMassMatrix(const NewtonBody* bodyPtr, dFloat mass, dFloat Ixx, dFloat Iyy, dFloat Izz)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix inertia(ndGetIdentityMatrix());
	inertia[0][0] = Ixx;
	inertia[1][1] = Iyy;
	inertia[2][2] = Izz;
	NewtonBodySetFullMassMatrix(bodyPtr, mass, &inertia[0][0]);
}

// Name: NewtonBodySetOmega 
// Set the global angular velocity of the body.
//
// Parameters:
// *const NewtonBody* *bodyPtr - is the pointer to the body.
// *const dFloat* *omega - pointer to an array of at least three floats containing the angular velocity vector.
//
// See also: NewtonBodyGetOmega
void NewtonBodySetOmega(const NewtonBody* bodyPtr, const dFloat* omega)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = PointerCast<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();

	const ndVector vector(omega[0], omega[1], omega[2], ndFloat32(0.0f));
	body->SetOmega(vector);
}
