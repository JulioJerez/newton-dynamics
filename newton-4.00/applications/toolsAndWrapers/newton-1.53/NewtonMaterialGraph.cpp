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
#include "NewtonMaterialGraph.h"

ndNewtonMaterial::ndNewtonMaterial()
	:ndApplicationMaterial()
	,m_usedData(nullptr)
	,m_onAABBOverlap(nullptr)
	,m_onContactsProcess(nullptr)
	,m_onContactEnd(nullptr)
	//,m_onSubShapeAABBOverlap(nullptr)
{
}

ndNewtonMaterial::ndNewtonMaterial(const ndNewtonMaterial& copy)
	:ndApplicationMaterial(copy)
	,m_usedData(copy.m_usedData)
	,m_onAABBOverlap(copy.m_onAABBOverlap)
	,m_onContactsProcess(copy.m_onContactsProcess)
	,m_onContactEnd(copy.m_onContactEnd)
	//,m_onSubShapeAABBOverlap(copy.m_onSubShapeAABBOverlap)
{
}

ndNewtonMaterial::~ndNewtonMaterial()
{
}

 ndApplicationMaterial* ndNewtonMaterial::Clone() const
{
	return new ndNewtonMaterial(*this);
}

void ndNewtonMaterial::OnContactCallback(const ndContact* const contactPtr, ndFloat32 timestep) const
{
	if (m_onContactsProcess)
	{
		const NewtonContact* const contact = PointerCast<NewtonContact, ndContact>(contactPtr);
		const ndContactPointList& contactPoints = contactPtr->GetContactPoints();
		for (ndContactPointList::ndNode* node = contactPoints.GetFirst(); node; node = node->GetNext())
		{
			const ndContactMaterial& contactPointMaterial = node->GetInfo();
			const NewtonContact* const materialPoint = PointerCast<NewtonContact, ndContactMaterial>(&contactPointMaterial);
			//there is a bug in 1.53, is use the materia as contact point, 
			//so pass the same pointer
			//m_onContactsProcess(material, materialPoint);
			const NewtonMaterial* const material = PointerCast<NewtonMaterial, ndContactMaterial>(&contactPointMaterial);
			m_onContactsProcess(material, materialPoint);
		}

		if (m_onContactEnd)
		{
			//there is a bug in 1.53, is use the materia as contact point, 
			//this may be wrong, maybe it souel pass teh contact instead of the material
			const NewtonMaterial* const material = PointerCast<NewtonMaterial, ndMaterial>(contactPtr->GetMaterial());
			m_onContactEnd(material);
		}
	}
}

bool ndNewtonMaterial::OnAabbOverlap(const ndBodyKinematic* const bodyPtr0, const ndBodyKinematic* const bodyPtr1) const
{
	ndAssert(0);
	//if (m_onAABBOverlap)
	//{
	//	const NewtonBody* const body0 = PointerCast<NewtonBody, ndBodyKinematic>(bodyPtr0);
	//	const NewtonBody* const body1 = PointerCast<NewtonBody, ndBodyKinematic>(bodyPtr1);
	//	const ndContact* const contact = bodyPtr0->GetContactMap().FindContact(bodyPtr0, bodyPtr1);
	//	if (!contact)
	//	{
	//		return true;
	//	}
	//	return m_onAABBOverlap(PointerCast<NewtonJoint, ndContact>(contact), ndFloat32(0.0f), 0);
	//}
	return true;
}

bool ndNewtonMaterial::OnAabbOverlap(const ndContact* const contact, ndFloat32 timestep, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const
{
	//if (m_onSubShapeAABBOverlap && (instance0.m_subCollisionHandle || instance1.m_subCollisionHandle))
	//{
	//	NewtonBody* const body0 = PointerCast<NewtonBody, ndBodyKinematic>(contact->GetBody0());
	//	NewtonBody* const body1 = PointerCast<NewtonBody, ndBodyKinematic>(contact->GetBody1());
	//	return m_onSubShapeAABBOverlap(PointerCast<NewtonJoint, ndContact>(contact), timestep, body0, *instance0.m_subCollisionHandle, body1, *instance1.m_subCollisionHandle, 0);
	//}
	//else if (m_onAABBOverlap)
	//{
	//	return m_onAABBOverlap(PointerCast<NewtonJoint, ndContact>(contact), timestep, 0);
	//}

	if (m_onAABBOverlap)
	{
		NewtonBody* const body0 = PointerCast<NewtonBody, ndBodyKinematic>(contact->GetBody0());
		NewtonBody* const body1 = PointerCast<NewtonBody, ndBodyKinematic>(contact->GetBody1());
		NewtonMaterial* const material = PointerCast<NewtonMaterial, ndNewtonMaterial>(this);
		return m_onAABBOverlap(material, body0, body1);
	}
	return true;
}

#if 0

// ***************************************************************************************************************
//
// Name: Material setup interface
//
// ***************************************************************************************************************

// Name: NewtonMaterialSetDefaultCollidable 
// Set the material defined by the interaction between two physics groups to be collidable or non-collidable by default.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *int* id0 - group id0
// *int* id1 - group id1
// *int* state - state for this material: 1 = collidable; 0 = non collidable
// 
// Return: Nothing.
void NewtonMaterialSetDefaultCollidable(const NewtonWorld* newtonWorld, int id0, int id1, int state)
{
	Newton* world;
	dgContactMaterial* material;

	world = (Newton*)newtonWorld;
	material = world->GetMaterial(id0, id1);

	material->m_collisionEnable = state ? true : false;
}


// Name: NewtonMaterialSetContinuousCollisionMode 
// Set the material defined by the interaction between two physics groups to enable or desable continue collision.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *int* id0 - group id0
// *int* id1 - group id1
// *int* state - state for this material: 1 = continue collision on; 0 = continue collision off, default mode is on
// 
// Return: Nothing.
//
// Remarks: continue collision mode enable allow the engine to predict colliding contact on rigid bodies
// Moving at high speed of subject to strong forces.
//
// Remarks: continue collision mode does not prevent rigid bodies from interpenetration instead it prevent bodies from 
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
// See also: NewtonBodySetContinuousCollisionMode
void NewtonMaterialSetContinuousCollisionMode(const NewtonWorld* newtonWorld, int id0, int id1, int state)
{
	Newton* world;
	dgContactMaterial* material;

	world = (Newton*)newtonWorld;
	material = world->GetMaterial(id0, id1);

	material->m_collisionContinueCollisionEnable = state ? true : false;
}


// Name: NewtonMaterialSetDefaultFriction 
// Set the default coefficients of friction for the material defined by the interaction between two physics groups.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *int* id0 - group id0
// *int* id1 - group id1
// *dFloat* staticFriction -	static friction coefients
// *dFloat* kineticFriction - dynamic coefficient of friction
// 
// Return: Nothing.
//
// Remarks: *staticFriction* and *kineticFriction* must be positive values. *kineticFriction* must be lower than *staticFriction*.
// It is recommended that *staticFriction* and *kineticFriction* be set to a value lower or equal to 1.0, however because some synthetic materials
// can have higher than one coeficient of friction Newton allows for the coeficient of friction to be as high as 2.0.
void NewtonMaterialSetDefaultFriction(const NewtonWorld* newtonWorld, int id0, int id1, dFloat staticFriction, dFloat kineticFriction)
{
	dFloat stat;
	dFloat kine;
	Newton* world;
	dgContactMaterial* material;

	world = (Newton*)newtonWorld;
	material = world->GetMaterial(id0, id1);

	staticFriction = dgAbsf(staticFriction);
	kineticFriction = dgAbsf(kineticFriction);

	if (material) {
		if (staticFriction >= 1.e-2f) {
			stat = ClampValue(staticFriction, dFloat(0.01f), dFloat(2.0f));
			kine = ClampValue(kineticFriction, dFloat(0.01f), dFloat(2.0f));
			stat = GetMax(stat, kine);
			material->m_staticFriction0 = stat;
			material->m_staticFriction1 = stat;
			material->m_dynamicFriction0 = kine;
			material->m_dynamicFriction1 = kine;
		}
		else {
			material->m_friction0Enable = false;
			material->m_friction1Enable = false;
		}
	}
}


// Name: NewtonMaterialSetDefaultElasticity 
// Set the default coefficients of restitution (elasticity) for the material defined by the interaction between two physics groups.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *int* id0 - group id0
// *int* id1 - group id1
// *dFloat* elasticCoef - static friction coefficients
// 
// Return: Nothing.
//
// Remarks: *elasticCoef* must be a positive value. 
// It is recommended that *elasticCoef* be set to a value lower or equal to 1.0
void NewtonMaterialSetDefaultElasticity(const NewtonWorld* newtonWorld, int id0, int id1, dFloat elasticCoef)
{
	Newton* world;
	dgContactMaterial* material;

	world = (Newton*)newtonWorld;
	material = world->GetMaterial(id0, id1);

	material->m_restitution = ClampValue(elasticCoef, dFloat(0.01f), dFloat(2.0f));
}



// Name: NewtonMaterialSetDefaultSoftness 
// Set the default softness coefficients for the material defined by the interaction between two physics groups.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *int* id0 - group id0
// *int* id1 - group id1
// *dFloat* softnessCoef - softness coefficient
// 
// Return: Nothing.
//
// Remarks: *softnessCoef* must be a positive value. 
// It is recommended that *softnessCoef* be set to value lower or equal to 1.0
// A low value for *softnessCoef* will make the material soft. A typical value for *softnessCoef* is 0.15
void NewtonMaterialSetDefaultSoftness(const NewtonWorld* newtonWorld, int id0, int id1, dFloat softnessCoef)
{
	Newton* world;
	dgContactMaterial* material;

	world = (Newton*)newtonWorld;
	material = world->GetMaterial(id0, id1);

	material->m_softness = ClampValue(softnessCoef, dFloat(0.01f), dFloat(dgFloat32(1.0f)));
}




// Name: NewtonMaterialSetCollisionCallback 
// Set userData and the functions event handlers for the material defined by the interaction between two physics groups.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *int* id0 - group id0.
// *int* id1 - group id1.
// *void* *userData - user data value.
// *NewtonContactBegin* beginCallback - address of the event function called before contact calculation for collision. This parameter can be NULL.
// *NewtonContactProcess* processCallback - address of the event function called for every contact resulting from contact calculation. This parameter can be NULL.
// *NewtonContactEnd* endCallback - address of the event function called after all contacts are processed. This parameter can be NULL.
// 
// Return: Nothing.
//
// Remarks: When the AABB extend of the collision geometry of two bodies overlap, Newton collision system retrieves the material 
// interaction that defines the behavior between the pair of bodies. The material interaction is collected from a database of materials, 
//  indexed by the material gruopID assigned to the bodies. If the material is tagged as non collidable, 
// then no action is taken and the simulation continues. 
// If the material is tagged as collidable, and a *beginCallback* was set for this material, then the *beginCallback* function is called. 
// If the function  *beginCallback* returns 0, no further action is taken for this material (this can be use to ignore the interaction under 
// certain conditions). If the function  *beginCallback* returns 1, Newton proceeds to calculate the array of contacts for the pair of 
// colliding bodies. If the function *processCallback* was set, the application receives a callback for every contact found between the 
// two colliding bodies. Here the application can perform fine grain control over the behavior of the collision system. For example, 
// rejecting the contact, making the contact frictionless, applying special effects to the surface etc. 
// After all contacts are processed and if the function *endCallback* was set, Newton calls *endCallback*. 
// Here the application can collect information gathered during the contact-processing phase and provide some feedback to the player. 
// A typical use for the material callback is to play sound effects. The application passes the address of structure in the *userData* along with 
// three event function callbacks. When the function *beginCallback* is called by Newton, the application resets a variable say *maximumImpactSpeed*. 
// Then for every call to the function *processCallback*, the application compares the impact speed for this contact with the value of 
// *maximumImpactSpeed*, if the value is larger, then the application stores the new value along with the position, and any other quantity desired. 
// When the application receives the call to *endCallback* the application plays a 3d sound based in the position and strength of the contact.
void NewtonMaterialSetCompondCollisionCallback(const NewtonWorld* newtonWorld, int id0, int id1, NewtonContactBegin beginCallback)
{
	Newton* world;
	dgContactMaterial* material;

	world = (Newton*)newtonWorld;
	material = world->GetMaterial(id0, id1);

	material->SetCompoundCollisionCallback((OnContactBegin)beginCallback);
}


// Name: NewtonMaterialGetUserData 
// Get userData associated with this material.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *int* id0 - group id0.
// *int* id1 - group id1.
// 
// Return: Nothing.
//
void* NewtonMaterialGetUserData(const NewtonWorld* newtonWorld, int id0, int id1)
{
	Newton* world;
	dgContactMaterial* material;

	world = (Newton*)newtonWorld;
	material = world->GetMaterial(id0, id1);

	return material->m_userData;
}


// ***************************************************************************************************************
//
// Name: Contact behavior control interface
//
// ***************************************************************************************************************


// Name: NewtonMaterialDisableContact 
// Disable processing for the contact. 
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair
// 
// Return: Nothing.
//
// Remarks: This function can only be called from a material callback event handler.
// 
// See also: NewtonMaterialSetCollisionCallback
void NewtonMaterialDisableContact(const NewtonMaterial* materialHandle)
{
	dgContactMaterial* material;

	material = (dgContactMaterial*)materialHandle;
	material->m_collisionEnable = false;
}


// Name: NewtonMaterialGetMaterialPairUserData 
// Get the userData set by the application when it created this material pair.
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair
// 
// Return: Application user data.
//
// Remarks: This function can only be called from a material callback event handler.
// 
// See also: NewtonMaterialSetCollisionCallback
void* NewtonMaterialGetMaterialPairUserData(const NewtonMaterial* materialHandle)
{
	dgContactMaterial* material;

	material = (dgContactMaterial*)materialHandle;
	return material->m_userData;
}

// Name: NewtonMaterialGetContactFaceAttribute 
// Return the face attribute assigned to this face when for a user defined collision or a Newton collision tree.
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair
// 
// Return: face attribute for collision trees. Zero if the contact was generated by two convex collisions. 
//
// Remarks: This function can only be called from a material callback event handler.
//
// Remarks: this function can be used by the application to retrieve the face id of a polygon for a collision tree.
// 
// See also: NewtonMaterialSetCollisionCallback
unsigned NewtonMaterialGetContactFaceAttribute(const NewtonMaterial* materialHandle)
{
	dgContactMaterial* material;

	material = (dgContactMaterial*)materialHandle;
	return (unsigned)material->m_userId;
}

// Name: NewtonMaterialGetCurrentTimestep 
// Get the current time step.
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair
// 
// Return: the current time step.
//
// Remarks: This function can only be called from a material callback event handler. The function can be useful for the implementation of powered contacts.
// 
// See also: NewtonMaterialSetCollisionCallback
dFloat NewtonMaterialGetCurrentTimestep(const NewtonMaterial* materialHandle)
{
	dgContactMaterial* material;

	material = (dgContactMaterial*)materialHandle;
	//	return material->m_currTimestep;

	_ASSERTE(material->m_body0);
	return material->m_body0->GetWorld()->GetTimeStep();
}

// Name: NewtonMaterialGetContactForce 
// Get the contact force vector in global space.
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair.
// *dFloat* *forcePtr,  - pointer to an array of at least three floats to hold the force vector in global space.
// 
// Return: Nothing.
//
// Remarks: The contact force value is only valid when calculating resting contacts. This means if two bodies collide with 
// non zero relative velocity, the reaction force will be an impulse, which is not a reaction force, this will return zero vector. 
// this function will only return meaningful values when the colliding bodies are at rest.
//
// Remarks: This function can only be called from a material callback event handler.
// 
// See also: NewtonMaterialSetCollisionCallback
void NewtonMaterialGetContactForce(const NewtonMaterial* materialHandle, dFloat* forcePtr)
{
	dgContactMaterial* material;

	material = (dgContactMaterial*)materialHandle;

	dgVector force(material->m_normal.Scale(material->m_normal_Force) +
		material->m_dir0.Scale(material->m_dir0_Force) +
		material->m_dir1.Scale(material->m_dir1_Force));

	forcePtr[0] = force.m_x;
	forcePtr[1] = force.m_y;
	forcePtr[2] = force.m_z;
}



// Name: NewtonMaterialGetContactTangentDirections 
// Get the contact tangent vector to the contact point. 
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair.
// *dFloat* *dir0  - pointer to an array of at least three floats to hold the contact primary tangent vector.
// *dFloat* *dir1  - pointer to an array of at least three floats to hold the contact secondary tangent vector.
// 
// Return: Nothing.
//
// Remarks: This function can only be called from a material callback event handler.
// 
// See also: NewtonMaterialSetCollisionCallback
void NewtonMaterialGetContactTangentDirections(const NewtonMaterial* materialHandle, dFloat* dir0, dFloat* dir1)
{
	dgContactMaterial* material;

	material = (dgContactMaterial*)materialHandle;

	dir0[0] = material->m_dir0.m_x;
	dir0[1] = material->m_dir0.m_y;
	dir0[2] = material->m_dir0.m_z;

	dir1[0] = material->m_dir1.m_x;
	dir1[1] = material->m_dir1.m_y;
	dir1[2] = material->m_dir1.m_z;
}


// Name: NewtonMaterialGetBodyCollisionID
// Retrieve a user defined value stored with a convex collision primitive.
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair.
// *const NewtonBody* *bodyPtr - is the pointer to the body.
// 
// Return: a user defined value. Zero if not id was stored with the collision primitive.
//
// Remarks: the application can store an id with any collision primitive. This id can be used to identify what type of collision primitive generated a contact.
// This function can only be called from a contact callback,  
//
// Remarks: this function can only be called from a contact process callback. If called from contact callback begin this function will crash the application.
//
// See also: NewtonConvexCollisionSetUserID, NewtonCreateBox, NewtonCreateSphere
unsigned NewtonMaterialGetBodyCollisionID(const NewtonMaterial* material, const NewtonBody* body)
{
	dgUnsigned64 val;
	dgBody* bodyPtr;
	dgCollision* collision;
	dgContactMaterial* materialPtr;

	bodyPtr = (dgBody*)body;
	materialPtr = (dgContactMaterial*)material;

	collision = materialPtr->m_collision0;
	if (bodyPtr == materialPtr->m_body1) {
		collision = materialPtr->m_collision1;
	}

	val = 0;
	// the convex type is 0x7437fbdd
	if (collision->IsType(0x7437fbdd)) {
		val = dgUnsigned64(PointerToInt(collision->GetUserData()));
	}
	return dgUnsigned32(val);
}



// Name: NewtonMaterialSetContactSoftness 
// Override the default softness value for the contact.
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair.
// *dFloat* softness  - softness value, must be positive.
// 
// Return: Nothing.
//
// Remarks: This function can only be called from a material callback event handler.
// 
// See also: NewtonMaterialSetCollisionCallback, NewtonMaterialSetDefaultSoftness
void NewtonMaterialSetContactSoftness(const NewtonMaterial* materialHandle, dFloat softness)
{
	dgContactMaterial* material;

	material = (dgContactMaterial*)materialHandle;
	material->m_softness = ClampValue(softness, dFloat(0.01f), dFloat(0.7f));
}


// Name: NewtonMaterialSetContactNormalAcceleration
// Force the contact point to have a non-zero acceleration aligned this the contact normal.
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair.
// *dFloat* accel - desired contact acceleration, Must be a positive value
// 
// Return: Nothing.
//
// Remarks: This function can only be called from a material callback event handler.
//
// Remarks: This function can be used for spacial effects like implementing jump, of explosive contact in a call back.
// 
// See also: NewtonMaterialSetCollisionCallback
void NewtonMaterialSetContactNormalAcceleration(const NewtonMaterial* materialHandle, dFloat accel)
{
	dgContactMaterial* material;

	material = (dgContactMaterial*)materialHandle;

	//	if (accel > dFloat (0.0f)) {
	material->m_normal_Force = accel;
	material->m_overrideNormalAccel = true;
	//	}
}

// Name: NewtonMaterialSetContactTangentAcceleration
// Force the contact point to have a non-zero acceleration along the surface plane.
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair.
// *dFloat* accel - desired contact acceleration.
// *int* index - index to the tangent vector. 0 for primary tangent vector or 1 for the secondary tangent vector.
// 
// Return: Nothing.
//
// Remarks: This function can only be called from a material callback event handler.
// 
// See also: NewtonMaterialSetCollisionCallback, NewtonMaterialContactRotateTangentDirections
void NewtonMaterialSetContactTangentAcceleration(const NewtonMaterial* materialHandle, dFloat accel, int index)
{
	dgContactMaterial* material;

	material = (dgContactMaterial*)materialHandle;
	if (index) {
		material->m_dir1_Force = accel;
		material->m_override1Accel = true;
	}
	else {
		material->m_dir0_Force = accel;
		material->m_override0Accel = true;
	}
}

/*!
  Remove all groups ID from the Newton world.

  @param *newtonWorld pointer to the Newton world.

  @return Nothing.

  This function removes all groups ID from the Newton world.
  This function must be called after there are no more rigid bodies in the word.

  See also: ::NewtonDestroyAllBodies
*/
void NewtonMaterialDestroyAllGroupID(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->ClearMaterials();
}


/*!
  Set an imaginary thickness between the collision geometry of two colliding bodies whose physics
  properties are defined by this material pair

  @param *newtonWorld pointer to the Newton world.
  @param  id0 - group id0
  @param  id1 - group id1
  @param thickness material thickness a value form 0.0 to 0.125; the default surface value is 0.0

  @return Nothing.

  when two bodies collide the engine resolve contact inter penetration by applying a small restoring
  velocity at each contact point. By default this restoring velocity will stop when the two contacts are
  at zero inter penetration distance. However by setting a non zero thickness the restoring velocity will
  continue separating the contacts until the distance between the two point of the collision geometry is equal
  to the surface thickness.

  Surfaces thickness can improve the behaviors of rolling objects on flat surfaces.

  Surface thickness does not alter the performance of contact calculation.
*/
void NewtonMaterialSetSurfaceThickness(const NewtonWorld* const newtonWorld, int id0, int id1, dFloat thickness)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//dgContactMaterial* const material = world->GetMaterial(dgUnsigned32(id0), dgUnsigned32(id1));
	//material->m_skinThickness = dgClamp(thickness, dgFloat32(0.0f), DG_MAX_COLLISION_AABB_PADDING * dgFloat32(0.5f));
	ndAssert(0);
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
	//Newton* const world = (Newton*)newtonWorld;
	//dgContactMaterial* const material = world->GetMaterial(dgUnsigned32(id0), dgUnsigned32(id1));
	//
	//material->m_softness = dgClamp(softnessCoef, dFloat(0.01f), dFloat(dgFloat32(1.0f)));

	ndAssert(0);
}

void NewtonMaterialJointResetIntraJointCollision(const NewtonWorld* const newtonWorld, int id0, int id1)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//dgContactMaterial* const material = world->GetMaterial(dgUnsigned32(id0), dgUnsigned32(id1));
	//material->m_flags |= dgContactMaterial::m_resetSkeletonIntraCollision;
	ndAssert(0);
}

void NewtonMaterialJointResetSelftJointCollision(const NewtonWorld* const newtonWorld, int id0, int id1)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//dgContactMaterial* const material = world->GetMaterial(dgUnsigned32(id0), dgUnsigned32(id1));
	//material->m_flags |= dgContactMaterial::m_resetSkeletonSelfCollision;

	ndAssert(0);
}



/*!
  Return the face attribute assigned to this face when for a user defined collision or a Newton collision tree.

  @param materialHandle pointer to a material pair

  @return face attribute for collision trees. Zero if the contact was generated by two convex collisions.

  This function can only be called from a material callback event handler.

  this function can be used by the application to retrieve the face id of a polygon for a collision tree.

  See also: ::NewtonMaterialSetCollisionCallback
*/
unsigned NewtonMaterialGetContactFaceAttribute(const NewtonMaterial* const materialHandle)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContactMaterial* const material = (dgContactMaterial*)materialHandle;
	//return (unsigned)(material->m_shapeId1);
	ndAssert(0);
	return 0;
}


dFloat NewtonMaterialGetContactMaxTangentImpact(const NewtonMaterial* const materialHandle, int index)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgContactMaterial* const material = (dgContactMaterial*)materialHandle;
	//return index ? material->m_dir1_Force.m_impact : material->m_dir0_Force.m_impact;
	ndAssert(0);
	return 0;
}

dFloat NewtonMaterialGetContactPenetration(const NewtonMaterial* const materialHandle)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContactMaterial* const material = (dgContactMaterial*)materialHandle;
	//return material->m_penetration;
	ndAssert(0);
	return 0;
}

/*!
  Override the default softness value for the contact.

  @param materialHandle pointer to a material pair.
  @param softness softness value, must be positive.

  @return Nothing.

  This function can only be called from a material callback event handler.

  See also: ::NewtonMaterialSetCollisionCallback, ::NewtonMaterialSetDefaultSoftness
*/
void NewtonMaterialSetContactSoftness(const NewtonMaterial* const materialHandle, dFloat softness)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContactMaterial* const material = (dgContactMaterial*)materialHandle;
	//material->m_softness = dgClamp(softness, dFloat(0.01f), dFloat(0.7f));
	ndAssert(0);
}

/*!
  Override the default contact skin thickness value for the contact.

  @param materialHandle pointer to a material pair.
  @param thickness skin thickness value, must be positive.

  @return Nothing.

  This function can only be called from a material callback event handler.

  See also: ::NewtonMaterialSetCollisionCallback, ::NewtonMaterialSetDefaultSoftness
*/
void NewtonMaterialSetContactThickness(const NewtonMaterial* const materialHandle, dFloat thickness)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgAssert(thickness >= dgFloat32(0.0f));
	//dgContactMaterial* const material = (dgContactMaterial*)materialHandle;
	//material->m_skinThickness = thickness;
	ndAssert(0);
}

/*!
  Enable or disable friction calculation for this contact.

  @param materialHandle pointer to a material pair.
  @param state* new state. 0 makes the contact frictionless along the index tangent vector.
  @param index index to the tangent vector. 0 for primary tangent vector or 1 for the secondary tangent vector.

  @return Nothing.

  This function can only be called from a material callback event handler.

  See also: ::NewtonMaterialSetCollisionCallback
*/
void NewtonMaterialSetContactFrictionState(const NewtonMaterial* const materialHandle, int state, int index)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContactMaterial* const material = (dgContactMaterial*)materialHandle;
	//
	//if (index) {
	//	if (state) {
	//		material->m_flags |= dgContactMaterial::m_friction1Enable;
	//	}
	//	else {
	//		material->m_flags &= ~dgContactMaterial::m_friction1Enable;
	//	}
	//}
	//else {
	//	if (state) {
	//		material->m_flags |= dgContactMaterial::m_friction0Enable;
	//	}
	//	else {
	//		material->m_flags &= ~dgContactMaterial::m_friction0Enable;
	//	}
	//}
	ndAssert(0);
}


/*!
  Force the contact point to have a non-zero acceleration aligned this the contact normal.

  @param materialHandle pointer to a material pair.
  @param accel desired contact acceleration, Must be a positive value

  @return Nothing.

  This function can only be called from a material callback event handler.

  This function can be used for spacial effects like implementing jump, of explosive contact in a call back.

  See also: ::NewtonMaterialSetCollisionCallback
*/
void NewtonMaterialSetContactNormalAcceleration(const NewtonMaterial* const materialHandle, dFloat accel)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContactMaterial* const material = (dgContactMaterial*)materialHandle;
	//material->m_normal_Force.m_force = accel;
	//material->m_flags |= dgContactMaterial::m_overrideNormalAccel;
	ndAssert(0);
}

void NewtonMaterialSetAsSoftContact(const NewtonMaterial* const materialHandle, dFloat relaxation)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContactMaterial* const material = (dgContactMaterial*)materialHandle;
	//material->SetAsSoftContact(relaxation);
	ndAssert(0);
}

/*!
  Force the contact point to have a non-zero acceleration along the surface plane.

  @param materialHandle pointer to a material pair.
  @param accel desired contact acceleration.
  @param index index to the tangent vector. 0 for primary tangent vector or 1 for the secondary tangent vector.

  @return Nothing.

  This function can only be called from a material callback event handler.

  See also: ::NewtonMaterialSetCollisionCallback, ::NewtonMaterialContactRotateTangentDirections
*/
void NewtonMaterialSetContactTangentAcceleration(const NewtonMaterial* const materialHandle, dFloat accel, int index)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContactMaterial* const material = (dgContactMaterial*)materialHandle;
	//if (index) {
	//	material->m_dir1_Force.m_force = accel;
	//	material->m_flags |= dgContactMaterial::m_override1Accel;
	//}
	//else {
	//	material->m_dir0_Force.m_force = accel;
	//	material->m_flags |= dgContactMaterial::m_override0Accel;
	//}
	ndAssert(0);
}

void NewtonMaterialSetContactTangentFriction(const NewtonMaterial* const materialHandle, dFloat friction, int index)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContactMaterial* const material = (dgContactMaterial*)materialHandle;
	//friction = dgMax(dFloat(0.01f), dgAbs(friction));
	//if (index) {
	//	material->m_flags |= dgContactMaterial::m_override1Friction;
	//	dgAssert(index == 1);
	//	material->m_staticFriction1 = friction;
	//	material->m_dynamicFriction1 = friction;
	//}
	//else {
	//	material->m_flags |= dgContactMaterial::m_override0Friction;
	//	material->m_staticFriction0 = friction;
	//	material->m_dynamicFriction0 = friction;
	//}
	ndAssert(0);
}

/*!
  Set the new direction of the for this contact point.

  @param materialHandle pointer to a material pair.
  @param *direction pointer to an array of at least three floats holding the direction vector.

  @return Nothing.

  This function can only be called from a material callback event handler.
  This function changes the basis of the contact point to one where the contact normal is aligned to the new direction vector
  and the tangent direction are recalculated to be perpendicular to the new contact normal.

  In 99.9% of the cases the collision system can calculates a very good contact normal.
  however this algorithm that calculate the contact normal use as criteria the normal direction
  that will resolve the inter penetration with the least amount on motion.
  There are situations however when this solution is not the best. Take for example a rolling
  ball over a tessellated floor, when the ball is over a flat polygon, the contact normal is always
  perpendicular to the floor and pass by the origin of the sphere, however when the sphere is going
  across two adjacent polygons, the contact normal is now perpendicular to the polygons edge and this does
  not guarantee they it will pass bay the origin of the sphere, but we know that the best normal is always
  the one passing by the origin of the sphere.

  See also: ::NewtonMaterialSetCollisionCallback, ::NewtonMaterialContactRotateTangentDirections
*/
void NewtonMaterialSetContactNormalDirection(const NewtonMaterial* const materialHandle, const dFloat* const direction)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContactMaterial* const material = (dgContactMaterial*)materialHandle;
	//dgVector normal(direction[0], direction[1], direction[2], dgFloat32(0.0f));
	//
	////dgAssert (normal.DotProduct3(material->m_normal) > dgFloat32 (0.01f));
	//if (normal.DotProduct(material->m_normal).GetScalar() < dgFloat32(0.0f)) {
	//	normal = normal * dgVector::m_negOne;
	//}
	//material->m_normal = normal;
	//
	//dgMatrix matrix(normal);
	//material->m_dir1 = matrix.m_up;
	//material->m_dir0 = matrix.m_right;
	ndAssert(0);
}

void NewtonMaterialSetContactPosition(const NewtonMaterial* const materialHandle, const dFloat* const position)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContactMaterial* const material = (dgContactMaterial*)materialHandle;
	//dgVector point(position[0], position[1], position[2], dgFloat32(1.0f));
	//material->m_point = point;
	ndAssert(0);
}


/*!
  Rotate the tangent direction of the contacts until the primary direction is aligned with the alignVector.

  @param *materialHandle pointer to a material pair.
  @param *alignVector pointer to an array of at least three floats holding the aligning vector.

  @return Nothing.

  This function can only be called from a material callback event handler.
  This function rotates the tangent vectors of the contact point until the primary tangent vector and the align vector
  are perpendicular (ex. when the dot product between the primary tangent vector and the alignVector is 1.0). This
  function can be used in conjunction with NewtonMaterialSetContactTangentAcceleration in order to
  create special effects. For example, conveyor belts, cheap low LOD vehicles, slippery surfaces, etc.

  See also: ::NewtonMaterialSetCollisionCallback, ::NewtonMaterialSetContactNormalDirection
*/
void NewtonMaterialContactRotateTangentDirections(const NewtonMaterial* const materialHandle, const dFloat* const alignVector)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContactMaterial* const material = (dgContactMaterial*)materialHandle;
	//
	//const dgVector dir0(alignVector[0], alignVector[1], alignVector[2], dgFloat32(0.0f));
	//
	//dgVector dir1(material->m_normal.CrossProduct(dir0));
	//dgAssert(dir1.m_w == dgFloat32(0.0f));
	//dFloat mag2 = dir1.DotProduct(dir1).GetScalar();
	//if (mag2 > dgFloat32(1.0e-6f)) {
	//	material->m_dir1 = dir1.Normalize();
	//	material->m_dir0 = material->m_dir1.CrossProduct(material->m_normal);
	//}
	ndAssert(0);
}



/*!
  Get the contact tangent vector to the contact point.

  @param materialHandle pointer to a material pair.
  @param *body pointer to body
  @param  *dir0 - pointer to an array of at least three floats to hold the contact primary tangent vector.
  @param  *dir1 - pointer to an array of at least three floats to hold the contact secondary tangent vector.

  @return Nothing.

  This function can only be called from a material callback event handler.

  See also: ::NewtonMaterialSetCollisionCallback
*/
void NewtonMaterialGetContactTangentDirections(const NewtonMaterial* const materialHandle, const NewtonBody* const body, dFloat* const dir0, dFloat* const dir1)
{
	TRACE_FUNCTION(__FUNCTION__);
	const ndContactMaterial* const material = PointerCast<ndContactMaterial, NewtonMaterial>(materialHandle);

	dir0[0] = material->m_dir0.m_x;
	dir0[1] = material->m_dir0.m_y;
	dir0[2] = material->m_dir0.m_z;
	
	dir1[0] = material->m_dir1.m_x;
	dir1[1] = material->m_dir1.m_y;
	dir1[2] = material->m_dir1.m_z;
	
	if (PointerCast<ndBody, NewtonBody>(body) != material->m_body0)
	{
		dir0[0] *= ndFloat32(-1.0f);
		dir0[1] *= ndFloat32(-1.0f);
		dir0[2] *= ndFloat32(-1.0f);
				   
		dir1[0] *= ndFloat32(-1.0f);
		dir1[1] *= ndFloat32(-1.0f);
		dir1[2] *= ndFloat32(-1.0f);
	}
}

/*!
  Get the contact force vector in global space.

  @param materialHandle pointer to a material pair.
  @param *body pointer to body
  @param *forcePtr pointer to an array of at least three floats to hold the force vector in global space.

  @return Nothing.

  The contact force value is only valid when calculating resting contacts. This means if two bodies collide with
  non zero relative velocity, the reaction force will be an impulse, which is not a reaction force, this will return zero vector.
  this function will only return meaningful values when the colliding bodies are at rest.

  This function can only be called from a material callback event handler.

  See also: ::NewtonMaterialSetCollisionCallback
*/
void NewtonMaterialGetContactForce(const NewtonMaterial* const materialHandle, const NewtonBody* const body, dFloat* const forcePtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	const ndContactMaterial* const material = PointerCast<ndContactMaterial, NewtonMaterial>(materialHandle);

	ndVector force(material->m_normal.Scale(material->m_normal_Force.m_force) + material->m_dir0.Scale(material->m_dir0_Force.m_force) + material->m_dir1.Scale(material->m_dir1_Force.m_force));
	
	forcePtr[0] = force.m_x;
	forcePtr[1] = force.m_y;
	forcePtr[2] = force.m_z;
	
	if (PointerCast<ndBody, NewtonBody>(body) != material->m_body0)
	{
		forcePtr[0] *= ndFloat32(-1.0f);
		forcePtr[1] *= ndFloat32(-1.0f);
		forcePtr[2] *= ndFloat32(-1.0f);
	}
}


/*!
  Override the default value of the kinetic and static coefficient of friction for this contact.

  @param materialHandle pointer to a material pair.
  @param staticFrictionCoef static friction coefficient. Must be positive.
  @param kineticFrictionCoef static friction coefficient. Must be positive.
  @param index index to the tangent vector. 0 for primary tangent vector or 1 for the secondary tangent vector.

  @return Nothing.

  This function can only be called from a material callback event handler.

  It is recommended that *coef* be set to a value lower or equal to 1.0, however because some synthetic materials
  can have hight than one coefficient of friction Newton allows for the coefficient of friction to be as high as 2.0.

  the value *staticFrictionCoef* and *kineticFrictionCoef* will be clamped between 0.01f and 2.0.
  If the application wants to set a kinetic friction higher than the current static friction it must increase the static friction first.

  See also: ::NewtonMaterialSetCollisionCallback, ::NewtonMaterialSetDefaultFriction
*/
void NewtonMaterialSetContactFrictionCoef(const NewtonMaterial* const materialHandle, dFloat staticFrictionCoef, dFloat kineticFrictionCoef, int index)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndContactMaterial* const material = PointerCast<ndContactMaterial, NewtonMaterial>(materialHandle);
	if (staticFrictionCoef < kineticFrictionCoef) 
	{
		staticFrictionCoef = kineticFrictionCoef;
	}
	
	if (index) 
	{
		material->m_material.m_staticFriction1 = ndClamp(staticFrictionCoef, dFloat(0.01f), dFloat(2.0f));
		material->m_material.m_kineticFriction1 = ndClamp(kineticFrictionCoef, dFloat(0.01f), dFloat(2.0f));
	}
	else 
	{
		material->m_material.m_staticFriction0 = ndClamp(staticFrictionCoef, dFloat(0.01f), dFloat(2.0f));
		material->m_material.m_kineticFriction0 = ndClamp(kineticFrictionCoef, dFloat(0.01f), dFloat(2.0f));
	}
}

/*!
  Override the default elasticity (coefficient of restitution) value for the contact.

  @param materialHandle pointer to a material pair.
  @param restitution elasticity value, must be positive.

  @return Nothing.

  This function can only be called from a material callback event handler.

  See also: ::NewtonMaterialSetCollisionCallback, ::NewtonMaterialSetDefaultElasticity
*/
void NewtonMaterialSetContactElasticity(const NewtonMaterial* const materialHandle, dFloat restitution)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndContactMaterial* const material = PointerCast<ndContactMaterial, NewtonMaterial>(materialHandle);
	material->m_material.m_restitution = ndClamp(restitution, dFloat(0.0f), dFloat(1.0f));
}


void NewtonMaterialSetCallbackUserData(const NewtonWorld* const newtonWorld, int id0, int id1, void* const userData)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//dgContactMaterial* const material = world->GetMaterial(dgUnsigned32(id0), dgUnsigned32(id1));
	//material->SetUserData(userData);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndNewtonMaterial* const material = PointerCast<ndNewtonMaterial, ndMaterial>(world->GetMaterial(id0, id1));
	material->m_usedData = userData;
}

/*!
  Get userData associated with this material.

  @param *newtonWorld Pointer to the Newton world.
  @param  id0 - group id0.
  @param  id1 - group id1.

  @return Nothing.
*/
void* NewtonMaterialGetUserData(const NewtonWorld* const newtonWorld, int id0, int id1)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//dgContactMaterial* const material = world->GetMaterial(dgUnsigned32(id0), dgUnsigned32(id1));
	//
	//return material->GetUserData();

	ndAssert(0);
	return 0;
}


NewtonCollision* NewtonMaterialGetBodyCollidingShape(const NewtonMaterial* const materialHandle, const NewtonBody* const bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndContactMaterial* const material = PointerCast<ndContactMaterial, NewtonMaterial>(materialHandle);
	ndBodyKinematic* const body = PointerCast<ndBodyKinematic, NewtonBody>(bodyPtr);

	const ndShapeInstance* const instance = (body == material->m_body0) ? material->m_shapeInstance0 : material->m_shapeInstance1;
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
}

dFloat NewtonMaterialGetContactMaxNormalImpact(const NewtonMaterial* const materialHandle)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndContactMaterial* const material = PointerCast<ndContactMaterial, NewtonMaterial>(materialHandle);
	return material->m_normal_Force.m_impact;
}

void NewtonMaterialSetContactGenerationCallback(const NewtonWorld* const newtonWorld, int id0, int id1, NewtonOnContactGeneration contactGeneration)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndNewtonMaterial* const material = PointerCast<ndNewtonMaterial, ndMaterial>(world->GetMaterial(id0, id1));
	ndAssert(material);
	material->m_onContactGeneration = contactGeneration;
}

#endif

/*!
  Get the value of the default MaterialGroupID.

  @param *newtonWorld pointer to the Newton world.

  @return The ID number for the default Group ID.

  Group IDs can be interpreted as the nodes of a dense graph. The edges of the graph are the physics materials.
  When the Newton world is created, the default Group ID is created by the engine.
  When bodies are created the application assigns a group ID to the body.
*/
int NewtonMaterialGetDefaultGroupID(const NewtonWorld* const)
{
	TRACE_FUNCTION(__FUNCTION__);
	return 0;
}

unsigned NewtonMaterialGetBodyCollisionID(const NewtonMaterial* material, const NewtonBody* body)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContactMaterial* const material = (dgContactMaterial*)materialHandle;
	//return material->GetUserData();
	ndAssert(0);
	return 0;
}

/*!
  Set the default coefficients of restitution (elasticity) for the material interaction between two physics materials .

  @param *newtonWorld pointer to the Newton world.
  @param  id0 - group id0
  @param  id1 - group id1
  @param elasticCoef static friction coefficients

  @return Nothing.

  *elasticCoef* must be a positive value.
  It is recommended that *elasticCoef* be set to a value lower or equal to 1.0
*/
void NewtonMaterialSetDefaultElasticity(const NewtonWorld* const newtonWorld, int id0, int id1, dFloat elasticCoef)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);

	ndMaterial* const material = world->GetMaterial(id0, id1);
	material->m_restitution = ndClamp(elasticCoef, ndFloat32(0.0f), ndFloat32(1.0f));
}

/*!
  Set the material interaction between two physics materials  to be collidable or non-collidable by default.

  @param *newtonWorld pointer to the Newton world.
  @param  id0 - group id0
  @param  id1 - group id1
  @param state state for this material: 1 = collidable; 0 = non collidable

  @return Nothing.
*/
void NewtonMaterialSetDefaultCollidable(const NewtonWorld* const newtonWorld, int id0, int id1, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndMaterial* const material = world->GetMaterial(id0, id1);
	if (state)
	{
		material->m_flags |= ndContactOptions::m_collisionEnable;
	}
	else
	{
		material->m_flags &= ~ndContactOptions::m_collisionEnable;
	}
}

/*!
  Set the default coefficients of friction for the material interaction between two physics materials .

  @param *newtonWorld pointer to the Newton world.
  @param  id0 - group id0
  @param  id1 - group id1
  @param staticFriction static friction coefficients
  @param kineticFriction dynamic coefficient of friction

  @return Nothing.

  *staticFriction* and *kineticFriction* must be positive values. *kineticFriction* must be lower than *staticFriction*.
  It is recommended that *staticFriction* and *kineticFriction* be set to a value lower or equal to 1.0, however because some synthetic materials
  can have higher than one coefficient of friction Newton allows for the coefficient of friction to be as high as 2.0.
*/
void NewtonMaterialSetDefaultFriction(const NewtonWorld* const newtonWorld, int id0, int id1, dFloat staticFriction, dFloat kineticFriction)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndMaterial* const material = world->GetMaterial(id0, id1);

	staticFriction = ndAbs(staticFriction);
	kineticFriction = ndAbs(kineticFriction);
	if (staticFriction >= ndFloat32(1.0e-2f))
	{
		ndFloat32 stat0 = ndClamp(staticFriction, ndFloat32(0.01f), ndFloat32(2.0f));
		ndFloat32 kine0 = ndClamp(kineticFriction, ndFloat32(0.01f), ndFloat32(2.0f));
		ndFloat32 stat = ndMax(stat0, kine0);
		ndFloat32 kine = ndMin(stat0, kine0);
		material->m_staticFriction0 = stat;
		material->m_staticFriction1 = stat;
		material->m_kineticFriction0 = kine;
		material->m_kineticFriction1 = kine;
		material->m_flags |= (ndContactOptions::m_friction0Enable | ndContactOptions::m_friction1Enable);
	}
	else
	{
		material->m_flags &= ~(ndContactOptions::m_friction0Enable | ndContactOptions::m_friction1Enable);
	}
}

/*!
  Set userData and the functions event handlers for the material interaction between two physics materials .

  @param *newtonWorld Pointer to the Newton world.
  @param  id0 - group id0.
  @param  id1 - group id1.
  @param *aabbOverlap address of the event function called when the AABB of tow bodyes overlap. This parameter can be NULL.
  @param *processCallback address of the event function called for every contact resulting from contact calculation. This parameter can be NULL.

  @return Nothing.

  When the AABB extend of the collision geometry of two bodies overlap, Newton collision system retrieves the material
  interaction that defines the behavior between the pair of bodies. The material interaction is collected from a database of materials,
  indexed by the material gruopID assigned to the bodies. If the material is tagged as non collidable,
  then no action is taken and the simulation continues.
  If the material is tagged as collidable, and a *aabbOverlap* was set for this material, then the *aabbOverlap* function is called.
  If the function  *aabbOverlap* returns 0, no further action is taken for this material (this can be use to ignore the interaction under
  certain conditions). If the function  *aabbOverlap* returns 1, Newton proceeds to calculate the array of contacts for the pair of
  colliding bodies. If the function *processCallback* was set, the application receives a callback for every contact found between the
  two colliding bodies. Here the application can perform fine grain control over the behavior of the collision system. For example,
  rejecting the contact, making the contact frictionless, applying special effects to the surface etc.
  After all contacts are processed and if the function *endCallback* was set, Newton calls *endCallback*.
  Here the application can collect information gathered during the contact-processing phase and provide some feedback to the player.
  A typical use for the material callback is to play sound effects. The application passes the address of structure in the *userData* along with
  three event function callbacks. When the function *aabbOverlap* is called by Newton, the application resets a variable say *maximumImpactSpeed*.
  Then for every call to the function *processCallback*, the application compares the impact speed for this contact with the value of
  *maximumImpactSpeed*, if the value is larger, then the application stores the new value along with the position, and any other quantity desired.
  When the application receives the call to *endCallback* the application plays a 3d sound based in the position and strength of the contact.
*/
//void NewtonMaterialSetCollisionCallback(const NewtonWorld* const newtonWorld, int id0, int id1, NewtonOnAABBOverlap aabbOverlap, NewtonContactsProcess processCallback)
NEWTON_API void NewtonMaterialSetCollisionCallback(const NewtonWorld* newtonWorld, int id0, int id1, void* userData, NewtonContactBegin begin, NewtonContactProcess process, NewtonContactEnd end)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndNewtonMaterial* const material = PointerCast<ndNewtonMaterial, ndMaterial>(world->GetMaterial(id0, id1));

	material->m_usedData = userData;
	material->m_onContactEnd = end;
	material->m_onAABBOverlap = begin;
	material->m_onContactsProcess = process;
}

/*!
  Create a new MaterialGroupID.

  @param *newtonWorld pointer to the Newton world.

  @return The ID of a new GroupID.

  Group IDs can be interpreted as the nodes of a dense graph. The edges of the graph are the physics materials.
  When the Newton world is created, the default Group ID is created by the engine.
  When bodies are created the application assigns a group ID to the body.

  Note: The only way to destroy a Group ID after its creation is by destroying all the bodies and calling the function  *NewtonMaterialDestroyAllGroupID*.

  See also: ::NewtonMaterialDestroyAllGroupID
*/
int NewtonMaterialCreateGroupID(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->m_bodyMaterialGroup++;
	return world->m_bodyMaterialGroup - 1;
}

// Name: NewtonMaterialDestroyAllGroupID 
// Remove all groups ID from the Newton world.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// 
// Return: Nothing.
//
// Remarks: This function removes all groups ID from the Newton world. 
// This function must be called after there are no more rigid bodies in the word.
//
// See also: NewtonDestroyAllBodies
void NewtonMaterialDestroyAllGroupID(const NewtonWorld* newtonWorld)
{
	ndAssert(0);
	//Newton* world;
	//
	////	_ASSERTE (0);
	//world = (Newton*)newtonWorld;
	//world->RemoveAllGroupID();
}


// Name: NewtonMaterialSetContactNormalDirection 
// Set the new direction of the for this contact point.
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair.
// *const dFloat* *direction  - pointer to an array of at least three floats holding the direction vector.
// 
// Return: Nothing.
//
// Remarks: This function can only be called from a material callback event handler.
// This function changes the basis of the contact point to one where the contact normal is alinend to the new direction vector
// and the tangent direction are recalculated to be perpendicular to teh new contact normal.
//
// Remarks: In 99.9% of the cases the collision system can calculates a very good contact normal.
// however this algorithm that calculate the contact normal use as criteria the normal direction
// that will resolve the interpenetration with the least amount on motion. 
// There are situations however when this solution is not the best. Take for example a rolling
// ball over a texelated floor, when the ball is over a flat polygon, the contact normal is always
// perpendicular to the floor and pass by the origin of the sphere, however when the sphere is going 
// across two adjacent polygons, the contact normal is now perpendicular to the polygons edge and this does 
// not guarantee they it will pass bay the origin of the sphere, but we know that the best normal is always 
// the one passing by the origin of the sphere.
// 
// See also: NewtonMaterialSetCollisionCallback, NewtonMaterialContactRotateTangentDirections
void NewtonMaterialSetContactNormalDirection(const NewtonMaterial* materialHandle, const dFloat* direction)
{
	//dgContactMaterial* material;
	//material = (dgContactMaterial*)materialHandle;
	//dgVector normal(direction[0], direction[1], direction[2]);
	//
	//_ASSERTE((dgAbsf(normal % material->m_normal) - dgFloat32(1.0f)) < dgFloat32(0.01f));
	//if ((normal % material->m_normal) < dgFloat32(0.0f)) {
	//	normal = normal.Scale(-dgFloat32(1.0f));
	//}
	////	material->m_normal = normal;
	////	NewtonMaterialContactRotateTangentDirections(materialHandle, &material->m_dir0[0]);
	TRACE_FUNCTION(__FUNCTION__);
	const ndContactMaterial* const material = PointerCast<ndContactMaterial, NewtonMaterial>(materialHandle);
	ndAssert(0);
}

// Name: NewtonMaterialContactRotateTangentDirections 
// Rotate the tangent direction of the contacts until the primary direction is aligned with the alignVector.
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair.
// *const dFloat* *alignVector  - pointer to an array of at least three floats holding the aligning vector.
// 
// Return: Nothing.
//
// Remarks: This function can only be called from a material callback event handler.
// This function rotates the tangent vectors of the contact point until the primary tangent vector and the align vector
// are perpendicular (ex. when the dot product between the primary tangent vector and the alignVector is 1.0). This 
// function can be used in conjunction with NewtonMaterialSetContactTangentAcceleration in order to
// create special effects. For example, conveyor belts, cheap low lod vehicles, slippery surfaces, etc.
// 
// See also: NewtonMaterialSetCollisionCallback, NewtonMaterialSetContactNormalDirection
void NewtonMaterialContactRotateTangentDirections(const NewtonMaterial* materialHandle, const dFloat* alignVector)
{
	//dFloat mag2;
	//dgContactMaterial* material;
	//
	//material = (dgContactMaterial*)materialHandle;
	//
	//const dgVector dir0(alignVector[0], alignVector[1], alignVector[2]);
	//
	//dgVector dir1 = material->m_normal * dir0;
	//mag2 = dir1 % dir1;
	//if (mag2 > 1.0e-6f) {
	//	material->m_dir1 = dir1.Scale(dgRsqrt(mag2));
	//	material->m_dir0 = material->m_dir1 * material->m_normal;
	//}
	TRACE_FUNCTION(__FUNCTION__);
	const ndContactMaterial* const material = PointerCast<ndContactMaterial, NewtonMaterial>(materialHandle);
	ndAssert(0);
}

/*!
  Get the userData set by the application when it created this material pair.

  @param materialHandle pointer to a material pair

  @return Application user data.

  This function can only be called from a material callback event handler.

  See also: ::NewtonMaterialSetCollisionCallback
*/
void* NewtonMaterialGetMaterialPairUserData(const NewtonMaterial* const materialHandle)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContactMaterial* const material = (dgContactMaterial*)materialHandle;
	//return material->GetUserData();
	ndNewtonMaterial* const material = PointerCast<ndNewtonMaterial, NewtonMaterial>(materialHandle);
	return *material->m_usedData;
}


/*!
  Get the contact position and normal in global space.

  @param materialHandle pointer to a material pair.
  @param *body pointer to body
  @param *positPtr pointer to an array of at least three floats to hold the contact position.
  @param *normalPtr pointer to an array of at least three floats to hold the contact normal.

  @return Nothing.

  This function can only be called from a material callback event handle.

  See also: ::NewtonMaterialSetCollisionCallback
*/
void NewtonMaterialGetContactPositionAndNormal(const NewtonMaterial* materialHandle, dFloat* positPtr, dFloat* normalPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	const ndContactMaterial* const material = PointerCast<ndContactMaterial, NewtonMaterial>(materialHandle);

	positPtr[0] = material->m_point.m_x;
	positPtr[1] = material->m_point.m_y;
	positPtr[2] = material->m_point.m_z;

	normalPtr[0] = material->m_normal.m_x;
	normalPtr[1] = material->m_normal.m_y;
	normalPtr[2] = material->m_normal.m_z;

	//if (PointerCast<ndBody, NewtonBody>(body) != material->m_body0)
	//{
	//	normalPtr[0] *= ndFloat32(-1.0f);
	//	normalPtr[1] *= ndFloat32(-1.0f);
	//	normalPtr[2] *= ndFloat32(-1.0f);
	//}
}

/*!
  Calculate the speed of this contact along the normal vector of the contact.

  @param materialHandle pointer to a material pair

  @return Contact speed. A positive value means the contact is repulsive.

  This function can only be called from a material callback event handler.

  See also: ::NewtonMaterialSetCollisionCallback
*/
dFloat NewtonMaterialGetContactNormalSpeed(const NewtonMaterial* materialHandle, const NewtonContact* contactlHandle)
{
	TRACE_FUNCTION(__FUNCTION__);
	const ndContactMaterial* const contactPoint = PointerCast<ndContactMaterial, NewtonContact>(contactlHandle);
	const ndBodyKinematic* const body0 = contactPoint->m_body0;
	const ndBodyKinematic* const body1 = contactPoint->m_body1;

	const ndVector v0(body0->GetVelocityAtPoint(contactPoint->m_point));
	const ndVector v1(body1->GetVelocityAtPoint(contactPoint->m_point));
	const ndVector dv(v1 - v0);

	ndAssert(contactPoint->m_normal.m_w == ndFloat32(0.0f));
	dFloat speed = dv.DotProduct(contactPoint->m_normal).GetScalar();
	return speed;
}

/*!
  Calculate the speed of this contact along the tangent vector of the contact.

  @param materialHandle pointer to a material pair.
  @param index index to the tangent vector. This value can be 0 for primary tangent direction or 1 for the secondary tangent direction.

  @return Contact tangent speed.

  This function can only be called from a material callback event handler.

  See also: ::NewtonMaterialSetCollisionCallback
*/
dFloat NewtonMaterialGetContactTangentSpeed(const NewtonMaterial* materialHandle, const NewtonContact* contactlHandle, int index)
{
	TRACE_FUNCTION(__FUNCTION__);
	const ndContactMaterial* const material = PointerCast<ndContactMaterial, NewtonMaterial>(materialHandle);
	const ndBodyKinematic* const body0 = material->m_body0;
	const ndBodyKinematic* const body1 = material->m_body1;

	const ndVector v0(body0->GetVelocityAtPoint(material->m_point));
	const ndVector v1(body1->GetVelocityAtPoint(material->m_point));
	const ndVector dv(v1 - v0);

	ndVector dir(index ? material->m_dir1 : material->m_dir0);
	ndAssert(dir.m_w == ndFloat32(0.0f));
	dFloat speed = dv.DotProduct(dir).GetScalar();
	return -speed;
}

// Name: NewtonMaterialSetContactElasticity 
// Override the default elasticity (coefficient of restitution) value for the contact.
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair.
// *dFloat* restitution  - elasticity value, must be positive.
// 
// Return: Nothing.
//
// Remarks: This function can only be called from a material callback event handler.
// 
// See also: NewtonMaterialSetCollisionCallback, NewtonMaterialSetDefaultElasticity
void NewtonMaterialSetContactElasticity(const NewtonMaterial* materialHandle, dFloat restitution)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContactMaterial* material;
	//material = (dgContactMaterial*)materialHandle;
	//material->m_restitution = ClampValue(restitution, dFloat(0.01f), dFloat(2.0f));
	ndContactMaterial* const material = PointerCast<ndContactMaterial, NewtonMaterial>(materialHandle);
	material->m_material.m_restitution = ndClamp(restitution, dFloat(0.0f), dFloat(1.0f));
}

// Name: NewtonMaterialSetContactFrictionState 
// Enable or disable friction calculation for this contact.
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair.
// *int* state* - new state. 0 makes the contact frictionless along the index tangent vector.
// *int* index - index to the tangent vector. 0 for primary tangent vector or 1 for the secondary tangent vector.
// 
// Return: Nothing.
//
// Remarks: This function can only be called from a material callback event handler.
// 
// See also: NewtonMaterialSetCollisionCallback
void NewtonMaterialSetContactFrictionState(const NewtonMaterial* materialHandle, int state, int index)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//dgContactMaterial* material;
	//
	//material = (dgContactMaterial*)materialHandle;
	//
	//if (index) {
	//	material->m_friction1Enable = state ? true : false;
	//}
	//else {
	//	material->m_friction0Enable = state ? true : false;
	//}
}

// Name: NewtonMaterialSetContactStaticFrictionCoef
// Override the default value of the static coefficient of friction for this contact.
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair.
// *dFloat* coef - static friction coeficient. Must be positive.
// *int* index - index to the tangent vector. 0 for primary tangent vector or 1 for the secondary tangent vector.
// 
// Return: nothing
//
// Remarks: this function can only be called from a material callback event handler.
//
// Remarks: It is recommended that *coef* be set to a value lower or equal to 1.0, however because some synthetic materials
// can have hight than one coeficient of friction Newton allows for the coeficient of friction to be as high as 2.0.
//
// Remarks: the value *coef* will be clamped between the current kinetic friction and 2.0 (the maximum friction allowed).
// If the application wants to set a static friction lower than the current kinetic friction it must decrease the kinetic friction first.
// 
// See also: NewtonMaterialSetCollisionCallback, NewtonMaterialSetDefaultFriction, NewtonMaterialSetContactKineticFrictionCoef
void NewtonMaterialSetContactStaticFrictionCoef(const NewtonMaterial* materialHandle, dFloat coef, int index)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//dgContactMaterial* material;
	//
	//material = (dgContactMaterial*)materialHandle;
	//
	//if (index) {
	//	material->m_staticFriction1 = ClampValue(coef, material->m_dynamicFriction1, dFloat(2.0f));
	//}
	//else {
	//	material->m_staticFriction0 = ClampValue(coef, material->m_dynamicFriction0, dFloat(2.0f));
	//}
}

// Name: NewtonMaterialSetContactKineticFrictionCoef
// Override the default value of the kinetic coefficient of friction for this contact.
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair.
// *dFloat* coef - static friction coefficient. Must be positive.
// *int* index - index to the tangent vector. 0 for primary tangent vector or 1 for the secondary tangent vector.
// 
// Return: Nothing.
//
// Remarks: This function can only be called from a material callback event handler.
//
// Remarks: It is recommended that *coef* be set to a value lower or equal to 1.0, however because some synthetic materials
// can have hight than one coeficient of friction Newton allows for the coeficient of friction to be as high as 2.0.
//
// Remarks: the value *coef* will be clamped between 0.01f (minimum friction value) and that of the current static friction.
// If the application wants to set a kinetic friction higher than the current static friction it must increase the static friction first.
// 
// See also: NewtonMaterialSetCollisionCallback, NewtonMaterialSetDefaultFriction, NewtonMaterialSetContactStaticFrictionCoef
void NewtonMaterialSetContactKineticFrictionCoef(const NewtonMaterial* materialHandle, dFloat coef, int index)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//dgContactMaterial* material;
	//
	//material = (dgContactMaterial*)materialHandle;
	//
	//if (index) {
	//	material->m_dynamicFriction1 = ClampValue(coef, dFloat(0.01f), material->m_staticFriction1);
	//}
	//else {
	//	material->m_dynamicFriction0 = ClampValue(coef, dFloat(0.01f), material->m_staticFriction0);
	//}
}
