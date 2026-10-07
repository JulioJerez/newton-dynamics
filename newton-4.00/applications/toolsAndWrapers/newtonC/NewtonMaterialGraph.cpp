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
	ndNewtonWorld* const world = ObjectFromHandle<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->m_bodyMaterialGroup++;
	return world->m_bodyMaterialGroup - 1;
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
	ndNewtonWorld* const world = ObjectFromHandle<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->ClearMaterials();
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
	ndNewtonWorld* const world = ObjectFromHandle<ndNewtonWorld, NewtonWorld>(newtonWorld);

	ndMaterial* const material = world->GetMaterial(id0, id1);
	material->m_restitution = ndClamp(elasticCoef, ndFloat32(0.0f), ndFloat32(1.0f));
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
	ndNewtonWorld* const world = ObjectFromHandle<ndNewtonWorld, NewtonWorld>(newtonWorld);
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
void NewtonMaterialSetCollisionCallback(const NewtonWorld* const newtonWorld, int id0, int id1, NewtonOnAABBOverlap aabbOverlap, NewtonContactsProcess processCallback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = ObjectFromHandle<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndNewtonMaterial* const material = static_cast<ndNewtonMaterial*>(world->GetMaterial(id0, id1));

	material->m_onAABBOverlap = aabbOverlap;
	material->m_onContactsProcess = processCallback;
}
