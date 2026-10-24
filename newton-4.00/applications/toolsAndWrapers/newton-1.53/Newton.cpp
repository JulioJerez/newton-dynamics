// Name: Newton Game Dynamics 1.5
// Advanced physics engine for real-time simulation of rigid bodies.
// Created by Julio Jerez 2003 - 2005. All rights reserved. 

//#include "stdafx.h"
#include "newtonStdafx.h"
#include "Newton.h"
//#include "NewtonClass.h"
//#include "NewtonRagdoll.h"

//#define DG_USED_DEBUG_EXCEPTIONS


#define DG_CRC_CHECK dgUnsigned32(~0x39540166)
static char* name = "Newton Game Dynamics is the property of Julio Jerez. You need explicit permission from Julio Jerez for distribution of this product";

/*
// *dFloat* globalScale - global scale factor that will scale all internal tolerance.
// Remarks: the physics system in theory should be dimensionless, however in practice the engine have to be implemented with
// limited precision floating numbers and is also built for real-time simulation, it is inevitable that tolerances have to be used in order to increase performance, and 
// reduce instabilities. These tolerances make the engine dimension dependent, for example let say a rigid body is considered at rest when 
// its velocity is less than 0.01 units per second for some frames. For a program using meters as unit this translate to 0.01 meters per second, 
// however for a program using centimeter this translate to 0.0001 meter per second, since the precession of speed is independent of the unit system, 
// this means that in the second system the engine has to work much harder to bring the body to rest. A solution for this is to scale all tolerances 
// to match the unit system. The argument *globalScale* must be a constant to convert the unit system in the game to meters, for example if in your game you are using 39 units is a meter, 
// the *globaScale* must be 39. The exact conversion factor does not have to be exact, but the closer it is to reality the better performance the application will get.
// Applications that are already using meters as the unit system must pass 1.0 as *globalSscale*.
*/

#if 0
// ***************************************************************************************************************
//
// Name: World interface 
//
// ***************************************************************************************************************



// Name: NewtonGetGlobalScale
// Get the global scale factor.
// 
// Remarks: the physics system in theory should be dimensionless, however in practice the engine have to be implemented with
// limited precision floating numbers and is also built for real-time simulation, it is inevitable that tolerances have to be used in order to increase performance, and 
// reduce instabilities. These tolerances make the engine dimension dependent, for example let say a rigid body is considered at rest when 
// its velocity is less than 0.01 units per second for some frames. For a program using meters as unit this translate to 0.01 meters per second, 
// however for a program using centimeter this translate to 0.0001 meter per second, since the precession of speed is independent of the unit system, 
// this means that in the second system the engine has to work much harder to bring the body to rest. A solution for this is to scale all tolerances 
// to match the unit system. The argument *globalScale* must be a constant to convert the unit system in the game to meters, for example if in your game you are using 39 units is a meter, 
// the *globaScale* must be 39. The exact conversion factor does not have to be exact, but the closer it is to reality the better performance the application will get.
// Applications that are already using meters as the unit system must pass 1.0 as *globalScale*.
//
//See also: NewtonCreate
//dFloat NewtonGetGlobalScale(const NewtonWorld* newtonWorld)
//{
//	Newton *world;
//	world = (Newton *) newtonWorld;
//	return world->GetGlobalScale();
//	return dgFloat32(1.0f);
//}


// Name: NewtonSetPlatformArchitecture 
// Set the current platform hardware architecture.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *int* model - model of operation 0 = default, 1 = medium, n = best.
// 
// Return: Nothing
//
// Remarks: This function allows the application to configure the Newton to take advantage  
// for specific hardware architecture in the same platform.
//
// 0: force the hardware lower common denominator for the running platform.
//
// 1: will try to use common floating point enhancement like spacial instruction set 
// on the specific architecture. This mode made lead to result that differ from mode 1 and 2 as the accumulation 
// round off errors maybe different. 
//
// 2: the engine will try to use the best possible hardware setting found in the current platform 
// this is the default configuration. This mode made lead to result that differ from mode 1 and 2 as the accumulation 
// round off errors maybe different. 
//
// Remarks: the only hardware mode guarantee to work is mode 0. all other are only 
// hints to the engine, for example setting mode 1 will take not effect on CPUs without  
// specially floating point instructions set. 
void NewtonSetPlatformArchitecture(const NewtonWorld* newtonWorld, int mode)
{
	Newton *world;
	world = (Newton *)newtonWorld;

	world->SetHardwareMode(mode);
}


// Name: NewtonSetSolverModel 
// Set the solver precision mode.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *int* model - model of operation 0 = exact, 1 = adaptive, n = linear. The default is exact.
// 
// Return: Nothing
//
// Remarks: This function allows the application to configure the Newton solver to work in three different modes.  
//
// 0: Is the exact mode. This is good for application where precision is more important than speed, ex: realistic simulation.
//
// 1: Is the adaptive mode, the solver is not as exact but the simulation will still maintain a high degree of accuracy. 
// This mode is good for applications were a good degree of stability is important but not as important as speed. 
//
// n: Linear mode. The solver will not try to reduce the joints relative acceleration errors to below some limit, 
// instead it will perform up to n passes over the joint configuration each time reducing the acceleration error, 
// but it will terminate when the number of passes is exhausted regardless of the error magnitude. 
// In general this is the fastest mode and is is good for applications where speed is the only important factor, ex: video games.
//
// Remarks: the adaptive friction model combined with the linear model make for the fastest possible configuration 
// of the Newton solver. This setup is best for games.
// If you need the best realistic behavior, we recommend the use of the exact solver and exact friction model which are the defaults.
//
// See also: NewtonSetFrictionModel
void NewtonSetSolverModel(const NewtonWorld* newtonWorld, int model)
{
	Newton *world;
	world = (Newton *)newtonWorld;

	world->SetSolverMode (model);
}

// Name: NewtonSetFrictionModel 
// Set coulomb model of friction.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *int* model - friction model;  0 = exact coulomb, 1 = adaptive coulomb, The default is exact.
// 
// Return: Nothing.
//
// Remarks: This function allows the application to chose between and exact or an adaptive coulomb friction model 
// 
// 0: Is the exact model. Friction forces are calculated in each frame. 
// This model is good for applications where precision is more important than speed, ex: realistic simulation.
//
// 1: Is the adaptive model. Here values from previous frames are used to determine the maximum friction values of the current frame. 
// This is about 10% faster than the exact model however it may introduce strange friction behaviors. For example a 
// bouncing object tumbling down a ramp will act as a friction less object because the contacts do not have continuity. 
// In general each time a new contact is generated the friction value is zero, only if the contact persist a non zero 
// friction values is used. The second effect is that if a normal force is very strong, and if the contact is suddenly 
// destroyed, a very strong friction force will be generated at the contact point making the object react in a non-familiar way. 
//
// Remarks: the adaptive friction model combined with the linear model make for the fastest possible configuration 
// of the Newton solver. This setup is best for games.
// If you need the best realistic behavior, we recommend the use of the exact solver and exact friction model which are the defaults.
//
// See also: NewtonSetSolverModel
void NewtonSetFrictionModel(const NewtonWorld* newtonWorld, int model)
{
	Newton *world;
	world = (Newton *)newtonWorld;

	world->SetFrictionMode (model);
}


#ifdef DG_USED_DEBUG_EXCEPTIONS
dgInt32 ExecptionHandler (void *exceptPtr)
{
	EXCEPTION_RECORD *record;

	record = ((EXCEPTION_POINTERS *)exceptPtr)->ExceptionRecord;
//	if (record->ExceptionCode != EXCEPTION_ACCESS_VIOLATION) {
//		return 0;
//	}
	return 1;
}
#endif

// Name: NewtonSetMinimumFrameRate 
// Set the minimum frame rate at which the simulation can run.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *dFloat* frameRate - minimum frame rate of the simulation in frame per second. This value is clamped between 60fps and 1000fps
// 
// Return: nothing
//
// Remarks: the default minimum frame rate of the simulation is 60 frame per second. When the simulation falls below the specified minimum frame, Newton will
// perform sub steps in order to meet the desired minimum FPS. 
void NewtonSetMinimumFrameRate(const NewtonWorld* newtonWorld, dFloat frameRate)
{
	Newton *world;

	world = (Newton *) newtonWorld;

	_ASSERTE (dgFloat32(1.0f) / dgFloat32 (MAX_TIMESTEP) < dgFloat32(1.0f) / dgFloat32 (MIN_TIMESTEP));
	frameRate = ClampValue (frameRate, dgFloat32(1.0f) / dgFloat32 (MAX_TIMESTEP), dgFloat32(1.0f) / dgFloat32 (MIN_TIMESTEP));  
	world->g_maxTimeStep = dgFloat32(1.0f) / frameRate;
}

// Name: NewtonGetTimeStep 
// Return the correct time step for this simulation update.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
//
// Remark: This application can used this function to get the current simulation time step. 
// 
// Return: correct update timestep.
dFloat NewtonGetTimeStep(const NewtonWorld* newtonWorld)
{
	Newton *world;
	world = (Newton *)newtonWorld;
	return world->GetTimeStep();
}


// Name: NewtonDestroyAllBodies 
// Remove all bodies and joints from the newton world.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is a pointer to the Newton world
// 
// Return: Nothing
//
// Remarks: This function will destroy all bodies and all joints in the Newton world, but it will retain group IDs. 
// Use this function for when you want to clear the world but preserve all the group IDs and material pairs.
//
// See also: NewtonMaterialDestroyAllGroupID
void NewtonDestroyAllBodies(const NewtonWorld* newtonWorld)
{
	dFloat p0[4];
	dFloat p1[4];
	Newton *world;

	p0[0] = -100.0f;
	p0[1] = -100.0f;
	p0[2] = -100.0f;
	p1[0] = 100.0f;
	p1[1] = 100.0f;
	p1[2] = 100.0f;

	NewtonSetWorldSize (newtonWorld, p0, p1);

	world = (Newton *) newtonWorld;

	world->RagDollList::DestroyAll();
	world->DestroyAllBodies ();
}

// Name: NewtonSetWorldSize 
// Set the size of the Newton world.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *const dFloat* *minPtr - is the minimum point of the world bounding box 
// *const dFloat* *maxPtr - is the maximum point of the world bounding box 
// 
// Return: Nothing.
//
// Remarks: The Newton world must have a finite size. The size of the world is set to a box +- 100 units in all three dimensions
// at creation time and after a call to the function _NewtonRemoveAllBodies_
//
// See also: NewtonSetBodyLeaveWorldEvent
void NewtonSetWorldSize(const NewtonWorld* newtonWorld, const dFloat* minPtr, const dFloat* maxPtr)
{
	Newton *world;

	world = (Newton *) newtonWorld;
	dgVector p0 (minPtr[0], minPtr[1], minPtr[2], dgFloat32(1.0f)); 
	dgVector p1 (maxPtr[0], maxPtr[1], maxPtr[2], dgFloat32(1.0f)); 
	world->SetWorldSize(p0, p1); 
}


// Name: NewtonSetBodyLeaveWorldEvent 
// Set the event callback function to be called in the event a body is escaping the limits of the world
// during simulation time.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *NewtonBodyLeaveWorld* callback - is the pointer to the function callback
// 
// Return: Nothing
//
// Remarks: When a body moves outside the bounding box that defines the world space the body is automatically disabled
// and Newton calls the application defined callback function *NewtonBodyLeaveWorld callback*. 
// The application should decide how to handle the event, because Newton will make the callback once.
// The only options available to the application are: do nothing or destroy the object. 
//
// Remarks: if the application wants to reinitialize the body, it will have to used the function 
// NewtonBodySetMatrixRecursive, this is because it is possible that the rigid body is part of a set of 
// articulate bodies and only one part leave the world while the others are still inside the world volume,
// and in and situation like this the function NewtonSetMatrix will lead to unpredictable results.
//
// See also: NewtonSetWorldSize, NewtonBodySetMatrixRecursive
void NewtonSetBodyLeaveWorldEvent(const NewtonWorld* newtonWorld, NewtonBodyLeaveWorld callback) 
{
	Newton *world;
	world = (Newton *) newtonWorld;
	world->SetLeavingWorldCallback ((OnLeavingWorldAction) callback);		
}


// Name: NewtonWorldFreezeBody 
// Remove the body from the active simulation list.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *const NewtonBody* *body - is the pointer to the body to be frozen
// 
// Return: Nothing.
//
// Remarks: When a body is created it is automatically placed in the active simulation list. As an optimization 
// for large scenes, you may use this function to put background bodies in an inactive equilibrium state.
//
// Remarks: This function tells Newton that this body does not currently need to be simulated.
// However, if the body is part of a larger configuration it may be affected indirectly by the reaction forces
// of objects that it is connected to.
//
// See also: NewtonWorldUnfreezeBody, NewtonBodySetAutoFreeze, NewtonBodyGetAutoFreeze
void NewtonWorldFreezeBody(const NewtonWorld* newtonWorld, const NewtonBody* body)
{
	Newton *world;
	world = (Newton *) newtonWorld;
	world->FreezeBody ((dgBody*) body);		
}


// Name: NewtonWorldUnfreezeBody 
// Add the body to the active simulation list.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *const NewtonBody* *body - is the pointer to the body to be activated
// 
// Return: Nothing
//
// Remarks: This function tells Newton to simulate the rigid body *body*.
//
// See also: NewtonWorldFreezeBody, NewtonBodySetAutoFreeze, NewtonBodyGetAutoFreeze
void NewtonWorldUnfreezeBody(const NewtonWorld* newtonWorld, const NewtonBody* body)
{
	Newton *world;
	world = (Newton *) newtonWorld;
	world->UnfreezeBody ((dgBody*) body);		
}

// Name: NewtonWorldForEachBodyDo 
// Iterate thought every body in the world calling the function callback.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *NewtonBodyIterator* callback - application define callback 
//
// Return: nothing
// 
// Remarks: The application can call this function to iterate thought every body in the world. 
// the application should provide the function *NewtonBodyIterator callback* to be called by Newton for every body in the world 
//
// Remarks: this function affect severally the performance of Newton. The application should call this function only for debugging purpose
// See also: NewtonBodyForEachPolygonDo, NewtonWorldForEachBodyInAABBDo
void NewtonWorldForEachBodyDo(const NewtonWorld* newtonWorld, NewtonBodyIterator callback)
{
	Newton *world;
	dgBody *body;

	world = (Newton *) newtonWorld;

	dgBodyMasterList &masterList = *world;
	dgBodyMasterList::Iterator iter0 (masterList);
	for (iter0.Begin(); iter0; iter0 ++) {
		body = *iter0;
		callback ((NewtonBody*) body);
	}
}



// Name: NewtonWorldForEachBodyInAABBDo 
// Iterate thought every body in the world that intersect the AABB calling the function callback.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const dFloat* *p0 - pointer to an array of at least three floats to hold minimum value for the AABB.
// *const dFloat* *p1 - pointer to an array of at least three floats to hold maximum value for the AABB.
// *NewtonBodyIterator* callback - application define callback 
//
// Return: nothing
// 
// Remarks: The application can call this function to iterate thought every body in the world. 
// the application should provide the function *NewtonBodyIterator callback* to be called by Newton for every body in the world 
//
// Remarks: For relatively small AABB volumes this function is much more inefficients 
// that NewtonWorldForEachBodyDo, however in case where the AABB contain must of the objects in the scene, 
// the overhead of scanning the internal Broad face collision plus the AABB test make this function more expensive.
//
// See also: NewtonBodyForEachPolygonDo, NewtonWorldForEachBodyDo
void NewtonWorldForEachBodyInAABBDo(const NewtonWorld* newtonWorld, const dFloat* p0, const dFloat* p1, NewtonBodyIterator callback)
{
	Newton *world;

	world = (Newton *) newtonWorld;
	dgVector q0 (p0[0], p0[1], p0[2], dgFloat32 (0.0f));
	dgVector q1 (p1[0], p1[1], p1[2], dgFloat32 (0.0f));
	world->ForEachBodyInAABB (q0, q1, (OnLeavingWorldAction) callback);
}


// Name: NewtonGetVersion 
// Return the current library version number.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the newton world.
//
// Return: release decimal three digit value x.xx
// the first digit:  is mayor version number (interface changes among other things)   
// the second digit: is mayor patch number (new features, and bug fixes) 
// third digit: is minor bug fixed patch.
int NewtonWorldGetVersion(const NewtonWorld* newtonWorld)
{
	return DG_CURRENT_VERSION;
}


// Name: NewtonWorldSetUserData 
// Store a user defined data value with the world.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the newton world.
// *void* *userDataPtr - pointer to the user defined user data value.
//
// Return: Nothing.
//
// Remarks: The application can store a user defined value with the Newton world. The user data is useful for application developing 
// object oriented classes based on the Newton API.
// 
// See also: NewtonWorldGetUserData 
void NewtonWorldSetUserData(const NewtonWorld* newtonWorld, void* userData)
{
	Newton *world;
	world = (Newton *) newtonWorld;

	world->SetUserData (userData);
}

// Name: NewtonWorldGetUserData 
// Retrieve a user previously stored user define value with the world.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
//
// Return: user data value.
//
// Remarks: The application can store a user defined value with the Newton world. The user data is useful for application developing 
// object oriented classes based on the Newton API.
// 
// See also: NewtonWorldGetUserData  
void* NewtonWorldGetUserData(const NewtonWorld* newtonWorld)
{
	Newton *world;
	world = (Newton *) newtonWorld;

	return world->GetUserData();
}


// Name: NewtonWorldRayCast 
// Shoot a ray from p0 to p1 and call the application callback with each ray intersection.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the world.
// *const dFloat* *p0 - pointer to an array of at least three floats containing the beginning of the ray in global space.
// *const dFloat* *p1 - pointer to an array of at least three floats containing the end of the ray in global space.
// *NewtonWorldRayFilterCallback* filter - user define function to be called for each body hit during the ray scan.
// *void* *userData - user data to be passed to the filter callback.
// *NewtonWorldRayPrefilterCallback* prefilter - user define function to be called for each body before intesection.
//
// Return: nothing
// 
// Remarks: The ray cast function will call the application with each body intersecting the line segment. 
// By writing the callback filter function in different ways the application can implement different flavors of ray casting. 
// For example an all body ray cast can be easily implemented by having the filter function always returning 1.0, and copying each 
// rigid body into an array of pointers; a closest hit ray cast can be implemented by saving the body with the smaller intersection 
// parameter and returning the parameter t; and a report the first body hit can be implemented by having the filter function returning 
// zero after the first call and saving the pointer to the rigid body.
//
// Remarks: The most common use for the ray cast function is the closest body hit, In this case it is important, for performance reasons, 
// that the filter function returns the intersection parameter. If the filter function returns a value of zero the ray cast will terminate 
// immediately.
//
// Remarks: if prefilter is not NULL, Newton will call the application right before executing the intersections between the ray and the primitive.
// if the function returns zero the Newton will not raycast the primitive. passing a NULL pointer will raycast the. 
// The application can use this implement faster or smarter filters when implementing complex logic, otherwise for normal all ray cast
// this parameter could be NULL.
//
// Remarks: The ray cast function is provided as an utility function, this means that even thought the function is very high performance 
// by function standards, it can not by batched and therefore it can not be an incremental function. For example the cost of calling 1000 
// ray cast is 1000 times the cost of calling one ray cast. This is much different than the collision system where the cost of calculating 
// collision for 1000 pairs in much, much less that the 1000 times the cost of one pair. Therefore this function must be used with care, 
// as excessive use of it can degrade performance.
void NewtonWorldRayCast(const NewtonWorld* newtonWorld, const dFloat* p0, const dFloat* p1, NewtonWorldRayFilterCallback filter, void* userData, NewtonWorldRayPrefilterCallback prefilter)
{
	Newton *world;
	if (filter) {
		dgVector pp0 (p0[0], p0[1], p0[2]);
		dgVector pp1 (p1[0], p1[1], p1[2]);
		world = (Newton *) newtonWorld;
		world->RayCast (pp0, pp1, (OnRayCastAction) filter, (OnRayPrecastAction) prefilter, userData);
	}
}



// ***************************************************************************************************************
//
// Name: GroupID interface
//
// ***************************************************************************************************************


// Name: NewtonMaterialGetDefaultGroupID 
// Get the value of the default MaterialGroupID.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// 
// Return: The ID number for the default Group ID.
//
// Remarks: Group IDs can be interpreted as the nodes of a dense graph. The edges of the graph are the physics materials.
// When the Newton world is created, the default Group ID is created by the engine.	
// When bodies are created the application assigns a group ID to the body. 
int NewtonMaterialGetDefaultGroupID(const NewtonWorld* newtonWorld)
{
	Newton *world;

	world = (Newton *)newtonWorld;
	return world->GetDefualtBodyGroupID();
}


// Name: NewtonMaterialCreateGroupID 
// Create a new MaterialGroupID.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// 
// Return: The ID of a new GroupID.
//
// Remarks: Group IDs can be interpreted as the nodes of a dense graph. The edges of the graph are the physics materials.
// When the Newton world is created, the default Group ID is created by the engine.	
// When bodies are created the application assigns a group ID to the body. 
//
// Note: The only way to destroy a Group ID after its creation is by destroying all the bodies and calling the function  *NewtonMaterialDestroyAllGroupID*.
//
// See also: NewtonMaterialDestroyAllGroupID
int NewtonMaterialCreateGroupID(const NewtonWorld* newtonWorld)
{
	Newton *world;

	world = (Newton *)newtonWorld;
	return world->CreateBodyGroupID();
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
	Newton *world;

//	_ASSERTE (0);
	world = (Newton *)newtonWorld;
	world->	RemoveAllGroupID();
}


//int NewtonGetBodiesCount()
//{
//	_ASSERTE (0);
//	return 0;
//}


//int NewtonGetActiveBodiesCount()
//{
//	_ASSERTE (0);
//	return 0;
//}

//int NewtonGetActiveConstraintsCount()
//{
//	_ASSERTE (0);
//	return 0;
//}


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
	Newton *world;
	dgContactMaterial* material;

	world = (Newton *)newtonWorld;
	material = world->GetMaterial (id0, id1);

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
	Newton *world;
	dgContactMaterial* material;

	world = (Newton *)newtonWorld;
	material = world->GetMaterial (id0, id1);

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
	Newton *world;
	dgContactMaterial* material;

	world = (Newton *)newtonWorld;
	material = world->GetMaterial (id0, id1);

	staticFriction = dgAbsf (staticFriction);
	kineticFriction = dgAbsf (kineticFriction);

	if (material) {
		if (staticFriction >= 1.e-2f) {
			stat = ClampValue (staticFriction, dFloat(0.01f), dFloat(2.0f));
			kine = ClampValue (kineticFriction, dFloat(0.01f), dFloat(2.0f));
			stat = GetMax (stat, kine);
			material->m_staticFriction0 = stat;
			material->m_staticFriction1 = stat;
			material->m_dynamicFriction0 = kine;
			material->m_dynamicFriction1 = kine;
		} else {
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
	Newton *world;
	dgContactMaterial* material;

	world = (Newton *)newtonWorld;
	material = world->GetMaterial (id0, id1);

	material->m_restitution = ClampValue (elasticCoef, dFloat(0.01f), dFloat(2.0f));
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
	Newton *world;
	dgContactMaterial* material;

	world = (Newton *)newtonWorld;
	material = world->GetMaterial (id0, id1);

	material->m_softness = ClampValue (softnessCoef, dFloat(0.01f), dFloat(dgFloat32(1.0f)));
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
void NewtonMaterialSetCollisionCallback(const NewtonWorld* newtonWorld, 
	int id0, 
	int id1, 
	void *userData,
	NewtonContactBegin beginCallback, 
	NewtonContactProcess processCallback, 
	NewtonContactEnd endCallback)
{
	Newton *world;
	dgContactMaterial* material;

	world = (Newton *)newtonWorld;
	material = world->GetMaterial (id0, id1);

	material->m_userData = userData;
	material->SetCollisionCallback ((OnContactCallback) processCallback, (OnContactBegin) beginCallback, (OnContactEnd) endCallback);
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
	Newton *world;
	dgContactMaterial* material;

	world = (Newton *)newtonWorld;
	material = world->GetMaterial (id0, id1);

	material->SetCompoundCollisionCallback ((OnContactBegin) beginCallback);
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
void* NewtonMaterialGetUserData (const NewtonWorld* newtonWorld, int id0, int id1)
{
	Newton *world;
	dgContactMaterial* material;

	world = (Newton *)newtonWorld;
	material = world->GetMaterial (id0, id1);

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

	material = (dgContactMaterial*) materialHandle;
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

	material = (dgContactMaterial*) materialHandle;
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

	material = (dgContactMaterial*) materialHandle;
	return (unsigned) material->m_userId;
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

	material = (dgContactMaterial*) materialHandle;
//	return material->m_currTimestep;

	_ASSERTE (material->m_body0);
	return material->m_body0->GetWorld()->GetTimeStep();
}



// Name: NewtonMaterialGetContactNormalSpeed 
// Calculate the speed of this contact along the normal vector of the contact. 
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair
// *const NewtonContact*  contactlHandle - pointer to the contact data
// 
// Return: Contact speed. A positive value means the contact is repulsive.
//
// Remarks: This function can only be called from a material callback event handler.
// 
// See also: NewtonMaterialSetCollisionCallback
dFloat NewtonMaterialGetContactNormalSpeed(const NewtonMaterial* materialHandle, const NewtonContact* contactlHandle)
{
	dgBody *body0;
	dgBody *body1;
	dFloat speed;
	dgContact* contact;
	dgContactMaterial* material;

	material = (dgContactMaterial*) materialHandle;
	contact = (dgContact*) contactlHandle;

	body0 = contact->GetBody0();
	body1 = contact->GetBody1();

	dgVector p0 (material->m_point - body0->GetPosition());
	dgVector p1 (material->m_point - body1->GetPosition());

	dgVector v0 (body0->GetVelocity() + body0->GetOmega() * p0);
	dgVector v1 (body1->GetVelocity() + body1->GetOmega() * p1);

	dgVector dv (v1 - v0);

	speed = dv % material->m_normal;
	return speed;
}

// Name: NewtonMaterialGetContactTangentSpeed 
// Calculate the speed of this contact along the tangent vector of the contact. 
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair.
// *const NewtonContact*  contactlHandle - pointer to the contact data.
// *int* index - index to the tangent vector. This value can be 0 for primary tangent direction or 1 for the secondary tangent direction.
// 
// Return: Contact tangent speed. 
//
// Remarks: This function can only be called from a material callback event handler.
// 
// See also: NewtonMaterialSetCollisionCallback
dFloat NewtonMaterialGetContactTangentSpeed(const NewtonMaterial* materialHandle, const NewtonContact* contactlHandle, int index)
{
	dgBody *body0;
	dgBody *body1;
	dFloat speed;
	dgVector dir;
	dgContact* contact;
	dgContactMaterial* material;

	material = (dgContactMaterial*) materialHandle;
	contact = (dgContact*) contactlHandle;

	body0 = contact->GetBody0();
	body1 = contact->GetBody1();

	dgVector p0 (material->m_point - body0->GetPosition());
	dgVector p1 (material->m_point - body1->GetPosition());

	dgVector v0 (body0->GetVelocity() + body0->GetOmega() * p0);
	dgVector v1 (body1->GetVelocity() + body1->GetOmega() * p1);

	dgVector dv (v1 - v0);

	if (index) {
		dir = material->m_dir1;
	} else {
		dir = material->m_dir0;
	}
	speed = dv % dir;
	return - speed;
}


// Name: NewtonMaterialGetContactPositionAndNormal 
// Get the contact position and normal in global space.
//
// Parameters:
// *const NewtonMaterial* materialHandle - pointer to a material pair.
// *dFloat* *positPtr,  - pointer to an array of at least three floats to hold the contact position.
// *dFloat* *normalPtr  - pointer to an array of at least three floats to hold the contact normal.
// 
// Return: Nothing.
//
// Remarks: This function can only be called from a material callback event handle.
// 
// See also: NewtonMaterialSetCollisionCallback
void NewtonMaterialGetContactPositionAndNormal(const NewtonMaterial* materialHandle, dFloat* positPtr, dFloat* normalPtr)
{
	dgContactMaterial* material;

	material = (dgContactMaterial*) materialHandle;

	positPtr[0] = material->m_point.m_x;
	positPtr[1] = material->m_point.m_y;
	positPtr[2] = material->m_point.m_z;

	normalPtr[0] = material->m_normal.m_x;
	normalPtr[1] = material->m_normal.m_y;
	normalPtr[2] = material->m_normal.m_z;
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

	material = (dgContactMaterial*) materialHandle;

	dgVector force (material->m_normal.Scale(material->m_normal_Force) +
					material->m_dir0.Scale (material->m_dir0_Force) +
					material->m_dir1.Scale (material->m_dir1_Force) );

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

	material = (dgContactMaterial*) materialHandle;

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
	dgBody *bodyPtr;
	dgCollision* collision;
	dgContactMaterial* materialPtr;

	bodyPtr = (dgBody*) body;
	materialPtr = (dgContactMaterial*) material;

	collision = materialPtr->m_collision0; 
	if (bodyPtr == materialPtr->m_body1) {
		collision = materialPtr->m_collision1; 
	}

	val = 0;
	// the convex type is 0x7437fbdd
	if (collision->IsType (0x7437fbdd)) {
		val = dgUnsigned64 (PointerToInt (collision->GetUserData()));
	}
	return dgUnsigned32 (val);
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

	material = (dgContactMaterial*) materialHandle;
	material->m_softness = ClampValue (softness, dFloat(0.01f), dFloat(0.7f));
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
	dgContactMaterial* material;

	material = (dgContactMaterial*) materialHandle;

	material->m_restitution = ClampValue (restitution, dFloat(0.01f), dFloat(2.0f));
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
	dgContactMaterial* material;

	material = (dgContactMaterial*) materialHandle;

	if (index) {
		material->m_friction1Enable = state ? true : false;
	} else {
		material->m_friction0Enable = state ? true : false;
	}
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
	dgContactMaterial* material;

	material = (dgContactMaterial*) materialHandle;

	if (index) {
		material->m_staticFriction1 = ClampValue (coef, material->m_dynamicFriction1, dFloat(2.0f));
	} else {
		material->m_staticFriction0 = ClampValue (coef, material->m_dynamicFriction0, dFloat(2.0f));
	}
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
	dgContactMaterial* material;

	material = (dgContactMaterial*) materialHandle;

	if (index) {
		material->m_dynamicFriction1 = ClampValue (coef, dFloat(0.01f), material->m_staticFriction1);
	} else {
		material->m_dynamicFriction0 = ClampValue (coef, dFloat(0.01f), material->m_staticFriction0);
	}
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

	material = (dgContactMaterial*) materialHandle;

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

	material = (dgContactMaterial*) materialHandle;
	if (index) {
		material->m_dir1_Force = accel;
		material->m_override1Accel = true;
	} else {
		material->m_dir0_Force = accel;
		material->m_override0Accel = true;
	}
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
	dgContactMaterial* material;
	material = (dgContactMaterial*) materialHandle;
	dgVector normal (direction[0], direction[1], direction[2]);

	_ASSERTE ((dgAbsf (normal % material->m_normal) - dgFloat32(1.0f)) <dgFloat32 (0.01f));
	if ((normal % material->m_normal) < dgFloat32 (0.0f)) {
		normal = normal.Scale (-dgFloat32(1.0f));
	}
//	material->m_normal = normal;
//	NewtonMaterialContactRotateTangentDirections(materialHandle, &material->m_dir0[0]);
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
	dFloat mag2;
	dgContactMaterial* material;

	material = (dgContactMaterial*) materialHandle;

	const dgVector dir0 (alignVector[0], alignVector[1], alignVector[2]);
    	
	dgVector dir1 = material->m_normal * dir0;
	mag2 = dir1 % dir1;
	if (mag2 > 1.0e-6f) {
		material->m_dir1 = dir1.Scale (dgRsqrt (mag2));
		material->m_dir0 = material->m_dir1 * material->m_normal;
	}
}


// **********************************************************************************************
//
// Name: Convex collision primitives interface
//
// **********************************************************************************************

// Name: NewtonCreateNull 
// Create a transparent collision primitive.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// 
// Return: Pointer to the collision object.
//
// Remarks: Some times the application needs to create helper rigid bodies that will never collide with other bodies,
// for example the neck of a rag doll, or an internal part of an articulated structure. This can be done by using the material system
// but it too much work and it will increase unnecessarily the material count, and therefore the project complexity. The Null collision  
// is a collision object that satisfy all this conditions without having to change the engine philosophy.
//
// Remarks: Collision primitives are reference counted objects. The application should call *NewtonReleaseCollision* in order to release references to the objects.
// Neglecting to release references to collision primitives is a common cause of memory leaks.
// Collision primitives can be reused with more than one body. This will reduce the amount of memory used by the engine, as well
// as speed up some calculations.
//
// See also: NewtonReleaseCollision
NewtonCollision* NewtonCreateNull(const NewtonWorld* newtonWorld)
{
	Newton *world;

	world = (Newton *)newtonWorld;
	return (NewtonCollision*) world->CreateNull();
}


// Name: NewtonCreateBox 
// Create a box primitive for collision.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *dFloat* dx  - box side one x dimension.  
// *dFloat* dy  - box side one y dimension.  
// *dFloat* dz  - box side one z dimension.  
// *const dFloat* *offsetMatrix - pointer to an array of 16 floats containing the offset matrix of the box relative to the body. If this parameter is NULL, then the primitive is centered at the origin of the body.
// 
// Return: Pointer to the box
//
// Remarks: Collision primitives are reference counted objects. The application should call *NewtonReleaseCollision* in order to release references to the object.
// Neglecting to release references to collision primitives is a common cause of memory leaks.
// Collision primitives can be reused with more than one body. This will reduce the amount of memory used be the engine, as well
// as speed up some calculations.
//
// See also: NewtonReleaseCollision, NewtonCreateConvexHullModifier
NewtonCollision* NewtonCreateBox(const NewtonWorld* newtonWorld, dFloat dx, dFloat dy, dFloat dz, const dFloat *offsetMatrix)
{
	Newton *world;

	world = (Newton *)newtonWorld;
	dgMatrix matrix (dgGetIdentityMatrix());
	if (offsetMatrix) {
		 matrix = *((dgMatrix*) offsetMatrix);
	}
	return (NewtonCollision*) world->CreateBox (dx, dy, dz, matrix);
}

// Name: NewtonCreateCone 
// Create a cone primitive for collision.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *dFloat* radius  - cone radius at the base.  
// *dFloat* height  - cone height along the x local axis from base to tip.  
// *const dFloat* *offsetMatrix - pointer to an array of 16 floats containing the offset matrix of the box relative to the body. If this parameter is NULL, then the primitive is centered at the origin of the body.
// 
// Return: Pointer to the box
// 
// Remark: the cone height must equal of larger than the sum of the cap radius. if this is not the case the height will be clamped the 2 * radius.
//
// Remarks: Collision primitives are reference counted objects. The application should call *NewtonReleaseCollision* in order to release references to the object.
// Neglecting to release references to collision primitives is a common cause of memory leaks.
// Collision primitives can be reused with more than one body. This will reduce the amount of memory used be the engine, as well
// as speed up some calculations.
//
// See also: NewtonReleaseCollision, NewtonCreateConvexHullModifier
NewtonCollision* NewtonCreateCone(const NewtonWorld* newtonWorld, dFloat radius, dFloat height, const dFloat *offsetMatrix)
{
	Newton *world;

	world = (Newton *)newtonWorld;
	dgMatrix matrix (dgGetIdentityMatrix());
	if (offsetMatrix) {
		 matrix = *((dgMatrix*) offsetMatrix);
	}
	return (NewtonCollision*) world->CreateCone (radius, height, matrix);
}


// Name: NewtonCreateCapsule 
// Create a capsule primitive for collision.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *dFloat* radius  - capsule radius at the base.  
// *dFloat* height  - capsule height along the x local axis from tip to tip.  
// *const dFloat* *offsetMatrix - pointer to an array of 16 floats containing the offset matrix of the box relative to the body. If this parameter is NULL, then the primitive is centered at the origin of the body.
// 
// Return: Pointer to the box
// 
// Remark: the capsule height must equal of larger than the sum of the cap radius. If this is not the case the height will be clamped the 2 * radius.
//
// Remarks: Collision primitives are reference counted objects. The application should call *NewtonReleaseCollision* in order to release references to the object.
// Neglecting to release references to collision primitives is a common cause of memory leaks.
// Collision primitives can be reused with more than one body. This will reduce the amount of memory used be the engine, as well
// as speed up some calculations.
//
// See also: NewtonReleaseCollision, NewtonCreateConvexHullModifier
NewtonCollision* NewtonCreateCapsule(const NewtonWorld* newtonWorld, dFloat radius, dFloat height, const dFloat *offsetMatrix)
{
	Newton *world;

	world = (Newton *)newtonWorld;
	dgMatrix matrix (dgGetIdentityMatrix());
	if (offsetMatrix) {
		 matrix = *((dgMatrix*) offsetMatrix);
	}
	return (NewtonCollision*) world->CreateCapsule (radius, height, matrix);
}


// Name: NewtonCreateCylinder 
// Create a cylinder primitive for collision.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *dFloat* radius  - cylinder radius at the base.  
// *dFloat* height  - cylinder height along the x local axis.  
// *const dFloat* *offsetMatrix - pointer to an array of 16 floats containing the offset matrix of the box relative to the body. If this parameter is NULL, then the primitive is centered at the origin of the body.
// 
// Return: Pointer to the box
//
// Remarks: Collision primitives are reference counted objects. The application should call *NewtonReleaseCollision* in order to release references to the object.
// Neglecting to release references to collision primitives is a common cause of memory leaks.
// Collision primitives can be reused with more than one body. This will reduce the amount of memory used be the engine, as well
// as speed up some calculations.
//
// See also: NewtonReleaseCollision, NewtonCreateConvexHullModifier
NewtonCollision* NewtonCreateCylinder(const NewtonWorld* newtonWorld, dFloat radius, dFloat height, const dFloat *offsetMatrix)
{
	Newton *world;

	world = (Newton *)newtonWorld;
	dgMatrix matrix (dgGetIdentityMatrix());
	if (offsetMatrix) {
		 matrix = *((dgMatrix*) offsetMatrix);
	}
	return (NewtonCollision*) world->CreateCylinder (radius, height, matrix);
}

NewtonCollision* NewtonCreateChamferCylinder (const NewtonWorld* newtonWorld, dFloat radius, dFloat height, const dFloat *offsetMatrix);

// Name: NewtonCreateChamferCylinder 
// Create a ChamferCylinder primitive for collision.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *dFloat* radius  - ChamferCylinder radius at the base.  
// *dFloat* height  - ChamferCylinder height along the x local axis.  
// *const dFloat* *offsetMatrix - pointer to an array of 16 floats containing the offset matrix of the box relative to the body. If this parameter is NULL, then the primitive is centered at the origin of the body.
// 
// Return: Pointer to the box
//
// Remarks: Collision primitives are reference counted objects. The application should call *NewtonReleaseCollision* in order to release references to the object.
// Neglecting to release references to collision primitives is a common cause of memory leaks.
// Collision primitives can be reused with more than one body. This will reduce the amount of memory used be the engine, as well
// as speed up some calculations.
//
// See also: NewtonReleaseCollision, NewtonCreateConvexHullModifier
NewtonCollision* NewtonCreateChamferCylinder(const NewtonWorld* newtonWorld, dFloat radius, dFloat height, const dFloat *offsetMatrix)
{
	Newton *world;

	world = (Newton *)newtonWorld;
	dgMatrix matrix (dgGetIdentityMatrix());
	if (offsetMatrix) {
		 matrix = *((dgMatrix*) offsetMatrix);
	}
	return (NewtonCollision*) world->CreateChamferCylinder (radius, height, matrix);
}


// Name: NewtonCreateConvexHull 
// Create a ConvexHull primitive for collision from a cloud of points.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *dFloat* count  - number of consecutive point to follow must be at least 4.  
// *const dFloat* vertexCloud - pointer to and array of point.  
// *dFloat* strideInBytes - vertex size in bytes, must be at least 12.  
// *const dFloat* *offsetMatrix - pointer to an array of 16 floats containing the offset matrix of the box relative to the body. If this parameter is NULL, then the primitive is centered at the origin of the body.
// 
// Return: Pointer to the box
//
// Remarks: Convex hulls are the solution to collision primitive that can not be easily represented by and implicit solid.
// The implicit solid primitives (spheres, cubes, cylinders, capsules, cones, etc.), have constant time complexity for contact calculation
// and are also extremely efficient on memory usage, therefore the application get perfect smooth behavior.
// However for cases where the shape is too difficult or a polygonal representation is desired convex hulls are the ultimate solution. 
// For example it is a mistake to model a 10000 point sphere as a convex hull, when the perfect sphere is available.
//
// Remarks: There is not upper limit as to how many vertex the application can pass to Newton to make a hull shape, 
// however for performance and memory usage concern it is the application responsibility to keep the max vertex at the possible minimum. 
// The minimum number of vertex should be equal or larger than 4 and it is the application responsibility that the points are part of a solid geometry. 
// Unpredictable results will occur if all points happen to be collinear or coplanar.  
//
// Remarks: Collision primitives are reference counted objects. The application should call *NewtonReleaseCollision* in order to release references to the object.
// Neglecting to release references to collision primitives is a common cause of memory leaks.
// Collision primitives can be reused with more than one body. This will reduce the amount of memory used be the engine, as well
// as speed up some calculations.
//
// See also: NewtonReleaseCollision, NewtonCreateConvexHullModifier
NewtonCollision* NewtonCreateConvexHull(const NewtonWorld* newtonWorld, int count, const dFloat* vertexCloud, int strideInBytes, const dFloat *offsetMatrix)
{
	Newton *world;

	world = (Newton *)newtonWorld;
	dgMatrix matrix (dgGetIdentityMatrix());
	if (offsetMatrix) {
		 matrix = *((dgMatrix*) offsetMatrix);
	}
	return (NewtonCollision*) world->CreateConvexHull (count, vertexCloud, strideInBytes, matrix);
}


// Name: NewtonCreateConvexHullModifier
// Create a collision modifier for any convex collision part.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *NewtonCollision* convexHullCollision.
// 
// Return: Pointer to the collision modifier
//
// Remarks: The matrix should be arranged in row-major order (this is the way directX stores matrices).
// a collision modifier can take any type of transformation matrix, as long as the matrix can be invertible by straight
// Gaussian elimination process. Typical uses are non-uniform scaling, translation and skewing.
//
// Remarks: Collision modifier can be used by the application to achieve effects like animating collision geometry at run time, 
// however care must taken as animation of a collision primitive could result in unwanted penetrations. 
//
// See also: NewtonReleaseCollision, NewtonConvexHullModifierSetMatrix, NewtonConvexHullModifierGetMatrix
NewtonCollision* NewtonCreateConvexHullModifier(const NewtonWorld* newtonWorld, const NewtonCollision* convexHullCollision)
{
	Newton *world;
	world = (Newton *)newtonWorld;
	return (NewtonCollision*) world->CreateConvexModifier ((dgCollision*)convexHullCollision);
}

// Name: NewtonConvexHullModifierGetMatrix
// Get the transformation matrix of a convex hull modifier collision.
//
// Parameters:
// *const NewtonCollision* *convexHullModifier - pointer to the body.
// *dFloat* *matrixPtr - pointer to an array of 16 floats containing the global matrix of the collision modifier.
//
// Return: Nothing.
//
// Remarks: The matrix should be arranged in row-major order (this is the way directX stores matrices).
// a collision modifier can take any type of transformation matrix, as long as the matrix can be invertible by straight
// Gaussian elimination process. Typical uses are non-uniform scaling, translation and skewing.
//
// Remarks: Collision modifier can be used by the application to achieve effects like animating collision geometry at run time, 
// however care must taken as animation of a collision primitive could result into unwanted penetrations. 
//
// See also: NewtonCreateConvexHullModifier, NewtonConvexHullModifierSetMatrix
void NewtonConvexHullModifierGetMatrix (const NewtonCollision* convexHullModifier, dFloat* matrixPtr)
{
	dgCollision* collision;
	collision = (dgCollision*) convexHullModifier;

	dgMatrix& matrix = (*((dgMatrix*) matrixPtr));
	matrix = collision->ModifierGetMatrix();
}


// Name: NewtonConvexHullModifierSetMatrix
// Set the transformation matrix of a convex hull modifier collision.
//
// Parameters:
// *const NewtonCollision* *convexHullModifier - pointer to the body.
// *const dFloat* *matrixPtr - pointer to an array of 16 floats containing the global matrix of the collision modifier.
//
// Return: Nothing.
//
// Remarks: The matrix should be arranged in row-major order (this is the way directX stores matrices).
// a collision modifier can take any type of transformation matrix, as long as the matrix can be invertible by straight
// Gaussian elimination process. Typical uses are non-uniform scaling, translation and skewing.
//
// Remarks: Collision modifier can be used by the application to achieve effects like animating collision geometry at run time, 
// however care must taken as animation of a collision primitive could result into unwanted penetrations. 
//
// See also: NewtonCreateConvexHullModifier, NewtonConvexHullModifierGetMatrix
void NewtonConvexHullModifierSetMatrix(const NewtonCollision* convexHullModifier, const dFloat* matrixPtr)
{
	dgCollision* collision;
	collision = (dgCollision*) convexHullModifier;

	const dgMatrix& matrix = (*((dgMatrix*) matrixPtr));
	collision->ModifierSetMatrix(matrix);
}




// Name: NewtonCreateCompoundCollision 
// Create a container to hold an array of convex collision primitives. 
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *int* count - number of primitives in the array.
// *const NewtonCollision* **collisionPrimitiveArray - pointer to an array of convex collision primitives. This array must be filled with convex collision primitives before this function is called.
//
// Return: Pointer to the compound collision.
//
// Remarks: Compound collision primitives can only be made of convex collision primitives and they can not contain compound collision. Therefore they are treated as convex primitives.
//
// Remarks: Compound collision primitives are treated as instance collision objects that can not shared by multiples rigid bodies.
//
// Remarks: Collision primitives are reference counted objects. The application should call *NewtonReleaseCollision* in order to release references to the object.
// Neglecting to release references to collision primitives is a common cause of memory leaks.
//
// See also: NewtonReleaseCollision
NewtonCollision* NewtonCreateCompoundCollision(const NewtonWorld* newtonWorld, int count, NewtonCollision* const collisionPrimitiveArray[])
{
	Newton *world;

	world = (Newton *)newtonWorld;
	return (NewtonCollision*) world->CreateCompoundCollision (count, (dgCollision**) collisionPrimitiveArray);
}

// Name: NewtonCollisionApplyScale
// Apply a global uniform scale to the collision geometry.
//
// Parameters:
// *const NewtonCollision* collision - is the pointer to a convex collision primitive.
// *dFloat* scale - scale factor.
// 
// Remarks: this function apply a scale factor to the collision geometry the result is irreversible.
//
// Return: nothing
void NewtonCollisionApplyScale(const NewtonCollision* collision, dFloat scale)
{
	dgCollision* coll;
	NewtonCollisionTree *dataBase;
	dgCollisionPolygonalSoup* polySoup;
	coll = (dgCollision*) collision;

	if (coll->IsType (dgConvexCollision::GetRttiType())) {
		coll->ApplyUniformScale(scale);
	} else if (coll->IsType (dgCollisionPolygonalSoup::GetRttiType())) {
		polySoup = (dgCollisionPolygonalSoup*) coll;
		coll->ApplyUniformScale(scale);
		dataBase = (NewtonCollisionTree *)polySoup->GetUserData();
		dataBase->ApplyScale (scale);
	}
}

// Name: NewtonConvexCollisionSetUserID 
// Store a user defined value with a convex collision primitive.
//
// Parameters:
// *const NewtonCollision* convexCollision - is the pointer to a convex collision primitive.
// *unsigned* id - value to store with the collision primitive.
// 
// Return: nothing
//
// Remarks: the application can store an id with any collision primitive. This id can be used to identify what type of collision primitive generated a contact.
//
// See also: NewtonMaterialGetBodyCollisionID, NewtonConvexCollisionGetUserID, NewtonCreateBox, NewtonCreateSphere
void NewtonConvexCollisionSetUserID(const NewtonCollision* convexCollision, unsigned id)
{
	dgCollision* collision;
	collision = (dgCollision*) convexCollision;
	// the convex type is 0x7437fbdd
	if (collision->IsType (0x7437fbdd)) {
		//collision->SetUserData ((void*) (dgUnsigned64 (id)));
		collision->SetUserData (IntToPointer (id));
	}
}


// Name: NewtonConvexCollisionGetUserID 
// Return a user define value with a convex collision primitive.
//
// Parameters:
// *const NewtonCollision* convexCollision - is the pointer to a convex collision primitive.
// 
// Return: user id
//
// Remarks: the application can store an id with any collision primitive. This id can be used to identify what type of collision primitive generated a contact.
//
// See also: NewtonMaterialGetBodyCollisionID, NewtonMaterialGetBodyCollisionID, NewtonCreateBox, NewtonCreateSphere
unsigned NewtonConvexCollisionGetUserID(const NewtonCollision* convexCollision)
{
	dgCollision* collision;
	collision = (dgCollision*) convexCollision;
	//return unsigned (dgUnsigned64 (collision->GetUserData()));
	return unsigned (PointerToInt (collision->GetUserData()));
	
}

// Name: NewtonConvexCollisionCalculateVolume
// calculate the total volume defined by a convex collision geometry.
//
// Parameters:
// *const NewtonCollision* *convexCollision - pointer to the collision.
//
// Return: collision geometry volume. This function will return zero if the body collision geometry is no convex.
//
// Remarks: The total volume calculated by the function is only an approximation of the ideal volume. This is not an error, it is a fact resulting from the polygonal representation of convex solids.
//
// Remarks: This function can be used to assist the application in calibrating features like fluid density weigh factor when calibrating buoyancy forces for more realistic result.
//
// See also: NewtonBodyAddBuoyancyForce
dFloat NewtonConvexCollisionCalculateVolume(const NewtonCollision* convexCollision)
{
	dgCollision *collision;
	collision = (dgCollision *)convexCollision;
	return collision->GetVolume();
}


// Name: NewtonConvexCollisionCalculateInertialMatrix 
// Calculate the three principal axis and the the values of the inertia matrix of a convex collision objects.
//
// Parameters:
// *const NewtonCollision* convexCollision - is the pointer to a convex collision primitive.
// *dFloat* *inertia - pointer to and array of a least 3 floats to hold the values of the principal inertia.
// *dFloat* *origin - pointer to and array of a least 3 floats to hold the values of the center of mass for the principal inertia.
// 
// Remarks: This function calculate a general inertial matrix for arbitrary convex collision including compound collisions.
//
// See also: NewtonBodySetMassMatrix, NewtonBodyGetMassMatrix, NewtonBodySetCentreOfMass, NewtonBodyGetCentreOfMass
void NewtonConvexCollisionCalculateInertialMatrix(const NewtonCollision* convexCollision, dFloat* inertia, dFloat* origin)
{
	dgCollision *collision;
	collision = (dgCollision *)convexCollision;
	collision->CalculateInertia(*((dgVector*) inertia), *((dgVector*) origin));
}

// **********************************************************************************************
//
// Name: Complex collision primitives interface
//
// **********************************************************************************************




// Name: NewtonCreateUserMeshCollision 
// Create a complex collision geometry to be controlled by the application.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const dFloat* *minBox - pointer to an array of at least three floats to hold minimum value for the box relative to the collision.
// *const dFloat* *maxBox - pointer to an array of at least three floats to hold maximum value for the box relative to the collision.
// *void* *userData - pointer to user data to be used as context for event callback.
// *NewtonUserMeshCollisionCollideCallback* collideCallback - pointer to an event function for providing Newton with the polygon inside a given box region.
// *NewtonUserMeshCollisionRayHitCallback* rayHitCallBack   - pointer to an event function for providing Newton with ray intersection information.
// *NewtonUserMeshCollisionDestroyCallback* destroyCallback	- pointer to an event function for destroying any data allocated for use by the application.
//
// Return: Pointer to the user collision.
//
// Remarks: *UserMeshCollision* provides the application with a method of overloading the built-in collision system for background objects.
// UserMeshCollision can be used for implementing collisions with height maps, collisions with BSP, and any other collision structure the application
// supports and wishes to preserve.
// However, *UserMeshCollision* can not take advantage of the efficient and sophisticated algorithms and data structures of the 
// built-in *TreeCollision*. We suggest you experiment with both methods and use the method best suited to your situation.
//
// Remarks: When a *UserMeshCollision* is assigned to a body, the mass of the body is ignored in all dynamics calculations. 
// This make the body behave as a static body.
//
// Remarks: The most significant bit of a triangle index list is use to indite to Newton that edge between
// that index and next index is not part of collision calculation. 
// The application can use this feature to mark concave and coplanar shared edges as non collidables.
// for example is two triangles has indices 1, 2, 3  and 3, 2, 4 respective, if the two triangle are coplanar, then the application can mark edge 2, 3 as:
// 1, 2 | 0x80000000, 3 | 0x80000000 for triangle 1 and 3 | 0x80000000, 2 | 0x80000000, 4 for the second triangle.
// these feature greatly enhances the quality of contact generation meshes with flat and concave faces. 
//
// Remarks: Collision primitives are reference counted objects. The application should call *NewtonReleaseCollision* in order to release references to the object.
// Neglecting to release references to collision primitives is a common cause of memory leaks.
// Collision primitives can be reused with more than one body. This will reduce the amount of memory used be the engine, as well
// as speed up some calculations.
//
// See also: NewtonReleaseCollision
NewtonCollision* NewtonCreateUserMeshCollision(
	const NewtonWorld* newtonWorld, 
	const dFloat *minBox, 
	const dFloat *maxBox, 
	void *userData,
	NewtonUserMeshCollisionCollideCallback collideCallback, 
	NewtonUserMeshCollisionRayHitCallback rayHitCallBack,
	NewtonUserMeshCollisionDestroyCallback destroyCallback)
{
	Newton *world;
	dgCollision* collision;

	dgVector p0 (minBox[0], minBox[1], minBox[2], dgFloat32(1.0f)); 
	dgVector p1 (maxBox[0], maxBox[1], maxBox[2], dgFloat32(1.0f)); 

	world = (Newton *)newtonWorld;
/*
	return (NewtonCollision*) world->CreatePolygonSoup (p0, p1, userData, 
												   (OnPolygonSoupCollideCallback) collideCallback, 
												   (OnPolygonSoupRayHitCallback) rayHitCallBack, 
												   (OnPolygonSoupDestroyCallback) destroyCallback);
*/

	collision =  world->CreatePolygonSoup (userData, 
										  (OnPolygonSoupCollideCallback) collideCallback, 
										  (OnPolygonSoupRayHitCallback) rayHitCallBack, 
										  (OnPolygonSoupDestroyCallback) destroyCallback);

	collision->SetCollisionBBox(p0, p1);
	return (NewtonCollision*)collision; 

}



// Name: NewtonCreateTreeCollision 
// Create an empty complex collision geometry tree. 
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *NewtonTreeCollisionCallback *userCallback - pointer to an event function to call before Newton evaluates the polygons colliding with a body. This parameter can be NULL.     
//
// Return: Pointer to the collision tree.
//
// Remarks: *TreeCollision* is the preferred method within Newton for collision with polygonal meshes of arbitrary complexity.
// The mesh must be made of flat non-intersecting polygons, but they do not explicitly need to be triangles.
// *TreeCollision* can be serialized by the application to/from an arbitrary storage device. 
//
// Remarks: When a *TreeCollision* is assigned to a body the mass of the body is ignored in all dynamics calculations. 
// This makes the body behave as a static body.
//
// Remarks: Collision primitives are reference counted objects. The application should call *NewtonReleaseCollision* in order to release references to the object.
// Neglecting to release references to collision primitives is a common cause of memory leaks.
// Collision primitives can be reused with more than one body. This will reduce the amount of memory used by the engine, as well
// as speed up some calculations.
//
// See also: NewtonTreeCollisionBeginBuild, NewtonTreeCollisionAddFace, NewtonTreeCollisionEndBuild, NewtonTreeCollisionSerialize, NewtonCreateTreeCollisionFromSerialization, NewtonTreeCollisionGetFaceAtribute, NewtonTreeCollisionSetFaceAtribute, NewtonReleaseCollision 
NewtonCollision* NewtonCreateTreeCollision(const NewtonWorld* newtonWorld, NewtonTreeCollisionCallback userCallback)
{

	Newton *world;
	NewtonCollisionTree *dataBase;

	world = (Newton *)newtonWorld;

	dataBase = new NewtonCollisionTree (world, userCallback);
	return (NewtonCollision*) world->CreatePolygonSoup (dataBase,
												   NewtonCollisionTree::GetIntersectingPolygons, 
												   NewtonCollisionTree::RayHit, 
												   NewtonCollisionTree::Destroy);
}


// Name: NewtonTreeCollisionBeginBuild 
// Prepare a *TreeCollision* to begin to accept the polygons that comprise the collision mesh.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
//
// Return: Nothing.
//
// See also: NewtonTreeCollisionAddFace, NewtonTreeCollisionEndBuild
void NewtonTreeCollisionBeginBuild(const NewtonCollision* treeCollision)
{
	dgCollision* collision;

	collision = (dgCollision*) treeCollision;
	NewtonCollisionTree& tree = *((NewtonCollisionTree*)  collision->GetUserData());

	tree.m_builder = new dgPolygonSoupDatabaseBuilder;
	tree.m_builder->Begin();
}

// Name: NewtonTreeCollisionAddFace 
// Add an individual polygon to a *TreeCollision*.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *int* vertexCount - number of vertex in *vertexPtr*
// *const dFloat* *vertexPtr - pointer to an array of vertex. The vertex should consist of at least 3 floats each.
// *int* strideInBytes - size of each vertex in bytes. This value should be 12 or larger.
// *int* faceAttribute - id that identifies the polygon. The application can use this value to customize the behavior of the collision geometry.
//
// Return: Nothing.
//
// Remarks: After the call to *NewtonTreeCollisionBeginBuild* the *TreeCollision* is ready to accept polygons. The application should iterate
// through the application's mesh, adding the mesh polygons to the *TreeCollision* one at a time.
// The polygons must be flat and non-self intersecting.
//
// See also: NewtonTreeCollisionAddFace, NewtonTreeCollisionEndBuild
void NewtonTreeCollisionAddFace(
	const NewtonCollision* treeCollision,
	int vertexCount, 
	const dFloat* vertexPtr, 
	int strideInBytes, 
	int faceAttribute)
{
	dgInt32 i;
	dgInt32 faceArray;
	dgCollision* collision;
	dgInt32 indices[1024];

	collision = (dgCollision*) treeCollision;
	NewtonCollisionTree& tree = *((NewtonCollisionTree*)  collision->GetUserData());
	dgPolygonSoupDatabaseBuilder& builder = *tree.m_builder;
	
	for (i = 0; i < vertexCount; i ++) {
		indices[i] = i;
	}
	faceArray = vertexCount;
	
	builder.AddMesh (vertexPtr, vertexCount, strideInBytes, 1, &faceArray, indices, &faceAttribute, dgGetIdentityMatrix());
}

// Name: NewtonTreeCollisionEndBuild 
// Finalize the construction of the polygonal mesh.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *int* optimize - flag that indicates to Newton whether it should optimize this mesh. Set to 1 to optimize the mesh, otherwise 0.
//
// Return: Nothing.
//
// Remarks: After the application has finished adding polygons to the *TreeCollision*, it must call this function to finalize the construction of the collision mesh.
// If concave polygons are added to the *TreeCollision*, the application must call this function with the parameter *optimize* set to 1.
// With the *optimize* parameter set to 1, Newton will optimize the collision mesh by removing non essential edges from adjacent flat polygons.
// Newton will not change the topology of the mesh but significantly reduces the number of polygons in the mesh. The reduction factor of the number of polygons in the mesh depends upon the irregularity of the mesh topology.
// A reduction factor of 1.5 to 2.0 is common. 
// Calling this function with the parameter *optimize* set to zero, will leave the mesh geometry unaltered.
//
// See also: NewtonTreeCollisionAddFace, NewtonTreeCollisionEndBuild
void NewtonTreeCollisionEndBuild(const NewtonCollision* treeCollision, int optimize)
{
	dgCollision* collision;
	collision = (dgCollision*) treeCollision;
	NewtonCollisionTree& tree = *((NewtonCollisionTree*)  collision->GetUserData());
	
    tree.m_builder->End(optimize ? true : false);
//	if (tree.IsType (dgAABBPolygonSoup::GetRttiType())) {
	tree.m_builder->SingleFaceFixup();
//	}

	tree.Create (*tree.m_builder);

	dgVector p0;
	dgVector p1;

	tree.GetAABB (p0, p1);
//	tree.MarkConcaveEdges ();

	collision->SetCollisionBBox (p0, p1);

	delete tree.m_builder;
	tree.m_builder = NULL;
}


// Name: NewtonTreeCollisionSerialize
// Serialize a *TreeCollision*.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *NewtonSerialize* callback - pointer to the event function that will do the serialization. 
// *void* *serializeHandle	- user data that will be passed to the *NewtonSerialize* callback.
//
// Return: Nothing.
//
// Remarks: Small and medium size *TreeCollision* objects (under 50000 polygons) can be constructed at application startup without significant processing overhead.
// However, for very large polygons sets (over 50000 polygons) it is recommended that the application use *NewtonCreateTreeCollision*
// in an off-line tool. Then the application can call this function to store the *TreeCollision* to a file or 
// any file packer system the application is using. At run time the application can use the function *NewtonCreateTreeCollisionFromSerialization* 
// to create and load a pre-made *TreeCollision*
//
// See also: NewtonCreateTreeCollisionFromSerialization
void NewtonTreeCollisionSerialize(const NewtonCollision* treeCollision, NewtonSerialize callback, void* serializeHandle)
{
	dgCollision* collision;
	collision = (dgCollision*) treeCollision;
	NewtonCollisionTree& tree = *((NewtonCollisionTree*)  collision->GetUserData());

	tree.Serialize ((dgSerialize) callback, serializeHandle);
}

// Name: NewtonCreateTreeCollisionFromSerialization
// Create a tree collision and load the polygon mesh via a serialization function.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *NewtonTreeCollisionCallback *userCallback - pointer to an event function to call before Newton is begins collecting polygons that are colliding with a body. This parameter can be NULL.
// *NewtonSerialize* callback - pointer to the callback function that will handle the serialization.
// *void* *userData	- user data that will be passed as the argument to *NewtonSerialize* callback.
//
// Return: Nothing.
//
// Remarks: Small and medium size *TreeCollision* objects (under 50000 polygons) can be constructed at application startup without significant processing overhead.
// However, for very large polygons sets (over 50000 polygons) it is recommended that the application use *NewtonCreateTreeCollision* 
// in an off-line tool. Then the application can call this function to store the *TreeCollision* to a file or 
// any file packer system the application is using. At run time the application can use the function *NewtonCreateTreeCollisionFromSerialization* 
// to create and load a pre-made *TreeCollision*
//
// See also: NewtonTreeCollisionSerialize, NewtonReleaseCollision
NewtonCollision* NewtonCreateTreeCollisionFromSerialization(
	const NewtonWorld* newtonWorld, 
	NewtonTreeCollisionCallback userCallback,
	NewtonDeserialize deserializeFunction, 
	void* serializeHandle)
{
	Newton *world;
	dgCollision* collision;
	NewtonCollisionTree *dataBase;

	world = (Newton *)newtonWorld;

	dataBase = new NewtonCollisionTree (world, userCallback);
	dataBase->Deserialize (deserializeFunction, serializeHandle);

	dgVector p0; 
	dgVector p1; 

	dataBase->GetAABB (p0, p1);

	world = (Newton *)newtonWorld;
/*
	return (NewtonCollision*) world->CreatePolygonSoup (p0, p1, dataBase,
						 NewtonCollisionTree::GetIntersectingPolygons, 
						 NewtonCollisionTree::RayHit, 
						 NewtonCollisionTree::Destroy);
*/

	collision = world->CreatePolygonSoup (dataBase,
										  NewtonCollisionTree::GetIntersectingPolygons, 
										  NewtonCollisionTree::RayHit, 
										  NewtonCollisionTree::Destroy);

	collision->SetCollisionBBox(p0, p1);
	return (NewtonCollision*)collision;

}


// Name: NewtonTreeCollisionGetFaceAtribute 
// Get the user defined collision attributes stored with each face of the collision mesh.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const int* *faceIndexArray - pointer to the face index list passed to the function *NewtonTreeCollisionCallback userCallback*
//
// Return: User id of the face.
//
// Remarks: This function is used to obtain the user data stored in faces of the collision geometry.
// The application can use this user data to achieve per polygon material behavior in large static collision meshes.
//
// See also: NewtonTreeCollisionSetFaceAtribute, NewtonCreateTreeCollision, NewtonCreateTreeCollisionFromSerialization
int NewtonTreeCollisionGetFaceAtribute(const NewtonCollision* treeCollision, const int* faceIndexArray)
{
	dgCollision* collision;
	collision = (dgCollision*) treeCollision;
	NewtonCollisionTree& tree = *((NewtonCollisionTree*)  collision->GetUserData());

	return tree.GetTagId (faceIndexArray);
}

// Name: NewtonTreeCollisionSetFaceAtribute 
// Change the user defined collision attribute stored with faces of the collision mesh.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const int* *faceIndexArray - pointer to the face index list passed to the function *NewtonTreeCollisionCallback userCallback*
// *int* attribute - value of the user defined attribute to be stored with the face.
//
// Return: User id of the face.
//
// Remarks: This function is used to obtain the user data stored in faces of the collision geometry.
// The application can use this user data to achieve per polygon material behavior in large static collision meshes.
// By changing the value of this user data the application can achieve modifiable surface behavior with the collision geometry.
// For example, in a driving game, the surface of a polygon that represents the street can changed from pavement to oily or wet after
// some collision event occurs.
//
// See also: NewtonTreeCollisionGetFaceAtribute, NewtonCreateTreeCollision, NewtonCreateTreeCollisionFromSerialization
void NewtonTreeCollisionSetFaceAtribute(const NewtonCollision* treeCollision, const int* faceIndexArray, int attribute)
{
	dgCollision* collision;
	collision = (dgCollision*) treeCollision;
	NewtonCollisionTree& tree = *((NewtonCollisionTree*)  collision->GetUserData());

	return tree.SetTagId (faceIndexArray, attribute);
}



// Name: NewtonTreeCollisionGetVertexListIndexListInAABB 
//
int NewtonTreeCollisionGetVertexListIndexListInAABB(const NewtonCollision* treeCollision, const dFloat* p0, const dFloat* p1,
													const dFloat** vertexArray, int* vertexCount, int* vertexStrideInBytes, 
													const int* indexList, int maxIndexCount) 
{
	dgInt32 count;
	dgCollision* collision;
	dgCollisionPolygonalSoup* collisionTree;

	count = 0;
	collision = (dgCollision*) treeCollision;
	if (collision->IsType(dgCollisionPolygonalSoup::GetRttiType())) {
		collisionTree = (dgCollisionPolygonalSoup*) collision;
		if (collisionTree->GetCallBack() ==  NewtonCollisionTree::GetIntersectingPolygons) {
			NewtonCollisionTree& tree = *((NewtonCollisionTree*) collisionTree->GetUserData());
			dgVector pmin (p0[0], p0[1], p0[2], dgFloat32 (0.0f));
			dgVector pmax (p1[0], p1[1], p1[2], dgFloat32 (0.0f));


			NewtonCollisionTree::dgGetVertexListIndexList data;

			data.m_indexList = (dgInt32 *)indexList;
			data.m_maxIndexCount = maxIndexCount;
			data.m_indexCount = 0; 

			tree.GetVertexListIndexList (pmin, pmax, data);

			count = data.m_indexCount;
			*vertexCount = tree.GetVertexCount();
			*vertexArray = tree.GetLocalVertexPool();
			*vertexStrideInBytes = tree.GetStrideInBytes();
		}
	}
	return count;
}


// **********************************************************************************************
//
// Name: Generic collision library functions
//
// **********************************************************************************************


// Name: NewtonCollisionPointDistance 
// Calculate the closest point between a point and convex collision primitive.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *dFloat* *point - pointer to and array of a least 3 floats representing the origin.
// *const NewtonCollision* *collision -  pointer to collision primitive.
// *const dFloat* *matrix - pointer to an array of 16 floats containing the offset matrix of collision primitiveA.
// *dFloat* *contact - pointer to and array of a least 3 floats to contain the closest point to collisioA.
// *dFloat* *normal - pointer to and array of a least 3 floats to contain the separating vector normal.
// 
// Return: one if the two bodies are disjoint and the closest point could be found, 
// zero if the point is inside the convex primitive.
//
// Remarks: This function can be used as a low-level building block for a stand-alone collision system. 
// Applications that have already there own physics system, and only want and quick and fast collision solution, 
// can use Newton advanced collision engine as the low level collision detection part. 
// To do this the application only needs to initialize Newton, create the collision primitives at application discretion, 
// and just call this function when the objects are in close proximity. Applications using Newton as a collision system 
// only, are responsible for implementing their own broad phase collision determination, based on any high level tree structure. 
// Also the application should implement their own trivial aabb test, before calling this function .
//
// Remarks: the current implementation of this function do work on collision trees, or user define collision.
//
// See also: NewtonCollisionCollideContinue, NewtonCollisionClosestPoint, NewtonCollisionCollide, NewtonCollisionRayCast, NewtonCollisionCalculateAABB
int NewtonCollisionPointDistance(const NewtonWorld* newtonWorld, const dFloat *point,
								 const NewtonCollision* collision, const dFloat* matrix,
								 dFloat* contact, dFloat* normal)
{
	Newton *world;
	world = (Newton *)newtonWorld;
	return world->ClosestPoint (*((dgTriplex*) point), (dgCollision*)collision, *((dgMatrix*) matrix), 
								*((dgTriplex*) contact), *((dgTriplex*) normal));
}



// Name: NewtonCollisionClosestPoint 
// Calculate the closest points between two disjoint convex collision primitive.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const NewtonCollision* *collisionA -  pointer to collision primitive A.
// *const dFloat* *matrixA - pointer to an array of 16 floats containing the offset matrix of collision primitiveA.
// *const NewtonCollision* *collisionB - pointer to collision primitive B.
// *const dFloat* *matrixB - pointer to an array of 16 floats containing the offset matrix of collision primitiveB.
// *dFloat* *contactA - pointer to and array of a least 3 floats to contain the closest point to collisionA.
// *dFloat* *contactB - pointer to and array of a least 3 floats to contain the closest point to collisionB.
// *dFloat* *normalAB - pointer to and array of a least 3 floats to contain the separating vector normal.
// 
// Return: one if the tow bodies are disjoint and he closest point could be found, 
// zero if the two collision primitives are intersecting.
//
// Remarks: This function can be used as a low-level building block for a stand-alone collision system. 
// Applications that have already there own physics system, and only want and quick and fast collision solution, 
// can use Newton advanced collision engine as the low level collision detection part. 
// To do this the application only needs to initialize Newton, create the collision primitives at application discretion, 
// and just call this function when the objects are in close proximity. Applications using Newton as a collision system 
// only, are responsible for implementing their own broad phase collision determination, based on any high level tree structure. 
// Also the application should implement their own trivial aabb test, before calling this function .
//
// Remarks: the current implementation of this function does not work on collision trees, or user define collision.
//
// See also: NewtonCollisionCollideContinue, NewtonCollisionPointDistance, NewtonCollisionCollide, NewtonCollisionRayCast, NewtonCollisionCalculateAABB
int NewtonCollisionClosestPoint(const NewtonWorld* newtonWorld, 
								const NewtonCollision* collisionA, const dFloat* matrixA,
								const NewtonCollision* collisionB, const dFloat* matrixB,
								dFloat* contactA, dFloat* contactB, dFloat* normalAB)
{
	Newton *world;
	world = (Newton *)newtonWorld;
	return world->ClosestPoint ((dgCollision*)collisionA, *((dgMatrix*) matrixA), 
								(dgCollision*)collisionB, *((dgMatrix*) matrixB), 
								*((dgTriplex*) contactA), *((dgTriplex*) contactB), *((dgTriplex*) normalAB));
}


// Name: NewtonCollisionCollide 
// Calculate contact points between two collision primitive.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *int* maxSize - size of maximum number of elements in contacts, normals, and penetration.
// *const NewtonCollision* *collisionA -  pointer to collision primitive A.
// *const dFloat* *matrixA - pointer to an array of 16 floats containing the offset matrix of collision primitiveA.
// *const NewtonCollision* *collisionB - pointer to collision primitive B.
// *const dFloat* *matrixB - pointer to an array of 16 floats containing the offset matrix of collision primitiveB.
// *dFloat* *contacts - pointer to and array of a least 3 times maxSize floats to contain the collision contact points.
// *dFloat* *normals - pointer to and array of a least 3 times maxSize floats to contain the collision contact normals.
// *dFloat* *penetration - pointer to and array of a least maxSize floats to contain the collision penetration at each contact.
// 
// Return: the number of contact points.
//
// Remarks: This function can be used as a low-level building block for a stand-alone collision system. 
// Applications that have already there own physics system, and only want and quick and fast collision solution, 
// can use Newton advanced collision engine as the low level collision detection part. 
// To do this the application only needs to initialize Newton, create the collision primitives at application discretion, 
// and just call this function when the objects are in close proximity. Applications using Newton as a collision system 
// only, are responsible for implementing their own broad phase collision determination, based on any high level tree structure. 
// Also the application should implement their own trivial aabb test, before calling this function .
//
// See also: NewtonCollisionCollideContinue, NewtonCollisionClosestPoint, NewtonCollisionPointDistance, NewtonCollisionRayCast, NewtonCollisionCalculateAABB
int NewtonCollisionCollide(const NewtonWorld* newtonWorld, int maxSize,
						   const NewtonCollision* collisionA, const dFloat* matrixA,
						   const NewtonCollision* collisionB, const dFloat* matrixB,
						   dFloat* contacts, dFloat* normals, dFloat* penetration)
{
	Newton *world;
	world = (Newton *)newtonWorld;
	return world->Collide ((dgCollision*)collisionA, *((dgMatrix*) matrixA), 
		(dgCollision*)collisionB, *((dgMatrix*) matrixB), 
		(dgTriplex*) contacts, (dgTriplex*) normals, penetration, maxSize);
}



// Name: NewtonCollisionCollideContinue 
// Calculate time of impact of impact and contact points between two collision primitive.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *int* maxSize - size of maximum number of elements in contacts, normals, and penetration.
// *const dFloat* timestep - maximum time interval consided for the continue collision claculation. 
// *const NewtonCollision* *collisionA -  pointer to collision primitive A.
// *const dFloat* *matrixA - pointer to an array of 16 floats containing the offset matrix of collision primitiveA.
// *const dFloat* *velocA - pointer to and array of a least 3 times maxSize floats containing the linear velocity of collision primitiveA.
// *const dFloat* *omegaA - pointer to and array of a least 3 times maxSize floats containing the angular velocity of collision primitiveA.
// *const NewtonCollision* *collisionB - pointer to collision primitive B.
// *const dFloat* *matrixB - pointer to an array of 16 floats containing the offset matrix of collision primitiveB.
// *const dFloat* *velocB - pointer to and array of a least 3 times maxSize floats containing the linear velocity of collision primitiveB.
// *const dFloat* *omegaB - pointer to and array of a least 3 times maxSize floats containing the angular velocity of collision primitiveB.
// *dFloat* *timeOfImpact - pointer to least 1 float variable to contain the time of the intersection.
// *dFloat* *contacts - pointer to and array of a least 3 times maxSize floats to contain the collision contact points.
// *dFloat* *normals - pointer to and array of a least 3 times maxSize floats to contain the collision contact normals.
// *dFloat* *penetration - pointer to and array of a least maxSize floats to contain the collision penetration at each contact.
// 
// Return: the number of contact points.
//
// Remarks: by passing zero as *maxSize* not contact will be calculated and the function will just determine the time of impact is any.
//
// Remarks: if the body are interpenetrating the time of impact will be zero.
//
// Remarks: if the bodies do not collide time of impact will be set to *timestep*
//
// Remarks: This function can be used as a low-level building block for a stand-alone collision system. 
// Applications that have already there own physics system, and only want and quick and fast collision solution, 
// can use Newton advanced collision engine as the low level collision detection part. 
// To do this the application only needs to initialize Newton, create the collision primitives at application discretion, 
// and just call this function when the objects are in close proximity. Applications using Newton as a collision system 
// only, are responsible for implementing their own broad phase collision determination, based on any high level tree structure. 
// Also the application should implement their own trivial aabb test, before calling this function .
//
// See also: NewtonCollisionCollide, NewtonCollisionClosestPoint, NewtonCollisionPointDistance, NewtonCollisionRayCast, NewtonCollisionCalculateAABB
int NewtonCollisionCollideContinue(const NewtonWorld* newtonWorld, int maxSize, const dFloat timestep, 
		const NewtonCollision* collisionA, const dFloat* matrixA, const dFloat* velocA, const dFloat* omegaA, 
		const NewtonCollision* collisionB, const dFloat* matrixB, const dFloat* velocB, const dFloat* omegaB, 
		dFloat* timeOfImpact, dFloat* contacts, dFloat* normals, dFloat* penetration)
{
//	Newton *world;
//	world = (Newton *)newtonWorld;
//	return world->Collide ((dgCollision*)collisionA, *((dgMatrix*) matrixA), 
//		(dgCollision*)collisionB, *((dgMatrix*) matrixB), 
//		(dgTriplex*) contacts, (dgTriplex*) normals, penetration, maxSize);
//	_ASSERTE (0);
	Newton *world;
	world = (Newton *)newtonWorld;

	*timeOfImpact = timestep;
	return world->CollideContinue ((dgCollision*)collisionA, *((dgMatrix*) matrixA), *((dgVector*) velocA), *((dgVector*) omegaA), 
								   (dgCollision*)collisionB, *((dgMatrix*) matrixB), *((dgVector*) velocB), *((dgVector*) omegaB), 
								   *timeOfImpact, (dgTriplex*) contacts, (dgTriplex*) normals, penetration, maxSize);
}

// Name: NewtonCollisionRayCast 
// Ray cast specific collision object.
//
// Parameters:
// *const NewtonCollision* *collisionPtr - pointer to the collision object.
// *dFloat* *p0 - pointer to an array of at least three floats representing the ray origin in the local space of the geometry.
// *dFloat* *p1 - pointer to an array of at least three floats representing the ray end in the local space of the geometry.
// *dFloat* *normal - pointer to an array of at least three floats to hold the normal at the intersection point.
// *int* *attribute - pointer to an array of at least one floats to hold the ID of the face hit by the ray.
//
// Return: the parametric value of the intersection, between 0.0 and 1.0, an value larger than 1.0 if the ray miss.
//
// Remarks: This function is intended for applications using newton collision system separate from the dynamics system, also for applications  
// implementing any king of special purpose logic like sensing distance to another object.
//
// Remarks: the ray most be local to the collisions geometry, for example and application ray casting the collision geometry of 
// of a rigid body, must first take the points p0, and p1 to the local space of the rigid body by multiplying the points by the 
// inverse of he rigid body transformation matrix. 
//
// See also: NewtonCollisionClosestPoint, NewtonCollisionPointDistance, NewtonCollisionCollide, NewtonCollisionCalculateAABB
dFloat NewtonCollisionRayCast(const NewtonCollision* collisionPtr, const dFloat* p0, const dFloat* p1, dFloat* normal, int* attribute)
{
	dFloat t;
	dgCollision* collision;
	collision = (dgCollision*) collisionPtr;


	const dgMatrix& matrix = collision->GetOffsetMatrix ();

	dgVector q0 (matrix.UntransformVector (dgVector (p0[0], p0[1], p0[2]))); 
	dgVector q1 (matrix.UntransformVector (dgVector (p1[0], p1[1], p1[2]))); 
	dgContactPoint contact;

	t = collision->RayCast (q0, q1, contact, NULL, NULL, NULL);
	if (t >= dFloat (0.0f) && t <= dFloat (dgFloat32(1.0f))) {
		attribute[0] = (int)contact.m_userId;

		dgVector n (matrix.RotateVector (contact.m_normal));
		normal[0] = n[0];
		normal[1] = n[1];
		normal[2] = n[2];
	}
	return t;
}

// Name: NewtonCollisionCalculateAABB 
// Calculate an axis-aligned bounding box for this collision, the box is calculated relative to *offsetMatrix*. 
//
// Parameters:
// *const NewtonCollision* *collisionPtr - pointer to the collision object.
// *const dFloat* *offsetMatrix - pointer to an array of 16 floats containing the offset matrix used as the coordinate system and center of the AABB. 
// *dFloat* *p0 - pointer to an array of at least three floats to hold minimum value for the AABB.
// *dFloat* *p1 - pointer to an array of at least three floats to hold maximum value for the AABB.
//
// Return: Nothing.
//
// See also: NewtonCollisionClosestPoint, NewtonCollisionPointDistance, NewtonCollisionCollide, NewtonCollisionRayCast
void NewtonCollisionCalculateAABB(const NewtonCollision* collisionPtr, const dFloat *offsetMatrix, dFloat* p0, dFloat* p1)
{
	dgCollision* collision;
	collision = (dgCollision*) collisionPtr;
	const dgMatrix& matrix = *((dgMatrix*) offsetMatrix);

	dgVector q0;
	dgVector q1;

	collision->CalcAABB (matrix, q0, q1);
	p0[0] = q0.m_x;
	p0[1] = q0.m_y;
	p0[2] = q0.m_z;

	p1[0] = q1.m_x;
	p1[1] = q1.m_y;
	p1[2] = q1.m_z;
}

// Name: NewtonCollisionMakeUnique 
// Convert a collision primitive to a unique instance by removing it for the collision cache.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const NewtonCollision* *collisionPtr - pointer to the collision object
//
// Return: Nothing.
//
// Remarks: 
// This function will not make preexisting collision object unique instances, so for best result this function should be call immediately after the 
// creation of the collision object.
//
// Remarks: 
// Collision objects are reference counted objects. The application should call *NewtonReleaseCollision* in order to release references to the object.
// Neglecting to release references to collision primitives is a common cause of memory leaks.
void NewtonCollisionMakeUnique(const NewtonWorld* newtonWorld, const NewtonCollision* collisionPtr)
{
	Newton *world;
	dgCollision* collision;

	world = (Newton *)newtonWorld;
	collision = (dgCollision*) collisionPtr;
	world->RemoveFromCache (collision);
}

// **********************************************************************************************
//
// Name: Transform utility functions 
//
// **********************************************************************************************


// Name: NewtonGetEulerAngle 
// Get the three Euler angles from a 4x4 rotation matrix arranged in row-major order.
//
// Parameters:
// *const dFloat* matrix - pointer to the 4x4 rotation matrix.
// *dFloat* angles - pointer to an array of at least three floats to hold the Euler angles.
//
// Return: Nothing.
//
// Remarks: The motivation for this function is that many graphics engines still use Euler angles to represent the orientation
// of graphics entities. 
// The angles are expressed in radians and represent: 
// *angle[0]* - rotation about first matrix row
// *angle[1]* - rotation about second matrix row
// *angle[2]* - rotation about third matrix row
// 	
// See also: NewtonSetEulerAngle
void NewtonGetEulerAngle(const dFloat* matrix, dFloat* angles)
{
	const dgMatrix& mat = *((dgMatrix*) matrix);

	dgVector eulers (mat.CalcPitchYawRoll());
	angles[0] = eulers.m_x;
	angles[1] = eulers.m_y;
	angles[2] = eulers.m_z;
}


// Name: NewtonSetEulerAngle 
// Build a rotation matrix from the Euler angles in radians.
//
// Parameters:
// *dFloat* matrix - pointer to the 4x4 rotation matrix.
// *const dFloat* angles - pointer to an array of at least three floats to hold the Euler angles.
//
// Return: Nothing.
//
// Remarks: The motivation for this function is that many graphics engines still use Euler angles to represent the orientation
// of graphics entities. 
// The angles are expressed in radians and represent: 
// *angle[0]* - rotation about first matrix row
// *angle[1]* - rotation about second matrix row
// *angle[2]* - rotation about third matrix row
// 	
// See also: NewtonGetEulerAngle
void NewtonSetEulerAngle(const dFloat* angles, dFloat* matrix)
{
	dgInt32 i;
	dgInt32 j;

	dgMatrix mat (dgPitchMatrix (angles[0]) * dgYawMatrix(angles[1]) * dgRollMatrix(angles[2]));
	dgMatrix& retMatrix = *((dgMatrix*) matrix);

	retMatrix[3][3] = dgFloat32(1.0f);
	for (i = 0; i < 3; i ++) {
		retMatrix[3][i] = 0.0f;
		for (j = 0; j < 4; j ++) {
			retMatrix[i][j] = mat[i][j]; 
		}
	}
}


// ***************************************************************************************************************
//
// Name: Ball and Socket joint interface
//
// ***************************************************************************************************************

// Name: NewtonConstraintCreateBall
// Create a ball an socket joint. 
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const NewtonCollision* *pivotPoint - is origin of ball and socket in global space.
// *const NewtonBody* *childBody - is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
// *const NewtonBody* *parentBody - is the pointer to the parent rigid body, this body can be NULL or any kind of rigid body.
//
// Return: Pointer to the ball and socket joint.
//
// Remarks: This function creates a ball and socket and add it to the world. By default joint disables collision with the linked bodies. 
NewtonJoint* NewtonConstraintCreateBall(const NewtonWorld* newtonWorld, 
	const dFloat* pivotPoint, 
	const NewtonBody* childBody, 
	const NewtonBody* parentBody)
{
	dgBody *body0;
	dgBody *body1;
	Newton *world;

	world = (Newton *)newtonWorld;
	body0 = (dgBody *)childBody;
	body1 = (dgBody *)parentBody;
	dgVector pivot (pivotPoint[0], pivotPoint[1], pivotPoint[2]);
	return (NewtonJoint*) world->CreateBallConstraint (pivot, body0, body1);
}

// Name: NewtonBallSetConeLimits
// Set the ball and socket cone and twist limits.
//
// Parameters:
// *const NewtonJoint* *ball - is the pointer to a ball and socket joint.
// *const NewtonCollision* *pin - pointer to a unit vector defining the cone axis in global space.
// *const dFloat* maxConeAngle - max angle in radians the attached body is allow to swing relative to the pin axis, a value of zero will disable this limits. 
// *const dFloat* maxTwistAngle - max angle in radians the attached body is allow to twist relative to the pin axis, a value of zero will disable this limits. 
//
// Remarks: limits are disabled at creation time. A value of zero for *maxConeAngle* disable the cone limit, a value of zero for *maxTwistAngle* disable the twist limit
// all non-zero value for *maxConeAngle* are clamped between 5 degree and 175 degrees   
//
// See also: NewtonConstraintCreateBall
void NewtonBallSetConeLimits(const NewtonJoint* ball, const dFloat* pin, dFloat maxConeAngle, dFloat maxTwistAngle)
{
	dgBallConstraint* joint;

	joint = (dgBallConstraint*) ball;

	dgVector coneAxis (pin[0], pin[1], pin[2]); 

	if ((coneAxis % coneAxis) < 1.0e-3f) {
		coneAxis.m_x = dgFloat32(1.0f);
	}
	dgVector tmp (1.0, 0.0f, 0.0f); 
	if (dgAbsf (tmp % coneAxis) > 0.999f) {
		tmp = dgVector (0.0, dgFloat32(1.0f), 0.0f); 
		if (dgAbsf (tmp % coneAxis) > 0.999f) {
			tmp = dgVector (0.0, 0.0f, dgFloat32(1.0f)); 
			_ASSERTE (dgAbsf (tmp % coneAxis) < 0.999f);
		}
	}
	dgVector lateral (tmp * coneAxis); 
	lateral = lateral.Scale (dgRsqrt (lateral % lateral));
	coneAxis = coneAxis.Scale (dgRsqrt (coneAxis % coneAxis));

	maxConeAngle = dgAbsf (maxConeAngle);
	maxTwistAngle = dgAbsf (maxTwistAngle);
	joint->SetConeLimitState ((maxConeAngle > dgDEG2RAD) ? true : false); 
	joint->SetTwistLimitState ((maxTwistAngle > dgDEG2RAD) ? true : false);
	joint->SetLatealLimitState (false); 
	joint->SetLimits (coneAxis, -maxConeAngle, maxConeAngle, maxTwistAngle, lateral, 0.0f, 0.0f);
}


// Name: NewtonBallSetUserCallback
// Set an update call back to be called when either of the two bodies linked by the joint is active.
//
// Parameters:
// *const NewtonJoint* *ball - pointer to the joint.
// *NewtonBallCallBack* callback - pointer to the joint function call back.
//
// Return: nothing.
//
// Remarks: if the application wants to have some feedback from the joint simulation, the application can register a function
// update callback to be called every time any of the bodies linked by this joint is active. This is useful to provide special
// effects like particles, sound or even to simulate breakable moving parts.
// 
// See also: NewtonJointSetUserData
void NewtonBallSetUserCallback(const NewtonJoint* ball, NewtonBallCallBack callback)
{
	dgBallConstraint* contraint;

	contraint = (dgBallConstraint*) ball;
	contraint->SetJointParameterCallBack ((dgBallJointFriction)callback);
}


// Name: NewtonBallGetJointAngle
// Get the relative joint angle between the two bodies.
//
// Parameters:
// *const NewtonJoint* *ball - pointer to the joint.
// *dFloat* *angle - pointer to an array of a least three floats to hold the joint relative Euler angles.
//
// Return: nothing.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play a bell sound when the joint angle passes some max value.
// 
// See also: NewtonBallSetUserCallback
void NewtonBallGetJointAngle (const NewtonJoint* ball, dFloat* angle)
{
	dgBallConstraint* contraint;

	contraint = (dgBallConstraint*) ball;
	dgVector angleVector (contraint->GetJointAngle ());

	angle[0] = angleVector.m_x;
	angle[1] = angleVector.m_y;
	angle[2] = angleVector.m_z;
}

// Name: NewtonBallGetJointOmega
// Get the relative joint angular velocity between the two bodies.
//
// Parameters:
// *const NewtonJoint* *ball - pointer to the joint.
// *dFloat* *omega - pointer to an array of a least three floats to hold the joint relative angular velocity.
//
// Return: nothing.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play the creaky noise of a hanging lamp.
// 
// See also: NewtonBallSetUserCallback
void NewtonBallGetJointOmega(const NewtonJoint* ball, dFloat* omega)
{
	dgBallConstraint* contraint;

	contraint = (dgBallConstraint*) ball;
	dgVector omegaVector (contraint->GetJointOmega ());
	omega[0] = omegaVector.m_x;
	omega[1] = omegaVector.m_y;
	omega[2] = omegaVector.m_z;
}

// Name: NewtonBallGetJointForce
// Get the total force asserted over the joint pivot point, to maintain the constraint.
//
// Parameters:
// *const NewtonJoint* *ball - pointer to the joint.
// *dFloat* *force - pointer to an array of a least three floats to hold the force value of the joint.
//
// Return: nothing.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can destroy the joint if the force exceeds some predefined value.
// 
// See also: NewtonBallSetUserCallback
void NewtonBallGetJointForce(const NewtonJoint* ball, dFloat* force)
{
	dgBallConstraint* contraint;

	contraint = (dgBallConstraint*) ball;
	dgVector forceVector (contraint->GetJointForce ());
	force[0] = forceVector.m_x;
	force[1] = forceVector.m_y;
	force[2] = forceVector.m_z;
}


// ***************************************************************************************************************
//
// Name: Hinge joint interface
//
// ***************************************************************************************************************

// Name: NewtonConstraintCreateHinge
// Create a hinge joint. 
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const NewtonCollision* *pivotPoint - is origin of the hinge in global space.
// *const NewtonCollision* *pinDir - is the line of action of the hinge in global space.
// *const NewtonBody* *childBody - is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
// *const NewtonBody* *parentBody - is the pointer to the parent rigid body, this body can be NULL or any kind of rigid body.
//
// Return: Pointer to the hinge joint.
//
// Remarks: This function creates a hinge and add it to the world. By default joint disables collision with the linked bodies. 
NewtonJoint* NewtonConstraintCreateHinge(const NewtonWorld* newtonWorld, const dFloat* pivotPoint, const dFloat* pinDir, const NewtonBody* childBody, const NewtonBody* parentBody)
{
	dgBody *body0;
	dgBody *body1;
	Newton *world;

	world = (Newton *)newtonWorld;
	body0 = (dgBody *)childBody;
	body1 = (dgBody *)parentBody;
	dgVector pivot (pivotPoint[0], pivotPoint[1], pivotPoint[2]);
	dgVector pin (pinDir[0], pinDir[1], pinDir[2]);
	return (NewtonJoint*) world->CreateHingeConstraint (pivot, pin, body0, body1);
}


// Name: NewtonHingeSetUserCallback
// Set an update call back to be called when either of the two body linked by the joint is active.
//
// Parameters:
// *const NewtonJoint* *Hinge - pointer to the joint.
// *NewtonHingeCallBack* callback - pointer to the joint function call back.
//
// Return: nothing.
//
// Remarks: if the application wants to have some feedback from the joint simulation, the application can register a function
// update callback to be call every time any of the bodies linked by this joint is active. This is useful to provide special
// effects like particles, sound or even to simulate breakable moving parts.
// 
// See also: NewtonJointGetUserData, NewtonJointSetUserData
void NewtonHingeSetUserCallback(const NewtonJoint* Hinge, NewtonHingeCallBack callback)
{
	dgHingeConstraint* contraint;

	contraint = (dgHingeConstraint*) Hinge;
	contraint->SetJointParameterCallBack ((dgHingeJointAcceleration)callback);
}


// Name: NewtonHingeGetJointAngle
// Get the relative joint angle between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Hinge - pointer to the joint.
//
// Return: the joint angle relative to the hinge pin.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play a bell sound when the joint angle passes some max value.
// 
// See also: NewtonHingeSetUserCallback
dFloat NewtonHingeGetJointAngle (const NewtonJoint* Hinge)
{
	dgHingeConstraint* contraint;

	contraint = (dgHingeConstraint*) Hinge;
	return contraint->GetJointAngle ();

}

// Name: NewtonHingeGetJointOmega
// Get the relative joint angular velocity between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Hinge - pointer to the joint.
//
// Return: the joint angular velocity relative to the pin axis.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play the creaky noise of a hanging lamp.
// 
// See also: NewtonHingeSetUserCallback
dFloat NewtonHingeGetJointOmega(const NewtonJoint* Hinge)
{
	dgHingeConstraint* contraint;

	contraint = (dgHingeConstraint*) Hinge;
	return contraint->GetJointOmega ();
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
dFloat NewtonHingeCalculateStopAlpha (const NewtonJoint* hinge, const NewtonHingeSliderUpdateDesc* desc, dFloat angle)
{
	dgHingeConstraint* contraint;

	contraint = (dgHingeConstraint*) hinge;
	return contraint->CalculateStopAlpha (angle, (dgJointCallBackParam*) desc);
}

// Name: NewtonHingeGetJointForce
// Get the total force asserted over the joint pivot point, to maintain the constraint.
//
// Parameters:
// *const NewtonJoint* *Hinge - pointer to the joint.
// *dFloat* *force - pointer to an array of a least three floats to hold the force value of the joint.
//
// Return: nothing.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can destroy the joint if the force exceeds some predefined value.
// 
// See also: NewtonHingeSetUserCallback
void NewtonHingeGetJointForce(const NewtonJoint* Hinge, dFloat* force)
{
	dgHingeConstraint* contraint;

	contraint = (dgHingeConstraint*) Hinge;
	dgVector forceVector (contraint->GetJointForce ());
	force[0] = forceVector.m_x;
	force[1] = forceVector.m_y;
	force[2] = forceVector.m_z;
}

// ***************************************************************************************************************
//
// Name: Slider joint interface
//
// ***************************************************************************************************************

// Name: NewtonConstraintCreateSlider
// Create a slider joint. 
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const NewtonCollision* *pivotPoint - is origin of the slider in global space.
// *const NewtonCollision* *pinDir - is the line of action of the slider in global space.
// *const NewtonBody* *childBody - is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
// *const NewtonBody* *parentBody - is the pointer to the parent rigid body, this body can be NULL or any kind of rigid body.
//
// Return: Pointer to the slider joint.
//
// Remarks: This function creates a slider and add it to the world. By default joint disables collision with the linked bodies. 
NewtonJoint* NewtonConstraintCreateSlider(const NewtonWorld* newtonWorld, const dFloat* pivotPoint, const dFloat* pinDir, const NewtonBody* childBody, const NewtonBody* parentBody)
{
	dgBody *body0;
	dgBody *body1;
	Newton *world;

	world = (Newton *)newtonWorld;
	body0 = (dgBody *)childBody;
	body1 = (dgBody *)parentBody;
	dgVector pin (pinDir[0], pinDir[1], pinDir[2]);
	dgVector pivot (pivotPoint[0], pivotPoint[1], pivotPoint[2]);
	return (NewtonJoint*) world->CreateSlidingConstraint (pivot, pin, body0, body1);
}


// Name: NewtonSliderSetUserCallback
// Set an update call back to be called when either of the two body linked by the joint is active.
//
// Parameters:
// *const NewtonJoint* *Slider - pointer to the joint.
// *NewtonSliderCallBack* callback - pointer to the joint function call back.
//
// Return: nothing.
//
// Remarks: if the application wants to have some feedback from the joint simulation, the application can register a function
// update callback to be call every time any of the bodies linked by this joint is active. This is useful to provide special
// effects like particles, sound or even to simulate breakable moving parts.
// 
// See also: NewtonJointGetUserData, NewtonJointSetUserData
void NewtonSliderSetUserCallback(const NewtonJoint* Slider, NewtonSliderCallBack callback)
{
	dgSlidingConstraint* contraint;

	contraint = (dgSlidingConstraint*) Slider;
	contraint->SetJointParameterCallBack ((dgSlidingJointAcceleration)callback);
}

// Name: NewtonSliderGetJointPosit
// Get the relative joint angle between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Slider - pointer to the joint.
//
// Return: the joint angle relative to the hinge pin.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play a bell sound when the joint angle passes some max value.
// 
// See also: NewtonSliderSetUserCallback
dFloat NewtonSliderGetJointPosit (const NewtonJoint* Slider)
{
	dgSlidingConstraint* contraint;

	contraint = (dgSlidingConstraint*) Slider;
	return contraint->GetJointPosit ();
}

// Name: NewtonSliderGetJointVeloc
// Get the relative joint angular velocity between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Slider - pointer to the joint.
//
// Return: the joint angular velocity relative to the pin axis.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play the creaky noise of a hanging lamp.
// 
// See also: NewtonSliderSetUserCallback
dFloat NewtonSliderGetJointVeloc(const NewtonJoint* Slider)
{
	dgSlidingConstraint* contraint;

	contraint = (dgSlidingConstraint*) Slider;
	return contraint->GetJointVeloc ();
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
dFloat NewtonSliderCalculateStopAccel(const NewtonJoint* slider, const NewtonHingeSliderUpdateDesc* desc, dFloat distance)
{
	dgSlidingConstraint* contraint;

	contraint = (dgSlidingConstraint*) slider;
	return contraint->CalculateStopAccel (distance, (dgJointCallBackParam*) desc);
}

// Name: NewtonSliderGetJointForce
// Get the total force asserted over the joint pivot point, to maintain the constraint.
//
// Parameters:
// *const NewtonJoint* *Slider - pointer to the joint.
// *dFloat* *force - pointer to an array of a least three floats to hold the force value of the joint.
//
// Return: nothing.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can destroy the joint if the force exceeds some predefined value.
// 
// See also: NewtonSliderSetUserCallback
void NewtonSliderGetJointForce(const NewtonJoint* Slider, dFloat* force)
{
	dgSlidingConstraint* contraint;

	contraint = (dgSlidingConstraint*) Slider;
	dgVector forceVector (contraint->GetJointForce ());
	force[0] = forceVector.m_x;
	force[1] = forceVector.m_y;
	force[2] = forceVector.m_z;
}


// ***************************************************************************************************************
//
// Name: Corkscrew joint interface
//
// ***************************************************************************************************************

// Name: NewtonConstraintCreateCorkscrew
// Create a corkscrew joint. 
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const NewtonCollision* *pivotPoint - is origin of the corkscrew in global space.
// *const NewtonCollision* *pinDir - is the line of action of the corkscrew in global space.
// *const NewtonBody* *childBody - is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
// *const NewtonBody* *parentBody - is the pointer to the parent rigid body, this body can be NULL or any kind of rigid body.
//
// Return: Pointer to the corkscrew joint.
//
// Remarks: This function creates a corkscrew and add it to the world. By default joint disables collision with the linked bodies. 
NewtonJoint* NewtonConstraintCreateCorkscrew(const NewtonWorld* newtonWorld, const dFloat* pivotPoint, const dFloat* pinDir, const NewtonBody* childBody, const NewtonBody* parentBody)
{
	dgBody *body0;
	dgBody *body1;
	Newton *world;

	world = (Newton *)newtonWorld;
	body0 = (dgBody *)childBody;
	body1 = (dgBody *)parentBody;
	dgVector pin (pinDir[0], pinDir[1], pinDir[2]);
	dgVector pivot (pivotPoint[0], pivotPoint[1], pivotPoint[2]);
	return (NewtonJoint*) world->CreateCorkscrewConstraint (pivot, pin, body0, body1);
}

// Name: NewtonCorkscrewSetUserCallback
// Set an update call back to be called when either of the two body linked by the joint is active.
//
// Parameters:
// *const NewtonJoint* *Corkscrew - pointer to the joint.
// *NewtonCorkscrewCallBack* callback - pointer to the joint function call back.
//
// Return: nothing.
//
// Remarks: if the application wants to have some feedback from the joint simulation, the application can register a function
// update callback to be call every time any of the bodies linked by this joint is active. This is useful to provide special
// effects like particles, sound or even to simulate breakable moving parts.
//
// Remarks: the function *NewtonCorkscrewCallBack callback* should return a bit field code.
// if the application does not want to set the joint acceleration the return code is zero
// if the application only wants to change the joint linear acceleration the return code is 1
// if the application only wants to change the joint angular acceleration the return code is 2
// if the application only wants to change the joint angular and linear acceleration the return code is 3
// 
// See also: NewtonJointGetUserData, NewtonJointSetUserData
void NewtonCorkscrewSetUserCallback(const NewtonJoint* Corkscrew, NewtonCorkscrewCallBack callback)
{
	dgCorkscrewConstraint* contraint;

	contraint = (dgCorkscrewConstraint*) Corkscrew;
	contraint->SetJointParameterCallBack ((dgCorkscrewJointAcceleration)callback);
}

// Name: NewtonCorkscrewGetJointPosit
// Get the relative joint angle between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Corkscrew - pointer to the joint.
//
// Return: the joint angle relative to the hinge pin.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play a bell sound when the joint angle passes some max value.
// 
// See also: NewtonCorkscrewSetUserCallback
dFloat NewtonCorkscrewGetJointPosit (const NewtonJoint* Corkscrew)
{
	dgCorkscrewConstraint* contraint;

	contraint = (dgCorkscrewConstraint*) Corkscrew;
	return contraint->GetJointPosit ();
}

// Name: NewtonCorkscrewGetJointVeloc
// Get the relative joint angular velocity between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Corkscrew - pointer to the joint.
//
// Return: the joint angular velocity relative to the pin axis.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play the creaky noise of a hanging lamp.
// 
// See also: NewtonCorkscrewSetUserCallback
dFloat NewtonCorkscrewGetJointVeloc(const NewtonJoint* Corkscrew)
{
	dgCorkscrewConstraint* contraint;

	contraint = (dgCorkscrewConstraint*) Corkscrew;
	return contraint->GetJointVeloc ();
}

// Name: NewtonCorkscrewGetJointAngle
// Get the relative joint angle between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Corkscrew - pointer to the joint.
//
// Return: the joint angle relative to the corkscrew pin.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play a bell sound when the joint angle passes some max value.
// 
// See also: NewtonCorkscrewSetUserCallback
dFloat NewtonCorkscrewGetJointAngle (const NewtonJoint* Corkscrew)
{
	dgCorkscrewConstraint* contraint;

	contraint = (dgCorkscrewConstraint*) Corkscrew;
	return contraint->GetJointAngle ();

}

// Name: NewtonCorkscrewGetJointOmega
// Get the relative joint angular velocity between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Corkscrew - pointer to the joint.
//
// Return: the joint angular velocity relative to the pin axis.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play the creaky noise of a hanging lamp.
// 
// See also: NewtonCorkscrewSetUserCallback
dFloat NewtonCorkscrewGetJointOmega(const NewtonJoint* Corkscrew)
{
	dgCorkscrewConstraint* contraint;

	contraint = (dgCorkscrewConstraint*) Corkscrew;
	return contraint->GetJointOmega ();
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
dFloat NewtonCorkscrewCalculateStopAlpha (const NewtonJoint* corkscrew, const NewtonHingeSliderUpdateDesc* desc, dFloat angle)
{
	dgCorkscrewConstraint* contraint;

	contraint = (dgCorkscrewConstraint*) corkscrew;
	return contraint->CalculateStopAlpha (angle, (dgJointCallBackParam*) desc);
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
dFloat NewtonCorkscrewCalculateStopAccel(const NewtonJoint* corkscrew, const NewtonHingeSliderUpdateDesc* desc, dFloat distance)
{
	dgCorkscrewConstraint* contraint;
	contraint = (dgCorkscrewConstraint*) corkscrew;
	return contraint->CalculateStopAccel (distance, (dgJointCallBackParam*) desc);
}

// Name: NewtonCorkscrewGetJointForce
// Get the total force asserted over the joint pivot point, to maintain the constraint.
//
// Parameters:
// *const NewtonJoint* *Corkscrew - pointer to the joint.
// *dFloat* *force - pointer to an array of a least three floats to hold the force value of the joint.
//
// Return: nothing.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can destroy the joint if the force exceeds some predefined value.
// 
// See also: NewtonCorkscrewSetUserCallback
void NewtonCorkscrewGetJointForce(const NewtonJoint* Corkscrew, dFloat* force)
{
	dgCorkscrewConstraint* contraint;

	contraint = (dgCorkscrewConstraint*) Corkscrew;
	dgVector forceVector (contraint->GetJointForce ());
	force[0] = forceVector.m_x;
	force[1] = forceVector.m_y;
	force[2] = forceVector.m_z;
}


// ***************************************************************************************************************
//
// Name: Universal joint interface
//
// ***************************************************************************************************************

// Name: NewtonConstraintCreateUniversal
// Create a universal joint. 
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const NewtonCollision* *pivotPoint - is origin of the universal joint in global space.
// *const NewtonCollision* *pinDir0 - first axis of rotation fixed on childBody body and perpendicular to pinDir1.
// *const NewtonCollision* *pinDir1 - second axis of rotation fixed on parentBody body and perpendicular to pinDir0.
// *const NewtonBody* *childBody - is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
// *const NewtonBody* *parentBody - is the pointer to the parent rigid body, this body can be NULL or any kind of rigid body.
//
// Return: Pointer to the universal joint.
//
// Remarks: This function creates a universal joint and add it to the world. By default joint disables collision with the linked bodies. 
//
// Remark: a universal joint is a constraint that restricts twp rigid bodies to be connected to a point fixed on both bodies, 
// while and allowing one body to spin around a fix axis in is own frame, and the other body to spin around another axis fixes on 
// it own frame. Both axis must be mutually perpendicular. 
NewtonJoint* NewtonConstraintCreateUniversal(const NewtonWorld* newtonWorld, const dFloat* pivotPoint, 
	const dFloat* pinDir0, const dFloat* pinDir1, const NewtonBody* childBody, const NewtonBody* parentBody)
{
	dgBody *body0;
	dgBody *body1;
	Newton *world;

	world = (Newton *)newtonWorld;
	body0 = (dgBody *)childBody;
	body1 = (dgBody *)parentBody;
	dgVector pin0 (pinDir0[0], pinDir0[1], pinDir0[2]);
	dgVector pin1 (pinDir1[0], pinDir1[1], pinDir1[2]);
	dgVector pivot (pivotPoint[0], pivotPoint[1], pivotPoint[2]);
	return (NewtonJoint*) world->CreateUniversalConstraint (pivot, pin0, pin1, body0, body1);
}


// Name: NewtonUniversalSetUserCallback
// Set an update call back to be called when either of the two body linked by the joint is active.
//
// Parameters:
// *const NewtonJoint* *Universal - pointer to the joint.
// *NewtonUniversalCallBack* callback - pointer to the joint function call back.
//
// Return: nothing.
//
// Remarks: if the application wants to have some feedback from the joint simulation, the application can register a function
// update callback to be called every time any of the bodies linked by this joint is active. This is useful to provide special
// effects like particles, sound or even to simulate breakable moving parts.
//
// Remarks: the function *NewtonUniversalCallBack callback* should return a bit field code.
// if the application does not want to set the joint acceleration the return code is zero
// if the application only wants to change the joint linear acceleration the return code is 1
// if the application only wants to change the joint angular acceleration the return code is 2
// if the application only wants to change the joint angular and linear acceleration the return code is 3
// 
// See also: NewtonJointGetUserData, NewtonJointSetUserData
void NewtonUniversalSetUserCallback(const NewtonJoint* Universal, NewtonUniversalCallBack callback)
{
	dgUniversalConstraint* contraint;

	contraint = (dgUniversalConstraint*) Universal;
	contraint->SetJointParameterCallBack ((dgUniversalJointAcceleration)callback);
}


// Name: NewtonUniversalGetJointAngle0
// Get the relative joint angle between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Universal - pointer to the joint.
//
// Return: the joint angle relative to the universal pin0.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play a bell sound when the joint angle passes some max value.
// 
// See also: NewtonUniversalSetUserCallback
dFloat NewtonUniversalGetJointAngle0(const NewtonJoint* Universal)
{
	dgUniversalConstraint* contraint;

	contraint = (dgUniversalConstraint*) Universal;
	return contraint->GetJointAngle0 ();
}

// Name: NewtonUniversalGetJointAngle1
// Get the relative joint angle between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Universal - pointer to the joint.
//
// Return: the joint angle relative to the universal pin1.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play a bell sound when the joint angle passes some max value.
// 
// See also: NewtonUniversalSetUserCallback
dFloat NewtonUniversalGetJointAngle1(const NewtonJoint* Universal)
{
	dgUniversalConstraint* contraint;

	contraint = (dgUniversalConstraint*) Universal;
	return contraint->GetJointAngle1 ();
}


// Name: NewtonUniversalGetJointOmega0
// Get the relative joint angular velocity between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Universal - pointer to the joint.
//
// Return: the joint angular velocity relative to the pin0 axis.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play the creaky noise of a hanging lamp.
// 
// See also: NewtonUniversalSetUserCallback
dFloat NewtonUniversalGetJointOmega0(const NewtonJoint* Universal)
{
	dgUniversalConstraint* contraint;

	contraint = (dgUniversalConstraint*) Universal;
	return contraint->GetJointOmega0 ();
}


// Name: NewtonUniversalGetJointOmega1
// Get the relative joint angular velocity between the two bodies.
//
// Parameters:
// *const NewtonJoint* *Universal - pointer to the joint.
//
// Return: the joint angular velocity relative to the pin1 axis.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can play the creaky noise of a hanging lamp.
// 
// See also: NewtonUniversalSetUserCallback
dFloat NewtonUniversalGetJointOmega1(const NewtonJoint* Universal)
{
	dgUniversalConstraint* contraint;

	contraint = (dgUniversalConstraint*) Universal;
	return contraint->GetJointOmega1 ();
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
dFloat NewtonUniversalCalculateStopAlpha0(const NewtonJoint* universal, const NewtonHingeSliderUpdateDesc* desc, dFloat angle)
{
	dgUniversalConstraint* contraint;

	contraint = (dgUniversalConstraint*) universal;
	return contraint->CalculateStopAlpha0 (angle, (dgJointCallBackParam*) desc);
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
dFloat NewtonUniversalCalculateStopAlpha1(const NewtonJoint* universal, const NewtonHingeSliderUpdateDesc* desc, dFloat angle)
{
	dgUniversalConstraint* contraint;

	contraint = (dgUniversalConstraint*) universal;
	return contraint->CalculateStopAlpha1 (angle, (dgJointCallBackParam*) desc);
}



// Name: NewtonUniversalGetJointForce
// Get the total force asserted over the joint pivot point, to maintain the constraint.
//
// Parameters:
// *const NewtonJoint* *Universal - pointer to the joint.
// *dFloat* *force - pointer to an array of a least three floats to hold the force value of the joint.
//
// Return: nothing.
//
// Remarks: this function can be used during a function update call back to provide the application with some special effect.
// for example the application can destroy the joint if the force exceeds some predefined value.
// 
// See also: NewtonUniversalSetUserCallback
void NewtonUniversalGetJointForce(const NewtonJoint* Universal, dFloat* force)
{
	dgUniversalConstraint* contraint;

	contraint = (dgUniversalConstraint*) Universal;
	dgVector forceVector (contraint->GetJointForce ());
	force[0] = forceVector.m_x;
	force[1] = forceVector.m_y;
	force[2] = forceVector.m_z;
}


// ***************************************************************************************************************
//
// Name: UpVector joint Interface
//
// ***************************************************************************************************************

// Name: NewtonConstraintCreateUpVector
// Create a UpVector joint. 
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const NewtonCollision* *pinDir - is the aligning vector.
// *const NewtonBody* *body - is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
//
// Return: Pointer to the up vector joint.
//
// Remarks: This function creates an up vector joint. An up vector joint is a constraint that allows a body to translate freely in 3d space,
// but it only allows the body to rotate around the pin direction vector. This could be use by the application to control a character 
// with physics and collision.
//
// Remark: Since the UpVector joint is a unary constraint, there is not need to have user callback or user data assigned to it.
// The application can simple hold to the joint handle and update the pin on the force callback function of the rigid body owning the joint.
NewtonJoint* NewtonConstraintCreateUpVector (const NewtonWorld* newtonWorld, const dFloat* pinDir, const NewtonBody* body)
{
	dgBody *body0;
	Newton *world;

	world = (Newton *)newtonWorld;
	body0 = (dgBody *)body;
	dgVector pin (pinDir[0], pinDir[1], pinDir[2]);
	return (NewtonJoint*) world->CreateUpVectorConstraint(pin, body0);
}


// Name: NewtonUpVectorGetPin
// Get the up vector pin of this joint in global space.
//
// Parameters:
// *const NewtonJoint* *upVector - pointer to the joint.
// *dFloat* *pin - pointer to an array of a least three floats to hold the up vector direction in global space.
//
// Return: nothing.
//
// Remarks: the application ca call this function to read the up vector, this is useful to animate the up vector.
// if the application is going to animated the up vector, it must do so by applying only small rotation, 
// too large rotation can cause vibration of the joint.
// 
// See also: NewtonUpVectorSetUserCallback, NewtonUpVectorSetPin
void NewtonUpVectorGetPin(const NewtonJoint* upVector, dFloat *pin)
{
	dgUpVectorConstraint* contraint;

	contraint = (dgUpVectorConstraint*) upVector;

	dgVector pinVector (contraint ->GetPinDir ());
	pin[0] = pinVector.m_x;
	pin[1] = pinVector.m_y;
	pin[2] = pinVector.m_z;
}


// Name: NewtonUpVectorSetPin
// Set the up vector pin of this joint in global space.
//
// Parameters:
// *const NewtonJoint* *upVector - pointer to the joint.
// *dFloat* *pin - pointer to an array of a least three floats containing the up vector direction in global space.
//
// Return: nothing.
//
// Remarks: the application ca call this function to change the joint up vector, this is useful to animate the up vector.
// if the application is going to animated the up vector, it must do so by applying only small rotation, 
// too large rotation can cause vibration of the joint.
// 
// See also: NewtonUpVectorSetUserCallback, NewtonUpVectorGetPin
void NewtonUpVectorSetPin(const NewtonJoint* upVector, const dFloat *pin)
{
	dgUpVectorConstraint* contraint;

	contraint = (dgUpVectorConstraint*) upVector;

	dgVector pinVector (pin[0], pin[1], pin[2]);
	contraint->SetPinDir (pinVector);
}


// ***************************************************************************************************************
//
// Name: User defined joint interface
//
// ***************************************************************************************************************

// Name: NewtonConstraintCreateUserJoint
// Create a user define bilateral joint. 
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *in*t maxDOF - is the maximum number of degree of freedom controlled by this joint. 
// *NewtonUpVectorCallBack* callback - pointer to the joint function call back.
// *const NewtonBody* *childBody - is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
// *const NewtonBody* *parentBody - is the pointer to the parent rigid body, this body can be NULL or any kind of rigid body.
//
// Remark: Bilateral joint are constraints that can have up to 6 degree of freedoms, 3 linear and 3 angular. 
// By restricting the motion along any number of these degree of freedom a very large number of useful joint between 
// two rigid bodies can be accomplished. Some of the degree of freedoms restriction makes no sense, and also some 
// combinations are so rare that only make sense to a very specific application, the Newton engine implements the more 
// commons combinations like, hinges, ball and socket, etc. However if and application is in the situation that any of 
// the provided joints can achieve the desired effect, then the application can design it own joint.
//
// Remark: User defined joint is a very advance feature that should be look at, only for very especial situations. 
// The designer must be a person with a very good understanding of constrained dynamics, and it may be the case 
// that many trial have to be made before a good result can be accomplished.
//
// Remark: maxDOF is and upper bound as to how many degrees of freedoms the joint can control, usually this value 
// can be 6 for bilateral joints, but it can be higher for special joints like vehicles where by the used of friction clamping
// the number of rows can be higher.
// In general the application should determine maxDof correctly, passing an unnecessary excessive value will lead to performance decreased.
NewtonJoint* NewtonConstraintCreateUserJoint(const NewtonWorld* newtonWorld, int maxDOF, NewtonUserBilateralCallBack callback, const NewtonBody* childBody, const NewtonBody* parentBody)
{
	dgBody *body0;
	dgBody *body1;
	Newton *world;

	world = (Newton *)newtonWorld;
	body0 = (dgBody *)childBody;
	body1 = (dgBody *)parentBody;

	return (NewtonJoint*) new NewtonUserJoint (world, maxDOF, callback, body0, body1);
}


// Name: NewtonUserJointAddLinearRow
// Add a linear restricted degree of freedom. 
//
// Parameters:
// *const NewtonJoint* *joint - pointer to the joint.
// *const dFloat* *pivot0 - pointer of a vector in global space fixed on body zero.  
// *const dFloat* *pivot1 - pointer of a vector in global space fixed on body one.  
// *const dFloat* *pin - pointer of a unit vector in global space along which the relative position, velocity and acceleration between the bodies will be driven to zero.
//
// Remark: A linear contraint row calculates the Jacobian derivatives and relative acceleration required to enforce the constraint condition at
// the attachment point and the pin direction considered fixed to both bodies. 
//
// Remark: The acceleration is calculated such that the relative linear motion between the two points is zero, the application can
// afterward override this value to create motors.
// 
// Remark: after this function is call and internal DOF index will point to the current row entry in the constraint matrix.
//
// Remark: This function call only be called from inside a *NewtonUserBilateralCallBack* callback.
//
// See also: NewtonUserJointAddAngularRow,  
void NewtonUserJointAddLinearRow(const NewtonJoint* joint, const dFloat *pivot0, const dFloat *pivot1, const dFloat *dir)
{
	NewtonUserJoint* userJoint;
	userJoint = (NewtonUserJoint*) joint;

	dgVector direction (dir[0], dir[1], dir[2]); 
	_ASSERTE (dgAbsf (direction % direction - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
	dgVector pivotPoint0 (pivot0[0], pivot0[1], pivot0[2]); 
	dgVector pivotPoint1 (pivot1[0], pivot1[1], pivot1[2]); 
	
	userJoint->AddLinearRowJacobian (pivotPoint0, pivotPoint1, direction);
}


// Name: NewtonUserJointAddAngularRow
// Add an angular restricted degree of freedom. 
//
// Parameters:
// *const NewtonJoint* *joint - pointer to the joint.
// *dFloat* relativeAngleError - relative angle error between both bodies around pin axis.  
// *const dFloat* *pin - pointer of a unit vector in global space along which the relative position, velocity and acceleration between the bodies will be driven to zero.
//
// Remark: An angular contraint row calculates the Jacobian derivatives and relative acceleration required to enforce the constraint condition at
// pin direction considered fixed to both bodies. 
//
// Remark: The acceleration is calculated such that the relative angular motion between the two points is zero, The application can
// afterward override this value to create motors.
// 
// Remark: After this function is called and internal DOF index will point to the current row entry in the constraint matrix.
//
// Remark: This function call only be called from inside a *NewtonUserBilateralCallBack* callback.
//
// Remark: This function is of not practical to enforce hard constraints, but it is very useful for making angular motors.
//
// See also: NewtonUserJointAddLinearRow, NewtonUserJointAddIndependentAngularRow  
void NewtonUserJointAddAngularRow(const NewtonJoint* joint, dFloat relativeAngleError, const dFloat *pin)
{
	NewtonUserJoint* userJoint;
	userJoint = (NewtonUserJoint*) joint;
	dgVector direction (pin[0], pin[1], pin[2]); 
	_ASSERTE (dgAbsf (direction % direction - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));

	userJoint->AddAngularRowJacobian (direction, relativeAngleError);
}

// Name: NewtonUserJointAddGeneralRow
// set the general linear and angular Jacobian for the desired degree of freedom  
//
// Parameters:
// *const NewtonJoint* *joint - pointer to the joint.
// *const dFloat* *jacobian0 - pointer of a set of six values defining the linear and angular Jacobian for body0.
// *const dFloat* *jacobian1 - pointer of a set of six values defining the linear and angular Jacobian for body1.
//
// Remark: In general this function must be used for very special effects and in combination with other joints. 
// it is expected that the user have a knowledge of Constrained dynamics to make a good used of this function.
// Must typical application of this function are the creation of synchronization or control joints like gears, pulleys, 
// worm gear and some other mechanical control.
//
// Remark: this function set the relative acceleration for this degree of freedom to zero. It is the 
// application responsibility to set the relative acceleration after a call to this function
// 
// See also: NewtonUserJointAddLinearRow, NewtonUserJointAddAngularRow  
void NewtonUserJointAddGeneralRow(const NewtonJoint* joint, const dFloat *jacobian0, const dFloat *jacobian1)
{
	NewtonUserJoint* userJoint;
	userJoint = (NewtonUserJoint*) joint;
	userJoint->AddGeneralRowJacobian (jacobian0, jacobian1);
}


// Name: NewtonUserJointSetRowMaximumFriction
// Set the maximum friction value the solver is allow to apply to the joint row. 
//
// Parameters:
// *const NewtonJoint* *joint - pointer to the joint.
// *dFloat* friction - maximum friction value for this row. It must be a positive value between 0.0 and INFINITY.  
//
// Remark: This function will override the default friction values set after a call to NewtonUserJointAddLinearRow or NewtonUserJointAddAngularRow.
// friction value is context sensitive, if for linear constraint friction is a Max friction force, for angular constraint friction is a 
// max friction is a Max friction torque.
//
// See also: NewtonUserJointSetRowMinimumFriction, NewtonUserJointAddLinearRow, NewtonUserJointAddAngularRow  
void NewtonUserJointSetRowMaximumFriction(const NewtonJoint* joint, dFloat friction)
{
	NewtonUserJoint* userJoint;
	userJoint = (NewtonUserJoint*) joint;
	
	userJoint->SetHighFriction (friction);
}

// Name: NewtonUserJointSetRowMinimumFriction
// Set the minimum friction value the solver is allow to apply to the joint row. 
//
// Parameters:
// *const NewtonJoint* *joint - pointer to the joint.
// *dFloat* friction -  friction value for this row. It must be a negative value between 0.0 and -INFINITY.  
//
// Remark: This function will override the default friction values set after a call to NewtonUserJointAddLinearRow or NewtonUserJointAddAngularRow.
// friction value is context sensitive, if for linear constraint friction is a Min friction force, for angular constraint friction is a 
// friction is a Min friction torque.
//
// See also: NewtonUserJointSetRowMaximumFriction, NewtonUserJointAddLinearRow, NewtonUserJointAddAngularRow  
void NewtonUserJointSetRowMinimumFriction(const NewtonJoint* joint, dFloat friction)
{
	NewtonUserJoint* userJoint;
	userJoint = (NewtonUserJoint*) joint;
	
	userJoint->SetLowerFriction (friction);
}

// Name: NewtonUserJointSetRowAcceleration
// Set the value for the desired acceleration for the current constraint row. 
//
// Parameters:
// *const NewtonJoint* *joint - pointer to the joint.
// *dFloat* acceleration -  desired acceleration value for this row. 
//
// Remark: This function will override the default acceleration values set after a call to NewtonUserJointAddLinearRow or NewtonUserJointAddAngularRow.
// friction value is context sensitive, if for linear constraint acceleration is a linear acceleration, for angular constraint acceleration is an 
// angular acceleration.
//
// See also: NewtonUserJointAddLinearRow, NewtonUserJointAddAngularRow   
void NewtonUserJointSetRowAcceleration(const NewtonJoint* joint, dFloat acceleration)
{
	NewtonUserJoint* userJoint;
	userJoint = (NewtonUserJoint*) joint;
	
	userJoint->SetAcceleration (acceleration);
}

// Name: NewtonUserJointSetRowSpringDamperAcceleration
// Calculates the row acceleration to satisfy the specified the spring damper system. 
//
// Parameters:
// *const NewtonJoint* *joint - pointer to the joint.
// *dFloat* springK - desired spring stiffness, it must be a positive value.
// *dFloat* springD - desired spring damper, it must be a positive value.
//
// Remark: This function will override the default acceleration values set after a call to NewtonUserJointAddLinearRow or NewtonUserJointAddAngularRow.
// friction value is context sensitive, if for linear constraint acceleration is a linear acceleration, for angular constraint acceleration is an 
// angular acceleration.
//
// Remark: the acceleration calculated by this function represent the mass, spring system of the form
// a = -ks * x - kd * v.
//
// Remark: for this function to take place the joint stiffness must be set to a values lower than 1.0
//
// See also: NewtonUserJointSetRowAcceleration, NewtonUserJointSetRowStiffness  
void NewtonUserJointSetRowSpringDamperAcceleration(const NewtonJoint* joint, dFloat springK, dFloat springD)
{
	NewtonUserJoint* userJoint;
	userJoint = (NewtonUserJoint*) joint;
	userJoint->SetSpringDamperAcceleration (springK, springD);
}


// Name: NewtonUserJointSetRowStiffness
// Set the maximum percentage of the constraint force that will be applied to the constraint row. 
//
// Parameters:
// *const NewtonJoint* *joint - pointer to the joint.
// *dFloat* stiffness - row stiffness, it must be a values between 0.0 and 1.0, the default is 0.9. 
//
// Remark: This function will override the default stiffness value set after a call to NewtonUserJointAddLinearRow or NewtonUserJointAddAngularRow.
// the row stiffness is the percentage of the constraint force that will be applied to the rigid bodies. Ideally the value should be
// 1.0 (100% stiff) but dues to numerical integration error this could be the joint a little unstable, and lower values are preferred.
//
// See also: NewtonUserJointAddLinearRow, NewtonUserJointAddAngularRow, NewtonUserJointSetRowSpringDamperAcceleration  
void NewtonUserJointSetRowStiffness(const NewtonJoint* joint, dFloat stiffness)
{
	NewtonUserJoint* userJoint;
	userJoint = (NewtonUserJoint*) joint;
	
	userJoint->SetRowStiffness (stiffness);
}

// Name: NewtonUserJointGetRowForce
// Return the magnitude previews force or torque value calculated by the solver for this contraint row. 
//
// Parameters:
// *const NewtonJoint* *joint - pointer to the joint.
// *int*  row - index to the constraint row. 
//
// Remark: This function can be call for any of the previews row for this particular joint, The application must keep track of the meaning of the row.
// 
// Remark: This function can be used to produce special effects like breakable or malleable joints, fro example a hinge can turn into ball and sockect 
// after the force in some of the row excide certain high value. 
dFloat NewtonUserJointGetRowForce(const NewtonJoint* joint, int row)
{
	NewtonUserJoint* userJoint;
	userJoint = (NewtonUserJoint*) joint;

	return userJoint->GetRowForce (row);
}


// ***************************************************************************************************************
//
// Name: Joint common function s
//
// ***************************************************************************************************************

// Name: NewtonJointSetUserData
// Store a user defined data value with the joint.
//
// Parameters:
// *const NewtonJoint* *joint - pointer to the joint.
// *void* *userDataPtr - pointer to the user defined user data value.
//
// Return: Nothing.
//
// Remarks: The application can store a user defined value with the Joint. This value can be the pointer to a structure containing some application data for special effect. 
// if the application allocate some resource to store the user data, the application can register a joint destructor to get rid of the allocated resource when the Joint is destroyed
// 
// See also: NewtonConstraintCreateJoint, NewtonJointSetDestructor
void NewtonJointSetUserData(const NewtonJoint* joint, void* userData)
{
	dgConstraint* contraint;

	contraint = (dgConstraint*) joint;
	contraint->SetUserData (userData);
}

// Name: NewtonJointGetUserData
// Retrieve a user defined data value stored with the joint.
//
// Parameters:
// *const NewtonJoint* *joint - pointer to the joint.
//
// Return: The user defined data.
//
// Remarks: The application can store a user defined value with a joint. This value can be the pointer
// to a structure to store some game play data for special effect.
// 
// See also: NewtonJointSetUserData
void* NewtonJointGetUserData(const NewtonJoint* joint)
{
	dgConstraint* contraint;

	contraint = (dgConstraint*) joint;
	return contraint->GetUserData();
}

// Name: NewtonJointSetCollisionState
// Enable or disable collision between the two bodies linked by this joint. The default state is collision disable when the joint is created.
//
// Parameters:
// *const NewtonJoint* *joint - pointer to the joint.
// *int* state - collision state, zero mean disable collision, non zero enable collision between linked bodies.
//
// Return: nothing.
//
// Remarks: usually when two bodies are linked by a joint, the application wants collision between this two bodies to be disabled. 
// This is the default behavior of joints when they are created, however when this behavior is not desired the application can change
// it by setting collision on. If the application decides to enable collision between jointed bodies, the application should make sure the 
// collision geometry do not collide in the work space of the joint.
//
// Remarks: if the joint is destroyed the collision state of the two bodies linked by this joint is determined by the material pair assigned to each body.
// 
// See also: NewtonJointGetCollisionState, NewtonBodySetJointRecursiveCollision
void NewtonJointSetCollisionState(const NewtonJoint* joint, int state)
{
	dgConstraint* contraint;

	contraint = (dgConstraint*) joint;
	return contraint->SetCollidable (state ? true : false);
}

// Name: NewtonJointGetCollisionState
// Get the collision state of the two bodies linked by the joint.
//
// Parameters:
// *const NewtonJoint* *joint - pointer to the joint.
//
// Return: the collision state.
//
// Remarks: usually when two bodies are linked by a joint, the application wants collision between this two bodies to be disabled. 
// This is the default behavior of joints when they are created, however when this behavior is not desired the application can change
// it by setting collision on. If the application decides to enable collision between jointed bodies, the application should make sure the 
// collision geometry do not collide in the work space of the joint.
// 
// See also: NewtonJointSetCollisionState
int NewtonJointGetCollisionState(const NewtonJoint* joint)
{
	dgConstraint* contraint;

	contraint = (dgConstraint*) joint;
	return contraint->IsCollidable () ? 1 : 0;
}


// Name: NewtonJointSetStiffness
// Set the strength coeficient to be applied to the joint reaction forces.
//
// Parameters:
// *const NewtonJoint* *joint - pointer to the joint.
// *dFloat* stiffness - stiffness coeficient, a value between 0, and 1.0, the default value for most joint is 0.9
//
// Return: nothing.
//
// Remarks: Constraint keep bodies together by calculating the exact force necessary to cancel the relative acceleration between one or 
// more common points fixed in the two bodies. The problem is that when the bodies drift apart due to numerical integration inaccuracies, 
// the reaction force work to pull eliminated the error but at the expense of adding extra energy to the system, does violating the rule 
// that constraint forces must be work less. This is a inevitable situation and the only think we can do is to minimize the effect of the 
// extra energy by dampening the force by some amount. In essence the stiffness coefficient tell Newton calculate the precise reaction force 
// by only apply a fraction of it to the joint point. And value of 1.0 will apply the exact force, and a value of zero will apply only 
// 10 percent. 
//
// Remark: The stiffness is set to a all around value that work well for most situation, however the application can play with these 
// parameter to make finals adjustment. A high value will make the joint stronger but more prompt to vibration of instability; a low 
// value will make the joint more stable but weaker.  
// 
// See also: NewtonJointGetStiffness
void NewtonJointSetStiffness(const NewtonJoint* joint, dFloat stiffness)
{
	dgConstraint* contraint;

	contraint = (dgConstraint*) joint;
	contraint->SetStiffness(stiffness);
}

// Name: NewtonJointGetStiffness
// Get the strength coefficient bing applied to the joint reaction forces.
//
// Parameters:
// *const NewtonJoint* *joint - pointer to the joint.
// *dFloat* stiffness - stiffness coeficient, a value between 0, and 1.0, the default value for most joint is 0.9
//
// Return: stiffness coeficient.
//
// Remarks: Constraint keep bodies together by calculating the exact force necessary to cancel the relative acceleration between one or 
// more common points fixed in the two bodies. The problem is that when the bodies drift apart due to numerical integration inaccuracies, 
// the reaction force work to pull eliminated the error but at the expense of adding extra energy to the system, does violating the rule 
// that constraint forces must be work less. This is a inevitable situation and the only think we can do is to minimize the effect of the 
// extra energy by dampening the force by some amount. In essence the stiffness coefficient tell Newton calculate the precise reaction force 
// by only apply a fraction of it to the joint point. And value of 1.0 will apply the exact force, and a value of zero will apply only 
// 10 percent. 
//
// Remark: The stiffness is set to a all around value that work well for most situation, however the application can play with these 
// parameter to make finals adjustment. A high value will make the joint stronger but more prompt to vibration of instability; a low 
// value will make the joint more stable but weaker.  
// 
// See also: NewtonJointSetStiffness
dFloat NewtonJointGetStiffness(const NewtonJoint* joint)
{
	dgConstraint* contraint;

	contraint = (dgConstraint*) joint;
	return contraint->GetStiffness();
}


// Name: NewtonJointSetDestructor
// Register a destructor callback to be called when the joint is about to be destroyed.
//
// Parameters:
// *const NewtonJoint* *joint - pointer to the joint.
// *NewtonJointCallBack* destructor - pointer to the joint destructor callback.
//
// Return: nothing.
//
// Remarks: If application stores any resource with the joint, or the application wants to be notified when the
// joint is about to be destroyed. The application can register a destructor call back with the joint.
// 
// See also: NewtonJointSetUserData
void NewtonJointSetDestructor(const NewtonJoint* joint, NewtonConstraintDestructor destructor)
{
	dgConstraint* contraint;

	contraint = (dgConstraint*) joint;
	contraint->SetDestructorCallback ((OnConstraintDestroy) destructor);
}


// Name: NewtonDestroyJoint 
// destroy a joint.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the body.
// *const NewtonJoint* *joint - pointer to joint to be destroyed
//
// Return: nothing
// 
// Remarks: The application can call this function when it wants to destroy a joint. This function can be used by the application to simulate
// breakable joints
//
// See also: NewtonConstraintCreateJoint, NewtonConstraintCreateHinge, NewtonConstraintCreateSlider
void NewtonDestroyJoint(const NewtonWorld* newtonWorld, const NewtonJoint* joint)
{
	Newton *world;

	world = (Newton *) newtonWorld;
	world->DestroyJoint ((dgConstraint*) joint);
}


// ***************************************************************************************************************
//
// Name: Rag doll joint container Interface
//
// ***************************************************************************************************************

// Name: NewtonCreateRagDoll
// Create an empty rag doll container.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
//
// Return: handle to an empty rag doll container.
//
// Remarks: A rag doll container is the encapsulation of a group of ball and socket joints, under a common object. It provides common functional 
// shared by all joints in the tree like structure. The rag doll object job is to simplify some of the common task that the programmer encounters 
// when it tries to make a rag doll model by connecting joints.
// It also has some limitations, for example the hierarchy of joints is made of Ball and socket joints, it only support tree like structures of 
// joints, the joints can not by detached from the array once they are added to the rag doll.
// Rag doll joints are good to make articulated creatures, like humans, monsters, horses, etc. 
// They are good to simulate effects like death of a character in a game.
//
// See also: NewtonDestroyRagDoll
NewtonRagDoll* NewtonCreateRagDoll(const NewtonWorld* newtonWorld)
{
	Newton *world;

	world = (Newton *)newtonWorld;
	return (NewtonRagDoll*) world->RagDollList::Create(world);
}

// Name: NewtonDestroyRagDoll
// Destroy a ragdoll containers and all it components.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const NewtonRagDoll* *ragDoll - handle to a rag doll container.
//
// Return: nothing
//
// See also: NewtonCreateRagDoll
void NewtonDestroyRagDoll(const NewtonWorld* newtonWorld, const NewtonRagDoll* ragDoll)
{
	Newton *world;

	world = (Newton *)newtonWorld;
	world->RagDollList::Destroy ((RagdollHeader*) ragDoll);
}

// Name: NewtonRagDollBegin
// Prepare a rag doll object to accept bones and making the skeleton.
//
// Parameters:
// *const NewtonRagDoll* *ragDoll - handle to a rag doll container.
//
// Return: nothing
//
// Remarks: This function will clean up the rag doll container of any previously 
// added bone. After the constructions process of the rag doll is completed the application should call  
// *NewtonRagDollEnd*, not doing so will result in unpredictable results.
//
// See also: NewtonRagDollEnd, NewtonRagDollAddBone, SetExtForceAndTorqueCallback, SetTransformCallback
void NewtonRagDollBegin(const NewtonRagDoll* ragDoll)
{
	RagdollHeader* ragDollHeader;

	ragDollHeader = (RagdollHeader*) ragDoll;
	ragDollHeader->Begin();
}

// Name: NewtonRagDollEnd
// Finalized the rag doll construction process.
//
// Parameters:
// *const NewtonRagDoll* *ragDoll - handle to a rag doll container.
//
// Return: nothing
//
// Remarks: This function should be called after the constructions process of the rag doll in completed. 
// calling this function without first calling *NewtonRagDollBegin* will produce unpredictable results.
//
// See also: NewtonRagDollBegin 
void NewtonRagDollEnd(const NewtonRagDoll* ragDoll)
{
	RagdollHeader* ragDollHeader;

	ragDollHeader = (RagdollHeader*) ragDoll;
	ragDollHeader->End();
}


// Name: NewtonRagDollAddBone
// Add a bone to a rag doll objects.
//
// Parameters:
// *const NewtonRagDoll* *ragDoll - handle to a rag doll container.
// *const NewtonRagDollBone* *parent - handle to the bone parent for this bone. If this is the root bone the *parent* should be NULL.
// *void* *userData - user data value. The application can use this parameter to store the pointer on index into the graphical part representing this bone.
// *dFloat* mass - mass of this body part.
// *const dFloat* *matrix - pointer to a 4x4 transformation matrix in the local space of the bone. The matrix should be expressed relative to the parent bone. 
// *const NewtonCollision* *boneCollision - bone collision geometry.
// *const dFloat* size - pointer to an array of three dFloat specifying the size of this body part. The first component of the array is interpreted as the length of the bone.
//
// Return: the handle to a Rag doll bone
//
// Remarks: The user data value of a rag doll rigid body is set to the *NewtonRagDollBone*, Unpredictable result will happens if the application set the 
// user data of a rag doll node rigid body.
//
// Remarks: this function can only be called from inside of a NewtonRagDollBegin/NewtonRagDollEnd pair.
//
// See also: NewtonRagDollBegin 
NewtonRagDollBone* NewtonRagDollAddBone(const NewtonRagDoll* ragDoll, const NewtonRagDollBone* parent, void *userData, 
   dFloat mass, const dFloat* matrix, const NewtonCollision* boneCollision, const dFloat* size)
{
	dgCollision *collision;
	RagdollHeader* ragDollHeader;

	ragDollHeader = (RagdollHeader*) ragDoll;

	dgMatrix boneMatrix (*(dgMatrix*) matrix);
	dgVector boneSize (size[0], size[1], size[2]); 
	collision = (dgCollision *)boneCollision;

	return (NewtonRagDollBone*) ragDollHeader->AddBone ((RagdollBone*) parent, userData, mass, boneMatrix, collision, boneSize);
}

// Name: NewtonRagDollBoneGetUserData
// Retrieve a user defined data value stored with the rag doll bone.
//
// Parameters:
// *const NewtonRagDollBone* *bone - pointer to the bone.
//
// Return: The user defined data.
//
// Remarks: The application can store a user defined value with a rigid body. This value can be the pointer
// to a structure which is the graphical representation of the bone.
// 
// See also: NewtonRagDollAddBone
void* NewtonRagDollBoneGetUserData(const NewtonRagDollBone* bone)
{
	RagdollBone* dollBone;

	dollBone = (RagdollBone*) bone;
	return dollBone->GetUserData();
}

// Name: NewtonRagDollSetTransformCallback 
// Assign a transformation event function to a rag doll object.
//
// Parameters:
// *const NewtonRagDoll* *ragDoll - handle to a rag doll container.
// *NewtonSetRagDollTransform callback - pointer to a function callback is used to update the transformation matrix of the visual object that represents the rag doll.
//
// Return: Nothing.
//
// Remarks: The function *NewtonSetRagDollTransform callback* is called by the Newton engine every time a visual object that represent a bone of a rag doll has changed.
// The application can obtain the pointer user data value that points to the visual object. 
// The Newton engine does not call the *NewtonSetRagDollTransform callback* function for bones that are inactive or have reached a state of stable equilibrium.
//
// Remarks: The user data value of a rag doll rigid body is set to the *NewtonRagDollBone*, Unpredictable result will happens if the application set the 
// user data of a rag doll node rigid body.
// The application can get the pointer to the application graphical data by retrieving the user data stored with the *NewtonRagDollBone*. 
//
// Remarks: The matrix should be organized in row-major order (this is the way directX stores matrices).
// If you are using OpenGL matrices (column-major) you will need to transpose the matrices into a local array, before
// you pass them to Newton.
//
// Remarks: this function can only be called from inside of a NewtonRagDollBegin/NewtonRagDollEnd pair.
// 
// See also: NewtonRagDollBegin, NewtonRagDollAddBone 
void NewtonRagDollSetTransformCallback(const NewtonRagDoll* ragDoll, NewtonSetRagDollTransform callback)
{
	RagdollHeader* ragDollHeader;

	ragDollHeader = (RagdollHeader*) ragDoll;
	ragDollHeader->SetTransformCallback (callback);
}

// Name: NewtonRagDollSetForceAndTorqueCallback 
// Assign an event function for applying external force and torque to a Rag doll.
//
// Parameters:
// *const NewtonRagDoll* *ragDoll - handle to a rag doll container.
// *NewtonApplyForceAndTorque callback - pointer to a function callback used to apply force and torque to a rigid body bone.
//
// Return: Nothing.
//
// Remarks: this function can be seen as a utility function that will call *NewtonBodySetForceAndTorqueCallback* for every bone of a rag doll object.
//
// Remarks: The user data value of a rag doll rigid body is set to the *NewtonRagDollBone*, unpredictable result will happens if the application set the 
// user data of a rag doll node rigid body. 
// The application can get the pointer to the application graphical data by retrieving the user data stored with the *NewtonRagDollBone*. 
//
// Remarks: Before the *NewtonApplyForceAndTorque callback* is called for a body, Newton first clears the net force and net torque for the body. 
//
// Remarks: The function *NewtonApplyForceAndTorque callback* is called by the Newton Engine every time an active body is going to be simulated. 
// The Newton Engine does not call the *NewtonApplyForceAndTorque callback* function for bodies that are inactive or have reached a state of stable equilibrium.
// 
// See also: NewtonRagDollBegin, NewtonRagDollAddBone 
void NewtonRagDollSetForceAndTorqueCallback(const NewtonRagDoll* ragDoll, NewtonApplyForceAndTorque callback)
{
	RagdollHeader* ragDollHeader;

	ragDollHeader = (RagdollHeader*) ragDoll;
	ragDollHeader->SetExtForceAndTorqueCallback ((OnApplyExtForceAndTorque) callback);
}




// Name: NewtonRagDollBoneSetID 
// Set an id for this particular bone.
//
// Parameters:
// *const NewtonRagDollBone* *bone - handle to the bone.
// *int* id - identifier for this bone.
//
// Return: Nothing.
//
// Remarks: during the construction, the application has the option to set an identifier for each bone. It is good idea to make this identifier unique for the rag doll.
// 
// See also: NewtonRagDollAddBone, NewtonRagDollFindBone 
void NewtonRagDollBoneSetID (const NewtonRagDollBone* bone, int id)
{
	RagdollBone* dollBone;

	dollBone = (RagdollBone*) bone;
	dollBone->SetNameID (id);
}

// Name: NewtonRagDollFindBone 
// Find the first bone with this id in this rag doll.
//
// Parameters:
// *const NewtonRagDoll* *ragDoll - handle to a rag doll container.
// *int* id - identifier for this bone.
//
// Return: the handle to the bone with this identifier. NULL if no bone in the rag doll has this id
//
// Remarks: during the construction, the application has the option to set an identifier for each bone. It is good idea to make this identifier unique for the rag doll.
// the application can use this id to find particular bones in the rag doll body. This is useful for authoring tolls.
// 
// See also: NewtonRagDollBoneSetID 
NewtonRagDollBone* NewtonRagDollFindBone(const NewtonRagDoll* ragDoll, int id)
{
	RagdollHeader* ragDollHeader;
	ragDollHeader = (RagdollHeader*) ragDoll;
	return (NewtonRagDollBone*) ragDollHeader->FindBone (id);
}


// Name: NewtonRagDollBoneGetBody 
// Retrieve the rigid body assigned to this bone.
//
// Parameters:
// *const NewtonRagDollBone* *bone - handle to the bone.
//
// Return: The rigid body assigned to this bone
//
// Remarks: this function can be used to customized some of the properties of the rigid body assigned to the bone.
// the application should not override the pointer to *TransformCallback* or *ApplyForceAndTorque* of a rigid body assigned 
// to a Rag doll bone. It should call the functions *NewtonRagDollSetTransformCallback* and *NewtonRagDollSetForceAndTorqueCallback* instead.
// 
// See also: NewtonRagDollAddBone, NewtonRagDollSetTransformCallback, NewtonRagDollSetForceAndTorqueCallback
NewtonBody* NewtonRagDollBoneGetBody(const NewtonRagDollBone* bone)
{
	RagdollBone* dollBone;

	dollBone = (RagdollBone*) bone;
	return (NewtonBody*) dollBone->m_body;
}

// Name: NewtonRagDollBoneSetLimits
// Set the stop limits for this bone.
//
// Parameters:
// *const NewtonRagDollBone* *bone - handle to this particular bone. If this parameter is the root bone, then the limits do not have any effect.
// *const dFloat* *coneDir - pointer to an array of tree dFloat specifying the direction in global space of the cone limits for this bone.
// *dFloat* minConeAngle - minimum value of the cone of the cone limit. 
// *dFloat* maxConeAngle - maximum value of the cone of the cone limit. 
// *dFloat* maxTwistAngle - maximum and minimum that this bone is allow to spin around the coneDir.
// *const dFloat* *lateralConeDir - this parameter is ignored in this release.
// *dFloat* negativeBilateralConeAngle - this parameter is ignored in this release.
// *dFloat* negativeBilateralConeAngle - this parameter is ignored in this release.
//
//
// Remarks: This function set a cone fixed on the frame of the parent of this bone and defining the work space of the bone.
//
// See also: NewtonRagDollBegin 
void NewtonRagDollBoneSetLimits(const NewtonRagDollBone* bone, const dFloat* coneDir, dFloat minConeAngle, dFloat maxConeAngle, dFloat maxTwistAngle, const dFloat* lateralConeDir, dFloat negativeBilateralConeAngle, dFloat positiveBilateralConeAngle)
{
	RagdollBone* dollBone;

	dollBone = (RagdollBone*) bone;

	dgVector coneAxis (coneDir[0], coneDir[1], coneDir[2]); 
//	dgVector lateralAxis (lateralConeDir[0], lateralConeDir[1], lateralConeDir[2]); 
	dgVector lateralAxis (dgFloat32(1.0f), 0.0f, 0.0f); 
	dollBone->SetJointLimits (coneDir, minConeAngle, maxConeAngle, maxTwistAngle, lateralAxis, negativeBilateralConeAngle, positiveBilateralConeAngle);
}
	

// Name: NewtonRagDollBoneGetLocalMatrix
// Get the transformation matrix for this bone in the local space of the bone.
//
// Parameters:
// *const NewtonRagDollBone* *bone - handle to this particular bone. If this parameter is the root bone, then the limits do not have any effect.
// *dFloat* *matrix - pointer to a 4x4 transformation matrix to receive the transformation matrix for this bone. 
//
// Remarks: the application can call this function from a NewtonSetRagDollTransform to get the transformation matrix for the graphical representation of the bone in local space.
//
// See also: NewtonRagDollSetTransformCallback 
void NewtonRagDollBoneGetLocalMatrix(const NewtonRagDollBone* bone, dFloat* matrixPtr)
{
	RagdollBone* dollBone;

	dgMatrix& matrix (*((dgMatrix*) matrixPtr));
	dollBone = (RagdollBone*) bone;
	matrix = dollBone->GetBoneLocalMatrix ();
}

// Name: NewtonRagDollBoneGetGlobalMatrix
// Get the transformation matrix for this bone in the global space of the bone.
//
// Parameters:
// *const NewtonRagDollBone* *bone - handle to this particular bone. If this parameter is the root bone, then the limits do not have any effect.
// *dFloat* *matrix - pointer to a 4x4 transformation matrix to receive the transformation matrix for this bone. 
//
// Remarks: the application can call this function from a NewtonSetRagDollTransform to get the transformation matrix for the graphical representation of the bone in global space.
//
// See also: NewtonRagDollSetTransformCallback 
void NewtonRagDollBoneGetGlobalMatrix(const NewtonRagDollBone* bone, dFloat* matrixPtr)
{
	RagdollBone* dollBone;

	dgMatrix& matrix (*((dgMatrix*) matrixPtr));
	dollBone = (RagdollBone*) bone;
	matrix = dollBone->GetBoneMatrix ();
}

//void NewtonRagDollSetFriction (const NewtonRagDoll* ragDoll, dFloat friction)
//{
	//RagdollHeader* ragDollHeader;
	//ragDollHeader = (RagdollHeader*) ragDoll;
	//ragDollHeader->SetFriction(friction);
//}



// ***************************************************************************************************************
//
// Name: Vehicle constraint interface
//
// ***************************************************************************************************************

// Name: NewtonConstraintCreateVehicle
// Create and empty vehicle joint.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const dFloat* *upDir - is the unit vector defined by the tires pivot points, usually the opposite direction to the gravity vector.
// *const NewtonBody* *body - is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
//
// Return: Pointer to the vehicle joint.
//
// Remarks: This joint provides basics functionality for simulating real-time simplistic vehicle dynamics. The joint is not meant to be and 
// accurate and realistic representation of a real vehicle, that is out of the scope of a real-time physics engine. The vehicle is made out 
// of a main rigid body visible to the application and attached to it a set of tires not visible to the application directly as rigid bodies.
// The tires are connected to the body via rigid wheel joints providing the ability for the tire to spin, have suspension, and turn. The 
// internal vehicle mechanics like transmission, power transfer shaft, suspension mechanism, doors etc. It only models the vehicle body 
// mounted on a set of wheels, with suspension and the ability to roll. 
NewtonJoint* NewtonConstraintCreateVehicle(const NewtonWorld* newtonWorld, const dFloat* upDir, const NewtonBody* body)
{
	Newton *world;
	dgBody *carBody;

	carBody = (dgBody *)body;
	world = (Newton *)newtonWorld;
	dgVector pin (upDir[0], upDir[1], upDir[2]);
	return (NewtonJoint*) world->CreateVehicleContraint(pin, carBody);
}

// Name: NewtonVehicleSetTireCallback
// Store a function callback pointer for vehicle update.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *NewtonVehicleTireUpdate* update - pointer to the callback function.
//
// Return: nothing
//
// Remarks: The only way to control a vehicle is by implementing the vehicle update callback. 
// The application should iterate through each tire applying tire dynamics to each one. 
//
// See also: NewtonVehicleGetFirstTireID, NewtonVehicleGetNextTireID
void NewtonVehicleSetTireCallback(const NewtonJoint* vehicle, NewtonVehicleTireUpdate update)
{
	dgVehicleConstraint* joint;
	
	joint = (dgVehicleConstraint*)vehicle;
	joint->SetTireCallback ((OnVehicleUpdate) update);
}


// Name: NewtonVehicleAddTire
// Add a new tire to the vehicle container.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *const dFloat* *localMatrix - pointer to an array of 16 floats containing the offset of the tire relative to the vehicle body.
// *const dFloat* pin - pointer to an array of 3 floats containing the rotation axis of the tire, in the space of the tire.
// *dFloat* mass - tire mass, must be much smaller than the vehicle body. ratio of 50:1 to 100:1 are the recommended values. 
// *dFloat* width - width of the tire, must be smaller than the tire radius.
// *dFloat* radius - tire radius.
// *dFloat* suspesionShock - parametrized damping constant for a spring, mass, damper system. A value of one corresponds to a critically damped system.
// *dFloat* suspesionSpring - parametrized spring constant for a spring, mass, damper system. A value of one corresponds to a critically damped system.
// *dFloat* suspesionLength - distance from the tire set position to the upper stop on the vehicle body frame. The total suspension length is twice that. 
// *void* *userData - pointer to a user define data value. Usually used to store the pointer to the graphical representation of the tire.
// *int* collisionID - the collision ID use by the application to identify the tire contacts in a contact callback function.
//
// Return: the tire ID.
//
// Remarks: After the application creates the vehicle joint, it must add the tires. 
// Tires are added one at a time at the graphics set position and with the appropriate.
// the application should calculate the correct tire parameters, like tire mass, position, width height,
// spring and damper constants.
//
// See also: NewtonVehicleRemoveTire
void* NewtonVehicleAddTire(const NewtonJoint* vehicle, const dFloat* localMatrix, const dFloat* pin, dFloat mass, dFloat width, dFloat radius, 
	dFloat suspesionShock, dFloat suspesionSpring, dFloat suspesionLength, void* userData, int collisionID)
{
	dgVehicleConstraint* joint;

	joint = (dgVehicleConstraint*)vehicle;

	dgMatrix& matrix (*((dgMatrix*) localMatrix));
	dgVector tirePin (pin[0], pin[1], pin[2]);
	//return joint->AddTire (matrix, pin, mass, width, radius, suspesionShock, suspesionSpring, suspesionLength, (void*)(dgUnsigned64(collisionID)), userData);
	return joint->AddTire (matrix, pin, mass, width, radius, suspesionShock, suspesionSpring, suspesionLength, IntToPointer(collisionID), userData);
}

// Name: NewtonVehicleReset
// Reset all tires velocities to zero. 
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
//
// Return: nothing.
//
// This function is useful for reposition the vehicle.
void NewtonVehicleReset(const NewtonJoint* vehicle)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->Reset();
}


// Name: NewtonVehicleRemoveTire
// Detach and destroy a tire from the vehicle.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - tire index to be destroyed.
//
// Return: nothing.
void NewtonVehicleRemoveTire(const NewtonJoint* vehicle, void* tireId)
{
	dgVehicleConstraint* joint;

	joint = (dgVehicleConstraint*)vehicle;
	joint->RemoveTire (tireId);
}



// Name: NewtonVehicleGetFirstTireID
// Get the index of the first tire of the vehicle tire set.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
//
// Return: tire index.
//
// Remarks: This function is usually used from inside the vehicle update callback. It is used to iterate through the tire set applying the tire dynamics.
//
// See also: NewtonVehicleGetNextTireID
void* NewtonVehicleGetFirstTireID(const NewtonJoint* vehicle)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->GetFirstTireIndex();
}

// Name: NewtonVehicleGetNextTireID
// Get the index of the next tire on the tire set.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: Next tire index, or zero if index *tireId* was pointing to the last tire in the set.
//
// Remarks: This function is usually used from inside a tire update callback. It is used to iterate through the tire set applying the tire dynamics.
//
// See also: NewtonVehicleGetFirstTireID
void* NewtonVehicleGetNextTireID(const NewtonJoint* vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;
	return joint->GetNextTireIndex(tireId);
}

// Name: NewtonVehicleGetTireUserData
// Retrieve the pointer to the tire user data.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: tire user data.
//
// Remarks: This function is usually used from the vehicle update callback or from transformation callback of the vehicle body, 
// It can used do set the transformation matrix of the tire graphical representation.
void* NewtonVehicleGetTireUserData (const NewtonJoint* vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;
	return joint->GetTireUserData (tireId);
}

// Name: NewtonVehicleGetTireMatrix
// Retrieve the transformation matrix of the tire in global space.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
// *dFloat* matrix - pointer to an array of 16 floats containing the global matrix of the tire.
//
// Return: nothing
//
// Remarks: This function is usually used from the tire update callback or from transformation callback of the vehicle body, 
// It can be used to set the transformation of the tire graphical representation.
void NewtonVehicleGetTireMatrix(const NewtonJoint* vehicle, void* tireId, dFloat* matrix)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	dgMatrix& retMatrix = *((dgMatrix*) matrix);
	joint->GetTireMatrix (tireId, retMatrix);
}

// Name: NewtonVehicleGetTireSteerAngle
// Get the tire steering angle.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: steering angle.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application wants to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
dFloat NewtonVehicleGetTireSteerAngle(const NewtonJoint* vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->GetSteerAngle(tireId);
}


// Name: NewtonVehicleSetTireSteerAngle
// Set the tire steering angle.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
// *dFloat* angle - new steering angle.
//
// Return: nothing.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application wants to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
void NewtonVehicleSetTireSteerAngle(const NewtonJoint* vehicle, void* tireId, dFloat angle)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->SetSteerAngle(tireId, angle);
}


// Name: NewtonVehicleSetTireTorque
// Set the tire torque.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
// *dFloat* torque - new torque value.
//
// Return: nothing.
//
// Remarks: This function is useful to simulate normal vehicles with wheels that propel by applying torque to a the tire axis
// in order to move.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application wants to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
void NewtonVehicleSetTireTorque(const NewtonJoint* vehicle, void* tireId, dFloat torque)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->SetTireTorque(tireId, torque);
}

// Name: NewtonVehicleGetTireOmega
// Retrieve the tire angular velocity.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: angular velocity.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application wants to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
dFloat NewtonVehicleGetTireOmega(const NewtonJoint* vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->GetTireOmega(tireId);
}

// Name: NewtonVehicleGetTireNormalLoad
// Return the part of the vehicle weight supported by this tire.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: magnitude of the vehicle weight supported by this tire.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application wants to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
dFloat NewtonVehicleGetTireNormalLoad(const NewtonJoint* vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->GetTireNormalLoad(tireId);
}

// Name: NewtonVehicleTireSetBrakeAcceleration
// Apply the acceleration and max friction torque to a tire axis.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
// *dFloat* acceleration - desire tire acceleration.
// *dFloat* maxFrictionTorque - maximum friction torque the tire brake, or tire motor can withstand.
//
// Return: noting.
//
// Remarks: This is a multipurpose function. The more common use is to apply hand or soft brakes to a vehicle. 
// To apply brakes the application may use the function *NewtonVehicleTireSetBrakeAcceleration* to determine the exact acceleration
// needed to stop the tire from continue to spin in one frame. To simulated the variable brakes strength the application can use 
// a nominal maximum friction torque (just like in real life any device will withstand a max value) and modulate this value with an analog 
// control. For hand brakes the application set the control to the maximum and for soft brakes it can just modulate the variable friction.
// Another use for this function is to simulate rolling friction, For this effect the application apply the acceleration to stop
// but with a friction value set to a minimum non zero fixed value. Note that brakes and tire torque are not mutually exclusive, 
// the application can apply then simultaneously. As a matter of fact doing so is quite a satisfying test showing how the vehicles rocks 
// forth and back due to the engine torque, while the tire prevent it from moving. Another use for this function is the simulation of 
// track based vehicles. For this the application apply an arbitrary fix acceleration to 
// the tires on each side of the vehicle. A function as simple as *A = Ad minus Ks x Omega* can do the trick, where Ad is the desire acceleration 
// controlled by the application joystick, Ks is some viscous velocity damping, and omega is the current tire angular velocity reported by the 
// function *NewtonVehicleGetTireOmega*. 
// To make the vehicle take turns the application can elaborate the equation like *A = Ad + At minus Ks x Omega* where At is the differential 
// acceleration supplied by the steering control, for the tires on the right side At is positive while for tires of the left side At is negative.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application wants to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
//
// See also: NewtonVehicleTireCalculateMaxBrakeAcceleration, NewtonVehicleGetTireOmega
void NewtonVehicleTireSetBrakeAcceleration(const NewtonJoint* vehicle, void* tireId, dFloat acceleration, dFloat maxFrictionTorque)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->TireSetBrakeAcceleration(tireId, acceleration, maxFrictionTorque);
}

// Name: NewtonVehicleTireCalculateMaxBrakeAcceleration
// Calculate the exact acceleration needed to be applied to a tire axis in order to bring it to full stop in one time step.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: exact acceleration for full stop of the tire.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
//
// See also: NewtonVehicleTireSetBrakeAcceleration
dFloat NewtonVehicleTireCalculateMaxBrakeAcceleration(const NewtonJoint* vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->TireCalculateMaxBrakeAcceleration(tireId);
}


// Name: NewtonVehicleGetTireLateralSpeed
// Return the tire speed along the tire pin axis.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: tire lateral speed.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
dFloat NewtonVehicleGetTireLateralSpeed(const NewtonJoint* vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->GetTireLateralSpeed(tireId);
}

// Name: NewtonVehicleGetTireLongitudinalSpeed
// Return the tire speed along tire center line.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: tire longitudinal speed.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
dFloat NewtonVehicleGetTireLongitudinalSpeed(const NewtonJoint* vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->GetTireLongitudinalSpeed(tireId);
}


// Name: NewtonVehicleSetTireMaxSideSleepSpeed
// Set the maximum side slip velocity for the tire to be considered to lose grip.
// 
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
// *dFloat* speed - maximum side speed before the vehicle is considered to loose side grip.
//
// Return: nothing.
//
// Remarks: Tire operation involve a mix of elastic distortion and sliding friction. To have and idea how to code a convincing 
// approximation of a real tire we must run some experiment and make some reflection upon the result. First we will run a static 
// test: keeping a tire at equilibrium applying a constant load and without rolling, we will apply a lateral force perpendicular 
// to the tire plane and applied at the tire center. If we run this experiment we will find that the tire will deflect in response 
// to the lateral force. If we increase the magnitude of the lateral force, the magnitude of the deflection is proportional to the 
// magnitude of the lateral force, until the tire begins to slide. This show that when a tire is not moving it behaves like a spring 
// (elastic distortion). If we repeat this experiment but this time increasing the tire load, but still not moving the tire, we will 
// see that the max deflection is proportional to the magnitude of the tire load. This indicates the tire behavior is proportional 
// to two variables, the lateral force and the tire load. (Fortunately the side force in practice is a linear function of the tire 
// load so this keeps the model simple) Now we will run the first experiment but this time we will rotate the tire with a constant 
// angular velocity (think of those tune up machines at check up stations.) With the tire rolling at constant angular velocity if we 
// apply a lateral force we will see that as the tire deflect, the part of the tire in contact with the floor keeps rolling and another
// part take its place, but this part also start to deflect, allowing the tire to move sideways with a velocity proportional to the 
// tire rolling angular velocity. Notice that the tire does this without sliding as the part of it in contact with the floor never 
// loses grip. Now if we increase the lateral force we will find that the lateral speed of the tire will also increase. This suggests 
// that the side speed of the tire is proportional to the lateral force and also proportional to the rolling angular velocity. This 
// is the tire elastic properties give then some kind of damping property when they are rolling. There is not known macroscopic 
// mathematical model that can explain this behavior. The best we can do is to write the values of the experiment and use then to 
// interpolate and extrapolate intermediate values. One thing we know is that the tires operates within some limits, and we can use 
// those parameters to treat then as a constraint optimization problem, which is the Newton approach. When the tire is rolling and 
// side slipping is not that the tire lost grip, nor that the tire is generating some force. It is rather that the tire have the 
// capacity to absorb some of the lateral force by sliding and convert it to side velocity, this means that for the tire to 
// loose grip a stronger force is necessary. In another word at rest a tire will lose grip under a much lower lateral force than 
// if the tire was rolling. In Newton this behavior is treaded as a constrain optimization problem by asking the application how 
// much side slip velocity is the tire allow to have before it is considered to lose grip, and how much of the lateral forces 
// generated by the rigid body dynamics will be adsorbed by the tire at a given speed. It is the application responsibility to 
// set these parameters as close to the real tire as it chooses. This approach allows for a very wide range of behaviors form arcade, 
// to toy cars to very realistic.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
//
// See also: NewtonVehicleSetTireSideSleepCoeficient
void NewtonVehicleSetTireMaxSideSleepSpeed(const NewtonJoint* vehicle, void* tireId, dFloat speed)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->SetTireMaxSideSleepSpeed(tireId, speed);
}

// Name: NewtonVehicleSetTireSideSleepCoeficient
// Set the coefficient that tell the engine how much of the lateral force can be absorbed by the tire.
// 
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
// *dFloat* coeficient - side slip coeficient.
//
// Return: nothing.
//
// Remarks: See description of side slip on function *NewtonVehicleSetTireMaxSideSleepSpeed*
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
//
// See also: NewtonVehicleSetTireMaxSideSleepSpeed
void NewtonVehicleSetTireSideSleepCoeficient(const NewtonJoint* vehicle, void* tireId, dFloat coeficient)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->SetTireSideSleepCoeficient(tireId, coeficient);
}


// Name: NewtonVehicleSetTireMaxLongitudinalSlideSpeed
// Set the maximum side slide velocity for the tire to be considered to lose traction.
// 
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
// *dFloat* speed - maximum side speed before the vehicle is considered to loose side traction.
//
// Return: nothing.
//
// Remarks: The explanation of longitudinal slide is similar to the side slip, however it is not so critical to achieve realistic behavior.
// See description of side slip on function *NewtonVehicleSetTireMaxSideSleepSpeed*
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
//
// See also: NewtonVehicleSetTireLongitudinalSlideCoeficient
void NewtonVehicleSetTireMaxLongitudinalSlideSpeed(const NewtonJoint* vehicle, void* tireId, dFloat speed)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->SetTireMaxLongitudinalSlideSpeed(tireId, speed);
}

// Name: NewtonVehicleSetTireLongitudinalSlideCoeficient
// Set the coefficient that tell the engine how much of the longitudinal force can be absorbed by the tire.
// 
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
// *dFloat* coefficient - longitudinal slide coefficient.
//
// Return: nothing.
//
// Remarks: The explanation of longitudinal slide is similar to the side slip, however it is not so critical to achieve realistic behavior.
// See description of side slip on function *NewtonVehicleSetTireMaxSideSleepSpeed*
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
//
// See also: NewtonVehicleSetTireMaxLongitudinalSlideSpeed
void NewtonVehicleSetTireLongitudinalSlideCoeficient(const NewtonJoint* vehicle, void* tireId, dFloat coeficient)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->SetTireLongitudinalSlideCoeficient(tireId, coeficient);
}


// Name: NewtonVehicleTireIsAirBorne
// Return a boolean value that tells the application if this tire is touching the ground.
// 
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: airborne state 1 on the air, 0 on the ground.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
int NewtonVehicleTireIsAirBorne(const NewtonJoint* vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->IsTireAirBorned(tireId) ? 1 : 0;
}

// Name: NewtonVehicleTireLostSideGrip
// Return a boolean value that tell the application if this tire lost side grip..
// 
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: Grip state.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
int NewtonVehicleTireLostSideGrip(const NewtonJoint* vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;
	return joint->TireLostSideGrip(tireId) ? 1 : 0;
}


// Name: NewtonVehicleTireLostTraction
// Return a boolean value that tell the application if this tire lost longitudinal traction.
// 
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: traction state.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
int NewtonVehicleTireLostTraction(const NewtonJoint* vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;
	return joint->TireLostTraction(tireId) ? 1 : 0;
}

#endif

