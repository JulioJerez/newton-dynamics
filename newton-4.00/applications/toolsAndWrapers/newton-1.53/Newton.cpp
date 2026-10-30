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


// Name: NewtonCreateTreeCollision 

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

