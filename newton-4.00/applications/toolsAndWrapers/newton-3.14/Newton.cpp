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

#ifdef _NEWTON_BUILD_DLL
	#if (defined (__MINGW32__) || defined (__MINGW64__))
		int main(int argc, char* argv[])
		{
			return 0;
		}
	#endif

	#ifdef _MSC_VER
		BOOL APIENTRY DllMain( HMODULE hModule,	DWORD  ul_reason_for_call, LPVOID lpReserved)
		{
			switch (ul_reason_for_call)
			{
				case DLL_THREAD_ATTACH:
				case DLL_PROCESS_ATTACH:
					// check for memory leaks
					#ifdef _DEBUG
						// Track all memory leaks at the operating system level.
						// make sure no Newton tool or utility leaves leaks behind.
						_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CRTDBG_REPORT_FLAG);
					#endif

				case DLL_THREAD_DETACH:
				case DLL_PROCESS_DETACH:
				break;
			}
			return TRUE;
		}
	#endif
#endif

#if 0

//#define SAVE_COLLISION

#ifdef SAVE_COLLISION
void SerializeFile (void* serializeHandle, const void* buffer, size_t size)
{
	fwrite (buffer, size, 1, (FILE*) serializeHandle);
}

void DeSerializeFile (void* serializeHandle, void* buffer, size_t size)
{
	fread (buffer, size, 1, (FILE*) serializeHandle);
}


void SaveCollision (const NewtonCollision* const collisionPtr)
{
	FILE* file;
	// save the collision file
	file = fopen ("collisiontest.bin", "wb");
	//SerializeFile (file, MAGIC_NUMBER, strlen (MAGIC_NUMBER) + 1);
	NewtonCollisionSerialize (collisionPtr, SerializeFile, file);
	fclose (file);
}
#endif


/*! @defgroup JointUniversal JointUniversal
Universal joint interface
@{
*/

/*!
  Create a universal joint.

  @param *newtonWorld Pointer to the Newton world.
  @param *pivotPoint is origin of the universal joint in global space.
  @param  *pinDir0 - first axis of rotation fixed on childBody body and perpendicular to pinDir1.
  @param  *pinDir1 - second axis of rotation fixed on parentBody body and perpendicular to pinDir0.
  @param *childBody is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
  @param *parentBody is the pointer to the parent rigid body, this body can be NULL or any kind of rigid body.

  @return Pointer to the universal joint.

  This function creates a universal joint and add it to the world. By default joint disables collision with the linked bodies.

  a universal joint is a constraint that restricts twp rigid bodies to be connected to a point fixed on both bodies,
  while and allowing one body to spin around a fix axis in is own frame, and the other body to spin around another axis fixes on
  it own frame. Both axis must be mutually perpendicular.
*/
NewtonJoint* NewtonConstraintCreateUniversal(const NewtonWorld* const newtonWorld, const dFloat* pivotPoint, 
	const dFloat* pinDir0, const dFloat* pinDir1, const NewtonBody* const childBody, const NewtonBody* const parentBody)
{
	TRACE_FUNCTION(__FUNCTION__);
	Newton* const world = (Newton *)newtonWorld;
	dgBody* const body0 = (dgBody *)childBody;
	dgBody* const body1 = (dgBody *)parentBody;
	dgVector pin0 (pinDir0[0], pinDir0[1], pinDir0[2], dgFloat32 (0.0f));
	dgVector pin1 (pinDir1[0], pinDir1[1], pinDir1[2], dgFloat32 (0.0f));
	dgVector pivot (pivotPoint[0], pivotPoint[1], pivotPoint[2], dgFloat32 (0.0f));
	return (NewtonJoint*) world->CreateUniversalConstraint (pivot, pin0, pin1, body0, body1);
}


/*!
  Set an update call back to be called when either of the two body linked by the joint is active.

  @param *universal pointer to the joint.
  @param callback pointer to the joint function call back.

  @return nothing.

  if the application wants to have some feedback from the joint simulation, the application can register a function
  update callback to be called every time any of the bodies linked by this joint is active. This is useful to provide special
  effects like particles, sound or even to simulate breakable moving parts.

  the function *NewtonUniversalCallback callback* should return a bit field code.
  if the application does not want to set the joint acceleration the return code is zero
  if the application only wants to change the joint linear acceleration the return code is 1
  if the application only wants to change the joint angular acceleration the return code is 2
  if the application only wants to change the joint angular and linear acceleration the return code is 3

  See also: ::NewtonJointGetUserData, ::NewtonJointSetUserData
*/
void NewtonUniversalSetUserCallback(const NewtonJoint* const universal, NewtonUniversalCallback callback)
{
	dgUniversalConstraint* contraint;

	TRACE_FUNCTION(__FUNCTION__);
	contraint = (dgUniversalConstraint*) universal;
	contraint->SetJointParameterCallback ((dgUniversalJointAcceleration)callback);
}


/*!
  Get the relative joint angle between the two bodies.

  @param *universal pointer to the joint.

  @return the joint angle relative to the universal pin0.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can play a bell sound when the joint angle passes some max value.

  See also: ::NewtonUniversalSetUserCallback
*/
dFloat NewtonUniversalGetJointAngle0(const NewtonJoint* const universal)
{
	dgUniversalConstraint* contraint;

	TRACE_FUNCTION(__FUNCTION__);
	contraint = (dgUniversalConstraint*) universal;
	return contraint->GetJointAngle0 ();
}

/*!
  Get the relative joint angle between the two bodies.

  @param *universal pointer to the joint.

  @return the joint angle relative to the universal pin1.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can play a bell sound when the joint angle passes some max value.

  See also: ::NewtonUniversalSetUserCallback
*/
dFloat NewtonUniversalGetJointAngle1(const NewtonJoint* const universal)
{
	dgUniversalConstraint* contraint;

	TRACE_FUNCTION(__FUNCTION__);
	contraint = (dgUniversalConstraint*) universal;
	return contraint->GetJointAngle1 ();
}


/*!
  Get the relative joint angular velocity between the two bodies.

  @param *universal pointer to the joint.

  @return the joint angular velocity relative to the pin0 axis.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can play the creaky noise of a hanging lamp.

  See also: ::NewtonUniversalSetUserCallback
*/
dFloat NewtonUniversalGetJointOmega0(const NewtonJoint* const universal)
{
	dgUniversalConstraint* contraint;

	TRACE_FUNCTION(__FUNCTION__);
	contraint = (dgUniversalConstraint*) universal;
	return contraint->GetJointOmega0 ();
}


/*!
  Get the relative joint angular velocity between the two bodies.

  @param *universal pointer to the joint.

  @return the joint angular velocity relative to the pin1 axis.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can play the creaky noise of a hanging lamp.

  See also: ::NewtonUniversalSetUserCallback
*/
dFloat NewtonUniversalGetJointOmega1(const NewtonJoint* const universal)
{
	dgUniversalConstraint* contraint;

	TRACE_FUNCTION(__FUNCTION__);
	contraint = (dgUniversalConstraint*) universal;
	return contraint->GetJointOmega1 ();
}



/*!
  Calculate the angular acceleration needed to stop the universal at the desired angle.

  @param *universal pointer to the joint.
  @param *desc is the pointer to the Universal or slide structure.
  @param angle is the desired universal stop angle rotation around pin0

  @return the relative angular acceleration needed to stop the universal.

  this function can only be called from a *NewtonUniversalCallback* and it can be used by the application to implement universal limits.

  See also: ::NewtonUniversalSetUserCallback
*/
dFloat NewtonUniversalCalculateStopAlpha0(const NewtonJoint* const universal, const NewtonHingeSliderUpdateDesc* const desc, dFloat angle)
{
	dgUniversalConstraint* contraint;

	TRACE_FUNCTION(__FUNCTION__);
	contraint = (dgUniversalConstraint*) universal;
	return contraint->CalculateStopAlpha0 (angle, (dgJointCallbackParam*) desc);
}

/*!
  Calculate the angular acceleration needed to stop the universal at the desired angle.

  @param *universal pointer to the joint.
  @param *desc is the pointer to and the Universal or slide structure.
  @param angle is the desired universal stop angle rotation around pin1

  @return the relative angular acceleration needed to stop the universal.

  this function can only be called from a *NewtonUniversalCallback* and it can be used by the application to implement universal limits.

  See also: ::NewtonUniversalSetUserCallback
*/
dFloat NewtonUniversalCalculateStopAlpha1(const NewtonJoint* const universal, const NewtonHingeSliderUpdateDesc* const desc, dFloat angle)
{
	dgUniversalConstraint* contraint;

	TRACE_FUNCTION(__FUNCTION__);
	contraint = (dgUniversalConstraint*) universal;
	return contraint->CalculateStopAlpha1 (angle, (dgJointCallbackParam*) desc);
}



/*!
  Get the total force asserted over the joint pivot point, to maintain the constraint.

  @param *universal pointer to the joint.
  @param *force pointer to an array of a least three floats to hold the force value of the joint.

  @return nothing.

  this function can be used during a function update call back to provide the application with some special effect.
  for example the application can destroy the joint if the force exceeds some predefined value.

  See also: ::NewtonUniversalSetUserCallback
*/
void NewtonUniversalGetJointForce(const NewtonJoint* const universal, dFloat* const force)
{
	dgUniversalConstraint* contraint;

	TRACE_FUNCTION(__FUNCTION__);
	contraint = (dgUniversalConstraint*) universal;
	dgVector forceVector (contraint->GetJointForce ());
	force[0] = forceVector.m_x;
	force[1] = forceVector.m_y;
	force[2] = forceVector.m_z;
}


/*! @} */ // end of JointUniversal

/*! @defgroup JointUpVector JointUpVector
UpVector joint Interface
@{
*/

/*!
  Create a UpVector joint.

  @param *newtonWorld Pointer to the Newton world.
  @param *pinDir is the aligning vector.
  @param *body is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.

  @return Pointer to the up vector joint.

  This function creates an up vector joint. An up vector joint is a constraint that allows a body to translate freely in 3d space,
  but it only allows the body to rotate around the pin direction vector. This could be use by the application to control a character
  with physics and collision.

  Since the UpVector joint is a unary constraint, there is not need to have user callback or user data assigned to it.
  The application can simple hold to the joint handle and update the pin on the force callback function of the rigid body owning the joint.
*/
NewtonJoint* NewtonConstraintCreateUpVector (const NewtonWorld* const newtonWorld, const dFloat* pinDir, const NewtonBody* const body)
{
	TRACE_FUNCTION(__FUNCTION__);
	Newton* const world = (Newton *)newtonWorld;
	dgBody* const body0 = (dgBody *)body;
	dgVector pin (pinDir[0], pinDir[1], pinDir[2], dgFloat32 (0.0f));
	return (NewtonJoint*) world->CreateUpVectorConstraint(pin, body0);
}


/*!
  Get the up vector pin of this joint in global space.

  @param *upVector pointer to the joint.
  @param *pin pointer to an array of a least three floats to hold the up vector direction in global space.

  @return nothing.

  the application ca call this function to read the up vector, this is useful to animate the up vector.
  if the application is going to animated the up vector, it must do so by applying only small rotation,
  too large rotation can cause vibration of the joint.

  See also: ::NewtonUpVectorSetPin
*/
void NewtonUpVectorGetPin(const NewtonJoint* const upVector, dFloat *pin)
{
	dgUpVectorConstraint* contraint;

	TRACE_FUNCTION(__FUNCTION__);
	contraint = (dgUpVectorConstraint*) upVector;

	dgVector pinVector (contraint ->GetPinDir ());
	pin[0] = pinVector.m_x;
	pin[1] = pinVector.m_y;
	pin[2] = pinVector.m_z;
}


/*!
  Set the up vector pin of this joint in global space.

  @param *upVector pointer to the joint.
  @param *pin pointer to an array of a least three floats containing the up vector direction in global space.

  @return nothing.

  the application ca call this function to change the joint up vector, this is useful to animate the up vector.
  if the application is going to animated the up vector, it must do so by applying only small rotation,
  too large rotation can cause vibration of the joint.

  See also: ::NewtonUpVectorGetPin
*/
void NewtonUpVectorSetPin(const NewtonJoint* const upVector, const dFloat *pin)
{
	dgUpVectorConstraint* contraint;

	TRACE_FUNCTION(__FUNCTION__);
	contraint = (dgUpVectorConstraint*) upVector;

	dgVector pinVector (pin[0], pin[1], pin[2], dgFloat32 (0.0f));
	contraint->SetPinDir (pinVector);
}

/*! @} */ // end of JointUpVector

/*! @defgroup JointUser JointUser
User defined joint interface
@{
*/

/*! @} */ // end of JointUser

/*! @defgroup JointCommon JointCommon
Joint common function s
@{
*/

/*! @} */ // end of JointCommon


/*
void NewtonDeformableMeshConstraintParticle(NewtonCollision* const deformableMesh, int particleIndex, const dFloat* const posit, const NewtonBody* const body)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgCollisionInstance* const collision = (dgCollisionInstance*)deformableMesh;
	if (collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const deformableShape = (dgCollisionDeformableMesh*)collision->GetChildShape();
		dgVector position(posit[0], posit[1], posit[2], dgFloat32(0.0f));
		deformableShape->ConstraintParticle(particleIndex, position, (dgBody*)body);
	}
}



void NewtonDeformableMeshCreateClusters (NewtonCollision* const deformableMesh, int clunsterCount, dFloat overlapingWidth)
{
	dgAssert(0);

	TRACE_FUNCTION(__FUNCTION__);
	dgCollisionInstance* const collision = (dgCollisionInstance*) deformableMesh;
	if (collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const deformableShape = (dgCollisionDeformableMesh*) collision->GetChildShape();
		deformableShape->CreateClusters(clunsterCount, overlapingWidth);
	}

}

void NewtonDeformableMeshSetDebugCallback (NewtonCollision* const deformableMesh, NewtonCollisionIterator callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgCollisionInstance* const collision = (dgCollisionInstance*) deformableMesh;
	if (collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const deformableShape = (dgCollisionDeformableMesh*) collision->GetChildShape();
		deformableShape->SetOnDebugDisplay((dgCollision::OnDebugCollisionMeshCallback)callback); 
	}
}

void NewtonDeformableMeshGetParticlePosition (NewtonCollision* const deformableMesh, int particleIndex, dFloat* const posit)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgCollisionInstance* const collision = (dgCollisionInstance*) deformableMesh;
	if (collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const deformableShape = (dgCollisionDeformableMesh*) collision->GetChildShape();
		dgVector p (deformableShape->GetParticlePosition(particleIndex));
		posit[0] = p[0];
		posit[1] = p[1];
		posit[2] = p[2];
	}
}

void NewtonDeformableMeshBeginConfiguration (const NewtonCollision* const deformableMesh)
{
}

void NewtonDeformableMeshEndConfiguration (const NewtonCollision* const deformableMesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgCollisionInstance* const collision = (dgCollisionInstance*) deformableMesh;
	if (collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const deformableShape = (dgCollisionDeformableMesh*) collision->GetChildShape();
		deformableShape->EndConfiguration();
	}
}

void NewtonDeformableMeshUnconstraintParticle (NewtonCollision* const deformableMesh, int partivleIndex)
{
}



void NewtonDeformableMeshSetSkinThickness (NewtonCollision* const deformableMesh, dFloat skinThickness)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgCollisionInstance* const collision = (dgCollisionInstance*) deformableMesh;
	if (collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const deformableShape = (dgCollisionDeformableMesh*) collision->GetChildShape();
		deformableShape->SetSkinThickness(skinThickness);
	}
}

void NewtonDeformableMeshSetPlasticity (NewtonCollision* const deformableMesh, dFloat plasticity)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAssert (0);

	dgCollisionInstance* const collision = (dgCollisionInstance*) deformableMesh;
	if (collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const deformable = (dgCollisionDeformableMesh*) collision;
		deformable->SetPlasticity (plasticity);
	}
}

void NewtonDeformableMeshSetStiffness (NewtonCollision* const deformableMesh, dFloat stiffness)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAssert (0);

	dgCollisionInstance* const collision = (dgCollisionInstance*) deformableMesh;
	if (collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const deformable = (dgCollisionDeformableMesh*) collision;
		deformable->SetStiffness(stiffness);
	}
}


int NewtonDeformableMeshGetVertexCount (const NewtonCollision* const deformableMesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgCollisionInstance* const collision = (dgCollisionInstance*) deformableMesh;
	if (collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const deformableShape = (dgCollisionDeformableMesh*) collision->GetChildShape();
		return deformableShape->GetVisualPointsCount();
	}
	return 0;
}

void NewtonDeformableMeshUpdateRenderNormals (const NewtonCollision* const deformableMesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgCollisionInstance* const collision = (dgCollisionInstance*) deformableMesh;
	if (collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const deformableShape = (dgCollisionDeformableMesh*) collision->GetChildShape();
		deformableShape->UpdateVisualNormals();
	}
}

void NewtonDeformableMeshGetVertexStreams (const NewtonCollision* const deformableMesh, int vertexStrideInByte, dFloat* const vertex, int normalStrideInByte, dFloat* const normal, int uvStrideInByte0, dFloat* const uv0)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgCollisionInstance* const collision = (dgCollisionInstance*) deformableMesh;
	if (collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const deformableShape = (dgCollisionDeformableMesh*) collision->GetChildShape();
		deformableShape->GetVisualVertexData(vertexStrideInByte, vertex, normalStrideInByte, normal, uvStrideInByte0, uv0);
	}
}

NewtonDeformableMeshSegment* NewtonDeformableMeshGetFirstSegment (const NewtonCollision* const deformableMesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgCollisionInstance* const collision = (dgCollisionInstance*) deformableMesh;
	if (collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const deformableShape = (dgCollisionDeformableMesh*) collision->GetChildShape();
		return (NewtonDeformableMeshSegment*) deformableShape->GetFirtVisualSegment();
	}
	return NULL;
}

NewtonDeformableMeshSegment* NewtonDeformableMeshGetNextSegment (const NewtonCollision* const deformableMesh, const NewtonDeformableMeshSegment* const segment)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgCollisionInstance* const collision = (dgCollisionInstance*) deformableMesh;
	if (collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const deformableShape = (dgCollisionDeformableMesh*) collision->GetChildShape();
		return (NewtonDeformableMeshSegment*) deformableShape->GetNextVisualSegment((void*)segment);
	}
	return NULL;
}

int NewtonDeformableMeshSegmentGetMaterialID (const NewtonCollision* const deformableMesh, const NewtonDeformableMeshSegment* const segment)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgCollisionInstance* const collision = (dgCollisionInstance*) deformableMesh;
	if (collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const deformableShape = (dgCollisionDeformableMesh*) collision->GetChildShape();
		return deformableShape->GetSegmentMaterial((void*)segment);
	}
	return 0;
}

int NewtonDeformableMeshSegmentGetIndexCount (const NewtonCollision* const deformableMesh, const NewtonDeformableMeshSegment* const segment)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgCollisionInstance* const collision = (dgCollisionInstance*) deformableMesh;
	if (collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const deformableShape = (dgCollisionDeformableMesh*) collision->GetChildShape();
		return deformableShape->GetSegmentIndexCount((void*)segment);
	}
	return 0;
}

const int* NewtonDeformableMeshSegmentGetIndexList (const NewtonCollision* const deformableMesh, const NewtonDeformableMeshSegment* const segment)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgCollisionInstance* const collision = (dgCollisionInstance*) deformableMesh;
	if (collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI)) {
		dgCollisionDeformableMesh* const deformableShape = (dgCollisionDeformableMesh*) collision->GetChildShape();
		return deformableShape->GetSegmentIndexList((void*)segment);
	}
	return NULL;
}
*/

/*! @} */ // end of


#endif

// ***************************************************************
// 
// ported code
// 
// ***************************************************************

/*!
  Create an instance of the Newton world.

  @return Pointer to new Newton world.

  This function must be called before any of the other API functions.

  See also: ::NewtonDestroy, ::NewtonDestroyAllBodies
*/
NewtonWorld* NewtonCreate()
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = new ndNewtonWorld();
	return PointerCast<NewtonWorld, ndNewtonWorld>(world);
}

/*!
  Destroy an instance of the Newton world.

  @param *newtonWorld Pointer to the Newton world.
  @return Nothing.

  This function will destroy the entire Newton world.

  See also: ::NewtonCreate, ::NewtonDestroyAllBodies
*/
void NewtonDestroy(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	//ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	delete world;
}

/*!
  Store a user defined data value with the world.

  @param *newtonWorld is the pointer to the newton world.
  @param *userData pointer to the user defined user data value.

  @return Nothing.

  The application can attach custom data to the Newton world. Newton will never
  look at this data.

  The user data is useful for application developing object oriented classes
  based on the Newton API.

  See also: ::NewtonBodyGetUserData, ::NewtonWorldSetUserData, ::NewtonWorldGetUserData
*/
void NewtonWorldSetUserData(const NewtonWorld* const newtonWorld, void* const userData)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->m_userData = userData;
}

/*!
  Retrieve the user data attached to the world.

  @param *newtonWorld Pointer to the Newton world.

  @return Pointer to user data.

  See also: ::NewtonBodySetUserData, ::NewtonWorldSetUserData, ::NewtonWorldGetUserData
  */
void* NewtonWorldGetUserData(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	return *world->m_userData;
}

/*!
  Reset all internal engine states.

  @param *newtonWorld Pointer to the Newton world.

  Call this function whenever you want to create a reproducible simulation from
  a pre-defined initial condition.

  It does *not* suffice to merely reset the position and velocity of
  objects. This is because Newton takes advantage of frame-to-frame coherence for
  performance reasons.

  This function must be called outside of a Newton Update.

  Note: this kind of synchronization incurs a heavy performance penalty if
  called during each update.

  See also: ::NewtonUpdate
*/
void NewtonInvalidateCache(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->ClearCache();
}


/*!
  Remove all bodies and joints from the Newton world.

  @param *newtonWorld Pointer to the Newton world.

  @return Nothing

  This function will destroy all bodies and all joints in the Newton world, but
  will retain group IDs.

  Use this function for when you want to clear the world but preserve all the
  group IDs and material pairs.

  See also: ::NewtonMaterialDestroyAllGroupID
*/
void NewtonDestroyAllBodies(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	const ndBodyListView& bodyList = world->GetBodyList();
	while (bodyList.GetCount())
	{
		ndBodyListView::ndNode* const node = bodyList.GetLast();
		ndSharedPtr<ndBody>& body = node->GetInfo();
		world->RemoveBody(*body);
	}
}

/*!
  Advance the simulation by a user defined amount of time.

  @param *newtonWorld is the pointer to the Newton world
  @param timestep time step in seconds.

  @return Nothing

  This function will advance the simulation by the specified amount of time.

  The Newton Engine does not perform sub-steps, nor  does it need
  tuning parameters. As a consequence, the application is responsible for
  requesting sane time steps.

  See also: ::NewtonInvalidateCache
*/
void NewtonUpdate(const NewtonWorld* const newtonWorld, dFloat timestep)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);

	world->Update(timestep);
	world->Sync();
}

void NewtonUpdateAsync(const NewtonWorld* const newtonWorld, dFloat timestep)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);

	world->Sync();
	world->Update(timestep);
}

void* NewtonGetPreferedPlugin(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndWorld::ndSolverModes mode = ndWorld::ndSolverModes(ndWorld::ndSimd8Solver);
	return reinterpret_cast<void*>(mode);
}

void* NewtonCurrentPlugin(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndWorld::ndSolverModes mode = ndWorld::ndSolverModes(world->GetSelectedSolver());
	return reinterpret_cast<void*>(mode);
}

void* NewtonGetFirstPlugin(const NewtonWorld* const)
{
	TRACE_FUNCTION(__FUNCTION__);
	return reinterpret_cast<void*>(ndWorld::ndStandardSolver);
}

void* NewtonGetNextPlugin(const NewtonWorld* const newtonWorld, const void* const plugin)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);

	ndInt32 enumerator = static_cast<ndInt32>(reinterpret_cast<uintptr_t>(plugin));
	ndWorld::ndSolverModes mode = ndWorld::ndSolverModes(enumerator);
	switch (mode)
	{
		case ndWorld::ndStandardSolver:
		{
			mode = ndWorld::ndSimd8Solver;
			break;
		}
		case ndWorld::ndSimd8Solver:
		{
			mode = ndWorld::ndSimd16Solver;
			break;
		}

		case ndWorld::ndSimd16Solver:
		{
			mode = ndWorld::ndSolverModes(0);
			break;
		}
	}

	return reinterpret_cast<void*>(mode);
}

void NewtonSelectPlugin(const NewtonWorld* const newtonWorld, const void* const plugin)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);

	ndInt32 enumerator = static_cast<ndInt32>(reinterpret_cast<uintptr_t>(plugin));
	ndWorld::ndSolverModes mode = ndWorld::ndSolverModes(enumerator);
	world->SelectSolver(mode);
}

const char* NewtonGetPluginString(const NewtonWorld* const newtonWorld, const void* const plugin)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndInt32 enumerator = static_cast<ndInt32>(reinterpret_cast<uintptr_t>(plugin));
	ndWorld::ndSolverModes mode = ndWorld::ndSolverModes(enumerator);
	switch (mode)
	{
		case ndWorld::ndStandardSolver:
		{
			return "default";
			break;
		}
		case ndWorld::ndSimd8Solver:
		{
			return "simd8";
			break;
		}

		case ndWorld::ndSimd16Solver:
		{
			return "simd16";
			break;
		}
	}

	return "default";
}

/*!
  Set the solver precision mode.

  @param *newtonWorld is the pointer to the Newton world
  @param model model of operation n = number of iteration default value is 4.

  @return Nothing

  n: the solve will execute a maximum of n iteration per cluster of connected joints and will terminate regardless of the
  of the joint residual acceleration.
  If it happen that the joints residual acceleration fall below the minimum tolerance 1.0e-5
  then the solve will terminate before the number of iteration reach N.
*/
void NewtonSetSolverIterations(const NewtonWorld* const newtonWorld, int iterations)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->SetSolverIterations(iterations);
}

/*!
Get the solver precision mode.
*/
int NewtonGetSolverIterations(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	return world->GetSolverIterations();
}

void NewtonSetNumberOfSubsteps(const NewtonWorld* const newtonWorld, int subSteps)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->SetSubSteps(subSteps);
}

int NewtonGetNumberOfSubsteps(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	return world->GetSubSteps();
}

dFloat NewtonGetLastUpdateTime(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	return world->GetUpdateTime();
}

void NewtonWaitForUpdateToFinish(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->Sync();
}


/*!
  Set the maximum number of threads the engine can use.

  @param *newtonWorld Pointer to the Newton world.
  @param threads Maximum number of allowed threads.

  @return Nothing

  The maximum number of threaded is set on initialization to the maximum number
  of CPU in the system.
  fixme: this appears to be wrong. It is set to 1.

  See also: ::NewtonGetThreadsCount
*/
void NewtonSetThreadsCount(const NewtonWorld* const newtonWorld, int threads)
{
	TRACE_FUNCTION(__FUNCTION__);

	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->SetThreadCount(threads);
}

/*!
  Return the number of threads currently used by the engine.

  @param *newtonWorld Pointer to the Newton world.

  @return Number threads

  See also: ::NewtonSetThreadsCount, ::NewtonSetParallelSolverOnLargeIsland
*/
int NewtonGetThreadsCount(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);

	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	return world->GetThreadCount();
}

/*!
  Return the maximum number of threads supported on this platform.

  @param *newtonWorld Pointer to the Newton world.

  @return Number threads.

  This function will return 1 on single core version of the library.
  // fixme; what is a single core version?

  See also: ::NewtonSetThreadsCount, ::NewtonSetParallelSolverOnLargeIsland
*/
int NewtonGetMaxThreadsCount(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);

	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	return world->GetScene()->GetMaxThreads();
}

/*!
  Return the total number of rigid bodies in the world.

  @param *newtonWorld Pointer to the Newton world.

  @return Number of rigid bodies in the world.

*/
int NewtonWorldGetBodyCount(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	return world->GetBodyList().GetCount();
}

/*!
  Return the total number of constraints in the world.

  @param *newtonWorld pointer to the Newton world.

  @return number of constraints.

*/
int NewtonWorldGetConstraintCount(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	return world->GetJointList().GetCount();
}

void NewtonSetPostUpdateCallback(const NewtonWorld* const newtonWorld, NewtonPostUpdateCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->m_onPostUpdate = callback;
}

NewtonPostUpdateCallback NewtonGetPostUpdateCallback(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	return world->m_onPostUpdate;
}

void NewtonSerializeToFile(const NewtonWorld* const newtonWorld, const char* const filename, NewtonOnBodySerializationCallback bodyCallback, void* const bodyUserData)
{
	TRACE_FUNCTION(__FUNCTION__);
	//FILE* const file = fopen(filename, "wb");
	//if (file) {
	//	NewtonSerializeScene(newtonWorld, bodyCallback, bodyUserData, dgWorld::OnSerializeToFile, file);
	//	fclose(file);
	//}
	ndAssert(0);
}

void NewtonDeserializeFromFile(const NewtonWorld* const newtonWorld, const char* const filename, NewtonOnBodyDeserializationCallback bodyCallback, void* const bodyUserData)
{
	TRACE_FUNCTION(__FUNCTION__);
	//FILE* const file = fopen(filename, "rb");
	//if (file) {
	//	NewtonDeserializeScene(newtonWorld, bodyCallback, bodyUserData, dgWorld::OnDeserializeFromFile, file);
	//	fclose(file);
	//}
	ndAssert(0);
}

void NewtonSerializeScene(const NewtonWorld* const newtonWorld, NewtonOnBodySerializationCallback bodyCallback, void* const bodyUserData,
	NewtonSerializeCallback serializeCallback, void* const serializeHandle)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//world->SerializeScene(bodyUserData, dgWorld::OnBodySerialize(bodyCallback), (dgSerialize)serializeCallback, serializeHandle);
	ndAssert(0);
}

void NewtonDeserializeScene(const NewtonWorld* const newtonWorld, NewtonOnBodyDeserializationCallback bodyCallback, void* const bodyUserData,
	NewtonDeserializeCallback deserializeCallback, void* const serializeHandle)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//world->DeserializeScene(bodyUserData, (dgWorld::OnBodyDeserialize)bodyCallback, (dgDeserialize)deserializeCallback, serializeHandle);
	ndAssert(0);
}

NewtonBody* NewtonFindSerializedBody(const NewtonWorld* const newtonWorld, int bodySerializedID)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//dgAssert(0);
	//return (NewtonBody*)world->FindBodyFromSerializedID(bodySerializedID);
	ndAssert(0);
	return 0;
}

void NewtonSetJointSerializationCallbacks(const NewtonWorld* const newtonWorld, NewtonOnJointSerializationCallback serializeJoint, NewtonOnJointDeserializationCallback deserializeJoint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//world->SetJointSerializationCallbacks(dgWorld::OnJointSerializationCallback(serializeJoint), dgWorld::OnJointDeserializationCallback(deserializeJoint));
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->m_onJointSerialize = serializeJoint;
	world->m_onJointDeserialize = deserializeJoint;
}

void NewtonGetJointSerializationCallbacks(const NewtonWorld* const newtonWorld, NewtonOnJointSerializationCallback* const serializeJoint, NewtonOnJointDeserializationCallback* const deserializeJoint)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	*serializeJoint = world->m_onJointSerialize;
	*deserializeJoint = world->m_onJointDeserialize;
}

/*!
  Specify a custom destructor callback for destroying the world.

  @param *newtonWorld Pointer to the Newton world.
  @param destructor function poiter callback

  The application may specify its own world destructor.

  See also: ::NewtonWorldSetDestructorCallback, ::NewtonWorldGetUserData
*/
void NewtonWorldSetDestructorCallback(const NewtonWorld* const newtonWorld, NewtonWorldDestructorCallback destructor)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//world->m_destructor = destructor;
	ndAssert(0);
}

/*!
  Return pointer to destructor call back function.

  @param *newtonWorld Pointer to the Newton world.

  See also: ::NewtonWorldGetUserData, ::NewtonWorldSetDestructorCallback
*/
NewtonWorldDestructorCallback NewtonWorldGetDestructorCallback(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return world->m_destructor;
	ndAssert(0);
	return 0;
}

void NewtonWorldSetCreateDestroyContactCallback(const NewtonWorld* const newtonWorld, NewtonCreateContactCallback createContact, NewtonDestroyContactCallback destroyContact)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//world->SetCreateDestroyContactCallback((dgWorld::OnCreateContact)createContact, (dgWorld::OnDestroyContact)destroyContact);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->m_onCreateContact = createContact;
	world->m_onDestroyContact = destroyContact;
}

void NewtonWorldSetCollisionConstructorDestructorCallback(const NewtonWorld* const newtonWorld, NewtonCollisionCopyConstructionCallback constructor, NewtonCollisionDestructorCallback destructor)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//world->SetCollisionInstanceConstructorDestructor((dgWorld::OnCollisionInstanceDuplicate)constructor, (dgWorld::OnCollisionInstanceDestroy)destructor);
	ndAssert(0);
}
