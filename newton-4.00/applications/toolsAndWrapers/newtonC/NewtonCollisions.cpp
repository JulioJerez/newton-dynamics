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
  Create a box primitive for collision.

  @param *newtonWorld Pointer to the Newton world.
  @param dx box side one x dimension.
  @param dy box side one y dimension.
  @param dz box side one z dimension.
  @param shapeID fixme
  @param *offsetMatrix pointer to an array of 16 floats containing the offset matrix of the box relative to the body. If this parameter is NULL, then the primitive is centered at the origin of the body.

  @return Pointer to the box

*/
NewtonCollision* NewtonCreateBox(const NewtonWorld* const, dFloat dx, dFloat dy, dFloat dz, int shapeID, const dFloat* const offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}
	ndSharedPtr<ndShapeInstance>* const shape = new ndSharedPtr<ndShapeInstance>(new ndShapeInstance(new ndShapeBox(dx, dy, dz)));
	ndShapeInstance* const instance = **shape;
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	return reinterpret_cast<NewtonCollision*>(shape);
}

/*!
  Release a reference from this collision object returning control to Newton.

  @param *collisionPtr pointer to the collision object

  @return Nothing.

  to get the correct reference count of a collision primitive the application can call function *NewtonCollisionGetInfo*

*/
void NewtonDestroyCollision(const NewtonCollision* const collisionPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndSharedPtr<ndShapeInstance>* const instance(SharedObjectFromHandle<ndShapeInstance, NewtonCollision>(collisionPtr));
	delete instance;
}

void NewtonCollisionSetUserData(const NewtonCollision* const collision, void* const userData)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = ObjectFromHandle<ndShapeInstance, NewtonCollision>(collision);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userParam[0].m_ptrData = userData;
}

void NewtonCollisionSetMatrix(const NewtonCollision* collision, const dFloat* const matrixPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance instance(*ObjectFromHandle<ndShapeInstance, NewtonCollision>(collision));

	ndMatrix matrix(matrixPtr);
	if (!CheckFloat(&matrix[0][0], 16))
	{
		ndExpandTraceMessage(("uninitialized matrix, setting to identity\n"));
		matrix = ndGetIdentityMatrix();
	}
	instance.SetLocalMatrix(matrix);
}

void NewtonCollisionGetMatrix(const NewtonCollision* const collision, dFloat* const matrixPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance instance(*ObjectFromHandle<ndShapeInstance, NewtonCollision>(collision));

	const ndMatrix instanceMatrix(instance.GetLocalMatrix());
	ndMemCpy(matrixPtr, &instanceMatrix[0][0], sizeof(ndMatrix) / sizeof(ndFloat32));
}


NewtonCollision* NewtonCreateTreeCollisionFromMesh(const NewtonWorld* const newtonWorld, const NewtonMesh* const mesh, int shapeID)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = ObjectFromHandle<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);

	ndSharedPtr<ndShapeInstance>* const shape = new ndSharedPtr<ndShapeInstance>(meshEffect->CreateCollisionTree(false));
	ndShapeInstance* const instance = **shape;
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;

	return reinterpret_cast<NewtonCollision*>(shape);
}

/*!
  Create a container to hold an array of convex collision primitives.

  @param *newtonWorld Pointer to the Newton world.
  @param  shapeID: fixme

  @return Pointer to the compound collision.

  Compound collision primitives can only be made of convex collision primitives and they can not contain compound collision. Therefore they are treated as convex primitives.

  Compound collision primitives are treated as instance collision objects that can not shared by multiples rigid bodies.

*/
NewtonCollision* NewtonCreateCompoundCollision(const NewtonWorld* const, int shapeID)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndSharedPtr<ndShapeInstance>* const shape = new ndSharedPtr<ndShapeInstance>(new ndShapeInstance(new ndShapeCompound()));
	ndShapeInstance* const instance = **shape;
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	return reinterpret_cast<NewtonCollision*>(shape);
}

void NewtonCompoundCollisionBeginAddRemove(NewtonCollision* const compoundCollision)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = ObjectFromHandle<ndShapeInstance, NewtonCollision>(compoundCollision);
	ndShapeCompound* const collision = instance->GetShape()->GetAsShapeCompound();
	if (collision)
	{
		collision->BeginAddRemove();
	}
}

void NewtonCompoundCollisionEndAddRemove(NewtonCollision* const compoundCollision)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const compoundInstance = ObjectFromHandle<ndShapeInstance, NewtonCollision>(compoundCollision);
	ndShapeCompound* const collision = compoundInstance->GetShape()->GetAsShapeCompound();
	if (collision)
	{
		collision->EndAddRemove();
	}
}


void* NewtonCompoundCollisionAddSubCollision(NewtonCollision* const compoundCollision, const NewtonCollision* const convexCollision)
{
	TRACE_FUNCTION(__FUNCTION__);

	ndShapeInstance* const compoundInstance = ObjectFromHandle<ndShapeInstance, NewtonCollision>(compoundCollision);
	ndShapeInstance* const childInstance = ObjectFromHandle<ndShapeInstance, NewtonCollision>(convexCollision);

	ndShapeCompound* const collision = compoundInstance->GetShape()->GetAsShapeCompound();
	if (collision && childInstance->GetShape()->GetAsShapeConvex())
	{
		return collision->AddCollision(childInstance);
	}
	return nullptr;
}
