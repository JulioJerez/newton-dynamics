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
	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeCompound());
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	return reinterpret_cast<NewtonCollision*>(instance);
}

/*!
  Create a compound collision from a concave mesh by an approximate convex partition

  @param *newtonWorld Pointer to the Newton world.
  @param *convexAproximation fixme
  @param hullTolerance fixme
  @param shapeID fixme
  @param subShapeID fixme


  @return Pointer to the compound collision.

  The algorithm will separated the the original mesh into a series of sub meshes until either
  the worse concave point is smaller than the specified min concavity or the max number convex shapes is reached.

  is is recommended that convex approximation are made by person with a graphics toll by physically overlaying collision primitives over the concave mesh.
  but for quit test of maybe for simple meshes and algorithm approximations can be used.

  is is recommended that for best performance this function is used in an off line toll and serialize the output.

  Compound collision primitives are treated as instanced collision objects that cannot be shared by multiples rigid bodies.

*/
NewtonCollision* NewtonCreateCompoundCollisionFromMesh(const NewtonWorld* const newtonWorld, const NewtonMesh* const convexAproximation, dFloat hullTolerance, int shapeID, int subShapeID)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonCollision* const compound = NewtonCreateCompoundCollision(newtonWorld, shapeID);
	//NewtonCompoundCollisionBeginAddRemove(compound);
	//
	//NewtonMesh* nextSegment = NULL;
	//for (NewtonMesh* segment = NewtonMeshCreateFirstSingleSegment(convexAproximation); segment; segment = nextSegment) {
	//	nextSegment = NewtonMeshCreateNextSingleSegment(convexAproximation, segment);
	//
	//	NewtonCollision* const convexHull = NewtonCreateConvexHullFromMesh(newtonWorld, segment, hullTolerance, subShapeID);
	//	if (convexHull) {
	//		NewtonCompoundCollisionAddSubCollision(compound, convexHull);
	//		NewtonDestroyCollision(convexHull);
	//	}
	//	NewtonMeshDestroy(segment);
	//}
	//
	//NewtonCompoundCollisionEndAddRemove(compound);
	//
	//return compound;
	ndAssert(0);
	return 0;
}

void NewtonCompoundCollisionBeginAddRemove(NewtonCollision* const compoundCollision)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = const_cast<ndShapeInstance*>(reinterpret_cast<const ndShapeInstance*>(compoundCollision));
	ndShapeCompound* const collision = instance->GetShape()->GetAsShapeCompound();
	if (collision)
	{
		collision->BeginAddRemove();
	}
}

void NewtonCompoundCollisionEndAddRemove(NewtonCollision* const compoundCollision)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = const_cast<ndShapeInstance*>(reinterpret_cast<const ndShapeInstance*>(compoundCollision));
	ndShapeCompound* const collision = instance->GetShape()->GetAsShapeCompound();
	if (collision)
	{
		collision->EndAddRemove();
	}
}

void* NewtonCompoundCollisionAddSubCollision(NewtonCollision* const compoundCollision, const NewtonCollision* const convexCollision)
{
	TRACE_FUNCTION(__FUNCTION__);

	ndShapeInstance* const compoundInstance = const_cast<ndShapeInstance*>(reinterpret_cast<const ndShapeInstance*>(compoundCollision));
	ndShapeInstance* const childInstance = const_cast<ndShapeInstance*>(reinterpret_cast<const ndShapeInstance*>(convexCollision));

	ndShapeCompound* const collision = compoundInstance->GetShape()->GetAsShapeCompound();
	if (collision && childInstance->GetShape()->GetAsShapeConvex())
	{
		return collision->AddCollision(childInstance);
	}
	return nullptr;
}


void NewtonCompoundCollisionRemoveSubCollision(NewtonCollision* const compoundCollision, const void* const collisionNode)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const instance = (dgCollisionInstance*)compoundCollision;
	//if (instance->IsType(dgCollision::dgCollisionCompound_RTTI)) {
	//	dgCollisionCompound* const collision = (dgCollisionCompound*)instance->GetChildShape();
	//	dgCollisionInstance* const childCollision = collision->GetCollisionFromNode((dgCollisionCompound::dgTreeArray::dgTreeNode*)collisionNode);
	//	if (childCollision && childCollision->IsType(dgCollision::dgCollisionConvexShape_RTTI)) {
	//		collision->RemoveCollision((dgCollisionCompound::dgTreeArray::dgTreeNode*)collisionNode);
	//	}
	//}
	ndAssert(0);
}

void NewtonCompoundCollisionRemoveSubCollisionByIndex(NewtonCollision* const compoundCollision, int nodeIndex)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const instance = (dgCollisionInstance*)compoundCollision;
	//if (instance->IsType(dgCollision::dgCollisionCompound_RTTI)) {
	//	dgCollisionCompound* const collision = (dgCollisionCompound*)instance->GetChildShape();
	//	NewtonCompoundCollisionRemoveSubCollision(compoundCollision, collision->FindNodeByIndex(nodeIndex));
	//}
	ndAssert(0);
}


void NewtonCompoundCollisionSetSubCollisionMatrix(NewtonCollision* const compoundCollision, const void* const collisionNode, const dFloat* const matrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const compoundInstance = (dgCollisionInstance*)compoundCollision;
	//if (compoundInstance->IsType(dgCollision::dgCollisionCompound_RTTI)) {
	//	dgCollisionCompound* const collision = (dgCollisionCompound*)compoundInstance->GetChildShape();
	//	collision->SetCollisionMatrix((dgCollisionCompound::dgTreeArray::dgTreeNode*)collisionNode, dgMatrix(matrix));
	//}
	ndAssert(0);
}


void* NewtonCompoundCollisionGetFirstNode(NewtonCollision* const compoundCollision)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const instance = (dgCollisionInstance*)compoundCollision;
	//if (instance->IsType(dgCollision::dgCollisionCompound_RTTI)) {
	//	dgCollisionCompound* const collision = (dgCollisionCompound*)instance->GetChildShape();
	//	return collision->GetFirstNode();
	//}
	//return NULL;
	ndAssert(0);
	return 0;
}

void* NewtonCompoundCollisionGetNextNode(NewtonCollision* const compoundCollision, const void* const node)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const instance = (dgCollisionInstance*)compoundCollision;
	//if (instance->IsType(dgCollision::dgCollisionCompound_RTTI)) {
	//	dgCollisionCompound* const collision = (dgCollisionCompound*)instance->GetChildShape();
	//	return collision->GetNextNode((dgCollisionCompound::dgTreeArray::dgTreeNode*)node);
	//}
	//return NULL;
	ndAssert(0);
	return 0;
}

void* NewtonCompoundCollisionGetNodeByIndex(NewtonCollision* const compoundCollision, int index)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const instance = (dgCollisionInstance*)compoundCollision;
	//if (instance->IsType(dgCollision::dgCollisionCompound_RTTI)) {
	//	dgCollisionCompound* const collision = (dgCollisionCompound*)instance->GetChildShape();
	//	return collision->FindNodeByIndex(index);
	//}
	//return NULL;
	ndAssert(0);
	return 0;
}

int NewtonCompoundCollisionGetNodeIndex(NewtonCollision* const compoundCollision, const void* const node)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const instance = (dgCollisionInstance*)compoundCollision;
	//if (instance->IsType(dgCollision::dgCollisionCompound_RTTI)) {
	//	dgCollisionCompound* const collision = (dgCollisionCompound*)instance->GetChildShape();
	//	return collision->GetNodeIndex((dgCollisionCompound::dgTreeArray::dgTreeNode*)node);
	//}
	//return -1;
	ndAssert(0);
	return 0;
}


NewtonCollision* NewtonCompoundCollisionGetCollisionFromNode(NewtonCollision* const compoundCollision, const void* const node)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const compoundInstance = (dgCollisionInstance*)compoundCollision;
	//if (compoundInstance->IsType(dgCollision::dgCollisionCompound_RTTI)) {
	//	dgCollisionCompound* const collision = (dgCollisionCompound*)compoundInstance->GetChildShape();
	//	return (NewtonCollision*)collision->GetCollisionFromNode((dgCollisionCompound::dgTreeArray::dgTreeNode*)node);
	//}
	//return NULL;
	ndAssert(0);
	return 0;
}



/*!
  Create a height field collision geometry.

  @param *newtonWorld Pointer to the Newton world.
  @param shapeID fixme

  @return Pointer to the collision.

*/
NewtonCollision* NewtonCreateSceneCollision(const NewtonWorld* const newtonWorld, int shapeID)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//
	//dgCollisionInstance* const collision = world->CreateScene();
	//
	//collision->SetUserDataID(dgUnsigned32(shapeID));
	//return (NewtonCollision*)collision;
	ndAssert(0);
	return nullptr;
}

NewtonCollision* NewtonSceneCollisionGetCollisionFromNode(NewtonCollision* const sceneCollision, const void* const node)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
	//return NewtonCompoundCollisionGetCollisionFromNode(sceneCollision, node);
	return nullptr;
}

void* NewtonSceneCollisionGetFirstNode(NewtonCollision* const sceneCollision)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return NewtonCompoundCollisionGetFirstNode(sceneCollision);
	ndAssert(0);
	return nullptr;
}

void* NewtonSceneCollisionGetNextNode(NewtonCollision* const sceneCollision, const void* const node)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return NewtonCompoundCollisionGetNextNode(sceneCollision, node);
	ndAssert(0);
	return nullptr;
}

void NewtonSceneCollisionBeginAddRemove(NewtonCollision* const sceneCollision)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonCompoundCollisionBeginAddRemove(sceneCollision);
	ndAssert(0);
}

void NewtonSceneCollisionEndAddRemove(NewtonCollision* const sceneCollision)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonCompoundCollisionEndAddRemove(sceneCollision);
	ndAssert(0);
}

void NewtonSceneCollisionSetSubCollisionMatrix(NewtonCollision* const sceneCollision, const void* const collisionNode, const dFloat* const matrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonCompoundCollisionSetSubCollisionMatrix(sceneCollision, collisionNode, matrix);
	ndAssert(0);
}

void* NewtonSceneCollisionAddSubCollision(NewtonCollision* const sceneCollision, const NewtonCollision* const collision)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgCollisionInstance* const sceneInstance = (dgCollisionInstance*)sceneCollision;
	//dgCollisionInstance* const sceneInstanceChild = (dgCollisionInstance*)collision;
	//if (sceneInstance->IsType(dgCollision::dgCollisionScene_RTTI) && !sceneInstanceChild->IsType(dgCollision::dgCollisionCompound_RTTI)) {
	//	dgCollisionScene* const collision1 = (dgCollisionScene*)sceneInstance->GetChildShape();
	//	return collision1->AddCollision(sceneInstanceChild);
	//}
	//return NULL;
	ndAssert(0);
	return nullptr;
}

void NewtonSceneCollisionRemoveSubCollision(NewtonCollision* const sceneCollision, const void* const collisionNode)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const sceneInstance = (dgCollisionInstance*)sceneCollision;
	//if (sceneInstance->IsType(dgCollision::dgCollisionScene_RTTI)) {
	//	dgCollisionScene* const collision = (dgCollisionScene*)sceneInstance->GetChildShape();
	//	dgCollisionInstance* const childCollision = collision->GetCollisionFromNode((dgCollisionCompound::dgTreeArray::dgTreeNode*)collisionNode);
	//	if (childCollision) {
	//		collision->RemoveCollision((dgCollisionCompound::dgTreeArray::dgTreeNode*)collisionNode);
	//	}
	//}
	ndAssert(0);
}

void NewtonSceneCollisionRemoveSubCollisionByIndex(NewtonCollision* const sceneCollision, int nodeIndex)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const instance = (dgCollisionInstance*)sceneCollision;
	//if (instance->IsType(dgCollision::dgCollisionCompound_RTTI)) {
	//	dgCollisionCompound* const collision = (dgCollisionCompound*)instance->GetChildShape();
	//	NewtonSceneCollisionRemoveSubCollision(sceneCollision, collision->FindNodeByIndex(nodeIndex));
	//}
	ndAssert(0);
}

void* NewtonSceneCollisionGetNodeByIndex(NewtonCollision* const sceneCollision, int index)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return NewtonCompoundCollisionGetNodeByIndex(sceneCollision, index);
	ndAssert(0);
	return nullptr;
}

int NewtonSceneCollisionGetNodeIndex(NewtonCollision* const sceneCollision, const void* const collisionNode)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return NewtonCompoundCollisionGetNodeIndex(sceneCollision, collisionNode);
	ndAssert(0);
	return 0;
}
