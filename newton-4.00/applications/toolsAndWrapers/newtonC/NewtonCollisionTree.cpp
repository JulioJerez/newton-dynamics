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


class NewtonCollisionTree : public ndShapeStatic_bvh
{
	public:
	D_CLASS_REFLECTION(NewtonCollisionTree, ndShapeStatic_bvh)
	NewtonCollisionTree()
		:ndShapeStatic_bvh()
		,m_builder(nullptr)
	{
	}

	NewtonCollisionTree(const ndPolygonSoupBuilder& builder)
		:ndShapeStatic_bvh(builder)
		,m_builder(nullptr)
	{
	}

	~NewtonCollisionTree() override
	{
	}

	ndSharedPtr<ndPolygonSoupBuilder> m_builder;
};



/*!
  set a function call back to be called during the face query of a collision tree.

  @param *treeCollision is the pointer to the collision tree.
  @param rayHitCallback pointer to an event function for providing Newton with ray intersection information.

  In general a ray cast on a collision tree will stops at the first intersections with the closest face in the tree
  that was hit by the ray. In some cases the application may be interested in the intesation with faces other than the fiorst hit.
  In this cases the application can set this alternate callback and the ray scanner will notify the application of each face hit by the ray scan.

  since this function faces the ray scanner to visit all of the potential faces intersected by the ray,
  setting the function call back make the ray casting on collision tree less efficient than the default behavior.
  So it is this functionality is only recommended for cases were the application is using especial effects like transparencies, or other effects

  calling this function with *rayHitCallback* = NULL will rest the collision tree to it default raycast mode, which is return with the closest hit.

  when *rayHitCallback* is not null then the callback is dalled with the follwing arguments
  *const NewtonCollisio* collision - pointer to the collision tree
  interseption - inetstion parameters of the ray
  *normal - unnormalized face mormal in the space fo eth parent of the collision.
  faceId -  id of this face in the collision tree.

  See also: ::NewtonTreeCollisionGetFaceAttribute, ::NewtonTreeCollisionSetFaceAttribute
*/
void NewtonTreeCollisionSetUserRayCastCallback(const NewtonCollision* const treeCollision, NewtonCollisionTreeRayCastCallback rayHitCallback)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)treeCollision;
	////	dgCollisionBVH* const collision = (dgCollisionBVH*) treeCollision;
	//if (collision->IsType(dgCollision::dgCollisionBVH_RTTI)) {
	//	dgCollisionBVH* const shape = (dgCollisionBVH*)collision->GetChildShape();
	//	shape->SetCollisionRayCastCallback((dgCollisionBVHUserRayCastCallback)rayHitCallback);
	//}
	ndAssert(0);
}


/*!
  Get the user defined collision attributes stored with each face of the collision mesh.

  @param treeCollision fixme
  @param *faceIndexArray pointer to the face index list passed to the function *NewtonTreeCollisionCallback userCallback
  @param indexCount fixme

  @return User id of the face.

  This function is used to obtain the user data stored in faces of the collision geometry.
  The application can use this user data to achieve per polygon material behavior in large static collision meshes.

  See also: ::NewtonTreeCollisionSetFaceAttribute, ::NewtonCreateTreeCollision
*/
int NewtonTreeCollisionGetFaceAttribute(const NewtonCollision* const treeCollision, const int* const faceIndexArray, int indexCount)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionBVH* const collision = (dgCollisionBVH*)((dgCollisionInstance*)treeCollision)->GetChildShape();
	//dgAssert(collision->IsType(dgCollision::dgCollisionBVH_RTTI));
	//
	//return int(collision->GetTagId(faceIndexArray, indexCount));
	ndAssert(0);
	return 0;
}

/*!
  Change the user defined collision attribute stored with faces of the collision mesh.

  @param *treeCollision fixme
  @param *faceIndexArray pointer to the face index list passed to the NewtonTreeCollisionCallback function
  @param indexCount fixme
  @param attribute value of the user defined attribute to be stored with the face.

  @return User id of the face.

  This function is used to obtain the user data stored in faces of the collision geometry.
  The application can use this user data to achieve per polygon material behavior in large static collision meshes.
  By changing the value of this user data the application can achieve modifiable surface behavior with the collision geometry.
  For example, in a driving game, the surface of a polygon that represents the street can changed from pavement to oily or wet after
  some collision event occurs.

  See also: ::NewtonTreeCollisionGetFaceAttribute, ::NewtonCreateTreeCollision
*/
void NewtonTreeCollisionSetFaceAttribute(const NewtonCollision* const treeCollision, const int* const faceIndexArray, int indexCount, int attribute)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionBVH* const collision = (dgCollisionBVH*)((dgCollisionInstance*)treeCollision)->GetChildShape();
	//dgAssert(collision->IsType(dgCollision::dgCollisionBVH_RTTI));
	//
	//collision->SetTagId(faceIndexArray, indexCount, dgUnsigned32(attribute));
	ndAssert(0);
}

void NewtonTreeCollisionForEachFace(const NewtonCollision* const treeCollision, NewtonTreeCollisionFaceCallback forEachFaceCallback, void* const context)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionBVH* const collision = (dgCollisionBVH*)((dgCollisionInstance*)treeCollision)->GetChildShape();
	//dgAssert(collision->IsType(dgCollision::dgCollisionBVH_RTTI));
	//
	//collision->ForEachFace((dgAABBIntersectCallback)forEachFaceCallback, context);
	ndAssert(0);
}



/*!
  collect the vertex list index list mesh intersecting the AABB in collision mesh.

  @param *treeCollision fixme
  @param  *p0 - pointer to an array of at least three floats representing the ray origin in the local space of the geometry.
  @param  *p1 - pointer to an array of at least three floats representing the ray end in the local space of the geometry.
  @param **vertexArray pointer to a the vertex array of vertex.
  @param *vertexCount pointer int to return the number of vertex in vertexArray.
  @param *vertexStrideInBytes pointer to int to return the size of each vertex in vertexArray.
  @param *indexList pointer to array on integers containing the triangles intersection the aabb.
  @param maxIndexCount maximum number of indices the function will copy to indexList.
  @param *faceAttribute pointer to array on integers top contain the face containing the .

  @return the number of triangles in indexList.

  indexList should be a list 3 * maxIndexCount the number of elements.

  faceAttributet should be a list maxIndexCount the number of elements.

  this function could be used by the application for many purposes.
  for example it can be used to draw the collision geometry intersecting a collision primitive instead
  of drawing the entire collision tree in debug mode.
  Another use for this function is to to efficient draw projective texture shadows.
*/
int NewtonTreeCollisionGetVertexListTriangleListInAABB(const NewtonCollision* const treeCollision, const dFloat* const p0, const dFloat* const p1,
	const dFloat** const vertexArray, int* const vertexCount, int* const vertexStrideInBytes,
	const int* const indexList, int maxIndexCount, const int* const faceAttribute)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgInt32 count = 0;
	//dgCollisionInstance* meshColl = (dgCollisionInstance*)treeCollision;
	//if (meshColl->IsType(dgCollision::dgCollisionMesh_RTTI)) {
	//	dgCollisionMesh* const collision = (dgCollisionMesh*)((dgCollisionInstance*)treeCollision)->GetChildShape();
	//
	//	dgVector pmin(p0[0], p0[1], p0[2], dgFloat32(0.0f));
	//	dgVector pmax(p1[0], p1[1], p1[2], dgFloat32(0.0f));
	//
	//	dgCollisionMesh::dgMeshVertexListIndexList data;
	//	data.m_indexList = (dgInt32*)indexList;
	//	data.m_userDataList = (dgInt32*)faceAttribute;
	//	data.m_maxIndexCount = maxIndexCount;
	//	data.m_triangleCount = 0;
	//	collision->GetVertexListIndexList(pmin, pmax, data);
	//
	//	count = data.m_triangleCount;
	//	*vertexArray = data.m_veterxArray;
	//	*vertexCount = data.m_vertexCount;
	//	*vertexStrideInBytes = data.m_vertexStrideInBytes;
	//}
	//return count;
	ndAssert(0);
	return 0;
}


/*!
  Create an empty complex collision geometry tree.

  @param *newtonWorld Pointer to the Newton world.
  @param shapeID fixme

  @return Pointer to the collision tree.

  *TreeCollision* is the preferred method within Newton for collision with polygonal meshes of arbitrary complexity.
  The mesh must be made of flat non-intersecting polygons, but they do not explicitly need to be triangles.
  *TreeCollision* can be serialized by the application to/from an arbitrary storage device.

  When a *TreeCollision* is assigned to a body the mass of the body is ignored in all dynamics calculations.
  This makes the body behave as a static body.

  See also: ::NewtonTreeCollisionBeginBuild, ::NewtonTreeCollisionAddFace, ::NewtonTreeCollisionEndBuild, ::NewtonStaticCollisionSetDebugCallback, ::NewtonTreeCollisionGetFaceAttribute, ::NewtonTreeCollisionSetFaceAttribute
*/
NewtonCollision* NewtonCreateTreeCollision(const NewtonWorld* const newtonWorld, int shapeID)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = new ndShapeInstance(new NewtonCollisionTree());

	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	return reinterpret_cast<NewtonCollision*>(instance);
}

NewtonCollision* NewtonCreateTreeCollisionFromMesh(const NewtonWorld* const, const NewtonMesh* const mesh, int shapeID)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);

	ndShapeInstance* const instance = meshEffect->CreateCollisionTree(false);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;

	return reinterpret_cast<NewtonCollision*>(instance);
}


/*!
  Prepare a *TreeCollision* to begin to accept the polygons that comprise the collision mesh.

  @param *treeCollision is the pointer to the collision tree.

  @return Nothing.

  See also: ::NewtonTreeCollisionAddFace, ::NewtonTreeCollisionEndBuild
*/
void NewtonTreeCollisionBeginBuild(const NewtonCollision* const treeCollision)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = const_cast<ndShapeInstance*>(reinterpret_cast<const ndShapeInstance*>(treeCollision));
	NewtonCollisionTree* const shape = static_cast<NewtonCollisionTree*>(instance->GetShape()->GetAsShapeStaticBVH());
	ndAssert(strcmp(shape->ClassName(), NewtonCollisionTree::StaticClassName()) == 0);
	ndAssert(shape);

	shape->m_builder = ndSharedPtr<ndPolygonSoupBuilder>(new ndPolygonSoupBuilder);
	shape->m_builder->Begin();
}


/*!
  Add an individual polygon to a *TreeCollision*.

  @param *treeCollision is the pointer to the collision tree.
  @param vertexCount number of vertex in *vertexPtr*
  @param *vertexPtr pointer to an array of vertex. The vertex should consist of at least 3 floats each.
  @param strideInBytes size of each vertex in bytes. This value should be 12 or larger.
  @param faceAttribute id that identifies the polygon. The application can use this value to customize the behavior of the collision geometry.

  @return Nothing.

  After the call to *NewtonTreeCollisionBeginBuild* the *TreeCollision* is ready to accept polygons. The application should iterate
  through the application's mesh, adding the mesh polygons to the *TreeCollision* one at a time.
  The polygons must be flat and non-self intersecting.

  See also: ::NewtonTreeCollisionAddFace, ::NewtonTreeCollisionEndBuild
*/
void NewtonTreeCollisionAddFace(const NewtonCollision* const treeCollision, int vertexCount, const dFloat* const vertexPtr, int strideInBytes, int faceAttribute)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = const_cast<ndShapeInstance*>(reinterpret_cast<const ndShapeInstance*>(treeCollision));
	NewtonCollisionTree* const shape = static_cast<NewtonCollisionTree*>(instance->GetShape()->GetAsShapeStaticBVH());
	ndAssert(strcmp(shape->ClassName(), NewtonCollisionTree::StaticClassName()) == 0);
	ndAssert(shape);

	ndFixSizeArray<ndVector, 64> points;
	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		ndInt32 offset = i * strideInBytes / sizeof(dFloat);
		ndVector p(vertexPtr[offset + 0], vertexPtr[offset + 1], vertexPtr[offset + 2], dFloat(1.0f));
		points.PushBack(p);
	}
	shape->m_builder->AddFace(&points[0], points.GetCount(), faceAttribute);
}

/*!
  Finalize the construction of the polygonal mesh.

  @param *treeCollision is the pointer to the collision tree.
  @param optimize flag that indicates to Newton whether it should optimize this mesh. Set to 1 to optimize the mesh, otherwise 0.

  @return Nothing.


  After the application has finished adding polygons to the *TreeCollision*, it must call this function to finalize the construction of the collision mesh.
  If concave polygons are added to the *TreeCollision*, the application must call this function with the parameter *optimize* set to 1.
  With the *optimize* parameter set to 1, Newton will optimize the collision mesh by removing non essential edges from adjacent flat polygons.
  Newton will not change the topology of the mesh but significantly reduces the number of polygons in the mesh. The reduction factor of the number of polygons in the mesh depends upon the irregularity of the mesh topology.
  A reduction factor of 1.5 to 2.0 is common.
  Calling this function with the parameter *optimize* set to zero, will leave the mesh geometry unaltered.

  See also: ::NewtonTreeCollisionAddFace, ::NewtonTreeCollisionEndBuild
*/
void NewtonTreeCollisionEndBuild(const NewtonCollision* const treeCollision, int optimize)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = const_cast<ndShapeInstance*>(reinterpret_cast<const ndShapeInstance*>(treeCollision));
	NewtonCollisionTree* const shape = static_cast<NewtonCollisionTree*>(instance->GetShape()->GetAsShapeStaticBVH());
	ndAssert(strcmp(shape->ClassName(), NewtonCollisionTree::StaticClassName()) == 0);
	ndAssert(shape);
	shape->m_builder->End(optimize);

	NewtonCollisionTree* const newShape = new NewtonCollisionTree(**shape->m_builder);
	instance->SetShape(newShape);
}
