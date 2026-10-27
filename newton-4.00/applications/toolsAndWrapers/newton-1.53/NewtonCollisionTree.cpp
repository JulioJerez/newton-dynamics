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

class NewtonCollisionTree : public ndShapeStatic_bvh
{
	typedef void (*NewtonTreeCollisionCallback) (const NewtonBody* bodyWithTreeCollision, const NewtonBody* body,
		const dFloat* vertex, int vertexstrideInBytes,
		int indexCount, const int* indexArray);

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
		,m_callback(nullptr)
	{
	}

	~NewtonCollisionTree() override
	{
	}

	ndSharedPtr<ndPolygonSoupBuilder> m_builder;
	NewtonTreeCollisionCallback m_callback;
};


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
//NewtonCollision* NewtonCreateTreeCollision(const NewtonWorld* const, int shapeID)
NewtonCollision* NewtonCreateTreeCollision(const NewtonWorld* newtonWorld, NewtonTreeCollisionCallback userCallback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = new ndShapeInstance(new NewtonCollisionTree());
	NewtonCollisionTree* const shape = PointerCast<NewtonCollisionTree, ndShapeStatic_bvh>(instance->GetShape()->GetAsShapeStaticBVH());

	shape->m_callback = userCallback;
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
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
	ndShapeInstance* const instance = PointerCast< ndShapeInstance, NewtonCollision>(treeCollision);
	NewtonCollisionTree* const shape = PointerCast<NewtonCollisionTree, ndShapeStatic_bvh>(instance->GetShape()->GetAsShapeStaticBVH());
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
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(treeCollision);
	NewtonCollisionTree* const shape = PointerCast<NewtonCollisionTree, ndShapeStatic_bvh>(instance->GetShape()->GetAsShapeStaticBVH());
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
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(treeCollision);
	NewtonCollisionTree* const shape = PointerCast<NewtonCollisionTree, ndShapeStatic_bvh>(instance->GetShape()->GetAsShapeStaticBVH());
	ndAssert(strcmp(shape->ClassName(), NewtonCollisionTree::StaticClassName()) == 0);
	ndAssert(shape);
	shape->m_builder->End(optimize);
	instance->m_shape = ndSharedPtr<ndShape>(new NewtonCollisionTree(**shape->m_builder));
}

int NewtonTreeCollisionGetVertexListIndexListInAABB(const NewtonCollision* treeCollision, const dFloat* p0, const dFloat* p1,
	const dFloat** vertexArray, int* vertexCount, int* vertexStrideInBytes,
	const int* indexList, int maxIndexCount)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndTrace(("TO DO: return 0 for now\n"));
	return 0;
}
