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

static void MapShapeId(const ndShapeInfo& info, NewtonCollisionInfoRecord& collisionInfo)
{
	collisionInfo.m_collisionMaterial.m_userId = info.m_shapeMaterial.m_userId;
	collisionInfo.m_collisionMaterial.m_userData.m_ptr = info.m_shapeMaterial.m_data.m_userData;

	const ndInt32 count0 = sizeof(info.m_shapeMaterial.m_userParam) / sizeof(info.m_shapeMaterial.m_userParam[0]);
	const ndInt32 count1 = sizeof(collisionInfo.m_collisionMaterial.m_userParam) / sizeof(collisionInfo.m_collisionMaterial.m_userParam[0]);
	const ndInt32 count = ndMin(count0, count1);
	for (ndInt32 i = 0; i < count; ++i)
	{
		collisionInfo.m_collisionMaterial.m_userParam[i].m_ptr = info.m_shapeMaterial.m_userParam[i].m_ptrData;
	}

	switch (info.m_collisionType)
	{
		case m_sphere:
		{
			collisionInfo.m_collisionType = SERIALIZE_ID_SPHERE;
			collisionInfo.m_sphere.m_radio = info.m_sphere.m_radius;
			break;
		}

		case m_capsule:
		{
			collisionInfo.m_collisionType = SERIALIZE_ID_CAPSULE;
			collisionInfo.m_capsule.m_radio0 = info.m_capsule.m_radio0;
			collisionInfo.m_capsule.m_radio1 = info.m_capsule.m_radio1;
			collisionInfo.m_capsule.m_height = info.m_capsule.m_height;
			break;
		}

		case m_cylinder:
		{
			collisionInfo.m_collisionType = SERIALIZE_ID_CYLINDER;
			collisionInfo.m_cylinder.m_radio0 = info.m_cylinder.m_radio0;
			collisionInfo.m_cylinder.m_radio1 = info.m_cylinder.m_radio1;
			collisionInfo.m_cylinder.m_height = info.m_cylinder.m_height;
			break;
		}

		case m_chamferCylinder:
		{
			collisionInfo.m_collisionType = SERIALIZE_ID_CHAMFERCYLINDER;
			collisionInfo.m_chamferCylinder.m_radio = info.m_chamferCylinder.m_radius;
			collisionInfo.m_chamferCylinder.m_height = info.m_chamferCylinder.m_height;
			break;
		}

		case m_box:
		{
			collisionInfo.m_collisionType = SERIALIZE_ID_BOX;
			collisionInfo.m_box.m_x = info.m_box.m_x;
			collisionInfo.m_box.m_y = info.m_box.m_y;
			collisionInfo.m_box.m_z = info.m_box.m_z;
			break;
		}

		case m_cone:
		{
			collisionInfo.m_collisionType = SERIALIZE_ID_CONE;
			collisionInfo.m_cone.m_radio = info.m_cone.m_radius;
			collisionInfo.m_cone.m_height = info.m_cone.m_height;
			break;
		}

		case m_convexHull:
		{
			collisionInfo.m_collisionType = SERIALIZE_ID_CONVEXHULL;
			collisionInfo.m_convexHull.m_faceCount = info.m_convexhull.m_faceCount;
			collisionInfo.m_convexHull.m_vertex = &info.m_convexhull.m_vertex[0].m_x;
			collisionInfo.m_convexHull.m_vertexCount = info.m_convexhull.m_vertexCount;
			collisionInfo.m_convexHull.m_vertexStrideInBytes = info.m_convexhull.m_strideInBytes;
			break;
		}

		case m_compound:
		{
			collisionInfo.m_collisionType = SERIALIZE_ID_COMPOUND;
			collisionInfo.m_compoundCollision.m_chidrenCount = info.m_compound.m_noUsed;
			break;
		}

		case m_boundingBoxHierachy:
		{
			collisionInfo.m_collisionType = SERIALIZE_ID_TREE;
			collisionInfo.m_collisionTree.m_indexCount = info.m_bvh.m_indexCount;
			collisionInfo.m_collisionTree.m_vertexCount = info.m_bvh.m_vertexCount;
			break;
		}

		case m_heightField:
		{
			collisionInfo.m_collisionType = SERIALIZE_ID_HEIGHTFIELD;
			collisionInfo.m_heightField.m_width = info.m_heightfield.m_width;
			collisionInfo.m_heightField.m_height = info.m_heightfield.m_height;
			collisionInfo.m_heightField.m_gridsDiagonals = info.m_heightfield.m_gridsDiagonals;
			//collisionInfo.m_heightField.m_elevationDataType = 0info.m_heightfield.m_elevationDataType;
			collisionInfo.m_heightField.m_elevationDataType = 0;
			collisionInfo.m_heightField.m_verticalScale = ndFloat32 (1.0f);
			collisionInfo.m_heightField.m_horizonalScale_x = info.m_heightfield.m_horizonalScale_x;
			collisionInfo.m_heightField.m_horizonalScale_z = info.m_heightfield.m_horizonalScale_z;
			collisionInfo.m_heightField.m_atributes = PointerCast<char, ndInt8>(info.m_heightfield.m_atributes);
			collisionInfo.m_heightField.m_vertialElevation = PointerCast<float, ndReal>(info.m_heightfield.m_elevation);
			break;
		}

		case m_staticProceduralMesh:
		{
			collisionInfo.m_collisionType = SERIALIZE_ID_USERMESH;
			break;
		}

		default:
			ndAssert(0);
	}
}

class NewtonProceduralCollition : public ndShapeStaticProceduralMesh
{
	public:
	D_CLASS_REFLECTION(NewtonProceduralCollitiuon, ndShapeStaticProceduralMesh)

	typedef void (*NewtonUserMeshCollisionDestroyCallback) (void* const userData);
	typedef dFloat(*NewtonUserMeshCollisionRayHitCallback) (NewtonUserMeshCollisionRayHitDesc* const lineDescData);
	typedef void (*NewtonUserMeshCollisionGetCollisionInfo) (void* const userData, NewtonCollisionInfoRecord* const infoRecord);
	typedef int (*NewtonUserMeshCollisionAABBTest) (void* const userData, const dFloat* const boxP0, const dFloat* const boxP1);
	typedef void (*NewtonUserMeshCollisionCollideCallback) (NewtonUserMeshCollisionCollideDesc* const collideDescData, const void* const continueCollisionHandle);
	typedef void (*NewtonOnUserCollisionSerializationCallback) (void* const userData, NewtonSerializeCallback function, void* const serializeHandle);
	typedef int (*NewtonUserMeshCollisionGetFacesInAABB) (
		void* const userData, const dFloat* const p0, const dFloat* const p1,
		const dFloat** const vertexArray, int* const vertexCount, int* const vertexStrideInBytes,
		const int* const indexList, int maxIndexCount, const int* const userDataList);


	NewtonProceduralCollition(void* const userData, const ndVector& box0, const ndVector& box1)
		:ndShapeStaticProceduralMesh()
		,m_userData(userData)
		,m_onDestroy(nullptr)
		,m_box0(box0)
		,m_box1(box1)
	{
		m_boxSize = (m_box1 - m_box0) * ndVector::m_half;
		m_boxOrigin = (m_box1 + m_box0) * ndVector::m_half;
	}

	virtual ~NewtonProceduralCollition() override
	{
		if (m_onDestroy)
		{
			m_onDestroy(PointerCast<void, void>(*m_userData));
		}
	}

	virtual void CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const override
	{
		ndAssert(0);
	}

	virtual void GetFacesPatch(ndPatchMesh& patch) const override
	{
		ndVector minBox;
		ndVector maxBox;
		if (m_onGetFacePatch && ndBoxBoxIntersection(minBox, maxBox, m_box0, m_box1, patch.m_boxP0, patch.m_boxP1))
		{
			NewtonUserMeshCollisionCollideDesc collideDescData;
			//typedef struct NewtonUserMeshCollisionCollideDesc
			//{
			//	int	m_faceCount;                        	// the application should set here how many polygons intersect the query box
			//	int m_vertexStrideInBytes;              	// the application should set here the size of each vertex
			//	dFloat m_skinThickness;                     // this is the minimum skin separation specified by the material between these two colliding shapes
			//	void* m_userData;                       	// user data passed to the collision geometry at creation time
			//
			//	NewtonBody* m_objBody;                  	// pointer to the colliding body
			//	NewtonBody* m_polySoupBody;             	// pointer to the rigid body owner of this collision tree 
			//	NewtonCollision* m_objCollision;			// collision shape of the colliding body, (no necessarily the collision of m_objBody)
			//	NewtonCollision* m_polySoupCollision;		// collision shape of the collision tree, (no necessarily the collision of m_polySoupBody)
			//
			//	dFloat* m_vertex;                       	// the application should set here the pointer to the global vertex of the mesh. 
			//	int* m_faceIndexCount;                  	// the application should set here the pointer to the vertex count of each face.
			//	int* m_faceVertexIndex;                 	// the application should set here the pointer index array for each vertex on a face.
			//	// the format of a face is I0, I1, I2, I3, ..., M, N, E0, E1, E2, ..., A
			//	// I0, I1, I2, .. are the indices to the vertex, relative to m_vertex pointer
			//	// M is the index to the material sub shape id
			//	// N in the index to the vertex normal relative to m_vertex pointer
			//	// E0, E1, E2, ... are the indices of the the face normal that is shared to that face edge, when the edge does not share a face normal then the edge index is set to index N, which the index to the face normal    
			//	// A is and estimate of the largest diagonal of the face, this used internally as a hint to improve floating point accuracy and algorithm performance. 
			//} NewtonUserMeshCollisionCollideDesc;

			ndMemSet(PointerCast<char, NewtonUserMeshCollisionCollideDesc>(&collideDescData), char(0), sizeof(NewtonUserMeshCollisionCollideDesc));
			for (ndInt32 i = 0; i < 4; ++i)
			{
				collideDescData.m_boxP0[i] = minBox[i];
				collideDescData.m_boxP1[i] = maxBox[i];
				collideDescData.m_boxDistanceTravel[i] = ndFloat32(0.0f);
			}
			collideDescData.m_threadNumber = 0;
			collideDescData.m_userData = PointerCast<void,void>(*m_userData);
			collideDescData.m_skinThickness = ndFloat32(0.0f);

			collideDescData.m_objBody = nullptr;                  	// pointer to the colliding body
			collideDescData.m_polySoupBody = nullptr;             	// pointer to the rigid body owner of this collision tree 
			collideDescData.m_polySoupCollision = nullptr;		// collision shape of the collision tree, (no necessarily the collision of m_polySoupBody)
			collideDescData.m_objCollision = PointerCast<NewtonCollision, ndShapeInstance>(patch.m_convexShapeInstance);
			m_onGetFacePatch(&collideDescData, nullptr);
			if (collideDescData.m_faceCount)
			{
				if (patch.m_queryType == patch.m_vertexListOnly)
				{
					patch.m_pointArray.PushBack(ndVector::m_zero);
				}
				else
				{
					// the application should set here the pointer index array for each vertex on a face.
					//int* m_faceVertexIndex;                 	
					//the format of a face is I0, I1, I2, I3, ..., M, N, E0, E1, E2, ..., A
					//I0, I1, I2, .. are the indices to the vertex, relative to m_vertex pointer
					//M is the index to the material sub shape id
					//N in the index to the vertex normal relative to m_vertex pointer
					//E0, E1, E2, ... are the indices of the the face normal that is shared to that face edge, when the edge does not share a face normal then the edge index is set to index N, which the index to the face normal    
					//A is and estimate of the largest diagonal of the face, this used internally as a hint to improve floating point accuracy and algorithm performance. 

					ndInt32 faceIndex = 0;
					patch.m_vertexArrayHasDuplicated = true;
					const ndFloat32* const vertex = collideDescData.m_vertex;
					const ndInt32* const indexArray = collideDescData.m_faceVertexIndex;
					const ndInt32* const facesIndexArray = collideDescData.m_faceIndexCount;
					const ndInt32 stride = collideDescData.m_vertexStrideInBytes / sizeof(ndFloat32);
					for (ndInt32 i = 0; i < collideDescData.m_faceCount; ++i)
					{
						const ndInt32 indexCount = facesIndexArray[i];
						patch.m_faceArray.PushBack(indexCount);
						patch.m_faceMaterialArray.PushBack(indexArray[faceIndex + indexCount]);

						ndInt32 n = stride * indexArray[faceIndex + indexCount + 1];
						const ndVector normal(vertex[n + 0], vertex[n + 1], vertex[n + 2], ndFloat32(0.0f));
						patch.m_normalArray.PushBack(normal);
						for (ndInt32 j = 0; j < indexCount; ++j)
						{
							ndInt32 k = stride * indexArray[faceIndex + j];
							const ndVector point(vertex[k + 0], vertex[k + 1], vertex[k + 2], ndFloat32(1.0f));
							patch.m_indexArray.PushBack(patch.m_pointArray.GetCount());
							patch.m_pointArray.PushBack(point);
						}
						faceIndex += 2 * (indexCount + 1);
					}
				}
			}
		}
	}

	virtual ndFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
	{
		if (m_onRaycast)
		{
			//typedef struct NewtonUserMeshCollisionRayHitDesc
			//{
			//	dFloat m_p0[4];							// ray origin in collision local space
			//	dFloat m_p1[4];                         // ray destination in collision local space   
			//	dFloat m_normalOut[4];					// copy here the normal at the ray intersection
			//	dLong m_userIdOut;						// copy here a user defined id for further feedback  
			//	void* m_userData;                       // user data passed to the collision geometry at creation time
			//} NewtonUserMeshCollisionRayHitDesc;

			NewtonUserMeshCollisionRayHitDesc desc;
			desc.m_userData = PointerCast<void, void>(*m_userData);
			for (ndInt32 i = 0; i < 4; ++i)
			{
				desc.m_p0[i] = localP0[i];
				desc.m_p1[i] = localP1[i];
			}

			ndFloat32 param = m_onRaycast(&desc);
			if (param < ndFloat32(1.0f))
			{
				contactOut.m_shapeId0 = desc.m_userIdOut;
				contactOut.m_normal.m_x = desc.m_normalOut[0];
				contactOut.m_normal.m_y = desc.m_normalOut[1];
				contactOut.m_normal.m_z = desc.m_normalOut[2];
				contactOut.m_normal.m_w = ndFloat32 (0.0f);
			}
			return param;
		}
		return ndFloat32 (1.2f);
	}

	ndWeakPtr<void> m_userData;
	NewtonUserMeshCollisionRayHitCallback m_onRaycast;
	NewtonUserMeshCollisionDestroyCallback m_onDestroy;
	NewtonUserMeshCollisionCollideCallback m_onGetFacePatch;

	//typedef void (*NewtonUserMeshCollisionGetCollisionInfo) (void* const userData, NewtonCollisionInfoRecord* const infoRecord);
	//typedef int (*NewtonUserMeshCollisionAABBTest) (void* const userData, const dFloat* const boxP0, const dFloat* const boxP1);
	//typedef void (*NewtonUserMeshCollisionCollideCallback) (NewtonUserMeshCollisionCollideDesc* const collideDescData, const void* const continueCollisionHandle);
	//typedef void (*NewtonOnUserCollisionSerializationCallback) (void* const userData, NewtonSerializeCallback function, void* const serializeHandle);

	ndVector m_box0;
	ndVector m_box1;
};

/*!
  Release a reference from this collision object returning control to Newton.

  @param *collisionPtr pointer to the collision object

  @return Nothing.

  to get the correct reference count of a collision primitive the application can call function *NewtonCollisionGetInfo*

*/
void NewtonDestroyCollision(const NewtonCollision* const collision)
{
	TRACE_FUNCTION(__FUNCTION__);
	const ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);
	delete instance;
}

void NewtonCollisionSetMatrix(const NewtonCollision* collision, const dFloat* const matrixPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);

	const ndMatrix matrix(GetMatrix(matrixPtr));
	instance->SetLocalMatrix(matrix);
}

void NewtonCollisionGetMatrix(const NewtonCollision* const collision, dFloat* const matrixPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	const ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);

	const ndMatrix instanceMatrix(instance->GetLocalMatrix());
	ndMemCpy(matrixPtr, &instanceMatrix[0][0], sizeof(ndMatrix) / sizeof(ndFloat32));
}

/*!
  Create a transparent collision primitive.

  @param *newtonWorld Pointer to the Newton world.

  @return Pointer to the collision object.

  Some times the application needs to create helper rigid bodies that will never collide with other bodies,
  for example the neck of a rag doll, or an internal part of an articulated structure. This can be done by using the material system
  but it too much work and it will increase unnecessarily the material count, and therefore the project complexity. The Null collision
  is a collision object that satisfy all this conditions without having to change the engine philosophy.

*/
NewtonCollision* NewtonCreateNull(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeNull());
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
}

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
	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeBox(dx, dy, dz));
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	instance->SetMaterial(material);
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
}

/*!
  Create a generalized ellipsoid primitive..

  @param *newtonWorld Pointer to the Newton world.
  @param radius sphere radius
  @param shapeID user specified collision index that can be use for multi material collision.
  @param *offsetMatrix pointer to an array of 16 floats containing the offset matrix of the sphere relative to the body. If this parameter is NULL then the sphere is centered at the origin of the body.

  @return Pointer to the generalized sphere.

  Sphere collision are generalized ellipsoids, the application can create many different kind of objects by just playing with dimensions of the radius.
  for example to make a sphere set all tree radius to the same value, to make a ellipse of revolution just set two of the tree radius to the same value.

  General ellipsoids are very good hull geometries to represent the outer shell of avatars in a game.

*/
NewtonCollision* NewtonCreateSphere(const NewtonWorld* const, dFloat radius, int shapeID, const dFloat* const offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}
	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeSphere(radius));
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	instance->SetMaterial(material);
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
}

/*!
  Create a cone primitive for collision.

  @param *newtonWorld Pointer to the Newton world.
  @param radius cone radius at the base.
  @param height cone height along the x local axis from base to tip.
  @param shapeID user specified collision index that can be use for multi material collision.
  @param *offsetMatrix pointer to an array of 16 floats containing the offset matrix of the box relative to the body. If this parameter is NULL, then the primitive is centered at the origin of the body.

  @return Pointer to the box

*/
NewtonCollision* NewtonCreateCone(const NewtonWorld* const,  dFloat radius, dFloat height, int shapeID, const dFloat* const offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}
	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeCone(radius, height));
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	instance->SetMaterial(material);
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
}

/*!
  Create a capsule primitive for collision.

  @param *newtonWorld Pointer to the Newton world.
  @param  radio0 - fixme
  @param  radio1 - fixme
  @param height capsule height along the x local axis from tip to tip.
  @param shapeID fixme
  @param *offsetMatrix pointer to an array of 16 floats containing the offset matrix of the box relative to the body. If this parameter is NULL, then the primitive is centered at the origin of the body.

  @return Pointer to the box

  the capsule height must equal of larger than the sum of the cap radius. If this is not the case the height will be clamped the 2 * radius.

*/
NewtonCollision* NewtonCreateCapsule(const NewtonWorld* const,  dFloat radio0, dFloat radio1, dFloat height, int shapeID, const dFloat* const offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}
	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeCapsule (radio0, radio1, height));
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	instance->SetMaterial(material);
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
}

/*!
  Create a cylinder primitive for collision.

  @param *newtonWorld Pointer to the Newton world.
  @param  radio0 - fixme
  @param  radio1 - fixme
  @param height cylinder height along the x local axis.
  @param shapeID fixme
  @param *offsetMatrix pointer to an array of 16 floats containing the offset matrix of the box relative to the body. If this parameter is NULL, then the primitive is centered at the origin of the body.

  @return Pointer to the box

*/
NewtonCollision* NewtonCreateCylinder(const NewtonWorld* const,  dFloat radio0, dFloat radio1, dFloat height, int shapeID, const dFloat* const offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}
	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeCylinder(radio0, radio1, height));
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	instance->SetMaterial(material);
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
}

/*!
  Create a ChamferCylinder primitive for collision.

  @param *newtonWorld Pointer to the Newton world.
  @param radius ChamferCylinder radius at the base.
  @param height ChamferCylinder height along the x local axis.
  @param shapeID fixme
  @param *offsetMatrix pointer to an array of 16 floats containing the offset matrix of the box relative to the body. If this parameter is NULL, then the primitive is centered at the origin of the body.

  @return Pointer to the box

*/
NewtonCollision* NewtonCreateChamferCylinder(const NewtonWorld* const,  dFloat radius, dFloat height, int shapeID, const dFloat* const offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}
	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeChamferCylinder(radius, height));
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	instance->SetMaterial(material);
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
}


/*!
  Create a ConvexHull primitive from collision from a cloud of points.

  @param *newtonWorld Pointer to the Newton world.
  @param count number of consecutive point to follow must be at least 4.
  @param *vertexCloud pointer to and array of point.
  @param strideInBytes vertex size in bytes, must be at least 12.
  @param tolerance tolerance value for the hull generation.
  @param shapeID fixme
  @param *offsetMatrix pointer to an array of 16 floats containing the offset matrix of the box relative to the body. If this parameter is NULL, then the primitive is centered at the origin of the body.

  @return Pointer to the collision mesh, NULL if the function fail to generate convex shape

  Convex hulls are the solution to collision primitive that can not be easily represented by an implicit solid.
  The implicit solid primitives (spheres, cubes, cylinders, capsules, cones, etc.), have constant time complexity for contact calculation
  and are also extremely efficient on memory usage, therefore the application get perfect smooth behavior.
  However for cases where the shape is too difficult or a polygonal representation is desired, convex hulls come closest to the to the model shape.
  For example it is a mistake to model a 10000 point sphere as a convex hull when the perfect sphere is available, but it is better to represent a
  pyramid by a convex hull than with a sphere or a box.

  There is not upper limit as to how many vertex the application can pass to make a hull shape,
  however for performance and memory usage concern it is the application responsibility to keep the max vertex at the possible minimum.
  The minimum number of vertex should be equal or larger than 4 and it is the application responsibility that the points are part of a solid geometry.
  Unpredictable results will occur if all points happen to be collinear or coplanar.

  The performance of collision with convex hull proxies is sensitive to the vertex count of the hull. Since a the convex hull
  of a visual geometry is already an approximation of the mesh, for visual purpose there is not significant difference between the
  appeal of a exact hull and one close to the exact hull but with but with a smaller vertex count.
  It just happens that sometime complex meshes lead to generation of convex hulls with lots of small detail that play not
  roll of the quality of the simulation but that have a significant impact on the performance because of a large vertex count.
  For this reason the application have the option to set a *tolerance* parameter.
  *tolerance* is use to post process the final geometry in the following faction, a point on the surface of the hull can
  be remove if the distance of all of the surrounding vertex immediately adjacent to the average plane equation formed the
  faces adjacent to that point, is smaller than the tolerance. A value of zero in *tolerance* will generate an exact hull and a value langer that zero
  will generate a loosely fitting hull and it willbe faster to generate.

*/
NewtonCollision* NewtonCreateConvexHull(const NewtonWorld* const,  int count, const dFloat* const vertexCloud, int strideInBytes, dFloat tolerance, int shapeID, const dFloat* const offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}
	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeConvexHull(count, strideInBytes, tolerance, vertexCloud));
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	instance->SetMaterial(material);
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
}

/*!
  Create a ConvexHull primitive from a special effect mesh.

  @param *newtonWorld Pointer to the Newton world.
  @param *mesh special effect mesh
  @param tolerance tolerance value for the hull generation.
  @param shapeID fixme

  @return Pointer to the collision mesh, NULL if the function fail to generate convex shape

  Because the in general this function is used for runtime special effect like debris and or solid particles
  it is recommended that the source mesh complexity is kept small.

  See also: ::NewtonCreateConvexHull, ::NewtonMeshCreate
*/
NewtonCollision* NewtonCreateConvexHullFromMesh(const NewtonWorld* const,  const NewtonMesh* const mesh, dFloat tolerance, int shapeID)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = PointerCast<ndMeshEffect, NewtonMesh>(mesh);

	ndShapeInstance* const instance = meshEffect->CreateConvexCollision(tolerance);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	instance->SetMaterial(material);
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
}


/*!
  Create a height field collision geometry.

  @param *newtonWorld Pointer to the Newton world.
  @param width the number of sample points in the x direction (fixme)
  @param height the number of sample points in the y direction (fixme)
  @param gridsDiagonals fixme
  @param elevationdatType fixme
  @param elevationMap array holding elevation data of size = width*height (fixme)
  @param attributeMap array holding attribute data of size = width*height (fixme)
  @param verticalScale scale of the elevation (fixme)
  @param horizontalScale scale in the xy direction. (fixme)
  @param shapeID fixme

  @return Pointer to the collision.

  NewtonCollision* NewtonCreateHeightFieldCollision(const NewtonWorld* const,  int width, int height, int cellsDiagonals,
  const dFloat* const elevationMap, const char* const atributeMap,
  dFloat horizontalScale, int shapeID)
*/
NewtonCollision* NewtonCreateHeightFieldCollision(const NewtonWorld* const, int width, int height, int gridsDiagonals, int elevationdatType,
	const void* const elevationMap, const char* const attributeMap, dFloat verticalScale, dFloat horizontalScale_x, dFloat horizontalScale_z, int shapeID)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeHeightfield(width, height, ndShapeHeightfield::ndGridConstruction(gridsDiagonals), horizontalScale_x, horizontalScale_z));
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	instance->SetMaterial(material);

	ndShapeHeightfield* const heighfield = instance->GetShape()->GetAsShapeHeightfield();
	ndArray<ndReal>& heightMap = heighfield->GetElevationMap();
	ndArray<ndInt8>& materialMap = heighfield->GetAttributeMap();

	if (elevationdatType)
	{
		const ndUnsigned16* const elevations = PointerCast<ndUnsigned16, void>(elevationMap);
		for (ndInt32 i = 0; i < heightMap.GetCount(); ++i)
		{
			ndFloat32 high = ndFloat32(elevations[i]) * verticalScale;
			heightMap[i] = ndReal(high);
			materialMap[i] = attributeMap[i];
		}
	}
	else
	{
		const ndReal* const elevations = PointerCast<ndReal, void>(elevationMap);
		for (ndInt32 i = 0; i < heightMap.GetCount(); ++i)
		{
			ndFloat32 high = elevations[i];
			heightMap[i] = ndReal(high);
			materialMap[i] = attributeMap[i];
		}
	}
	heighfield->UpdateElevationMapAabb();

	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
}

int NewtonCollisionIsConvexShape(const NewtonCollision* const collision)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const instance = (dgCollisionInstance*)collision;
	//return instance->IsType(dgCollision::dgCollisionConvexShape_RTTI) ? 1 : 0;
	ndAssert(0);
	return 0;
}

int NewtonCollisionIsStaticShape(const NewtonCollision* const collision)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const instance = (dgCollisionInstance*)collision;
	//return (instance->IsType(dgCollision::dgCollisionMesh_RTTI) || instance->IsType(dgCollision::dgCollisionScene_RTTI)) ? 1 : 0;
	ndAssert(0);
	return 0;
}

void NewtonCollisionGetScale(const NewtonCollision* const collision, dFloat* const scaleX, dFloat* const scaleY, dFloat* const scaleZ)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const instance = (dgCollisionInstance*)collision;
	//
	//dgVector scale(instance->GetScale());
	//*scaleX = scale.m_x;
	//*scaleY = scale.m_y;
	//*scaleZ = scale.m_z;

	ndAssert(0);
}


dFloat NewtonCollisionGetSkinThickness(const NewtonCollision* const collisionPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)collisionPtr;
	//return collision->GetSkinThickness();

	ndAssert(0);
	return 0;
}

// Return the trigger volume flag of this shape.
//
// @param convexCollision is the pointer to a convex collision primitive.
// 
// @return 0 if collision shape is solid, non zero is collision shape is a trigger volume.
//
// this function can be used to place collision triggers in the scene. 
// Setting this flag is not really a necessary to place a collision trigger however this option hint the engine that 
// this particular shape is a trigger volume and no contact calculation is desired.
int NewtonCollisionGetMode(const NewtonCollision* const convexCollision)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)convexCollision;
	//TRACE_FUNCTION(__FUNCTION__);
	//return collision->GetCollisionMode() ? 1 : 0;
	ndAssert(0);
	return 0;
}

/*!
  Calculate the closest point between a point and convex collision primitive.

  @param *newtonWorld Pointer to the Newton world.
  @param *point pointer to and array of a least 3 floats representing the origin.
  @param *collision pointer to collision primitive.
  @param *matrix pointer to an array of 16 floats containing the offset matrix of collision primitiveA.
  @param *contact pointer to and array of a least 3 floats to contain the closest point to collisioA.
  @param *normal pointer to and array of a least 3 floats to contain the separating vector normal.
  @param  threadIndex -Thread index form where the call is made from, zeor otherwize

  @return one if the two bodies are disjoint and the closest point could be found,
  zero if the point is inside the convex primitive.

  This function can be used as a low-level building block for a stand-alone collision system.
  Applications that have already there own physics system, and only want and quick and fast collision solution,
  can use Newton advanced collision engine as the low level collision detection part.
  To do this the application only needs to initialize Newton, create the collision primitives at application discretion,
  and just call this function when the objects are in close proximity. Applications using Newton as a collision system
  only, are responsible for implementing their own broad phase collision determination, based on any high level tree structure.
  Also the application should implement their own trivial aabb test, before calling this function .

  the current implementation of this function do work on collision trees, or user define collision.

  See also: ::NewtonCollisionCollideContinue, ::NewtonCollisionClosestPoint, ::NewtonCollisionCollide, ::NewtonCollisionRayCast, ::NewtonCollisionCalculateAABB
*/
int NewtonCollisionPointDistance(const NewtonWorld* const newtonWorld, const dFloat* const point,
	const NewtonCollision* const collision, const dFloat* const matrix,
	dFloat* const contact, dFloat* const normal, int threadIndex)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return world->ClosestPoint(*((dgTriplex*)point), (dgCollisionInstance*)collision, dgMatrix(matrix), *((dgTriplex*)contact), *((dgTriplex*)normal), threadIndex);
	ndAssert(0);
	return 0;
}


/*!
  Calculate time of impact of impact and contact points between two collision primitive.

  @param *newtonWorld Pointer to the Newton world.
  @param maxSize size of maximum number of elements in contacts, normals, and penetration.
  @param timestep maximum time interval considered for the continuous collision calculation.
  @param *collisionA pointer to collision primitive A.
  @param *matrixA pointer to an array of 16 floats containing the offset matrix of collision primitiveA.
  @param *velocA pointer to and array of a least 3 times maxSize floats containing the linear velocity of collision primitiveA.
  @param *omegaA pointer to and array of a least 3 times maxSize floats containing the angular velocity of collision primitiveA.
  @param *collisionB pointer to collision primitive B.
  @param *matrixB pointer to an array of 16 floats containing the offset matrix of collision primitiveB.
  @param *velocB pointer to and array of a least 3 times maxSize floats containing the linear velocity of collision primitiveB.
  @param *omegaB pointer to and array of a least 3 times maxSize floats containing the angular velocity of collision primitiveB.
  @param *timeOfImpact pointer to least 1 float variable to contain the time of the intersection.
  @param *contacts pointer to and array of a least 3 times maxSize floats to contain the collision contact points.
  @param *normals pointer to and array of a least 3 times maxSize floats to contain the collision contact normals.
  @param *penetration pointer to and array of a least maxSize floats to contain the collision penetration at each contact.
  @param attributeA fixme
  @param attributeB fixme
  @param  threadIndex -Thread index form where the call is made from, zeor otherwize

  @return the number of contact points.

  by passing zero as *maxSize* not contact will be calculated and the function will just determine the time of impact is any.

  if the body are inter penetrating the time of impact will be zero.

  if the bodies do not collide time of impact will be set to *timestep*

  This function can be used as a low-level building block for a stand-alone collision system.
  Applications that have already there own physics system, and only want and quick and fast collision solution,
  can use Newton advanced collision engine as the low level collision detection part.
  To do this the application only needs to initialize Newton, create the collision primitives at application discretion,
  and just call this function when the objects are in close proximity. Applications using Newton as a collision system
  only, are responsible for implementing their own broad phase collision determination, based on any high level tree structure.
  Also the application should implement their own trivial aabb test, before calling this function .

  See also: ::NewtonCollisionCollide, ::NewtonCollisionClosestPoint, ::NewtonCollisionPointDistance, ::NewtonCollisionRayCast, ::NewtonCollisionCalculateAABB
*/
int NewtonCollisionCollideContinue(const NewtonWorld* const newtonWorld, int maxSize, dFloat timestep,
	const NewtonCollision* const collisionA, const dFloat* const matrixA, const dFloat* const velocA, const dFloat* const omegaA,
	const NewtonCollision* const collisionB, const dFloat* const matrixB, const dFloat* const velocB, const dFloat* const omegaB,
	dFloat* const timeOfImpact, dFloat* const contacts, dFloat* const normals, dFloat* const penetration,
	dLong* const attributeA, dLong* const attributeB, int threadIndex)
{
	TRACE_FUNCTION(__FUNCTION__);

	//Newton* const world = (Newton*)newtonWorld;
	//
	//*timeOfImpact = timestep;
	//
	//return world->CollideContinue((dgCollisionInstance*)collisionA, dgMatrix(matrixA), *((dgVector*)velocA), *((dgVector*)omegaA),
	//	(dgCollisionInstance*)collisionB, dgMatrix(matrixB), *((dgVector*)velocB), *((dgVector*)omegaB),
	//	*timeOfImpact, (dgTriplex*)contacts, (dgTriplex*)normals, penetration, attributeA, attributeB, maxSize, threadIndex);
	ndAssert(0);
	return 0;
}

int NewtonConvexHullGetVertexData(const NewtonCollision* const convexHullCollision, dFloat** const vertexData, int* strideInBytes)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgAssert(0);
	//return 0;
	ndAssert(0);
	return 0;
}

/*!
  Return the number of vertices of face and copy each index into array faceIndices.

  @param convexHullCollision is the pointer to a convex collision hull primitive.
  @param face fixme
  @param faceIndices fixme

  @return user face count of face.

  this function will return zero on all shapes other than a convex full collision shape.

  To get the number of faces of a convex hull shape see function *NewtonCollisionGetInfo*

  See also: ::NewtonCollisionGetInfo, ::NewtonCreateConvexHull
*/
int NewtonConvexHullGetFaceIndices(const NewtonCollision* const convexHullCollision, int face, int* const faceIndices)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const coll = (dgCollisionInstance*)convexHullCollision;
	//
	//if (coll->IsType(dgCollision::dgCollisionConvexHull_RTTI)) {
	//	//return ((dgCollisionConvexHull*)coll)->GetFaceIndices (face, faceIndices);
	//	return ((dgCollisionConvexHull*)coll->GetChildShape())->GetFaceIndices(face, faceIndices);
	//}
	//else {
	//	return 0;
	//}

	ndAssert(0);
	return 0;

}


const void* NewtonCollisionDataPointer(const NewtonCollision* const convexCollision)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const coll = (dgCollisionInstance*)convexCollision;
	//return coll->GetChildShape();

	ndAssert(0);
	return 0;
}

/*!
  Serialize a general collision shape.

  @param *newtonWorld Pointer to the Newton world.
  @param *collision is the pointer to the collision tree shape.
  @param serializeFunction pointer to the event function that will do the serialization.
  @param  *serializeHandle	- user data that will be passed to the _NewtonSerialize_ callback.

  @return Nothing.

  Small and medium collision shapes like *TreeCollision* (under 50000 polygons) small convex hulls or compude collision can be constructed at application
  startup without significant processing overhead.


  See also: ::NewtonCollisionGetInfo
*/
void NewtonCollisionSerialize(const NewtonWorld* const newtonWorld, const NewtonCollision* const collision, NewtonSerializeCallback serializeFunction, void* const serializeHandle)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//world->SerializeCollision((dgCollisionInstance*)collision, (dgSerialize)serializeFunction, serializeHandle);
	ndAssert(0);
}


/*!
  Create a collision shape via a serialization function.

  @param *newtonWorld Pointer to the Newton world.
  @param deserializeFunction pointer to the event function that will do the deserialization.
  @param *serializeHandle user data that will be passed to the _NewtonSerialize_ callback.

  @return Nothing.

  this function is useful to to load collision primitive for and archive file. In the case of complex shapes like convex hull and compound collision the
  it save a significant amount of construction time.

  if this function is called to load a serialized tree collision, the tree collision will be loaded, but the function pointer callback will be set to NULL.
  for this operation see function *NewtonCreateTreeCollisionFromSerialization*

  See also: ::NewtonCollisionSerialize, ::NewtonCollisionGetInfo
*/
NewtonCollision* NewtonCreateCollisionFromSerialization(const NewtonWorld* const newtonWorld, NewtonDeserializeCallback deserializeFunction, void* const serializeHandle)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return  (NewtonCollision*)world->CreateCollisionFromSerialization((dgDeserialize)deserializeFunction, serializeHandle);
	ndAssert(0);
	return 0;
}

void NewtonHeightFieldSetUserRayCastCallback(const NewtonCollision* const heightField, NewtonHeightFieldRayCastCallback rayHitCallback)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)heightField;
	//if (collision->IsType(dgCollision::dgCollisionHeightField_RTTI)) {
	//	dgCollisionHeightField* const shape = (dgCollisionHeightField*)collision->GetChildShape();
	//	shape->SetCollisionRayCastCallback((dgCollisionHeightFieldRayCastCallback)rayHitCallback);
	//}
	ndAssert(0);
}


/*!
  Create a complex collision geometry to be controlled by the application.

  @param *newtonWorld Pointer to the Newton world.
  @param *minBox pointer to an array of at least three floats to hold minimum value for the box relative to the collision.
  @param *maxBox pointer to an array of at least three floats to hold maximum value for the box relative to the collision.
  @param *userData pointer to user data to be used as context for event callback.
  @param collideCallback pointer to an event function for providing Newton with the polygon inside a given box region.
  @param rayHitCallback pointer to an event function for providing Newton with ray intersection information.
  @param destroyCallback pointer to an event function for destroying any data allocated for use by the application.
  @param getInfoCallback fixme
  @param getAABBOverlapTestCallback fixme
  @param facesInAABBCallback fixme
  @param serializeCallback fixme
  @param shapeID fixme

  @return Pointer to the user collision.

  *UserMeshCollision* provides the application with a method of overloading the built-in collision system for background objects.
  UserMeshCollision can be used for implementing collisions with height maps, collisions with BSP, and any other collision structure the application
  supports and wishes to preserve.
  However, *UserMeshCollision* can not take advantage of the efficient and sophisticated algorithms and data structures of the
  built-in *TreeCollision*. We suggest you experiment with both methods and use the method best suited to your situation.

  When a *UserMeshCollision* is assigned to a body, the mass of the body is ignored in all dynamics calculations.
  This make the body behave as a static body.

*/
NewtonCollision* NewtonCreateUserMeshCollision(
	const NewtonWorld* const newtonWorld,
	const dFloat* const minBox,
	const dFloat* const maxBox,
	void* const userData,
	NewtonUserMeshCollisionCollideCallback collideCallback,
	NewtonUserMeshCollisionRayHitCallback rayHitCallback,
	NewtonUserMeshCollisionDestroyCallback destroyCallback,
	NewtonUserMeshCollisionGetCollisionInfo getInfoCallback,
	NewtonUserMeshCollisionAABBTest getAABBOverlapTestCallback,
	NewtonUserMeshCollisionGetFacesInAABB facesInAABBCallback,
	NewtonOnUserCollisionSerializationCallback serializeCallback,
	int shapeID)
{
	TRACE_FUNCTION(__FUNCTION__);
	const ndVector p0(minBox[0], minBox[1], minBox[2], ndFloat32(1.0f));
	const ndVector p1(maxBox[0], maxBox[1], maxBox[2], ndFloat32(1.0f));
	NewtonProceduralCollition* const shape = new NewtonProceduralCollition(userData, p0, p1);

	shape->m_onRaycast = rayHitCallback;
	shape->m_onDestroy = destroyCallback;
	shape->m_onGetFacePatch = collideCallback;

	ndShapeInstance* const instance = new ndShapeInstance(shape);

	//dgUserMeshCreation data;
	//data.m_userData = userData;
	//data.m_collideCallback = (dgCollisionUserMesh::OnUserMeshCollideCallback)collideCallback;
	//data.m_rayHitCallback = (dgCollisionUserMesh::OnUserMeshRayHitCallback)rayHitCallback;
	//data.m_destroyCallback = (dgCollisionUserMesh::OnUserMeshDestroyCallback)destroyCallback;
	//data.m_getInfoCallback = (dgCollisionUserMesh::OnUserMeshCollisionInfo)getInfoCallback;
	//data.m_getAABBOvelapTestCallback = (dgCollisionUserMesh::OnUserMeshAABBOverlapTest)getAABBOverlapTestCallback;
	//data.m_faceInAABBCallback = (dgCollisionUserMesh::OnUserMeshFacesInAABB)facesInAABBCallback;
	//data.m_serializeCallback = (dgCollisionUserMesh::OnUserMeshSerialize)serializeCallback;
	//ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	//dgCollisionInstance* const collision = world->CreateStaticUserMesh(p0, p1, data);
	//instance->SetUserDataID(dgUnsigned32(shapeID));
	//return (NewtonCollision*)collision;

	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	instance->SetMaterial(material);
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
}

int NewtonUserMeshCollisionContinuousOverlapTest(const NewtonUserMeshCollisionCollideDesc* const collideDescData, const void* const rayHandle, const dFloat* const minAabb, const dFloat* const maxAabb)
{
	TRACE_FUNCTION(__FUNCTION__);
	//const dgFastRayTest* const ray = (dgFastRayTest*)rayHandle;
	//
	//dgVector p0(minAabb);
	//dgVector p1(maxAabb);
	//
	//dgVector q0(collideDescData->m_boxP0);
	//dgVector q1(collideDescData->m_boxP1);
	//
	//p0 = p0 & dgVector::m_triplexMask;
	//p1 = p1 & dgVector::m_triplexMask;
	//q0 = q0 & dgVector::m_triplexMask;
	//q1 = q1 & dgVector::m_triplexMask;
	//
	//dgVector box0(p0 - q1);
	//dgVector box1(p1 - q0);
	//
	//dgFloat32 dist = ray->BoxIntersect(box0, box1);
	//return (dist < dgFloat32(1.0f)) ? 1 : 0;
	ndAssert(0);
	return 0;
}

/*!
  Get creation parameters for this collision objects.

  @param collision is the pointer to a convex collision primitive.
  @param *collisionInfo pointer to a collision information record.

  This function can be used by the application for writing file format and for serialization.

  See also: ::NewtonCollisionGetInfo, ::NewtonCollisionSerialize
*/
void NewtonCollisionGetInfo(const NewtonCollision* const collision, NewtonCollisionInfoRecord* const collisionInfo)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);
	const ndShapeInfo info(instance->GetShapeInfo());
	
	ndMemSet(collisionInfo->m_paramArray, dFloat(0.0f), sizeof (collisionInfo->m_paramArray) / sizeof (collisionInfo->m_paramArray[0]));

	collisionInfo->m_collisionMaterial.m_userId = info.m_shapeMaterial.m_userId;
	collisionInfo->m_collisionMaterial.m_userData.m_ptr = info.m_shapeMaterial.m_userParam->m_ptrData;
	ndMemCpy(&collisionInfo->m_offsetMatrix[0][0], &info.m_offsetMatrix[0][0], 16);
	MapShapeId(info, *collisionInfo);
}


NewtonCollision* NewtonCollisionGetParentInstance(const NewtonCollision* const collision)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);
	//ndShapeCompound* const compoundCollision = instance->GetShape()->GetAsShapeCompound();
	//return !compoundCollision ? reinterpret_cast<NewtonCollision*> (compoundCollision->GetOwner()) : nullptr;
	return reinterpret_cast<NewtonCollision*> (instance->GetParent());
}

/*!
  Store a user defined value with a convex collision primitive.

  @param collision is the pointer to a collision primitive.
  @param id value to store with the collision primitive.

  @return nothing

  the application can store an id with any collision primitive. This id can be used to identify what type of collision primitive generated a contact.

  See also: ::NewtonCollisionGetUserID, ::NewtonCreateBox, ::NewtonCreateSphere
*/
void NewtonCollisionSetUserID(const NewtonCollision* const collision, dLong id)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);

	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = id;
	instance->SetMaterial(material);
}

/*!
  Return a user define value with a convex collision primitive.

  @param collision is the pointer to a convex collision primitive.

  @return user id

  the application can store an id with any collision primitive. This id can be used to identify what type of collision primitive generated a contact.

  See also: ::NewtonCreateBox, ::NewtonCreateSphere
*/
dLong NewtonCollisionGetUserID(const NewtonCollision* const collision)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);
	const ndShapeMaterial material = instance->GetMaterial();
	return material.m_userId;
}

void NewtonCollisionGetMaterial(const NewtonCollision* const collision, NewtonCollisionMaterial* const userData)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);

	const ndShapeMaterial material (instance->GetMaterial());
	userData->m_userId = material.m_userId;
	userData->m_userData.m_int = material.m_data.m_intData;

	const ndInt32 count1 = ndInt32(sizeof(material.m_userParam) / sizeof(material.m_userParam[0]));
	const ndInt32 count0 = ndInt32(sizeof(userData->m_userParam) / sizeof(userData->m_userParam[0]));
	const ndInt32 count = ndMin(count0, count1);
	for (ndInt32 i = 0; i < count; ++i)
	{
		userData->m_userParam[i].m_ptr = material.m_userParam[i].m_ptrData;
	}
}

void NewtonCollisionSetMaterial(const NewtonCollision* const collision, const NewtonCollisionMaterial* const userData)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);
	ndShapeMaterial material;
	material.m_userId = userData->m_userId;
	material.m_data.m_intData = userData->m_userData.m_int;

	const ndInt32 count1 = ndInt32(sizeof(material.m_userParam) / sizeof(material.m_userParam[0]));
	const ndInt32 count0 = ndInt32(sizeof(userData->m_userParam) / sizeof(userData->m_userParam[0]));
	const ndInt32 count = ndMin(count0, count1);
	for (ndInt32 i = 0; i < count; ++i)
	{
		material.m_userParam[i].m_ptrData = userData->m_userParam[i].m_ptr;
	}
	instance->SetMaterial(material);
}

int NewtonCollisionGetType(const NewtonCollision* const collision)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);
	NewtonCollisionInfoRecord collisionInfo;
	MapShapeId(instance->GetShapeInfo(), collisionInfo);
	return collisionInfo.m_collisionType;
}

void NewtonCollisionSetScale(const NewtonCollision* const collision, dFloat scaleX, dFloat scaleY, dFloat scaleZ)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);
	instance->SetScale(ndVector(scaleX, scaleY, scaleZ, ndFloat32(0.0f)));
}

void* NewtonCollisionGetSubCollisionHandle(const NewtonCollision* const collision)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);
	return PointerCast<void, void>(*instance->m_subCollisionHandle);
}

// Set a flag on a convex collision shape to indicate that no contacts should calculated for this shape.
//
// @param convexCollision is the pointer to a convex collision primitive.
// @param triggerMode 1 disable contact calculation, 0 enable contact calculation.
// 
// @return nothing
//
// this function can be used to place collision triggers in the scene. 
// Setting this flag is not really a necessary to place a collision trigger however this option hint the engine that 
// this particular shape is a trigger volume and no contact calculation is desired.
//
void NewtonCollisionSetMode(const NewtonCollision* const collision, int mode)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);
	instance->SetCollisionMode(mode ? true : false);
}


/*!
  Calculate the three principal axis and the the values of the inertia matrix of a convex collision objects.

  @param convexCollision is the pointer to a convex collision primitive.
  @param *inertia pointer to and array of a least 3 floats to hold the values of the principal inertia.
  @param *origin pointer to and array of a least 3 floats to hold the values of the center of mass for the principal inertia.

  This function calculate a general inertial matrix for arbitrary convex collision including compound collisions.

  See also: ::NewtonBodySetMassMatrix, ::NewtonBodyGetMass, ::NewtonBodySetCentreOfMass, ::NewtonBodyGetCentreOfMass
*/
void NewtonConvexCollisionCalculateInertialMatrix(const NewtonCollision* const convexCollision, dFloat* const inertia, dFloat* const origin)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(convexCollision);
	const ndMatrix tmpInertia(instance->CalculateInertia());
	
	inertia[0] = tmpInertia[0][0];
	inertia[1] = tmpInertia[1][1];
	inertia[2] = tmpInertia[2][2];
	origin[0] = tmpInertia[3][0];
	origin[1] = tmpInertia[3][1];
	origin[2] = tmpInertia[3][2];
}

/*!
  Calculate an axis-aligned bounding box for this collision, the box is calculated relative to *offsetMatrix*.

  @param *collisionPtr pointer to the collision object.
  @param *offsetMatrix pointer to an array of 16 floats containing the offset matrix used as the coordinate system and center of the AABB.
  @param  *p0 - pointer to an array of at least three floats to hold minimum value for the AABB.
  @param  *p1 - pointer to an array of at least three floats to hold maximum value for the AABB.

  @return Nothing.

  See also: ::NewtonCollisionClosestPoint, ::NewtonCollisionPointDistance, ::NewtonCollisionCollide, ::NewtonCollisionRayCast
*/
void NewtonCollisionCalculateAABB(const NewtonCollision* const collisionPtr, const dFloat* const offsetMatrix, dFloat* const p0, dFloat* const p1)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collisionPtr);
	ndMatrix matrix(instance->GetLocalMatrix() * GetMatrix(offsetMatrix));
	
	ndVector q0;
	ndVector q1;
	instance->CalculateAabb(matrix, q0, q1);
	p0[0] = q0.m_x;
	p0[1] = q0.m_y;
	p0[2] = q0.m_z;
	
	p1[0] = q1.m_x;
	p1[1] = q1.m_y;
	p1[2] = q1.m_z;
}

NewtonCollision* NewtonCollisionCreateInstance(const NewtonCollision* const collision)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = new ndShapeInstance(*PointerCast<ndShapeInstance, NewtonCollision>(collision));
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
}

void NewtonCollisionSetUserData(const NewtonCollision* const collision, void* const userData)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userParam[0].m_ptrData = userData;
	instance->SetMaterial(material);
}

void* NewtonCollisionGetUserData(const NewtonCollision* const collision)
{
	TRACE_FUNCTION(__FUNCTION__);
	const ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);
	const ndShapeMaterial material = instance->GetMaterial();
	return material.m_userParam[0].m_ptrData;
}
