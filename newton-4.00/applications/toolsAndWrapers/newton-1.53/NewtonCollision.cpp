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
#include "NewtonBody.h"
#include "NewtonUtils.h"
#include "newtonWorld.h"

class NewtonProceduralCollition : public ndShapeStaticProceduralMesh
{
	public:
	D_CLASS_REFLECTION(NewtonProceduralCollitiuon, ndShapeStaticProceduralMesh)

	typedef void (*NewtonUserMeshCollisionDestroyCallback) (void* const userData);
	typedef dFloat(*NewtonUserMeshCollisionRayHitCallback) (NewtonUserMeshCollisionRayHitDesc* lineDescData);
	typedef int (*NewtonUserMeshCollisionAABBTest) (void* const userData, const dFloat* const boxP0, const dFloat* const boxP1);
	typedef void (*NewtonUserMeshCollisionCollideCallback) (NewtonUserMeshCollisionCollideDesc* collideDescData);
	typedef int (*NewtonUserMeshCollisionGetFacesInAABB) (
		void* const userData, const dFloat* const p0, const dFloat* const p1,
		const dFloat** const vertexArray, int* const vertexCount, int* const vertexStrideInBytes,
		const int* const indexList, int maxIndexCount, const int* const userDataList);

	NewtonProceduralCollition(void* const userData, const ndVector& box0, const ndVector& box1)
		:ndShapeStaticProceduralMesh()
		,m_userData(userData)
		,m_onRaycast(nullptr)
		,m_onDestroy(nullptr)
		,m_onGetFacePatch(nullptr)
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
			}

			collideDescData.m_userData = PointerCast<void, void>(*m_userData);

			collideDescData.m_objBody = nullptr;                  	// pointer to the colliding body
			collideDescData.m_polySoupBody = nullptr;             	// pointer to the rigid body owner of this collision tree 
			//collideDescData.m_polySoupCollision = nullptr;		// collision shape of the collision tree, (no necessarily the collision of m_polySoupBody)
			//collideDescData.m_objCollision = PointerCast<NewtonCollision, ndShapeInstance>(patch.m_convexShapeInstance);
			m_onGetFacePatch(&collideDescData);

			if (collideDescData.m_faceCount)
			{
				if (patch.m_queryType == patch.m_vertexListOnly)
				{
					patch.m_pointArray.PushBack(ndVector::m_zero);
				}
				else
				{
					ndInt32 faceIndex = 0;
					patch.m_vertexArrayHasDuplicated = true;
					const ndFloat32* const vertex = collideDescData.m_vertex;
					const ndInt32* const indexArray = collideDescData.m_faceVertexIndex;
					const ndInt32* const attributes = collideDescData.m_userAttribute;
					const ndInt32* const facesIndexArray = collideDescData.m_faceIndexCount;
					const ndInt32 stride = collideDescData.m_vertexStrideInBytes / sizeof(ndFloat32);
					for (ndInt32 i = 0; i < collideDescData.m_faceCount; ++i)
					{
						const ndInt32 indexCount = facesIndexArray[i];
						patch.m_faceArray.PushBack(indexCount);
						patch.m_faceMaterialArray.PushBack(attributes[indexCount]);

						for (ndInt32 j = 0; j < indexCount; ++j)
						{
							ndInt32 k = stride * indexArray[faceIndex + j];
							const ndVector point(vertex[k + 0], vertex[k + 1], vertex[k + 2], ndFloat32(1.0f));
							patch.m_indexArray.PushBack(patch.m_pointArray.GetCount());
							patch.m_pointArray.PushBack(point);
						}
						ndInt32 start = patch.m_pointArray.GetCount() - indexCount;
						const ndVector e0(patch.m_pointArray[start + 1] - patch.m_pointArray[start]);
						const ndVector e1(patch.m_pointArray[start + 2] - patch.m_pointArray[start]);
						const ndVector normal((e0.CrossProduct(e1) & ndVector::m_triplexMask).Normalize());
						patch.m_normalArray.PushBack(normal);
						faceIndex += indexCount;
					}
				}
			}
		}
	}

	virtual ndFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut, ndShapeInstance* const owner) const override
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
				contactOut.m_normal.m_w = ndFloat32(0.0f);
			}
			return param;
		}
		return ndFloat32(1.2f);
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


#include "newtonStdafx.h"
#include "Newton.h"
#include "NewtonUtils.h"
#include "newtonWorld.h"

// Name: NewtonReleaseCollision 
// Release a reference from this collision object returning control to Newton.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const NewtonCollision* *collisionPtr - pointer to the collision object
//
// Return: Nothing.
//
// Remarks: 
// Collision objects are reference counted objects. The application should call *NewtonReleaseCollision* in order to release references to the object.
// Neglecting to release references to collision primitives is a common cause of memory leaks.
void NewtonReleaseCollision(const NewtonWorld* newtonWorld, const NewtonCollision* collision)
{
	TRACE_FUNCTION(__FUNCTION__);
	const ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(collision);
	delete instance;
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

// Name: NewtonCreateSphere 
// Create a generalized ellipsoid primitive..
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *dFloat* radiusX  - sphere radius along x axis.  
// *dFloat* radiusY  - sphere radius along x axis.  
// *dFloat* radiusZ  - sphere radius along x axis.  
// *const dFloat* *offsetMatrix - pointer to an array of 16 floats containing the offset matrix of the sphere relative to the body. If this parameter is NULL then the sphere is centered at the origin of the body.
// 
// Return: Pointer to the generalized sphere.
//
// Remarks: Sphere collision are generalized ellipsoids, the application can create many different kind of objects by just playing with dimensions of the radius.
// for example to make a sphere set all tree radius to the same value, to make a ellipse of revolution just set two of the tree radius to the same value.
//
// Remarks: General ellipsoids are very good hull geometries to represent the outer shell of avatars in a game. 
//
// Remarks: Collision primitives are reference counted objects. The application should call *NewtonReleaseCollision* in order to release references to the object.
// Neglecting to release references to collision primitives is a common cause of memory leaks.
// Collision primitives can be reused with more than one body. This will reduce the amount of memory used be the engine, as well
// as speed up some calculations.
//
// See also: NewtonReleaseCollision, NewtonCreateConvexHullModifier
NewtonCollision* NewtonCreateSphere(const NewtonWorld* newtonWorld, dFloat radiusX, dFloat radiusY, dFloat radiusZ, const dFloat* offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}

	radiusX = ndAbs(radiusX);
	radiusY = ndAbs(radiusY);
	radiusZ = ndAbs(radiusZ);
	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeSphere(radiusX));

	const ndVector scale(ndFloat32(1.0f), radiusY / radiusX, radiusZ / radiusX, ndFloat32(1.0f));
	instance->SetScale(scale);
	instance->SetLocalMatrix(matrix);
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
NewtonCollision* NewtonCreateBox(const NewtonWorld* const, dFloat dx, dFloat dy, dFloat dz, const dFloat* const offsetMatrix)
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
	//material.m_userId = shapeID;
	//instance->SetMaterial(material);
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
NewtonCollision* NewtonCreateCone(const NewtonWorld* const, dFloat radius, dFloat height, const dFloat* const offsetMatrix)
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
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
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
NewtonCollision* NewtonCreateCapsule(const NewtonWorld* newtonWorld, dFloat radius, dFloat height, const dFloat* offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}

	height -= ndFloat32(2.0f) * radius;
	if (height < ndFloat32(0.01f))
	{
		height = ndFloat32(0.01f);
	}

	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeCapsule(radius, radius, height));
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
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
NewtonCollision* NewtonCreateCylinder(const NewtonWorld* newtonWorld, dFloat radius, dFloat height, const dFloat* offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}
	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeCylinder(radius, radius, height));
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
}

NewtonCollision* NewtonCreateChamferCylinder(const NewtonWorld* newtonWorld, dFloat radius, dFloat height, const dFloat* offsetMatrix);

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
NewtonCollision* NewtonCreateChamferCylinder(const NewtonWorld* newtonWorld, dFloat radius, dFloat height, const dFloat* offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}
	radius -= ndFloat32(0.5f) * height;
	if (radius < ndFloat32(0.01f))
	{
		radius = ndFloat32(0.01f);
	}

	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeChamferCylinder(radius, height));
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
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
NewtonCollision* NewtonCreateConvexHull(const NewtonWorld* newtonWorld, int count, const dFloat* vertexCloud, int strideInBytes, const dFloat* offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}
	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeConvexHull(count, strideInBytes, ndFloat32(1.0e-3f), vertexCloud));
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
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
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(convexHullCollision);
	ndShapeInstance* const clonedInstance = new ndShapeInstance(*instance, instance->m_shape);
	return PointerCast<NewtonCollision, ndShapeInstance>(clonedInstance);
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
void NewtonConvexHullModifierGetMatrix(const NewtonCollision* convexHullModifier, dFloat* matrixPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(convexHullModifier);
	ndMatrix matrix(instance->GetLocalMatrix());
	ndMemCpy(matrixPtr, &matrix[0][0], sizeof(ndMatrix) / sizeof(ndFloat32));
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
	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = PointerCast<ndShapeInstance, NewtonCollision>(convexHullModifier);
	ndMatrix matrix(GetMatrix(matrixPtr));

	matrix.m_front.m_w = ndFloat32(0.0f);
	matrix.m_up.m_w = ndFloat32(0.0f);
	matrix.m_right.m_w = ndFloat32(0.0f);
	matrix.m_posit.m_w = ndFloat32(1.0f);
	instance->SetLocalMatrix(matrix);
}

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
	const dFloat* minBox,
	const dFloat* maxBox,
	void* userData,
	NewtonUserMeshCollisionCollideCallback collideCallback,
	NewtonUserMeshCollisionRayHitCallback rayHitCallBack,
	NewtonUserMeshCollisionDestroyCallback destroyCallback)
{
	TRACE_FUNCTION(__FUNCTION__);
	const ndVector p0(minBox[0], minBox[1], minBox[2], ndFloat32(1.0f));
	const ndVector p1(maxBox[0], maxBox[1], maxBox[2], ndFloat32(1.0f));
	NewtonProceduralCollition* const shape = new NewtonProceduralCollition(userData, p0, p1);

	shape->m_onRaycast = rayHitCallBack;
	shape->m_onDestroy = destroyCallback;
	shape->m_onGetFacePatch = collideCallback;

	ndShapeInstance* const instance = new ndShapeInstance(shape);
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
}

