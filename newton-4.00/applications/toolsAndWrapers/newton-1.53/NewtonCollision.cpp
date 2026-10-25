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
//#include "newtonMaterial.h"
//#include "newtonBodyNotify.h"


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
	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeChamferCylinder(radius, height));
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
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
	ndShapeInstance* const instance = new ndShapeInstance(new ndShapeConvexHull(count, strideInBytes, ndFloat32 (1.0e-3f), vertexCloud));
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
