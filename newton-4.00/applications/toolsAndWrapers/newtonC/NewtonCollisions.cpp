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

NewtonCollision* NewtonCreateTreeCollisionFromMesh(const NewtonWorld* const, const NewtonMesh* const mesh, int shapeID)
{
	TRACE_FUNCTION(__FUNCTION__);
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
	ndSharedPtr<ndShapeInstance>* const shape = new ndSharedPtr<ndShapeInstance>(new ndShapeInstance(new ndShapeNull()));
	return reinterpret_cast<NewtonCollision*>(shape);
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
	ndSharedPtr<ndShapeInstance>* const shape = new ndSharedPtr<ndShapeInstance>(new ndShapeInstance(new ndShapeBox(dx, dy, dz)));
	ndShapeInstance* const instance = **shape;
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	return reinterpret_cast<NewtonCollision*>(shape);
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
NewtonCollision* NewtonCreateSphere(const NewtonWorld* const newtonWorld, dFloat radius, int shapeID, const dFloat* const offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}
	ndSharedPtr<ndShapeInstance>* const shape = new ndSharedPtr<ndShapeInstance>(new ndShapeInstance(new ndShapeSphere(radius)));
	ndShapeInstance* const instance = **shape;
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	return reinterpret_cast<NewtonCollision*>(shape);
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
NewtonCollision* NewtonCreateCone(const NewtonWorld* const newtonWorld, dFloat radius, dFloat height, int shapeID, const dFloat* const offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}
	ndSharedPtr<ndShapeInstance>* const shape = new ndSharedPtr<ndShapeInstance>(new ndShapeInstance(new ndShapeCone(radius, height)));
	ndShapeInstance* const instance = **shape;
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	return reinterpret_cast<NewtonCollision*>(shape);
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
NewtonCollision* NewtonCreateCapsule(const NewtonWorld* const newtonWorld, dFloat radio0, dFloat radio1, dFloat height, int shapeID, const dFloat* const offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}
	ndSharedPtr<ndShapeInstance>* const shape = new ndSharedPtr<ndShapeInstance>(new ndShapeInstance(new ndShapeCapsule (radio0, radio1, height)));
	ndShapeInstance* const instance = **shape;
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	return reinterpret_cast<NewtonCollision*>(shape);
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
NewtonCollision* NewtonCreateCylinder(const NewtonWorld* const newtonWorld, dFloat radio0, dFloat radio1, dFloat height, int shapeID, const dFloat* const offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}
	ndSharedPtr<ndShapeInstance>* const shape = new ndSharedPtr<ndShapeInstance>(new ndShapeInstance(new ndShapeCylinder(radio0, radio1, height)));
	ndShapeInstance* const instance = **shape;
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	return reinterpret_cast<NewtonCollision*>(shape);
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
NewtonCollision* NewtonCreateChamferCylinder(const NewtonWorld* const newtonWorld, dFloat radius, dFloat height, int shapeID, const dFloat* const offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}
	ndSharedPtr<ndShapeInstance>* const shape = new ndSharedPtr<ndShapeInstance>(new ndShapeInstance(new ndShapeChamferCylinder(radius, height)));
	ndShapeInstance* const instance = **shape;
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	return reinterpret_cast<NewtonCollision*>(shape);
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
NewtonCollision* NewtonCreateConvexHull(const NewtonWorld* const newtonWorld, int count, const dFloat* const vertexCloud, int strideInBytes, dFloat32 tolerance, int shapeID, const dFloat* const offsetMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(ndGetIdentityMatrix());
	if (offsetMatrix)
	{
		matrix = ndMatrix(offsetMatrix);
	}
	ndSharedPtr<ndShapeInstance>* const shape = new ndSharedPtr<ndShapeInstance>(new ndShapeInstance(new ndShapeConvexHull(count, strideInBytes, tolerance, vertexCloud)));
	ndShapeInstance* const instance = **shape;
	instance->SetLocalMatrix(matrix);
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;
	return reinterpret_cast<NewtonCollision*>(shape);
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
NewtonCollision* NewtonCreateConvexHullFromMesh(const NewtonWorld* const, const NewtonMesh* const mesh, dFloat tolerance, int shapeID)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);

	ndSharedPtr<ndShapeInstance>* const shape = new ndSharedPtr<ndShapeInstance>(meshEffect->CreateConvexCollision(tolerance));
	ndShapeInstance* const instance = **shape;
	ndShapeMaterial material = instance->GetMaterial();
	material.m_userId = shapeID;

	return reinterpret_cast<NewtonCollision*>(shape);
}
