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
	//ndShapeMaterial material = instance->GetMaterial();
	//material.m_userId = shapeID;
	//instance->SetMaterial(material);
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);

}
