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

class NewtonCompoundCollision : public ndShapeCompound
{
	public:
	D_CLASS_REFLECTION(NewtonCompoundCollision, ndShapeCompound)
		NewtonCompoundCollision()
		:ndShapeCompound()
	{
	}

	NewtonCompoundCollision(const NewtonCompoundCollision& source, const ndShapeInstance* const parent)
		:ndShapeCompound(source, parent)
	{
	}

	virtual ndShape* Clone(ndShapeInstance* const parent) const override
	{
		return new NewtonCompoundCollision(*this, parent);
	}
};

// Name: NewtonCreateCompoundCollision 
// Create a container to hold an array of convex collision primitives. 
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *int* count - number of primitives in the array.
// *const NewtonCollision* **collisionPrimitiveArray - pointer to an array of convex collision primitives. This array must be filled with convex collision primitives before this function is called.
//
// Return: Pointer to the compound collision.
//
// Remarks: Compound collision primitives can only be made of convex collision primitives and they can not contain compound collision. Therefore they are treated as convex primitives.
//
// Remarks: Compound collision primitives are treated as instance collision objects that can not shared by multiples rigid bodies.
//
// Remarks: Collision primitives are reference counted objects. The application should call *NewtonReleaseCollision* in order to release references to the object.
// Neglecting to release references to collision primitives is a common cause of memory leaks.
//
// See also: NewtonReleaseCollision
NewtonCollision* NewtonCreateCompoundCollision(const NewtonWorld* newtonWorld, int count, NewtonCollision* const collisionPrimitiveArray[])
{
	//Newton* world;
	//world = (Newton*)newtonWorld;
	//return (NewtonCollision*)world->CreateCompoundCollision(count, (dgCollision**)collisionPrimitiveArray);

	TRACE_FUNCTION(__FUNCTION__);
	ndShapeInstance* const instance = new ndShapeInstance(new NewtonCompoundCollision());
	ndShapeCompound* const collision = PointerCast<NewtonCompoundCollision, ndShapeCompound>(instance->GetShape()->GetAsShapeCompound());

	collision->BeginAddRemove();
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndShapeInstance* const childInstance = PointerCast<ndShapeInstance, NewtonCollision>(collisionPrimitiveArray[i]);
		collision->AddCollision(childInstance);
	}
	collision->EndAddRemove();
	return PointerCast<NewtonCollision, ndShapeInstance>(instance);
}
