/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_SHAPE_STATIC_PROCEDURAL_MESH__
#define __ND_SHAPE_STATIC_PROCEDURAL_MESH__

#include "ndCollisionStdafx.h"
#include "ndShapeStaticMesh.h"

D_MSV_NEWTON_CLASS_ALIGN_32
class ndShapeStaticProceduralMesh: public ndShapeStaticMesh
{
	public:

	D_CLASS_REFLECTION(ndShapeStaticProceduralMesh, ndShapeStaticMesh)
	D_COLLISION_API ndShapeStaticProceduralMesh();
	D_COLLISION_API virtual ~ndShapeStaticProceduralMesh();

	D_COLLISION_API void SetAABB(const ndVector& p0, const ndVector& p1);
	D_COLLISION_API void GetAABB(const ndVector& p0, const ndVector& p1);

	virtual void GetFacesPatch(ndPatchMesh& patch) const = 0;
	virtual ndShapeStaticProceduralMesh* GetAsShapeStaticProceduralMesh() override { return this; }

	protected:
	D_COLLISION_API virtual ndShapeInfo GetShapeInfo() const override;
	D_COLLISION_API virtual ndUnsigned64 GetHash(ndUnsigned64 hash) const override;
	D_COLLISION_API virtual void GetCollidingFaces(ndPolygonMeshDesc* const data) const override;

	private:
	friend class ndContactSolver;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif
