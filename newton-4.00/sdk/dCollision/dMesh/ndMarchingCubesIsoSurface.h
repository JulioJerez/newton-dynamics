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

#ifndef __ND_MARCHING_CUBES_ISO_SURFACE_H__
#define __ND_MARCHING_CUBES_ISO_SURFACE_H__

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"

class ndPatchMesh;
class ndMarchingCubeIsoSurface : public ndMarchingCubes
{
	class ndTriangle
	{
		public:
		ndVector m_p0;
		ndVector m_p1;
		ndVector m_p2;
	};

	class ndIsoCell
	{
		public:
		ndVector m_isoValues[8];
	};

	public:
	D_COLLISION_API ndMarchingCubeIsoSurface(ndThreadPool* const threadPool, const ndVector& boxP0, const ndVector& boxP1, ndFloat32 gridSize);
	D_COLLISION_API virtual ~ndMarchingCubeIsoSurface();

	D_COLLISION_API void GetBox (ndVector& boxP0, ndVector& boxP1) const;
	D_COLLISION_API void SetBox(const ndVector& boxP0, const ndVector& boxP1);

	D_COLLISION_API virtual void GenerateMesh() override;
	D_COLLISION_API ndVector PositionToGridSpace(const ndVector& posit) const;
	D_COLLISION_API ndVector GridSpaceToPosition(const ndVector& gridPosit) const;

	D_COLLISION_API void GetFacesPatch(ndPatchMesh& patch) const;
	D_COLLISION_API ndFloat32 RayCast(const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, ndContactPoint& contactOut) const;

	virtual ndReal GetIsoValue(const ndVector& posit) const = 0;

	protected:
	class ndGridInfo
	{
		public:
		ndGridInfo(ndInt32 faceVount = 0)
			:m_triangleCount(faceVount)
		{
		}
		ndInt32 m_triangleCount;
		ndInt32 m_x;
		ndInt32 m_z;
		ndInt32 m_tableIndex;
	};

	enum ndCellFill
	{
		m_empty,
		m_solid,
		m_partial,
	};

	void GenerateIndexList();
	bool CalculateMinExtend3d(const ndVector& p0, const ndVector& p1, ndVector& boxP0, ndVector& boxP1) const;
	ndFloat32 RayCastCell(const ndFastRay& ray, ndInt32 xIndex0, ndInt32 yIndex0, ndInt32 zIndex0, ndVector& normalOut, ndFloat32 maxT) const;

	ndVector m_boxP0;
	ndVector m_boxP1;
	ndFixSizeArray<ndVector, 8> m_gridStep;
	ndInt32 m_maxGrid_x;
	ndInt32 m_maxGrid_y;
	ndInt32 m_maxGrid_z;
	bool m_generateNormals;
};

#endif