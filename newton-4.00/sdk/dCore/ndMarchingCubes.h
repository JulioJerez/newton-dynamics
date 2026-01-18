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

#ifndef __ND_MARCHING_CUBES_H__
#define __ND_MARCHING_CUBES_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndArray.h"
#include "ndTree.h"

class ndMarchingCubes_legacy: public ndClassAlloc
{
	public:
	class ndCalculateIsoValue
	{
		public:
		ndCalculateIsoValue()
		{
		}

		virtual ~ndCalculateIsoValue()
		{
		}

		virtual ndFloat32 CalculateIsoValue(const ndVector& point) const = 0;
	};

	class ndImplementation;

	D_CORE_API ndMarchingCubes_legacy();
	D_CORE_API ~ndMarchingCubes_legacy();

	D_CORE_API ndVector GetOrigin() const;
	D_CORE_API const ndArray<ndVector>& GetPoints() const;

	D_CORE_API void GenerateMesh(const ndArray<ndVector>& pointCloud, ndFloat32 gridSize, ndCalculateIsoValue* const computeIsoValue = nullptr);
	D_CORE_API ndInt32 GenerateListIndexList(ndInt32 * const indexList, ndInt32 strideInFloat32, ndReal* const posit, ndReal* const normals) const;

	private:
	ndVector m_origin;
	ndArray<ndVector> m_points;
	ndImplementation* m_implement;
	ndFloat32 m_gridSize;
	ndInt32 m_volumeSizeX;
	ndInt32 m_volumeSizeY;
	ndInt32 m_volumeSizeZ;
	bool m_isLowRes;
};

// ***********************************************************
//
// ***********************************************************
class ndThreadPool;

class ndMarchingCubes : public ndClassAlloc
{
	public:
	class ndCalculateIsoValue : public ndClassAlloc
	{
		public:

		class ndGridHash
		{
			public:
			ndGridHash()
			{
			}

			ndGridHash(const ndGridHash& src, ndUnsigned16 cellType)
			{
				m_gridCellHash = src.m_gridCellHash;
				m_cellType = cellType;
			}

			ndGridHash(ndInt32 x, ndInt32 y, ndInt32 z)
			{
				m_gridCellHash = 0;
				m_x = ndUnsigned16(x);
				m_y = ndUnsigned16(y);
				m_z = ndUnsigned16(z);
			}

			ndGridHash(const ndVector& grid)
			{
				ndAssert(grid.m_x >= ndFloat32(0.0f));
				ndAssert(grid.m_y >= ndFloat32(0.0f));
				ndAssert(grid.m_z >= ndFloat32(0.0f));
				ndAssert(grid.m_x < ndFloat32(256.0f * 256.0f));
				ndAssert(grid.m_y < ndFloat32(256.0f * 256.0f));
				ndAssert(grid.m_z < ndFloat32(256.0f * 256.0f));

				ndVector hash(grid.GetInt());
				m_gridCellHash = 0;
				m_x = ndUnsigned16(hash.m_ix);
				m_y = ndUnsigned16(hash.m_iy);
				m_z = ndUnsigned16(hash.m_iz);
			}

			union
			{
				struct
				{
					ndUnsigned16 m_x;
					ndUnsigned16 m_y;
					ndUnsigned16 m_z;
					ndUnsigned16 m_cellType;
				};
				struct
				{
					ndUnsigned8 m_xLow;
					ndUnsigned8 m_xHigh;
					ndUnsigned8 m_yLow;
					ndUnsigned8 m_yHigh;
					ndUnsigned8 m_zLow;
					ndUnsigned8 m_zHigh;
				};
				ndUnsigned64 m_gridCellHash : 48;
				ndUnsigned64 m_gridFullHash;
			};
		};

		D_CORE_API ndCalculateIsoValue(ndFloat32 gridSize);
		D_CORE_API virtual ~ndCalculateIsoValue();

		virtual void CalculateAABB() = 0;
		virtual void RemoveDuplicates() = 0;
		virtual ndFloat32 CalculateIsoValue(const ndVector& point) const = 0;

		ndVector m_boxP0;
		ndVector m_boxP1;
		ndFloat32 m_gridSize;
		ndArray<ndGridHash> m_hashGridMap;
		ndArray<ndGridHash> m_hashGridMapScratchBuffer;
	};

	D_CORE_API ndMarchingCubes();
	D_CORE_API ~ndMarchingCubes();

	D_CORE_API void GenerateMesh(ndCalculateIsoValue* const computeIsoValue);
};

class ndMarchingCubeParticleIsoValue: public ndMarchingCubes::ndCalculateIsoValue
{
	public:
	D_CORE_API ndMarchingCubeParticleIsoValue(ndThreadPool* const threadPool,ndFloat32 gridSize);
	D_CORE_API virtual ~ndMarchingCubeParticleIsoValue();

	D_CORE_API virtual void CalculateAABB() override;
	D_CORE_API virtual ndFloat32 CalculateIsoValue(const ndVector& point) const override;

	D_CORE_API void RemoveDuplicates() override;

	protected:
	ndArray<ndVector> m_points;
	ndArray<ndVector> m_uniquePoints;
	ndThreadPool* m_threadPool;
};

#endif