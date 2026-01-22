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

class ndThreadPool;

class ndMarchingCubes : public ndClassAlloc
{
	public:
	class ndKey_xlow;
	class ndKey_xhigh;
	class ndKey_ylow;
	class ndKey_yhigh;
	class ndKey_zlow;
	class ndKey_zhigh;

	class ndEdge;
	class ndIsoCell;
	class ndGridHashSteps;

	class ndGridHash
	{
		public:
		ndGridHash();
		ndGridHash(const ndVector& grid);
		ndGridHash(const ndGridHash& src, ndUnsigned16 cellType);
		ndGridHash(ndInt32 x, ndInt32 y, ndInt32 z);

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
			ndUnsigned64 m_gridFullHash;
		};
	};

	class ndCalculateIsoValue : public ndClassAlloc
	{
		public:
		
		D_CORE_API ndCalculateIsoValue(ndFloat32 gridSize, ndThreadPool* const threadPool);
		D_CORE_API virtual ~ndCalculateIsoValue();

		D_CORE_API const ndArray<ndInt32>& GetTriangles() const;
		D_CORE_API const ndArray<ndVector>& GetMeshVertex() const;
		D_CORE_API const ndArray<ndVector>& GetMeshNormals() const;

		virtual void GenerateMesh() = 0;

		protected:
		void GenerateIndexList();

		ndVector m_boxP0;
		ndVector m_boxP1;
		ndVector m_gridSize;
		ndVector m_invGridSize;
		ndVector m_volumeInGrids;
		ndArray<ndVector> m_meshPoints;
		ndArray<ndVector> m_meshNormals;
		ndArray<ndInt32> m_meshIndices;

		ndArray<ndInt32> m_cellScans;
		ndArray<ndInt32> m_cellTrianglesScans;
		ndArray<ndGridHash> m_hashGridMap;
		ndArray<ndGridHash> m_hashGridMapScratchBuffer;

		ndFloat32 m_isoSufaceValue;
		ndThreadPool* m_threadPool;

		static ndEdge m_edges[];
		static ndInt32 m_faces[][3];
		static ndInt32 m_edgeScan[];
		static ndInt32 m_facesScan[];
		static ndVector m_gridCorners[];
	};

	D_CORE_API ndMarchingCubes();
	D_CORE_API ~ndMarchingCubes();

	D_CORE_API void GenerateMesh(ndCalculateIsoValue* const computeIsoValue);
};

class ndMarchingCubeFromParticleArray: public ndMarchingCubes::ndCalculateIsoValue
{
	public:
	D_CORE_API ndMarchingCubeFromParticleArray(ndThreadPool* const threadPool,ndFloat32 gridSize);
	D_CORE_API virtual ~ndMarchingCubeFromParticleArray();

	D_CORE_API virtual void GenerateMesh() override;

	protected:
	void GenerateGrids();
	void CalculateAABB();
	void RemoveDuplicates();
	void GenerateTriangles();
	ndArray<ndVector> m_points;
};

class ndMarchingCubeIsoFunction : public ndMarchingCubes::ndCalculateIsoValue
{
	public:
	D_CORE_API ndMarchingCubeIsoFunction(ndThreadPool* const threadPool, ndFloat32 gridSize);
	D_CORE_API virtual ~ndMarchingCubeIsoFunction();

	D_CORE_API virtual void GenerateMesh() override;
	D_CORE_API virtual ndReal GetIsoValue(ndInt32 x, ndInt32 y, ndInt32 z) const = 0;

//protected:
//	D_CORE_API virtual void GenerateGrids();
//	D_CORE_API virtual void CalculateAABB();
//	D_CORE_API virtual void RemoveDuplicates();
//	D_CORE_API virtual void GenerateTriangles();
//	D_CORE_API virtual void GenerateIndexList();
//
//	ndArray<ndVector> m_points;
//	ndArray<ndVector> m_meshPoints;
//	ndArray<ndVector> m_meshNormals;
//	ndArray<ndInt32> m_meshIndices;
//

};

#endif