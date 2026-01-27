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

#include "ndCoreStdafx.h"
#include "ndSort.h"
#include "ndDebug.h"
#include "ndVector.h"
#include "ndMatrix.h"
#include "ndProfiler.h"
#include "ndMarchingCubesIsoSurface.h"

// adapted from code by written by Paul Bourke may 1994
//http://paulbourke.net/geometry/polygonise/


class ndMarchingCubeIsoSurface::ndIsoCell
{
	public:
	ndVector m_isoValues[8];
};

ndMarchingCubeIsoSurface::ndMarchingCubeIsoSurface(ndThreadPool* const threadPool, const ndVector& boxP0, const ndVector& boxP1, ndFloat32 gridSize)
	:ndMarchingCubes(threadPool, gridSize)
	,m_boxP0(boxP0 & ndVector::m_triplexMask)
	,m_boxP1(boxP1 & ndVector::m_triplexMask)
	,m_densityWindow0()
	,m_densityWindow1()
{
	SetBox(boxP0, boxP1);
}

ndMarchingCubeIsoSurface::~ndMarchingCubeIsoSurface()
{
}

void ndMarchingCubeIsoSurface::GetBox(ndVector& boxP0, ndVector& boxP1) const
{
	boxP0 = m_boxP0;
	boxP1 = m_boxP1;
}

void ndMarchingCubeIsoSurface::SetBox(const ndVector& boxP0, const ndVector& boxP1)
{
	m_boxP0 = boxP0 & ndVector::m_triplexMask;
	m_boxP1 = boxP1 & ndVector::m_triplexMask;
}


ndVector ndMarchingCubeIsoSurface::PositionToGrid(const ndVector& posit) const
{
	return m_invGridSize * (posit - m_boxP0);
}

void ndMarchingCubeIsoSurface::GenerateIndexList()
{
	class ndKey_lowX
	{
		public:
		ndKey_lowX(void* const) {}
		ndInt32 GetKey(const ndVector& point) const
		{
			const ndFloat32 intBase = ndFloor(point.m_x);
			const ndInt32 frac = ((point.m_x - intBase) > ndFloat32(0.0f)) ? 1 : 0;
			const ndInt32 key = 2 * ndInt32(intBase) + frac;
			return key & 0xff;
		}
	};
	
	class ndKey_highX
	{
		public:
		ndKey_highX(void* const) {}
		ndInt32 GetKey(const ndVector& point) const
		{
			const ndFloat32 intBase = ndFloor(point.m_x);
			const ndInt32 frac = ((point.m_x - intBase) > ndFloat32(0.0f)) ? 1 : 0;
			const ndInt32 key = (2 * ndInt32(intBase) + frac) >> 8;
			return key & 0xff;
		}
	};
	
	class ndKey_lowY
	{
		public:
		ndKey_lowY(void* const) {}
		ndInt32 GetKey(const ndVector& point) const
		{
			const ndFloat32 intBase = ndFloor(point.m_y);
			const ndInt32 frac = ((point.m_y - intBase) > ndFloat32(0.0f)) ? 1 : 0;
			const ndInt32 key = 2 * ndInt32(intBase) + frac;
			return key & 0xff;
		}
	};
	
	class ndKey_highY
	{
		public:
		ndKey_highY(void* const) {}
		ndInt32 GetKey(const ndVector& point) const
		{
			const ndFloat32 intBase = ndFloor(point.m_y);
			const ndInt32 frac = ((point.m_y - intBase) > ndFloat32(0.0f)) ? 1 : 0;
			const ndInt32 key = (2 * ndInt32(intBase) + frac) >> 8;
			return key & 0xff;
		}
	};
	
	class ndKey_lowZ
	{
		public:
		ndKey_lowZ(void* const) {}
		ndInt32 GetKey(const ndVector& point) const
		{
			const ndFloat32 intBase = ndFloor(point.m_z);
			const ndInt32 frac = ((point.m_z - intBase) > ndFloat32(0.0f)) ? 1 : 0;
			const ndInt32 key = 2 * ndInt32(intBase) + frac;
			return key & 0xff;
		}
	};
	
	class ndKey_highZ
	{
		public:
		ndKey_highZ(void* const) {}
		ndInt32 GetKey(const ndVector& point) const
		{
			const ndFloat32 intBase = ndFloor(point.m_z);
			const ndInt32 frac = ((point.m_z - intBase) > ndFloat32(0.0f)) ? 1 : 0;
			const ndInt32 key = (2 * ndInt32(intBase) + frac) >> 8;
			return key & 0xff;
		}
	};
	
	for (ndInt32 i = 0; i < ndInt32 (m_meshPoints.GetCount()); ++i)
	{
		m_meshPoints[i].m_w = ndFloat32(i);
	}

	m_meshNormals.SetCount(m_meshPoints.GetCount());
	ndCountingSort<ndVector, ndKey_lowX, 8>(m_meshPoints, m_meshNormals, nullptr, nullptr);
	ndCountingSort<ndVector, ndKey_highX, 8>(m_meshPoints, m_meshNormals, nullptr, nullptr);
	ndCountingSort<ndVector, ndKey_lowY, 8>(m_meshPoints, m_meshNormals, nullptr, nullptr);
	ndCountingSort<ndVector, ndKey_highY, 8>(m_meshPoints, m_meshNormals, nullptr, nullptr);
	ndCountingSort<ndVector, ndKey_lowZ, 8>(m_meshPoints, m_meshNormals, nullptr, nullptr);
	ndCountingSort<ndVector, ndKey_highZ, 8>(m_meshPoints, m_meshNormals, nullptr, nullptr);

	class ndHash : public ndGridHash
	{
		public:
		ndHash(const ndVector& p)
			:ndGridHash()
		{
			auto GetHash = [this](ndFloat32 x)
			{
				const ndFloat32 intBase = ndFloor(x);
				const ndInt32 frac = ((x - intBase) > ndFloat32(0.0f)) ? 1 : 0;
				const ndInt32 hash = 2 * ndInt32(intBase) + frac;
				return ndInt16(hash);
			};
			m_x = GetHash(p.m_x);
			m_y = GetHash(p.m_y);
			m_z = GetHash(p.m_z);
			m_cellType = 0;
		}
	};

	ndInt32 vertexCount = 0;
	m_meshIndices.SetCount(m_meshPoints.GetCount());
	m_meshPoints.PushBack(ndVector (ndFloat32 (128 * 256)));
	for (ndInt32 i = 0; i < ndInt32(m_meshPoints.GetCount()) - 1; ++i)
	{
		const ndInt32 index = ndInt32(m_meshPoints[i].m_w);
		m_meshIndices[index] = vertexCount;
		m_meshPoints[vertexCount] = m_meshPoints[i] & ndVector::m_triplexMask;
	
		const ndHash hash0(m_meshPoints[i]);
		for (i = i + 1; i < ndInt32(m_meshPoints.GetCount()); ++i)
		{
			const ndHash hash1(m_meshPoints[i]);
			if (hash0.m_gridFullHash != hash1.m_gridFullHash)
			{
				break;
			}
			const ndInt32 duplicateIndex = ndInt32(m_meshPoints[i].m_w);
			m_meshIndices[duplicateIndex] = vertexCount;
		}
		--i;
		vertexCount++;
	}
	m_meshPoints.SetCount(vertexCount);
	m_meshNormals.SetCount(vertexCount);
	
	auto ApplyScale = [this](ndInt32 groupId)
	{
		m_meshNormals[groupId] = ndVector::m_zero;
		m_meshPoints[groupId] = m_meshPoints[groupId] * m_gridSize + m_boxP0;
	};
	for (ndInt32 i = 0; i < ndInt32(m_meshPoints.GetCount()); ++i)
	{
		ApplyScale(i);
	}

	auto CalculateVertexNormals = [this](ndInt32 groupId)
	{
		const ndInt32 index = groupId * 3;
		const ndInt32 id0 = m_meshIndices[index + 0];
		const ndInt32 id1 = m_meshIndices[index + 1];
		const ndInt32 id2 = m_meshIndices[index + 2];

		const ndVector p0(m_meshPoints[id0]);
		const ndVector p1(m_meshPoints[id1]);
		const ndVector p2(m_meshPoints[id2]);

		const ndVector e_10(p1 - p0);
		const ndVector e_20(p2 - p0);
		const ndVector normal(e_10.CrossProduct(e_20) & ndVector::m_triplexMask);
		m_meshNormals[id0] += normal;
		m_meshNormals[id1] += normal;
		m_meshNormals[id2] += normal;
	};

	const ndInt32 triangleCount = ndInt32(m_meshIndices.GetCount()) / 3;
	// need to build an edge list to parallelize this,
	// for now just add the contributions 
	//m_threadPool->ParallelExecute(CalculateVertexNormals, triangleCount, jobStride);
	for (ndInt32 i = 0; i < triangleCount; ++i)
	{
		CalculateVertexNormals(i);
	}

	// Normalize normals.
	for (ndInt32 i = 0; i < ndInt32(m_meshPoints.GetCount()); ++i)
	{
		m_meshNormals[i] = m_meshNormals[i].Normalize();
	}
}

void ndMarchingCubeIsoSurface::GenerateMesh()
{
	const ndVector boxSize(m_boxP1 - m_boxP0);
	const ndVector boxSizeInGrids((boxSize * m_invGridSize).Ceiling().GetInt());

	ndFloat32 high = m_boxP0.m_y;
	auto ReadLayerDensity = [this, &boxSizeInGrids, &high]()
	{
		ndInt32 row = 0;
		ndFloat32 posit_z = m_boxP0.m_z;
		const ndInt32 stride = boxSizeInGrids.m_ix;
		for (ndInt32 z = 0; z < boxSizeInGrids.m_iz; ++z)
		{
			ndFloat32 posit_x = m_boxP0.m_x;
			for (ndInt32 x = 0; x < boxSizeInGrids.m_ix; ++x)
			{
				const ndVector posit(posit_x, high, posit_z, ndFloat32(0.0f));
				m_densityWindow1[row + x] = GetIsoValue(posit);
				posit_x += m_gridSize.m_x;
			}
			posit_z += m_gridSize.m_z;
			row += stride;
		}
	};

	m_densityWindow0.SetCount(boxSizeInGrids.m_ix * boxSizeInGrids.m_iz);
	m_densityWindow1.SetCount(boxSizeInGrids.m_ix * boxSizeInGrids.m_iz);

	ndFloat32 grid_y0 = ndFloat32(0.0f);
	ndFloat32 grid_y1 = ndFloat32(1.0f);
	ReadLayerDensity();
	m_densityWindow0.Swap(m_densityWindow1);

	for (ndInt32 y = 1; y < (boxSizeInGrids.m_iy - 1); ++y)
	{
		high += m_gridSize.m_y;
		ReadLayerDensity();

		ndInt32 stride = 0;
		ndFloat32 grid_Z0 = ndFloat32(0.0f);
		ndFloat32 grid_Z1 = ndFloat32(1.0f);
		for (ndInt32 z = 0; z < boxSizeInGrids.m_iz - 1; ++z)
		{
			for (ndInt32 x = 0; x < boxSizeInGrids.m_ix - 1; ++x)
			{
				ndIsoCell cell;
				const ndInt32 i0 = stride + x;
				const ndInt32 i1 = stride + x + 1;
				const ndInt32 i2 = boxSizeInGrids.m_ix + stride + x + 1;
				const ndInt32 i3 = boxSizeInGrids.m_ix + stride + x;

				ndFloat32 isoValues[8];
				isoValues[0] = m_densityWindow0[i0];
				isoValues[1] = m_densityWindow0[i1];
				isoValues[2] = m_densityWindow0[i2];
				isoValues[3] = m_densityWindow0[i3];
				isoValues[4] = m_densityWindow1[i0];
				isoValues[5] = m_densityWindow1[i1];
				isoValues[6] = m_densityWindow1[i2];
				isoValues[7] = m_densityWindow1[i3];

				ndInt32 tableIndex = 0;
				for (ndInt32 i = 0; i < 8; ++i)
				{
					tableIndex |= (isoValues[i] <= 0.0f) << i;
				}

				const ndInt32 edgeStart = m_edgeScan[tableIndex];
				const ndInt32 edgeCount = m_edgeScan[tableIndex + 1] - edgeStart;
				if (edgeCount)
				{
					cell.m_isoValues[0] = ndVector(ndFloat32(x + 0), grid_y0, grid_Z0, isoValues[0]);
					cell.m_isoValues[1] = ndVector(ndFloat32(x + 1), grid_y0, grid_Z0, isoValues[1]);
					cell.m_isoValues[2] = ndVector(ndFloat32(x + 1), grid_y0, grid_Z1, isoValues[2]);
					cell.m_isoValues[3] = ndVector(ndFloat32(x + 0), grid_y0, grid_Z1, isoValues[3]);
					cell.m_isoValues[4] = ndVector(ndFloat32(x + 0), grid_y1, grid_Z0, isoValues[4]);
					cell.m_isoValues[5] = ndVector(ndFloat32(x + 1), grid_y1, grid_Z0, isoValues[5]);
					cell.m_isoValues[6] = ndVector(ndFloat32(x + 1), grid_y1, grid_Z1, isoValues[6]);
					cell.m_isoValues[7] = ndVector(ndFloat32(x + 0), grid_y1, grid_Z1, isoValues[7]);

					ndVector vertlist[12];
					for (ndInt32 i = 0; i < edgeCount; ++i)
					{
						const ndEdge& edge = m_edges[edgeStart + i];
						const ndInt32 midPoint = edge.m_midPoint;
							
						const ndVector p0(cell.m_isoValues[edge.m_p0]);
						const ndVector p1(cell.m_isoValues[edge.m_p1]);
						ndAssert((p1.m_w * p0.m_w) <= ndFloat32(0.0f));
						const ndVector p1p0(p1 - p0);
						ndFloat32 param = p0.m_w / (p1.m_w - p0.m_w);
						const ndVector p3 (ndVector::m_triplexMask & (p0 - p1p0.Scale(param)));
						vertlist[midPoint] = p3;
					}

					const ndInt32 faceStart = m_facesScan[tableIndex];
					const ndInt32 triangleStart = m_facesScan[tableIndex];
					const ndInt32 triangleCount = m_facesScan[tableIndex + 1] - triangleStart;
					for (ndInt32 i = 0; i < triangleCount; ++i)
					{
						const ndInt32 j0 = m_faces[faceStart + i][0];
						const ndInt32 j1 = m_faces[faceStart + i][1];
						const ndInt32 j2 = m_faces[faceStart + i][2];

						const ndVector& p0 = vertlist[j0];
						const ndVector& p1 = vertlist[j1];
						const ndVector& p2 = vertlist[j2];
						const ndVector p10(p1 - p0);
						const ndVector p20(p2 - p0);
						const ndVector area(p10.CrossProduct(p20));
						ndFloat32 areaMag2 = area.DotProduct(area & ndVector::m_triplexMask).GetScalar();
						if (areaMag2 > ndFloat32(1.0e-6f))
						{
							m_meshPoints.PushBack(p0);
							m_meshPoints.PushBack(p1);
							m_meshPoints.PushBack(p2);
						}
						else
						{
							areaMag2 *= -1;
						}
					}
				}
			}
			stride += boxSizeInGrids.m_ix;
			grid_Z0 += ndFloat32(1.0f);
			grid_Z1 += ndFloat32(1.0f);
		}

		grid_y0 += ndFloat32(1.0f);
		grid_y1 += ndFloat32(1.0f);
		m_densityWindow0.Swap(m_densityWindow1);
	}
	high += m_gridSize.m_y;

	GenerateIndexList();
}
