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
#include "ndMarchingCubes.h"

// adapted from code by written by Paul Bourke may 1994
//http://paulbourke.net/geometry/polygonise/

class ndMarchingCubes::ndKey_xlow
{
	public:
	ndKey_xlow(void* const) {}
	ndInt32 GetKey(const ndMarchingCubes::ndGridHash& cell) const
	{
		return cell.m_xLow;
	}
};

class ndMarchingCubes::ndKey_xhigh
{
	public:
	ndKey_xhigh(void* const) {}
	ndInt32 GetKey(const ndMarchingCubes::ndGridHash& cell) const
	{
		return cell.m_xHigh;
	}
};

class ndMarchingCubes::ndKey_ylow
{
	public:
	ndKey_ylow(void* const) {}
	ndInt32 GetKey(const ndMarchingCubes::ndGridHash& cell) const
	{
		return cell.m_yLow;
	}
};

class ndMarchingCubes::ndKey_yhigh
{
	public:
	ndKey_yhigh(void* const) {}
	ndInt32 GetKey(const ndMarchingCubes::ndGridHash& cell) const
	{
		return cell.m_yHigh;
	}
};

class ndMarchingCubes::ndKey_zlow
{
	public:
	ndKey_zlow(void* const) {}
	ndInt32 GetKey(const ndMarchingCubes::ndGridHash& cell) const
	{
		return cell.m_zLow;
	}
};

class ndMarchingCubes::ndKey_zhigh
{
	public:
	ndKey_zhigh(void* const) {}
	ndInt32 GetKey(const ndMarchingCubes::ndGridHash& cell) const
	{
		return cell.m_zHigh;
	}
};

class ndMarchingCubes::ndIsoCell
{
	public:
	ndVector m_isoValues[8];
};

class ndMarchingCubes::ndGridHashSteps
{
	public:
	ndGridHashSteps()
	{
		m_steps[0] = ndGridHash(-1, -1, -1);
		m_steps[1] = ndGridHash(0, -1, -1);
		m_steps[2] = ndGridHash(-1, 0, -1);
		m_steps[3] = ndGridHash(0, 0, -1);
		m_steps[4] = ndGridHash(-1, -1, 0);
		m_steps[5] = ndGridHash(0, -1, 0);
		m_steps[6] = ndGridHash(-1, 0, 0);
		m_steps[7] = ndGridHash(0, 0, 0);

		m_cellType[0] = 5;
		m_cellType[1] = 6;
		m_cellType[2] = 1;
		m_cellType[3] = 2;
		m_cellType[4] = 4;
		m_cellType[5] = 7;
		m_cellType[6] = 0;
		m_cellType[7] = 3;
	}

	ndGridHash m_steps[8];
	ndUnsigned8 m_cellType[8];
};

ndVector ndMarchingCubes::ndCalculateIsoValue::m_gridCorners[] =
{
	ndVector(ndFloat32(0.0f), ndFloat32(-1.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.0f), ndFloat32(-1.0f), ndFloat32(0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(-1.0f), ndFloat32(-1.0f), ndFloat32(0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(-1.0f), ndFloat32(-1.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(-1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(-1.0f), ndFloat32(0.0f), ndFloat32(-1.0f), ndFloat32(0.0f))
};

ndInt32 ndMarchingCubes::ndCalculateIsoValue::m_facesScan[] = { 0,0,1,2,4,5,7,9,12,13,15,17,20,22,25,28,30,31,33,35,38,40,43,46,50,52,55,58,62,65,69,73,76,77,79,81,84,86,89,92,96,98,101,104,108,111,115,119,122,124,127,130,132,135,139,143,146,149,153,157,160,164,169,174,176,177,179,181,184,186,189,192,196,198,201,204,208,211,215,219,222,224,227,230,234,237,241,245,250,253,257,261,266,270,275,280,284,286,289,292,296,299,303,305,308,311,315,319,324,328,333,336,338,341,345,349,352,356,361,364,366,370,375,380,384,389,391,395,396,397,399,401,404,406,409,412,416,418,421,424,428,431,435,439,442,444,447,450,454,457,461,465,470,473,475,479,482,486,489,494,496,498,501,504,508,511,515,519,524,527,531,535,540,544,549,554,558,561,565,569,572,576,581,586,590,594,597,602,604,609,613,615,616,618,621,624,628,631,635,639,644,647,651,655,660,662,665,668,670,673,677,681,686,690,695,700,702,706,709,714,718,721,723,727,728,731,735,739,744,748,753,756,760,764,769,774,776,779,783,785,786,788,791,794,796,799,803,805,806,809,811,815,816,818,819,820 };


ndMarchingCubes::ndGridHash::ndGridHash()
{
}

ndMarchingCubes::ndGridHash::ndGridHash(const ndGridHash& src, ndUnsigned16 cellType)
{
	m_gridFullHash = src.m_gridFullHash;
	m_cellType = cellType;
}

ndMarchingCubes::ndGridHash::ndGridHash(ndInt32 x, ndInt32 y, ndInt32 z)
{
	m_gridFullHash = 0;
	m_x = ndUnsigned16(x);
	m_y = ndUnsigned16(y);
	m_z = ndUnsigned16(z);
}

ndMarchingCubes::ndGridHash::ndGridHash(const ndVector& grid)
{
	ndAssert(grid.m_x >= ndFloat32(0.0f));
	ndAssert(grid.m_y >= ndFloat32(0.0f));
	ndAssert(grid.m_z >= ndFloat32(0.0f));
	ndAssert(grid.m_x < ndFloat32(256.0f * 256.0f));
	ndAssert(grid.m_y < ndFloat32(256.0f * 256.0f));
	ndAssert(grid.m_z < ndFloat32(256.0f * 256.0f));

	ndVector hash(grid.Floor().GetInt());
	m_gridFullHash = 0;
	m_x = ndUnsigned16(hash.m_ix);
	m_y = ndUnsigned16(hash.m_iy);
	m_z = ndUnsigned16(hash.m_iz);
}

// ***********************************************************
//
// ***********************************************************
ndMarchingCubes::ndCalculateIsoValue::ndCalculateIsoValue(ndFloat32 gridSize)
	:m_gridSize(gridSize)
	,m_invGridSize(ndFloat32 (1.0f)/ gridSize)
	,m_isoSufaceValue(ndFloat32(0.5f))
{
}

ndMarchingCubes::ndCalculateIsoValue::~ndCalculateIsoValue()
{
}

// ***********************************************************
//
// ***********************************************************
ndMarchingCubes::ndMarchingCubes()
{
}

ndMarchingCubes::~ndMarchingCubes()
{
}

void ndMarchingCubes::GenerateMesh(ndCalculateIsoValue* const computeIsoValue)
{
	computeIsoValue->GenerateMesh();
}


// ***********************************************************
//
// ***********************************************************
ndMarchingCubeParticleIsoValue::ndMarchingCubeParticleIsoValue(ndThreadPool* const threadPool, ndFloat32 gridSize)
	:ndCalculateIsoValue(gridSize)
	,m_threadPool(threadPool)
{
}

ndMarchingCubeParticleIsoValue::~ndMarchingCubeParticleIsoValue()
{
}

void ndMarchingCubeParticleIsoValue::GenerateMesh()
{
	m_cellScans.SetCount(0);
	m_hashGridMap.SetCount(0);
	m_cellTrainglesScans.SetCount(0);
	m_hashGridMapScratchBuffer.SetCount(0);

	CalculateAABB();
	RemoveDuplicates();
	GenerateGrids();
	GenerateTriangles();
}

void ndMarchingCubeParticleIsoValue::CalculateAABB()
{
	ndFixSizeArray<ndVector, D_MAX_THREADS_COUNT * 2> partialAABB(m_threadPool->GetThreadCount() * 2);
	auto CalculateAABB = ndMakeObject::ndFunction([this, &partialAABB](ndInt32 groupId, ndInt32 threadIndex)
	{
		const ndVector p(m_points[groupId]);
		partialAABB[threadIndex * 2 + 0] = p.GetMin(partialAABB[threadIndex * 2 + 0]);
		partialAABB[threadIndex * 2 + 1] = p.GetMax(partialAABB[threadIndex * 2 + 1]);
	});

	for (ndInt32 i = 0; i < m_threadPool->GetThreadCount(); ++i)
	{
		partialAABB[i * 2 + 0] = ndVector(ndFloat32(1.0e10f));
		partialAABB[i * 2 + 1] = ndVector(ndFloat32(-1.0e10f));
	}
	const ndInt32 jobStride = 256;
	const ndInt32 count = ndInt32(m_points.GetCount());
	m_threadPool->ParallelExecute(CalculateAABB, count, jobStride);

	ndVector boxP0(partialAABB[0]);
	ndVector boxP1(partialAABB[1]);
	for (ndInt32 i = 1; i < m_threadPool->GetThreadCount(); ++i)
	{
		boxP0 = boxP0.GetMin(partialAABB[i * 2 + 0]);
		boxP1 = boxP1.GetMax(partialAABB[i * 2 + 1]);
	}
	boxP0 -= m_gridSize;
	boxP1 += m_gridSize;

	boxP0 = m_gridSize * (boxP0 * m_invGridSize).Floor();
	boxP1 = m_gridSize * (boxP1 * m_invGridSize).Ceiling();

	m_boxP0 = boxP0 & ndVector::m_triplexMask;
	m_boxP1 = boxP1 & ndVector::m_triplexMask;
}

void ndMarchingCubeParticleIsoValue::RemoveDuplicates()
{
	m_hashGridMapScratchBuffer.SetCount(m_points.GetCount());
	ndFixSizeArray<ndMarchingCubes::ndUpperDigit, D_MAX_THREADS_COUNT> upperDigit(m_threadPool->GetThreadCount());
	auto CalculateHashes = ndMakeObject::ndFunction([this, &upperDigit](ndInt32 groupId, ndInt32 threadIndex)
	{
		ndMarchingCubes::ndUpperDigit upperDigitsIsValid(upperDigit[threadIndex]);
		 
		const ndVector r(m_points[groupId] - m_boxP0);
		const ndVector p(r * m_invGridSize);
		const ndMarchingCubes::ndGridHash hashKey(p);
		m_hashGridMapScratchBuffer[groupId] = hashKey;
		
		upperDigitsIsValid.m_x = ndMax(upperDigitsIsValid.m_x, ndInt32(hashKey.m_xHigh));
		upperDigitsIsValid.m_y = ndMax(upperDigitsIsValid.m_y, ndInt32(hashKey.m_yHigh));
		upperDigitsIsValid.m_z = ndMax(upperDigitsIsValid.m_z, ndInt32(hashKey.m_zHigh));
		upperDigit[threadIndex] = upperDigitsIsValid;
	});

	const ndInt32 jobStride = 256;
	const ndInt32 count = ndInt32(m_points.GetCount());
	m_threadPool->ParallelExecute(CalculateHashes, count, jobStride);

	ndMarchingCubes::ndUpperDigit upperDigitsIsValid(upperDigit[0]);
	for (ndInt32 i = 1; i < m_threadPool->GetThreadCount(); ++i)
	{
		const ndMarchingCubes::ndUpperDigit digitsIsValid(upperDigit[i]);
		upperDigitsIsValid.m_x = ndMax(upperDigitsIsValid.m_x, digitsIsValid.m_x);
		upperDigitsIsValid.m_y = ndMax(upperDigitsIsValid.m_y, digitsIsValid.m_y);
		upperDigitsIsValid.m_z = ndMax(upperDigitsIsValid.m_z, digitsIsValid.m_z);
	}
	m_upperDigitsIsValid = upperDigitsIsValid;

	ndCountingSort<ndMarchingCubes::ndGridHash, ndMarchingCubes::ndKey_xlow, 8>(*m_threadPool, m_hashGridMapScratchBuffer, m_hashGridMap, nullptr, nullptr);
	if (m_upperDigitsIsValid.m_x)
	{
		ndCountingSort<ndMarchingCubes::ndGridHash, ndMarchingCubes::ndKey_xhigh, 8>(*m_threadPool, m_hashGridMapScratchBuffer, m_hashGridMap, nullptr, nullptr);
	}
	
	ndCountingSort<ndMarchingCubes::ndGridHash, ndMarchingCubes::ndKey_ylow, 8>(*m_threadPool, m_hashGridMapScratchBuffer, m_hashGridMap, nullptr, nullptr);
	if (m_upperDigitsIsValid.m_y)
	{
		ndCountingSort<ndMarchingCubes::ndGridHash, ndMarchingCubes::ndKey_yhigh, 8>(*m_threadPool, m_hashGridMapScratchBuffer, m_hashGridMap, nullptr, nullptr);
	}
	
	ndCountingSort<ndMarchingCubes::ndGridHash, ndMarchingCubes::ndKey_zlow, 8>(*m_threadPool, m_hashGridMapScratchBuffer, m_hashGridMap, nullptr, nullptr);
	if (m_upperDigitsIsValid.m_z)
	{
		ndCountingSort<ndMarchingCubes::ndGridHash, ndMarchingCubes::ndKey_zhigh, 8>(*m_threadPool, m_hashGridMapScratchBuffer, m_hashGridMap, nullptr, nullptr);
	}
	
	ndInt32 gridCount = 0;
	for (ndInt32 i = 1; i < m_hashGridMapScratchBuffer.GetCount(); ++i)
	{
		const ndMarchingCubes::ndGridHash cell(m_hashGridMapScratchBuffer[i]);
		ndAssert(cell.m_cellType == 0);
		ndAssert(m_hashGridMapScratchBuffer[i - 1].m_cellType == 0);
		const ndInt32 uniqueGrid = (cell.m_gridFullHash != m_hashGridMapScratchBuffer[i - 1].m_gridFullHash) ? 1 : 0;
		gridCount += uniqueGrid;
		m_hashGridMapScratchBuffer[gridCount] = cell;
	}
	gridCount++;
	m_hashGridMapScratchBuffer.SetCount(gridCount);

//ndAssert(!xxxxxxx || (xxxxxxx == m_hashGridMapScratchBuffer.GetCount()));
//if (xxxxxxx)
//{
//	ndAssert(xxxxxxx == m_hashGridMapScratchBuffer.GetCount());
//	//ndAssert(xxxxx.GetCount() == m_hashGridMapScratchBuffer.GetCount());
////	for (ndInt32 i = 0; i < m_hashGridMapScratchBuffer.GetCount(); ++i)
////	{
////		ndAssert (xxxxx[i].m_gridFullHash == m_hashGridMapScratchBuffer[i].m_gridFullHash);
////	}
//}
}

void ndMarchingCubeParticleIsoValue::GenerateGrids()
{
	const ndMarchingCubes::ndGridHashSteps steps;
	m_hashGridMap.SetCount(m_hashGridMapScratchBuffer.GetCount() * 8);
	auto GenerateGrids = ndMakeObject::ndFunction([this, &steps](ndInt32 groupId, ndInt32)
	{
		const ndInt32 gridStartIndex = groupId;
		const ndMarchingCubes::ndGridHash hashKey(m_hashGridMapScratchBuffer[gridStartIndex]);
		for (ndInt32 j = 0; j < 8; ++j)
		{
			ndMarchingCubes::ndGridHash cell(hashKey);
			cell.m_x += steps.m_steps[j].m_x;
			cell.m_y += steps.m_steps[j].m_y;
			cell.m_z += steps.m_steps[j].m_z;
			cell.m_cellType = steps.m_cellType[j];
			ndAssert(cell.m_x < 0x7fff);
			ndAssert(cell.m_y < 0x7fff);
			ndAssert(cell.m_z < 0x7fff);
			m_hashGridMap[gridStartIndex * 8 + j] = cell;
		}
	});

	const ndInt32 jobStride = 256;
	const ndInt32 count = ndInt32(m_hashGridMapScratchBuffer.GetCount());
	m_threadPool->ParallelExecute(GenerateGrids, count, jobStride);

	ndCountingSort<ndMarchingCubes::ndGridHash, ndMarchingCubes::ndKey_xlow, 8>(*m_threadPool, m_hashGridMap, m_hashGridMapScratchBuffer, nullptr, nullptr);
	if (m_upperDigitsIsValid.m_x)
	{
		ndCountingSort<ndMarchingCubes::ndGridHash, ndMarchingCubes::ndKey_xhigh, 8>(*m_threadPool, m_hashGridMap, m_hashGridMapScratchBuffer, nullptr, nullptr);
	}
	
	ndCountingSort<ndMarchingCubes::ndGridHash, ndMarchingCubes::ndKey_ylow, 8>(*m_threadPool, m_hashGridMap, m_hashGridMapScratchBuffer, nullptr, nullptr);
	if (m_upperDigitsIsValid.m_y)
	{
		ndCountingSort<ndMarchingCubes::ndGridHash, ndMarchingCubes::ndKey_yhigh, 8>(*m_threadPool, m_hashGridMap, m_hashGridMapScratchBuffer, nullptr, nullptr);
	}
	
	ndCountingSort<ndMarchingCubes::ndGridHash, ndMarchingCubes::ndKey_zlow, 8>(*m_threadPool, m_hashGridMap, m_hashGridMapScratchBuffer, nullptr, nullptr);
	if (m_upperDigitsIsValid.m_z)
	{
		ndCountingSort<ndMarchingCubes::ndGridHash, ndMarchingCubes::ndKey_zhigh, 8>(*m_threadPool, m_hashGridMap, m_hashGridMapScratchBuffer, nullptr, nullptr);
	}

	ndInt32 start = 0;
	m_cellScans.SetCount(0);
	for (ndInt32 i = 0; i < ndInt32 (m_hashGridMap.GetCount()) - 1; ++i)
	{
		const ndMarchingCubes::ndGridHash hash (m_hashGridMap[i], 0);
		for (ndInt32 j = i + 1; j < ndInt32(m_hashGridMap.GetCount()); ++j)
		{
			const ndMarchingCubes::ndGridHash hash1(m_hashGridMap[j], 0);
			if (hash.m_gridFullHash != hash1.m_gridFullHash)
			{
				i = j;
				break;
			}
		}
		ndAssert((i - start) > 0);
		ndAssert((i - start) <= 8);
		m_cellScans.PushBack(i - start);
		start = i;
		i--;
	}
	ndAssert((ndInt32(m_hashGridMap.GetCount()) - start) > 0);
	ndAssert((ndInt32(m_hashGridMap.GetCount()) - start) <= 8);
	m_cellScans.PushBack(ndInt32(m_hashGridMap.GetCount()) - start);

	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < ndInt32(m_cellScans.GetCount()); ++i)
	{
		ndInt32 sum1 = sum + m_cellScans[i];
		m_cellScans[i] = sum;
		sum = sum1;
	}
	m_cellScans.PushBack(sum);
	m_cellTrainglesScans.SetCount(m_cellScans.GetCount());
	ndAssert(sum == m_hashGridMap.GetCount());
}

void ndMarchingCubeParticleIsoValue::GenerateTriangles()
{
	auto CountTriangles = ndMakeObject::ndFunction([this](ndInt32 groupId, ndInt32)
	{
		//ndFloat32 isoSufaceValue = 0.5f;
		m_cellTrainglesScans[groupId] = 0;
		const ndInt32 cellStart = m_cellScans[groupId];
		const ndInt32 cellCount = m_cellScans[groupId + 1] - cellStart;
			
		if (cellCount < 8)
		{
			const ndMarchingCubes::ndGridHash startGrid(m_hashGridMap[cellStart], 0);
		
			ndMarchingCubes::ndIsoCell cell;
			ndVector* const isoValue = &cell.m_isoValues[0];
			ndVector origin(ndFloat32(startGrid.m_x + 1), ndFloat32(startGrid.m_y + 1), ndFloat32(startGrid.m_z + 1), ndFloat32(0.0f));
			for (ndInt32 j = 0; j < 8; j++)
			{
				isoValue[j] = origin + m_gridCorners[j];
			}
		
			for (ndInt32 j = 0; j < cellCount; j++)
			{
				ndInt32 index = m_hashGridMap[cellStart + j].m_cellType;
				isoValue[index].m_w = ndFloat32(1.0f);
			}
		
			ndInt32 tableIndex = 0;
			for (ndInt32 j = 0; j < 8; ++j)
			{
				tableIndex |= (cell.m_isoValues[j].m_w > m_isoSufaceValue) << j;
			}
		
			const ndInt32 triangleStart = m_facesScan[tableIndex];
			const ndInt32 triangleCount = m_facesScan[tableIndex + 1] - triangleStart;
			m_cellTrainglesScans[groupId] = triangleCount;
		}
	});

	const ndInt32 jobStride = 256;
	const ndInt32 count = ndInt32(m_cellScans.GetCount())-1;
	m_threadPool->ParallelExecute(CountTriangles, count, jobStride);

	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndInt32 sum1 = sum + m_cellTrainglesScans[i];
		m_cellTrainglesScans[i] = sum;
		sum = sum1;
	}
	m_cellTrainglesScans[count] = sum;
}
