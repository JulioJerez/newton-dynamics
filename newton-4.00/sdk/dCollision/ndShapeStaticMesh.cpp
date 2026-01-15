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
#include "ndCollisionStdafx.h"
#include "ndShapeInstance.h"
#include "ndContactSolver.h"
#include "ndShapeStaticMesh.h"
#include "ndPolygonMeshDesc.h"

void ndShapeStaticMesh::ndPatchMesh::GetFacesPatch(ndPolygonMeshDesc* const data)
{
	ndFixSizeArray<ndInt32, MESH_SIZE> patchScan(0);
	if (m_vertexArrayHasDuplicated)
	{
		// remove all vertex duplicated
		patchScan.SetCount(m_pointArray.GetCount());
		ndInt32 vertexCount = ndVertexListToIndexList(&m_pointArray[0].m_x, sizeof(ndVector), 3, ndInt32(m_pointArray.GetCount()), &patchScan[0]);
		m_pointArray.SetCount(vertexCount);

		// remap index array
		for (ndInt32 i = 0; i < ndInt32(m_indexArray.GetCount()); ++i)
		{
			ndInt32 index = m_indexArray[i];
			m_indexArray[i] = patchScan[index];
		}
		patchScan.SetCount(0);
	}

	// generate the scan prefit
	ndInt32 scanSum = 0;
	for (ndInt32 i = 0; i < m_faceArray.GetCount(); ++i)
	{
		ndInt32 count = m_faceArray[i];
		m_faceArray[i] = scanSum;
		scanSum += count;
	}
	m_faceArray.PushBack(scanSum);

	// build the mesh
	ndArray<ndVector>& vertex = data->m_proceduralStaticMeshFaceQuery->m_vertex;
	vertex.SetCount(m_pointArray.GetCount() + m_faceArray.GetCount() - 1);

	// add all the vertices
	for (ndInt32 i = 0; i < m_pointArray.GetCount(); ++i)
	{
		vertex[i] = m_pointArray[i];
	}

	// add all faces
	const ndInt32 normalStart = ndInt32(m_pointArray.GetCount());

	ndPolygonMeshDesc::ndStaticMeshFaceQuery& query = *data->m_staticMeshQuery;
	ndArray<ndInt32>& indexArray = query.m_faceVertexIndex;
	ndArray<ndInt32>& faceIndexCount = query.m_faceIndexCount;

	ndInt32 patchScanSum = 0;
	for (ndInt32 i = 0; i < m_faceArray.GetCount() - 1; ++i)
	{
		const ndInt32 indexStart = m_faceArray[i];
		const ndInt32 indexCount = m_faceArray[i + 1] - indexStart;

		vertex[normalStart + i] = m_normalArray[i];

		patchScan.PushBack(patchScanSum);
		patchScanSum += indexCount * 2 + 3;

		// push the face number of vertices
		faceIndexCount.PushBack(indexCount);

		// push the face indices
		for (ndInt32 j = 0; j < indexCount; ++j)
		{
			indexArray.PushBack(m_indexArray[indexStart + j]);
		}
		// push the material 
		indexArray.PushBack(m_faceMaterialArray[i]);

		// push the normal index
		for (ndInt32 j = 0; j < indexCount + 1; ++j)
		{
			indexArray.PushBack(normalStart + i);
		}

		// push the face area, for now just asume 1;
		indexArray.PushBack(1);
	}

	// add the adjacency info
	ndFixSizeArray<ndEdgeList, MESH_SIZE * 4> egdeArray(0);
	for (ndInt32 i = 0; i < m_faceArray.GetCount() - 1; ++i)
	{
		const ndInt32 indexStart = m_faceArray[i];
		const ndInt32 indexCount = m_faceArray[i + 1] - indexStart;
		ndInt32 v0 = m_indexArray[indexStart + indexCount - 1];
		for (ndInt32 j = 0; j < indexCount; ++j)
		{
			ndInt32 v1 = m_indexArray[indexStart + j];
			ndEdgeList edge;
			edge.m_key = (ndMin(v0, v1) << 16) + ndMax(v0, v1);
			edge.m_faceStart = patchScan[i];
			edge.m_faceVertexCount = indexCount;
			edge.m_edge = (j + indexCount - 1) % indexCount;
			v0 = v1;
			egdeArray.PushBack(edge);
		}
	}

	class CompareKey
	{
		public:
		CompareKey(void* const)
		{
		}

		ndInt32 Compare(const ndEdgeList& elementA, const ndEdgeList& elementB) const
		{
			ndInt32 indexA = elementA.m_key;
			ndInt32 indexB = elementB.m_key;
			if (indexA < indexB)
			{
				return -1;
			}
			else if (indexA > indexB)
			{
				return 1;
			}
			return 0;
		}
	};
	ndSort<ndEdgeList, CompareKey>(&egdeArray[0], egdeArray.GetCount(), nullptr);

	for (ndInt32 i = 0; i < egdeArray.GetCount() - 1; ++i)
	{
		const ndEdgeList& edge0 = egdeArray[i];
		const ndEdgeList& edge1 = egdeArray[i + 1];
		if (edge0.m_key == edge1.m_key)
		{
			ndInt32 originIndex = indexArray[edge0.m_faceStart + edge0.m_edge];
			ndInt32 normalIndex = indexArray[edge0.m_faceStart + edge0.m_faceVertexCount + 1];
			const ndVector normal(vertex[normalIndex]);
			const ndVector origin(vertex[originIndex]);

			ndFloat32 absDist = ndFloat32(0.0f);
			ndFloat32 maxDist = ndFloat32(0.0f);
			const ndInt32 faceVertexCount = edge1.m_faceVertexCount;
			for (ndInt32 j = 0; j < faceVertexCount; ++j)
			{
				ndInt32 vertexIndex = indexArray[edge1.m_faceStart + j];
				const ndVector p(vertex[vertexIndex]);
				ndFloat32 maxDist1 = normal.DotProduct(p - origin).GetScalar();
				ndFloat32 absDist1 = ndAbs(maxDist1);
				if (absDist1 > absDist)
				{
					maxDist = maxDist1;
					absDist = absDist1;
				}
			}
			if (maxDist < ndFloat32(1.0e-3f))
			{
				ndInt32 normalIndex0 = edge0.m_faceStart + edge0.m_faceVertexCount + edge0.m_edge + 2;
				ndInt32 normalIndex1 = edge1.m_faceStart + edge1.m_faceVertexCount + edge1.m_edge + 2;
				ndSwap(indexArray[normalIndex0], indexArray[normalIndex1]);
			}
			i++;
		}
	}
}

ndShapeStaticMesh::ndShapeStaticMesh(ndShapeID id)
	:ndShape(id)
{
	ndAssert(ndMemory::CheckMemory(this));
}

ndShapeStaticMesh::~ndShapeStaticMesh()
{
	ndAssert(ndMemory::CheckMemory(this));
}

ndFloat32 ndShapeStaticMesh::GetVolume() const
{
	return ndFloat32(0.0f);
}

ndFloat32 ndShapeStaticMesh::GetBoxMinRadius() const
{
	return ndFloat32(0.0f);
}

ndFloat32 ndShapeStaticMesh::GetBoxMaxRadius() const
{
	return ndFloat32(0.0f);
}

ndVector ndShapeStaticMesh::SupportVertex(const ndVector& dir) const
{
	const ndVector size(GetObbSize());
	const ndVector origin (GetObbOrigin());
	const ndVector p0(origin - size);
	const ndVector p1(origin + size);

	const ndVector mask(dir < ndVector::m_zero);
	const ndVector support(p1.Select(p0, mask));
	return support;
}

ndVector ndShapeStaticMesh::SupportVertexSpecial(const ndVector& dir, ndFloat32) const
{
	ndAssert(0);
	return SupportVertex(dir);
}

ndVector ndShapeStaticMesh::SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector&) const
{
	ndAssert(0);
	return point;
}

ndInt32 ndShapeStaticMesh::CalculatePlaneIntersection(const ndVector&, const ndVector&, ndVector* const) const
{
	return 0;
}

ndVector ndShapeStaticMesh::CalculateVolumeIntegral(const ndMatrix&, const ndVector&, const ndShapeInstance&) const
{
	return ndVector::m_zero;
}

ndShapeStaticMesh* ndShapeStaticMesh::GetAsShapeStaticMesh()
{
	return this;
}

void ndShapeStaticMesh::DebugShape(const ndMatrix&, ndShapeDebugNotify&) const
{
}

ndFloat32 ndShapeStaticMesh::RayCast(ndRayCastNotify&, const ndVector&, const ndVector&, ndFloat32, const ndBody* const, ndContactPoint&) const
{
	return ndFloat32(1.2f);
}

void ndShapeStaticMesh::GetCollidingFaces(ndPolygonMeshDesc* const) const
{
}

void ndShapeStaticMesh::CalculateAabb(const ndMatrix& matrix, ndVector &p0, ndVector &p1) const
{
	const ndVector origin(matrix.TransformVector(m_boxOrigin));
	const ndVector size(matrix.m_front.Abs().Scale(m_boxSize.m_x) + matrix.m_up.Abs().Scale(m_boxSize.m_y) + matrix.m_right.Abs().Scale(m_boxSize.m_z));
	p0 = (origin - size) & ndVector::m_triplexMask;
	p1 = (origin + size) & ndVector::m_triplexMask;
}

ndInt32 ndShapeStaticMesh::CalculatePlaneIntersection(const ndFloat32* const, const ndInt32* const, ndInt32, ndInt32, const ndPlane&, ndVector* const) const
{
	ndAssert(0);
	return 0;
	//ndInt32 count = 0;
	//ndInt32 j = index[indexCount - 1] * stride;
	//ndVector p0(&vertex[j]);
	//p0 = p0 & ndVector::m_triplexMask;
	//ndFloat32 side0 = localPlane.Evalue(p0);
	//for (ndInt32 i = 0; i < indexCount; ++i) 
	//{
	//	j = index[i] * stride;
	//	ndVector p1(&vertex[j]);
	//	p1 = p1 & ndVector::m_triplexMask;
	//	ndFloat32 side1 = localPlane.Evalue(p1);
	//
	//	if (side0 < ndFloat32(0.0f)) {
	//		if (side1 >= ndFloat32(0.0f)) {
	//			ndVector dp(p1 - p0);
	//			ndAssert(dp.m_w == ndFloat32(0.0f));
	//			ndFloat32 t = localPlane.DotProduct(dp).GetScalar();
	//			ndAssert(dgAbs(t) >= ndFloat32(0.0f));
	//			if (dgAbs(t) < ndFloat32(1.0e-8f)) {
	//				t = dgSign(t) * ndFloat32(1.0e-8f);
	//			}
	//			ndAssert(0);
	//			contactsOut[count] = p0 - dp.Scale(side0 / t);
	//			count++;
	//
	//		}
	//	}
	//	else if (side1 <= ndFloat32(0.0f)) {
	//		ndVector dp(p1 - p0);
	//		ndAssert(dp.m_w == ndFloat32(0.0f));
	//		ndFloat32 t = localPlane.DotProduct(dp).GetScalar();
	//		ndAssert(dgAbs(t) >= ndFloat32(0.0f));
	//		if (dgAbs(t) < ndFloat32(1.0e-8f)) {
	//			t = dgSign(t) * ndFloat32(1.0e-8f);
	//		}
	//		ndAssert(0);
	//		contactsOut[count] = p0 - dp.Scale(side0 / t);
	//		count++;
	//	}
	//
	//	side0 = side1;
	//	p0 = p1;
	//}
	//
	//return count;
}

