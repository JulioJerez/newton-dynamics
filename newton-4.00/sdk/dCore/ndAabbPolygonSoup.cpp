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
#include "ndTypes.h"
#include "ndHeap.h"
#include "ndList.h"
#include "ndSort.h"
#include "ndStack.h"
#include "ndMatrix.h"
#include "ndPolyhedra.h"
#include "ndAabbPolygonSoup.h"
#include "ndPolygonSoupBuilder.h"

#define ND_TOP_DOWN_BUILD
#define ND_STACK_DEPTH		512

D_MSV_NEWTON_CLASS_ALIGN_32
class ndAabbPolygonSoup::ndNodeBuilder: public ndAabbPolygonSoup::ndNode
{
	public:
	ndNodeBuilder (const ndVector& p0, const ndVector& p1)
		:ndNode()
		,m_left (nullptr)
		,m_right (nullptr)
		,m_parent (nullptr)
		,m_enumeration(-1)
		,m_faceIndex(0)
		,m_indexCount(0)
		,m_faceIndices(nullptr)
	{
		SetBox (p0, p1);
	}

	ndNodeBuilder (const ndVector* const vertexArray, ndInt32 faceIndex, ndInt32 indexCount, const ndInt32* const indexArray)
		:ndNode()
		,m_left (nullptr)
		,m_right (nullptr)
		,m_parent (nullptr)
		,m_enumeration(-1)
		,m_faceIndex(faceIndex)
		,m_indexCount(indexCount)
		,m_faceIndices(indexArray)
		,m_bhvLinked(0)
	{
		ndVector minP ( ndFloat32 (1.0e15f)); 
		ndVector maxP (-ndFloat32 (1.0e15f)); 
		for (ndInt32 i = 0; i < indexCount; ++i) 
		{
			ndInt32 index = indexArray[i];
			const ndVector& p (vertexArray[index]);
			minP = p.GetMin(minP); 
			maxP = p.GetMax(maxP); 
		}
		minP -= ndVector (ndFloat32 (1.0e-3f));
		maxP += ndVector (ndFloat32 (1.0e-3f));
		minP = minP & ndVector::m_triplexMask;
		maxP = maxP & ndVector::m_triplexMask;
		SetBox (minP, maxP);
	}

	ndNodeBuilder (ndNodeBuilder* const left, ndNodeBuilder* const right)
		:ndNode()
		,m_left(left)
		,m_right(right)
		,m_parent(nullptr)
		,m_enumeration(-1)
		,m_faceIndex(0)
		,m_indexCount(0)
		,m_faceIndices(nullptr)
		,m_bhvLinked(0)
	{
		m_left->m_parent = this;
		m_right->m_parent = this;

		ndVector p0 (left->m_p0.GetMin(right->m_p0));
		ndVector p1 (left->m_p1.GetMax(right->m_p1));
		SetBox(p0, p1);
	}

	void SetBox (const ndVector& p0, const ndVector& p1)
	{
		m_p0 = p0;
		m_p1 = p1;
	}

	void Init(ndNodeBuilder* const left, ndNodeBuilder* const right)
	{
		m_bhvLinked = 1;
		m_faceIndex = 0;
		m_indexCount = 0;
		m_enumeration = -1;
		m_parent = nullptr;
		m_faceIndices = nullptr;
		left->m_bhvLinked = 1;
		right->m_bhvLinked = 1;
		ndVector p0(left->m_p0.GetMin(right->m_p0));
		ndVector p1(left->m_p1.GetMax(right->m_p1));
		SetBox(p0, p1);
	}

	bool SanityCheck() const
	{
		if (m_parent)
		{
			return false;
		}

		ndAssert(m_left);
		ndAssert(m_right);
		ndFixSizeArray<const ndNodeBuilder*, 256> stack;
		stack.PushBack(m_left);
		stack.PushBack(m_right);
		while (stack.GetCount())
		{
			const ndNodeBuilder* const node = stack.Pop();
			if (!node->m_parent)
			{
				return false;
			}

			if (node->m_left)
			{
				stack.PushBack(node->m_left);
			}
			if (node->m_right)
			{
				stack.PushBack(node->m_right);
			}
		}
		return true;
	}

#if defined(_DEBUG)
	ndInt32 m_debugIndex;
#endif

	ndVector m_p0;
	ndVector m_p1;
	ndNodeBuilder* m_left;
	ndNodeBuilder* m_right;
	ndNodeBuilder* m_parent;

	ndInt32 m_enumeration;
	ndInt32 m_faceIndex;
	ndInt32 m_indexCount;
	const ndInt32* m_faceIndices;
	ndUnsigned8 m_bhvLinked;
} D_GCC_NEWTON_CLASS_ALIGN_32;

ndAabbPolygonSoup::ndAabbPolygonSoup ()
	:ndPolygonSoupDatabase()
	,m_aabb(nullptr)
	,m_indices(nullptr)
	,m_nodesCount(0)
	,m_indexCount(0)
{
}

ndAabbPolygonSoup::ndAabbPolygonSoup (const ndArray<ndVector>& points, const ndArray<ndInt32>& indices, const ndArray<ndNode>& nodes)
	:ndPolygonSoupDatabase()
	,m_aabb(nullptr)
	,m_indices(nullptr)
	,m_nodesCount(0)
	,m_indexCount(0)
{
	m_vertexCount = ndInt32 (points.GetCount());
	m_localVertex = new ndVector[size_t(m_vertexCount)];
	ndMemCpy(m_localVertex, &points[0], m_vertexCount);

	m_indexCount = ndInt32 (indices.GetCount());
	m_indices = new ndInt32[size_t(m_indexCount)];
	ndMemCpy(m_indices, &indices[0], m_indexCount);

	m_nodesCount = ndInt32(nodes.GetCount());
	m_aabb = new ndNode[size_t(m_nodesCount)];
	ndMemCpy(m_aabb, &nodes[0], m_nodesCount);
}

ndAabbPolygonSoup::~ndAabbPolygonSoup ()
{
	if (m_aabb) 
	{
		ndMemory::Free(m_aabb);
		ndMemory::Free(m_indices);
	}
}

ndFloat32 ndAabbPolygonSoup::CalculateFaceMaxDiagonal (const ndVector* const vertex, ndInt32 indexCount, const ndInt32* const indexArray) const
{
	ndFloat32 maxSize = ndFloat32 (0.0f);
	ndInt32 index = indexArray[indexCount - 1];
	ndVector p0 (vertex[index]);
	for (ndInt32 i = 0; i < indexCount; ++i) 
	{
		ndInt32 index1 = indexArray[i];
		ndVector p1 (vertex[index1]);

		ndVector dir (p1 - p0);
		ndAssert (dir.m_w == ndFloat32 (0.0f));
		dir = dir.Normalize();

		ndFloat32 maxVal = ndFloat32 (-1.0e10f);
		ndFloat32 minVal = ndFloat32 ( 1.0e10f);
		for (ndInt32 j = 0; j < indexCount; ++j) 
		{
			ndInt32 index2 = indexArray[j];
			ndVector q (vertex[index2]);
			ndFloat32 val = dir.DotProduct(q).GetScalar();
			minVal = ndMin(minVal, val);
			maxVal = ndMax(maxVal, val);
		}

		ndFloat32 size = maxVal - minVal;
		maxSize = ndMax(maxSize, size);
		p0 = p1;
	}

	return maxSize;
}

void ndAabbPolygonSoup::GetAABB (ndVector& p0, ndVector& p1) const
{
	if (m_aabb) 
	{ 
		GetNodeAabb (m_aabb, p0, p1);
	} 
	else 
	{
		p0 = ndVector::m_zero;
		p1 = ndVector::m_zero;
	}
}

void ndAabbPolygonSoup::CalculateAdjacent ()
{
	ndVector p0;
	ndVector p1;
	GetAABB (p0, p1);
	ndFastAabb box (p0, p1);

	ndPolyhedra adjacentMesh;
	adjacentMesh.BeginFace();
	ForAllSectors(box, ndVector::m_zero, ndFloat32(1.0f), CalculateAllFaceEdgeNormals, &adjacentMesh);
	adjacentMesh.EndFace();

	ndInt32 mark = adjacentMesh.IncLRU();
	ndPolyhedra::Iterator iter(adjacentMesh);
	const ndVector* const vertexArray = const_cast<ndVector*>(GetLocalVertexPool());

	for (iter.Begin(); iter; iter++)
	{
		ndEdge* const edge = &(*iter);
		if ((edge->m_mark != mark) && (edge->m_incidentFace >= 0) && (edge->m_twin->m_incidentFace >= 0))
		{
			ndInt32 indexCount0 = 0;
			ndInt32 indexCount1 = 0;
			ndInt32 offsetIndex0 = -1;
			ndInt32 offsetIndex1 = -1;
			ndInt32* const indexArray0 = reinterpret_cast<ndInt32*>(edge->m_userData);
			ndInt32* const indexArray1 = reinterpret_cast<ndInt32*>(edge->m_twin->m_userData);
			ndEdge* ptr = edge;
			do
			{
				indexCount0++;
				ptr = ptr->m_next;
			} while (ptr != edge);

			ptr = edge->m_twin;
			do
			{
				indexCount1++;
				ptr = ptr->m_next;
			} while (ptr != edge->m_twin);

			const ndVector n0(vertexArray[indexArray0[indexCount0 + 1]]);
			const ndVector q0(vertexArray[indexArray0[0]]);
			ndAssert(n0.m_w == ndFloat32(0.0f));
			ndAssert(q0.m_w == ndFloat32(0.0f));

			const ndVector n1(vertexArray[indexArray1[indexCount1 + 1]]);
			const ndVector q1(vertexArray[indexArray1[0]]);
			ndAssert(n1.m_w == ndFloat32(0.0f));
			ndAssert(q1.m_w == ndFloat32(0.0f));

			const ndPlane plane0(n0, -n0.DotProduct(q0).GetScalar());
			const ndPlane plane1(n1, -n1.DotProduct(q1).GetScalar());

			ndFloat32 maxDist0 = ndFloat32(-1.0f);
			for (ndInt32 i = 0; i < indexCount1; ++i)
			{
				if (edge->m_twin->m_incidentVertex == indexArray1[i])
				{
					offsetIndex1 = i;
				}
				ndVector point(&vertexArray[indexArray1[i]].m_x);
				ndFloat32 dist(plane0.Evalue(point & ndVector::m_triplexMask));
				maxDist0 = ndMax(maxDist0, dist);
			}

			ndFloat32 maxDist1 = ndFloat32(-1.0f);
			for (ndInt32 i = 0; i < indexCount0; ++i)
			{
				if (edge->m_incidentVertex == indexArray0[i])
				{
					offsetIndex0 = i;
				}
				ndVector point(&vertexArray[indexArray0[i]].m_x);
				ndFloat32 dist(plane1.Evalue(point & ndVector::m_triplexMask));
				maxDist1 = ndMax(maxDist1, dist);
			}

			bool edgeIsConvex = (maxDist0 <= ndFloat32(1.0e-3f));
			edgeIsConvex = edgeIsConvex && (maxDist1 <= ndFloat32(1.0e-3f));
			edgeIsConvex = edgeIsConvex || (n0.DotProduct(n1).GetScalar() > ndFloat32(0.9991f));

			//hacks for testing adjacency
			//edgeIsConvex = edgeIsConvex || (n0.DotProduct(n1).GetScalar() > ndFloat32(0.5f));
			//edgeIsConvex = true;
			if (edgeIsConvex)
			{
				ndAssert(offsetIndex0 >= 0);
				ndAssert(offsetIndex1 >= 0);
				indexArray0[indexCount0 + 2 + offsetIndex0] = indexArray1[indexCount1 + 1];
				indexArray1[indexCount1 + 2 + offsetIndex1] = indexArray0[indexCount0 + 1];
			}
		}
		edge->m_mark = mark;
		edge->m_twin->m_mark = mark;
	}

	ndStack<ndVector> pool ((m_indexCount / 2) - 1);
	ndInt32 normalCount = 0;
	for (ndInt32 i = 0; i < m_nodesCount; ++i) 
	{
		const ndNode* const node = &m_aabb[i];
		if (node->m_leftInfo.IsLeaf())
		{
			ndInt32 vCount = ndInt32 (node->m_leftInfo.GetCount());
			if (vCount) 
			{
				ndInt32 index = ndInt32 (node->m_leftInfo.GetIndex());
				ndInt32* const face = &m_indices[index];
	
				ndInt32 j0 = 2 * (vCount + 1) - 1;
				ndVector normal (vertexArray[face[vCount + 1]]);
				ndAssert(normal.m_w == ndFloat32(0.0f));
				ndAssert (ndAbs (normal.DotProduct(normal).GetScalar() - ndFloat32 (1.0f)) < ndFloat32 (1.0e-6f));
				ndVector q0 (vertexArray[face[vCount - 1]]);
				ndAssert(q0.m_w == ndFloat32(0.0f));
				for (ndInt32 j = 0; j < vCount; ++j) 
				{
					ndInt32 j1 = vCount + 2 + j;
					const ndVector q1 (vertexArray[face[j]]);
					ndAssert(q1.m_w == ndFloat32(0.0f));
					if (face[j0] & D_CONCAVE_EDGE_MASK)
					{
						const ndVector e (q1 - q0);
						const ndVector n (e.CrossProduct(normal).Normalize());
						ndAssert (ndAbs (n.DotProduct(n).GetScalar() - ndFloat32 (1.0f)) < ndFloat32 (1.0e-6f));
						pool[normalCount].m_x = n.m_x;
						pool[normalCount].m_y = n.m_y;
						pool[normalCount].m_z = n.m_z;
						pool[normalCount].m_w = ndFloat32 (0.0f);
						face[j0] = normalCount | ndInt32(D_CONCAVE_EDGE_MASK);
						normalCount ++;
					}
					q0 = q1;
					j0 = j1;
				}
			}
		}
	
		if (node->m_rightInfo.IsLeaf())
		{
			ndInt32 vCount = ndInt32 (node->m_rightInfo.GetCount());
			if (vCount) 
			{
				ndInt32 index = ndInt32 (node->m_rightInfo.GetIndex());
				ndInt32* const face = &m_indices[index];
	
				ndInt32 j0 = 2 * (vCount + 1) - 1;
				const ndVector normal (vertexArray[face[vCount + 1]]);
				ndAssert (ndAbs (normal.DotProduct(normal).GetScalar() - ndFloat32 (1.0f)) < ndFloat32 (1.0e-6f));
				ndVector q0 (vertexArray[face[vCount - 1]]);
				for (ndInt32 j = 0; j < vCount; ++j) 
				{
					ndInt32 j1 = vCount + 2 + j;
					const ndVector q1 (vertexArray[face[j]]);
					if (face[j0] & D_CONCAVE_EDGE_MASK)
					{
						const ndVector e (q1 - q0);
						const ndVector n (e.CrossProduct(normal).Normalize());
						ndAssert (ndAbs (n.DotProduct(n).GetScalar() - ndFloat32 (1.0f)) < ndFloat32 (1.0e-6f));
						pool[normalCount].m_x = n.m_x;
						pool[normalCount].m_y = n.m_y;
						pool[normalCount].m_z = n.m_z;
						pool[normalCount].m_w = ndFloat32(0.0f);
						face[j0] = normalCount | ndInt32(D_CONCAVE_EDGE_MASK);
						normalCount ++;
					}
					q0 = q1;
					j0 = j1;
				}
			}
		}
	}

	if (normalCount) 
	{
		ndStack<ndInt32> indexArray (normalCount);
		ndInt32 newNormalCount = ndVertexListToIndexList (&pool[0].m_x, sizeof (ndVector), 3, normalCount, &indexArray[0], ndFloat64 (1.0e-6f));
	
		ndInt32 oldCount = GetVertexCount();
		ndVector* const vertexArray1 = reinterpret_cast<ndVector*>(ndMemory::Malloc (sizeof (ndVector) * (oldCount + newNormalCount)));
		ndMemCpy(vertexArray1, const_cast<ndVector*>(GetLocalVertexPool()), oldCount);
		ndMemCpy(&vertexArray1[oldCount], &pool[0], newNormalCount);

		ndMemory::Free(GetLocalVertexPool());
	
		m_localVertex = vertexArray1;
		m_vertexCount = oldCount + newNormalCount;
	
		for (ndInt32 i = 0; i < m_nodesCount; ++i) 
		{
			const ndNode* const node = &m_aabb[i];
			if (node->m_leftInfo.IsLeaf())
			{
				ndInt32 vCount = ndInt32 (node->m_leftInfo.GetCount());
				ndInt32 index = ndInt32 (node->m_leftInfo.GetIndex());
				ndInt32* const face = &m_indices[index];
				for (ndInt32 j = 0; j < vCount; ++j) 
				{
					ndInt32 edgeIndexNormal = face[vCount + 2 + j];
					if (edgeIndexNormal & D_CONCAVE_EDGE_MASK)
					{
						ndInt32 k = edgeIndexNormal & ndInt32(~D_CONCAVE_EDGE_MASK);
						face[vCount + 2 + j] = (indexArray[k] + oldCount) | ndInt32(D_CONCAVE_EDGE_MASK);
					}
					#ifdef _DEBUG	
						//ndVector normal (&vertexArray1[face[vCount + 2 + j]].m_x);
						ndVector normal (vertexArray1[face[vCount + 2 + j] & (~D_CONCAVE_EDGE_MASK)]);
						ndAssert (ndAbs (normal.DotProduct(normal).GetScalar() - ndFloat32 (1.0f)) < ndFloat32 (1.0e-6f));
					#endif
				}
			}
	
			if (node->m_rightInfo.IsLeaf())
			{
				ndInt32 vCount = ndInt32 (node->m_rightInfo.GetCount());
				ndInt32 index = ndInt32 (node->m_rightInfo.GetIndex());
				ndInt32* const face = &m_indices[index];
				for (ndInt32 j = 0; j < vCount; ++j) 
				{
					ndInt32 edgeIndexNormal = face[vCount + 2 + j];
					if (edgeIndexNormal & ndInt32(D_CONCAVE_EDGE_MASK))
					{
						ndInt32 k = edgeIndexNormal & ndInt32(~D_CONCAVE_EDGE_MASK);
						face[vCount + 2 + j] = (indexArray[k] + oldCount) | ndInt32(D_CONCAVE_EDGE_MASK);
					}
	
					#ifdef _DEBUG	
						ndVector normal(vertexArray1[face[vCount + 2 + j] & ndInt32(~D_CONCAVE_EDGE_MASK)]);
						ndAssert (ndAbs (normal.DotProduct(normal).GetScalar() - ndFloat32 (1.0f)) < ndFloat32 (1.0e-6f));
					#endif
				}
			}
		}
	}
}

ndIntersectStatus ndAabbPolygonSoup::CalculateAllFaceEdgeNormals(void* const context, const ndVector* const, const ndInt32* const indexArray, ndInt32 indexCount, ndFloat32)
{
	ndInt32 face[256];
	ndInt64 data[256];
	ndPolyhedra& adjacency = *reinterpret_cast<ndPolyhedra*>(context);

	ndIntPtr userData;
	userData.m_ptr = static_cast<void*>(const_cast<ndInt32 *>(indexArray));
	for (ndInt32 i = 0; i < indexCount; ++i)
	{
		face[i] = indexArray[i];
		data[i] = userData.m_int;
	}
	adjacency.AddFace(indexCount, face, data);
	return m_continueSearh;
}

void ndAabbPolygonSoup::Serialize (const char* const path) const
{
	FILE* const file = fopen(path, "wb");
	if (file)
	{
		fwrite(&m_vertexCount, sizeof(ndInt32), 1, file);
		fwrite(&m_indexCount, sizeof(ndInt32), 1, file);
		fwrite(&m_nodesCount, sizeof(ndInt32), 1, file);
		if (m_aabb)
		{
			fwrite(m_localVertex, sizeof(ndVector) * m_vertexCount, 1, file);
			fwrite(m_indices, sizeof(ndInt32) * m_indexCount, 1, file);
			fwrite(m_aabb, sizeof(ndNode) * m_nodesCount, 1, file);
		}
		fclose(file);
	}
}

void ndAabbPolygonSoup::Deserialize (const char* const path)
{
	FILE* const file = fopen(path, "rb");
	if (file)
	{
		fread(&m_vertexCount, sizeof(ndInt32), 1, file);
		fread(&m_indexCount, sizeof(ndInt32), 1, file);
		fread(&m_nodesCount, sizeof(ndInt32), 1, file);

		if (m_vertexCount) 
		{
			m_localVertex = reinterpret_cast<ndVector*>(ndMemory::Malloc(sizeof(ndVector) * m_vertexCount));
			m_indices = reinterpret_cast<ndInt32*>(ndMemory::Malloc(sizeof(ndInt32) * m_indexCount));
			m_aabb = reinterpret_cast<ndNode*>(ndMemory::Malloc(sizeof(ndNode) * m_nodesCount));

			fread(m_localVertex, sizeof(ndVector) * m_vertexCount, 1, file);
			fread(m_indices, sizeof(ndInt32) * m_indexCount, 1, file);
			fread(m_aabb, sizeof(ndNode) * m_nodesCount, 1, file);
		}
		else 
		{
			m_localVertex = nullptr;
			m_indices = nullptr;
			m_aabb = nullptr;
		}

		fclose(file);
	}
}

ndVector ndAabbPolygonSoup::ForAllSectorsSupportVertex (const ndVector& dir) const
{
	ndVector supportVertex (ndVector::m_zero);
	if (m_aabb) 
	{
		ndFixSizeArray<ndNode*, ND_STACK_DEPTH> stackPool;
		ndFixSizeArray<ndFloat32, ND_STACK_DEPTH> aabbProjection;

		stackPool.PushBack(m_aabb);
		aabbProjection.PushBack(ndFloat32 (1.0e10f));
		const ndVector* const boxArray = m_localVertex;

		ndFloat32 maxProj = ndFloat32 (-1.0e20f); 
		ndInt32 ix = (dir[0] > ndFloat32 (0.0f)) ? 1 : 0;
		ndInt32 iy = (dir[1] > ndFloat32 (0.0f)) ? 1 : 0;
		ndInt32 iz = (dir[2] > ndFloat32 (0.0f)) ? 1 : 0;

		while (stackPool.GetCount())
		{
			const ndNode* const me = stackPool.Pop();
			ndFloat32 boxSupportValue = boxSupportValue = aabbProjection.Pop();
			if (boxSupportValue > maxProj) 
			{
				ndFloat32 backSupportDist = ndFloat32 (0.0f);
				ndFloat32 frontSupportDist = ndFloat32 (0.0f);
				if (me->m_leftInfo.IsLeaf())
				{
					backSupportDist = ndFloat32 (-1.0e20f);
					const ndInt32 index = ndInt32 (me->m_leftInfo.GetIndex());
					const ndInt32 vCount = ndInt32 (me->m_leftInfo.GetCount());
					ndVector vertex (ndVector::m_zero);
					for (ndInt32 j = 0; j < vCount; ++j) 
					{
						ndInt32 i0 = m_indices[index + j];
						ndVector p (&boxArray[i0].m_x);
						p = p & ndVector::m_triplexMask;
						ndFloat32 dist = p.DotProduct(dir).GetScalar();
						if (dist > backSupportDist) 
						{
							backSupportDist = dist;
							vertex = p;
						}
					}

					if (backSupportDist > maxProj) 
					{
						maxProj = backSupportDist;
						supportVertex = vertex; 
					}
				} 
				else 
				{
					ndVector box[2];
					const ndNode* const node = me->m_leftInfo.GetNode(m_aabb);
					box[0].m_x = boxArray[node->m_indexBox0].m_x;
					box[0].m_y = boxArray[node->m_indexBox0].m_y;
					box[0].m_z = boxArray[node->m_indexBox0].m_z;
					box[1].m_x = boxArray[node->m_indexBox1].m_x;
					box[1].m_y = boxArray[node->m_indexBox1].m_y;
					box[1].m_z = boxArray[node->m_indexBox1].m_z;

					ndVector supportPoint (box[ix].m_x, box[iy].m_y, box[iz].m_z, ndFloat32 (0.0));
					backSupportDist = supportPoint.DotProduct(dir).GetScalar();
				}

				if (me->m_rightInfo.IsLeaf())
				{
					frontSupportDist = ndFloat32 (-1.0e20f);
					const ndInt32 index = ndInt32 (me->m_rightInfo.GetIndex());
					const ndInt32 vCount = ndInt32 (me->m_rightInfo.GetCount());
					ndVector vertex(ndVector::m_zero);
					for (ndInt32 j = 0; j < vCount; ++j) 
					{
						ndInt32 i0 = m_indices[index + j];
						ndVector p (&boxArray[i0].m_x);
						p = p & ndVector::m_triplexMask;
						ndFloat32 dist = p.DotProduct(dir).GetScalar();
						if (dist > frontSupportDist) 
						{
							frontSupportDist = dist;
							vertex = p;
						}
					}
					if (frontSupportDist > maxProj) 
					{
						maxProj = frontSupportDist;
						supportVertex = vertex; 
					}
				} 
				else 
				{
					ndVector box[2];
					const ndNode* const node = me->m_rightInfo.GetNode(m_aabb);
					box[0].m_x = boxArray[node->m_indexBox0].m_x;
					box[0].m_y = boxArray[node->m_indexBox0].m_y;
					box[0].m_z = boxArray[node->m_indexBox0].m_z;
					box[1].m_x = boxArray[node->m_indexBox1].m_x;
					box[1].m_y = boxArray[node->m_indexBox1].m_y;
					box[1].m_z = boxArray[node->m_indexBox1].m_z;

					ndVector supportPoint (box[ix].m_x, box[iy].m_y, box[iz].m_z, ndFloat32 (0.0f));
					frontSupportDist = supportPoint.DotProduct(dir).GetScalar();
				}

				if (frontSupportDist >= backSupportDist) 
				{
					if (!me->m_leftInfo.IsLeaf())
					{
						aabbProjection.PushBack(backSupportDist);
						stackPool.PushBack(me->m_leftInfo.GetNode(m_aabb));
					}

					if (!me->m_rightInfo.IsLeaf())
					{
						aabbProjection.PushBack(frontSupportDist);
						stackPool.PushBack(me->m_rightInfo.GetNode(m_aabb));
					}

				} 
				else 
				{
					if (!me->m_rightInfo.IsLeaf())
					{
						aabbProjection.PushBack(frontSupportDist);
						stackPool.PushBack(me->m_rightInfo.GetNode(m_aabb));
					}

					if (!me->m_leftInfo.IsLeaf())
					{
						aabbProjection.PushBack(backSupportDist);
						stackPool.PushBack(me->m_leftInfo.GetNode(m_aabb));
					}
				}
			}
		}
	}
	return supportVertex;
}

void ndAabbPolygonSoup::ForAllSectorsRayHit (const ndFastRay& raySrc, ndFloat32 maxParam, ndRayIntersectCallback callback, void* const context) const
{
	ndFastRay ray (raySrc);
	ndFixSizeArray<ndFloat32, ND_STACK_DEPTH> distance;
	ndFixSizeArray<const ndNode*, ND_STACK_DEPTH> stackPool;

	const ndVector* const vertexArray = m_localVertex;
	
	stackPool.PushBack(m_aabb);
	distance.PushBack(m_aabb->RayDistance(ray, vertexArray));
	while (stackPool.GetCount()) 
	{
		ndFloat32 dist = distance.Pop();
		const ndNode* const me = stackPool.Pop();
		if (dist > maxParam) 
		{
			break;
		} 
		else 
		{
			if (me->m_leftInfo.IsLeaf())
			{
				ndInt32 vCount = ndInt32 (me->m_leftInfo.GetCount());
				if (vCount > 0) 
				{
					ndInt32 index = ndInt32 (me->m_leftInfo.GetIndex());
					ndFloat32 param = callback(context, vertexArray, &m_indices[index], vCount);
					ndAssert (param >= ndFloat32 (0.0f));
					if (param < maxParam) 
					{
						maxParam = param;
						if (maxParam == ndFloat32 (0.0f)) 
						{
							break;
						}
					}
				}
			} 
			else 
			{
				const ndNode* const node = me->m_leftInfo.GetNode(m_aabb);
				ndFloat32 dist1 = node->RayDistance(ray, vertexArray);
				if (dist1 < maxParam) 
				{
					ndInt32 j = stackPool.GetCount();
					stackPool.PushBack(node);
					distance.PushBack(dist1);
					for ( ; j && (dist1 > distance[j - 1]); j --) 
					{
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					stackPool[j] = node;
					distance[j] = dist1;
				}
			}
	
			if (me->m_rightInfo.IsLeaf())
			{
				ndInt32 vCount = ndInt32 (me->m_rightInfo.GetCount());
				if (vCount > 0) 
				{
					ndInt32 index = ndInt32 (me->m_rightInfo.GetIndex());
					ndFloat32 param = callback(context, vertexArray, &m_indices[index], vCount);
					ndAssert (param >= ndFloat32 (0.0f));
					if (param < maxParam) 
					{
						maxParam = param;
						if (maxParam == ndFloat32 (0.0f)) 
						{
							break;
						}
					}
				}
			} 
			else 
			{
				const ndNode* const node = me->m_rightInfo.GetNode(m_aabb);
				ndFloat32 dist1 = node->RayDistance(ray, vertexArray);
				if (dist1 < maxParam) 
				{
					ndInt32 j = stackPool.GetCount();
					stackPool.PushBack(node);
					distance.PushBack(dist1);
					for ( ; j && (dist1 > distance[j - 1]); j --) 
					{
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					stackPool[j] = node;
					distance[j] = dist1;
				}
			}
		}
	}
}

void ndAabbPolygonSoup::ForAllSectors (const ndFastAabb& obbAabbInfo, const ndVector& boxDistanceTravel, ndFloat32, ndAaabbIntersectCallback callback, void* const context) const
{
	ndAssert (ndAbs(ndAbs(obbAabbInfo[0][0]) - obbAabbInfo.m_absDir[0][0]) < ndFloat32 (1.0e-4f));
	ndAssert (ndAbs(ndAbs(obbAabbInfo[1][1]) - obbAabbInfo.m_absDir[1][1]) < ndFloat32 (1.0e-4f));
	ndAssert (ndAbs(ndAbs(obbAabbInfo[2][2]) - obbAabbInfo.m_absDir[2][2]) < ndFloat32 (1.0e-4f));

	ndAssert (ndAbs(ndAbs(obbAabbInfo[0][1]) - obbAabbInfo.m_absDir[1][0]) < ndFloat32 (1.0e-4f));
	ndAssert (ndAbs(ndAbs(obbAabbInfo[0][2]) - obbAabbInfo.m_absDir[2][0]) < ndFloat32 (1.0e-4f));
	ndAssert (ndAbs(ndAbs(obbAabbInfo[1][2]) - obbAabbInfo.m_absDir[2][1]) < ndFloat32 (1.0e-4f));

	if (m_aabb) 
	{
		ndFixSizeArray<ndFloat32, ND_STACK_DEPTH> distance;
		ndFixSizeArray<const ndNode*, ND_STACK_DEPTH> stackPool;
		const ndVector* const vertexArray = m_localVertex;

		ndAssert (boxDistanceTravel.m_w == ndFloat32 (0.0f));
		if (boxDistanceTravel.DotProduct(boxDistanceTravel).GetScalar() < ndFloat32 (1.0e-8f)) 
		{
			stackPool.PushBack(m_aabb);
			distance.PushBack(m_aabb->BoxPenetration(obbAabbInfo, vertexArray));
			if (distance[0] <= ndFloat32(0.0f)) 
			{
				obbAabbInfo.m_separationDistance = ndMin(obbAabbInfo.m_separationDistance[0], -distance[0]);
			}
			while (stackPool.GetCount())
			{
				ndFloat32 dist = distance.Pop();
				const ndNode* const me = stackPool.Pop();
				if (dist > ndFloat32 (0.0f)) 
				{
					if (me->m_leftInfo.IsLeaf())
					{
						ndInt32 index = ndInt32 (me->m_leftInfo.GetIndex());
						ndInt32 vCount = ndInt32 (me->m_leftInfo.GetCount());
						if (vCount > 0) 
						{
							const ndInt32* const indices = &m_indices[index];
							ndInt32 normalIndex = indices[vCount + 1];
							const ndVector& faceNormal = vertexArray[normalIndex];
							ndFloat32 dist1 = obbAabbInfo.PolygonBoxDistance (faceNormal, vCount, indices, vertexArray);
							if (dist1 > ndFloat32 (0.0f)) 
							{
								obbAabbInfo.m_separationDistance = ndFloat32(0.0f);
								ndAssert (vCount >= 3);
								if (callback(context, vertexArray, indices, vCount, dist1) == m_stopSearch) 
								{
									return;
								}
							} 
							else 
							{
								obbAabbInfo.m_separationDistance = ndMin(obbAabbInfo.m_separationDistance[0], -dist1);
							}
						}
					} 
					else 
					{
						const ndNode* const node = me->m_leftInfo.GetNode(m_aabb);
						ndFloat32 dist1 = node->BoxPenetration(obbAabbInfo, vertexArray);
						if (dist1 > ndFloat32 (0.0f)) 
						{
							ndInt32 j = stackPool.GetCount();
							stackPool.PushBack(nullptr);
							distance.PushBack(ndFloat32 (0.0f));
							for ( ; j && (dist1 > distance[j - 1]); j --) 
							{
								stackPool[j] = stackPool[j - 1];
								distance[j] = distance[j - 1];
							}
							stackPool[j] = node;
							distance[j] = dist1;
						} 
						else 
						{
							obbAabbInfo.m_separationDistance = ndMin(obbAabbInfo.m_separationDistance[0], -dist1);
						}
					}

					if (me->m_rightInfo.IsLeaf())
					{
						ndInt32 index = ndInt32 (me->m_rightInfo.GetIndex());
						ndInt32 vCount = ndInt32 (me->m_rightInfo.GetCount());
						if (vCount > 0) 
						{
							const ndInt32* const indices = &m_indices[index];
							ndInt32 normalIndex = indices[vCount + 1];
							ndVector faceNormal (&vertexArray[normalIndex].m_x);
							ndFloat32 dist1 = obbAabbInfo.PolygonBoxDistance (faceNormal, vCount, indices, vertexArray);
							if (dist1 > ndFloat32 (0.0f)) 
							{
								ndAssert (vCount >= 3);
								obbAabbInfo.m_separationDistance = ndFloat32(0.0f);
								if (callback(context, vertexArray, indices, vCount, dist1) == m_stopSearch) 
								{
									return;
								}
							} 
							else 
							{
								obbAabbInfo.m_separationDistance = ndMin(obbAabbInfo.m_separationDistance[0], -dist1);
							}
						}
					} 
					else 
					{
						const ndNode* const node = me->m_rightInfo.GetNode(m_aabb);
						ndFloat32 dist1 = node->BoxPenetration(obbAabbInfo, vertexArray);
						if (dist1 > ndFloat32 (0.0f)) 
						{
							ndInt32 j = stackPool.GetCount();
							stackPool.PushBack(nullptr);
							distance.PushBack(ndFloat32(0.0f));
							for ( ; j && (dist1 > distance[j - 1]); j --) 
							{
								stackPool[j] = stackPool[j - 1];
								distance[j] = distance[j - 1];
							}
							stackPool[j] = node;
							distance[j] = dist1;
						} 
						else 
						{
							obbAabbInfo.m_separationDistance = ndMin(obbAabbInfo.m_separationDistance[0], -dist1);
						}
					}
				}
			}
		} 
		else 
		{
			ndFastRay ray (ndVector::m_zero, boxDistanceTravel);
			ndFastRay obbRay (ndVector::m_zero, obbAabbInfo.UnrotateVector(boxDistanceTravel));
			stackPool.PushBack(m_aabb);
			distance.PushBack(m_aabb->BoxIntersect(ray, obbRay, obbAabbInfo, vertexArray));
		
			while (stackPool.GetCount())
			{
				const ndFloat32 dist = distance.Pop();
				const ndNode* const me = stackPool.Pop();
				if (dist < ndFloat32 (1.0f)) 
				{
					if (me->m_leftInfo.IsLeaf())
					{
						ndInt32 index = ndInt32 (me->m_leftInfo.GetIndex());
						ndInt32 vCount = ndInt32 (me->m_leftInfo.GetCount());
						if (vCount > 0) 
						{
							const ndInt32* const indices = &m_indices[index];
							ndInt32 normalIndex = indices[vCount + 1];
							const ndVector& faceNormal = vertexArray[normalIndex];
							ndFloat32 hitDistance = obbAabbInfo.PolygonBoxRayDistance (faceNormal, vCount, indices, vertexArray, ray);
							if (hitDistance < ndFloat32 (1.0f)) 
							{
								ndAssert (vCount >= 3);
								if (callback(context, vertexArray, indices, vCount, hitDistance) == m_stopSearch) 
								{
									return;
								}
							}
						}
					} 
					else 
					{
						const ndNode* const node = me->m_leftInfo.GetNode(m_aabb);
						ndFloat32 dist1 = node->BoxIntersect (ray, obbRay, obbAabbInfo, vertexArray);
						if (dist1 < ndFloat32 (1.0f)) 
						{
							ndInt32 j = stackPool.GetCount();
							stackPool.PushBack(node);
							distance.PushBack(dist1);
							for ( ; j && (dist1 > distance[j - 1]); j --) 
							{
								stackPool[j] = stackPool[j - 1];
								distance[j] = distance[j - 1];
							}
							stackPool[j] = node;
							distance[j] = dist1;
						}
					}

					if (me->m_rightInfo.IsLeaf())
					{
						ndInt32 index = ndInt32 (me->m_rightInfo.GetIndex());
						ndInt32 vCount = ndInt32 (me->m_rightInfo.GetCount());
						if (vCount > 0) 
						{
							const ndInt32* const indices = &m_indices[index];
							ndInt32 normalIndex = indices[vCount + 1];
							const ndVector faceNormal = vertexArray[normalIndex];
							ndFloat32 hitDistance = obbAabbInfo.PolygonBoxRayDistance (faceNormal, vCount, indices, vertexArray, ray);
							if (hitDistance < ndFloat32 (1.0f)) 
							{
								ndAssert (vCount >= 3);
								if (callback(context, vertexArray, indices, vCount, hitDistance) == m_stopSearch) {
									return;
								}
							}
						}
					} 
					else 
					{
						const ndNode* const node = me->m_rightInfo.GetNode(m_aabb);
						ndFloat32 dist1 = node->BoxIntersect (ray, obbRay, obbAabbInfo, vertexArray);
						if (dist1 < ndFloat32 (1.0f)) 
						{
							ndInt32 j = stackPool.GetCount();
							stackPool.PushBack(node);
							distance.PushBack(dist1);
							for ( ; j && (dist1 > distance[j - 1]); j --) 
							{
								stackPool[j] = stackPool[j - 1];
								distance[j] = distance[j - 1];
							}
							stackPool[j] = node;
							distance[j] = dist1;
						}
					}
				}
			}
		}
	}
}

void ndAabbPolygonSoup::ForThisSector(const ndAabbPolygonSoup::ndNode* const node, const ndFastAabb& obbAabbInfo, const ndVector& boxDistanceTravel, ndFloat32, ndAaabbIntersectCallback callback, void* const context) const
{
	ndAssert(ndAbs(ndAbs(obbAabbInfo[0][0]) - obbAabbInfo.m_absDir[0][0]) < ndFloat32(1.0e-4f));
	ndAssert(ndAbs(ndAbs(obbAabbInfo[1][1]) - obbAabbInfo.m_absDir[1][1]) < ndFloat32(1.0e-4f));
	ndAssert(ndAbs(ndAbs(obbAabbInfo[2][2]) - obbAabbInfo.m_absDir[2][2]) < ndFloat32(1.0e-4f));

	ndAssert(ndAbs(ndAbs(obbAabbInfo[0][1]) - obbAabbInfo.m_absDir[1][0]) < ndFloat32(1.0e-4f));
	ndAssert(ndAbs(ndAbs(obbAabbInfo[0][2]) - obbAabbInfo.m_absDir[2][0]) < ndFloat32(1.0e-4f));
	ndAssert(ndAbs(ndAbs(obbAabbInfo[1][2]) - obbAabbInfo.m_absDir[2][1]) < ndFloat32(1.0e-4f));

	if (m_aabb)
	{
		const ndVector* const vertexArray = m_localVertex;
		
		ndAssert(boxDistanceTravel.m_w == ndFloat32(0.0f));
		if (boxDistanceTravel.DotProduct(boxDistanceTravel).GetScalar() < ndFloat32(1.0e-8f))
		{
			ndFloat32 dist = node->BoxPenetration(obbAabbInfo, vertexArray);
			if (dist <= ndFloat32(0.0f))
			{
				obbAabbInfo.m_separationDistance = ndMin(obbAabbInfo.m_separationDistance[0], dist);
			}
			if (dist > ndFloat32(0.0f))
			{
				if (node->m_leftInfo.IsLeaf())
				{
					ndInt32 index = ndInt32(node->m_leftInfo.GetIndex());
					ndInt32 vCount = ndInt32(node->m_leftInfo.GetCount());
					if (vCount > 0)
					{
						const ndInt32* const indices = &m_indices[index];
						ndInt32 normalIndex = indices[vCount + 1];
						ndVector faceNormal(&vertexArray[normalIndex].m_x);
						faceNormal = faceNormal & ndVector::m_triplexMask;
						ndFloat32 dist1 = obbAabbInfo.PolygonBoxDistance(faceNormal, vCount, indices, vertexArray);
						if (dist1 > ndFloat32(0.0f))
						{
							obbAabbInfo.m_separationDistance = ndFloat32(0.0f);
							ndAssert(vCount >= 3);
							callback(context, vertexArray, indices, vCount, dist1);
						}
					}
				}
			
				if (node->m_rightInfo.IsLeaf())
				{
					ndInt32 index = ndInt32(node->m_rightInfo.GetIndex());
					ndInt32 vCount = ndInt32(node->m_rightInfo.GetCount());
					if (vCount > 0)
					{
						const ndInt32* const indices = &m_indices[index];
						ndInt32 normalIndex = indices[vCount + 1];
						ndVector faceNormal(&vertexArray[normalIndex].m_x);
						faceNormal = faceNormal & ndVector::m_triplexMask;
						ndFloat32 dist1 = obbAabbInfo.PolygonBoxDistance(faceNormal, vCount, indices, vertexArray);
						if (dist1 > ndFloat32(0.0f))
						{
							ndAssert(vCount >= 3);
							obbAabbInfo.m_separationDistance = ndFloat32(0.0f);
							callback(context, vertexArray, indices, vCount, dist1);
						}
					}
				}
			}
		}
		else
		{
			ndAssert(0);
			//ndFastRay ray(ndVector(ndFloat32(0.0f)), boxDistanceTravel);
			//ndFastRay obbRay(ndVector(ndFloat32(0.0f)), obbAabbInfo.UnrotateVector(boxDistanceTravel));
			//ndInt32 stack = 1;
			//stackPool[0] = m_aabb;
			//distance[0] = m_aabb->BoxIntersect(ray, obbRay, obbAabbInfo, vertexArray);
			//
			//while (stack)
			//{
			//	stack--;
			//	const ndFloat32 dist = distance[stack];
			//	const ndNode* const me = stackPool[stack];
			//	if (dist < ndFloat32(1.0f))
			//	{
			//		if (me->m_left.IsLeaf())
			//		{
			//			ndInt32 index = ndInt32(me->m_left.GetIndex());
			//			ndInt32 vCount = ndInt32(me->m_left.GetCount());
			//			if (vCount > 0)
			//			{
			//				const ndInt32* const indices = &m_indices[index];
			//				ndInt32 normalIndex = indices[vCount + 1];
			//				ndVector faceNormal(&vertexArray[normalIndex].m_x);
			//				faceNormal = faceNormal & ndVector::m_triplexMask;
			//				ndFloat32 hitDistance = obbAabbInfo.PolygonBoxRayDistance(faceNormal, vCount, indices, stride, &vertexArray[0].m_x, ray);
			//				if (hitDistance < ndFloat32(1.0f))
			//				{
			//					ndAssert(vCount >= 3);
			//					if (callback(context, &vertexArray[0].m_x, sizeof(ndTriplex), indices, vCount, hitDistance) == m_stopSearch)
			//					{
			//						return;
			//					}
			//				}
			//			}
			//		}
			//		else
			//		{
			//			const ndNode* const node = me->m_left.GetNode(m_aabb);
			//			ndFloat32 dist1 = node->BoxIntersect(ray, obbRay, obbAabbInfo, vertexArray);
			//			if (dist1 < ndFloat32(1.0f))
			//			{
			//				ndInt32 j = stack;
			//				for (; j && (dist1 > distance[j - 1]); j--)
			//				{
			//					stackPool[j] = stackPool[j - 1];
			//					distance[j] = distance[j - 1];
			//				}
			//				ndAssert(stack < ND_STACK_DEPTH);
			//				stackPool[j] = node;
			//				distance[j] = dist1;
			//				stack++;
			//			}
			//		}
			//
			//		if (me->m_right.IsLeaf())
			//		{
			//			ndInt32 index = ndInt32(me->m_right.GetIndex());
			//			ndInt32 vCount = ndInt32(me->m_right.GetCount());
			//			if (vCount > 0)
			//			{
			//				const ndInt32* const indices = &m_indices[index];
			//				ndInt32 normalIndex = indices[vCount + 1];
			//				ndVector faceNormal(&vertexArray[normalIndex].m_x);
			//				faceNormal = faceNormal & ndVector::m_triplexMask;
			//				ndFloat32 hitDistance = obbAabbInfo.PolygonBoxRayDistance(faceNormal, vCount, indices, stride, &vertexArray[0].m_x, ray);
			//				if (hitDistance < ndFloat32(1.0f))
			//				{
			//					ndAssert(vCount >= 3);
			//					if (callback(context, &vertexArray[0].m_x, sizeof(ndTriplex), indices, vCount, hitDistance) == m_stopSearch) {
			//						return;
			//					}
			//				}
			//			}
			//		}
			//		else
			//		{
			//			const ndNode* const node = me->m_right.GetNode(m_aabb);
			//			ndFloat32 dist1 = node->BoxIntersect(ray, obbRay, obbAabbInfo, vertexArray);
			//			if (dist1 < ndFloat32(1.0f))
			//			{
			//				ndInt32 j = stack;
			//				for (; j && (dist1 > distance[j - 1]); j--)
			//				{
			//					stackPool[j] = stackPool[j - 1];
			//					distance[j] = distance[j - 1];
			//				}
			//				ndAssert(stack < ND_STACK_DEPTH);
			//				stackPool[j] = node;
			//				distance[j] = dist1;
			//				stack++;
			//			}
			//		}
			//	}
			//}
		}
	}
}

void ndAabbPolygonSoup::Serialize(ndArray<ndVector>& points, ndArray<ndInt32>& indices, ndArray<ndNode>& nodes) const
{
	const ndInt32 vertexCount = GetVertexCount();
	const ndVector* const vertex = GetLocalVertexPool();

	points.SetCount(vertexCount);
	points.SetCount(0);
	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		points.PushBack(vertex[i]);
	}

	indices.SetCount(m_indexCount);
	indices.SetCount(0);
	for (ndInt32 i = 0; i < m_indexCount; ++i)
	{
		indices.PushBack(m_indices[i]);
	}

	nodes.SetCount(m_nodesCount);
	nodes.SetCount(0);
	for (ndInt32 i = 0; i < m_nodesCount; ++i)
	{
		nodes.PushBack(m_aabb[i]);
	}
}

void ndAabbPolygonSoup::Create(const ndPolygonSoupBuilder& builder)
{
	if (builder.m_faceVertexCount.GetCount() == 0)
	{
		return;
	}
	ndAssert(builder.m_faceVertexCount.GetCount() >= 1);
	m_nodesCount = ((builder.m_faceVertexCount.GetCount() - 1) < 1) ? 1 : ndInt32(builder.m_faceVertexCount.GetCount()) - 1;
	m_aabb = reinterpret_cast<ndNode*>(ndMemory::Malloc(sizeof(ndNode) * m_nodesCount));
	m_indexCount = ndInt32(builder.m_vertexIndex.GetCount() * 2 + builder.m_faceVertexCount.GetCount());

	if (builder.m_faceVertexCount.GetCount() == 1)
	{
		m_indexCount *= 2;
	}

	m_indices = reinterpret_cast<ndInt32*>(ndMemory::Malloc(sizeof(ndInt32) * m_indexCount));
	ndStack<ndVector> tmpVertexArrayCount(ndInt32(builder.m_vertexPoints.GetCount() + builder.m_normalPoints.GetCount() + builder.m_faceVertexCount.GetCount() * 2 + 4));

	ndVector* const tmpVertexArray = &tmpVertexArrayCount[0];
	for (ndInt32 i = 0; i < builder.m_vertexPoints.GetCount(); ++i)
	{
		tmpVertexArray[i] = builder.m_vertexPoints[i];
	}

	for (ndInt32 i = 0; i < builder.m_normalPoints.GetCount(); ++i)
	{
		tmpVertexArray[i + builder.m_vertexPoints.GetCount()] = builder.m_normalPoints[i];
	}

	const ndInt32* const indices = &builder.m_vertexIndex[0];
	ndStack<ndNodeBuilder> constructor(ndInt32(builder.m_faceVertexCount.GetCount() * 2 + 16));

	ndInt32 polygonIndex = 0;
	ndInt32 allocatorIndex = 0;
	if (builder.m_faceVertexCount.GetCount() == 1)
	{
		ndInt32 indexCount = builder.m_faceVertexCount[0] - 1;
		new (&constructor[allocatorIndex]) ndNodeBuilder(&tmpVertexArray[0], 0, indexCount, &indices[0]);
		allocatorIndex++;
	}

	for (ndInt32 i = 0; i < ndInt32 (builder.m_faceVertexCount.GetCount()); ++i)
	{
		ndInt32 indexCount = builder.m_faceVertexCount[i] - 1;
		new (&constructor[allocatorIndex]) ndNodeBuilder(&tmpVertexArray[0], i, indexCount, &indices[polygonIndex]);
		allocatorIndex++;
		polygonIndex += (indexCount + 1);
	}
#if defined(ND_TOP_DOWN_BUILD)
	ndNodeBuilder* constructorAllocator = &constructor[allocatorIndex];
	ndNodeBuilder* const root = BuildTopDown(&constructor[0], 0, allocatorIndex - 1, &constructorAllocator);
#else
	ndNodeBuilder* const root = BuildBottomUp(&constructor[0], ndInt32(builder.m_faceVertexCount.GetCount()));
#endif

	ndAssert(root->SanityCheck());

	ndList<ndNodeBuilder*, ndContainersFreeListAlloc<ndNodeBuilder*>> list;
	ndAssert(root);
	list.Append(root);
	ndInt32 nodeIndex = 0;
	while (list.GetCount())
	{
		ndNodeBuilder* const node = list.GetFirst()->GetInfo();
		list.Remove(list.GetFirst());
	
		if (node->m_left)
		{
			ndAssert(node->m_right);
			node->m_enumeration = nodeIndex;
			nodeIndex++;
			list.Append(node->m_left);
			list.Append(node->m_right);
		}
	}
	ndAssert(!list.GetCount());

	ndInt32 aabbBase = ndInt32(builder.m_vertexPoints.GetCount() + builder.m_normalPoints.GetCount());

	ndVector* const aabbPoints = &tmpVertexArray[aabbBase];

	ndInt32 vertexIndex = 0;
	ndInt32 aabbNodeIndex = 0;
	list.Append(root);
	ndInt32 indexMap = 0;
	while (list.GetCount())
	{
		ndNodeBuilder* const node = list.GetFirst()->GetInfo();
		list.Remove(list.GetFirst());

		if (node->m_enumeration >= 0)
		{
			ndAssert(node->m_left);
			ndAssert(node->m_right);
			ndNode& aabbNode = m_aabb[aabbNodeIndex];
			aabbNodeIndex++;
			ndAssert(aabbNodeIndex <= m_nodesCount);

			if (node->m_parent)
			{
				if (node->m_parent->m_left == node)
				{
					m_aabb[node->m_parent->m_enumeration].m_leftInfo = ndNode::ndLeafNodePtr(ndUnsigned32(&m_aabb[node->m_enumeration] - m_aabb));
				}
				else
				{
					ndAssert(node->m_parent->m_right == node);
					m_aabb[node->m_parent->m_enumeration].m_rightInfo = ndNode::ndLeafNodePtr(ndUnsigned32(&m_aabb[node->m_enumeration] - m_aabb));
				}
			}

			aabbPoints[vertexIndex + 0] = node->m_p0;
			aabbPoints[vertexIndex + 1] = node->m_p1;

			aabbNode.m_indexBox0 = aabbBase + vertexIndex;
			aabbNode.m_indexBox1 = aabbBase + vertexIndex + 1;

			vertexIndex += 2;
		}
		else
		{
			ndAssert(!node->m_left);
			ndAssert(!node->m_right);

			if (node->m_parent)
			{
				if (node->m_parent->m_left == node)
				{
					m_aabb[node->m_parent->m_enumeration].m_leftInfo = ndNode::ndLeafNodePtr(ndUnsigned32(node->m_indexCount), ndUnsigned32(indexMap));
				}
				else
				{
					ndAssert(node->m_parent->m_right == node);
					m_aabb[node->m_parent->m_enumeration].m_rightInfo = ndNode::ndLeafNodePtr(ndUnsigned32(node->m_indexCount), ndUnsigned32(indexMap));
				}
			}

			// index format i0, i1, i2, ... , id, normal, e0Normal, e1Normal, e2Normal, ..., faceSize
			for (ndInt32 j = 0; j < node->m_indexCount; ++j)
			{
				m_indices[indexMap + j] = node->m_faceIndices[j];
				m_indices[indexMap + j + node->m_indexCount + 2] = D_CONCAVE_EDGE_MASK | 0xffffffff;
			}

			// face attribute
			m_indices[indexMap + node->m_indexCount] = node->m_faceIndices[node->m_indexCount];
			// face normal
			m_indices[indexMap + node->m_indexCount + 1] = ndInt32(builder.m_vertexPoints.GetCount()) + builder.m_normalIndex[node->m_faceIndex];
			// face size
			ndFloat32 faceMaxDiag = CalculateFaceMaxDiagonal(&tmpVertexArray[0], node->m_indexCount, node->m_faceIndices);
			ndInt32 quantizedDiagSize = ndInt32(ndFloor(faceMaxDiag / D_FACE_CLIP_DIAGONAL_SCALE + ndFloat32(1.0f)));
			m_indices[indexMap + node->m_indexCount * 2 + 2] = quantizedDiagSize;

			indexMap += node->m_indexCount * 2 + 3;
		}

		if (node->m_left)
		{
			ndAssert(node->m_right);
			list.Append(node->m_left);
			list.Append(node->m_right);
		}
	}

	ndStack<ndInt32> indexArray(vertexIndex);
	ndInt32 aabbPointCount = ndVertexListToIndexList(&aabbPoints[0].m_x, sizeof(ndVector), 3, vertexIndex, &indexArray[0], ndFloat64(1.0e-6f));

	m_vertexCount = aabbBase + aabbPointCount;
	m_localVertex = reinterpret_cast<ndVector*>(ndMemory::Malloc(sizeof(ndVector) * m_vertexCount));

	for (ndInt32 i = 0; i < m_vertexCount; ++i)
	{
		m_localVertex[i].m_x = tmpVertexArray[i].m_x;
		m_localVertex[i].m_y = tmpVertexArray[i].m_y;
		m_localVertex[i].m_z = tmpVertexArray[i].m_z;
		m_localVertex[i].m_w = ndFloat32(0.0f);
	}

	for (ndInt32 i = 0; i < m_nodesCount; ++i)
	{
		ndNode& box = m_aabb[i];

		ndInt32 j = box.m_indexBox0 - aabbBase;
		box.m_indexBox0 = indexArray[j] + aabbBase;
		j = box.m_indexBox1 - aabbBase;
		box.m_indexBox1 = indexArray[j] + aabbBase;
	}

	if (builder.m_faceVertexCount.GetCount() == 1)
	{
		m_aabb[0].m_rightInfo = ndNode::ndLeafNodePtr(0, 0);
	}
}

ndAabbPolygonSoup::ndNodeBuilder* ndAabbPolygonSoup::BuildTopDown(ndNodeBuilder* const leafArray, ndInt32 firstBox, ndInt32 lastBox, ndNodeBuilder** const allocator) const
{
	class ndSplitInfo
	{
		public:
		ndSplitInfo(ndNodeBuilder* const boxArray, ndInt32 boxCount)
		{
			ndVector minP(ndFloat32(1.0e15f));
			ndVector maxP(-ndFloat32(1.0e15f));

			if (boxCount == 2)
			{
				m_axis = 1;
				for (ndInt32 i = 0; i < boxCount; ++i)
				{
					const ndNodeBuilder& box = boxArray[i];
					const ndVector& p0 = box.m_p0;
					const ndVector& p1 = box.m_p1;
					minP = minP.GetMin(p0);
					maxP = maxP.GetMax(p1);
				}
			}
			else
			{
				ndVector median(ndVector::m_zero);
				ndVector varian(ndVector::m_zero);
				for (ndInt32 i = 0; i < boxCount; ++i)
				{
					const ndNodeBuilder& box = boxArray[i];

					const ndVector& p0 = box.m_p0;
					const ndVector& p1 = box.m_p1;

					minP = minP.GetMin(p0);
					maxP = maxP.GetMax(p1);
					ndVector p(ndVector::m_half * (p0 + p1));

					median += p;
					varian += p * p;
				}

				varian = varian.Scale(ndFloat32(boxCount)) - median * median;

				ndInt32 index = 0;
				ndFloat32 maxVarian = ndFloat32(-1.0e10f);
				for (ndInt32 i = 0; i < 3; ++i)
				{
					if (varian[i] > maxVarian)
					{
						index = i;
						maxVarian = varian[i];
					}
				}

				ndVector center = median.Scale(ndFloat32(1.0f) / ndFloat32(boxCount));
				ndFloat32 test = center[index];
				ndInt32 i0 = 0;
				ndInt32 i1 = boxCount - 1;
				do
				{
					for (; i0 <= i1; ++i0)
					{
						const ndNodeBuilder& box = boxArray[i0];
						ndFloat32 val = (box.m_p0[index] + box.m_p1[index]) * ndFloat32(0.5f);
						if (val > test)
						{
							break;
						}
					}

					for (; i1 >= i0; --i1)
					{
						const ndNodeBuilder& box = boxArray[i1];
						ndFloat32 val = (box.m_p0[index] + box.m_p1[index]) * ndFloat32(0.5f);
						if (val < test)
						{
							break;
						}
					}

					if (i0 < i1)
					{
						ndSwap(boxArray[i0], boxArray[i1]);
						i0++;
						i1--;
					}
				} while (i0 <= i1);

				if (i0 > 0)
				{
					i0--;
				}
				if ((i0 + 1) >= boxCount)
				{
					i0 = boxCount - 2;
				}

				m_axis = i0 + 1;
			}

			ndAssert(maxP.m_x - minP.m_x >= ndFloat32(0.0f));
			ndAssert(maxP.m_y - minP.m_y >= ndFloat32(0.0f));
			ndAssert(maxP.m_z - minP.m_z >= ndFloat32(0.0f));
			m_p0 = minP;
			m_p1 = maxP;
		}

		ndInt32 m_axis;
		ndVector m_p0;
		ndVector m_p1;
	};

	ndAssert(firstBox >= 0);
	ndAssert(lastBox >= 0);

	if (lastBox == firstBox)
	{
		return &leafArray[firstBox];
	}
	else
	{
		ndSplitInfo info(&leafArray[firstBox], lastBox - firstBox + 1);

		ndNodeBuilder* const parent = new (*allocator) ndNodeBuilder(info.m_p0, info.m_p1);
		*allocator = *allocator + 1;

		ndAssert(parent);
		parent->m_right = BuildTopDown(leafArray, firstBox + info.m_axis, lastBox, allocator);
		parent->m_right->m_parent = parent;

		parent->m_left = BuildTopDown(leafArray, firstBox, firstBox + info.m_axis - 1, allocator);
		parent->m_left->m_parent = parent;
		return parent;
	}
}

ndAabbPolygonSoup::ndNodeBuilder* ndAabbPolygonSoup::BuildBottomUp(ndNodeBuilder* const array, ndInt32 count) const
{
	enum
	{
		m_linkedCell,
		m_insideCell,
		m_outsideCell,
	};

	class ndBottomUpCell
	{
		public:
		ndBottomUpCell()
		{
		}

		ndBottomUpCell(ndInt32 x, ndInt32 y, ndInt32 z, ndNodeBuilder* const node)
			:m_x(x)
			,m_y(y)
			,m_z(z)
			,m_node(node)
		{
		}

		union
		{
			ndInt32 m_index[3];
			struct
			{
				ndInt32 m_x;
				ndInt32 m_y;
				ndInt32 m_z;
			};
		};
		ndNodeBuilder* m_node;
	};

	class ndCellScanPrefix
	{
		public:
		ndCellScanPrefix() {}
		ndCellScanPrefix(ndInt32 location, ndUnsigned32 cellTest)
			:m_location(location)
			,m_cellTest(cellTest)
		{
		}
		ndInt32 m_location : 30;
		ndUnsigned32 m_cellTest : 1;
	};

	auto CalculateSize = [](ndInt32 nodeCount, ndNodeBuilder** const buffer)
	{
		ndFloat32 minSize = ndFloat32(1.0e15f);
		for (ndInt32 i = 0; i < nodeCount; ++i)
		{
			const ndNodeBuilder* const node = buffer[i];
			const ndVector size(node->m_p1 - node->m_p0);
			ndFloat32 maxDim = ndMax(ndMax(size.m_x, size.m_y), size.m_z);
			minSize = ndMin(maxDim, minSize);
		}
		return minSize;
	};

	if (count == 1)
	{
		ndNodeBuilder* const left = &array[0];
		ndNodeBuilder* const right = &array[1];
		ndNodeBuilder* const root = &array[2];
		left->m_parent = root;
		right->m_parent = root;

		root->m_left = left;
		root->m_right = right;
		root->Init(left, right);
		return root;
	}

	ndArray<ndNodeBuilder*> buffer;
	ndArray<ndBottomUpCell> cellBuffer0;
	ndArray<ndBottomUpCell> cellBuffer1;
	ndArray<ndNodeBuilder*> scratchBuffer;
	ndArray<ndCellScanPrefix> cellsCount;
	ndArray<ndCellScanPrefix> cellsCountScratch;

	buffer.SetCount(count * 2);
	cellBuffer0.SetCount(count * 2);
	cellBuffer1.SetCount(count * 2);
	scratchBuffer.SetCount(count * 2);
	cellsCount.SetCount(count * 2);
	cellsCountScratch.SetCount(count * 2);

	ndNodeBuilder** leafNodes = &buffer[0];
	ndNodeBuilder** freeNodes = &buffer[count];

	auto BuildSmallBvhTree = [&]()
	{
		auto MakeTwoNodesTree = [](ndNodeBuilder* const root, ndNodeBuilder* const left, ndNodeBuilder* const right)
		{
			ndAssert(!left->m_bhvLinked);
			ndAssert(!right->m_bhvLinked);

			root->Init(left, right);
			left->m_parent = root;
			right->m_parent = root;

			root->m_left = left;
			root->m_right = right;
		};

		auto MakeThreeNodesTree = [](ndNodeBuilder* const root, ndNodeBuilder* const subRoot, ndNodeBuilder* const node0, ndNodeBuilder* const node1, ndNodeBuilder* const node2)
		{
			class ndNodeOrder
			{
				public:
				ndVector m_p0;
				ndVector m_p1;
				ndNodeBuilder* m_node0;
				ndNodeBuilder* m_node1;
				ndNodeBuilder* m_node2;
				ndFloat32 m_area;
			};

			ndNodeOrder order[3];

			ndAssert(!node0->m_bhvLinked);
			ndAssert(!node1->m_bhvLinked);
			ndAssert(!node2->m_bhvLinked);

			order[0].m_node0 = node0;
			order[0].m_node1 = node1;
			order[0].m_node2 = node2;

			order[1].m_node0 = node1;
			order[1].m_node1 = node2;
			order[1].m_node2 = node0;

			order[2].m_node0 = node2;
			order[2].m_node1 = node0;
			order[2].m_node2 = node1;

			for (ndInt32 i = 0; i < 3; ++i)
			{
				order[i].m_p0 = order[i].m_node0->m_p0.GetMin(order[i].m_node1->m_p0);
				order[i].m_p1 = order[i].m_node0->m_p1.GetMax(order[i].m_node1->m_p1);
				const ndVector size(order[i].m_p1 - order[i].m_p0);
				order[i].m_area = size.DotProduct(size.ShiftTripleRight()).GetScalar();
			}

			if (order[2].m_area < order[1].m_area)
			{
				ndSwap(order[1], order[2]);
			}
			if (order[1].m_area < order[0].m_area)
			{
				ndSwap(order[1], order[0]);
			}

			subRoot->Init(order[0].m_node0, order[0].m_node1);
			subRoot->m_parent = root;
			subRoot->m_left = order[0].m_node0;
			subRoot->m_right = order[0].m_node1;
			subRoot->m_p0 = order[0].m_p0;
			subRoot->m_p1 = order[0].m_p1;
			subRoot->m_left->m_parent = subRoot;
			subRoot->m_right->m_parent = subRoot;
			
			root->Init(subRoot, order[0].m_node2);
			root->m_right = subRoot;
			root->m_left = order[0].m_node2;
			root->m_left->m_parent = root;
		};

		ndInt32 nextFreeNodes = 0;
		for (ndInt32 i = 0; i < ndInt32(cellsCountScratch.GetCount()) - 1; ++i)
		{
			const ndInt32 nodesCount = cellsCountScratch[i + 1].m_location - cellsCountScratch[i].m_location;
			if (nodesCount == 1)
			{
				const ndInt32 childIndex = cellsCount[i].m_location;
				ndNodeBuilder* const node0 = cellBuffer0[childIndex + 0].m_node;
				ndNodeBuilder* const node1 = cellBuffer0[childIndex + 1].m_node;
				
				ndNodeBuilder* const root = freeNodes[nextFreeNodes];
				nextFreeNodes++;
				ndAssert(root);
				MakeTwoNodesTree(root, node0, node1);
				root->m_bhvLinked = 0;
			}
			else if (nodesCount == 2)
			{
				const ndInt32 childIndex = cellsCount[i].m_location;
				
				ndNodeBuilder* const node0 = cellBuffer0[childIndex + 0].m_node;
				ndNodeBuilder* const node1 = cellBuffer0[childIndex + 1].m_node;
				ndNodeBuilder* const node2 = cellBuffer0[childIndex + 2].m_node;
				
				ndNodeBuilder* const root = freeNodes[nextFreeNodes];
				ndNodeBuilder* const subParent = freeNodes[nextFreeNodes + 1];
				nextFreeNodes += 2;

				MakeThreeNodesTree(root, subParent, node0, node1, node2);
				root->m_bhvLinked = 0;
			}
			else if (nodesCount > 2)
			{
				class ndBlockSegment
				{
					public:
					ndBlockSegment() {}
					ndBlockSegment(ndNodeBuilder* const parent, ndInt32 start, ndInt32 count)
						:m_parent(parent)
						,m_start(start)
						,m_count(count)
					{
					}
				
					ndNodeBuilder* m_parent;
					ndInt32 m_start;
					ndInt32 m_count;
				};
				
				ndFixSizeArray<ndBlockSegment, 128> stackPool;
				
				ndNodeBuilder* const rootNode = freeNodes[nextFreeNodes];
				nextFreeNodes++;
				rootNode->SetBox(ndVector::m_zero, ndVector::m_zero);
				rootNode->Init(rootNode, rootNode);
				
				rootNode->m_bhvLinked = 0;
				stackPool.PushBack(ndBlockSegment(rootNode, cellsCount[i].m_location, cellsCount[i + 1].m_location - cellsCount[i].m_location));
				
				while (stackPool.GetCount())
				{
					const ndBlockSegment block(stackPool.Pop());
					ndAssert(block.m_count > 2);
				
					ndVector minP(ndFloat32(1.0e15f));
					ndVector maxP(ndFloat32(-1.0e15f));
					ndVector median(ndVector::m_zero);
					ndVector varian(ndVector::m_zero);
				
					for (ndInt32 m = 0; m < block.m_count; ++m)
					{
						ndNodeBuilder* const node = cellBuffer0[block.m_start + m].m_node;
						minP = minP.GetMin(node->m_p0);
						maxP = maxP.GetMax(node->m_p1);
						ndVector p(ndVector::m_half * (node->m_p0 + node->m_p1));
						median += p;
						varian += p * p;
					}
				
					ndNodeBuilder* const root = block.m_parent;
					root->SetBox(minP, maxP);
				
					ndInt32 index = 0;
					ndFloat32 maxVarian = ndFloat32(-1.0e15f);
					varian = varian.Scale(ndFloat32(block.m_count)) - median * median;
					for (ndInt32 m = 0; m < 3; ++m)
					{
						if (varian[m] > maxVarian)
						{
							index = m;
							maxVarian = varian[m];
						}
					}
				
					ndInt32 index0 = block.m_start + block.m_count / 2;
					if (maxVarian > ndFloat32(1.0e-3f))
					{
						ndUnsigned32 scan[8];
						const ndFloat32 midPoint = median[index] / ndFloat32(block.m_count);
						ndCountingSortInPlace(&cellBuffer0[block.m_start], &cellBuffer1[0], block.m_count, scan, 2, [midPoint, index](const ndBottomUpCell& cell)
						{
							const ndNodeBuilder* const node = cell.m_node;
							const ndVector p(ndVector::m_half * (node->m_p0 + node->m_p1));
							ndInt32 key = p[index] >= midPoint;
							return key;
						});
				
						index0 = block.m_start + ndInt32(scan[1]);
						if (index0 == block.m_start)
						{
							index0 = block.m_start + block.m_count / 2;
						}
						if (index0 == (block.m_start + block.m_count))
						{
							index0 = block.m_start + block.m_count / 2;
						}
					}
				
					ndAssert(index0 > block.m_start);
					ndAssert(index0 < (block.m_start + block.m_count));
				
					ndInt32 count0 = index0 - block.m_start;
				
					ndAssert(count0);
					if (count0 == 1)
					{
						ndNodeBuilder* const node = cellBuffer0[block.m_start].m_node;
						node->m_bhvLinked = 1;
						node->m_parent = root;
						root->m_left = node;
					}
					else if (count0 == 2)
					{
						ndNodeBuilder* const node0 = cellBuffer0[block.m_start + 0].m_node;
						ndNodeBuilder* const node1 = cellBuffer0[block.m_start + 1].m_node;
						ndNodeBuilder* const leftChild = freeNodes[nextFreeNodes];
						nextFreeNodes++;
						MakeTwoNodesTree(leftChild, node0, node1);
						leftChild->m_parent = root;
						root->m_left = leftChild;
					}
					else if (count0 == 3)
					{
						ndNodeBuilder* const node0 = cellBuffer0[block.m_start + 0].m_node;
						ndNodeBuilder* const node1 = cellBuffer0[block.m_start + 1].m_node;
						ndNodeBuilder* const node2 = cellBuffer0[block.m_start + 2].m_node;
				
						ndNodeBuilder* const leftChild = freeNodes[nextFreeNodes];
						nextFreeNodes++;
				
						ndNodeBuilder* const children = freeNodes[nextFreeNodes];
						nextFreeNodes++;
				
						MakeThreeNodesTree(leftChild, children, node0, node1, node2);
						leftChild->m_parent = root;
						root->m_left = leftChild;
					}
					else
					{
						ndNodeBuilder* const leftChild = freeNodes[nextFreeNodes];
						nextFreeNodes++;
						leftChild->Init(root, root);
						leftChild->SetBox(ndVector::m_zero, ndVector::m_zero);

						leftChild->m_parent = root;
						leftChild->m_left = nullptr;
						leftChild->m_right = nullptr;
						root->m_left = leftChild;
						stackPool.PushBack(ndBlockSegment(leftChild, block.m_start, count0));
					}
				
					ndInt32 count1 = block.m_start + block.m_count - index0;
					ndAssert(count1);
					if (count1 == 1)
					{
						ndNodeBuilder* const node = cellBuffer0[index0].m_node;
						node->m_bhvLinked = 1;
						node->m_parent = root;
						root->m_right = node;
					}
					else if (count1 == 2)
					{
						ndNodeBuilder* const node0 = cellBuffer0[index0 + 0].m_node;
						ndNodeBuilder* const node1 = cellBuffer0[index0 + 1].m_node;
						ndNodeBuilder* const rightChild = freeNodes[nextFreeNodes];
						nextFreeNodes++;
						MakeTwoNodesTree(rightChild, node0, node1);
						rightChild->m_parent = root;
						root->m_right = rightChild;
					}
					else if (count1 == 3)
					{
						ndNodeBuilder* const node0 = cellBuffer0[index0 + 0].m_node;
						ndNodeBuilder* const node1 = cellBuffer0[index0 + 1].m_node;
						ndNodeBuilder* const node2 = cellBuffer0[index0 + 2].m_node;
				
						ndNodeBuilder* const rightChild = freeNodes[nextFreeNodes];
						nextFreeNodes++;
						
						ndNodeBuilder* const children = freeNodes[nextFreeNodes];
						nextFreeNodes++;
				
						MakeThreeNodesTree(rightChild, children, node0, node1, node2);
						rightChild->m_parent = root;
						root->m_right = rightChild;
					}
					else
					{
						ndNodeBuilder* const rightChild = freeNodes[nextFreeNodes];
						nextFreeNodes++;
						rightChild->Init(root, root);
						rightChild->SetBox(ndVector::m_zero, ndVector::m_zero);
						 
						rightChild->m_parent = root;
						rightChild->m_left = nullptr;
						rightChild->m_right = nullptr;
						root->m_right = rightChild;
					
						stackPool.PushBack(ndBlockSegment(rightChild, index0, count1));
					}
				}
				rootNode->m_bhvLinked = 0;
			}
		}
		return nextFreeNodes;
	};

	buffer.SetCount(0);
	for (ndInt32 i = 0; i < count * 2; ++i)
	{
		#if defined(_DEBUG)
			array[i].m_debugIndex = i;
		#endif
		buffer.PushBack(&array[i]);
	}

	ndVector origin(ndFloat32(1.0e15f));
	for (ndInt32 i = 0; i < count; ++i)
	{
		const ndNodeBuilder* const node = buffer[i];
		origin = origin.GetMin(node->m_p0);
	}

	ndInt32 leafNodesCount = count;
	ndFloat32 minSize = CalculateSize(leafNodesCount, &leafNodes[0]);
	while (leafNodesCount > 1)
	{
		minSize = minSize * ndFloat32(2.0f);

		// classify nodes categorically
		const ndVector invSize(ndVector::m_triplexMask& ndVector(ndFloat32(1.0f) / minSize));

		ndInt32 m_code[4];
		ndUnsigned32 prefixScan[5];
		m_code[0] = m_outsideCell;
		m_code[1] = m_insideCell;
		m_code[2] = m_linkedCell;
		m_code[3] = m_linkedCell;
		ndCountingSortInPlace(leafNodes, &scratchBuffer[0], leafNodesCount, prefixScan, 2, [&](const ndNodeBuilder* const node)
		{
			const ndVector minPosit((invSize * (node->m_p0 - origin)).GetInt());
			const ndVector maxPosit((invSize * (node->m_p1 - origin)).GetInt());
			const ndInt32 x0 = ndInt32(minPosit.m_ix);
			const ndInt32 y0 = ndInt32(minPosit.m_iy);
			const ndInt32 z0 = ndInt32(minPosit.m_iz);
			const ndInt32 x1 = ndInt32(maxPosit.m_ix);
			const ndInt32 y1 = ndInt32(maxPosit.m_iy);
			const ndInt32 z1 = ndInt32(maxPosit.m_iz);

			ndAssert(x0 >= 0);
			ndAssert(y0 >= 0);
			ndAssert(z0 >= 0);
			ndAssert(x1 >= 0);
			ndAssert(y1 >= 0);
			ndAssert(z1 >= 0);

			const ndUnsigned32 test_x = ndUnsigned32(((x1 - x0) >> 1) == 0);
			const ndUnsigned32 test_y = ndUnsigned32(((y1 - y0) >> 1) == 0);
			const ndUnsigned32 test_z = ndUnsigned32(((z1 - z0) >> 1) == 0);
			const ndUnsigned32 test = test_x & test_y & test_z;
			const ndUnsigned32 codeIndex = node->m_bhvLinked * 2 + test;
			return m_code[codeIndex];
		});

		const ndInt32 insideCellsCount = ndInt32(prefixScan[m_insideCell + 1] - prefixScan[m_insideCell]);
		if (insideCellsCount)
		{
			// make integer grids
			ndInt32 maxGrid[3];
			maxGrid[0] = 0;
			maxGrid[1] = 0;
			maxGrid[2] = 0;

			const ndUnsigned32 linkedNodes = prefixScan[m_linkedCell + 1] - prefixScan[m_linkedCell];
			leafNodes += linkedNodes;
			leafNodesCount -= linkedNodes;
			ndAssert(leafNodesCount > 0);
			ndAssert(leafNodesCount <= count);
			if (leafNodesCount == 1)
			{
				break;
			}

			cellBuffer0.SetCount(0);
			for (ndInt32 i = 0; i < insideCellsCount; ++i)
			{
				ndNodeBuilder* const node = leafNodes[i];

				const ndVector dist(node->m_p0 - origin);
				const ndVector posit(invSize * dist);
				const ndVector intPosit(posit.GetInt());
				cellBuffer0.PushBack(ndBottomUpCell(ndInt32(intPosit.m_ix), ndInt32(intPosit.m_iy), ndInt32(intPosit.m_iz), node));
				maxGrid[0] = ndMax(ndInt32(intPosit.m_ix), maxGrid[0]);
				maxGrid[1] = ndMax(ndInt32(intPosit.m_iy), maxGrid[1]);
				maxGrid[2] = ndMax(ndInt32(intPosit.m_iz), maxGrid[2]);
			}
			ndAssert(maxGrid[0] < 0x7fffffff);
			ndAssert(maxGrid[1] < 0x7fffffff);
			ndAssert(maxGrid[2] < 0x7fffffff);

			for (ndInt32 i = 0; i < 3; ++i)
			{
				ndInt64 shift = 0;
				do
				{
					ndCountingSortInPlace(&cellBuffer0[0], &cellBuffer1[0], ndInt32(cellBuffer0.GetCount()), nullptr, 8, [&](const ndBottomUpCell& cell)
					{
						ndInt32 val = cell.m_index[i] >> shift;
						return val & 0xff;
					});

					shift += 8;
				} while (maxGrid[i] >= (1 << shift));
			}

			ndBottomUpCell sentinelCell(0x7fffffff, 0x7fffffff, 0x7fffffff, nullptr);
			cellBuffer0.PushBack(sentinelCell);

			cellsCount.SetCount(0);
			cellsCount.PushBack(ndCellScanPrefix(0, 0));
			for (ndInt32 i = 0; i < insideCellsCount; ++i)
			{
				const ndBottomUpCell& cell0 = cellBuffer0[i];
				const ndBottomUpCell& cell1 = cellBuffer0[i + 1];
				const ndUnsigned8 test = ndUnsigned8((cell0.m_x == cell1.m_x) & (cell0.m_y == cell1.m_y) & (cell0.m_z == cell1.m_z) & (cell1.m_node != nullptr));
				cellsCount.PushBack(ndCellScanPrefix(i + 1, test));
			}
			ndUnsigned32 cellPrefixScan[3];
			ndCountingSortInPlace(&cellsCount[0], &cellsCountScratch[0], ndInt32(cellsCount.GetCount()), cellPrefixScan, 1, [](const ndCellScanPrefix& cell)
			{
				return ndInt32(cell.m_cellTest);
			});

			ndInt32 sum = 0;
			const ndInt32 batchCount = ndInt32(cellPrefixScan[1] - 1);
			cellsCountScratch.SetCount(0);
			for (ndInt32 i = 0; i < batchCount; ++i)
			{
				const ndInt32 nodesCount = cellsCount[i + 1].m_location - cellsCount[i].m_location - 1;
				cellsCountScratch.PushBack(ndCellScanPrefix(sum, 0));
				sum += nodesCount;
			}
			cellsCountScratch.PushBack(ndCellScanPrefix(sum, 0));

			if (sum)
			{
				ndInt32 newNodes = BuildSmallBvhTree();

				ndAssert(newNodes <= sum);
				freeNodes += newNodes;
				leafNodesCount += newNodes;

				// restart the size
				ndFloat32 restartSize = ndFloat32(1.0e15f);
				for (ndInt32 i = 0; i < leafNodesCount; ++i)
				{
					const ndNodeBuilder* const node = leafNodes[i];
					if (!node->m_bhvLinked)
					{
						const ndVector size(node->m_p1 - node->m_p0);
						ndFloat32 maxDim = ndMax(ndMax(size.m_x, size.m_y), size.m_z);
						restartSize = ndMin(maxDim, restartSize);
					}
				}
				minSize = ndMin(minSize, restartSize);
			}
		}
	}

	ndNodeBuilder* const root = *leafNodes;
	return root;
}