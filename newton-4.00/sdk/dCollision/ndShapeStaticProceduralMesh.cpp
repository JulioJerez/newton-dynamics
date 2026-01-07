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
#include "ndContact.h"
#include "ndBodyKinematic.h"
#include "ndShapeInstance.h"
#include "ndPolygonMeshDesc.h"
#include "ndShapeStaticProceduralMesh.h"

ndShapeStaticProceduralMesh::ndShapeStaticProceduralMesh()
	:ndShapeStaticMesh(m_staticProceduralMesh)
{
	SetAABB(ndVector::m_negOne, ndVector::m_one);
}

ndShapeStaticProceduralMesh::~ndShapeStaticProceduralMesh(void)
{
}

ndShapeInfo ndShapeStaticProceduralMesh::GetShapeInfo() const
{
	ndShapeInfo info(ndShapeStaticMesh::GetShapeInfo());
	info.m_procedural.m_noUsed = 0;
	return info;
}

void ndShapeStaticProceduralMesh::SetAABB(const ndVector& p0, const ndVector& p1)
{
	ndVector q0(p0.GetMin(p1));
	ndVector q1(p0.GetMax(p1));
	m_boxOrigin = ndVector::m_half * (q1 + q0) & ndVector::m_triplexMask;
	m_boxSize = ndVector::m_half * (q1 - q0) & ndVector::m_triplexMask;
}

void ndShapeStaticProceduralMesh::GetCollidingFaces(ndPolygonMeshDesc* const data) const
{
	ndPatchMesh patch;
	patch.m_boxP0 = data->GetOrigin();
	patch.m_boxP1 = data->GetTarget();
	patch.m_boxP0 += data->m_boxDistanceTravelInMeshSpace & (data->m_boxDistanceTravelInMeshSpace < ndVector::m_zero);
	patch.m_boxP1 += data->m_boxDistanceTravelInMeshSpace & (data->m_boxDistanceTravelInMeshSpace > ndVector::m_zero);
	GetFacesPatch(patch);
	data->SetSeparatingDistance(ndFloat32(0.0f));

	if (patch.m_faceArray.GetCount())
	{
		patch.GetFacesPatch(data);
		ndInt32 faceCount0 = 0;
		ndInt32 faceIndexCount0 = 0;
		ndInt32 faceIndexCount1 = 0;
		const ndInt32 stride = sizeof(ndVector) / sizeof(ndFloat32);

		ndPolygonMeshDesc::ndStaticMeshFaceQuery& query = *data->m_staticMeshQuery;
		ndPolygonMeshDesc::ndProceduralStaticMeshFaceQuery& meshPatch = *data->m_proceduralStaticMeshFaceQuery;
		ndArray<ndVector>& vertex = meshPatch.m_vertex;
		ndArray<ndInt32>& address = query.m_faceIndexStart;
		ndArray<ndInt32>& faceList = query.m_faceIndexCount;
		ndArray<ndInt32>& indices = query.m_faceVertexIndex;
		ndArray<ndFloat32>& hitDistance = query.m_hitDistance;
		ndArray<ndInt32>& faceIndexCount = query.m_faceIndexCount;
		if (data->m_doContinueCollisionTest)
		{
			ndFastRay ray(ndVector::m_zero, data->m_boxDistanceTravelInMeshSpace);
			for (ndInt32 i = 0; i < faceList.GetCount(); ++i)
			{
				const ndInt32 vertexCount = faceIndexCount[i];
				const ndInt32* const indexArray = &indices[faceIndexCount1];
				const ndVector& faceNormal = vertex[indexArray[4]];
				ndFloat32 dist = data->PolygonBoxRayDistance(faceNormal, 3, indexArray, stride, &vertex[0].m_x, ray);
				if (dist < ndFloat32(1.0f))
				{
					hitDistance.PushBack(dist);
					address.PushBack(faceIndexCount0);
					ndMemCpy(&indices[faceIndexCount0], indexArray, vertexCount * 2 + 3);
					faceCount0++;
					faceIndexCount0 += vertexCount * 2 + 3;
				}
				faceIndexCount1 += vertexCount * 2 + 3;
			}
		}
		else
		{
			for (ndInt32 i = 0; i < faceList.GetCount(); ++i)
			{
				const ndInt32 vertexCount = faceIndexCount[i];
				const ndInt32* const indexArray = &indices[faceIndexCount1];
				const ndVector& faceNormal = vertex[indexArray[vertexCount + 1]];
				ndFloat32 dist = data->PolygonBoxDistance(faceNormal, vertexCount, indexArray, stride, &vertex[0].m_x);
				if (dist > ndFloat32(0.0f))
				{
					hitDistance.PushBack(dist);
					address.PushBack(faceIndexCount0);
					ndMemCpy(&indices[faceIndexCount0], indexArray, vertexCount * 2 + 3);
					faceCount0++;
					faceIndexCount0 += vertexCount * 2 + 3;
				}
				faceIndexCount1 += vertexCount * 2 + 3;
			}
		}

		// initialize the callback data structure
		faceIndexCount.SetCount(faceCount0);
		data->m_vertex = &vertex[0].m_x;
		data->m_vertexStrideInBytes = sizeof(ndVector);
	}
}

ndUnsigned64 ndShapeStaticProceduralMesh::GetHash(ndUnsigned64 hash) const
{
	return hash + 1;
}