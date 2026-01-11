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

#ifndef __ND_SHAPE_STATIC_MESH_H__
#define __ND_SHAPE_STATIC_MESH_H__

#include "ndCollisionStdafx.h"
#include "ndShape.h"

class ndShapeInstance;
class ndPolygonMeshDesc;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndShapeStaticMesh: public ndShape
{
	public:
	class ndPatchMesh
	{
		#define MESH_SIZE 512

		public:
		enum ndQueryType
		{
			m_buildIndexList,
			m_vertexListOnly,
		};

		ndPatchMesh()
			:m_convexShapeInstance(nullptr)
			,m_queryType(m_buildIndexList)
			,m_vertexArrayHasDuplicated(true)
		{
		}
		
		ndVector m_boxP0;
		ndVector m_boxP1;

		ndFixSizeArray<ndVector, MESH_SIZE> m_pointArray;
		ndFixSizeArray<ndVector, MESH_SIZE> m_normalArray;

		ndFixSizeArray<ndInt32, MESH_SIZE> m_faceArray;
		ndFixSizeArray<ndInt32, MESH_SIZE * 4> m_indexArray;
		ndFixSizeArray<ndInt32, MESH_SIZE> m_faceMaterialArray;

		const ndShapeInstance* m_convexShapeInstance;

		ndQueryType m_queryType;
		bool m_vertexArrayHasDuplicated;

		private:
		class ndEdgeList
		{
			public:
			ndInt32 m_key;
			ndInt32 m_edge;
			ndInt32 m_faceStart;
			ndInt32 m_faceVertexCount;
		};

		void GetFacesPatch(ndPolygonMeshDesc* const data);

		friend class ndShapeHeightfield;
		friend class ndShapeStaticProceduralMesh;
	};

	D_CLASS_REFLECTION(ndShapeStaticMesh,ndShape)
	D_COLLISION_API ndShapeStaticMesh(ndShapeID id);
	D_COLLISION_API virtual ~ndShapeStaticMesh();

	D_COLLISION_API virtual void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const override;
	D_COLLISION_API virtual ndFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const override;
	D_COLLISION_API virtual void GetCollidingFaces(ndPolygonMeshDesc* const data) const;

	protected:
	D_COLLISION_API virtual ndFloat32 GetVolume() const override;
	D_COLLISION_API virtual ndFloat32 GetBoxMinRadius() const override;
	D_COLLISION_API virtual ndFloat32 GetBoxMaxRadius() const override;
	D_COLLISION_API virtual ndShapeStaticMesh* GetAsShapeStaticMesh() override;
	D_COLLISION_API virtual ndVector SupportVertex(const ndVector& dir) const override;
	D_COLLISION_API virtual ndVector SupportVertexSpecial(const ndVector& dir, ndFloat32 skinMargin) const override;
	D_COLLISION_API virtual ndVector SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector& dir) const override;
	D_COLLISION_API virtual ndInt32 CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const override;
	D_COLLISION_API virtual ndVector CalculateVolumeIntegral(const ndMatrix& globalMatrix, const ndVector& plane, const ndShapeInstance& parentScale) const override;

	D_COLLISION_API virtual void CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const override;
	D_COLLISION_API ndInt32 CalculatePlaneIntersection(const ndFloat32* const vertex, const ndInt32* const index, ndInt32 indexCount, ndInt32 strideInFloat, const ndPlane& localPlane, ndVector* const contactsOut) const;

	virtual void GetFacesPatch(ndPatchMesh& patch) const = 0;

	D_MSV_NEWTON_CLASS_ALIGN_32 
	class ndMeshVertexListIndexList
	{
		public:
		ndInt32* m_indexList;
		ndInt32* m_userDataList;
		ndFloat32* m_veterxArray;
		ndInt32 m_triangleCount; 
		ndInt32 m_maxIndexCount;
		ndInt32 m_vertexCount;
		ndInt32 m_vertexStrideInBytes;
	} D_GCC_NEWTON_CLASS_ALIGN_32;

	friend class ndContactSolver;
} D_GCC_NEWTON_CLASS_ALIGN_32;


#endif 



