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

#ifndef __ND_COLLISION_CONVEX_POLYGON_H__
#define __ND_COLLISION_CONVEX_POLYGON_H__

#include "ndShapeConvex.h"

#define D_CONVEX_POLYGON_MAX_VERTEX_COUNT	64

class ndContactSolver;
class ndShapeInstance;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndShapeConvexPolygon: public ndShapeConvex
{
	public:
	class dgClippedFaceEdge
	{
		public:
		dgClippedFaceEdge* m_next;
		dgClippedFaceEdge* m_twin;
		ndInt32 m_incidentNormal;
		ndInt32 m_incidentVertex;
	};

	class ndEdge
	{
		public:
		union
		{
			struct
			{
				ndInt16 m_low;
				ndInt16 m_high;
			};
			ndInt32 m_edge;
		};
		ndInt32 m_faceStart;
	};

	ndShapeConvexPolygon ();
	~ndShapeConvexPolygon ();

	virtual ndShapeConvexPolygon* GetAsShapeConvexPolygon();

	ndVector CalculateGlobalNormal(const ndVector& localNormal) const;
	ndInt32 CalculateContactToConvexHullDescrete(ndContactSolver& proxy);
	ndInt32 CalculateContactToConvexHullContinue(ndContactSolver& proxy);

	void GenerateConvexCap();
	virtual ndFloat32 GetVolume() const;
	virtual ndFloat32 GetBoxMinRadius() const;
	virtual ndFloat32 GetBoxMaxRadius() const;
	virtual ndVector SupportVertex(const ndVector& dir) const;
	bool BeamClipping(const ndVector& origin, ndFloat32 size);
	virtual ndInt32 CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const;

	virtual ndFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;

	virtual ndInt32 Release() const;

	ndVector m_normal;
	ndFixSizeArray<ndVector, D_CONVEX_POLYGON_MAX_VERTEX_COUNT> m_localPoly;
	ndFixSizeArray<ndInt32, 4 * D_CONVEX_POLYGON_MAX_VERTEX_COUNT> m_convexCapFace;
	ndFixSizeArray<ndInt32, 8 * D_CONVEX_POLYGON_MAX_VERTEX_COUNT> m_convexCapFaceIndex;
	ndFixSizeArray<ndEdge, 64> m_adjancentEdge;
	ndFloat32 m_faceClipSize;
	ndInt32 m_faceId;
	ndInt32 m_faceNormalIndex;

	const ndShapeInstance* m_owner;
	const ndVector* m_vertexArray;
	const ndInt32* m_vertexIndex;
	const ndInt32* m_adjacentFaceEdgeNormalIndex;
} D_GCC_NEWTON_CLASS_ALIGN_32;

inline ndShapeConvexPolygon* ndShapeConvexPolygon::GetAsShapeConvexPolygon()
{
	return this; 
}

inline ndFloat32 ndShapeConvexPolygon::RayCast(ndRayCastNotify&, const ndVector&, const ndVector&, ndFloat32, const ndBody* const, ndContactPoint&) const
{
	return ndFloat32(1.2f);
}

inline ndFloat32 ndShapeConvexPolygon::GetVolume() const
{
	return ndFloat32(0.0f);
}

inline ndFloat32 ndShapeConvexPolygon::GetBoxMinRadius() const
{
	return m_faceClipSize;
}

inline ndFloat32 ndShapeConvexPolygon::GetBoxMaxRadius() const
{
	return GetBoxMinRadius();
}

#endif

