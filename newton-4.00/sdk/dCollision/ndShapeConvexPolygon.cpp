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
#include "ndShapeInstance.h"
#include "ndContactSolver.h"
#include "ndBodyKinematic.h"
#include "ndShapeConvexPolygon.h"

//#define D_CONVEX_POLYGON_SKIRT_LENGTH ndFloat32 (0.025f)
#define D_CONVEX_POLYGON_SKIRT_LENGTH ndFloat32 (0.25f)

ndShapeConvexPolygon::ndShapeConvexPolygon ()
	:ndShapeConvex(m_polygonCollision)
	,m_faceClipSize(0)
	,m_faceNormalIndex(0)
	,m_vertexArray(nullptr)
	,m_vertexIndex(nullptr)
	,m_adjacentFaceEdgeNormalIndex(nullptr)
{
}

ndShapeConvexPolygon::~ndShapeConvexPolygon ()
{
}

ndInt32 ndShapeConvexPolygon::Release() const
{
	ndInt32 count = m_refCount.fetch_add(-1);
	return count;
}

ndVector ndShapeConvexPolygon::SupportVertex(const ndVector& dir) const
{
	ndAssert(ndAbs(dir.m_w) == ndFloat32(0.0f));
	ndAssert(ndAbs(dir.DotProduct(dir).GetScalar() - 1.0f) < ndFloat32(1.0e-2f));
	
	ndInt32 index = m_localPoly.GetCount() - 1;
	ndFloat32 val = m_localPoly[index].DotProduct(dir).GetScalar();
	for (ndInt32 i = m_localPoly.GetCount() - 2; i >= 0; --i)
	{
		ndFloat32 val1 = m_localPoly[i].DotProduct(dir).GetScalar();
		if (val1 > val) 
		{
			val = val1;
			index = i;
		}
	}
	//ndAssert(vertexIndex == nullptr);
	return m_localPoly[index];
}

ndVector ndShapeConvexPolygon::CalculateGlobalNormal(const ndVector& localNormal) const
{
	ndAssert(m_owner);
	const ndShapeInstance* const parentMesh = m_owner;
	const ndVector& invScale = parentMesh->GetInvScale();
	const ndMatrix& globalMatrix = parentMesh->GetGlobalMatrix();
	const ndMatrix& alignmentMatrix = parentMesh->GetAlignmentMatrix();

	ndVector normal(alignmentMatrix.RotateVector(localNormal));
	normal = normal * invScale;
	ndAssert(normal.m_w == ndFloat32(0.0f));
	normal = normal.Normalize();
	return globalMatrix.RotateVector(normal);
}

void ndShapeConvexPolygon::GenerateConvexCap()
{
	const ndInt32 count = m_localPoly.GetCount();
	const ndVector skirt(ndVector::m_triplexMask & ndVector(D_CONVEX_POLYGON_SKIRT_LENGTH));
	ndInt32 faceSum = m_convexCapFace[0];
	m_convexCapFace[0] = 0;

	ndInt32 i0 = count - 1;
	m_adjancentEdge.SetCount(0);
	for (ndInt32 i = 0; i < count; ++i)
	{
		if (m_adjacentFaceEdgeNormalIndex[i0] > 0)
		{
			const ndVector faceEddge(m_localPoly[i] - m_localPoly[i0]);
			ndAssert(faceEddge.m_w == ndFloat32(0.0f));
			ndAssert(faceEddge.DotProduct(faceEddge).GetScalar() > ndFloat32(0.0f));
			const ndVector edge(faceEddge.Normalize());
			const ndInt32 adjacentNormalIndex = m_adjacentFaceEdgeNormalIndex[i0] & (~D_CONCAVE_EDGE_MASK);
			const ndVector localAdjacentNormal(m_vertexArray[adjacentNormalIndex]);
			const ndVector adjacentNormal(CalculateGlobalNormal(localAdjacentNormal & ndVector::m_triplexMask));
			ndAssert(edge.DotProduct(adjacentNormal).GetScalar() < ndFloat32(2.0e-1f));

			const ndVector edgeSkirt(edge.CrossProduct(adjacentNormal) * skirt);

			m_localPoly.PushBack(m_localPoly[i] + edgeSkirt);
			m_localPoly.PushBack(m_localPoly[i0] + edgeSkirt);

			ndEdge adjacentEdge;
			adjacentEdge.m_low = ndInt16(i0);
			adjacentEdge.m_high = ndInt16(i);
			adjacentEdge.m_faceStart = faceSum;
			m_adjancentEdge.PushBack(adjacentEdge);

			m_convexCapFace.PushBack(faceSum);
			m_convexCapFaceIndex.PushBack(i);
			m_convexCapFaceIndex.PushBack(i0);
			m_convexCapFaceIndex.PushBack(m_localPoly.GetCount() - 1);
			m_convexCapFaceIndex.PushBack(m_localPoly.GetCount() - 2);
			faceSum += 4;
		}
		i0 = i;
	}
	m_convexCapFace.PushBack(faceSum);
}

bool ndShapeConvexPolygon::BeamClipping(const ndVector& origin, ndFloat32 dist)
{
	ndPlane planes[4];
	ndFixSizeArray<ndVector, 2 * D_CONVEX_POLYGON_MAX_VERTEX_COUNT> points(2 * D_CONVEX_POLYGON_MAX_VERTEX_COUNT);

	dgClippedFaceEdge clippedFace[2 * sizeof(m_localPoly) / sizeof(m_localPoly[0]) + 8];

	ndVector dir(m_localPoly[1] - m_localPoly[0]);
	ndAssert(dir.m_w == ndFloat32(0.0f));
	ndAssert(dir.DotProduct(dir).GetScalar() > ndFloat32(1.0e-8f));
	dir = dir.Normalize();

	ndFloat32 distH = origin.DotProduct(dir).GetScalar();
	planes[0] = ndPlane(dir, dist - distH);
	planes[2] = ndPlane(dir * ndVector::m_negOne, dist + distH);

	dir = m_normal.CrossProduct(dir);
	ndFloat32 distV = origin.DotProduct(dir).GetScalar();
	planes[1] = ndPlane(dir, dist - distV);
	planes[3] = ndPlane(dir * ndVector::m_negOne, dist + distV);

	const ndInt32 count = m_localPoly.GetCount();
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndInt32 j = i << 1;
		ndAssert(j < ndInt32 (sizeof(clippedFace) / sizeof(clippedFace[0])));

		points[i] = m_localPoly[i];

		clippedFace[j + 0].m_twin = &clippedFace[j + 1];
		clippedFace[j + 0].m_next = &clippedFace[j + 2];
		clippedFace[j + 0].m_incidentVertex = i;
		clippedFace[j + 0].m_incidentNormal = m_adjacentFaceEdgeNormalIndex[i] & (~D_CONCAVE_EDGE_MASK);

		clippedFace[j + 1].m_twin = &clippedFace[j + 0];
		clippedFace[j + 1].m_next = &clippedFace[j - 2];
		clippedFace[j + 1].m_incidentVertex = i + 1;
		clippedFace[j + 1].m_incidentNormal = -1;
	}

	clippedFace[1].m_next = &clippedFace[count * 2 - 2 + 1];
	ndAssert((count * 2 - 2) >= 0);
	clippedFace[count * 2 - 2].m_next = &clippedFace[0];
	clippedFace[count * 2 - 2 + 1].m_incidentVertex = 0;

	ndInt32 indexCount = count;
	ndInt32 edgeCount = count * 2;
	dgClippedFaceEdge* first = &clippedFace[0];

	const ndFloat32 tol = ndFloat32(1.0e-5f);
	for (ndInt32 i = 0; i < 4; ++i) 
	{
		const ndPlane& plane = planes[i];

		ndInt32 conectCount = 0;
		dgClippedFaceEdge* connect[2];
		dgClippedFaceEdge* ptr = first;
		dgClippedFaceEdge* newFirst = first;
		ndFloat32 test0 = plane.Evalue(points[ptr->m_incidentVertex]);
		do 
		{
			ndFloat32 test1 = plane.Evalue(points[ptr->m_next->m_incidentVertex]);

			if (test0 > tol) 
			{
				if (test1 <= -tol) 
				{
					const ndVector& p0 = points[ptr->m_incidentVertex];
					const ndVector& p1 = points[ptr->m_next->m_incidentVertex];
					const ndVector dp(p1 - p0);

					points[indexCount] = p0 - dp.Scale(test0 / dp.DotProduct(plane).GetScalar());

					dgClippedFaceEdge* const newEdge = &clippedFace[edgeCount];
					newEdge->m_twin = newEdge + 1;
					newEdge->m_twin->m_twin = newEdge;

					newEdge->m_twin->m_incidentNormal = ptr->m_incidentNormal;
					newEdge->m_incidentNormal = ptr->m_incidentNormal;

					newEdge->m_incidentVertex = indexCount;
					newEdge->m_twin->m_incidentVertex = ptr->m_next->m_incidentVertex;
					ptr->m_twin->m_incidentVertex = indexCount;

					newEdge->m_next = ptr->m_next;
					ptr->m_next->m_twin->m_next = newEdge->m_twin;
					newEdge->m_twin->m_next = ptr->m_twin;
					ptr->m_next = newEdge;

					connect[conectCount] = ptr;
					conectCount++;
					indexCount++;
					edgeCount += 2;
					ptr = newEdge;
				}
			}
			else 
			{
				if ((test1 > tol) && (test0 * test1) < ndFloat32(0.0f)) 
				{
					newFirst = ptr->m_next;
					const ndVector& p0 = points[ptr->m_incidentVertex];
					const ndVector& p1 = points[ptr->m_next->m_incidentVertex];
					const ndVector dp(p1 - p0);
					points[indexCount] = p0 - dp.Scale(test0 / dp.DotProduct(plane).GetScalar());

					dgClippedFaceEdge* const newEdge = &clippedFace[edgeCount];
					newEdge->m_twin = newEdge + 1;
					newEdge->m_twin->m_twin = newEdge;

					newEdge->m_twin->m_incidentNormal = ptr->m_incidentNormal;
					newEdge->m_incidentNormal = ptr->m_incidentNormal;

					newEdge->m_incidentVertex = indexCount;
					newEdge->m_twin->m_incidentVertex = ptr->m_next->m_incidentVertex;
					ptr->m_twin->m_incidentVertex = indexCount;

					newEdge->m_next = ptr->m_next;
					ptr->m_next->m_twin->m_next = newEdge->m_twin;
					newEdge->m_twin->m_next = ptr->m_twin;
					ptr->m_next = newEdge;

					connect[conectCount] = ptr;
					conectCount++;
					indexCount++;
					edgeCount += 2;

					ptr = newEdge;
				}
			}

			test0 = test1;
			ptr = ptr->m_next;
		} while (ptr != first);

		if (conectCount > 1) 
		{
			first = newFirst;
			ndAssert(conectCount == 2);

			dgClippedFaceEdge* const newEdge = &clippedFace[edgeCount];
			newEdge->m_twin = newEdge + 1;
			newEdge->m_twin->m_twin = newEdge;

			newEdge->m_incidentNormal = m_faceNormalIndex;
			newEdge->m_incidentVertex = connect[0]->m_next->m_incidentVertex;
			newEdge->m_twin->m_next = connect[0]->m_next;
			connect[0]->m_next = newEdge;

			newEdge->m_twin->m_incidentNormal = m_faceNormalIndex;
			newEdge->m_twin->m_incidentVertex = connect[1]->m_next->m_incidentVertex;
			newEdge->m_next = connect[1]->m_next;
			connect[1]->m_next = newEdge->m_twin;

			edgeCount += 2;
		}
	}

	dgClippedFaceEdge* ptr = first;
	do 
	{
		ndVector dist1(points[ptr->m_next->m_incidentVertex] - points[ptr->m_incidentVertex]);
		ndAssert(dist1.m_w == ndFloat32(0.0f));
		ndFloat32 error = dist1.DotProduct(dist1).GetScalar();
		if (error < ndFloat32(1.0e-6f)) 
		{
			ptr->m_next = ptr->m_next->m_next;
			first = ptr;
		}
		ptr = ptr->m_next;
	} while (ptr != first);

	//ndInt32 count = 0;
	//m_adjacentFaceEdgeNormalIndex = &m_clippEdgeNormal[0];
	m_localPoly.SetCount(0);
	do 
	{
		//m_clippEdgeNormal[count] = ptr->m_incidentNormal;
		//m_localPoly[count] = points[ptr->m_incidentVertex];
		m_localPoly.PushBack(points[ptr->m_incidentVertex]);
		ptr = ptr->m_next;
	} while (ptr != first);

	if (m_localPoly.GetCount() >= 3)
	{
		GenerateConvexCap();
	}
	return (m_localPoly.GetCount() >= 3);
}

ndInt32 ndShapeConvexPolygon::CalculateContactToConvexHullContinue(ndContactSolver& contactSolver)
{
	ndAssert(m_localPoly.GetCount());
	ndAssert(this == contactSolver.m_instance1.GetShape());
	ndAssert(contactSolver.m_instance0.GetShape()->GetAsShapeConvex());
	ndAssert(contactSolver.m_instance1.GetShape()->GetAsShapeConvexPolygon());
	ndAssert(contactSolver.m_instance1.GetGlobalMatrix().TestIdentity());

	const ndBodyKinematic* const body0 = contactSolver.m_contact->m_body0;
	const ndBodyKinematic* const body1 = contactSolver.m_contact->m_body1;

	ndVector relativeVelocity(body0->GetVelocity() - body1->GetVelocity());
	ndAssert(relativeVelocity.m_w == ndFloat32(0.0f));
	if (relativeVelocity.DotProduct(relativeVelocity).GetScalar() < ndFloat32(1.0e-4f))
	{
		return 0;
	}

	ndFloat32 den = m_normal.DotProduct(relativeVelocity).GetScalar();
	if (den > ndFloat32(-1.0e-10f))
	{
		return 0;
	}

	ndMatrix polygonMatrix;
	ndVector right(m_localPoly[1] - m_localPoly[0]);
	polygonMatrix[0] = right.Normalize();
	polygonMatrix[1] = m_normal;
	polygonMatrix[2] = polygonMatrix[0].CrossProduct(m_normal);
	polygonMatrix[3] = m_localPoly[0];
	polygonMatrix[3].m_w = ndFloat32(1.0f);
	ndAssert(polygonMatrix.TestOrthogonal());

	ndVector polyBoxP0(ndFloat32(1.0e15f));
	ndVector polyBoxP1(ndFloat32(-1.0e15f));
	for (ndInt32 i = m_localPoly.GetCount() - 1; i >= 0; --i)
	{
		const ndVector point(polygonMatrix.UntransformVector(m_localPoly[i]));
		polyBoxP0 = polyBoxP0.GetMin(point);
		polyBoxP1 = polyBoxP1.GetMax(point);
	}

	ndVector hullBoxP0;
	ndVector hullBoxP1;
	ndMatrix hullMatrix(contactSolver.m_instance0.m_globalMatrix * polygonMatrix.OrthoInverse());
	contactSolver.m_instance0.CalculateAabb(hullMatrix, hullBoxP0, hullBoxP1);
	ndVector minBox(polyBoxP0 - hullBoxP1);
	ndVector maxBox(polyBoxP1 - hullBoxP0);

	ndVector relStep(relativeVelocity.Scale(ndMax(contactSolver.m_timestep, ndFloat32(1.0e-12f))));
	ndFastRay ray(ndVector::m_zero, polygonMatrix.UnrotateVector(relStep));
	ndFloat32 distance = ray.BoxIntersect(minBox, maxBox);

	ndInt32 count = 0;
	ndFloat32 relStepSpeed = m_normal.DotProduct(relStep).GetScalar();
	if ((distance < ndFloat32(1.0f)) && (ndAbs(relStepSpeed) > ndFloat32(1.0e-12f)))
	{
		bool inside = false;
		ndAssert(m_normal.DotProduct(relStep).GetScalar() == relStepSpeed);
		ndFloat32 invSpeed = ndFloat32(1.0f) / relStepSpeed;
		ndVector sphOrigin(polygonMatrix.TransformVector((hullBoxP1 + hullBoxP0) * ndVector::m_half));
		ndVector pointInPlane(sphOrigin - relStep.Scale(m_normal.DotProduct(sphOrigin - m_localPoly[0]).GetScalar() * invSpeed));

		ndVector sphRadius(ndVector::m_half * (hullBoxP1 - hullBoxP0));
		ndFloat32 radius = ndSqrt(sphRadius.DotProduct(sphRadius).GetScalar());
		ndVector planeMinkStep(m_normal.Scale(radius));
		sphOrigin -= planeMinkStep;
		ndAssert(m_normal.DotProduct(relStep).GetScalar() == relStepSpeed);
		ndVector supportPoint(sphOrigin - relStep.Scale(m_normal.DotProduct(sphOrigin - m_localPoly[0]).GetScalar() * invSpeed));

		supportPoint -= pointInPlane;
		ndAssert(supportPoint.m_w == ndFloat32(0.0f));
		radius = ndMax(ndSqrt(supportPoint.DotProduct(supportPoint).GetScalar()), radius);

		inside = true;
		const ndInt32 vertexCount = m_localPoly.GetCount();
		ndInt32 i0 = vertexCount - 1;
		for (ndInt32 i = 0; i < vertexCount; ++i)
		{
			const ndVector e(m_localPoly[i] - m_localPoly[i0]);
			const ndVector n((e.CrossProduct(m_normal) & ndVector::m_triplexMask).Normalize());
			ndFloat32 dist1 = n.DotProduct(pointInPlane - m_localPoly[i0]).GetScalar();

			if (dist1 > radius)
			{
				return 0;
			}
			inside &= (dist1 <= ndFloat32(0.0f));
			i0 = i;
		}

		ndFloat32 convexSphapeUmbra = ndMax(contactSolver.m_instance0.GetUmbraClipSize(), radius);
		if (m_faceClipSize > convexSphapeUmbra)
		{
			BeamClipping(pointInPlane, convexSphapeUmbra);
			m_faceClipSize = contactSolver.m_instance0.GetShape()->GetBoxMaxRadius();
		}

		const ndUnsigned64 hullId = contactSolver.m_instance0.GetUserDataID();
		if (inside && !contactSolver.m_intersectionTestOnly)
		{
			const ndMatrix& matrixInstance0 = contactSolver.m_instance0.m_globalMatrix;
			const ndVector normalInHull(matrixInstance0.UnrotateVector(m_normal * ndVector::m_negOne));
			ndVector pointInHull(contactSolver.m_instance0.SupportVertex(normalInHull));
			const ndVector p0(matrixInstance0.TransformVector(pointInHull));

			ndFloat32 timetoImpact = ndFloat32(0.0f);
			ndAssert(m_normal.m_w == ndFloat32(0.0f));
			ndFloat32 penetration = m_normal.DotProduct(m_localPoly[0] - p0).GetScalar() + contactSolver.m_skinMargin;
			if (penetration < ndFloat32(0.0f))
			{
				timetoImpact = penetration / relativeVelocity.DotProduct(m_normal).GetScalar();
				ndAssert(timetoImpact >= ndFloat32(0.0f));
			}

			if (timetoImpact <= contactSolver.m_timestep)
			{
				ndVector contactPoints[64];
				contactSolver.m_timestep = timetoImpact;
				contactSolver.m_separatingVector = m_normal;
				contactSolver.m_closestPoint0 = p0;
				contactSolver.m_closestPoint1 = p0 + m_normal.Scale(penetration);

				if (!contactSolver.m_intersectionTestOnly)
				{
					pointInHull -= normalInHull.Scale(D_PENETRATION_TOL);
					count = contactSolver.m_instance0.CalculatePlaneIntersection(normalInHull, pointInHull, contactPoints);

					ndVector step(relativeVelocity.Scale(timetoImpact));
					penetration = ndMax(penetration, ndFloat32(0.0f));
					ndContactPoint* const contactsOut = contactSolver.m_contactBuffer;
					for (ndInt32 i = 0; i < count; ++i)
					{
						contactsOut[i].m_point = matrixInstance0.TransformVector(contactPoints[i]) + step;
						contactsOut[i].m_normal = m_normal;
						contactsOut[i].m_shapeId0 = ndInt64(hullId);
						contactsOut[i].m_shapeId1 = m_faceId;
						contactsOut[i].m_penetration = penetration;
					}
				}
			}
		}
		else
		{
			GenerateConvexCap();
			m_vertexCount = ndUnsigned16(vertexCount);
			count = contactSolver.ConvexToConvexContactsContinue();
			if (count >= 1)
			{
				ndContactPoint* const contactsOut = contactSolver.m_contactBuffer;
				for (ndInt32 i = 0; i < count; ++i)
				{
					contactsOut[i].m_shapeId0 = ndInt64(hullId);
					contactsOut[i].m_shapeId1 = m_faceId;
				}
			}
		}
	}
	return count;
}

ndInt32 ndShapeConvexPolygon::CalculateContactToConvexHullDescrete(ndContactSolver& contactSolver)
{
	ndAssert(m_localPoly.GetCount());
	ndAssert(this == contactSolver.m_instance1.GetShape());
	ndAssert(contactSolver.m_instance0.GetShape()->GetAsShapeConvex());
	ndAssert(contactSolver.m_instance1.GetShape()->GetAsShapeConvexPolygon());
	ndAssert(contactSolver.m_instance1.GetGlobalMatrix().TestIdentity());
	ndAssert(contactSolver.m_instance1.GetGlobalMatrix().TestIdentity());

	const ndMatrix& hullMatrix = contactSolver.m_instance0.m_globalMatrix;
	const ndShapeInstance* const hull = &contactSolver.m_instance0;

	ndAssert(m_normal.m_w == ndFloat32(0.0f));
	const ndVector obbOrigin(hullMatrix.TransformVector(contactSolver.m_instance0.GetShape()->GetObbOrigin()));
	const ndFloat32 shapeSide = m_normal.DotProduct(obbOrigin - m_localPoly[0]).GetScalar();
	if (shapeSide < ndFloat32(0.0f))
	{
		return 0;
	}

	const ndVector normalInHull(hullMatrix.UnrotateVector(m_normal));
	const ndVector pointInHull(hull->SupportVertex(normalInHull.Scale(ndFloat32(-1.0f))));
	const ndVector p0(hullMatrix.TransformVector(pointInHull));

	ndFloat32 penetration = m_normal.DotProduct(m_localPoly[0] - p0).GetScalar() + contactSolver.m_skinMargin;
	if (penetration < -(D_PENETRATION_TOL * ndFloat32(5.0f)))
	{
		contactSolver.m_separatingVector = m_normal;
		contactSolver.m_closestPoint0 = p0;
		contactSolver.m_closestPoint1 = p0 + m_normal.Scale(penetration);
		contactSolver.m_separationDistance = -penetration;
		return 0;
	}

	const ndVector p1(hullMatrix.TransformVector(hull->SupportVertex(normalInHull)));

	ndFloat32 distance = m_normal.DotProduct(m_localPoly[0] - p1).GetScalar();
	if (distance >= ndFloat32(0.0f))
	{
		return 0;
	}

	ndVector boxSize;
	ndVector boxOrigin;
	hull->CalculateObb(boxOrigin, boxSize);
	ndAssert(boxOrigin.m_w == ndFloat32(0.0f));
	boxOrigin += ndVector::m_wOne;

	bool inside = true;
	const ndInt32 vertexCount = m_localPoly.GetCount();
	ndInt32 i0 = vertexCount - 1;
	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		ndVector e(m_localPoly[i] - m_localPoly[i0]);
		ndVector edgeBoundaryNormal(m_normal.CrossProduct(e));
		ndAssert(edgeBoundaryNormal.m_w == ndFloat32(0.0f));
		ndPlane plane(edgeBoundaryNormal, -m_localPoly[i0].DotProduct(edgeBoundaryNormal).GetScalar());
		plane = hullMatrix.UntransformPlane(plane);

		ndFloat32 supportDist = boxSize.DotProduct(plane.Abs()).GetScalar();
		ndFloat32 centerDist = plane.DotProduct(boxOrigin).GetScalar();

		if ((centerDist + supportDist) < ndFloat32(0.0f))
		{
			return 0;
		}

		if ((centerDist - supportDist) < ndFloat32(0.0f))
		{
			inside = false;
			break;
		}
		i0 = i;
	}

	bool needSkirts = true;
	ndFloat32 convexSphapeUmbra = hull->GetUmbraClipSize();
	if (m_faceClipSize > convexSphapeUmbra)
	{
		ndVector boxP0;
		ndVector boxP1;
		hull->CalculateAabb(hullMatrix, boxP0, boxP1);
		ndVector origin(ndVector::m_half * (boxP1 + boxP1));

		if (!BeamClipping(origin, convexSphapeUmbra))
		{
			return 0;
		}
		needSkirts = false;
		m_faceClipSize = hull->GetShape()->GetBoxMaxRadius();
	}

	ndInt32 count = 0;
	const ndUnsigned64 hullId = hull->GetUserDataID();
	if (inside && !contactSolver.m_intersectionTestOnly)
	{
		contactSolver.m_separationDistance = -penetration;
		contactSolver.m_separatingVector = m_normal;
		contactSolver.m_closestPoint0 = p0;
		contactSolver.m_closestPoint1 = p0 + m_normal.Scale(penetration);

		penetration = ndMax(ndFloat32(0.0f), penetration);
		ndAssert(penetration >= ndFloat32(0.0f));
		ndVector contactPoints[128];
		ndVector point(pointInHull + normalInHull.Scale(penetration - D_PENETRATION_TOL));

		count = hull->CalculatePlaneIntersection(normalInHull.Scale(ndFloat32(-1.0f)), point, contactPoints);
		ndVector step(normalInHull.Scale((contactSolver.m_skinMargin - penetration) * ndFloat32(0.5f)));

		ndContactPoint* const contactsOut = contactSolver.m_contactBuffer;
		for (ndInt32 i = 0; i < count; ++i)
		{
			contactsOut[i].m_point = hullMatrix.TransformVector(contactPoints[i] + step);
			contactsOut[i].m_normal = m_normal;
			contactsOut[i].m_shapeId0 = ndInt64(hullId);
			contactsOut[i].m_shapeId1 = m_faceId;
			contactsOut[i].m_penetration = penetration;
		}
	}
	else
	{
		if (needSkirts)
		{
			GenerateConvexCap();
		}
		m_vertexCount = ndUnsigned16(vertexCount);
		count = contactSolver.ConvexToConvexContactsDiscrete();
		ndAssert(contactSolver.m_intersectionTestOnly || (count >= 0));
		if (count >= 1)
		{
			ndContactPoint* const contactsOut = contactSolver.m_contactBuffer;
			for (ndInt32 i = 0; i < count; ++i)
			{
				contactsOut[i].m_shapeId0 = ndInt64(hullId);
				contactsOut[i].m_shapeId1 = m_faceId;
			}
		}
	}
	return count;
}

ndInt32 ndShapeConvexPolygon::CalculatePlaneIntersection(const ndVector& normalIn, const ndVector& origin, ndVector* const contactsOut) const
{
#if 0
	ndAssert(normalIn.m_w == ndFloat32(0.0f));
	ndVector normal(normalIn);
	ndFloat32 maxDist = ndFloat32(1.0f);
	ndFloat32 projectFactor = m_normal.DotProduct(normal).GetScalar();
	if (projectFactor < ndFloat32(0.0f))
	{
		projectFactor *= ndFloat32(-1.0f);
		normal = normal * ndVector::m_negOne;
	}

	ndInt32 count = 0;
	const ndInt32 vertexCount = m_convexCapFace[1];
	if (projectFactor > ndFloat32(0.9999f))
	{
		for (ndInt32 i = 0; i < vertexCount; ++i)
		{
			contactsOut[count] = m_localPoly[i];
			count++;
		}

		#ifdef _DEBUG
			ndInt32 j = count - 1;
			for (ndInt32 i = 0; i < count; ++i)
			{
				ndVector error(contactsOut[i] - contactsOut[j]);
				ndAssert(error.m_w == ndFloat32(0.0f));
				ndAssert(error.DotProduct(error).GetScalar() > ndFloat32(1.0e-20f));
				j = i;
			}
		#endif
	}
	else if (projectFactor > ndFloat32(0.1736f))
	{
		maxDist = ndFloat32(0.0f);
		ndPlane plane(normal, -normal.DotProduct(origin).GetScalar());

		ndVector p0(m_localPoly[vertexCount - 1]);
		ndFloat32 side0 = plane.Evalue(p0);
		for (ndInt32 i = 0; i < vertexCount; ++i)
		{
			ndVector p1(m_localPoly[i]);
			ndFloat32 side1 = plane.Evalue(p1);

			if (side0 > ndFloat32(0.0f))
			{
				maxDist = ndMax(maxDist, side0);
				contactsOut[count] = p0 - normal.Scale(side0);
				count++;
				if (count > 1)
				{
					ndVector edgeSegment(contactsOut[count - 1] - contactsOut[count - 2]);
					ndAssert(edgeSegment.m_w == ndFloat32(0.0f));
					ndFloat32 error = edgeSegment.DotProduct(edgeSegment).GetScalar();
					if (error < ndFloat32(1.0e-8f)) {
						count--;
					}
				}

				if (side1 <= ndFloat32(0.0f))
				{
					ndVector dp(p1 - p0);
					ndFloat32 t = normal.DotProduct(dp).GetScalar();
					ndAssert(ndAbs(t) >= ndFloat32(0.0f));
					if (ndAbs(t) < ndFloat32(1.0e-8f))
					{
						t = ndSign(t) * ndFloat32(1.0e-8f);
					}
					contactsOut[count] = p0 - dp.Scale(side0 / t);
					count++;
					if (count > 1)
					{
						ndVector edgeSegment(contactsOut[count - 1] - contactsOut[count - 2]);
						ndAssert(edgeSegment.m_w == ndFloat32(0.0f));
						ndFloat32 error = edgeSegment.DotProduct(edgeSegment).GetScalar();
						if (error < ndFloat32(1.0e-8f))
						{
							count--;
						}
					}
				}
			}
			else if (side1 > ndFloat32(0.0f))
			{
				ndVector dp(p1 - p0);
				ndFloat32 t = normal.DotProduct(dp).GetScalar();
				ndAssert(ndAbs(t) >= ndFloat32(0.0f));
				if (ndAbs(t) < ndFloat32(1.0e-8f))
				{
					t = ndSign(t) * ndFloat32(1.0e-8f);
				}
				contactsOut[count] = p0 - dp.Scale(side0 / t);
				count++;
				if (count > 1)
				{
					ndVector edgeSegment(contactsOut[count - 1] - contactsOut[count - 2]);
					ndAssert(edgeSegment.m_w == ndFloat32(0.0f));
					ndFloat32 error = edgeSegment.DotProduct(edgeSegment).GetScalar();
					if (error < ndFloat32(1.0e-8f))
					{
						count--;
					}
				}
			}

			side0 = side1;
			p0 = p1;
		}
	}
	else
	{
		maxDist = ndFloat32(1.0e10f);
		ndPlane plane(normal, -normal.DotProduct(origin).GetScalar());

		ndVector p0(m_localPoly[vertexCount - 1]);
		ndFloat32 side0 = plane.Evalue(p0);
		for (ndInt32 i = 0; i < vertexCount; ++i)
		{
			ndVector p1(m_localPoly[i]);
			ndFloat32 side1 = plane.Evalue(p1);

			if ((side0 * side1) < ndFloat32(0.0f))
			{
				ndVector dp(p1 - p0);
				ndFloat32 t = normal.DotProduct(dp).GetScalar();
				ndAssert(ndAbs(t) >= ndFloat32(0.0f));
				if (ndAbs(t) < ndFloat32(1.0e-8f))
				{
					t = ndSign(t) * ndFloat32(1.0e-8f);
				}
				contactsOut[count] = p0 - dp.Scale(side0 / t);
				count++;
				if (count > 1)
				{
					ndVector edgeSegment(contactsOut[count - 1] - contactsOut[count - 2]);
					ndAssert(edgeSegment.m_w == ndFloat32(0.0f));
					ndFloat32 error = edgeSegment.DotProduct(edgeSegment).GetScalar();
					if (error < ndFloat32(1.0e-8f))
					{
						count--;
					}
				}
			}
			side0 = side1;
			p0 = p1;
		}
	}

	if (count > 1)
	{
		if (maxDist < ndFloat32(1.0e-3f))
		{
			ndVector maxPoint(contactsOut[0]);
			ndVector minPoint(contactsOut[0]);
			ndVector lineDir(m_normal.CrossProduct(normal));

			ndAssert(lineDir.m_w == ndFloat32(0.0f));
			ndFloat32 proj = contactsOut[0].DotProduct(lineDir).GetScalar();
			ndFloat32 maxProjection = proj;
			ndFloat32 minProjection = proj;
			for (ndInt32 i = 1; i < count; ++i)
			{
				proj = contactsOut[i].DotProduct(lineDir).GetScalar();
				if (proj > maxProjection)
				{
					maxProjection = proj;
					maxPoint = contactsOut[i];
				}
				if (proj < minProjection)
				{
					minProjection = proj;
					minPoint = contactsOut[i];
				}
			}

			contactsOut[0] = maxPoint;
			contactsOut[1] = minPoint;
			count = 2;
		}

		ndVector error(contactsOut[count - 1] - contactsOut[0]);
		ndAssert(error.m_w == ndFloat32(0.0f));
		if (error.DotProduct(error).GetScalar() < ndFloat32(1.0e-8f))
		{
			count--;
		}
	}
	return count;

#else

	// first check if this minimun translation is a face contact
	const ndVector faceOrigin(m_localPoly[0]);
	const ndVector e0(m_localPoly[1] - faceOrigin);
	const ndVector e1(m_localPoly[2] - faceOrigin);
	const ndVector faceNormal(e0.CrossProduct(e1));
	ndAssert(faceNormal.m_w == ndFloat32(0.0f));
	ndAssert(faceNormal.DotProduct(faceNormal).GetScalar() > ndFloat32(0.0f));
	const ndVector unitFaceNormal(faceNormal.Normalize());

	const ndVector step(origin - faceOrigin);
	const ndVector pointInFacePlane(origin - unitFaceNormal.Scale(unitFaceNormal.DotProduct(step).GetScalar()));

	bool isInsideFace = true;
	const ndInt32 vertexCount = m_convexCapFace[1];
	ndInt32 i0 = vertexCount - 1;
	ndVector p0(m_localPoly[i0]);

	ndEdge adjacentEdge;
	adjacentEdge.m_edge = -1;
	adjacentEdge.m_faceStart = -1;
	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		const ndVector& p1 = m_localPoly[i];
		ndVector sideDir(unitFaceNormal.CrossProduct(p1 - p0).Normalize());
		ndAssert(sideDir.m_w == ndFloat32(0.0f));
		ndFloat32 dist = sideDir.DotProduct(pointInFacePlane - p0).GetScalar();
		if (dist < ndFloat32(-1.0e-3f))
		{
			isInsideFace = false;
		}
		if (dist < ndFloat32(0.0f))
		{
			adjacentEdge.m_low = ndInt16(i0);
			adjacentEdge.m_high = ndInt16(i);
		}
		i0 = i;
		p0 = p1;
	}

	if (isInsideFace)
	{
		// translation is inside polygon
		// we now check if this it a face contact
		ndFloat32 cosDir = normalIn.DotProduct(unitFaceNormal).GetScalar();
		if (cosDir >= ndFloat32(0.9994f))
		{
			// if the normals are aligned within 2 degree, 
			// we consider this a face contact 
			for (ndInt32 i = 0; i < vertexCount; ++i)
			{
				contactsOut[i] = m_localPoly[i];
			}
			return vertexCount;
		}
		else
		{
			// handle edge contact
			ndInt32 closestEdge = -1;
			ndFloat32 param = ndFloat32(0.0);
			ndFloat32 minDistance2 = ndFloat32(1.0e10f);
			ndVector ray_p0(m_localPoly[vertexCount - 1]);
			for (ndInt32 i = 0; i < vertexCount; ++i)
			{
				const ndVector& ray_p1 = m_localPoly[i];
				const ndVector dp(ray_p1 - ray_p0);

				ndAssert(dp.m_w == ndFloat32(0.0f));
				ndFloat32 den = dp.DotProduct(dp).GetScalar();
				ndAssert(den > ndFloat32(0.0f));
				ndFloat32 parameter = dp.DotProduct(pointInFacePlane - ray_p0).GetScalar() / den;
				if ((parameter >= ndFloat32(-1.0e-4f)) && (parameter <= ndFloat32(1.0f + 1.0e-4f)))
				{
					const ndVector edgePoint(ray_p0 + dp.Scale(parameter));
					const ndVector dist(pointInFacePlane - edgePoint);
					ndFloat32 dist2 = dist.DotProduct(dist & ndVector::m_triplexMask).GetScalar();
					if (dist2 < minDistance2)
					{
						closestEdge = i;
						param = parameter;
						minDistance2 = dist2;
					}
				}
				ray_p0 = ray_p1;
			}

			if (closestEdge == -1)
			{
				ndAssert(0);
				return 0;
			}

			// we got an edge contact
			ndInt32 closestEdge0 = (closestEdge == 0) ? vertexCount - 1 : closestEdge - 1;
			contactsOut[0] = m_localPoly[closestEdge];
			contactsOut[1] = m_localPoly[closestEdge0];
			return 2;
		}
	}
	else
	{
		// if it comes here this is an edge contact 
		for (ndInt32 i = m_adjancentEdge.GetCount() - 1; i >= 0; --i)
		{
			if (m_adjancentEdge[i].m_edge == adjacentEdge.m_edge)
			{
				adjacentEdge.m_faceStart = m_adjancentEdge[i].m_faceStart;
				break;
			}
		}
		if (adjacentEdge.m_faceStart != -1)
		{
			// we fond the adjacent edge, we check if this is a face normal
			for (ndInt32 i = 0; i < vertexCount; ++i)
			{
				contactsOut[i] = m_localPoly[i];
			}
			return vertexCount;
		}
		// this should not happens
		ndAssert(0);
		//return 0;
	}
	// this should not happens
	ndAssert(0);
	return 0;
#endif
}