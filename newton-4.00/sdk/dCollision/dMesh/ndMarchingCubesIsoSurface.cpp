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
#include "ndShapeStaticMesh.h"
#include "ndMarchingCubesIsoSurface.h"

// adapted from code by written by Paul Bourke may 1994
//http://paulbourke.net/geometry/polygonise/

ndMarchingCubeIsoSurface::ndMarchingCubeIsoSurface(ndThreadPool* const threadPool, const ndVector& boxP0, const ndVector& boxP1, ndFloat32 gridSize)
	:ndMarchingCubes(threadPool, gridSize)
	,m_boxP0(boxP0 & ndVector::m_triplexMask)
	,m_boxP1(boxP1 & ndVector::m_triplexMask)
	,m_generateNormals(true)
{
	SetBox(boxP0, boxP1);

	m_gridStep.SetCount(8);
	m_gridStep[0] = ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	m_gridStep[1] = ndVector(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	m_gridStep[2] = ndVector(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f));
	m_gridStep[3] = ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f));
	m_gridStep[4] = ndVector(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	m_gridStep[5] = ndVector(ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	m_gridStep[6] = ndVector(ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(0.0f));
	m_gridStep[7] = ndVector(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(0.0f));
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

bool ndMarchingCubeIsoSurface::CalculateMinExtend3d(const ndVector& p0, const ndVector& p1, ndVector& boxP0, ndVector& boxP1) const
{
	ndAssert(p0.m_x <= p1.m_x);
	ndAssert(p0.m_y <= p1.m_y);
	ndAssert(p0.m_z <= p1.m_z);
	ndAssert(p0.m_w == ndFloat32(0.0f));
	ndAssert(p1.m_w == ndFloat32(0.0f));

	ndVector padding(ndFloat32(0.01f));
	boxP0 = (m_invGridSize * p0.Floor()) * m_gridSize;
	boxP1 = (m_invGridSize * (p1 + padding).Ceiling()) * m_gridSize;
	ndAssert(boxP0.m_x < boxP1.m_x);
	ndAssert(boxP0.m_y < boxP1.m_y);
	ndAssert(boxP0.m_z < boxP1.m_z);
	return ndBoxBoxIntestion(boxP0, boxP1, m_boxP0, m_boxP1, boxP0, boxP1);
}

ndMarchingCubeIsoSurface::ndCellFill ndMarchingCubeIsoSurface::GetFacesInCell(ndFixSizeArray<ndTriangle, 5>& patch, const ndVector& gridPosit) const
{
	ndInt32 tableIndex = 0;

	ndIsoCell cell;
	const ndVector origin(gridPosit * m_gridSize + m_boxP0);
	for (ndInt32 i = 0; i < 8; ++i)
	{
		const ndVector posit(origin + m_gridStep[i]);
		ndFloat32 isovalue = GetIsoValue(posit);
		cell.m_isoValues[i] = posit;
		cell.m_isoValues[i].m_w = isovalue;
		tableIndex |= (isovalue <= 0.0f) << i;
	}
	if (tableIndex == 0)
	{
		return m_empty;
	}

	if (tableIndex == 255)
	{
		return m_solid;
	}
	
	const ndInt32 edgeStart = m_edgeScan[tableIndex];
	const ndInt32 edgeCount = m_edgeScan[tableIndex + 1] - edgeStart;

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

	patch.SetCount(0);
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
			ndTriangle triangle;
			triangle.m_p0 = p0;
			triangle.m_p1 = p1;
			triangle.m_p2 = p2;
			patch.PushBack(triangle);
		}
	}

	return m_partial;
}

ndFloat32 ndMarchingCubeIsoSurface::RayCastCell(const ndFastRay& ray, ndInt32 xIndex0, ndInt32 yIndex0, ndInt32 zIndex0, ndVector& normalOut, ndFloat32 maxT) const
{
	ndFixSizeArray<ndTriangle, 5> patch;
	ndVector point(ndFloat32(xIndex0), ndFloat32(yIndex0), ndFloat32(zIndex0), ndFloat32(0.0f));
	ndCellFill fill = GetFacesInCell(patch, point);
	if (fill == m_empty)
	{
		return maxT;
	}

	if (fill == m_empty)
	{
		return ndFloat32 (0.0f);
	}

#if 0
	ndVector points[4];
	ndInt32 triangle[3];

	// get the 3d point at the corner of the cell
	if ((xIndex0 < 0) || (zIndex0 < 0) || (xIndex0 >= (D_TERRAIN_WIDTH - 1)) || (zIndex0 >= (D_TERRAIN_WIDTH - 1)))
	{
		return ndFloat32(1.2f);
	}
	maxT = ndMin(maxT, ndFloat32(1.0f));

	ndInt32 base = zIndex0 * D_TERRAIN_WIDTH + xIndex0;

	points[0 * 2 + 0] = ndVector((ndFloat32)(xIndex0 + 0) * D_TERRAIN_GRID_SIZE, ndFloat32(m_heightfield[base + 0]), (ndFloat32)(zIndex0 + 0) * D_TERRAIN_GRID_SIZE, ndFloat32(0.0f));
	points[0 * 2 + 1] = ndVector((ndFloat32)(xIndex0 + 1) * D_TERRAIN_GRID_SIZE, ndFloat32(m_heightfield[base + 1]), (ndFloat32)(zIndex0 + 0) * D_TERRAIN_GRID_SIZE, ndFloat32(0.0f));
	points[1 * 2 + 1] = ndVector((ndFloat32)(xIndex0 + 1) * D_TERRAIN_GRID_SIZE, ndFloat32(m_heightfield[base + D_TERRAIN_WIDTH + 1]), (ndFloat32)(zIndex0 + 1) * D_TERRAIN_GRID_SIZE, ndFloat32(0.0f));
	points[1 * 2 + 0] = ndVector((ndFloat32)(xIndex0 + 0) * D_TERRAIN_GRID_SIZE, ndFloat32(m_heightfield[base + D_TERRAIN_WIDTH + 0]), (ndFloat32)(zIndex0 + 1) * D_TERRAIN_GRID_SIZE, ndFloat32(0.0f));

	ndFloat32 t = ndFloat32(1.2f);
	triangle[0] = 1;
	triangle[1] = 2;
	triangle[2] = 3;

	ndVector e10(points[2] - points[1]);
	ndVector e20(points[3] - points[1]);
	ndVector normal(e10.CrossProduct(e20));
	normal = normal.Normalize();
	t = ray.PolygonIntersect(normal, maxT, points, triangle, 3);
	if (t < maxT)
	{
		normalOut = normal;
		return t;
	}

	triangle[0] = 1;
	triangle[1] = 0;
	triangle[2] = 2;

	ndVector e30(points[0] - points[1]);
	normal = e30.CrossProduct(e10);
	normal = normal.Normalize();
	t = ray.PolygonIntersect(normal, maxT, points, triangle, 3);
	if (t < maxT)
	{
		normalOut = normal;
	}
	return t;
#endif
return 10;
}

ndFloat32 ndMarchingCubeIsoSurface::RayCast(const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT) const
{
#if 0
	ndVector boxP0;
	ndVector boxP1;

	// make sure p0 and p1 are in the right order
	const ndVector q0(localP0.GetMin(localP1));
	const ndVector q1(localP0.GetMax(localP1));
	if (!CalculateMinExtend3d(q0, q1, boxP0, boxP1))
	{
		return ndFloat32(1.2f);
	}
	ndVector p0(localP0);
	ndVector p1(localP1);

	// check if the origin in indide the aabb
	bool test = ndOverlapTest(p0, p0, boxP0, boxP1) ? true : false;
	if (!test)
	{
		ndAssert(0);
		// orgin is outsize the intesection box. 
		// clip the ray 
		test = ndRayBoxClip(p0, p1, boxP0, boxP1);
	}
	if (!test)
	{
		return ndFloat32(1.2f);
	}

	ndVector dp(p1 - p0);
	ndVector normalOut(ndVector::m_zero);

	//ndVector positInGrids (PositionToGrid(p0));
	const ndVector positInGrids(PositionToGrid(p0).Floor());
	ndInt32 ix0 = ndInt32(ndFloor(positInGrids.m_x));
	ndInt32 iy0 = ndInt32(ndFloor(positInGrids.m_y));
	ndInt32 iz0 = ndInt32(ndFloor(positInGrids.m_z));

	// implement a 3ddda line algorithm 

	struct ndDiff
	{
		ndInt32 m_inc;
		ndFloat32 m_tx;
		ndFloat32 m_step;
	};
	ndDiff diff[3];

	for (ndInt32 i = 0; i < 3; i++)
	{
		ndDiff& dir = diff[i];
		if (dp[i] > ndFloat32(0.0f))
		{
			dir.m_inc = 1;
			ndFloat32 val = ndFloat32(1.0f) / dp[i];
			dir.m_step = m_gridSize[i] * val;
			dir.m_tx = (m_gridSize[i] * (positInGrids[i] + ndFloat32(1.0f)) - p0.m_x) * val;
		}
		else if (dp.m_x < ndFloat32(0.0f))
		{
			dir.m_inc = -1;
			ndFloat32 val = -ndFloat32(1.0f) / dp[i];
			dir.m_step = m_gridSize[i] * val;
			dir.m_tx = -(m_gridSize[i] * (positInGrids[i] - p0[i])) * val;
		}
		else
		{
			dir.m_inc = 0;
			dir.m_step = ndFloat32(0.0f);
			dir.m_tx = ndFloat32(1.0e10f);
		}
	}

	ndInt32 xInc;
	ndFloat32 tx;
	ndFloat32 stepX;
	if (dp.m_x > ndFloat32(0.0f))
	{
		xInc = 1;
		ndFloat32 val = ndFloat32(1.0f) / dp.m_x;
		stepX = m_gridSize.m_x * val;
		tx = (m_gridSize.m_x * ((ndFloat32)ix0 + ndFloat32(1.0f)) - p0.m_x) * val;
	}
	else if (dp.m_x < ndFloat32(0.0f))
	{
		xInc = -1;
		ndFloat32 val = -ndFloat32(1.0f) / dp.m_x;
		stepX = m_gridSize.m_x * val;
		tx = -(m_gridSize.m_x * (ndFloat32)ix0 - p0.m_x) * val;
	}
	else
	{
		xInc = 0;
		stepX = ndFloat32(0.0f);
		tx = ndFloat32(1.0e10f);
	}

	ndInt32 yInc;
	ndFloat32 ty;
	ndFloat32 stepY;
	if (dp.m_y > ndFloat32(0.0f))
	{
		yInc = 1;
		ndFloat32 val = ndFloat32(1.0f) / dp.m_y;
		stepY = m_gridSize.m_y * val;
		ty = (m_gridSize.m_y * ((ndFloat32)ix0 + ndFloat32(1.0f)) - p0.m_y) * val;
	}
	else if (dp.m_y < ndFloat32(0.0f))
	{
		yInc = -1;
		ndFloat32 val = -ndFloat32(1.0f) / dp.m_y;
		stepY = m_gridSize.m_y * val;
		ty = -(m_gridSize.m_y * (ndFloat32)iy0 - p0.m_y) * val;
	}
	else
	{
		yInc = 0;
		stepY = ndFloat32(0.0f);
		ty = ndFloat32(1.0e10f);
	}

	ndInt32 zInc;
	ndFloat32 tz;
	ndFloat32 stepZ;
	if (dp.m_z > ndFloat32(0.0f))
	{
		zInc = 1;
		ndFloat32 val = ndFloat32(1.0f) / dp.m_z;
		stepZ = m_gridSize.m_z * val;
		tz = (m_gridSize.m_z * ((ndFloat32)iz0 + ndFloat32(1.0f)) - p0.m_z) * val;
	}
	else if (dp.m_z < ndFloat32(0.0f))
	{
		zInc = -1;
		ndFloat32 val = -ndFloat32(1.0f) / dp.m_z;
		stepZ = m_gridSize.m_z * val;
		tz = -(m_gridSize.m_z * (ndFloat32)iz0 - p0.m_z) * val;
	}
	else
	{
		zInc = 0;
		stepZ = ndFloat32(0.0f);
		tz = ndFloat32(1.0e10f);
	}

	ndFloat32 txAcc = tx;
	ndFloat32 tyAcc = ty;
	ndFloat32 tzAcc = tz;
	ndInt32 xIndex0 = ix0;
	ndInt32 yIndex0 = iy0;
	ndInt32 zIndex0 = iz0;
	ndFastRay ray(localP0, localP1);

	// for each cell touched by the line
	do
	{
		ndFloat32 t = RayCastCell(ray, xIndex0, yIndex0, zIndex0, normalOut, maxT);
		if (t < maxT)
		{
			ndAssert(0);
			//// bail out at the first intersection and copy the data into the descriptor
			//ndAssert(normalOut.m_w == ndFloat32(0.0f));
			//contactOut.m_normal = normalOut.Normalize();
			//contactOut.m_shapeId0 = m_material[zIndex0 * D_TERRAIN_WIDTH + xIndex0];
			//contactOut.m_shapeId1 = m_material[zIndex0 * D_TERRAIN_WIDTH + xIndex0];
			//
			//return t;
		}

		if ((txAcc < tyAcc) && (txAcc < tzAcc))
		{
			tx = txAcc;
			xIndex0 += xInc;
			txAcc += stepX;
		}
		if (tyAcc < tzAcc)
		{
			ty = tyAcc;
			yIndex0 += yInc;
			tyAcc += stepY;
		}
		else
		{
			tz = tzAcc;
			zIndex0 += zInc;
			tzAcc += stepZ;
		}
	} while ((tx <= ndFloat32(1.0f)) || (ty <= ndFloat32(1.0f)) || (tz <= ndFloat32(1.0f)));
#endif
	// if no cell was hit, return a large value
	return ndFloat32(1.2f);
}

void ndMarchingCubeIsoSurface::GetFacesPatch(ndPatchMesh& patch) const
{
	ndVector boxP0;
	ndVector boxP1;

	patch.m_indexArray.SetCount(0);
	patch.m_pointArray.SetCount(0);
	patch.m_vertexArrayHasDuplicated = true;

	// if this is a aabb test, we just add one vertex 
	if (patch.m_queryType == ndPatchMesh::m_vertexListOnly)
	{
		patch.m_pointArray.PushBack(ndVector::m_zero);
		return;
	}

	if (!CalculateMinExtend3d(patch.m_boxP0, patch.m_boxP1, boxP0, boxP1))
	{
		return;
	}

	const ndVector gridP0(PositionToGrid(boxP0));
	const ndVector gridP1(PositionToGrid(boxP1));
	const ndVector sizeInGrids(gridP1 - gridP0);
	ndAssert(sizeInGrids.m_w == ndFloat32 (0.0f));

	if ((sizeInGrids.m_x == ndFloat32 (0.0f)) || (sizeInGrids.m_y == ndFloat32(0.0f)) || (sizeInGrids.m_z == ndFloat32(0.0f)))
	{
		return;
	}

	//const ndInt32 count_x = x1 - intP0.m_ix;
	//const ndInt32 count_y = y1 - intP0.m_iy;
	//const ndInt32 count_z = z1 - z0;

	//// start building the mesh
	//// build the array of unique vertices
	//const ndVector p0(ndFloat32(x0), ndFloat32(0.0f), ndFloat32(z0), ndFloat32(0.0f));
	//ndVector patchOrigin(p0 * m_gridSize);
	//for (ndInt32 iz = 0; iz <= count_z; iz++)
	//{
	//	ndVector point(patchOrigin);
	//	const ndReal* const heightfield = &m_heightfield[(iz + z0) * D_TERRAIN_WIDTH];
	//	for (ndInt32 ix = 0; ix <= count_x; ix++)
	//	{
	//		point.m_y = heightfield[ix + x0];
	//		patch.m_pointArray.PushBack(point);
	//		point.m_x += m_gridSize.m_x;
	//	}
	//	patchOrigin.m_z += m_gridSize.m_z;
	//}
	//
	//// add the face array 
	//ndInt32 vertexIndex = 0;
	//const ndInt32 step = x1 - x0 + 1;
	//for (ndInt32 z = z0; z < z1; ++z)
	//{
	//	for (ndInt32 x = x0; x < x1; ++x)
	//	{
	//		// for each quad
	//		const ndInt32 i0 = vertexIndex;
	//		const ndInt32 i1 = vertexIndex + 1;
	//		const ndInt32 i2 = vertexIndex + step;
	//		const ndInt32 i3 = vertexIndex + step + 1;
	//
	//		// we calculate the two triangle normals of this quad
	//		const ndVector e0(patch.m_pointArray[i0] - patch.m_pointArray[i1]);
	//		const ndVector e1(patch.m_pointArray[i2] - patch.m_pointArray[i1]);
	//		const ndVector e2(patch.m_pointArray[i3] - patch.m_pointArray[i1]);
	//		const ndVector n0(e0.CrossProduct(e1).Normalize());
	//		const ndVector n1(e1.CrossProduct(e2).Normalize());
	//		ndAssert(n0.m_w == ndFloat32(0.0f));
	//		ndAssert(n1.m_w == ndFloat32(0.0f));
	//
	//		ndAssert(n0.DotProduct(n0).GetScalar() > ndFloat32(0.0f));
	//		ndAssert(n1.DotProduct(n1).GetScalar() > ndFloat32(0.0f));
	//
	//		// we now check if the two triangles are coplanar
	//		const ndVector dp(patch.m_pointArray[i3] - patch.m_pointArray[i1]);
	//		ndAssert(dp.m_w == ndFloat32(0.0f));
	//		ndFloat32 dist = n0.DotProduct(dp).GetScalar();
	//
	//		if (ndAbs(dist) < ndFloat32(1.0e-3f))
	//		{
	//			// triangles are coplanal, so this is a quad
	//			patch.m_faceArray.PushBack(4);
	//			patch.m_normalArray.PushBack(n0);
	//			patch.m_faceMaterialArray.PushBack(0);
	//			patch.m_indexArray.PushBack(i2);
	//			patch.m_indexArray.PushBack(i3);
	//			patch.m_indexArray.PushBack(i1);
	//			patch.m_indexArray.PushBack(i0);
	//		}
	//		else
	//		{
	//			// triangles are not coplanal, triangulate the quad
	//			// into two triangles
	//			patch.m_faceArray.PushBack(3);
	//			patch.m_normalArray.PushBack(n0);
	//			patch.m_faceMaterialArray.PushBack(0);
	//			patch.m_indexArray.PushBack(i2);
	//			patch.m_indexArray.PushBack(i1);
	//			patch.m_indexArray.PushBack(i0);
	//
	//			patch.m_faceArray.PushBack(3);
	//			patch.m_normalArray.PushBack(n1);
	//			patch.m_faceMaterialArray.PushBack(0);
	//			patch.m_indexArray.PushBack(i1);
	//			patch.m_indexArray.PushBack(i2);
	//			patch.m_indexArray.PushBack(i3);
	//		}
	//		vertexIndex++;
	//	}
	//	vertexIndex++;
	//}

	ndVector origin(boxP0);
	ndFixSizeArray<ndReal, 1024> layer0(ndInt32(sizeInGrids.m_x * sizeInGrids.m_z) + 64);
	ndFixSizeArray<ndReal, 1024> layer1(ndInt32(sizeInGrids.m_x * sizeInGrids.m_z) + 64);

	ndInt32 layerIndex = 0;
	auto ReadLayerDensity = [this, &origin, sizeInGrids, &layerIndex, &layer0, &layer1](ndInt32 groupId)
	{
		ndVector posit(origin);
		posit.m_z += ndFloat32(groupId) * m_gridSize.m_z;

		const ndInt32 count_x = ndInt32(sizeInGrids.m_x);
		ndInt32 row = groupId * (count_x + 1);

		ndFixSizeArray<ndReal, 1024>& densityLayer = layerIndex ? layer1 : layer0;
		for (ndInt32 x = 0; x <= count_x; ++x)
		{
			ndReal density = GetIsoValue(posit);
			densityLayer[row + x] = density;
			posit.m_x += m_gridSize.m_x;
		}
	};
	const ndInt32 count_z = ndInt32(sizeInGrids.m_z);
	for (ndInt32 z = 0; z <= count_z; ++z)
	{
		ReadLayerDensity(z);
	}

	const ndInt32 count_x = ndInt32(sizeInGrids.m_x);
	const ndInt32 count_y = ndInt32(sizeInGrids.m_y);

static int xxxx;
	ndFloat32 fy = boxP0.m_y;
	for (ndInt32 y = 1; y < count_y; ++y)
	{
		layerIndex = 1 - layerIndex;
		origin.m_y += m_gridSize.m_y;
		for (ndInt32 z = 0; z <= count_z; ++z)
		{
			ReadLayerDensity(z);
		}

		//iterate over each cell of this layer and generate the faces if any is found
		const ndFixSizeArray<ndReal, 1024>& density0 = layerIndex ? layer0 : layer1;
		const ndFixSizeArray<ndReal, 1024>& density1 = layerIndex ? layer1 : layer0;

		ndInt32 row = 0;
		ndFloat32 fz = boxP0.m_z;
		for (ndInt32 z = 0; z < count_z; ++z)
		{
			ndFloat32 fx = boxP0.m_x;
			for (ndInt32 x = 0; x < count_x; ++x)
			{
				ndIsoCell cell;
				const ndInt32 i0 = row + x;
				const ndInt32 i1 = row + x + 1;
				const ndInt32 i2 = row + (count_x + 1) + x + 1;
				const ndInt32 i3 = row + (count_x + 1) + x;
		
				ndFloat32 isoValues[8];
				isoValues[0] = density0[i0];
				isoValues[1] = density0[i1];
				isoValues[2] = density0[i2];
				isoValues[3] = density0[i3];
				isoValues[4] = density1[i0];
				isoValues[5] = density1[i1];
				isoValues[6] = density1[i2];
				isoValues[7] = density1[i3];
		
				ndInt32 tableIndex = 0;
				for (ndInt32 i = 0; i < 8; ++i)
				{
					tableIndex |= (isoValues[i] <= 0.0f) << i;
				}

				if ((tableIndex != 0) && (tableIndex != 255))
				{
					xxxx++;
					const ndInt32 edgeStart = m_edgeScan[tableIndex];
					const ndInt32 edgeCount = m_edgeScan[tableIndex + 1] - edgeStart;
					ndAssert(edgeCount);

					//cell.m_isoValues[0] = ndVector(ndFloat32(x + 0), grid_y0, grid_Z0, isoValues[0]);
					//cell.m_isoValues[1] = ndVector(ndFloat32(x + 1), grid_y0, grid_Z0, isoValues[1]);
					//cell.m_isoValues[2] = ndVector(ndFloat32(x + 1), grid_y0, grid_Z1, isoValues[2]);
					//cell.m_isoValues[3] = ndVector(ndFloat32(x + 0), grid_y0, grid_Z1, isoValues[3]);
					//cell.m_isoValues[4] = ndVector(ndFloat32(x + 0), grid_y1, grid_Z0, isoValues[4]);
					//cell.m_isoValues[5] = ndVector(ndFloat32(x + 1), grid_y1, grid_Z0, isoValues[5]);
					//cell.m_isoValues[6] = ndVector(ndFloat32(x + 1), grid_y1, grid_Z1, isoValues[6]);
					//cell.m_isoValues[7] = ndVector(ndFloat32(x + 0), grid_y1, grid_Z1, isoValues[7]);

					cell.m_isoValues[0] = ndVector(fx,                  fy,                  fz,                  isoValues[0]);
					cell.m_isoValues[1] = ndVector(fx + m_gridSize.m_x, fy,                  fz,	              isoValues[1]);
					cell.m_isoValues[2] = ndVector(fx + m_gridSize.m_x, fy,                  fz + m_gridSize.m_z, isoValues[2]);
					cell.m_isoValues[3] = ndVector(fx,                  fy,                  fz + m_gridSize.m_z, isoValues[3]);

					cell.m_isoValues[4] = ndVector(fx,                  fy + m_gridSize.m_y, fz,                  isoValues[4]);
					cell.m_isoValues[5] = ndVector(fx + m_gridSize.m_x, fy + m_gridSize.m_y, fz,                  isoValues[5]);
					cell.m_isoValues[6] = ndVector(fx + m_gridSize.m_x, fy + m_gridSize.m_y, fz + m_gridSize.m_z, isoValues[6]);
					cell.m_isoValues[7] = ndVector(fx,                  fy + m_gridSize.m_y, fz + m_gridSize.m_z, isoValues[7]);
					
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
						const ndVector normal(p10.CrossProduct(p20) & ndVector::m_triplexMask);
						ndFloat32 areaMag2 = normal.DotProduct(normal).GetScalar();
						if (areaMag2 > ndFloat32(1.0e-6f))
						{
							const ndInt32 triangleIndex = patch.m_indexArray.GetCount();\

							patch.m_faceArray.PushBack(3);
							patch.m_faceMaterialArray.PushBack(0);

							patch.m_pointArray.PushBack(p0);
							patch.m_pointArray.PushBack(p1);
							patch.m_pointArray.PushBack(p2);
							patch.m_normalArray.PushBack(normal.Normalize());
							patch.m_indexArray.PushBack(triangleIndex + 0);
							patch.m_indexArray.PushBack(triangleIndex + 1);
							patch.m_indexArray.PushBack(triangleIndex + 2);
						}
					}
				}
				fx += m_gridSize.m_x;
			}
			
			row += count_x + 1;
			fz += m_gridSize.m_z;
		}
		fy += m_gridSize.m_y;
	}
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
	m_meshNormals.SetCount(0);
	m_meshPoints.SetCount(vertexCount);
	//m_meshNormals.SetCount(vertexCount);
	//auto ApplyScale = [this](ndInt32 groupId)

	auto ApplyScale = ndMakeObject::ndFunction([this](ndInt32 groupId, ndInt32)
	{
		//m_meshNormals[groupId] = ndVector::m_zero;
		m_meshPoints[groupId] = ndVector::m_triplexMask & (m_meshPoints[groupId] * m_gridSize + m_boxP0);
	});
	m_threadPool->ParallelExecute(ApplyScale, ndInt32 (m_meshPoints.GetCount()), 256);

	if (m_generateNormals)
	{
		m_meshNormals.SetCount(vertexCount);
		for (ndInt32 i = 0; i < ndInt32(m_meshPoints.GetCount()); ++i)
		{
			m_meshNormals[i] = ndVector::m_zero;
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
}

void ndMarchingCubeIsoSurface::GenerateMesh()
{
	ndAssert(m_threadPool);

	m_threadPool->Begin();
	const ndVector boxSize(m_boxP1 - m_boxP0);
	const ndVector boxSizeInGrids((boxSize * m_invGridSize).Ceiling().GetInt());

	ndArray<ndReal> densityWindow0;
	ndArray<ndReal> densityWindow1;
	densityWindow0.SetCount(boxSizeInGrids.m_ix * boxSizeInGrids.m_iz);
	densityWindow1.SetCount(boxSizeInGrids.m_ix * boxSizeInGrids.m_iz);

	ndFloat32 high = m_boxP0.m_y;
	auto ReadLayerDensity = ndMakeObject::ndFunction([this, &boxSizeInGrids, &high, &densityWindow0, &densityWindow1](ndInt32 groupId, ndInt32)
	{
		ndFloat32 posit_x = m_boxP0.m_x;
		ndFloat32 posit_z = m_boxP0.m_z + ndFloat32(groupId) * m_gridSize.m_z;
		ndVector posit(posit_x, high, posit_z, ndFloat32(0.0f));

		ndInt32 row = ndInt32 (groupId * boxSizeInGrids.m_ix);
		for (ndInt32 x = 0; x < boxSizeInGrids.m_ix; ++x)
		{
			densityWindow1[row + x] = GetIsoValue(posit);
			posit.m_x += m_gridSize.m_x;
		}
	});
	m_threadPool->ParallelExecute(ReadLayerDensity, ndInt32(boxSizeInGrids.m_iz), 1);
	densityWindow0.Swap(densityWindow1);

	ndFloat32 grid_y0 = ndFloat32(0.0f);
	ndFloat32 grid_y1 = ndFloat32(1.0f);
	for (ndInt32 y = 1; y < (boxSizeInGrids.m_iy - 1); ++y)
	{
		high += m_gridSize.m_y;
		m_threadPool->ParallelExecute(ReadLayerDensity, ndInt32(boxSizeInGrids.m_iz), 1);
		
		//ndInt32 stride = 0;
		ndFloat32 grid_Z0 = ndFloat32(0.0f);
		ndFloat32 grid_Z1 = ndFloat32(1.0f);
		//for (ndInt32 z = 0; z < boxSizeInGrids.m_iz - 1; ++z)
		//{
		//	for (ndInt32 x = 0; x < boxSizeInGrids.m_ix - 1; ++x)
		//	{
		//		ndIsoCell cell;
		//		const ndInt32 i0 = stride + x;
		//		const ndInt32 i1 = stride + x + 1;
		//		const ndInt32 i2 = boxSizeInGrids.m_ix + stride + x + 1;
		//		const ndInt32 i3 = boxSizeInGrids.m_ix + stride + x;
		//
		//		ndFloat32 isoValues[8];
		//		isoValues[0] = densityWindow0[i0];
		//		isoValues[1] = densityWindow0[i1];
		//		isoValues[2] = densityWindow0[i2];
		//		isoValues[3] = densityWindow0[i3];
		//		isoValues[4] = densityWindow1[i0];
		//		isoValues[5] = densityWindow1[i1];
		//		isoValues[6] = densityWindow1[i2];
		//		isoValues[7] = densityWindow1[i3];
		//
		//		ndInt32 tableIndex = 0;
		//		for (ndInt32 i = 0; i < 8; ++i)
		//		{
		//			tableIndex |= (isoValues[i] <= 0.0f) << i;
		//		}
		//
		//		const ndInt32 edgeStart = m_edgeScan[tableIndex];
		//		const ndInt32 edgeCount = m_edgeScan[tableIndex + 1] - edgeStart;
		//		if (edgeCount)
		//		{
		//			cell.m_isoValues[0] = ndVector(ndFloat32(x + 0), grid_y0, grid_Z0, isoValues[0]);
		//			cell.m_isoValues[1] = ndVector(ndFloat32(x + 1), grid_y0, grid_Z0, isoValues[1]);
		//			cell.m_isoValues[2] = ndVector(ndFloat32(x + 1), grid_y0, grid_Z1, isoValues[2]);
		//			cell.m_isoValues[3] = ndVector(ndFloat32(x + 0), grid_y0, grid_Z1, isoValues[3]);
		//			cell.m_isoValues[4] = ndVector(ndFloat32(x + 0), grid_y1, grid_Z0, isoValues[4]);
		//			cell.m_isoValues[5] = ndVector(ndFloat32(x + 1), grid_y1, grid_Z0, isoValues[5]);
		//			cell.m_isoValues[6] = ndVector(ndFloat32(x + 1), grid_y1, grid_Z1, isoValues[6]);
		//			cell.m_isoValues[7] = ndVector(ndFloat32(x + 0), grid_y1, grid_Z1, isoValues[7]);
		//
		//			ndVector vertlist[12];
		//			for (ndInt32 i = 0; i < edgeCount; ++i)
		//			{
		//				const ndEdge& edge = m_edges[edgeStart + i];
		//				const ndInt32 midPoint = edge.m_midPoint;
		//					
		//				const ndVector p0(cell.m_isoValues[edge.m_p0]);
		//				const ndVector p1(cell.m_isoValues[edge.m_p1]);
		//				ndAssert((p1.m_w * p0.m_w) <= ndFloat32(0.0f));
		//				const ndVector p1p0(p1 - p0);
		//				ndFloat32 param = p0.m_w / (p1.m_w - p0.m_w);
		//				const ndVector p3 (ndVector::m_triplexMask & (p0 - p1p0.Scale(param)));
		//				vertlist[midPoint] = p3;
		//			}
		//
		//			const ndInt32 faceStart = m_facesScan[tableIndex];
		//			const ndInt32 triangleStart = m_facesScan[tableIndex];
		//			const ndInt32 triangleCount = m_facesScan[tableIndex + 1] - triangleStart;
		//			for (ndInt32 i = 0; i < triangleCount; ++i)
		//			{
		//				const ndInt32 j0 = m_faces[faceStart + i][0];
		//				const ndInt32 j1 = m_faces[faceStart + i][1];
		//				const ndInt32 j2 = m_faces[faceStart + i][2];
		//
		//				const ndVector& p0 = vertlist[j0];
		//				const ndVector& p1 = vertlist[j1];
		//				const ndVector& p2 = vertlist[j2];
		//				const ndVector p10(p1 - p0);
		//				const ndVector p20(p2 - p0);
		//				const ndVector area(p10.CrossProduct(p20));
		//				ndFloat32 areaMag2 = area.DotProduct(area & ndVector::m_triplexMask).GetScalar();
		//				if (areaMag2 > ndFloat32(1.0e-6f))
		//				{
		//					m_meshPoints.PushBack(p0);
		//					m_meshPoints.PushBack(p1);
		//					m_meshPoints.PushBack(p2);
		//				}
		//			}
		//		}
		//	}
		//	grid_Z0 += ndFloat32(1.0f);
		//	grid_Z1 += ndFloat32(1.0f);
		//	stride += boxSizeInGrids.m_ix;
		//}

		ndArray<ndGridInfo> gridScansLayer;
		ndArray<ndGridInfo> gridScansLayerTemp;
		gridScansLayer.SetCount((boxSizeInGrids.m_ix - 1) * (boxSizeInGrids.m_iz - 1));
		gridScansLayer.PushBack(ndGridInfo());
		auto CountGrids = ndMakeObject::ndFunction([this, &boxSizeInGrids, &densityWindow0, &densityWindow1, &gridScansLayer](ndInt32 groupId, ndInt32)
		{
			ndIsoCell cell;
			ndFloat32 isoValues[8];
			const ndInt32 stride = ndInt32(groupId * boxSizeInGrids.m_ix);
			const ndInt32 row = ndInt32(groupId * (boxSizeInGrids.m_ix - 1));
			for (ndInt32 x = 0; x < boxSizeInGrids.m_ix - 1; ++x)
			{
				const ndInt32 i0 = stride + x;
				const ndInt32 i1 = stride + x + 1;
				const ndInt32 i2 = ndInt32(boxSizeInGrids.m_ix) + stride + x + 1;
				const ndInt32 i3 = ndInt32(boxSizeInGrids.m_ix) + stride + x;
				
				isoValues[0] = densityWindow0[i0];
				isoValues[1] = densityWindow0[i1];
				isoValues[2] = densityWindow0[i2];
				isoValues[3] = densityWindow0[i3];
				isoValues[4] = densityWindow1[i0];
				isoValues[5] = densityWindow1[i1];
				isoValues[6] = densityWindow1[i2];
				isoValues[7] = densityWindow1[i3];
		
				ndInt32 tableIndex = 0;
				for (ndInt32 i = 0; i < 8; ++i)
				{
					tableIndex |= (isoValues[i] <= 0.0f) << i;
				}
				ndGridInfo gridInfo(m_facesScan[tableIndex + 1] - m_facesScan[tableIndex]);
				gridInfo.m_x = x;
				gridInfo.m_z = groupId;
				gridInfo.m_tableIndex = tableIndex;
				gridScansLayer[row + x] = gridInfo;
			}
		});
		m_threadPool->ParallelExecute(CountGrids, ndInt32(boxSizeInGrids.m_iz) - 1, 1);

		class ndGridClassifier
		{
			public:
			ndGridClassifier(void* const) {}
			ndInt32 GetKey(const ndGridInfo& info) const
			{
				return (info.m_triangleCount == 0) ? 1 : 0;
			}
		};

		ndUnsigned32 scans[4];
		ndCountingSort<ndGridInfo, ndGridClassifier, 1>(*m_threadPool, gridScansLayer, gridScansLayerTemp, scans, nullptr);
		if (scans[1])
		{
			ndInt32 sum = 0;
			const ndInt32 gridsCount = ndInt32(scans[1]);
			for (ndInt32 i = 0; i < gridsCount; ++i)
			{
				ndInt32 acc = sum + gridScansLayer[i].m_triangleCount;
				gridScansLayer[i].m_triangleCount = sum;
				sum = acc;
			}
			gridScansLayer[gridsCount].m_triangleCount = sum;
			gridScansLayer.SetCount(gridsCount + 1);

			ndInt32 trianglesBase = ndInt32 (m_meshPoints.GetCount());
			m_meshPoints.SetCount(trianglesBase + sum * 3);
			auto GenerateTriangles = ndMakeObject::ndFunction([this, &boxSizeInGrids, trianglesBase, &grid_y0, &grid_y1, &densityWindow0, &densityWindow1, &gridScansLayer](ndInt32 groupId, ndInt32)
			{
				ndIsoCell cell;
				const ndGridInfo& info = gridScansLayer[groupId];
				ndFloat32 isoValues[8];

				const ndInt32 stride = ndInt32(info.m_z * boxSizeInGrids.m_ix);
				const ndInt32 i0 = stride + info.m_x;
				const ndInt32 i1 = stride + info.m_x + 1;
				const ndInt32 i2 = ndInt32(boxSizeInGrids.m_ix) + stride + info.m_x + 1;
				const ndInt32 i3 = ndInt32(boxSizeInGrids.m_ix) + stride + info.m_x;

				isoValues[0] = densityWindow0[i0];
				isoValues[1] = densityWindow0[i1];
				isoValues[2] = densityWindow0[i2];
				isoValues[3] = densityWindow0[i3];
				isoValues[4] = densityWindow1[i0];
				isoValues[5] = densityWindow1[i1];
				isoValues[6] = densityWindow1[i2];
				isoValues[7] = densityWindow1[i3];

				const ndInt32 tableIndex = info.m_tableIndex;
				const ndInt32 edgeStart = m_edgeScan[tableIndex];
				const ndInt32 edgeCount = m_edgeScan[tableIndex + 1] - edgeStart;
				ndAssert(edgeCount);

				cell.m_isoValues[0] = ndVector(ndFloat32(info.m_x + 0), grid_y0, ndFloat32(info.m_z + 0), isoValues[0]);
				cell.m_isoValues[1] = ndVector(ndFloat32(info.m_x + 1), grid_y0, ndFloat32(info.m_z + 0), isoValues[1]);
				cell.m_isoValues[2] = ndVector(ndFloat32(info.m_x + 1), grid_y0, ndFloat32(info.m_z + 1), isoValues[2]);
				cell.m_isoValues[3] = ndVector(ndFloat32(info.m_x + 0), grid_y0, ndFloat32(info.m_z + 1), isoValues[3]);
				cell.m_isoValues[4] = ndVector(ndFloat32(info.m_x + 0), grid_y1, ndFloat32(info.m_z + 0), isoValues[4]);
				cell.m_isoValues[5] = ndVector(ndFloat32(info.m_x + 1), grid_y1, ndFloat32(info.m_z + 0), isoValues[5]);
				cell.m_isoValues[6] = ndVector(ndFloat32(info.m_x + 1), grid_y1, ndFloat32(info.m_z + 1), isoValues[6]);
				cell.m_isoValues[7] = ndVector(ndFloat32(info.m_x + 0), grid_y1, ndFloat32(info.m_z + 1), isoValues[7]);

				const ndInt32 triangleStart = m_facesScan[tableIndex];
				const ndInt32 triangleCount = m_facesScan[tableIndex + 1] - triangleStart;
				ndAssert(triangleCount == (gridScansLayer[groupId + 1].m_triangleCount - info.m_triangleCount));

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
				
				const ndInt32 trianglesOffset = trianglesBase + info.m_triangleCount * 3;
				const ndInt32 faceStart = m_facesScan[tableIndex];
				for (ndInt32 i = 0; i < triangleCount; ++i)
				{
					const ndInt32 j0 = m_faces[faceStart + i][0];
					const ndInt32 j1 = m_faces[faceStart + i][1];
					const ndInt32 j2 = m_faces[faceStart + i][2];
				
					ndVector p0 (vertlist[j0]);
					const ndVector& p1 = vertlist[j1];
					const ndVector& p2 = vertlist[j2];
					const ndVector p10(p1 - p0);
					const ndVector p20(p2 - p0);
					const ndVector area(p10.CrossProduct(p20));
					ndFloat32 areaMag2 = area.DotProduct(area & ndVector::m_triplexMask).GetScalar();
					
					p0.m_w = areaMag2;
					m_meshPoints[trianglesOffset + i * 3 + 0] = p0;
					m_meshPoints[trianglesOffset + i * 3 + 1] = p1;
					m_meshPoints[trianglesOffset + i * 3 + 2] = p2;
					i *= 1;
				}
			});
			
			const ndInt32 jobStride = 32;
			m_threadPool->ParallelExecute(GenerateTriangles, gridsCount, jobStride);

			grid_Z0 += ndFloat32(1.0f);
			grid_Z1 += ndFloat32(1.0f);
		}

		grid_y0 += ndFloat32(1.0f);
		grid_y1 += ndFloat32(1.0f);
		densityWindow0.Swap(densityWindow1);
	}

	//high += m_gridSize.m_y;
	
	class ndTriangleClassifier
	{
		public:
		ndTriangleClassifier(void* const) {}
		ndInt32 GetKey(const ndTriangle& triangle) const
		{
			return (triangle.m_p0.m_w < ndFloat32(1.0e-6f)) ? 1 : 0;
		}
	};

	ndUnsigned32 scans[4];

	m_meshNormals.SetCount(m_meshPoints.GetCount());
	ndTriangle* const triangles = (ndTriangle*)&m_meshPoints[0];
	ndTriangle* const trianglesTmp = (ndTriangle*)&m_meshNormals[0];
	ndCountingSort<ndTriangle, ndTriangleClassifier, 1>(*m_threadPool, triangles, trianglesTmp, ndInt32(m_meshPoints.GetCount()) / 3, scans, nullptr);
	m_meshPoints.Swap(m_meshNormals);
	m_meshPoints.SetCount(scans[1] * 3);

	GenerateIndexList();

	m_threadPool->End();
}
