/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndPhysicsWorld.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndHeightFieldPrimitive.h"

//#define D_TERRAIN_WIDTH			1024
//#define D_TERRAIN_HEIGHT			1024
//#define D_TERRAIN_WIDTH			512
//#define D_TERRAIN_HEIGHT			512
#define D_TERRAIN_WIDTH				256
#define D_TERRAIN_HEIGHT			256

#define D_TERRAIN_NOISE_OCTAVES		8
#define D_TERRAIN_NOISE_PERSISTANCE	0.5f
#define D_TERRAIN_NOISE_GRID_SCALE  (1.0f / 500.0f)

//#define D_TERRAIN_GRID_SIZE			2.0f
#define D_TERRAIN_GRID_SIZE			1.0f
#define D_TERRAIN_TILE_SIZE			128
#define D_TERRAIN_ELEVATION_SCALE	32.0f

class ndProceduralTerrainShape : public ndShapeStaticProceduralMesh
{
	public:
	D_CLASS_REFLECTION(ndProceduralTerrainShape, ndShapeStaticProceduralMesh)

	ndProceduralTerrainShape()
		:ndShapeStaticProceduralMesh()
		,m_padding(ndVector::m_triplexMask & ndVector(0.1f))
		,m_gridSize(ndVector::m_triplexMask & ndVector (D_TERRAIN_GRID_SIZE))
		,m_invGridSize(ndVector::m_triplexMask & ndVector(ndFloat32(1.0f) / D_TERRAIN_GRID_SIZE))
	{
		MakeNoiseHeightfield();

		ndReal minY = 1.0e20f;
		ndReal maxY = -1.0e20f;
		for (ndInt32 i = ndInt32(m_heightfield.GetCount()) - 1; i >= 0; --i)
		{
			minY = ndMin(minY, m_heightfield[i]);
			maxY = ndMax(maxY, m_heightfield[i]);
		}

		ndVector p0(ndFloat32 (0.0f), minY, ndFloat32(0.0f), ndFloat32(1.0f));
		ndVector p1(ndFloat32(D_TERRAIN_WIDTH * D_TERRAIN_GRID_SIZE), maxY, ndFloat32(D_TERRAIN_WIDTH * D_TERRAIN_GRID_SIZE), ndFloat32(1.0f));
		SetAABB(p0, p1);

		// to account for rounding
		CalculateAabb(ndGetIdentityMatrix(), m_minBox, m_maxBox);
	}

	virtual ndUnsigned64 GetHash(ndUnsigned64 hash) const override
	{
		// return a unique hash code for this shape
		ndInt32 thisHash = 0x1234567;
		return ndCRC64(&thisHash, sizeof(ndInt32), hash);
	}

	void MakeNoiseHeightfield()
	{
		m_material.SetCount(D_TERRAIN_WIDTH * D_TERRAIN_HEIGHT);
		m_heightfield.SetCount(D_TERRAIN_WIDTH * D_TERRAIN_HEIGHT);

		const ndInt32 octaves = D_TERRAIN_NOISE_OCTAVES;
		const ndFloat32 persistance = D_TERRAIN_NOISE_PERSISTANCE;
		const ndFloat32 noiseGridScale = D_TERRAIN_NOISE_GRID_SCALE;

		ndReal minHeight = ndFloat32(1.0e10f);
		ndReal maxHeight = ndFloat32(-1.0e10f);
		for (ndInt32 z = 0; z < D_TERRAIN_HEIGHT; z++)
		{
			for (ndInt32 x = 0; x < D_TERRAIN_WIDTH; x++)
			{
				ndReal noiseVal = ndReal (BrownianMotion(octaves, persistance, noiseGridScale * ndFloat32(x), noiseGridScale * ndFloat32(z)));
				//noiseVal = 0.0f;

				m_heightfield[z * D_TERRAIN_WIDTH + x] = noiseVal;
				minHeight = ndMin(minHeight, noiseVal);
				maxHeight = ndMax(maxHeight, noiseVal);

				// that app should populate this with app materials ids.
				// just make a zero material index, for the demo
				m_material[z * D_TERRAIN_WIDTH + x] = 0;
			}
		}
		minHeight -= ndReal(m_padding.m_y);
		maxHeight += ndReal(m_padding.m_y);

		ndReal scale = D_TERRAIN_ELEVATION_SCALE / (maxHeight - minHeight);
		for (ndInt32 i = 0; i < m_heightfield.GetCapacity(); ++i)
		{
			ndReal y = m_heightfield[i];
			y = scale * y;
			m_heightfield[i] = y;
		}
	}

	virtual void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& notify) const override
	{
		ndVector face[4];
		ndShapeDebugNotify::ndEdgeType edgeType = ndShapeDebugNotify::m_shared;
		for (ndInt32 z = 0; z < D_TERRAIN_HEIGHT - 1; z++)
		{
			const ndReal* const row = &m_heightfield[z * D_TERRAIN_WIDTH];
			ndVector p0(ndFloat32(0.0f), ndFloat32(row[0]),				  ndFloat32(z + 0) * D_TERRAIN_GRID_SIZE, ndFloat32(1.0f));
			ndVector p1(ndFloat32(0.0f), ndFloat32(row[D_TERRAIN_WIDTH]), ndFloat32(z + 1) * D_TERRAIN_GRID_SIZE, ndFloat32(1.0f));
			p0 = matrix.TransformVector(p0);
			p1 = matrix.TransformVector(p1);

			for (ndInt32 x = 1; x < D_TERRAIN_WIDTH - 1; x++)
			{
				const ndVector q0(matrix.TransformVector(ndVector(ndFloat32(x) * D_TERRAIN_GRID_SIZE, ndFloat32(row[x]),                   ndFloat32(z + 0) * D_TERRAIN_GRID_SIZE, ndFloat32(1.0f))));
				const ndVector q1(matrix.TransformVector(ndVector(ndFloat32(x) * D_TERRAIN_GRID_SIZE, ndFloat32(row[x + D_TERRAIN_WIDTH]), ndFloat32(z + 1) * D_TERRAIN_GRID_SIZE, ndFloat32(1.0f))));

				const ndVector normal(((p0 - q0).CrossProduct(p1 - q0)).Normalize());
				ndAssert(normal.m_w == ndFloat32(0.0f));
				ndFloat32 dist = normal.DotProduct(q1 - q0).GetScalar();
				if (ndAbs(dist) < ndFloat32(1.0e-3f))
				{
					face[0] = p0;
					face[1] = p1;
					face[2] = q1;
					face[3] = q0;
					notify.DrawPolygon(4, face, &edgeType);
				}
				else
				{
					face[0] = p0;
					face[1] = p1;
					face[2] = q0;
					notify.DrawPolygon(3, face, &edgeType);

					face[0] = p1;
					face[1] = q1;
					face[2] = q0;
					notify.DrawPolygon(3, face, &edgeType);
				}

				p0 = q0;
				p1 = q1;
			}
		}
	}

	virtual ndFloat32 RayCast(ndRayCastNotify&, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const, ndContactPoint& contactOut) const override
	{
		ndVector boxP0;
		ndVector boxP1;
		
		// make sure p0 and p1 are in the right order
		const ndVector q0(localP0.GetMin(localP1) - m_padding);
		const ndVector q1(localP0.GetMax(localP1) + m_padding);
		CalculateMinExtend3d(q0, q1, boxP0, boxP1);

		// make the box a beam tha extend from 
		// infinite positive high to -infinity high.
		// 1.0e10 represents infinity.
		boxP0.m_y = -ndFloat32(1.0e10f);
		boxP1.m_y = ndFloat32(1.0e10f);

		ndVector p0(localP0);
		ndVector p1(localP1);
		
		// clip the line against the bounding box
		if (ndRayBoxClip(p0, p1, boxP0, boxP1))
		{
			ndVector dp(p1 - p0);
			ndVector normalOut(ndVector::m_zero);
		
			ndFloat32 scale_x = D_TERRAIN_GRID_SIZE;
			ndFloat32 invScale_x = ndFloat32(1.0f) / D_TERRAIN_GRID_SIZE;
			ndFloat32 scale_z = D_TERRAIN_GRID_SIZE;
			ndFloat32 invScale_z = ndFloat32(1.0f) / D_TERRAIN_GRID_SIZE;
			ndInt32 ix0 = ndInt32(ndFloor(p0.m_x * invScale_x));
			ndInt32 iz0 = ndInt32(ndFloor(p0.m_z * invScale_z));
		
			// implement a 3ddda line algorithm 
			ndInt32 xInc;
			ndFloat32 tx;
			ndFloat32 stepX;
			if (dp.m_x > ndFloat32(0.0f))
			{
				xInc = 1;
				ndFloat32 val = ndFloat32(1.0f) / dp.m_x;
				stepX = scale_x * val;
				tx = (scale_x * ((ndFloat32)ix0 + ndFloat32(1.0f)) - p0.m_x) * val;
			}
			else if (dp.m_x < ndFloat32(0.0f))
			{
				xInc = -1;
				ndFloat32 val = -ndFloat32(1.0f) / dp.m_x;
				stepX = scale_x * val;
				tx = -(scale_x * (ndFloat32)ix0 - p0.m_x) * val;
			}
			else
			{
				xInc = 0;
				stepX = ndFloat32(0.0f);
				tx = ndFloat32(1.0e10f);
			}
		
			ndInt32 zInc;
			ndFloat32 tz;
			ndFloat32 stepZ;
			if (dp.m_z > ndFloat32(0.0f))
			{
				zInc = 1;
				ndFloat32 val = ndFloat32(1.0f) / dp.m_z;
				stepZ = scale_z * val;
				tz = (scale_z * ((ndFloat32)iz0 + ndFloat32(1.0f)) - p0.m_z) * val;
			}
			else if (dp.m_z < ndFloat32(0.0f))
			{
				zInc = -1;
				ndFloat32 val = -ndFloat32(1.0f) / dp.m_z;
				stepZ = scale_z * val;
				tz = -(scale_z * (ndFloat32)iz0 - p0.m_z) * val;
			}
			else
			{
				zInc = 0;
				stepZ = ndFloat32(0.0f);
				tz = ndFloat32(1.0e10f);
			}
		
			ndFloat32 txAcc = tx;
			ndFloat32 tzAcc = tz;
			ndInt32 xIndex0 = ix0;
			ndInt32 zIndex0 = iz0;
			ndFastRay ray(localP0, localP1);
		
			// for each cell touched by the line
			do
			{
				ndFloat32 t = RayCastCell(ray, xIndex0, zIndex0, normalOut, maxT);
				if (t < maxT)
				{
					// bail out at the first intersection and copy the data into the descriptor
					ndAssert(normalOut.m_w == ndFloat32(0.0f));
					contactOut.m_normal = normalOut.Normalize();
					contactOut.m_shapeId0 = m_material[zIndex0 * D_TERRAIN_WIDTH + xIndex0];
					contactOut.m_shapeId1 = m_material[zIndex0 * D_TERRAIN_WIDTH + xIndex0];
		
					return t;
				}
		
				if (txAcc < tzAcc)
				{
					tx = txAcc;
					xIndex0 += xInc;
					txAcc += stepX;
				}
				else
				{
					tz = tzAcc;
					zIndex0 += zInc;
					tzAcc += stepZ;
				}
			} while ((tx <= ndFloat32(1.0f)) || (tz <= ndFloat32(1.0f)));
		}

		// if no cell was hit, return a large value
		return ndFloat32(1.2f);
	}

	void GetFacesPatch(ndPatchMesh& patch) const override
	{
		ndAssert(patch.m_convexShapeInstance);
		// calculate box extend rounded you the padding
		ndVector boxP0;
		ndVector boxP1;
		CalculateMinExtend3d(patch.m_boxP0, patch.m_boxP1, boxP0, boxP1);

		// clamp sweep box against shape bounds, and get the integer dimension
		const ndVector intP0((m_invGridSize * boxP0.GetMax(m_minBox)).GetInt());
		const ndVector intP1((m_invGridSize * boxP1.GetMin(m_maxBox)).GetInt());
		
		const ndInt32 x0 = ndInt32(intP0.m_ix);
		const ndInt32 x1 = ndInt32(intP1.m_ix);
		const ndInt32 z0 = ndInt32(intP0.m_iz);
		const ndInt32 z1 = ndInt32(intP1.m_iz);
		
		if ((x1 == x0) || (z1 == z0))
		{
			return;
		}
		
		ndFloat32 minHeight = ndFloat32(1.0e10f);
		ndFloat32 maxHeight = ndFloat32(-1.0e10f);
		CalculateMinAndMaxElevation(x0, x1, z0, z1, minHeight, maxHeight);
		
		if ((maxHeight < boxP0.m_y) || (minHeight > boxP1.m_y))
		{
			// the box does not interset the heightfield
			return;
		}
		
		const ndInt32 count_x = x1 - x0;
		const ndInt32 count_z = z1 - z0;
		ndInt32 numberOfQuad = (x1 - x0) * (z1 - z0);
		if (numberOfQuad == 0)
		{
			// box overlap but not faces are collected
			return;
		}

		// since the vertex pathc has no duplicate, 
		// we can skip the vertex sorting
		patch.m_vertexArrayHasDuplicated = false;

		// if this is a aabb test, we just add one vertex 
		if (patch.m_queryType == ndPatchMesh::m_vertexListOnly)
		{
			patch.m_pointArray.PushBack(ndVector::m_zero);
			return;
		}

		// start building the mesh
		// build the array of unique vertices
		const ndVector p0(ndFloat32(x0), ndFloat32(0.0f), ndFloat32(z0), ndFloat32(0.0f));
		ndVector patchOrigin(p0 * m_gridSize);
		for (ndInt32 iz = 0; iz <= count_z; iz++)
		{
			ndVector point(patchOrigin);
			const ndReal* const heightfield = &m_heightfield[(iz + z0) * D_TERRAIN_WIDTH];
			for (ndInt32 ix = 0; ix <= count_x; ix++)
			{
				point.m_y = heightfield[ix + x0];
				patch.m_pointArray.PushBack(point);
				point.m_x += m_gridSize.m_x;
			}
			patchOrigin.m_z += m_gridSize.m_z;
		}
		
		// add the face array 
		ndInt32 vertexIndex = 0;
		const ndInt32 step = x1 - x0 + 1;
		for (ndInt32 z = z0; z < z1; ++z)
		{
			for (ndInt32 x = x0; x < x1; ++x)
			{
				// for each quad
				const ndInt32 i0 = vertexIndex;
				const ndInt32 i1 = vertexIndex + 1;
				const ndInt32 i2 = vertexIndex + step;
				const ndInt32 i3 = vertexIndex + step + 1;
		
				// we calculate the two triangle normals of this quad
				const ndVector e0(patch.m_pointArray[i0] - patch.m_pointArray[i1]);
				const ndVector e1(patch.m_pointArray[i2] - patch.m_pointArray[i1]);
				const ndVector e2(patch.m_pointArray[i3] - patch.m_pointArray[i1]);
				const ndVector n0(e0.CrossProduct(e1).Normalize());
				const ndVector n1(e1.CrossProduct(e2).Normalize());
				ndAssert(n0.m_w == ndFloat32(0.0f));
				ndAssert(n1.m_w == ndFloat32(0.0f));
		
				ndAssert(n0.DotProduct(n0).GetScalar() > ndFloat32(0.0f));
				ndAssert(n1.DotProduct(n1).GetScalar() > ndFloat32(0.0f));
		
				// we now check if the two triangles are coplanar
				const ndVector dp(patch.m_pointArray[i3] - patch.m_pointArray[i1]);
				ndAssert(dp.m_w == ndFloat32(0.0f));
				ndFloat32 dist = n0.DotProduct(dp).GetScalar();
		
				if (ndAbs(dist) < ndFloat32(1.0e-3f))
				{
					// triangles are coplanal, so this is a quad
					patch.m_faceArray.PushBack(4);
					patch.m_normalArray.PushBack(n0);
					patch.m_faceMaterialArray.PushBack(0);
					patch.m_indexArray.PushBack(i2);
					patch.m_indexArray.PushBack(i3);
					patch.m_indexArray.PushBack(i1);
					patch.m_indexArray.PushBack(i0);
				}
				else
				{
					// triangles are not coplanal, triangulate the quad
					// into two triangles
					patch.m_faceArray.PushBack(3);
					patch.m_normalArray.PushBack(n0);
					patch.m_faceMaterialArray.PushBack(0);
					patch.m_indexArray.PushBack(i2);
					patch.m_indexArray.PushBack(i1);
					patch.m_indexArray.PushBack(i0);

					patch.m_faceArray.PushBack(3);
					patch.m_normalArray.PushBack(n1);
					patch.m_faceMaterialArray.PushBack(0);
					patch.m_indexArray.PushBack(i1);
					patch.m_indexArray.PushBack(i2);
					patch.m_indexArray.PushBack(i3);
				}
				vertexIndex++;
			}
			vertexIndex++;
		}
	}

	private:
	void CalculateMinExtend3d(const ndVector& p0, const ndVector& p1, ndVector& boxP0, ndVector& boxP1) const
	{
		ndAssert(p0.m_x <= p1.m_x);
		ndAssert(p0.m_y <= p1.m_y);
		ndAssert(p0.m_z <= p1.m_z);
		ndAssert(p0.m_w == ndFloat32(0.0f));
		ndAssert(p1.m_w == ndFloat32(0.0f));

		//boxP0 = (m_invGridSize * (p0 - m_padding).Floor()) * m_gridSize;
		//boxP1 = (m_invGridSize * (p1 + m_gridSize).Floor()) * m_gridSize;

		boxP0 = (m_invGridSize * p0.Floor()) * m_gridSize;
		boxP1 = (m_invGridSize * (p1 + m_padding).Ceiling()) * m_gridSize;

		boxP0.m_y = p0.m_y - m_padding.m_y;
		boxP1.m_y = p1.m_y + m_padding.m_y;

		ndAssert(boxP0.m_x < boxP1.m_x);
		ndAssert(boxP0.m_y < boxP1.m_y);
		ndAssert(boxP0.m_z < boxP1.m_z);
	}

	void CalculateMinAndMaxElevation(ndInt32 x0, ndInt32 x1, ndInt32 z0, ndInt32 z1, ndFloat32& minHeight, ndFloat32& maxHeight) const
	{
		ndReal minVal = ndReal(1.0e10f);
		ndReal maxVal = -ndReal(1.0e10f);

		ndInt32 base = z0 * D_TERRAIN_WIDTH;
		for (ndInt32 z = z0; z <= z1; ++z)
		{
			for (ndInt32 x = x0; x <= x1; ++x)
			{
				ndReal high = m_heightfield[base + x];
				minVal = ndMin(high, minVal);
				maxVal = ndMax(high, maxVal);
			}
			base += D_TERRAIN_WIDTH;
		}
		minHeight = minVal;
		maxHeight = maxVal;
	}

	ndFloat32 RayCastCell(const ndFastRay& ray, ndInt32 xIndex0, ndInt32 zIndex0, ndVector& normalOut, ndFloat32 maxT) const
	{
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
		t = ray.PolygonIntersect(normal, maxT, &points[0].m_x, sizeof(ndVector), triangle, 3);
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
		t = ray.PolygonIntersect(normal, maxT, &points[0].m_x, sizeof(ndVector), triangle, 3);
		if (t < maxT)
		{
			normalOut = normal;
		}
		return t;
	}

	public:
	ndVector m_minBox;
	ndVector m_maxBox;
	ndVector m_padding;
	ndVector m_gridSize;
	ndVector m_invGridSize;

	ndArray<ndInt8> m_material;
	ndArray<ndReal> m_heightfield;
};

class ndHeightfieldMesh : public ndRenderSceneNode
{
	public:
	ndHeightfieldMesh(ndRender* const render, const ndProceduralTerrainShape* const shape, const ndSharedPtr<ndRenderTexture>& texture, const ndMatrix& location)
		:ndRenderSceneNode(location)
	{
		ndMatrix uvMapping(ndGetIdentityMatrix());
		uvMapping[0][0] = 1.0f / 20.0f;
		uvMapping[1][1] = 1.0f / 20.0f;
		uvMapping[2][2] = 1.0f / 20.0f;

		for (ndInt32 z = 0; z < D_TERRAIN_HEIGHT - 1; z += D_TERRAIN_TILE_SIZE)
		{
			for (ndInt32 x = 0; x < D_TERRAIN_WIDTH - 1; x += D_TERRAIN_TILE_SIZE)
			{
				ndSharedPtr<ndShapeInstance> tileShape(BuildTile(shape, x, z));
				ndSharedPtr<ndRenderSceneNode> tileNode(new ndRenderSceneNode(ndGetIdentityMatrix()));
				AddChild(tileNode);

				ndRenderPrimitive::ndDescriptor descriptor(render);
				descriptor.m_collision = tileShape;
				descriptor.m_stretchMaping = false;
				descriptor.m_uvMatrix = uvMapping;
				descriptor.m_mapping = ndRenderPrimitive::m_box;
				ndRenderPrimitiveMaterial& material = descriptor.AddMaterial(texture);
				material.m_castShadows = false;

				ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));
				tileNode->SetPrimitive(mesh);
			}
		}
	}

	private:
	virtual void Render(const ndRender* const owner, const ndMatrix& parentMatrix, ndRenderPassMode renderMode) const override
	{
		// make a tiled rendered node.
		// the terrain is a array of tile subtable for colling,
		// but in this demo we are just rendering the map brute force
		ndRenderSceneNode::Render(owner, parentMatrix, renderMode);
	}

	ndSharedPtr<ndShapeInstance> BuildTile(const ndProceduralTerrainShape* const shape, ndInt32 x0, ndInt32 z0)
	{
		const ndInt32 xMax = ((x0 + D_TERRAIN_TILE_SIZE) >= D_TERRAIN_WIDTH) ? D_TERRAIN_TILE_SIZE - 1: D_TERRAIN_TILE_SIZE + 1;
		const ndInt32 zMax = ((z0 + D_TERRAIN_TILE_SIZE) >= D_TERRAIN_HEIGHT) ? D_TERRAIN_TILE_SIZE - 1: D_TERRAIN_TILE_SIZE + 1;

		// build a collision sub tile
		const ndArray<ndInt8>& materialMap = shape->m_material;
		const ndArray<ndReal>& heightMap = shape->m_heightfield;

		ndPolygonSoupBuilder tileBuilder;
		tileBuilder.Begin();
		for (ndInt32 z = 0; z < zMax; z++)
		{
			const ndReal* const row = &heightMap[(z + z0) * D_TERRAIN_WIDTH];
			const ndInt8* const materialRow = &materialMap[(z + z0) * D_TERRAIN_WIDTH];

			ndVector p0(ndFloat32(x0) * D_TERRAIN_GRID_SIZE, ndFloat32(row[x0]),				   ndFloat32(z0 + z + 0) * D_TERRAIN_GRID_SIZE, ndFloat32(1.0f));
			ndVector p1(ndFloat32(x0) * D_TERRAIN_GRID_SIZE, ndFloat32(row[x0 + D_TERRAIN_WIDTH]), ndFloat32(z0 + z + 1) * D_TERRAIN_GRID_SIZE, ndFloat32(1.0f));
			for (ndInt32 x = 1; x < xMax; x++)
			{
				const ndVector q0(ndFloat32(x0 + x) * D_TERRAIN_GRID_SIZE, ndFloat32(row[x0 + x]),                   ndFloat32(z0 + z + 0) * D_TERRAIN_GRID_SIZE, ndFloat32(1.0f));
				const ndVector q1(ndFloat32(x0 + x) * D_TERRAIN_GRID_SIZE, ndFloat32(row[x0 + x + D_TERRAIN_WIDTH]), ndFloat32(z0 + z + 1) * D_TERRAIN_GRID_SIZE, ndFloat32(1.0f));

				ndVector triangle[3];
				triangle[0] = p0;
				triangle[1] = p1;
				triangle[2] = q0;
				tileBuilder.AddFace(&triangle[0].m_x, sizeof(ndVector), 3, materialRow[x0 + x - 1]);
				
				triangle[0] = p1;
				triangle[1] = q1;
				triangle[2] = q0;
				tileBuilder.AddFace(&triangle[0].m_x, sizeof(ndVector), 3, materialRow[x0 + x]);

				p0 = q0;
				p1 = q1;
			}
		}
		tileBuilder.End(false);
		ndSharedPtr<ndShapeInstance> tileInstance(new ndShapeInstance(new ndShapeStatic_bvh(tileBuilder))); 
		return tileInstance;
	}
};

ndSharedPtr<ndBody> BuildProceduralTerrain(ndDemoEntityManager* const scene, const char* const textureName, const ndMatrix& location)
{
	ndShapeInstance proceduralInstance(new ndProceduralTerrainShape());

	ndProceduralTerrainShape* const heighfield = (ndProceduralTerrainShape*)proceduralInstance.GetShape()->GetAsShapeStaticProceduralMesh();
	
	ndMatrix heighfieldLocation(location);
	const ndVector origin (heighfield->GetObbOrigin());
	heighfieldLocation.m_posit.m_x -= origin.m_x;
	heighfieldLocation.m_posit.m_z -= origin.m_z;
	
	// add tile base sence node
	ndRender* const render = *scene->GetRenderer();
	ndSharedPtr<ndRenderTexture> texture(render->GetTextureCache()->GetTexture(ndGetWorkingFileName(textureName)));
	ndSharedPtr<ndRenderSceneNode> entity(new ndHeightfieldMesh(render, heighfield, texture, heighfieldLocation));
	
	// generate a rigibody and added to the scene and world
	ndPhysicsWorld* const world = scene->GetWorld();
	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(heighfieldLocation);
	body->GetAsBodyDynamic()->SetCollisionShape(proceduralInstance);
	
	world->AddBody(body);
	scene->AddEntity(entity);
	return body;
}