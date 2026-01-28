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

//#define D_TERRAIN_WIDTH		1024
//#define D_TERRAIN_HEIGHT		1024
//#define D_TERRAIN_WIDTH		512
//#define D_TERRAIN_HEIGHT		512
#define D_TERRAIN_WIDTH			256
#define D_TERRAIN_HEIGHT		256

#define D_TERRAIN_NOISE_OCTAVES		8
#define D_TERRAIN_NOISE_PERSISTANCE	0.5f
#define D_TERRAIN_NOISE_GRID_SCALE  (1.0f / 500.0f)

//#define D_TERRAIN_GRID_SIZE		2.0f
#define D_TERRAIN_GRID_SIZE			1.0f
#define D_TERRAIN_TILE_SIZE			128
#define D_TERRAIN_ELEVATION_SCALE	32.0f

class ndProceduralTerrainShape3d : 	public ndShapeStaticProceduralMesh
{
	public:
	D_CLASS_REFLECTION(ndProceduralTerrainShape3d, ndShapeStaticProceduralMesh)

	class ndIsoTerrain: public ndMarchingCubeIsoSurface
	{
		public:
		ndIsoTerrain(ndProceduralTerrainShape3d* const owner, ndDemoEntityManager* const scene)
			:ndMarchingCubeIsoSurface(scene->GetWorld()->GetScene(), ndVector(ndFloat32(-D_TERRAIN_WIDTH / 2)), ndVector(ndFloat32(D_TERRAIN_WIDTH / 2)), D_TERRAIN_GRID_SIZE)
			,m_material()
			,m_heightfield()
			,m_owner(owner)
		{
			MakeNoiseHeightfield();
			GenerateMesh();
		}

		// make a rolling terrain from a 2d noise function
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
					ndReal noiseVal = ndReal(BrownianMotion(octaves, persistance, noiseGridScale * ndFloat32(x), noiseGridScale * ndFloat32(z)));
					//noiseVal = 0.0f;

					m_heightfield[z * D_TERRAIN_WIDTH + x] = noiseVal;
					minHeight = ndMin(minHeight, noiseVal);
					maxHeight = ndMax(maxHeight, noiseVal);

					// that app should populate this with app materials ids.
					// just make a zero material index, for the demo
					m_material[z * D_TERRAIN_WIDTH + x] = 0;
				}
			}
			ndReal scale = D_TERRAIN_ELEVATION_SCALE;

			// resize the aabb
			ndVector boxP0;
			ndVector boxP1;
			GetBox(boxP0, boxP1);
			//need to calculate the min/max of the toroid
			//for now just add +-40 units in each side
			boxP1.m_y = ndCeil(maxHeight * scale + 40.0f);
			boxP0.m_y = ndFloor(minHeight * scale - 40.0f);
			//SetBox(boxP0, boxP1);

			for (ndInt32 i = 0; i < m_heightfield.GetCapacity(); ++i)
			{
				ndReal y = m_heightfield[i];
				//y = m_heightScale * y + m_heightOrigin;
				y = scale * y;
				m_heightfield[i] = y;
			}
		}

		virtual ndReal SampleTorus(const ndVector& posit, ndMatrix& matrix) const
		{
			// draw a torud
			ndFloat32 bigRadius = 40.0f;
			ndFloat32 smallRadius = 5.5f;

			const ndVector step(matrix.UntransformVector(posit));
			ndFloat32 y = ndSqrt(step.m_y * step.m_y + step.m_z * step.m_z) - bigRadius;
			ndFloat32 isoValue = ndSqrt(y * y + step.m_x * step.m_x) - smallRadius;
			return ndReal(isoValue);
		}

		// make a rolling terrain from a 2d noise function
		virtual ndReal GetIsoValue(const ndVector& posit) const override
		{
			//float xxxx = ndSqrt(posit.DotProduct(posit).GetScalar()) - 5.0f;
			//return xxxx;

			const ndVector gridSpace(PositionToGrid(posit));
			ndAssert(gridSpace.m_x >= 0);
			ndAssert(gridSpace.m_z >= 0);
			ndAssert(gridSpace.m_x < D_TERRAIN_WIDTH);
			ndAssert(gridSpace.m_z < D_TERRAIN_WIDTH);

			ndInt32 address = ndInt32(gridSpace.m_z * D_TERRAIN_WIDTH + gridSpace.m_x);
			ndReal heightField = ndReal(posit.m_y - m_heightfield[address]);

			static ndMatrix tunnelMatrix(ndCalculateMatrix(ndYawMatrix(90.0f * ndDegreeToRad), ndVector(20.0f, 0.0f, 0.0f, 1.0f)));
			ndReal tunnelValue = SampleTorus(posit, tunnelMatrix);
			ndReal isoValue = ndMax(heightField, -tunnelValue);

			static ndMatrix bridgeMatrix(ndCalculateMatrix(ndYawMatrix(0.0f * ndDegreeToRad), ndVector(00.0f, 0.0f, 40.0f, 1.0f)));
			ndReal bridgeValue = SampleTorus(posit, bridgeMatrix);
			isoValue = ndMin(isoValue, bridgeValue);

			return isoValue;
		}

		ndArray<ndInt8> m_material;
		ndArray<ndReal> m_heightfield;
		ndProceduralTerrainShape3d* m_owner;
	};

	ndProceduralTerrainShape3d(ndDemoEntityManager* const scene)
		:ndShapeStaticProceduralMesh()
		,m_terrain()
	{
		m_terrain = ndSharedPtr<ndIsoTerrain>(new ndIsoTerrain(this, scene));

		ndVector boxP0;
		ndVector boxP1;
		m_terrain->GetBox(boxP0, boxP1);
		//ndVector p0(ndFloat32(0.0f), minY, ndFloat32(0.0f), ndFloat32(1.0f));
		//ndVector p1(ndFloat32(D_TERRAIN_WIDTH * D_TERRAIN_GRID_SIZE), maxY, ndFloat32(D_TERRAIN_WIDTH * D_TERRAIN_GRID_SIZE), ndFloat32(1.0f));
		SetAABB(boxP0, boxP1);

		// to account for rounding
		//CalculateAabb(ndGetIdentityMatrix(), m_minBox, m_maxBox);
	}

	~ndProceduralTerrainShape3d()
	{
	}

	virtual ndUnsigned64 GetHash(ndUnsigned64 hash) const override
	{
		// return a unique hash code for this shape
		ndInt32 thisHash = 0x48627;
		return ndCRC64(&thisHash, sizeof(ndInt32), hash);
	}

	virtual void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& notify) const override
	{
		const ndArray<ndInt32>& indexList = m_terrain->GetTriangles();
		const ndArray<ndVector>& vertexArray = m_terrain->GetMeshVertex();

		ndVector triangle[3];
		ndShapeDebugNotify::ndEdgeType edgeType = ndShapeDebugNotify::m_shared;
		for (ndInt32 faceIndex = 0; faceIndex < ndInt32(indexList.GetCount()); faceIndex += 3)
		{
			const ndInt32 i0 = indexList[faceIndex + 0];
			const ndInt32 i1 = indexList[faceIndex + 1];
			const ndInt32 i2 = indexList[faceIndex + 2];

			triangle[0] = matrix.TransformVector(vertexArray[i0]);
			triangle[1] = matrix.TransformVector(vertexArray[i1]);
			triangle[2] = matrix.TransformVector(vertexArray[i2]);
			notify.DrawPolygon(3, triangle, &edgeType);
		}
	}

	virtual ndFloat32 RayCast(ndRayCastNotify&, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const, ndContactPoint& contactOut) const override
	{
		//ndFloat32 t = m_terrain->RayCast(localP0, localP1, maxT);
		//return t;
		return 1.2f;
	}

	//void GetFacesPatch(ndPatchMesh& patch) const override
	void GetFacesPatch(ndPatchMesh&) const override
	{
		ndAssert(0);
		//ndAssert(patch.m_convexShapeInstance);
		//// calculate box extend rounded you the padding
		//ndVector boxP0;
		//ndVector boxP1;
		//CalculateMinExtend3d(patch.m_boxP0, patch.m_boxP1, boxP0, boxP1);
		//
		//// clamp sweep box against shape bounds, and get the integer dimension
		//const ndVector intP0((m_invGridSize * boxP0.GetMax(m_minBox)).GetInt());
		//const ndVector intP1((m_invGridSize * boxP1.GetMin(m_maxBox)).GetInt());
		//
		//const ndInt32 x0 = ndInt32(intP0.m_ix);
		//const ndInt32 x1 = ndInt32(intP1.m_ix);
		//const ndInt32 z0 = ndInt32(intP0.m_iz);
		//const ndInt32 z1 = ndInt32(intP1.m_iz);
		//
		//if ((x1 == x0) || (z1 == z0))
		//{
		//	return;
		//}
		//
		//ndFloat32 minHeight = ndFloat32(1.0e10f);
		//ndFloat32 maxHeight = ndFloat32(-1.0e10f);
		//CalculateMinAndMaxElevation(x0, x1, z0, z1, minHeight, maxHeight);
		//
		//if ((maxHeight < boxP0.m_y) || (minHeight > boxP1.m_y))
		//{
		//	// the box does not interset the heightfield
		//	return;
		//}
		//
		//const ndInt32 count_x = x1 - x0;
		//const ndInt32 count_z = z1 - z0;
		//ndInt32 numberOfQuad = (x1 - x0) * (z1 - z0);
		//if (numberOfQuad == 0)
		//{
		//	// box overlap but not faces are collected
		//	return;
		//}
		//
		//// since the vertex pathc has no duplicate, 
		//// we can skip the vertex sorting
		//patch.m_vertexArrayHasDuplicated = false;
		//
		//// if this is a aabb test, we just add one vertex 
		//if (patch.m_queryType == ndPatchMesh::m_vertexListOnly)
		//{
		//	patch.m_pointArray.PushBack(ndVector::m_zero);
		//	return;
		//}
		//
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
	}

	ndSharedPtr<ndIsoTerrain> m_terrain;
};

class ndHeightfieldMesh3d : public ndRenderSceneNode
{
	public:
	ndHeightfieldMesh3d(ndRender* const render, const ndProceduralTerrainShape3d* const shape, const ndSharedPtr<ndRenderTexture>& texture, const ndMatrix& location)
		:ndRenderSceneNode(location)
	{
		for (ndInt32 z = 0; z < D_TERRAIN_HEIGHT - 1; z += D_TERRAIN_TILE_SIZE)
		{
			for (ndInt32 x = 0; x < D_TERRAIN_WIDTH - 1; x += D_TERRAIN_TILE_SIZE)
			{
				ndSharedPtr<ndMeshEffect> tileMesh(BuildTileMesh(shape, x, z));

				ndSharedPtr<ndRenderSceneNode> tileNode(new ndRenderSceneNode(ndGetIdentityMatrix()));
				AddChild(tileNode);
				
				ndRenderPrimitive::ndDescriptor descriptor(render);
				descriptor.m_meshNode = tileMesh;
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

	ndSharedPtr<ndMeshEffect> BuildTileMesh(const ndProceduralTerrainShape3d* const shape, ndInt32 x0, ndInt32 z0)
	{
		const ndArray<ndInt32>& indexList = shape->m_terrain->GetTriangles();
		const ndArray<ndVector>& vertexArray = shape->m_terrain->GetMeshVertex();
		const ndArray<ndVector>& normalArray = shape->m_terrain->GetMeshNormals();
		
		ndFloat32 fx0 = ndFloat32(x0 - D_TERRAIN_WIDTH / 2);
		ndFloat32 fx1 = fx0 + D_TERRAIN_TILE_SIZE;
		
		ndFloat32 fz0 = ndFloat32(z0 - D_TERRAIN_HEIGHT / 2);
		ndFloat32 fz1 = fz0 + D_TERRAIN_TILE_SIZE;
		
		ndVector triangle[3];
		ndArray<ndInt32> vertexIndexArray;
		ndArray<ndInt32> faceMaterialArray;
		ndArray<ndInt32> faceIndexCountArray;
		for (ndInt32 faceIndex = 0; faceIndex < ndInt32(indexList.GetCount()); faceIndex += 3)
		{
			const ndInt32 i0 = indexList[faceIndex + 0];
			const ndInt32 i1 = indexList[faceIndex + 1];
			const ndInt32 i2 = indexList[faceIndex + 2];
			triangle[0] = vertexArray[i0];
			triangle[1] = vertexArray[i1];
			triangle[2] = vertexArray[i2];
		
			bool pass = true;
			pass = pass && (triangle[0].m_x >= fx0);
			pass = pass && (triangle[0].m_x <= fx1);
			pass = pass && (triangle[1].m_x >= fx0);
			pass = pass && (triangle[1].m_x <= fx1);
			pass = pass && (triangle[2].m_x >= fx0);
			pass = pass && (triangle[2].m_x <= fx1);
		
			pass = pass && (triangle[0].m_z >= fz0);
			pass = pass && (triangle[0].m_z <= fz1);
			pass = pass && (triangle[1].m_z >= fz0);
			pass = pass && (triangle[1].m_z <= fz1);
			pass = pass && (triangle[2].m_z >= fz0);
			pass = pass && (triangle[2].m_z <= fz1);
		
			if (pass)
			{
				faceMaterialArray.PushBack(0);
				faceIndexCountArray.PushBack(3);
				vertexIndexArray.PushBack(i0);
				vertexIndexArray.PushBack(i1);
				vertexIndexArray.PushBack(i2);
			}
		}

		ndArray<ndBigVector> meshVertexArray;
		ndArray<ndTriplexReal> meshNormalArray;
		meshNormalArray.SetCount(vertexArray.GetCount());
		meshVertexArray.SetCount(vertexArray.GetCount());
		ndAssert(meshNormalArray.GetCount() == meshVertexArray.GetCount());
		for (ndInt32 i = 0; i < vertexArray.GetCount(); ++i)
		{
			meshVertexArray[i] = vertexArray[i];
			meshNormalArray[i].m_x = ndReal(normalArray[i].m_x);
			meshNormalArray[i].m_y = ndReal(normalArray[i].m_y);
			meshNormalArray[i].m_z = ndReal(normalArray[i].m_z);
		}

		
		ndMeshEffect::ndMeshVertexFormat format;
		
		format.m_faceCount = ndInt32(faceMaterialArray.GetCount());
		format.m_faceMaterial = &faceMaterialArray[0];
		format.m_faceIndexCount = &faceIndexCountArray[0];
		
		format.m_vertex.m_data = &meshVertexArray[0].m_x;
		format.m_vertex.m_indexList = &vertexIndexArray[0];
		format.m_vertex.m_strideInBytes = sizeof(ndBigVector);

		format.m_normal.m_data = &meshNormalArray[0].m_x;
		format.m_normal.m_indexList = &vertexIndexArray[0];
		format.m_normal.m_strideInBytes = 3 * sizeof(ndReal);

		ndSharedPtr<ndMeshEffect> mesh(new ndMeshEffect());
		mesh->BuildFromIndexList(&format);

		ndMatrix uvMapping(ndGetIdentityMatrix());
		uvMapping[0][0] = 1.0f / 20.0f;
		uvMapping[1][1] = 1.0f / 20.0f;
		uvMapping[2][2] = 1.0f / 20.0f;

		mesh->UniformBoxMapping(0, uvMapping);
		return mesh;
	}
};

ndSharedPtr<ndBody> BuildProceduralTerrain3d(ndDemoEntityManager* const scene, const char* const textureName, const ndMatrix& location)
{
	ndShapeInstance proceduralInstance(new ndProceduralTerrainShape3d(scene));
	ndProceduralTerrainShape3d* const heighfield = (ndProceduralTerrainShape3d*)proceduralInstance.GetShape()->GetAsShapeStaticProceduralMesh();
	
	ndMatrix heighfieldLocation(location);
	const ndVector origin (heighfield->GetObbOrigin());
	heighfieldLocation.m_posit.m_x -= origin.m_x;
	heighfieldLocation.m_posit.m_z -= origin.m_z;
	
	// add tile base sence node
	ndRender* const render = *scene->GetRenderer();
	ndSharedPtr<ndRenderTexture> texture(render->GetTextureCache()->GetTexture(ndGetWorkingFileName(textureName)));
	ndSharedPtr<ndRenderSceneNode> entity(new ndHeightfieldMesh3d(render, heighfield, texture, heighfieldLocation));
	
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