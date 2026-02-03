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
#include "ndDebugDisplayRenderPass.h"

//#define D_TERRAIN__WIDTH			1024
//#define D_TERRAIN_HEIGHT			1024
#define D_TERRAIN__WIDTH			512
#define D_TERRAIN_HEIGHT			512
//#define D_TERRAIN__WIDTH			256
//#define D_TERRAIN_HEIGHT			256

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
			:ndMarchingCubeIsoSurface(scene->GetWorld()->GetScene(), ndVector(ndFloat32(-D_TERRAIN__WIDTH / 2)), ndVector(ndFloat32(D_TERRAIN__WIDTH / 2)), D_TERRAIN_GRID_SIZE)
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
			m_material.SetCount(D_TERRAIN__WIDTH * D_TERRAIN_HEIGHT);
			m_heightfield.SetCount(D_TERRAIN__WIDTH * D_TERRAIN_HEIGHT);

			const ndInt32 octaves = D_TERRAIN_NOISE_OCTAVES;
			const ndFloat32 persistance = D_TERRAIN_NOISE_PERSISTANCE;
			const ndFloat32 noiseGridScale = D_TERRAIN_NOISE_GRID_SCALE;

			ndReal minHeight = ndFloat32(1.0e10f);
			ndReal maxHeight = ndFloat32(-1.0e10f);
			for (ndInt32 z = 0; z < D_TERRAIN_HEIGHT; z++)
			{
				for (ndInt32 x = 0; x < D_TERRAIN__WIDTH; x++)
				{
					ndReal noiseVal = ndReal(BrownianMotion(octaves, persistance, noiseGridScale * ndFloat32(x), noiseGridScale * ndFloat32(z)));
					//noiseVal = 0.0f;

					m_heightfield[z * D_TERRAIN__WIDTH + x] = noiseVal;
					minHeight = ndMin(minHeight, noiseVal);
					maxHeight = ndMax(maxHeight, noiseVal);

					// that app should populate this with app materials ids.
					// just make a zero material index, for the demo
					m_material[z * D_TERRAIN__WIDTH + x] = 0;
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
			const ndVector gridSpace(PositionToGridSpace(posit));
			ndAssert(gridSpace.m_x >= 0);
			ndAssert(gridSpace.m_z >= 0);
			ndAssert(gridSpace.m_x < D_TERRAIN__WIDTH);
			ndAssert(gridSpace.m_z < D_TERRAIN__WIDTH);

			ndInt32 address = ndInt32(gridSpace.m_z * D_TERRAIN__WIDTH + gridSpace.m_x);
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
		,m_owner(scene)
	{
		m_terrain = ndSharedPtr<ndIsoTerrain>(new ndIsoTerrain(this, scene));

		ndVector boxP0;
		ndVector boxP1;
		m_terrain->GetBox(boxP0, boxP1);
		SetAABB(boxP0, boxP1);
	}

	~ndProceduralTerrainShape3d()
	{
	}

	virtual ndUnsigned64 GetHash(ndUnsigned64 hash) const override
	{
		// return a unique hash code for this shape
		ndInt32 thisHash = 0x486F27;
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
		return m_terrain->RayCast(localP0, localP1, maxT, contactOut);
	}

	void ShowDebugFaces(ndPatchMesh& patch) const
	{
		ndRenderPassDebug* const debugRenderPass = m_owner->GetDebugRenderPass();
		const ndRenderPassDebug::ndDebugOptions& options = debugRenderPass->GetDebugDisplayOptions();
		if (options.m_showStaticMeshCollidingFaces)
		{
			if (patch.m_pointArray.GetCount() > 1)
			{
				ndInt32 sum = 0;
				const ndVector color(1.0f, 1.0f, 0.0f, 1.0f);
				const ndMatrix& matrix(patch.m_worldMatrix);
				for (ndInt32 i = 0; i < ndInt32(patch.m_faceArray.GetCount()); ++i)
				{
					const ndInt32 faceCount = patch.m_faceArray[i];
					ndInt32 index = patch.m_indexArray[sum + faceCount - 1];
					ndVector p0(matrix.TransformVector(patch.m_pointArray[index]));
					for (ndInt32 j = 0; j < faceCount; ++j)
					{
						index = patch.m_indexArray[sum + j];
						const ndVector p1(matrix.TransformVector(patch.m_pointArray[index]));
						debugRenderPass->AddRuntimeLine(p0, p1, color);
						p0 = p1;
					}
					sum += faceCount;
				}
			}

			ndVector box[12][2];
			const ndVector p0(patch.m_boxP0);
			const ndVector p1(patch.m_boxP1);

			box[0][0] = ndVector(p0.m_x, p0.m_y, p0.m_z, ndFloat32(1.0f));
			box[0][1] = ndVector(p1.m_x, p0.m_y, p0.m_z, ndFloat32(1.0f));

			box[1][0] = ndVector(p0.m_x, p1.m_y, p0.m_z, ndFloat32(1.0f));
			box[1][1] = ndVector(p1.m_x, p1.m_y, p0.m_z, ndFloat32(1.0f));

			box[2][0] = ndVector(p0.m_x, p1.m_y, p1.m_z, ndFloat32(1.0f));
			box[2][1] = ndVector(p1.m_x, p1.m_y, p1.m_z, ndFloat32(1.0f));

			box[3][0] = ndVector(p0.m_x, p0.m_y, p1.m_z, ndFloat32(1.0f));
			box[3][1] = ndVector(p1.m_x, p0.m_y, p1.m_z, ndFloat32(1.0f));

			box[4][0] = ndVector(p0.m_x, p0.m_y, p0.m_z, ndFloat32(1.0f));
			box[4][1] = ndVector(p0.m_x, p1.m_y, p0.m_z, ndFloat32(1.0f));

			box[5][0] = ndVector(p1.m_x, p0.m_y, p0.m_z, ndFloat32(1.0f));
			box[5][1] = ndVector(p1.m_x, p1.m_y, p0.m_z, ndFloat32(1.0f));

			box[6][0] = ndVector(p0.m_x, p0.m_y, p1.m_z, ndFloat32(1.0f));
			box[6][1] = ndVector(p0.m_x, p1.m_y, p1.m_z, ndFloat32(1.0f));

			box[7][0] = ndVector(p1.m_x, p0.m_y, p1.m_z, ndFloat32(1.0f));
			box[7][1] = ndVector(p1.m_x, p1.m_y, p1.m_z, ndFloat32(1.0f));

			box[8][0] = ndVector(p0.m_x, p0.m_y, p0.m_z, ndFloat32(1.0f));
			box[8][1] = ndVector(p0.m_x, p0.m_y, p1.m_z, ndFloat32(1.0f));

			box[9][0] = ndVector(p1.m_x, p0.m_y, p0.m_z, ndFloat32(1.0f));
			box[9][1] = ndVector(p1.m_x, p0.m_y, p1.m_z, ndFloat32(1.0f));

			box[10][0] = ndVector(p0.m_x, p1.m_y, p0.m_z, ndFloat32(1.0f));
			box[10][1] = ndVector(p0.m_x, p1.m_y, p1.m_z, ndFloat32(1.0f));

			box[11][0] = ndVector(p1.m_x, p1.m_y, p0.m_z, ndFloat32(1.0f));
			box[11][1] = ndVector(p1.m_x, p1.m_y, p1.m_z, ndFloat32(1.0f));

			const ndVector boxColor(1.0f, 0.0f, 1.0f, 1.0f);
			for (ndInt32 i = 0; i < 12; ++i)
			{
				//DrawLine(box[i][0], box[i][1], color, ndFloat32(1.0f));
				debugRenderPass->AddRuntimeLine(box[i][0], box[i][1], boxColor);
			}
		}
	}

	void GetFacesPatch(ndPatchMesh& patch) const override
	{
		m_terrain->GetFacesPatch(patch);
		ShowDebugFaces(patch);
	}

	ndSharedPtr<ndIsoTerrain> m_terrain;
	ndWeakPtr<ndDemoEntityManager> m_owner;
};

class ndHeightfieldMesh3d : public ndRenderSceneNode
{
	public:
	ndHeightfieldMesh3d(ndDemoEntityManager* const scene, const ndProceduralTerrainShape3d* const shape, const ndSharedPtr<ndRenderTexture>& texture, const ndMatrix& location)
		:ndRenderSceneNode(location)
	{
		ndRender* const render = *scene->GetRenderer();

		struct TilePosit
		{
			ndInt32 m_x;
			ndInt32 m_z;
		};
		ndFixSizeArray<TilePosit, 256> tilesOrigin(0);
		ndFixSizeArray<ndSharedPtr<ndMeshEffect>, 256> tileSlots(0);

		for (ndInt32 z = 0; z < D_TERRAIN_HEIGHT - 1; z += D_TERRAIN_TILE_SIZE)
		{
			for (ndInt32 x = 0; x < D_TERRAIN__WIDTH - 1; x += D_TERRAIN_TILE_SIZE)
			{
				TilePosit posit;
				posit.m_x = x;
				posit.m_z = z;
				tilesOrigin.PushBack(posit);
				tileSlots.PushBack(ndSharedPtr<ndMeshEffect>(nullptr));
			}
		}

		auto BuildTiles = ndMakeObject::ndFunction([this, shape, &tilesOrigin, &tileSlots](ndInt32 groupId, ndInt32)
		{
			const TilePosit& posit = tilesOrigin[groupId];
			tileSlots[groupId] = BuildTile(shape, posit.m_x, posit.m_z);
		});

		// build all tiles in parallel
		ndThreadPool* const threadPool = scene->GetWorld()->GetScene();

		threadPool->Begin();
		threadPool->ParallelExecute(BuildTiles, ndInt32(tileSlots.GetCount()), 1);
		threadPool->End();

		// add each tile to the scene for visualization
		for (ndInt32 i = 0; i < ndInt32(tileSlots.GetCount()); ++i)
		{
			ndSharedPtr<ndMeshEffect> tileMesh(tileSlots[i]);
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

	private:
	virtual void Render(const ndRender* const owner, const ndMatrix& parentMatrix, ndRenderPassMode renderMode) const override
	{
		// make a tiled rendered node.
		// the terrain is a array of tile subtable for colling,
		// but in this demo we are just rendering the map brute force
		ndRenderSceneNode::Render(owner, parentMatrix, renderMode);
	}

	ndSharedPtr<ndMeshEffect> BuildTile(const ndProceduralTerrainShape3d* const shape, ndInt32 x0, ndInt32 z0)
	{
		const ndArray<ndInt32>& indexList = shape->m_terrain->GetTriangles();
		const ndArray<ndVector>& vertexArray = shape->m_terrain->GetMeshVertex();
		const ndArray<ndVector>& normalArray = shape->m_terrain->GetMeshNormals();
		
		ndFloat32 fx0 = ndFloat32(x0 - D_TERRAIN__WIDTH / 2);
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

ndSharedPtr<ndBody> BuildMarchingCubeHeighfield(ndDemoEntityManager* const scene, const char* const textureName, const ndMatrix& location)
{
	ndShapeInstance proceduralInstance(new ndProceduralTerrainShape3d(scene));
	ndProceduralTerrainShape3d* const heighfield = (ndProceduralTerrainShape3d*)proceduralInstance.GetShape()->GetAsShapeStaticProceduralMesh();
	
	ndMatrix heighfieldLocation(location);
	const ndVector origin (heighfield->GetObbOrigin());
	heighfieldLocation.m_posit.m_x -= origin.m_x;
	heighfieldLocation.m_posit.m_z -= origin.m_z;
	
	// add tile base sence node
	ndSharedPtr<ndRenderTexture> texture(scene->GetRenderer()->GetTextureCache()->GetTexture(ndGetWorkingFileName(textureName)));

	ndUnsigned64 time = ndGetTimeInMicroseconds();
	ndSharedPtr<ndRenderSceneNode> entity(new ndHeightfieldMesh3d(scene, heighfield, texture, heighfieldLocation));
	time = ndGetTimeInMicroseconds() - time;
	ndExpandTraceMessage("%s: build time %g (sec)\n", __FUNCTION__, ndFloat32(time) * ndFloat32(1.0e-6f));

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