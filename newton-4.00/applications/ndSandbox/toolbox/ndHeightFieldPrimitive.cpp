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
#define D_TERRAIN_WIDTH				512
#define D_TERRAIN_HEIGHT			512

#define D_TERRAIN_NOISE_OCTAVES		8
#define D_TERRAIN_NOISE_PERSISTANCE	0.5f
#define D_TERRAIN_NOISE_GRID_SCALE  (1.0f / 500.0f)

#define D_TERRAIN_GRID_SIZE			2.0f
#define D_TERRAIN_TILE_SIZE			128
#define D_TERRAIN_ELEVATION_SCALE	32.0f

class ndHeightfieldMesh : public ndRenderSceneNode
{
	public:
	ndHeightfieldMesh(ndDemoEntityManager* const scene, const ndShapeHeightfield* const shape, const ndSharedPtr<ndRenderTexture>& texture, const ndMatrix& location)
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
			for (ndInt32 x = 0; x < D_TERRAIN_WIDTH - 1; x += D_TERRAIN_TILE_SIZE)
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

		// build all tiles in parellel
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

	static void MakeNoiseHeightfield(ndArray<ndVector>& heightfield)
	{
		heightfield.SetCount(D_TERRAIN_WIDTH * D_TERRAIN_HEIGHT);

		const ndInt32 octaves = D_TERRAIN_NOISE_OCTAVES;
		const ndFloat32 cellSize = D_TERRAIN_GRID_SIZE;
		const ndFloat32 persistance = D_TERRAIN_NOISE_PERSISTANCE;
		const ndFloat32 noiseGridScale = D_TERRAIN_NOISE_GRID_SCALE;

		ndFloat32 minHeight = ndFloat32(1.0e10f);
		ndFloat32 maxHight = ndFloat32(-1.0e10f);
		for (ndInt32 z = 0; z < D_TERRAIN_HEIGHT; z++)
		{
			for (ndInt32 x = 0; x < D_TERRAIN_WIDTH; x++)
			{
				ndFloat32 noiseVal = BrownianMotion(octaves, persistance, noiseGridScale * ndFloat32(x), noiseGridScale * ndFloat32(z));
				heightfield[z * D_TERRAIN_WIDTH + x] = ndVector((ndFloat32)x * cellSize, noiseVal, (ndFloat32)z * cellSize, ndFloat32(0.0f));
				minHeight = ndMin(minHeight, noiseVal);
				maxHight = ndMax(maxHight, noiseVal);
			}
		}

		ndFloat32 highScale = D_TERRAIN_ELEVATION_SCALE;
		ndFloat32 scale = ndFloat32(2.0f) / (maxHight - minHeight);
		for (ndInt32 i = 0; i < heightfield.GetCapacity(); ++i)
		{
			ndFloat32 y = heightfield[i].m_y;
			y = scale * (y - minHeight) - ndFloat32(1.0f);
			heightfield[i].m_y *= highScale;
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

	ndSharedPtr<ndMeshEffect> BuildTile(const ndShapeHeightfield* const shape, ndInt32 x0, ndInt32 z0)
	{
		const ndInt32 xMax = ((x0 + D_TERRAIN_TILE_SIZE) >= D_TERRAIN_WIDTH) ? D_TERRAIN_TILE_SIZE - 1 : D_TERRAIN_TILE_SIZE + 1;
		const ndInt32 zMax = ((z0 + D_TERRAIN_TILE_SIZE) >= D_TERRAIN_HEIGHT) ? D_TERRAIN_TILE_SIZE - 1 : D_TERRAIN_TILE_SIZE + 1;

		const ndArray<ndReal>& heightMap = shape->GetElevationMap();
		const ndArray<ndInt8>& materialMap = shape->GetAttributeMap();

		ndArray<ndBigVector> meshVertexArray;
		for (ndInt32 z = 0; z < zMax; z++)
		{
			const ndReal* const row = &heightMap[(z + z0) * D_TERRAIN_WIDTH];
			ndFloat32 zf = ndFloat32(z0 + z) * D_TERRAIN_GRID_SIZE;
			for (ndInt32 x = 0; x < xMax; x++)
			{
				ndFloat32 h = ndFloat32(row[x0 + x]);
				ndFloat32 xf = ndFloat32(x0 + x) * D_TERRAIN_GRID_SIZE;
				const ndBigVector p(xf, h, zf, ndFloat32(1.0f));
				meshVertexArray.PushBack(p);
			}
		}

		ndArray<ndInt32> vertexIndexArray;
		ndArray<ndInt32> faceMaterialArray;
		ndArray<ndInt32> faceIndexCountArray;
		for (ndInt32 z = 0; z < zMax - 1; z++)
		{
			ndInt32 zStart = z * xMax;
			const ndInt8* const materialRow = &materialMap[(z + z0) * D_TERRAIN_WIDTH];
			for (ndInt32 x = 0; x < xMax - 1; x++)
			{
				ndInt32 i0 = zStart + x;
				ndInt32 i1 = zStart + xMax + x;
				ndInt32 i2 = zStart + x + 1;
				ndInt32 i3 = zStart + xMax + x + 1;

				vertexIndexArray.PushBack(i0);
				vertexIndexArray.PushBack(i1);
				vertexIndexArray.PushBack(i2);
				faceIndexCountArray.PushBack(3);
				faceMaterialArray.PushBack(materialRow[x0 + x]);

				vertexIndexArray.PushBack(i1);
				vertexIndexArray.PushBack(i3);
				vertexIndexArray.PushBack(i2);
				faceIndexCountArray.PushBack(3);
				faceMaterialArray.PushBack(materialRow[x0 + x]);
			}
		}

		ndMeshEffect::ndMeshVertexFormat format;

		format.m_faceCount = ndInt32(faceMaterialArray.GetCount());
		format.m_faceMaterial = &faceMaterialArray[0];
		format.m_faceIndexCount = &faceIndexCountArray[0];

		format.m_vertex.m_data = &meshVertexArray[0].m_x;
		format.m_vertex.m_indexList = &vertexIndexArray[0];
		format.m_vertex.m_strideInBytes = sizeof(ndBigVector);

		ndSharedPtr<ndMeshEffect> mesh(new ndMeshEffect());
		mesh->BuildFromIndexList(&format);

		ndMatrix uvMapping(ndGetIdentityMatrix());
		uvMapping[0][0] = 1.0f / 20.0f;
		uvMapping[1][1] = 1.0f / 20.0f;
		uvMapping[2][2] = 1.0f / 20.0f;

		mesh->CalculateNormals(60 * ndDegreeToRad);
		mesh->UniformBoxMapping(0, uvMapping);
		return mesh;
	}
};

ndSharedPtr<ndBody> BuildHeightFieldTerrain(ndDemoEntityManager* const scene, const char* const textureName, const ndMatrix& location)
{
	ndArray<ndVector> heightfield(D_TERRAIN_WIDTH * D_TERRAIN_HEIGHT);
	ndHeightfieldMesh::MakeNoiseHeightfield(heightfield);

	// create the height field collision and rigid body
	ndShapeInstance heighfieldInstance(new ndShapeHeightfield(D_TERRAIN_WIDTH, D_TERRAIN_WIDTH,
			ndShapeHeightfield::m_invertedDiagonals, D_TERRAIN_GRID_SIZE, D_TERRAIN_GRID_SIZE));
	//ndShapeInstance heighfieldInstance(new ndShapeHeightfield(D_TERRAIN_WIDTH, D_TERRAIN_WIDTH,
	//	ndShapeHeightfield::m_normalDiagonals, D_TERRAIN_GRID_SIZE, D_TERRAIN_GRID_SIZE));
	
	ndShapeHeightfield* const heighfield = heighfieldInstance.GetShape()->GetAsShapeHeightfield();
	ndArray<ndReal>& heightMap = heighfield->GetElevationMap();
	ndAssert(heightMap.GetCount() == heightfield.GetCount());
	for (ndInt32 i = 0; i < heightfield.GetCount(); ++i)
	{
		ndFloat32 high = heightfield[i].m_y;
		heightMap[i] = ndReal(high);
	}
	heighfield->UpdateElevationMapAabb();

	ndMatrix heighfieldLocation(location);
	heighfieldLocation.m_posit.m_x -= 0.5f * ndFloat32(heighfield->GetWith()) * heighfield->GetWithScale();
	heighfieldLocation.m_posit.m_z -= 0.5f * ndFloat32(heighfield->GetHeight()) * heighfield->GetHeightScale();

	// add tile base sence node
	ndSharedPtr<ndRenderTexture> texture(scene->GetRenderer()->GetTextureCache()->GetTexture(ndGetWorkingFileName(textureName)));
	ndUnsigned64 time = ndGetTimeInMicroseconds();
	ndSharedPtr<ndRenderSceneNode> entity(new ndHeightfieldMesh(scene, heighfield, texture, heighfieldLocation));
	time = ndGetTimeInMicroseconds() - time;
	ndExpandTraceMessage("%s: build time %g (sec)\n", __FUNCTION__, ndFloat32(time) * ndFloat32(1.0e-6f));
	
	// generate a rigibody and added to the scene and world
	ndPhysicsWorld* const world = scene->GetWorld();
	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(heighfieldLocation);
	body->GetAsBodyDynamic()->SetCollisionShape(heighfieldInstance);
	
	world->AddBody(body);
	scene->AddEntity(entity);
	return body;
}