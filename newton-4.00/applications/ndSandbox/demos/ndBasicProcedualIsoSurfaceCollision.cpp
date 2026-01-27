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
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndHeightFieldPrimitive.h"

class MarchingCubeTest : public ndDemoEntityManager::OnPostUpdate
{
	public:
	class ParticlesVolume : public ndMarchingCubesPaticles
	{
		public:
		//#define PARTICLE_SIZE ndFloat32 (1.0f / 50.0f)
		#define PARTICLE_SIZE ndFloat32 (1.0f)
		ParticlesVolume(ndDemoEntityManager* const scene)
			:ndMarchingCubesPaticles(scene->GetWorld()->GetScene(), PARTICLE_SIZE)
		{
			ndMatrix matrix(ndGetIdentityMatrix());
			matrix.m_posit.m_x = 0.25f;
			matrix.m_posit.m_y = 0.5f;
			matrix.m_posit.m_z = 0.75f;
			matrix.m_posit.m_w = 1.0f;

			//BuildBox(matrix, 32);
			//BuildBox(matrix, 20);
			BuildBox(matrix, 4);
		}

		void BuildBox(const ndMatrix& matrix, ndInt32 size)
		{
			ndFloat32 spacing = m_gridSize.m_x;
			//ndFloat32 sigma = spacing * ndFloat32(0.001f);
			//spacing *= ndFloat32(0.9f);
		
			for (ndInt32 z = 0; z < size; z++)
			{
				for (ndInt32 y = 0; y < size; y++)
				{
					for (ndInt32 x = 0; x < size; x++)
					{
						ndFloat32 xf = ndFloat32(x) * spacing;
						ndFloat32 yf = ndFloat32(y) * spacing;
						ndFloat32 zf = ndFloat32(z) * spacing;
		
						ndVector p(matrix.TransformVector(ndVector(xf, yf, zf, ndFloat32(1.0f))));
		
						//ndFloat32 noisex = ndGaussianRandom(ndFloat32(0.0f), sigma);
						//ndFloat32 noisey = ndGaussianRandom(ndFloat32(0.0f), sigma);
						//ndFloat32 noisez = ndGaussianRandom(ndFloat32(0.0f), sigma);
						//p.m_x += noisex;
						//p.m_y += noisey;
						//p.m_z += noisez;
						m_pointParticles.PushBack(p);
					}
				}
			}
		}
	};

	class IsoSurfaceVolume : public ndMarchingCubeIsoSurface
	{
		public:
		//#define PARTICLE_SIZE ndFloat32 (1.0f / 50.0f)
		#define PARTICLE_SIZE ndFloat32 (1.0f)

		IsoSurfaceVolume(ndDemoEntityManager* const scene)
			:ndMarchingCubeIsoSurface(scene->GetWorld()->GetScene(), ndVector(ndFloat32 (-128.0f)), ndVector(ndFloat32(128.0f)), PARTICLE_SIZE)
		{
			//ndMatrix matrix(ndGetIdentityMatrix());
			//matrix.m_posit.m_x = 0.25f;
			//matrix.m_posit.m_y = 0.5f;
			//matrix.m_posit.m_z = 0.75f;
			//matrix.m_posit.m_w = 1.0f;
			//
			////BuildBox(matrix, 32);
			////BuildBox(matrix, 20);
			//BuildBox(matrix, 4);
		}

		ndReal GetIsoValue(const ndVector& posit) const override
		{
			// draw a sphere
			//ndFloat32 radius = 60.0f;
			ndFloat32 radius = 10.5f;
			//ndVector origin(ndVector::m_zero);
			ndVector origin(0.0f);

			ndVector step(posit - origin);
			ndFloat32 dist = ndSqrt (step.DotProduct(step & ndVector::m_triplexMask).GetScalar()) - radius;
			
			if ((posit.m_y >= -0) && (posit.m_y <= 1))
			{
				if ((posit.m_z >= -0) && (posit.m_z <= 1))
				{
					if ((posit.m_x >= -0) && (posit.m_x <= 1))
					{
						dist *= 1;
					}
				}
			}
			return dist;
		}
	};

	MarchingCubeTest(ndDemoEntityManager* const scene)
		:OnPostUpdate()
		,m_isoSurface(scene)
	{
		m_isoSurface.GenerateMesh();
		AddTestMesh(scene);
	}

	void AddTestMesh(ndDemoEntityManager* const scene)
	{
		const ndArray<ndInt32>& indexList = m_isoSurface.GetTriangles();
		const ndArray<ndVector>& vertexArray = m_isoSurface.GetMeshVertex();

		if (!indexList.GetCount())
		{
			return;
		}
		
		ndPolygonSoupBuilder meshBuilder;
		meshBuilder.Begin();
		for (ndInt32 i = 0; i < ndInt32(indexList.GetCount()); i += 3)
		{
			meshBuilder.AddFaceIndirect(&vertexArray[0], 31, &indexList[i], 3);
		}
		meshBuilder.End(false);
		
		ndSharedPtr<ndShapeInstance> collision(new ndShapeInstance(new ndShapeStatic_bvh(meshBuilder)));
		
		ndMatrix location(ndGetIdentityMatrix());
		location.m_posit.m_y = 6.0f;
		location.m_posit.m_x = 0.0f;
		
		ndRenderPrimitive::ndDescriptor descriptor(*scene->GetRenderer());
		descriptor.m_collision = collision;
		descriptor.m_mapping = ndRenderPrimitive::m_box;
		ndRenderPrimitiveMaterial& material = descriptor.AddMaterial(scene->GetRenderer()->GetTextureCache()->GetTexture(ndGetWorkingFileName("metal_30.png")));
		material.m_opacity = ndFloat32(0.5f);
		
		ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));
		ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(location));
		entity->SetPrimitive(mesh);
		
		// make a trigger rigid body and asign mesh and collision
		ndSharedPtr<ndBody> body(new ndBodyKinematic());
		body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
		body->SetMatrix(location);
		body->GetAsBodyKinematic()->SetCollisionShape(**collision);
		scene->GetWorld()->AddBody(body);
		scene->AddEntity(entity);
	}

	void Update(ndDemoEntityManager* const, ndFloat32) override
	{
		//m_surface.GenerateMesh(&m_particleMesh);
	}

	//ParticlesVolume m_isoSurface;
	IsoSurfaceVolume m_isoSurface;
};

void ndBasicProcedualIsoSurfaceCollision(ndDemoEntityManager* const scene)
{
	//ndSharedPtr<ndBody> mapBody(BuildFlatPlane(scene, ndGetIdentityMatrix(), "marblecheckboard.png", true));
	//ndSharedPtr<ndBody> mapBody(BuildProceduralTerrain2d(scene, "grass.png", ndGetIdentityMatrix()));
	ndSharedPtr<ndBody> mapBody(BuildProceduralTerrain3d(scene, "grass.png", ndGetIdentityMatrix()));

	// build a placement matrix
	ndQuaternion rot(ndYawMatrix(0.0f * ndDegreeToRad));
	ndVector origin(10.5f, 0.5f, 0.0f, 1.0f);
	ndVector floor(FindFloor(*scene->GetWorld(), origin, 200.0f));

	ndMatrix originMatrix(ndCalculateMatrix(rot, floor));

	ndSharedPtr<ndDemoEntityManager::OnPostUpdate>marchingCubeMesh(new MarchingCubeTest(scene));
	scene->RegisterPostUpdate(marchingCubeMesh);

	// add single box for testing
	//ndSharedPtr<ndBody> testBody(AddSphere(scene, originMatrix, 1.0f, 0.25f, "wood_0.png"));
	//ndSharedPtr<ndBody> testBody(AddCapsule(scene, originMatrix, 1.0f, 0.5f, 0.5f, 1.0f, "wood_0.png"));
	//ndSharedPtr<ndBody> testBody(AddBox(scene, originMatrix, 1.0f, 1.0f, 1.0f, 1.0f, "wood_0.png"));
	//ndSharedPtr<ndBody> testBody(AddCylinder(scene, originMatrix, 1.0f, 0.5f, 0.5f, 1.0f, "wood_0.png"));
	//ndSharedPtr<ndBody> testBody(AddConvexHull(scene, originMatrix, 40.0f, 0.7f, 1.0f, 10, "wood_0.png"));
	//testBody->SetOmega(ndVector (-10.0f, 0.0f, 0.0f, 0.0f));

	//for (ndInt32 i = 0; i < 20; ++i)
	//{
	//	ndSharedPtr<ndBody> testBody(AddBox(scene, originMatrix, 1.0f, 1.0f, 1.0f, 1.0f, "wood_0.png"));
	//}

	//// add a stack of planks
	//AddPlanks(scene, originMatrix, 1.0f, 4);
	//
	//// add few props
	//originMatrix.m_posit.m_z += 30.0f;
	////originMatrix.m_posit -= originMatrix.m_front.Scale (ndFloat32 (30.0f));
	//AddCapsuleStacks(scene, originMatrix, 10.0f, 0.5f, 0.5f, 1.0f, 10, 10, 7);

	//ndSharedPtr<ndDemoEntityManager::OnPostUpdate>marchingCubeMesh(new MarchingCubeTest(scene));
	//scene->RegisterPostUpdate(marchingCubeMesh);
	
	// set the camera
	originMatrix.m_posit.m_y = 8.0f;
	originMatrix.m_posit.m_x = -23.0f;
	//originMatrix.m_posit.m_z -= 10.0f;
	scene->SetCameraMatrix(rot, originMatrix.m_posit);
}