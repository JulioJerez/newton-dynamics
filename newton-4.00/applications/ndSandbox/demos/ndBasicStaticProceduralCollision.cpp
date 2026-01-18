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
	class MatchingCubeParticle : public ndMarchingCubeParticleIsoValue
	{
		public:
		MatchingCubeParticle(ndDemoEntityManager* const scene)
			:ndMarchingCubeParticleIsoValue(scene->GetWorld()->GetScene(), ndFloat32(1.0f))
		{
			ndMatrix matrix(ndGetIdentityMatrix());
			BuildBox(matrix, 10);
		}

		void BuildBox(const ndMatrix& matrix, ndInt32 size)
		{
			ndFloat32 spacing = m_gridSize * ndFloat32(0.9f);
			ndFloat32 sigma = spacing * ndFloat32(0.01f);

			//ndVector v(ndVector::m_zero);
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

						p.m_x = ndGaussianRandom(p.m_x, sigma);
						p.m_y = ndGaussianRandom(p.m_y, sigma);
						p.m_z = ndGaussianRandom(p.m_z, sigma);
						m_points.PushBack(p);
					}
				}
			}
		}
	};

	MarchingCubeTest(ndDemoEntityManager* const scene)
		:OnPostUpdate()
		,m_surface()
		,m_particleMesh(scene)
	{
	}

	void Update(ndDemoEntityManager* const, ndFloat32) override
	{
		m_surface.GenerateMesh(&m_particleMesh);
	}

	ndMarchingCubes m_surface;
	MatchingCubeParticle m_particleMesh;
};


void ndBasicProcedualHeightfieldCollision(ndDemoEntityManager* const scene)
{
	//ndSharedPtr<ndBody> mapBody(BuildProceduralTerrain(scene, "grass.png", ndGetIdentityMatrix()));
	//ndSharedPtr<ndBody> mapBody(BuildFlatPlane(scene, ndGetIdentityMatrix(), "blueCheckerboard.png", true));
	ndSharedPtr<ndBody> mapBody(BuildFlatPlane(scene, ndGetIdentityMatrix(), "marblecheckboard.png", true));

	// build a placement matrix
	ndQuaternion rot(ndYawMatrix(180.0f * ndDegreeToRad));
	ndVector origin(10.5f, 0.5f, 0.0f, 1.0f);
	ndVector floor(FindFloor(*scene->GetWorld(), origin, 200.0f));

	ndMatrix originMatrix(ndCalculateMatrix(rot, floor));

	// add single box for testing
	//ndSharedPtr<ndBody> testBody(AddSphere(scene, originMatrix, 1.0f, 0.25f, "wood_0.png"));
	ndSharedPtr<ndBody> testBody(AddCapsule(scene, originMatrix, 1.0f, 0.5f, 0.5f, 1.0f, "wood_0.png"));
	//ndSharedPtr<ndBody> testBody(AddBox(scene, originMatrix, 1.0f, 1.0f, 1.0f, 1.0f, "wood_0.png"));
	//ndSharedPtr<ndBody> testBody(AddCylinder(scene, originMatrix, 1.0f, 0.5f, 0.5f, 1.0f, "wood_0.png"));
	//ndSharedPtr<ndBody> testBody(AddConvexHull(scene, originMatrix, 40.0f, 0.7f, 1.0f, 10, "wood_0.png"));
	testBody->SetOmega(ndVector (-10.0f, 0.0f, 0.0f, 0.0f));

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

	ndSharedPtr<ndDemoEntityManager::OnPostUpdate>marchingCubeMesh(new MarchingCubeTest(scene));
	scene->RegisterPostUpdate(marchingCubeMesh);
	
	// set the camera
	originMatrix.m_posit.m_y += 8.0f;
	originMatrix.m_posit.m_x += 40.0f;
	originMatrix.m_posit.m_z -= 10.0f;
	scene->SetCameraMatrix(rot, originMatrix.m_posit);
}