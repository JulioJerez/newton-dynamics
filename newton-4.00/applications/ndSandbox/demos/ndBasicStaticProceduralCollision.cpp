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

void ndBasicProcedualStaticCollision(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildProceduralTerrain(scene, "grass.png", ndGetIdentityMatrix()));

	// build a placemnet matrix
	ndQuaternion rot(ndYawMatrix(180.0f * ndDegreeToRad));
	ndVector origin(10.0f, 0.0f, 0.0f, 1.0f);
	ndVector floor(FindFloor(*scene->GetWorld(), origin, 200.0f));

	ndMatrix originMatrix(ndCalculateMatrix(rot, floor));

	// add single box for testing
	ndSharedPtr<ndBody> testBody(AddSphere(scene, originMatrix, 1.0f, 0.25f, "wood_0.png"));
	//ndSharedPtr<ndBody> testBody(AddCapsule(scene, originMatrix, 1.0f, 0.5f, 0.5f, 1.0f, "wood_0.png"));
	//ndSharedPtr<ndBody> testBody(AddBox(scene, originMatrix, 1.0f, 1.0f, 1.0f, 1.0f, "wood_0.png"));
	//ndSharedPtr<ndBody> testBody(AddCylinder(scene, originMatrix, 1.0f, 0.5f, 0.5f, 1.0f, "wood_0.png"));
	//ndSharedPtr<ndBody> testBody(AddConvexHull(scene, originMatrix, 40.0f, 0.7f, 1.0f, 10, "wood_0.png"));
	testBody->SetOmega(ndVector (-10.0f, 0.0f, 0.0f, 0.0f));

	//for (ndInt32 i = 0; i < 20; ++i)
	//{
	//	ndSharedPtr<ndBody> testBody(AddBox(scene, originMatrix, 1.0f, 1.0f, 1.0f, 1.0f, "wood_0.png"));
	//}


	////// add few props
	//originMatrix.m_posit += originMatrix.m_front.Scale (ndFloat32 (40.0f));
	//AddCapsuleStacks(scene, originMatrix, 10.0f, 0.5f, 0.5f, 1.0f, 10, 10, 7);
	//
	//originMatrix.m_posit += originMatrix.m_right.Scale(20.0f);
	//AddPlanks(scene, originMatrix, 1.0f, 4);

	// set the camera
	floor.m_y += 1.0f;
	floor.m_x += 4.0f;
	scene->SetCameraMatrix(rot, floor);
}
