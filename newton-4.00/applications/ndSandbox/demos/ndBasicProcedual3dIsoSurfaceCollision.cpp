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


void ndBasicMarchingCube3dCollision(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildMarchingCubeHeighfield(scene, "grass.png", ndGetIdentityMatrix()));

	// build a placement matrix
	const ndQuaternion rot(ndYawMatrix(0.0f * ndDegreeToRad));
	const ndVector origin(0.5f, 0.0f, -0.0f, 1.0f);
	const ndVector floor(FindFloor(*scene->GetWorld(), origin, 400.0f));
	ndMatrix originMatrix(ndCalculateMatrix(rot, floor));

	// add single box for testing
	//ndSharedPtr<ndBody> testBody(AddSphere(scene, originMatrix, 1.0f, 0.25f, "wood_0.png"));
	//ndSharedPtr<ndBody> testBody(AddCapsule(scene, originMatrix, 1.0f, 0.5f, 0.5f, 1.0f, "wood_0.png"));
	ndSharedPtr<ndBody> testBody(AddBox(scene, originMatrix, 1.0f, 1.0f, 1.0f, 1.0f, "wood_0.png"));
	//ndSharedPtr<ndBody> testBody(AddCylinder(scene, originMatrix, 1.0f, 0.5f, 0.5f, 1.0f, "wood_0.png"));
	//ndSharedPtr<ndBody> testBody(AddConvexHull(scene, originMatrix, 40.0f, 0.7f, 1.0f, 10, "wood_0.png"));
	//testBody->SetOmega(ndVector (-10.0f, 0.0f, 0.0f, 0.0f));

	// add a stack of planks
	originMatrix.m_posit.m_z += 3.0f;
	AddPlanks(scene, originMatrix, 1.0f, 4);
	
	// add few props
	originMatrix.m_posit.m_z += 15.0f;
	AddCapsuleStacks(scene, originMatrix, 10.0f, 0.5f, 0.5f, 1.0f, 10, 10, 7);

	// set the camera
	originMatrix.m_posit.m_x -= 30.0f;
	originMatrix.m_posit = FindFloor(*scene->GetWorld(), originMatrix.m_posit, 400.0f);
	originMatrix.m_posit.m_y += 3;
	scene->SetCameraMatrix(rot, originMatrix.m_posit);
}
