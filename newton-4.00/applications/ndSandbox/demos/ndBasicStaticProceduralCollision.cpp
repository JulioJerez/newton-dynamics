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
	ndVector floor(FindFloor(*scene->GetWorld(), ndVector::m_zero, 200.0f));

	// no ray case yet
	floor.m_y = 3.25f;
	floor.m_x += 1.0f;

	ndMatrix origin(ndCalculateMatrix(rot, floor));

	//// add few props
	//origin.m_posit += origin.m_front.Scale (ndFloat32 (40.0f));
	//AddCapsuleStacks(scene, origin, 10.0f, 0.5f, 0.5f, 1.0f, 10, 10, 7);
	//
	//origin.m_posit += origin.m_right.Scale(20.0f);
	//AddPlanks(scene, origin, 1.0f, 4);

	ndSharedPtr<ndBody> xxx(AddBox(scene, origin, 1.0f, 1.0f, 0.5f, 2.0f, "wood_0.png"));
	ndMatrix xxxxx(xxx->GetMatrix());
	xxxxx.m_posit.m_y = floor.m_y;
	xxx->SetMatrix(xxxxx);
	//xxx->GetAsBodyKinematic()->SetMatrixUpdateScene(xxxxx);
	//xxx->GetAsBodyKinematic()->SetAutoSleep(false);
	//xxx->GetNotifyCallback()->OnTransform(0.0f, xxxxx);
	
	//floor.m_y += 15.0f;
	floor = xxxxx.m_posit;
	floor.m_y += 1.0f;
	floor.m_x += 8.0f;
	scene->SetCameraMatrix(rot, floor);
}
