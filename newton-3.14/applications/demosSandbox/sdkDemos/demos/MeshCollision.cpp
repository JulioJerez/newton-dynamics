/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "toolbox_stdafx.h"
#include "SkyBox.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"

static void SimpleMeshLevel (DemoEntityManager* const scene, bool optimization)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
//	CreateLevelMesh (scene, "flatPlane.ngd", optimization);
	CreateLevelMesh (scene, "sponza.ngd", optimization);

//	dMatrix camMatrix (dRollMatrix(-20.0f * dDegreeToRad) * dYawMatrix(-45.0f * dDegreeToRad));
	dMatrix camMatrix (dGetIdentityMatrix());
	dQuaternion rot (camMatrix);
	dVector origin (-40.0f, 40.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);

	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (world);
	dVector location (0.0f, 0.0f, 0.0f, 0.0f);
	dVector size (0.25f, 0.25f, 0.5f, 0.0f);
	size = size.Scale (2.0f);

	int count = 5;
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
	for (int i = 0; i < 20; i ++) {
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _COMPOUND_CONVEX_CRUZ_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	}

	count = 8;
	for (int i = 0; i < 50; i ++){
	//AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	//AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	}
}

void OptimizedMeshLevelCollision (DemoEntityManager* const scene)
{
	SimpleMeshLevel (scene, true);
}

void SimpleMeshLevelCollision (DemoEntityManager* const scene)
{
#if 0
	SimpleMeshLevel (scene, false);
#else

	// load the skybox
	scene->CreateSkyBox();
	// load the scene from a ngd file format
	//CreateLevelMesh (scene, "flatPlane.ngd", false);

	char fileName[2048];
	dGetWorkingFileName("leadwerktest.ngd", fileName);
	scene->LoadScene(fileName);

	NewtonWorld* const world = scene->GetNewton();
	for (DemoEntityManager::dListNode* node = scene->GetLast(); node; node = node->GetPrev()) 
	{
		DemoEntity* const ent = node->GetInfo();
		for (DemoEntity* child = ent->GetChild(); child; child = child->GetSibling())
		{
			//const dString& name = child->GetName();
			//bool test = (name == "Box");
			//test = test || (name == "Cylinder");
			bool test = true;

			if (test && child->GetMesh())
			{
				NewtonCollision* const collision = child->CreateConvexHull(world);
				dAssert(child->GetMesh()->IsType(DemoMesh::GetRttiType()));

				dMatrix matrix (child->CalculateGlobalMatrix());
				NewtonBody* const rigidBody = NewtonCreateDynamicBody(world, collision, &matrix[0][0]);

				// set a destructor for this rigid body
				NewtonBodySetDestructorCallback(rigidBody, PhysicsBodyDestructor);

				// set the transform call back function
				NewtonBodySetTransformCallback(rigidBody, DemoEntity::TransformCallback);

				// set the force and torque call back function
				//NewtonBodySetForceAndTorqueCallback(rigidBody, PhysicsApplyGravityForce);

				NewtonDestroyCollision(collision);
			}
		}
	}

	// add a upright cylider.

	dVector size(0.7f, 1.8f, 0.7f, 0.0f);
	NewtonCollision* const collision = CreateConvexCollision(world, dRollMatrix(90.0f * dDegreeToRad), size, _CYLINDER_PRIMITIVE, 0);
	//dVector size(1.0);
	//NewtonCollision* const collision = CreateConvexCollision(world, dRollMatrix(00.0f * dDegreeToRad), size, _SPHERE_PRIMITIVE, 0);
	//NewtonCollision* const collision = CreateConvexCollision(world, dRollMatrix(00.0f * dDegreeToRad), size, _BOX_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_x = 10.0f;
	matrix.m_posit.m_z = 4.0f;
	matrix.m_posit.m_y = 2.0f;

	NewtonBody* body = CreateSimpleSolid(scene, geometry, 10.0f, matrix, collision, 0);
	// do not forget to release the assets	
	geometry->Release();
	NewtonDestroyCollision(collision);


	dMatrix camMatrix(dGetIdentityMatrix());
	dQuaternion rot(camMatrix);
	dVector origin(-0.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);

#endif
}