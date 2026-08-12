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

#include <ndNewtonInc.h>
#include <gtest/gtest.h>

/* Baseline test: create and destroy an empty Newton world. */
TEST(BuildNdMeshModel, CreateWorld) 
{
  //ndWorld world;
  //world.SetSubSteps(2);
  //world.Update(1.0f / 60.0f);
  //world.Sync();

	// create a mesh using a collision primitive
	ndShapeInstance boxShape(new ndShapeBox(0.5f, 0.5f, 0.5f));
	ndSharedPtr<ndMesh> rootMesh (new ndMesh(boxShape, ndMesh::ndUvMapingMode::m_box));
	ndMeshBodyDynamic* const bodyPtr = new ndMeshBodyDynamic(*rootMesh);
	ndSharedPtr<ndMeshBody> rootBody (bodyPtr);
	bodyPtr->m_shapeInstance = ndMeshShapeInstance(boxShape);
	bodyPtr->m_invMass = 1.0f;
	rootMesh->SetRigidBody(rootBody);

	// create a child mesh and attach to the root.
	ndSharedPtr<ndMesh> childMesh (new ndMesh(boxShape, ndMesh::ndUvMapingMode::m_box));
	rootMesh->AddChild(childMesh);
	ndMatrix location(ndGetIdentityMatrix());
	location.m_posit.m_x = 0.25;
	childMesh->SetMatrix(location);
	ndMeshBodyDynamic* const childBodyPtr = new ndMeshBodyDynamic(*childMesh);
	ndSharedPtr<ndMeshBody> childBody(childBodyPtr);
	childBodyPtr->m_shapeInstance = ndMeshShapeInstance(boxShape);
	childBodyPtr->m_invMass = 1.0f;
	childMesh->SetRigidBody(childBody);

	// glue the node with a fix joint
	ndSharedPtr<ndMeshJoint> joint(new ndMeshJointFix6dof(*childMesh));
	childMesh->SetJoint(joint);

	// at this point we can save the endMesh
	ndMeshLoader loader(rootMesh);
	loader.SaveMesh("tesmMesh.nd");

	// now make a model arculation ready to use by the physics engine 
	ndSharedPtr<ndModel> model(new ndModelArticulation());
	model->GetAsModelArticulation()->Deserialize(*rootMesh);
	
}
