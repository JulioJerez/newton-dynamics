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
#include "ndDemoCameraNodeFollow.h"
#include "ndHeightFieldPrimitive.h"

namespace ndRagdoll
{
	class ndRagDollControl : public ndJointUserData
	{
		public:
		ndRagDollControl(ndJointBilateralConstraint* const owner, ndModelArticulation* const appInterface)
			:ndJointUserData(owner)
			,m_model(appInterface)
		{
	
		}
	
		virtual void ApplyControl() = 0;
	
		ndWeakPtr<ndModelArticulation> m_model;
	};
	
	class ndMakeRagDoll : public ndRagDollControl
	{
		public:
		ndMakeRagDoll(ndJointBilateralConstraint* const owner, ndModelArticulation* const appInterface)
			:ndRagDollControl(owner, appInterface)
		{
		}
	
		virtual void ApplyControl() override
		{
			// TO DO: apply specialized application contoll here.
		}
	};


	class ndRagDollController : public ndModelNotify
	{ 
		public:
		ndRagDollController()
			:ndModelNotify()
		{
		}

		bool OnContactGeneration(const ndBodyKinematic* const, const ndBodyKinematic* const) override
		{
			// here the application can use filter to determine what body parts should collide.
			// this greatly improves performance because since articulated models,
			// in general do not self collide, but occasionally some parts do collide. 
			// for now we just return false (no collision)
			return false;
		}
	};

	ndSharedPtr<ndModelNotify> CreateRagdoll(ndDemoEntityManager* const scene, const ndRenderMeshLoader& loader, const ndMatrix& location)
	{
		// make a hierchical aticulated model
		ndSharedPtr<ndModel> model(new ndModelArticulation());
		
		// create a ragdoll controller 
		ndSharedPtr<ndModelNotify> controller(new ndRagDollController());
		model->SetNotifyCallback(controller);
		
		// create a model arculation from the mesh
		ndModelArticulation* const ragdoll = model->GetAsModelArticulation();
		ragdoll->Deserialize(*loader.m_mesh);

		// create a copy of the visual mesh
		ndSharedPtr<ndRenderSceneNode> visualMesh(loader.m_renderMesh->Clone());

		// bind the graphics model to the physics model,
		// also apply any application constomization.
		auto BindApplicationData = [scene, &ragdoll, &visualMesh](ndModelArticulation::ndNode* const node)
		{
			ndRenderSceneNode* const visualEntityPtr = visualMesh->FindByClosestMatch(node->m_name);
			ndAssert(visualEntityPtr);
			ndSharedPtr<ndRenderSceneNode> visualEntity((visualEntityPtr == *visualMesh) ? visualMesh : visualEntityPtr->GetSharedPtr());

			// add a rigid body with notification callback
			ndBodyKinematic* const parentBody = node->GetParent() ? node->GetParent()->m_body->GetAsBodyKinematic() : nullptr;
			ndSharedPtr<ndBodyNotify> notify(new ndDemoEntityNotify(scene, visualEntity, parentBody));
			node->m_body->SetNotifyCallback(notify);

			if (node->m_joint)
			{
				ndSharedPtr<ndJointUserData> userData(new ndMakeRagDoll(*node->m_joint, ragdoll));
				node->m_joint->SetUserData(userData);
			}
		};
		ragdoll->NodeIterator(BindApplicationData);

		// apply the global transforms to the visual mesh and physics model
		const ndMatrix matrix(ragdoll->GetRoot()->m_body->GetMatrix() * location);
		visualMesh->SetTransform(matrix);
		visualMesh->SetTransform(matrix);
		ragdoll->SetTransform(matrix);
		
		// add the visual model to the scene and physics model to the world
		ndWorld* const world = scene->GetWorld();
		world->AddModel(model);
		scene->AddEntity(visualMesh);
		return controller;
	}
}

using namespace ndRagdoll;
void ndBasicRagdoll (ndDemoEntityManager* const scene)
{
	// build a floor
	//ndSharedPtr<ndBody> bodyFloor(BuildPlayground(scene));
	//ndSharedPtr<ndBody> bodyFloor(BuildCompoundScene(scene, ndGetIdentityMatrix()));
	//ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marblecheckboard.png", 0.1f, true));
	ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "blueCheckerboard.png", 0.1f, true));

	class PlaceMatrix : public ndMatrix
	{
		public:
		PlaceMatrix(ndDemoEntityManager* const scene, ndFloat32 x, ndFloat32 y, ndFloat32 z)
			:ndMatrix(ndGetIdentityMatrix())
		{
			m_posit = FindFloor(*scene->GetWorld(), ndVector(x, y, z, ndFloat32 (1.0f)), 200.0f);
			m_posit.m_y += ndFloat32(1.5f);
		}
	};

	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("basicRagdoll.nd"));

	ndMatrix playerMatrix(PlaceMatrix(scene, 0.0f, 0.0f, 0.0f));
	CreateRagdoll(scene, loader, playerMatrix);

#if 0
	{
		// add few more rag dolls
		loader.LoadMesh(ndGetWorkingFileName("daveRagdoll1.nd"));
		
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 0.0f, 0.0f, 2.0f));
		//CreateRagdoll(scene, loader, PlaceMatrix(scene, 6.0f, 0.0f, -10.0f));
		//CreateRagdoll(scene, loader, PlaceMatrix(scene, 8.0f, 0.0f, -10.0f));

		loader.LoadMesh(ndGetWorkingFileName("daveRagdoll2.nd"));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, -0.0f, 0.0f, -2.0f));
		//CreateRagdoll(scene, loader, PlaceMatrix(scene, 6.0f, 0.0f, 10.0f));
		//CreateRagdoll(scene, loader, PlaceMatrix(scene, 8.0f, 0.0f, 10.0f));
	}
#endif

	ndFloat32 angle = ndFloat32(0.0f * ndDegreeToRad);
	playerMatrix = ndYawMatrix(angle) * playerMatrix;
	playerMatrix.m_posit += playerMatrix.m_front.Scale (-10.0f);
	playerMatrix.m_posit = FindFloor(*scene->GetWorld(), playerMatrix.m_posit, 200.0f);
	playerMatrix.m_posit.m_y += 2.0f;
	scene->SetCameraMatrix(playerMatrix, playerMatrix.m_posit);
}
