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
			// this greatly improves performance because since articulated models in general 
			// do not self collide, but occationaly some parts do collide. 
			// for now we just return false (no collision)
			return false;
		}
		
		void RagdollBuildScript(ndDemoEntityManager* const scene, const ndRenderMeshLoader& loader, const ndMatrix& location)
		{
			ndSharedPtr<ndRenderSceneNode> visualMesh(loader.m_renderMesh->Clone());
			visualMesh->SetTransform(location);
			visualMesh->SetTransform(location);
			scene->AddEntity(visualMesh);
		
			GetModel()->GetAsModelArticulation()->Deserialize(*loader.m_mesh);
			auto BindApplicationData = [this, scene, &visualMesh](ndModelArticulation::ndNode* const node)
			{
				ndRenderSceneNode* const visualEntityPtr = visualMesh->FindByClosestMatch(node->m_name);
				ndAssert(visualEntityPtr);
				ndSharedPtr<ndRenderSceneNode> visualEntity((visualEntityPtr == *visualMesh) ? visualMesh : visualEntityPtr->GetSharedPtr());
		
				// add a rigid body with notification callback
				ndBodyKinematic* const parentBody = node->GetParent() ? node->GetParent()->m_body->GetAsBodyKinematic() : nullptr;
				ndSharedPtr<ndBodyNotify> notify(new ndDemoEntityNotify(scene, visualEntity, parentBody));
				node->m_body->SetNotifyCallback(notify);
			};
			GetModel()->GetAsModelArticulation()->NodeIterator(BindApplicationData);
		
		}
	};

	ndSharedPtr<ndModelNotify> CreateRagdoll(ndDemoEntityManager* const scene, const ndRenderMeshLoader& loader, const ndMatrix& location)
	{
		// make a hierchical atriculate model
		ndSharedPtr<ndModel> model(new ndModelArticulation());
		
		// create a ragdoll controller 
		ndSharedPtr<ndModelNotify> controller(new ndRagDollController());
		model->SetNotifyCallback(controller);
		
		ndRagDollController* const ragdollController = (ndRagDollController*)*controller;
		ragdollController->RagdollBuildScript(scene, loader, location);
		
		ndWorld* const world = scene->GetWorld();
		world->AddModel(model);
		return controller;
	}
}

using namespace ndRagdoll;
void ndBasicRagdoll (ndDemoEntityManager* const scene)
{
	// build a floor
	//ndSharedPtr<ndBody> bodyFloor(BuildPlayground(scene));
	//ndSharedPtr<ndBody> bodyFloor(BuildCompoundScene(scene, ndGetIdentityMatrix()));
	ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marblecheckboard.png", 0.1f, true));

	class PlaceMatrix : public ndMatrix
	{
		public:
		PlaceMatrix(ndDemoEntityManager* const scene, ndFloat32 x, ndFloat32 y, ndFloat32 z)
			:ndMatrix(ndGetIdentityMatrix())
		{
			m_posit = FindFloor(*scene->GetWorld(), ndVector(x, y, z, ndFloat32 (1.0f)), 200.0f);
			m_posit.m_y += ndFloat32(10.0f);
		}
	};

	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("daveRagdoll2.nd"));
	//loader.ImportFbx(ndGetWorkingFileName("ragdoll.fbx"));
	//loader.ImportFbx(ndGetWorkingFileName("ragdoll.fbx"));
	//loader.LoadMesh(ndGetWorkingFileName("ragdoll.nd"));

	ndMatrix playerMatrix(PlaceMatrix(scene, 0.0f, 0.0f, 0.0f));
	ndSharedPtr<ndModelNotify> modelNotity(CreateRagdoll(scene, loader, playerMatrix));

	{
#if 0
		// add few more rag dolls
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 0.0f, 0.0f, 0.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 3.0f, 0.0f, 0.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 5.0f, 0.0f, 0.0f));

		CreateRagdoll(scene, loader, PlaceMatrix(scene, 0.0f, 0.0f, 10.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 3.0f, 0.0f, 10.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 5.0f, 0.0f, 10.0f));

		CreateRagdoll(scene, loader, PlaceMatrix(scene, 0.0f, 0.0f, -10.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 3.0f, 0.0f, -10.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 5.0f, 0.0f, -10.0f));
#endif
	}

	ndFloat32 angle = ndFloat32(90.0f * ndDegreeToRad);
	playerMatrix = ndYawMatrix(angle) * playerMatrix;
	playerMatrix.m_posit += playerMatrix.m_front.Scale (-5.0f);
	playerMatrix.m_posit = FindFloor(*scene->GetWorld(), playerMatrix.m_posit, 200.0f);
	playerMatrix.m_posit.m_y += 2.0f;
	scene->SetCameraMatrix(playerMatrix, playerMatrix.m_posit);
}
