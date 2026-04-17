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
	class ndRagDollJointControl : public ndJointUserData
	{
		public:
		ndRagDollJointControl(ndJointBilateralConstraint* const owner, ndModelArticulation* const appInterface)
			:ndJointUserData(owner)
			,m_model(appInterface)
		{
	
		}
	
		virtual void ApplyControl() = 0;
	
		ndWeakPtr<ndModelArticulation> m_model;
	};
	
	class ndMakeRagDoll : public ndRagDollJointControl
	{
		public:
		ndMakeRagDoll(ndJointBilateralConstraint* const owner, ndModelArticulation* const appInterface)
			:ndRagDollJointControl(owner, appInterface)
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
		class ndFilterPair
		{
			public:
			ndFilterPair(const ndBody* const body0, const ndBody* const body1)
				:m_id0(ndMin(body0->GetId(), body1->GetId()))
				,m_id1(ndMax(body0->GetId(), body1->GetId()))
			{
			}

			union
			{
				ndUnsigned64 m_id;
				struct
				{
					ndUnsigned32 m_id0;
					ndUnsigned32 m_id1;
				};
			};
		};

		ndRagDollController()
			:ndModelNotify()
		{
		}

		bool OnContactGeneration(const ndBodyKinematic* const body0, const ndBodyKinematic* const body1) override
		{
			// The application can use filtering here to decide which body parts should collide.
			// This significantly improves performance by preventing the generation of
			// unnecessary n^2 contact joints that will never collide. 
			// In general, articulated models do not self-collide, 
			// although some parts may occasionally interact. 

			const ndFilterPair code(body0, body1);
			for (ndInt32 i = ndInt32 (m_collisionFilter.GetCount()) - 1; i >= 0; --i)
			{
				if (code.m_id == m_collisionFilter[i].m_id)
				{
					return true;
				}
			}
			return false;
		}

		ndArray<ndFilterPair> m_collisionFilter;
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
		const ndMesh* const mesh = *loader.m_mesh;
		ragdoll->Deserialize(mesh);

		// create a copy of the visual mesh
		ndSharedPtr<ndRenderSceneNode> visualMesh(loader.m_renderMesh->Clone());

		// bind the graphics model to the physics model,
		// also apply any application constomization.
		ndRagDollController* const ragdollController = (ndRagDollController*)*controller;
		auto BindApplicationData = [scene, mesh, ragdollController, &ragdoll, &visualMesh](ndModelArticulation::ndNode* const node)
		{
			ndRenderSceneNode* const visualEntityPtr = visualMesh->FindByClosestMatch(node->m_name);
			ndAssert(visualEntityPtr);
			ndSharedPtr<ndRenderSceneNode> visualEntity((visualEntityPtr == *visualMesh) ? visualMesh : visualEntityPtr->GetSharedPtr());

			// add a rigid body with notification callback
			ndBodyKinematic* const parentBody = node->GetParent() ? node->GetParent()->m_body->GetAsBodyKinematic() : nullptr;
			ndSharedPtr<ndBodyNotify> notify(new ndDemoEntityNotify(scene, visualEntity, parentBody));
			node->m_body->SetNotifyCallback(notify);

			// get the collision pairs filters
			if (node->m_body)
			{
				const ndMesh* const meshOwner = mesh->FindByName(node->m_name);
				ndAssert(meshOwner);
				ndAssert(meshOwner->GetRigidBody());

				const ndMeshBodyDynamic* const rigidBodyInfo = (ndMeshBodyDynamic*)*meshOwner->GetRigidBody();

				ndBody* const body0 = *node->m_body;
				for (ndInt32 i = 0; i < rigidBodyInfo->m_collidingPair.GetCount(); ++i)
				{
					const ndString& name = rigidBodyInfo->m_collidingPair[i]->GetName();
					ndBody* const body1 = *ragdoll->FindByName(name.GetStr())->m_body;
				
					// we add all the colliding pairs, kknowing rher will be duplicates.
					ndRagDollController::ndFilterPair newPair(body0, body1);
					ragdollController->m_collisionFilter.PushBack(newPair);
				}
			}

			if (node->m_joint)
			{
				ndSharedPtr<ndJointUserData> userData(new ndMakeRagDoll(*node->m_joint, ragdoll));
				node->m_joint->SetUserData(userData);
			}
		};
		ragdoll->NodeIterator(BindApplicationData);

		// remove collindg pais duplicated, 
		// the list is small, we can just do a brute force sercj for duplicates
		for (ndInt32 i = ndInt32 (ragdollController->m_collisionFilter.GetCount()) - 1; i > 0; --i)
		{
			ndRagDollController::ndFilterPair code0(ragdollController->m_collisionFilter[i]);
			for (ndInt32 j = i - 1; j >= 0; --j)
			{
				ndRagDollController::ndFilterPair code1(ragdollController->m_collisionFilter[j]);
				if (code0.m_id == code1.m_id)
				{
					ndInt32 lastIndex = ndInt32 (ragdollController->m_collisionFilter.GetCount()) - 1;
					ragdollController->m_collisionFilter[j] = ragdollController->m_collisionFilter[lastIndex];
					ragdollController->m_collisionFilter.SetCount(lastIndex);
					break;
				}
			}
		}

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
		//loader.LoadMesh(ndGetWorkingFileName("xxx1.nd"));
		
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
