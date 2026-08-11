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

#include "ndPhysicsWorld.h"
#include "ndSoundManager.h"
#include "ndDemoCameraNode.h"
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndGameControllerInputs.h"
#include "ndDebugDisplayRenderPass.h"
#include "ndArchimedesBuoyancyVolume.h"

#define MAX_PHYSICS_STEPS			1
#define MAX_PHYSICS_FPS				60.0f

ndPhysicsWorld::ndDefferedBodyList::ndDefferedBodyList()
	:m_owner(nullptr)
{
}

void ndPhysicsWorld::ndDefferedBodyList::RemovePendingItems()
{
	Iterator it(*this);
	for (it.Begin(); it; it++)
	{
		ndSharedPtr<ndBody> item(it.GetKey());
		m_owner->RemoveBody(*item);
	}
	RemoveAll();
}

// **********************************************************
//
// **********************************************************
ndPhysicsWorld::ndDefferedJointList::ndDefferedJointList()
	:m_owner(nullptr)
{
}

void ndPhysicsWorld::ndDefferedJointList::RemovePendingItems()
{
	Iterator it(*this);
	for (it.Begin(); it; it++)
	{
		ndSharedPtr<ndJointBilateralConstraint> item(it.GetKey());
		m_owner->RemoveJoint(*item);
	}
	RemoveAll();
}

// **********************************************************
//
// **********************************************************
ndPhysicsWorld::ndDefferedModelList::ndDefferedModelList()
	:m_owner(nullptr)
{
}

void ndPhysicsWorld::ndDefferedModelList::RemovePendingItems()
{
	Iterator it(*this);
	for (it.Begin(); it; it++)
	{
		ndSharedPtr<ndModel> item(it.GetKey());
		m_owner->RemoveModel(*item);
	}
	RemoveAll();
}

// **********************************************************
//
// **********************************************************
ndPhysicsWorld::ndDefferedEntityList::ndDefferedEntityList()
	:m_owner(nullptr)
{
}

void ndPhysicsWorld::ndDefferedEntityList::RemovePendingItems()
{
	Iterator it(*this);
	for (it.Begin(); it; it++)
	{
		ndSharedPtr<ndRenderSceneNode> item(it.GetKey());
		m_owner->m_manager->RemoveEntity(item);
	}
	RemoveAll();
}

// **********************************************************
//
// **********************************************************
ndDemoContactCallback::ndDemoContactCallback()
{
}

ndDemoContactCallback::~ndDemoContactCallback()
{
}

ndPhysicsWorld::ndPhysicsWorld(ndDemoEntityManager* const manager)
	:ndWorld()
	,m_manager(manager)
	,m_timeAccumulator(0.0f)
	,m_deadBodies()
	,m_deadJoints()
	,m_deadModels()
	,m_deadEntities()
	,m_updateMode(false)
	,m_acceleratedUpdate(false)
{
	ClearCache();
	m_deadBodies.m_owner = this;
	m_deadJoints.m_owner = this;
	m_deadModels.m_owner = this;
	m_deadEntities.m_owner = this;
	SetContactNotify(new ndDemoContactCallback);
}

ndPhysicsWorld::~ndPhysicsWorld()
{
	CleanUp();
}

void ndPhysicsWorld::CleanUp()
{
	ndWorld::CleanUp();
}

ndDemoEntityManager* ndPhysicsWorld::GetManager() const
{
	return m_manager;
}

void ndPhysicsWorld::OnSubStepPostUpdate(ndFloat32 timestep)
{
	ndWorld::OnSubStepPostUpdate(timestep);
	m_manager->OnSubStepPostUpdate(timestep);
	m_manager->GetGameController()->Update(m_manager);

#if 0
	const ndFixSizeArray<bool, 32>& buttons = m_manager->GetGameController()->GetButtons();
	for (ndInt32 i = 0; i < buttons.GetCount(); ++i)
	{
		if (buttons[i])
		{
			ndTrace(("%d %s\n", i, m_manager->GetGameController()->m_buttonNames[i]));
		}
	}
#endif
}

void ndPhysicsWorld::OnAddBody(ndBody* const body) const
{
	ndWorld::OnAddBody(body);
	ndTrace(("adding a body %d to world\n", body->GetId()));

	ndBodyNotify* const notify = *body->GetNotifyCallback();
	if (notify && notify->IsType(ndDemoEntityNotify::StaticClassName()))
	{
		ndDemoEntityNotify* const entNotify = (ndDemoEntityNotify*)notify;
		entNotify->OnBodyAddedToWorld();
	}
}

void ndPhysicsWorld::OnRemoveBody(ndBody* const body) const
{
	ndWorld::OnRemoveBody(body);
	ndTrace(("removing a body %d from world\n", body->GetId()));
	ndBodyNotify* const notify = *body->GetNotifyCallback();
	if (notify && notify->IsType(ndDemoEntityNotify::StaticClassName()))
	{
		ndDemoEntityNotify* const entNotify = (ndDemoEntityNotify*)notify;
		entNotify->OnBodyRemovedFromWorld();
	}
}

void ndPhysicsWorld::OnAddJoint(ndJointBilateralConstraint* const joint) const
{
	ndWorld::OnAddJoint(joint);
	ndTrace(("adding a joint to world\n"));
}

void ndPhysicsWorld::OnRemoveJoint(ndJointBilateralConstraint* const joint) const
{
	ndWorld::OnRemoveJoint(joint);
	ndTrace(("removing a joint from world\n"));
}

void ndPhysicsWorld::OnAddModel(ndModel* const model) const
{
	ndWorld::OnAddModel(model);
	ndTrace(("adding a model to world\n"));
}

void ndPhysicsWorld::OnRemoveModel(ndModel* const model) const
{
	ndWorld::OnRemoveModel(model);
	ndTrace(("removing a model from world\n"));
}

void ndPhysicsWorld::NormalUpdates()
{
	m_acceleratedUpdate = false;
}

void ndPhysicsWorld::AccelerateUpdates()
{
	m_acceleratedUpdate = true;
}

void ndPhysicsWorld::SetUpdateMode(bool collisionOnly)
{
	m_updateMode = collisionOnly;
}

void ndPhysicsWorld::DefferedRemoveBody(ndSharedPtr<ndBody> body)
{
	ndScopeSpinLock Lock(m_lock);
	ndBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	ndAssert(kinematicBody);
	if (kinematicBody->GetScene())
	{
		ndModel* const model = kinematicBody->GetModel();
		if (model)
		{
			ndSharedPtr<ndModel> modelPtr(GetModel(model));
			if (*modelPtr)
			{
				m_deadModels.Insert(modelPtr);
			}
		}
		else
		{
			ndSharedPtr<ndBody> sharedPtr(kinematicBody->GetAsBodyKinematic()->GetScene()->GetBody(kinematicBody));
			ndDefferedBodyList::ndNode* const node = m_deadBodies.Find(sharedPtr);
			if (!node)
			{
				// we now find all bodies and joints linked to this body to this body
				ndFixSizeArray<ndSharedPtr<ndBody>, 256> stack;
				stack.PushBack(sharedPtr);
				while (stack.GetCount())
				{
					ndSharedPtr<ndBody> bodyNode(stack.Pop());
					if (m_deadBodies.Insert(0, bodyNode))
					{
						ndBodyKinematic* const pivotBody = bodyNode->GetAsBodyKinematic();
						ndBodyNotify* const notify = *body->GetNotifyCallback();
						if (notify->IsType(ndDemoEntityNotify::StaticClassName()))
						{
							ndDemoEntityNotify* const entNotify = (ndDemoEntityNotify*)notify;
							ndSharedPtr<ndRenderSceneNode> visualEntity(entNotify->GetUserData());
							if (*visualEntity)
							{
								DefferedRemoveSceneNode(visualEntity);
							}
						}

						const ndBodyKinematic::ndJointList& joints = pivotBody->GetJointList();
						for (ndBodyKinematic::ndJointList::ndNode* jointNode = joints.GetFirst(); jointNode; jointNode = jointNode->GetNext())
						{
							ndJointBilateralConstraint* const joint = jointNode->GetInfo();
							const ndBodyKinematic* const body0 = joint->GetBody0();
							const ndBodyKinematic* const body1 = joint->GetBody1();
							ndBodyKinematic* const childBody = (ndBodyKinematic*)((body0 == pivotBody) ? body1 : body0);
							if (childBody->GetInvMass() > ndFloat32(0.0f))
							{
								ndSharedPtr<ndBody> childBodyPtr(childBody->GetScene()->GetBody(childBody));
								stack.PushBack(childBodyPtr);
							}
						}
					}
				}
			}
		}
	}
}

void ndPhysicsWorld::DefferedRemoveSceneNode(ndSharedPtr<ndRenderSceneNode> entity)
{
	m_deadEntities.Insert(0, entity);
}

void ndPhysicsWorld::SetCamera(ndSharedPtr<ndRenderSceneNode>& camera)
{
	ndScopeSpinLock Lock(m_lock);
	m_manager->GetRenderer()->SetCamera(camera);
}

void ndPhysicsWorld::PreUpdate(ndFloat32 timestep)
{
	ndWorld::PreUpdate(timestep);

	const ndBodyListView& bodyArray = GetBodyList();
	const ndArray<ndBodyKinematic*>& view = bodyArray.GetView();
	for (ndInt32 i = ndInt32(view.GetCount()) - 2; i >= 0; --i)
	{
		ndBodyKinematic* const body = view[i];
		ndBodyNotify* const notify = *body->GetNotifyCallback();
		if (notify)
		{
			notify->OnPreUpdate(timestep);
		}
	}

	ndRenderPassDebug* const debugRenderPass = m_manager->GetDebugRenderPass();
	debugRenderPass->ClearRuntimeLines();
}

void ndPhysicsWorld::PostUpdate(ndFloat32 timestep)
{
	ndWorld::PostUpdate(timestep);

	if (m_manager->m_onPostUpdate)
	{
		m_manager->m_onPostUpdate->Update(m_manager, timestep);
		m_manager->m_onPostUpdate->OnDebug(m_manager, false);
	}

	ndScopeSpinLock Lock(m_lock);

	// update sound manager
	const ndMatrix listenerMatrix(m_manager->GetRenderer()->GetCamera()->CalculateGlobalMatrix());
	const ndMatrix oldListenerMatrix(m_manager->m_soundManager->GetMatrix());
	const ndVector listenerVeloc((listenerMatrix.m_posit - oldListenerMatrix.m_posit).Scale(ndFloat32(1.0f / timestep)));
	m_manager->m_soundManager->SetMatrix(listenerMatrix);
	m_manager->m_soundManager->SetVelocity(listenerVeloc);
	m_manager->m_soundManager->Update();

	m_manager->SetNextActiveCamera();

	const ndBodyListView& bodyArray = GetBodyList();
	const ndArray<ndBodyKinematic*>& view = bodyArray.GetView();
	for (ndInt32 i = ndInt32(view.GetCount()) - 2; i >= 0; --i)
	{
		ndBodyKinematic* const body = view[i];
		if (!body->GetSleepState())
		{
			ndBodyNotify* const notify = *body->GetNotifyCallback();
			if (notify->IsType(ndDemoEntityNotify::StaticClassName()))
			{
				ndDemoEntityNotify* const entityNotify = (ndDemoEntityNotify*)notify;
				entityNotify->OnPostUpdate(timestep);
				entityNotify->m_entity->SetTransform(entityNotify->m_transform.m_rotation, entityNotify->m_transform.m_position);

				const ndModel* const model = body->GetModel();
				if (model && model->IsType(ndConvexCastVehicle::StaticClassName()))
				{
					const ndConvexCastVehicle* const fastVehicle = (ndConvexCastVehicle*) model;
					if (*fastVehicle->GetRoot()->m_body == body)
					{
						const ndList<ndMultiBodyVehicleTireJoint*>& tireList = fastVehicle->GetTireList();
						for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* tireNode = tireList.GetFirst(); tireNode; tireNode = tireNode->GetNext())
						{
							ndBodyKinematic* const tireBody = tireNode->GetInfo()->GetBody0()->GetAsBodyKinematic();
							ndAssert(tireBody);
							ndAssert(tireBody->GetNotifyCallback()->IsType(ndDemoEntityNotify::StaticClassName()));
							ndDemoEntityNotify* const tireNotify = (ndDemoEntityNotify*)*tireBody->GetNotifyCallback();
							tireNotify->OnPostUpdate(timestep);
							tireNotify->m_entity->SetTransform(tireNotify->m_transform.m_rotation, tireNotify->m_transform.m_position);
						}
					}
				}
			}
		}
	}

	ndDemoCameraNode* const camera = (ndDemoCameraNode*)*m_manager->m_renderer->GetCamera();
	ndAssert(camera);
	camera->TickUpdate(timestep);
	ndAssert(camera);

	// remove all pending objects
	m_deadModels.RemovePendingItems();
	m_deadJoints.RemovePendingItems();
	m_deadBodies.RemovePendingItems();
	m_deadEntities.RemovePendingItems();

	// swap runtime line buffer
	ndRenderPassDebug* const debugRenderPass = m_manager->GetDebugRenderPass();
	debugRenderPass->SwapRuntimeLinesBuffers();
}

void ndPhysicsWorld::UpdateTransforms()
{
	ndWorld::UpdateTransforms();
}

void ndPhysicsWorld::PhysicsUpdate(ndFloat32 timestep)
{
	const ndFloat32 descreteStep = (1.0f / MAX_PHYSICS_FPS);

	if (m_acceleratedUpdate)
	{
		Update(descreteStep);
	}
	else
	{
		ndInt32 maxSteps = MAX_PHYSICS_STEPS;
		m_timeAccumulator += timestep;

		if (m_timeAccumulator > descreteStep * (ndFloat32)maxSteps)
		{
			// if the timestep is more than max timestep per frame, 
			// throw away the extra steps.
			ndFloat32 steps = ndFloor(m_timeAccumulator / descreteStep) - (ndFloat32)maxSteps;
			ndAssert(steps >= 0.0f);
			m_timeAccumulator -= descreteStep * steps;
		}

		while (m_timeAccumulator > descreteStep)
		{
			Update(descreteStep);
			m_timeAccumulator -= descreteStep;
		}
	}

	{
		ndScopeSpinLock Lock(m_lock);
		ndFloat32 param = m_timeAccumulator / descreteStep;
		m_manager->m_renderer->InterpolateTransforms(param);
	}

	if (m_manager->m_synchronousPhysicsUpdate)
	{
		Sync();
	}
}

void ndPhysicsWorld::CollisionUpdate(ndFloat32 timestep)
{
	ndWorld::CollisionUpdate(timestep);

	{
		ndScopeSpinLock Lock(m_lock);
		ndFloat32 param = 0.0f;
		m_manager->m_renderer->InterpolateTransforms(param);
	}
	Sync();
}

void ndPhysicsWorld::AdvanceTime(ndFloat32 timestep)
{
	ND_PROFILE_ZONE();
	if (m_updateMode)
	{
		CollisionUpdate(timestep);
	}
	else
	{
		PhysicsUpdate(timestep);
	}
}