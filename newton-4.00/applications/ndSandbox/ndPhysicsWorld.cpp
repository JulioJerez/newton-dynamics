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
#include "ndDemoCameraNode.h"
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndArchimedesBuoyancyVolume.h"

#define MAX_PHYSICS_STEPS			1
#define MAX_PHYSICS_FPS				60.0f

ndPhysicsWorld::ndDefferedBodyList::ndDefferedBodyList()
	:m_owner(nullptr)
{
}

void ndPhysicsWorld::ndDefferedBodyList::RemovePendingBodies()
{
	Iterator it(*this);
	for (it.Begin(); it; it++)
	{
		ndSharedPtr<ndBody> body(it.GetKey());
		m_owner->RemoveBody(*body);
	}
	RemoveAll();
}

ndPhysicsWorld::ndDefferedJointList::ndDefferedJointList()
	:m_owner(nullptr)
{
}

void ndPhysicsWorld::ndDefferedJointList::RemovePendingBodies()
{
	Iterator it(*this);
	for (it.Begin(); it; it++)
	{
		ndSharedPtr<ndJointBilateralConstraint> joint(it.GetKey());
		m_owner->RemoveJoint(*joint);
	}
	RemoveAll();
}


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
	,m_interplationParameter(0.0f)
	,m_deadBodies()
	//,m_deadModels()
	//,m_deadEntities()
	,m_deadJoints()
	,m_acceleratedUpdate(false)
{
	ClearCache();
	m_deadBodies.m_owner = this;
	m_deadJoints.m_owner = this;
	//m_deadModels.m_owner = this;
	//m_deadEntities.m_owner = this;
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

void ndPhysicsWorld::PreUpdate(ndFloat32 timestep)
{
	ndWorld::PreUpdate(timestep);
}

void ndPhysicsWorld::OnSubStepPostUpdate(ndFloat32 timestep)
{
	ndWorld::OnSubStepPostUpdate(timestep);
	m_manager->OnSubStepPostUpdate(timestep);
}

void ndPhysicsWorld::OnAddBody(ndBody* const body) const
{
	ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)*body->GetNotifyCallback();
	if (notify)
	{
		notify->OnBodyAddedToWorld();
	}
}

void ndPhysicsWorld::OnRemoveBody(ndBody* const body) const
{
	ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)*body->GetNotifyCallback();
	if (notify)
	{
		notify->OnBodyRemovedFromWorld();
	}
}

void ndPhysicsWorld::NormalUpdates()
{
	m_acceleratedUpdate = false;
}

void ndPhysicsWorld::AccelerateUpdates()
{
	m_acceleratedUpdate = true;
}

void ndPhysicsWorld::UpdateTransforms()
{
	// for some reason this cause a dead lock. I need to investigate.
	//ndScopeSpinLock Lock(m_lock); 
	ndWorld::UpdateTransforms();
}

void ndPhysicsWorld::PostUpdate(ndFloat32 timestep)
{
	ndWorld::PostUpdate(timestep);

	if (m_manager->m_onPostUpdate)
	{
		m_manager->m_onPostUpdate->Update(m_manager, timestep);
		//m_manager->m_onPostUpdate->OnDebug(m_manager, m_manager->m_hidePostUpdate);
		m_manager->m_onPostUpdate->OnDebug(m_manager, false);
	}

	ndScopeSpinLock Lock(m_lock);

	m_manager->SetNextActiveCamera();

	const ndBodyListView& bodyArray = GetBodyList();
	const ndArray<ndBodyKinematic*>& view = bodyArray.GetView();
	for (ndInt32 i = ndInt32(view.GetCount()) - 2; i >= 0; --i)
	{
		ndBodyKinematic* const body = view[i];
		if (!body->GetSleepState())
		{
			ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)*body->GetNotifyCallback();
			if (notify)
			{
				notify->m_entity->SetTransform(notify->m_transform.m_rotation, notify->m_transform.m_position);
			}
		}
	}

	ndDemoCameraNode* const camera = (ndDemoCameraNode*)*m_manager->m_renderer->GetCamera();
	ndAssert(camera);
	camera->TickUpdate(timestep);

	// remove all pending objects
	//m_deadModels.RemovePendingBodies();
	//m_deadJoints.RemovePendingBodies();
	m_deadBodies.RemovePendingBodies();
	//m_deadEntities.RemovePendingBodies();
}

void ndPhysicsWorld::DefferedRemoveBody(ndBody* const body)
{
	ndScopeSpinLock Lock(m_lock);
	ndAssert (body->GetAsBodyKinematic()->GetScene());
	if (body->GetAsBodyKinematic()->GetScene())
	{
		ndSharedPtr<ndBody> sharedPtr(body->GetAsBodyKinematic()->GetScene()->GetBody(body));
		ndDefferedBodyList::ndNode* const node = m_deadBodies.Find(sharedPtr);
		if (!node)
		{
			// we now find all bodies ann bodies and joints connected to this body
			//m_deadBodies.Insert(0, sharedPtr);

			ndFixSizeArray<ndSharedPtr<ndBody>, 256> stack;
			stack.PushBack(sharedPtr);
			while (stack.GetCount())
			{
				ndSharedPtr<ndBody> bodyNode(stack.Pop());
				if (!m_deadBodies.Find(bodyNode))
				{
					m_deadBodies.Insert(0, bodyNode);
					const ndBodyKinematic::ndJointList& joints = body->GetAsBodyDynamic()->GetJointList();
					for (ndBodyKinematic::ndJointList::ndNode* jointNode = joints.GetFirst(); jointNode; jointNode = jointNode->GetNext())
					{
						ndJointBilateralConstraint* const joint = jointNode->GetInfo();
						ndBodyKinematic* const childBody = (joint->GetBody0() == body) ? joint->GetBody1() : joint->GetBody0();
						ndSharedPtr<ndBody> childBodyPtr(childBody->GetScene()->GetBody(body));
						stack.PushBack(childBodyPtr);
					}
				}
			}
		}
	}
}

//void ndPhysicsWorld::RemoveDeadEntities()
//{
//	ndAssert(0);
//	//ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* nextNode;
//	//for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = m_defferedDeadEntities.GetFirst(); node; node = nextNode)
//	//{
//	//	nextNode = node->GetNext();
//	//	m_manager->RemoveEntity(node->GetInfo());
//	//	m_defferedDeadEntities.Remove(node);
//	//}
//}

void ndPhysicsWorld::AdvanceTime(ndFloat32 timestep)
{
	D_TRACKTIME();
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