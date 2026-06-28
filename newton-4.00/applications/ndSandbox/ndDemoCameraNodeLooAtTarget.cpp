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
#include "ndPhysicsUtils.h"
#include "ndDemoEntityManager.h"
#include "ndDemoCameraNodeLookAtTarget.h"

ndDemoCameraNodeLookAtTarget::ndDemoCameraNodeLookAtTarget(ndRender* const owner)
     :ndDemoCameraNodeFlyby(owner)
	,m_target(nullptr)
{
}

void ndDemoCameraNodeLookAtTarget::SetTarget(ndSharedPtr<ndRenderSceneNode>& target)
{
	m_target = target;
}

void ndDemoCameraNodeLookAtTarget::TickUpdate(ndFloat32 timestep)
{
	//if (!(*m_target))
	//{
	//	ndList<ndSharedPtr<ndRenderSceneNode>>& scene = m_owner->GetScene();
	//	for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* sceneNode = scene.GetFirst(); sceneNode; sceneNode = sceneNode->GetNext())
	//	{
	//		ndSharedPtr<ndRenderSceneNode>& node = sceneNode->GetInfo();
	//		const ndRenderSceneCamera* const cameraNode = node->FindCameraNode();
	//		if (cameraNode)
	//		{
	//			m_target = node;
	//			break;
	//		}
	//	}
	//}

	// calculate pitch and Yaw.
	if (*m_target)
	{
		const ndMatrix originMatrix(GetTransform().GetMatrix());
		const ndMatrix targetMatrix(m_target->GetTransform().GetMatrix());
		const ndVector offset((targetMatrix.m_posit - originMatrix.m_posit).Normalize());
		ndFloat32 yaw = ndAtan2 (-offset.m_z, offset.m_x);
		m_yaw += ndAnglesSub(yaw, m_yaw);
		m_roll = ndAsin(ndClamp (offset.m_y, ndFloat32 (-0.98f), ndFloat32(0.98f)));
	}
	 
	// do normal update
	ndDemoCameraNodeFlyby::TickUpdate(timestep);
}
