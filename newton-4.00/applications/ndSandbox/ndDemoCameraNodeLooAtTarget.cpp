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
{
}

void ndDemoCameraNodeLookAtTarget::TickUpdate(ndFloat32 timestep)
{
	// calculate pitch and Yaw.

	// do normal update
	ndDemoCameraNodeFlyby::TickUpdate(timestep);
	//ndRender* const renderer = GetOwner();
	//ndAssert(renderer);
	//ndDemoEntityManager::ndRenderCallback* const renderCallback = (ndDemoEntityManager::ndRenderCallback*)*renderer->GetOwner();
	//ndDemoEntityManager* const scene = renderCallback->m_owner;
	//
	//ndFloat32 mouseX;
	//ndFloat32 mouseY;
	//scene->GetMousePosition(mouseX, mouseY);
	//
	//// slow down the Camera if we have a Body
	//ndFloat32 slowDownFactor = scene->IsShiftKeyDown() ? 0.5f / 10.0f : 0.5f;
	//
	//ndMatrix targetMatrix(m_transform1.GetMatrix());
	//
	//// do camera translation
	//if (scene->GetKeyState(ImGuiKey_W))
	//{
	//	targetMatrix.m_posit += targetMatrix.m_front.Scale(m_frontSpeed * timestep * slowDownFactor);
	//}
	//if (scene->GetKeyState(ImGuiKey_S))
	//{
	//	targetMatrix.m_posit -= targetMatrix.m_front.Scale(m_frontSpeed * timestep * slowDownFactor);
	//}
	//if (scene->GetKeyState(ImGuiKey_A))
	//{
	//	targetMatrix.m_posit -= targetMatrix.m_right.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	//}
	//if (scene->GetKeyState(ImGuiKey_D))
	//{
	//	targetMatrix.m_posit += targetMatrix.m_right.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	//}
	//
	//if (scene->GetKeyState(ImGuiKey_Q))
	//{
	//	targetMatrix.m_posit -= targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	//}
	//
	//if (scene->GetKeyState(ImGuiKey_E))
	//{
	//	targetMatrix.m_posit += targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	//}
	//
	//const ndMatrix matrix(ndRollMatrix(m_pitch) * ndYawMatrix(m_yaw));
	//const ndQuaternion newRotation(matrix);
	//ndDemoCameraNode::SetTransform(newRotation, targetMatrix.m_posit);
	//
	//bool mouseState = !scene->GetCaptured() && (scene->GetMouseKeyState(0) && !scene->GetMouseKeyState(1));
	//// do camera rotation, only if we do not have anything picked
	//if (!UpdatePickBody() && mouseState)
	//{
	//	ndFloat32 mouseSpeedX = mouseX - m_mousePosX;
	//	ndFloat32 mouseSpeedY = mouseY - m_mousePosY;
	//
	//	if (ImGui::IsMouseDown(0))
	//	{
	//		if (mouseSpeedX > 0.0f)
	//		{
	//			m_yaw = ndAnglesAdd(m_yaw, -m_yawRate);
	//		}
	//		else if (mouseSpeedX < 0.0f)
	//		{
	//			m_yaw = ndAnglesAdd(m_yaw, m_yawRate);
	//		}
	//
	//		if (mouseSpeedY > 0.0f)
	//		{
	//			m_pitch -= m_pitchRate;
	//		}
	//		else if (mouseSpeedY < 0.0f)
	//		{
	//			m_pitch += m_pitchRate;
	//		}
	//		m_pitch = ndClamp(m_pitch, ndFloat32(-80.0f * ndDegreeToRad), ndFloat32(80.0f * ndDegreeToRad));
	//	}
	//}
	//
	//m_mousePosX = mouseX;
	//m_mousePosY = mouseY;
}
