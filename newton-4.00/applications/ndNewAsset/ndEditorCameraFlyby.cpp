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

#include "ndNewAssetStdafx.h"
#include "ndAssetEditor.h"
#include "ndEditorCameraFlyby.h"

ndEditorCameraFlyby::ndEditorCameraFlyby(ndRender* const owner)
	:ndEditorCameraNode(owner)
	,m_posit(ndVector::m_wOne)
	,m_yaw(ndFloat32(0.0f))
	,m_pitch(ndFloat32(0.0f))
	,m_yawRate(ndFloat32(0.2f * 60.0f))
	,m_pitchRate(ndFloat32(0.2f * 60.0f))
	,m_mousePosX(ndFloat32(0.0f))
	,m_mousePosY(ndFloat32(0.0f))
	,m_frontSpeed(ndFloat32(15.0f))
	,m_sidewaysSpeed(ndFloat32(10.0f))
{
}

void ndEditorCameraFlyby::SetTransform(const ndQuaternion& rotation, const ndVector& position)
{
	ndEditorCameraNode::SetTransform(rotation, position);
	const ndMatrix matrix(GetTransform().GetMatrix());
	m_pitch = ndAsin(matrix.m_front.m_y);
	m_yaw = ndAtan2(-matrix.m_front.m_z, matrix.m_front.m_x);
	m_posit = position;
}

void ndEditorCameraFlyby::TickUpdate(ndFloat32 timestep)
{
	ndRender* const renderer = GetOwner();
	ndAssert(renderer);
	ndAssetEditor::ndRenderCallback* const renderCallback = (ndAssetEditor::ndRenderCallback*)*renderer->GetOwner();
	ndAssetEditor* const scene = renderCallback->m_owner;
	
	ndFloat32 mouseX;
	ndFloat32 mouseY;
	scene->GetMousePosition(mouseX, mouseY);
	
	//// slow down the Camera if we have a Body
	//ndFloat32 slowDownFactor = scene->IsShiftKeyDown() ? 0.5f / 10.0f : 0.5f;
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
	
	//ndMatrix matrix(ndRollMatrix(m_pitch) * ndYawMatrix(m_yaw));
	//ndQuaternion newRotation(matrix);
	//ndEditorCameraNode::SetTransform(newRotation, targetMatrix.m_posit);
	
	bool mouseState = !scene->GetCaptured() && (scene->GetMouseKeyState(0) && !scene->GetMouseKeyState(1));
	// do camera rotation, only if we do not have anything picked
	//if (!UpdatePickBody() && mouseState)
	if (mouseState)
	{
		ndFloat32 mouseSpeedX = mouseX - m_mousePosX;
		ndFloat32 mouseSpeedY = mouseY - m_mousePosY;

		if (ImGui::IsMouseDown(0))
		{
			if (mouseSpeedX > 0.0f)
			{
				m_yaw = ndAnglesAdd(m_yaw, -m_yawRate * timestep);
			}
			else if (mouseSpeedX < 0.0f)
			{
				m_yaw = ndAnglesAdd(m_yaw, m_yawRate * timestep);
			}
	
			if (mouseSpeedY > 0.0f)
			{
				m_pitch -= m_pitchRate * timestep;
			}
			else if (mouseSpeedY < 0.0f)
			{
				m_pitch += m_pitchRate * timestep;
			}
			m_pitch = ndClamp(m_pitch, ndFloat32(-80.0f * ndDegreeToRad), ndFloat32(80.0f * ndDegreeToRad));
		}

		const ndMatrix newCameMatrix(ndRollMatrix(m_pitch) * ndYawMatrix(m_yaw));
		const ndQuaternion newRotation(newCameMatrix);
		const ndVector newPosit(newCameMatrix.RotateVector(m_posit));
		ndEditorCameraNode::SetTransform(newRotation, newPosit);

		const ndVector lightDir(newCameMatrix.RotateVector(ndVector(-1.0f, 1.0f, 0.f, 0.0f)));
		renderer->SetSunLight(lightDir, ndVector(0.7f, 0.7f, 0.7f, 0.0f));
	}
	
	m_mousePosX = mouseX;
	m_mousePosY = mouseY;
}
