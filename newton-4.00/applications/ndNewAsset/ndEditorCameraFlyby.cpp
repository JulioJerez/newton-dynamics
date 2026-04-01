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
	,m_frontSpeed(ndFloat32(0.25f * 60.0f))
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
	
	if (!scene->GetCaptured() && (scene->GetMouseKeyState(0) || scene->GetMouseKeyState(1)))
	{
		if (ImGui::IsMouseDown(0))
		{
			ndFloat32 mouseSpeedX = mouseX - m_mousePosX;
			ndFloat32 mouseSpeedY = mouseY - m_mousePosY;

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

		if (ImGui::IsMouseDown(1))
		{
			ndFloat32 zoom = mouseY - m_mousePosY;
			if (zoom > 0.0f)
			{
				m_posit.m_x += m_frontSpeed * timestep;
			}
			else if (zoom < 0.0f)
			{
				m_posit.m_x -= m_frontSpeed * timestep;
			}
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
