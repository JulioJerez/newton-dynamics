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

ndEditorCameraFlyby::ndEditorCameraFlyby(ndAssetEditor* const editor)
	:ndEditorCameraNode(*editor->GetRenderer())
	,m_posit(ndVector::m_wOne)
	,m_yaw(ndFloat32(0.0f))
	,m_pitch(ndFloat32(0.0f))
	,m_yawRate(ndFloat32(0.2f * 60.0f))
	,m_pitchRate(ndFloat32(0.2f * 60.0f))
	,m_mousePosX(ndFloat32(0.0f))
	,m_mousePosY(ndFloat32(0.0f))
	,m_frontSpeed(ndFloat32(10.0f))
	,m_editor(editor)
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

void ndEditorCameraFlyby::CalculateCameraMatrix()
{
	ndRender* const renderer = GetOwner();
	ndAssert(renderer);

	const ndMatrix newCameMatrix(ndRollMatrix(m_pitch) * ndYawMatrix(m_yaw));
	const ndQuaternion newRotation(newCameMatrix);
	const ndVector newPosit(newCameMatrix.RotateVector(m_posit));
	ndEditorCameraNode::SetTransform(newRotation, newPosit);

	const ndVector lightDir(newCameMatrix.RotateVector(ndVector(-1.0f, 1.0f, 0.f, 0.0f)));
	renderer->SetSunLight(lightDir, ndVector(0.7f, 0.7f, 0.7f, 0.0f));
}

void ndEditorCameraFlyby::TickUpdate(ndFloat32 timestep)
{
	ndRender* const renderer = GetOwner();
	ndAssert(renderer);
	ndAssetEditor::ndRenderCallback* const renderCallback = (ndAssetEditor::ndRenderCallback*)*renderer->GetOwner();
	ndAssetEditor* const scene = *renderCallback->m_owner;
	
	ndFloat32 mouseX;
	ndFloat32 mouseY;
	scene->GetMousePosition(mouseX, mouseY);
	
	if (!scene->GetCaptured() && (scene->GetMouseKeyState(0) || scene->GetMouseKeyState(1) || scene->GetMouseKeyState(2)))
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

			if (!m_mouseClick && !m_editor->m_lockSelection)
			{
				MouseSelection();
				m_mouseClick = true;
			}
		}

		if (ImGui::IsMouseDown(1))
		{
			ndFloat32 pan_x = mouseX - m_mousePosX;
			ndFloat32 pan_y = mouseY - m_mousePosY;

			if (pan_x < 0.0f)
			{
				m_posit.m_z += m_frontSpeed * timestep;
			}
			else if (pan_x > 0.0f)
			{
				m_posit.m_z -= m_frontSpeed * timestep;
			}

			if (pan_y > 0.0f)
			{
				m_posit.m_y += m_frontSpeed * timestep;
			}
			else if (pan_y < 0.0f)
			{
				m_posit.m_y -= m_frontSpeed * timestep;
			}
		}

		if (ImGui::IsMouseDown(2))
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
		//const ndMatrix newCameMatrix(ndRollMatrix(m_pitch) * ndYawMatrix(m_yaw));
		//const ndQuaternion newRotation(newCameMatrix);
		//const ndVector newPosit(newCameMatrix.RotateVector(m_posit));
		//ndEditorCameraNode::SetTransform(newRotation, newPosit);
		//
		//const ndVector lightDir(newCameMatrix.RotateVector(ndVector(-1.0f, 1.0f, 0.f, 0.0f)));
		//renderer->SetSunLight(lightDir, ndVector(0.7f, 0.7f, 0.7f, 0.0f));
		CalculateCameraMatrix();
	}
	else
	{
		m_mouseClick = false;
	}

	m_mousePosX = mouseX;
	m_mousePosY = mouseY;
}

void ndEditorCameraFlyby::SetView(ndAssetEditor::ndCameraMode mode)
{
	switch (mode)
	{
		case ndAssetEditor::m_backView:
		{
			m_pitch = 0.0f;
			m_yaw = 0.0f * ndDegreeToRad;
			CalculateCameraMatrix();
			break;
		}

		case ndAssetEditor::m_frontView:
		{
			m_pitch = 0.0f;
			m_yaw = 180.0f * ndDegreeToRad;
			CalculateCameraMatrix();
			break;
		}

		case ndAssetEditor::m_sideLeftView:
		{
			m_pitch = 0.0f;
			m_yaw = 90.0f * ndDegreeToRad;
			CalculateCameraMatrix();
			break;
		}

		case ndAssetEditor::m_sideRrightView:
		{
			m_pitch = 0.0f;
			m_yaw = -90.0f * ndDegreeToRad;
			CalculateCameraMatrix();
			break;
		}

		case ndAssetEditor::m_free:
		default:;
	}
}

void ndEditorCameraFlyby::MouseSelection()
{
	if (!*m_editor->m_mesh)
	{
		return;
	}

	ndFloat32 mouseX;
	ndFloat32 mouseY;
	m_editor->GetMousePosition(mouseX, mouseY);

	const ndRenderSceneCamera* const camera = FindCameraNode();
	const ndVector p0(camera->ScreenToWorld(ndVector(mouseX, mouseY, ndFloat32(0.0f), ndFloat32(0.0f))));
	const ndVector p1(camera->ScreenToWorld(ndVector(mouseX, mouseY, ndFloat32(1.0f), ndFloat32(0.0f))));

	ndFloat32 hitParam = 1.0f;
	ndSharedPtr<ndMesh> hitNode(nullptr);

	auto RayCast = [this, &hitNode, &hitParam, &p0, &p1](ndMesh* const node)
	{
		if (node->GetGeometry())
		{
			const ndMatrix matrix(node->GetGeometryMatrix() * node->CalculateGlobalMatrix());
			const ndVector localP0(matrix.UntransformVector(p0));
			const ndVector localP1(matrix.UntransformVector(p1));
			ndFloat32 param = node->GetGeometry()->RayCast(localP0, localP1);
			if (param < hitParam)
			{
				hitParam = param;
				hitNode = (node != *m_editor->m_mesh) ? node->GetSharedPtr() : m_editor->m_mesh;
			}
		}
	};
	m_editor->m_mesh->NodeIterator(RayCast);

	if (hitNode)
	{
		ndAssetEditor::ndSubSelectionMode selectionMode = m_editor->m_subSelection;
		switch (selectionMode)
		{
			case ndAssetEditor::m_loopJoint:
			{
				m_editor->SetLoopJointSelection(hitNode);
				break;
			}

			case ndAssetEditor::m_collidingPair:
			{
				m_editor->SetCollidingSubSelection(hitNode);
				break;
			}

			case ndAssetEditor::m_none:
			default:
			{
				m_editor->m_currentSelection = hitNode;
			}
		}
	}
}