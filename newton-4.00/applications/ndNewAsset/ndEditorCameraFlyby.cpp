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

#define D_ZOOM_FACTOR		ndFloat32 (10.0f)
#define D_PANNING_FACTOR	ndFloat32 (10.0f)

ndEditorCameraFlyby::ndEditorCameraFlyby(ndAssetEditor* const editor)
	:ndEditorCameraNode(*editor->GetRenderer())
	,m_posit(ndVector::m_wOne)
	,m_yaw(ndFloat32(0.0f))
	,m_pitch(ndFloat32(0.0f))
	,m_yawRate(ndFloat32(0.2f * 60.0f))
	,m_pitchRate(ndFloat32(0.2f * 60.0f))
	,m_mousePosX(ndFloat32(0.0f))
	,m_mousePosY(ndFloat32(0.0f))
	,m_panningSpeed(D_PANNING_FACTOR)
	,m_mouseClick(false)
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

	// calculate offset;
	ndMatrix offset(ndGetIdentityMatrix());
	if (!m_editor->m_orbitRootNode && m_editor->m_currentSelection)
	{
		const ndMatrix matrix(m_editor->m_currentSelection->CalculateGlobalMatrix());
		offset.m_posit = matrix.m_posit;
		offset = offset.OrthoInverse();
	}
	const ndMatrix rotation(ndRollMatrix(m_pitch) * ndYawMatrix(m_yaw));
	ndMatrix posit(ndGetIdentityMatrix());
	posit.m_posit = m_posit;
	const ndMatrix cameraMatrix(offset * posit * rotation * offset.OrthoInverse());
	ndEditorCameraNode::SetTransform(cameraMatrix, cameraMatrix.m_posit);

	const ndVector lightDir(cameraMatrix.RotateVector(ndVector(-1.0f, 1.0f, 0.f, 0.0f)));
	renderer->SetSunLight(lightDir, ndVector(0.7f, 0.7f, 0.7f, 0.0f));
}

void ndEditorCameraFlyby::TickUpdate(ndFloat32 timestep)
{
	ndRender* const renderer = GetOwner();
	ndAssert(renderer);
	ndAssetEditor::ndRenderCallback* const renderCallback = (ndAssetEditor::ndRenderCallback*)*renderer->GetOwner();
	ndAssetEditor* const scene = *renderCallback->m_owner;

	ndRenderSceneCamera* const camera = GetCamera();
	ndAssert(camera);
	
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
				m_posit.m_z += m_panningSpeed * timestep;
			}
			else if (pan_x > 0.0f)
			{
				m_posit.m_z -= m_panningSpeed * timestep;
			}

			if (pan_y > 0.0f)
			{
				m_posit.m_y += m_panningSpeed * timestep;
			}
			else if (pan_y < 0.0f)
			{
				m_posit.m_y -= m_panningSpeed * timestep;
			}
		}

		if (ImGui::IsMouseDown(2))
		{
			ndFloat32 zoom = mouseY - m_mousePosY;
			if (zoom > 0.0f)
			{
				ndFloat32 factor = ndFloat32(1.0f) + D_ZOOM_FACTOR * timestep;
				ndFloat32 zoomFactor = camera->m_zoom * factor;
				camera->m_zoom = ndClamp (zoomFactor, ndFloat32(0.01f), ndFloat32(100.f));
			}
			else if (zoom < 0.0f)
			{
				ndFloat32 factor = ndFloat32(1.0f) + D_ZOOM_FACTOR * timestep;
				ndFloat32 zoomFactor = camera->m_zoom / factor;
				camera->m_zoom = ndClamp(zoomFactor, ndFloat32(0.01f), ndFloat32(100.f));
			}
		}
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

		case ndAssetEditor::m_leftSideView:
		{
			m_pitch = 0.0f;
			m_yaw = 90.0f * ndDegreeToRad;
			CalculateCameraMatrix();
			break;
		}

		case ndAssetEditor::m_rightSideView:
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
	
	ndSharedPtr<ndMesh> hitNode(nullptr);
	if (m_editor->m_raycastBones)
	{
		// bone selection mode.
		ndFloat32 hitParam = ndFloat32(1.0e20f);
		auto BoneCast = [this, &hitNode, &hitParam, &p0, &p1](ndMesh* const node)
		{
			if (node->GetNodeType() == ndMesh::m_bone)
			{
				const ndMatrix boneMatrix(node->CalculateGlobalMatrix());
				const ndVector target(boneMatrix.TransformVector(node->GetBoneTarget()));

				ndBigVector p0Out;
				ndBigVector p1Out;
				ndRayToRayDistance(
					ndBigVector(boneMatrix.m_posit), 
					ndBigVector(target), 
					ndBigVector(p0), ndBigVector(p1), 
					p0Out, p1Out);

				ndBigVector dist(p1Out - p0Out);
				ndFloat32 dist2 = ndFloat32(dist.DotProduct(dist).GetScalar());
				if (dist2 < hitParam)
				{
					hitParam = dist2;
					hitNode = node->GetSharedPtr();
				}
			}
		};
		m_editor->m_mesh->NodeIterator(BoneCast);
		ndFloat32 maxDist = ndFloat32(0.1f);
		if (hitParam > (maxDist * maxDist))
		{
			hitNode = ndSharedPtr<ndMesh> (nullptr);
		}
	}
	else
	{
		ndFloat32 hitParam = ndFloat32(1.0f);
		auto RayCast = [this, &hitNode, &hitParam, &p0, &p1](ndMesh* const node)
		{
			if (node->GetVisibility() && node->GetGeometry())
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
	}

	if (hitNode)
	{
		m_editor->SelectCurrentNode(hitNode);
	}
}