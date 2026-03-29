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

void ndAssetEditor::ShowPropertiesPanel()
{
	ImGui::Begin("Properties Panel");

	WindowFrame frame;
	frame.m_posit = ImGui::GetWindowPos();
	frame.m_size = ImGui::GetWindowSize();
	m_windowSizes.PushBack(frame);

	if (*m_currentSelection)
	{
		ShowPropertiesMeshInfo();
		if (*m_currentSelection->GetRigidBody())
		{
			ShowPropertiesRigidBodyInfo();
		}
	}
	
	ImGui::End();
}

void ndAssetEditor::ShowPropertiesMeshInfo()
{
	if (ImGui::CollapsingHeader("Transforms"))
	{
		char nodeName[256];
		snprintf(nodeName, sizeof(nodeName) - 1, "%s", m_currentSelection->GetName().GetStr());
		if (ImGui::InputText("node Name1", nodeName, sizeof(nodeName) - 1))
		{
			if (strcmp(m_currentSelection->GetName().GetStr(), nodeName))
			{
				m_currentSelection->SetName(ndString(nodeName));
			}
		}

		// show node matrix
		{
			ImGui::SeparatorText("Transform");
			ndMatrix matrix(m_currentSelection->GetMatrix());
			ndReal position[3];
			position[0] = matrix.m_posit.m_x;
			position[1] = matrix.m_posit.m_y;
			position[2] = matrix.m_posit.m_z;
			if (ImGui::DragFloat3("position", position))
			{
				matrix.m_posit.m_x = position[0];
				matrix.m_posit.m_y = position[1];
				matrix.m_posit.m_z = position[2];
				m_currentSelection->SetMatrix(matrix);
			};

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			euler[0] = radians[0];
			euler[1] = radians[1];
			euler[2] = radians[2];
			if (ImGui::DragFloat3("rotation", euler))
			{
				ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
				newMatrix.m_posit = matrix.m_posit;
				m_currentSelection->SetMatrix(newMatrix);
			};
		}

		// show geometry node matrix
		if (*m_currentSelection->GetMesh())
		{
			ImGui::SeparatorText("geomtry transform");
			ndMatrix matrix(m_currentSelection->GetGeometryMatrix());
			ndReal position[3];
			position[0] = matrix.m_posit.m_x;
			position[1] = matrix.m_posit.m_y;
			position[2] = matrix.m_posit.m_z;
			if (ImGui::DragFloat3("position##1", position))
			{
				matrix.m_posit.m_x = position[0];
				matrix.m_posit.m_y = position[1];
				matrix.m_posit.m_z = position[2];
				m_currentSelection->SetGeometryMatrix(matrix);
			};

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			euler[0] = radians[0];
			euler[1] = radians[1];
			euler[2] = radians[2];
			if (ImGui::DragFloat3("rotation##1", euler))
			{
				ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
				newMatrix.m_posit = matrix.m_posit;
				m_currentSelection->SetGeometryMatrix(newMatrix);
			};
		}
	}
}

void ndAssetEditor::ShowPropertiesRigidBodyInfo()
{
	if (ImGui::CollapsingHeader("Rigid body"))
	{

	}
}