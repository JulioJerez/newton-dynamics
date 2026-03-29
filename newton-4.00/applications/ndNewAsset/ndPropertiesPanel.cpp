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
	}
	
	ImGui::End();
}

void ndAssetEditor::ShowPropertiesMeshInfo()
{
	if (ImGui::CollapsingHeader("Mesh node properties"))
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

		ImGui::SeparatorText("Transform");
		static ndReal v1[3];
		if (ImGui::DragFloat3("position", v1))
		{
			//ndAssert(0);
		};

		static ndReal v[3];
		if (ImGui::DragFloat3("rotation", v))
		{
			//ndAssert(0);
		};

		static ndReal v3[3];
		ImGui::SeparatorText("geomtry transform");
		if (ImGui::DragFloat3("position##1", v3))
		{
			//ndAssert(0);
		};

		static ndReal v2[3];
		if (ImGui::DragFloat3("rotation##1", v2))
		{
			//ndAssert(0);
		};
	}
}
