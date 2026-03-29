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


void ndAssetEditor::ShowOutlierToolBar()
{
	// draw some bottom to control the explorer
	ImGui::Button("undo");
	ImGui::SameLine();
	ImGui::Button("redo");
	ImGui::SameLine();
	if (m_runScene)
	{
		if (ImGui::Button("stop"))
		{
			m_runScene = false;
		}
	}
	else
	{
		if (ImGui::Button("run"))
		{
			m_runScene = true;
		}
	}
}

void ndAssetEditor::ShowOutlierExplorer(const ndSharedPtr<ndMesh>& root)
{
	ImGuiTreeNodeFlags options = 0;
	options |= ImGuiTreeNodeFlags_DefaultOpen;
	options |= ImGuiTreeNodeFlags_OpenOnArrow;

	if (m_currentSelection == root)
	{
		options |= ImGuiTreeNodeFlags_Selected;
	}

	char nodeName[256];
	snprintf(nodeName, sizeof(nodeName) - 1, "%s", root->GetName().GetStr());
	if (nodeName[0] == 0)
	{
		snprintf(nodeName, sizeof(nodeName) - 1, "unnamed");
	}
	if (ImGui::TreeNodeEx(nodeName, options))
	{
		if (ImGui::IsItemClicked())
		{
			//m_currentSelection = root;
			m_currentSelection = (m_currentSelection != root) ? root : ndSharedPtr<ndMesh>(nullptr);
		}

		options = 0;
		options |= ImGuiTreeNodeFlags_Bullet;

		if (root->GetJoint())
		{
			if (ImGui::TreeNodeEx("joint", options))
			{
				ImGui::TreePop();
			}
		}

		if (root->GetMesh())
		{
			if (ImGui::TreeNodeEx("geometry", options))
			{
				ImGui::TreePop();
			}
		}

		if (root->GetRigidBody())
		{
			if (ImGui::TreeNodeEx("rigidBody", options))
			{
				ImGui::TreePop();
			}
		}

		const ndList<ndSharedPtr<ndMesh>>& children = root->GetChildren();
		for (ndList<ndSharedPtr<ndMesh>>::ndNode* child = children.GetFirst(); child; child = child->GetNext())
		{
			ndSharedPtr<ndMesh> childMesh (child->GetInfo());
			ShowOutlierExplorer(childMesh);
		}
		ImGui::TreePop();
	}
}

void ndAssetEditor::ShowOutlierPanel()
{
	bool* open = false;
	ndInt32 flags = 0;
	ImGui::Begin("Oulier Panel", open, flags);
	//ImGui::Begin("Oulier Panel", open);
	//ImGui::PushItemWidth(ImGui::GetFontSize() * 12);
	//ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12);

	WindowFrame frame;
	frame.m_posit = ImGui::GetWindowPos();
	frame.m_size = ImGui::GetWindowSize();
	m_windowSizes.PushBack(frame);

	ShowOutlierToolBar();
	if (*m_model)
	{
		ShowOutlierExplorer(m_model);
	}

	//ImGui::PopItemWidth();
	ImGui::End();
}