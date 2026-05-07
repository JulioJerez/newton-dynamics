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

void ndAssetEditor::ShowOutlierExplorerCollidindPairs(const ndSharedPtr<ndMesh>& node)
{
	const ndCollidingPairs* const collidingPairs = node->GetAsCollidingPairs();

	if (ImGui::IsItemClicked())
	{
		m_subSelection = m_none;
		m_collidingPairIndex = 0;
	}

	ImGuiTreeNodeFlags options = 0;
	options |= ImGuiTreeNodeFlags_DefaultOpen;
	options |= ImGuiTreeNodeFlags_OpenOnArrow;

	for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* ptr = collidingPairs->m_collidingPairs.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		char name[256];
		ndSharedPtr<ndMeshCollidingPair>& pair = ptr->GetInfo();
		
		options = ImGuiTreeNodeFlags_Bullet;
		snprintf(name, sizeof(name) - 1, "%s-%s", pair->m_parentNode->GetName().GetStr(), pair->m_childNode->GetName().GetStr());
		if (ImGui::TreeNodeEx(name, options))
		{
			ImGui::TreePop();
		}
	}
}

void ndAssetEditor::ShowOutlierExplorerCloseLoop(const ndSharedPtr<ndMesh>& root)
{
	const ndCloseLoopConstraints* const closeLoop = root->GetAsCloseLoopConstraints();

	if (ImGui::IsItemClicked())
	{
		m_closeLoopIndex = 0;
		m_subSelection = m_none;
	}

	ImGuiTreeNodeFlags options = 0;
	options |= ImGuiTreeNodeFlags_DefaultOpen;
	options |= ImGuiTreeNodeFlags_OpenOnArrow;

	for (ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* ptr = closeLoop->m_loopJoints.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		ndSharedPtr<ndMeshLoopJoint>& loop = ptr->GetInfo();
		options = ImGuiTreeNodeFlags_Bullet;
		if (ImGui::TreeNodeEx(loop->m_name.GetStr(), options))
		{
			ImGui::TreePop();
		}
	}
}

void ndAssetEditor::ShowOutlierExplorer(const ndSharedPtr<ndMesh>& node)
{
	ImGuiTreeNodeFlags options = 0;
	options |= ImGuiTreeNodeFlags_DefaultOpen;
	options |= ImGuiTreeNodeFlags_OpenOnArrow;

	ImGuiTreeNodeFlags isSeleted = 0;
	isSeleted = isSeleted | ((*m_currentSelection == *node) ? ImGuiTreeNodeFlags_Selected : 0);

	ndAssert(node->GetName().Size());
	if (ImGui::TreeNodeEx(node->GetName().GetStr(), options | isSeleted))
	{
		bool isClicked = ImGui::IsItemClicked();
		if (isClicked)
		{
			m_currentSelection = ndWeakPtr<ndMesh>((ndMesh*)*node);
		}

		if (node->GetAsCloseLoopConstraints())
		{
			ShowOutlierExplorerCloseLoop(node);
		}
		else if (node->GetAsCollidingPairs())
		{
			ShowOutlierExplorerCollidindPairs(node);
		}
		else if (m_currentSelection && m_currentSelection->GetAsMesh())
		{
			options |= ImGuiTreeNodeFlags_Bullet;

			if (node->GetJoint())
			{
				if (ImGui::TreeNodeEx("joint", options))
				{
					ImGui::TreePop();
				}
			}

			if (node->GetGeometry())
			{
				if (ImGui::TreeNodeEx("geometry", options))
				{
					ImGui::TreePop();
				}
			}

			if (node->GetRigidBody())
			{
				if (ImGui::TreeNodeEx("rigidBody", options))
				{
					ImGui::TreePop();
				}
			}
		}

		const ndList<ndSharedPtr<ndMesh>>& children = node->GetChildren();
		for (ndList<ndSharedPtr<ndMesh>>::ndNode* child = children.GetFirst(); child; child = child->GetNext())
		{
			ndSharedPtr<ndMesh> childMesh(child->GetInfo());
			ShowOutlierExplorer(childMesh);
		}
		ImGui::TreePop();
	}
}

void ndAssetEditor::ShowOutlierPanel()
{
	ImGui::Begin("Oulier Panel");

	if (*m_mesh)
	{
		ShowOutlierExplorer(m_mesh);
	}

	ImGui::End();
}