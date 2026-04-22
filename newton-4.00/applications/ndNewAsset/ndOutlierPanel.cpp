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

void ndAssetEditor::ShowOutlierExplorerCollidindPairs(const ndSharedPtr<ndMesh>& root)
{
	const ndCollidingPairs* const collidingPairs = root->GetAsCollidingPairs();

	ImGuiTreeNodeFlags options = 0;
	options |= ImGuiTreeNodeFlags_DefaultOpen;
	options |= ImGuiTreeNodeFlags_OpenOnArrow;

	for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* ptr = collidingPairs->m_collingPairs.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		char name[256];
		ndSharedPtr<ndMeshCollidingPair>& pair = ptr->GetInfo();

		options = ImGuiTreeNodeFlags_Bullet;
		options = options | ((m_currentCollingPairSelection == pair) ? ImGuiTreeNodeFlags_Selected : 0);

		snprintf(name, sizeof(name) - 1, "%s-%s", pair->m_parentNode->GetName().GetStr(), pair->m_childNode->GetName().GetStr());
		if (ImGui::TreeNodeEx(name, options))
		{
			if (ImGui::IsItemClicked())
			{
				m_currentSelection = ndSharedPtr<ndMesh>(nullptr);
				m_currentLoopJointSelection = ndSharedPtr<ndMeshLoopJoint>(nullptr);
				m_currentCollingPairSelection = pair;
			}
			ImGui::TreePop();
		}
	}
}

void ndAssetEditor::ShowOutlierExplorerCloseLoop(const ndSharedPtr<ndMesh>& root)
{
	const ndCloseLoopConstraints* closeLoop = root->GetAsCloseLoopConstraints();

	for (ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* ptr = closeLoop->m_loopJoints.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		char name[256];
		ndSharedPtr<ndMeshLoopJoint>& loop = ptr->GetInfo();

		ImGuiTreeNodeFlags options = ImGuiTreeNodeFlags_Bullet;
		options = options | ((m_currentLoopJointSelection == loop) ? ImGuiTreeNodeFlags_Selected : 0);

		snprintf(name, sizeof(name) - 1, "%s", loop->m_name.GetStr());
		if (ImGui::TreeNodeEx(name, options))
		{
			if (ImGui::IsItemClicked())
			{
				m_currentLoopJointSelection = loop;
				m_currentSelection = ndSharedPtr<ndMesh>(nullptr);
				m_currentCollingPairSelection = ndSharedPtr<ndMeshCollidingPair>(nullptr);
			}
			ImGui::TreePop();
		}
	}
}

void ndAssetEditor::ShowOutlierExplorer(const ndSharedPtr<ndMesh>& root)
{
	ImGuiTreeNodeFlags options = 0;
	options |= ImGuiTreeNodeFlags_DefaultOpen;
	options |= ImGuiTreeNodeFlags_OpenOnArrow;

	ImGuiTreeNodeFlags isSeleted = (m_currentSelection == root) ? ImGuiTreeNodeFlags_Selected : 0;

	char nodeName[256];
	snprintf(nodeName, sizeof(nodeName) - 1, "%s", root->GetName().GetStr());
	if (ImGui::TreeNodeEx(nodeName, options | isSeleted))
	{
		if (root->GetAsCloseLoopConstraints())
		{
			ShowOutlierExplorerCloseLoop(root);
		}
		else if (root->GetAsCollidingPairs())
		{
			ShowOutlierExplorerCollidindPairs(root);
		}
		else
		{
			if (nodeName[0] == 0)
			{
				snprintf(nodeName, sizeof(nodeName) - 1, "unnamed");
			}

			//m_addCollidingBody = false;
			//m_removeCollidingBody = false;
			//m_addCollingPairSelection = -1;
			//m_addCollingPairCandidateSelection = -1;
			//m_secundarySelection.SetCount(0);
			bool isClicked = ImGui::IsItemClicked();
			if (isClicked)
			{
				m_currentLoopJointSelection = ndSharedPtr<ndMeshLoopJoint>(nullptr);
				m_currentCollingPairSelection = ndSharedPtr<ndMeshCollidingPair>(nullptr);
				m_currentSelection = (m_currentSelection != root) ? root : ndSharedPtr<ndMesh>(nullptr);
			}

			//options = 0;
			options |= ImGuiTreeNodeFlags_Bullet;

			if (root->GetJoint())
			{
				if (ImGui::TreeNodeEx("joint", options))
				{
					ImGui::TreePop();
				}
			}

			if (root->GetGeometry())
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
		}

		const ndList<ndSharedPtr<ndMesh>>& children = root->GetChildren();
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