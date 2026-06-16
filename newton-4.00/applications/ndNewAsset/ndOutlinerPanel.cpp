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

void ndAssetEditor::ShowOutlinerExplorerCollidindPairs(const ndSharedPtr<ndMesh>& node)
{
	const ndCollidingPairs* const collidingPairs = node->GetAsCollidingPairs();

	if (ImGui::IsItemClicked())
	{
		m_subSelection = m_none;
		m_materialIndex = 0;
		m_collidingPairIndex = 0;
		m_customPropertyIndex = 0;
	}

	ImGuiTreeNodeFlags options = 0;
	options |= ImGuiTreeNodeFlags_DefaultOpen;
	options |= ImGuiTreeNodeFlags_OpenOnArrow;

	for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* ptr = collidingPairs->m_collidingPairs.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		char name[256];
		ndSharedPtr<ndMeshCollidingPair>& pair = ptr->GetInfo();
		
		options = ImGuiTreeNodeFlags_Bullet;
		snprintf(name, sizeof(name) - 1, "%s_%s", pair->m_parentNode->GetName().GetStr(), pair->m_childNode->GetName().GetStr());
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

void ndAssetEditor::SelectCurrentNode(ndSharedPtr<ndMesh> node)
{
	ndAssetEditor::ndSubSelectionMode selectionMode = m_subSelection;
	switch (selectionMode)
	{
		case ndAssetEditor::m_none:
		{
			m_currentSelection = *node;
			break;
		}

		case ndAssetEditor::m_loopJoint:
		{
			SetLoopJointSelection(*node);
			break;
		}

		case ndAssetEditor::m_collidingPair:
		{
			SetCollidingSubSelection(*node);
			break;
		}

		case ndAssetEditor::m_transformModifier:
		{
			SetModifierSubSelection(*node);
			break;
		}

		case ndAssetEditor::m_selectCustomProNode:
		{
			SetCustomPropertySelection(*node);
			break;
		}

		case ndAssetEditor::m_alignToTarget:
		{
			m_currentSubSelection = ndWeakPtr<ndMesh>(*node);
			break;
		}

		default:
		{
			ndAssert(0);
			m_currentSelection = *node;
		}
	}

	for (ndMesh* ptr = *node; ptr; ptr = ptr->GetParent())
	{
		ptr->SetToolFlags((node->GetToolFlags() & -2));
	}
}

void ndAssetEditor::ShowOutlinerExplorer(const ndSharedPtr<ndMesh>& node)
{
	ImGuiTreeNodeFlags options = 0;
	options |= ImGuiTreeNodeFlags_OpenOnArrow;
	options |= ImGuiTreeNodeFlags_DefaultOpen;

	ImGuiTreeNodeFlags isSeleted = 0;
	isSeleted = isSeleted | ((*m_currentSelection == *node) ? ImGuiTreeNodeFlags_Selected : 0);

	if (ImGui::TreeNodeEx(node->GetName().GetStr(), options | isSeleted))
	{
		bool isClicked = ImGui::IsItemClicked();
		if (isClicked)
		{
			SelectCurrentNode(node);
		}

		if (!(node->GetToolFlags() & 1))
		{
			if (node->GetAsCloseLoopConstraints())
			{
				ShowOutlierExplorerCloseLoop(node);
			}
			else if (node->GetAsCollidingPairs())
			{
				ShowOutlinerExplorerCollidindPairs(node);
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
				ShowOutlinerExplorer(childMesh);
			}
		}
		ImGui::TreePop();
	}
}

void ndAssetEditor::ShowOutlierPanel()
{
	ImGui::Begin("Ouliner Panel");

	if (*m_mesh)
	{
		if (ImGui::Button("collapse all"))
		{
			auto CollapseAllNodes = [](ndMesh* const node)
			{
				node->SetToolFlags(1);
			};
			m_mesh->NodeIterator(CollapseAllNodes);
			m_mesh->SetToolFlags(0);
		}
		ShowOutlinerExplorer(m_mesh);
	}

	ImGui::End();
}