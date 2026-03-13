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

void ndAssetEditor::ShowOutlier()
{
	bool* open = false;

	ndInt32 flags = 0;
	ImGui::Begin("Oulier", open, flags);
	ImGui::PushItemWidth(ImGui::GetFontSize() * 12);

	ShowOutlierToolBar();
	if (*m_model)
	{
		ShowOutlierExplorer(*m_model);
	}

	ImGui::End();
}

void ndAssetEditor::ShowOutlierToolBar()
{
	// draw some bottom to control the explorer
	ImGui::Button("undo");
	ImGui::SameLine();
	ImGui::Button("redo");
}

void ndAssetEditor::ShowOutlierExplorer(ndMesh* const root)
{

	// draw explorer tree.
//if (ImGui::TreeNode("Mesh Model"))
//{
//	//IMGUI_DEMO_MARKER("Widgets/Trees/Basic trees");
//	if (ImGui::TreeNode("Basic trees"))
//	{
//		for (int i = 0; i < 5; i++)
//		{
//			// Use SetNextItemOpen() so set the default state of a node to be open. We could
//			// also use TreeNodeEx() with the ImGuiTreeNodeFlags_DefaultOpen flag to achieve the same thing!
//			if (i == 0)
//				ImGui::SetNextItemOpen(true, ImGuiCond_Once);
//
//			if (ImGui::TreeNode((void*)(intptr_t)i, "Child %d", i))
//			{
//				ImGui::Text("blah blah");
//				ImGui::SameLine();
//				if (ImGui::SmallButton("button")) {}
//				ImGui::TreePop();
//			}
//		}
//		ImGui::TreePop();
//	}
//
//	//IMGUI_DEMO_MARKER("Widgets/Trees/Advanced, with Selectable nodes");
//	if (ImGui::TreeNode("Advanced, with Selectable nodes"))
//	{
//		//HelpMarker("This is a more typical looking tree with selectable nodes.\n"
//		//	"Click to select, CTRL+Click to toggle, click on arrows or double-click to open.");
//		static ImGuiTreeNodeFlags base_flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick | ImGuiTreeNodeFlags_SpanAvailWidth;
//		static bool align_label_with_current_x_position = false;
//		static bool test_drag_and_drop = false;
//		ImGui::CheckboxFlags("ImGuiTreeNodeFlags_OpenOnArrow", &base_flags, ImGuiTreeNodeFlags_OpenOnArrow);
//		ImGui::CheckboxFlags("ImGuiTreeNodeFlags_OpenOnDoubleClick", &base_flags, ImGuiTreeNodeFlags_OpenOnDoubleClick);
//		//ImGui::CheckboxFlags("ImGuiTreeNodeFlags_SpanAvailWidth", &base_flags, ImGuiTreeNodeFlags_SpanAvailWidth); ImGui::SameLine(); HelpMarker("Extend hit area to all available width instead of allowing more items to be laid out after the node.");
//		ImGui::CheckboxFlags("ImGuiTreeNodeFlags_SpanFullWidth", &base_flags, ImGuiTreeNodeFlags_SpanFullWidth);
//		ImGui::Checkbox("Align label with current X position", &align_label_with_current_x_position);
//		ImGui::Checkbox("Test tree node as drag source", &test_drag_and_drop);
//		ImGui::Text("Hello!");
//		if (align_label_with_current_x_position)
//			ImGui::Unindent(ImGui::GetTreeNodeToLabelSpacing());
//
//		// 'selection_mask' is dumb representation of what may be user-side selection state.
//		//  You may retain selection state inside or outside your objects in whatever format you see fit.
//		// 'node_clicked' is temporary storage of what node we have clicked to process selection at the end
//		/// of the loop. May be a pointer to your own node type, etc.
//		static int selection_mask = (1 << 2);
//		int node_clicked = -1;
//		for (int i = 0; i < 6; i++)
//		{
//			// Disable the default "open on single-click behavior" + set Selected flag according to our selection.
//			// To alter selection we use IsItemClicked() && !IsItemToggledOpen(), so clicking on an arrow doesn't alter selection.
//			ImGuiTreeNodeFlags node_flags = base_flags;
//			const bool is_selected = (selection_mask & (1 << i)) != 0;
//			if (is_selected)
//				node_flags |= ImGuiTreeNodeFlags_Selected;
//			if (i < 3)
//			{
//				// Items 0..2 are Tree Node
//				bool node_open = ImGui::TreeNodeEx((void*)(intptr_t)i, node_flags, "Selectable Node %d", i);
//				if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
//					node_clicked = i;
//				if (test_drag_and_drop && ImGui::BeginDragDropSource())
//				{
//					ImGui::SetDragDropPayload("_TREENODE", NULL, 0);
//					ImGui::Text("This is a drag and drop source");
//					ImGui::EndDragDropSource();
//				}
//				if (node_open)
//				{
//					ImGui::BulletText("Blah blah\nBlah Blah");
//					ImGui::TreePop();
//				}
//			}
//			else
//			{
//				// Items 3..5 are Tree Leaves
//				// The only reason we use TreeNode at all is to allow selection of the leaf. Otherwise we can
//				// use BulletText() or advance the cursor by GetTreeNodeToLabelSpacing() and call Text().
//				node_flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen; // ImGuiTreeNodeFlags_Bullet
//				ImGui::TreeNodeEx((void*)(intptr_t)i, node_flags, "Selectable Leaf %d", i);
//				if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
//					node_clicked = i;
//				if (test_drag_and_drop && ImGui::BeginDragDropSource())
//				{
//					ImGui::SetDragDropPayload("_TREENODE", NULL, 0);
//					ImGui::Text("This is a drag and drop source");
//					ImGui::EndDragDropSource();
//				}
//			}
//		}
//		if (node_clicked != -1)
//		{
//			// Update selection state
//			// (process outside of tree loop to avoid visual inconsistencies during the clicking frame)
//			if (ImGui::GetIO().KeyCtrl)
//				selection_mask ^= (1 << node_clicked);          // CTRL+click to toggle
//			else //if (!(selection_mask & (1 << node_clicked))) // Depending on selection behavior you want, may want to preserve selection when clicking on item that is part of the selection
//				selection_mask = (1 << node_clicked);           // Click to single-select
//		}
//		if (align_label_with_current_x_position)
//			ImGui::Indent(ImGui::GetTreeNodeToLabelSpacing());
//		ImGui::TreePop();
//	}
//	ImGui::TreePop();
//}

	ImGuiTreeNodeFlags options = 0;
	//options |= ImGuiTreeNodeFlags_SpanAvailWidth;
	options |= ImGuiTreeNodeFlags_DefaultOpen;
	//options |= ImGuiTreeNodeFlags_Bullet;
	//options |= ImGuiTreeNodeFlags_Leaf;

	if (ImGui::TreeNodeEx(root->GetName().GetStr(), options))
	{
		//static bool closable_group = true;
		options = 0;
		options |= ImGuiTreeNodeFlags_Bullet;
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
				//ImGui::Text("IsItemHovered: %d", ImGui::IsItemHovered());
				//for (int i = 0; i < 5; i++)
				//	ImGui::Text("More content %d", i);

				ImGui::TreePop();
			}
		}

		const ndList<ndSharedPtr<ndMesh>>& children = root->GetChildren();
		for (ndList<ndSharedPtr<ndMesh>>::ndNode* child = children.GetFirst(); child; child = child->GetNext())
		{
			ShowOutlierExplorer(*child->GetInfo());
		}
		ImGui::TreePop();
	}
}
