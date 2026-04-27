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
#include "ndUndoRedo.h"
#include "ndAssetEditor.h"
#include "ndEditorCameraFlyby.h"

void ndAssetEditor::ShowMainToolbar()
{
	ImGui::Begin("Main Toolbar");
	
	if (ImGui::Button("undo"))
	{
		m_undoRedo.Undo(this);
	}
	ImGui::SameLine();
	if (ImGui::Button("redo"))
	{
		m_undoRedo.Redo(this);
	}

	ImGui::SameLine();
	if (m_lockSelection)
	{
		if (ImGui::Button("unlock selection"))
		{
			m_lockSelection = false;
		}
	}
	else
	{
		if (ImGui::Button("lock Selection"))
		{
			m_lockSelection = true;
		}
	}

	ImGui::SameLine();
	struct Names
	{
		ndCameraMode m_mode;
		const char* m_label;
	};
	Names names[] = 
	{
		{m_free, "free camera" },
		{m_backView, "back view" },
		{m_frontView, "front view" },
		{m_sideLeftView, "left side view" },
		{m_sideRrightView, "right side view" },
	};

	static int xxxx = 0;
	if (ImGui::BeginCombo(" ##1", names[xxxx].m_label, ImGuiComboFlags_MaxSize, 160))
	{
		for (ndInt32 i = 0; i < ndInt32 (sizeof(names) / sizeof(names[0])); ++i)
		{
			bool isSelected = strcmp(names[i].m_label, names[xxxx].m_label) ? false : true;
			if (ImGui::Selectable(names[i].m_label, isSelected))
			{
				xxxx = i;
				ndEditorCameraFlyby* const camera = (ndEditorCameraFlyby*)*m_defaultCamera;
				camera->SetView(names[i].m_mode);
			}
		}
	
		ImGui::EndCombo();
	}

	ImGui::End();
}