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
#include "ndNomalizeMassDistribution.h"

#if 0
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

	ImGui::End();
}

#endif

void ndNomalizeMassDistribution::Execute()
{
	ImGui::Begin("normalize mass distribution", &m_owner->m_toolActive);
	
	ndReal totalMass = 100.0f;
	if (ImGui::InputFloat("total mass", &totalMass, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
	
	}
	ImGui::End();

	if (!m_owner->m_toolActive)
	{
		m_owner->m_currentTool = ndSharedPtr<ndAssetTool>(nullptr);
	}
}