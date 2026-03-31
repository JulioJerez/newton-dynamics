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

void ndAssetEditor::ShowMainToolbar()
{
	ImGui::Begin("Main Toolbar");
	
	/// draw some bottom to control the explorer
	if (ImGui::Button("undo"))
	{
		ndTrace(("undo\n"));
	}
	ImGui::SameLine();
	if (ImGui::Button("redo"))
	{
		ndTrace(("redo\n"));
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