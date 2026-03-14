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
	//bool* open = false;
	//ndInt32 flags = 0;
	ImGui::Begin("Properties Panel");

	ShowPropertiesMeshInfo();

	
	ImGui::End();
}

void ndAssetEditor::ShowPropertiesMeshInfo()
{
	if (ImGui::CollapsingHeader("Mesh properties"))
	{
		static char xxxxx0[256];
		static char xxxxx1[256];
		//ImGuiInputTextFlags flags = 0;
		//flags = ImGuiInputTextFlags_CallbackAlways;
		//flags |= ImGuiInputTextFlags_CallbackHistory;
		//ImGui::InputText("node Name", xxx, 255, flags, ImGuiInputTextCallback callback = NULL, void* user_data = NULL);
		ImGui::InputText("node Name0", xxxxx0, 255);
		if (ImGui::InputText("node Name1", xxxxx1, 255))
		{

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
