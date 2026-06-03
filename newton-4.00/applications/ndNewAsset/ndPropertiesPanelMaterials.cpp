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
#include "ndDebugDisplayRenderPass.h"

void ndAssetEditor::ShowPropertiesMaterials()
{
	if (!(m_currentSelection && m_currentSelection->GetGeometry()))
	{
		return;
	}

	if (ImGui::CollapsingHeader("Render materials"))
	{
		ndFixSizeArray<const char*, 1024> names;
		ndArray<ndMeshEffect::ndMaterial>& materialsArray = m_currentSelection->GetGeometry()->GetMaterials();
		m_materialIndex = ndClamp(m_materialIndex, 0, ndInt32(materialsArray.GetCount()) - 1);
		for (ndInt32 i = 0; i < ndInt32 (materialsArray.GetCount()); ++i)
		{
			names.PushBack(materialsArray[i].m_name);
		}
		ImGui::ListBox(" ##10", &m_materialIndex, &names[0], names.GetCount(), 4);

		ndMeshEffect::ndMaterial& material = materialsArray[m_materialIndex];

		char tmpName[256];
		snprintf(tmpName, sizeof(tmpName) - 1, "%s", material.m_name);
		if (ImGui::InputText("material name", tmpName, sizeof(tmpName) - 1, ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
			strncpy(material.m_name, tmpName, sizeof(materialsArray[m_currentSelection].m_name) - 1);
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
		}

		auto EditMaterialVectorParam = [this](const char* const label, ndVector& param)
		{
			ndReal real[3];
			real[0] = ndReal(param.m_x);
			real[1] = ndReal(param.m_y);
			real[2] = ndReal(param.m_z);
			if (ImGui::InputFloat3(label, real, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				param.m_x = ndFloat32(real[0]);
				param.m_y = ndFloat32(real[1]);
				param.m_z = ndFloat32(real[2]);

				m_initCamera = false;
				ndWeakPtr<ndMesh> currentSelection(m_currentSelection);
				ndSharedPtr<ndRenderSceneNode> visualMesh(ndRenderMeshLoader::CreateRenderSceneMesh(*GetRenderer(), *m_mesh, GetPath().GetPath()));
				SetVisualScene(m_mesh, visualMesh);
				m_currentSelection = currentSelection;

				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
			};
		};

		auto EditMaterialFloatParam = [this](const char* const label, ndFloat32& param)
		{
			ndReal real = param;
			if (ImGui::InputFloat(label, &real, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				param = ndFloat32(real);

				m_initCamera = false;
				ndWeakPtr<ndMesh> currentSelection(m_currentSelection);
				ndSharedPtr<ndRenderSceneNode> visualMesh(ndRenderMeshLoader::CreateRenderSceneMesh(*GetRenderer(), *m_mesh, GetPath().GetPath()));
				SetVisualScene(m_mesh, visualMesh);
				m_currentSelection = currentSelection;

				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
			};
		};

		//ndVector m_ambient;
		//ndVector m_diffuse;
		//ndVector m_specular;
		//ndVector m_reflection;
		//ndFloat32 m_opacity;
		//ndFloat32 m_shiness;
		//char m_textureName[32];
		EditMaterialVectorParam("ambient", material.m_ambient);
		EditMaterialVectorParam("diffuse", material.m_diffuse);
		EditMaterialVectorParam("specular", material.m_specular);
		EditMaterialVectorParam("reflection", material.m_reflection);
		EditMaterialFloatParam("opacity", material.m_opacity);
		EditMaterialFloatParam("shiness", material.m_shiness);

		snprintf(tmpName, sizeof(tmpName) - 1, "%s", material.m_textureName);
		if (ImGui::InputText("texture", tmpName, sizeof(tmpName) - 1, ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
			strncpy(material.m_textureName, tmpName, sizeof(materialsArray[m_currentSelection].m_name) - 1);

			m_initCamera = false;
			ndWeakPtr<ndMesh> currentSelection(m_currentSelection);
			ndSharedPtr<ndRenderSceneNode> visualMesh(ndRenderMeshLoader::CreateRenderSceneMesh(*GetRenderer(), *m_mesh, GetPath().GetPath()));
			SetVisualScene(m_mesh, visualMesh);
			m_currentSelection = currentSelection;

			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
		}
	}
}
