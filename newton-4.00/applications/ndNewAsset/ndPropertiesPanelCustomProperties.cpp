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

void ndAssetEditor::ShowPropertiesCustomProperties()
{
	if (ImGui::CollapsingHeader("Custom properties"))
	{
		ndList<ndSharedPtr<ndMeshCustomProperty>>& propsList = m_currentSelection->GetCustomProperties();
		auto FindSelected = [this, &propsList]()
		{
			ndInt32 index = 0;
			ndList<ndSharedPtr<ndMeshCustomProperty>>::ndNode* node = propsList.GetFirst();
			while (index != m_customPropertyIndex)
			{
				node = node->GetNext();
				index++;
			}
			return node;
		};

		if (propsList.GetCount())
		{
			ndList<ndString> nameList;
			ndFixSizeArray<const char*, 1024> names;
			for (ndList<ndSharedPtr<ndMeshCustomProperty>>::ndNode* ptr = propsList.GetFirst(); ptr; ptr = ptr->GetNext())
			{
				ndSharedPtr<ndMeshCustomProperty>& prop = ptr->GetInfo();
				names.PushBack(prop->m_name.GetStr());
			}
			ImGui::ListBox(" ##10", &m_customPropertyIndex, &names[0], names.GetCount(), 4);

			ndList<ndSharedPtr<ndMeshCustomProperty>>::ndNode* const node = FindSelected();
			ndAssert(node);
			ndInt32 oldPropType = (strcmp(node->GetInfo()->ClassName(), ndMeshCustomPropertyFloat::StaticClassName()) == 0) ? 0 : 1;
			ndInt32 newPropType = oldPropType;

			ImGui::RadioButton("float property", &newPropType, 0);
			ImGui::RadioButton("string property", &newPropType, 1);

			ImGui::Text("type:");
			ImGui::SameLine();
			ImGui::Text(node->GetInfo()->ClassName());
			if (newPropType != oldPropType)
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				ndMeshCustomProperty* const newPropertyPtr = newPropType ? (ndMeshCustomProperty*)new ndMeshCustomPropertyString() : (ndMeshCustomProperty*)new ndMeshCustomPropertyFloat();
				ndSharedPtr<ndMeshCustomProperty> newProperty(newPropertyPtr);
				newProperty->m_name = node->GetInfo()->m_name;

				node->GetInfo() = newProperty;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
			}

			char propName[256];
			snprintf(propName, sizeof(propName) - 1, "%s", node->GetInfo()->m_name.GetStr());
			if (ImGui::InputText("prop name", propName, sizeof(propName) - 1, ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				node->GetInfo()->m_name = ndString(propName);
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
			}

			if (newPropType)
			{
				ndMeshCustomPropertyString* const stringProp = (ndMeshCustomPropertyString*)*node->GetInfo();
				snprintf(propName, sizeof(propName) - 1, "%s", stringProp->m_value.GetStr());
				if (ImGui::InputText("value", propName, sizeof(propName) - 1, ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
					stringProp->m_value = ndString(propName);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				}
			}
			else
			{
				ndMeshCustomPropertyFloat* const floatProp = (ndMeshCustomPropertyFloat*)*node->GetInfo();
				ndReal value = floatProp->m_value;
				if (ImGui::InputFloat("value", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
					floatProp->m_value = value;
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				}
			}
		}

		if (ImGui::Button("new property"))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

			ndSharedPtr<ndMeshCustomProperty> property (new ndMeshCustomPropertyFloat());
			m_currentSelection->GetCustomProperties().Append(property);
		
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
		}

		if (propsList.GetCount())
		{
			ImGui::SameLine();
			if (ImGui::Button("delete selected"))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

				ndList<ndSharedPtr<ndMeshCustomProperty>>::ndNode* const node = FindSelected();
				propsList.Remove(node);
				m_customPropertyIndex = 0;
	
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
			}
		}
	}
}
