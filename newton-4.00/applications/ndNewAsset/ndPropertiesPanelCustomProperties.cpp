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
			ndList<ndSharedPtr<ndMeshCustomProperty>>::ndNode* const propNode = FindSelected();
			ndAssert(propNode);

			char propName[256];
			snprintf(propName, sizeof(propName) - 1, "%s", propNode->GetInfo()->m_name.GetStr());
			if (ImGui::InputText("prop name", propName, sizeof(propName) - 1, ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				propNode->GetInfo()->m_name = ndString(propName);
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
			}

			ndList<ndString> nameList;
			ndFixSizeArray<const char*, 1024> names;
			for (ndList<ndSharedPtr<ndMeshCustomProperty>>::ndNode* ptr = propsList.GetFirst(); ptr; ptr = ptr->GetNext())
			{
				ndSharedPtr<ndMeshCustomProperty>& prop = ptr->GetInfo();
				names.PushBack(prop->m_name.GetStr());
			}
			ImGui::ListBox(" ##10", &m_customPropertyIndex, &names[0], names.GetCount(), 4);

			ndInt32 oldPropType = 0;
			if (strcmp(propNode->GetInfo()->ClassName(), ndMeshCustomPropertyString::StaticClassName()) == 0)
			{
				oldPropType = 1;
			}
			else if (strcmp(propNode->GetInfo()->ClassName(), ndMeshCustomPropertyNode::StaticClassName()) == 0)
			{
				oldPropType = 2;
			}
			ndInt32 newPropType = oldPropType;

			ImGui::RadioButton("float property", &newPropType, 0);
			ImGui::RadioButton("string property", &newPropType, 1);
			ImGui::RadioButton("node property", &newPropType, 2);

			ImGui::Text("type:");
			ImGui::SameLine();
			ImGui::Text(propNode->GetInfo()->ClassName());
			if (newPropType != oldPropType)
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

				ndMeshCustomProperty* newPropertyPtr = nullptr;
				if (newPropType == 0)
				{
					newPropertyPtr = (ndMeshCustomProperty*)new ndMeshCustomPropertyFloat(*m_currentSelection);
				}
				else if (newPropType == 1)
				{
					newPropertyPtr = (ndMeshCustomProperty*)new ndMeshCustomPropertyString(*m_currentSelection);
				}
				else
				{
					newPropertyPtr = (ndMeshCustomProperty*)new ndMeshCustomPropertyNode(*m_currentSelection);
				}
				ndSharedPtr<ndMeshCustomProperty> newProperty(newPropertyPtr);
				newProperty->m_name = propNode->GetInfo()->m_name;
				
				propNode->GetInfo() = newProperty;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
			}

			if (newPropType == 0)
			{
				ndMeshCustomPropertyFloat* const floatProp = (ndMeshCustomPropertyFloat*)*propNode->GetInfo();
				ndReal value = floatProp->m_value;
				if (ImGui::InputFloat("value", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
					floatProp->m_value = value;
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				}
			}
			else if (newPropType == 1)
			{
				ndMeshCustomPropertyString* const stringProp = (ndMeshCustomPropertyString*)*propNode->GetInfo();
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
				ndMeshCustomPropertyNode* const prop = (ndMeshCustomPropertyNode*)*propNode->GetInfo();
				if (m_subSelection != m_selectCustomProNode)
				{
					snprintf(propName, sizeof(propName) - 1, "(null)");
					if (*prop->m_value)
					{
						snprintf(propName, sizeof(propName) - 1, "%s", prop->m_value->GetName().GetStr());
					}

					ImGui::Text("target: ");
					ImGui::SameLine();
					ImGui::Text(propName);

					if (ImGui::Button("select target node"))
					{
						m_subSelection = m_selectCustomProNode;
					}
				}
				else
				{
					snprintf(propName, sizeof(propName) - 1, "(null)");
					if (*m_currentSubSelection)
					{
						snprintf(propName, sizeof(propName) - 1, "%s", m_currentSubSelection->GetName().GetStr());
					}

					ImGui::Text("target: ");
					ImGui::SameLine();
					ImGui::Text(propName);

					if (ImGui::Button("exit target node"))
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
						m_subSelection = m_none;
						prop->m_value = ndWeakPtr<ndMesh>(m_currentSubSelection);
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
					}
				}
			}
		}

		if (ImGui::Button("new property"))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

			m_customPropertyIndex = m_currentSelection->GetCustomProperties().GetCount();
			ndSharedPtr<ndMeshCustomProperty> property (new ndMeshCustomPropertyFloat(*m_currentSelection));
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
