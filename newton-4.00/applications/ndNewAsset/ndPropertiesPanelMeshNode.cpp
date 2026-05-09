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

ndUndoRedoMeshNode::ndUndoRedoMeshNode(ndAssetEditor* const editor, const ndMesh* const selectedNode)
	:ndUndoRedoCommand(editor, selectedNode)
	,m_copy(editor->m_mesh->CreateClone())
{
	m_selectedNode = m_copy->FindByName(m_selectedNodeName);
}

ndUndoRedoMeshNode* ndUndoRedoMeshNode::GetAsUndoRedoMeshNode() const
{ 
	return (ndUndoRedoMeshNode*)this;
}

bool ndUndoRedoMeshNode::operator!=(const ndUndoRedoCommand& command) const
{
	if (*m_selectedNode == *command.m_selectedNode)
	{
		ndUndoRedoMeshNode* const other = command.GetAsUndoRedoMeshNode();
		if (other)
		{
			bool test = true;
			auto CompareNodes = [this, &test, other](ndMesh* node)
			{
				const ndMesh* const otherNode = other->m_copy->FindByName(node->GetName());
				test = test && (otherNode ? true : false);
				test = test && (*node == *otherNode);
			};
			m_copy->NodeIterator(CompareNodes);
			
			if (test)
			{
				return false;
			}
		}
	}

	return true;
}
	
void ndUndoRedoMeshNode::Undo()
{
	ndSharedPtr<ndRenderSceneNode> visualMesh(ndRenderMeshLoader::CreateRenderSceneMesh(*m_editor->GetRenderer(), *m_copy, ndGetPath(m_editor->GetPath())));

	m_editor->m_initCamera = false;
	m_editor->SetVisualScene(m_copy, visualMesh);
	m_editor->m_currentSelection = m_selectedNode;
}

void ndAssetEditor::ApplyNodeTransform(const ndMatrix& matrix, ndRenderSceneNode* const entNode)
{
	const ndMatrix localMatrix(m_currentSelection->GetMatrix() * matrix.OrthoInverse());
	m_currentSelection->SetMatrix(matrix);
	entNode->SetTransform(matrix);
	entNode->SetTransform(matrix);
	if (m_transformPivotOnly)
	{
		const ndMatrix geoMatrix(m_currentSelection->GetGeometryMatrix() * localMatrix);
		entNode->SetPrimitiveMatrix(geoMatrix);
		m_currentSelection->SetGeometryMatrix(geoMatrix);

		for (ndList<ndSharedPtr<ndMesh>>::ndNode* childPtr = m_currentSelection->GetChildren().GetFirst(); childPtr; childPtr = childPtr->GetNext())
		{
			ndMesh* const child = *childPtr->GetInfo();
			child->SetMatrix(child->GetMatrix() * localMatrix);
		}

		for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* childPtr = entNode->GetChildren().GetFirst(); childPtr; childPtr = childPtr->GetNext())
		{
			ndRenderSceneNode* const child = *childPtr->GetInfo();
			const ndMatrix counterMatrix(child->GetMatrix() * localMatrix);
			child->SetTransform(counterMatrix);
			child->SetTransform(counterMatrix);
		}
	}
}

void ndAssetEditor::ShowPropertiesMeshInfo()
{
	if (ImGui::CollapsingHeader("Mesh node"))
	{
		char nodeName[256];
		snprintf(nodeName, sizeof(nodeName) - 1, "%s", m_currentSelection->GetName().GetStr());
		if (ImGui::InputText("Name", nodeName, sizeof(nodeName) - 1, ImGuiInputTextFlags_EnterReturnsTrue))
		{
			if (strcmp(m_currentSelection->GetName().GetStr(), nodeName))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				ndString newName(nodeName);
				while (m_mesh->FindByName(newName))
				{
					newName += "_1";
				}
				ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());

				m_currentSelection->SetName(newName);
				entNode->m_name = newName;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
			}
		}
		ImGui::Checkbox("override transform", &m_showTransformValues);
		ImGui::Checkbox("transform pivot only", &m_transformPivotOnly);

		// show node matrix
		{
			//if (m_showTransformValues)
			//{
			//	ImGui::SeparatorText("parent relative Tranform");
			//}
			//else
			//{
			//	ImGui::SeparatorText("child relative Tranform");
			//}

			ndReal position[3];
			ndMatrix matrix(m_currentSelection->GetMatrix());

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			ImGui::SeparatorText("mode transform");
			if (m_showTransformValues)
			{
				position[0] = ndReal(matrix.m_posit.m_x);
				position[1] = ndReal(matrix.m_posit.m_y);
				position[2] = ndReal(matrix.m_posit.m_z);
				if (ImGui::InputFloat3("posit", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				
					matrix.m_posit = ndVector(position[0], position[1], position[2], ndFloat32(1.0f));
					ApplyNodeTransform(matrix, entNode);
				
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				}

				euler[0] = ndReal(radians[0]);
				euler[1] = ndReal(radians[1]);
				euler[2] = ndReal(radians[2]);
				if (ImGui::InputFloat3("rotation", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				
					ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
					newMatrix.m_posit = matrix.m_posit;
					ApplyNodeTransform(newMatrix, entNode);
				
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				}
			}
			else
			{
				position[0] = ndReal(0.0f);
				position[1] = ndReal(0.0f);
				position[2] = ndReal(0.0f);
				if (ImGui::InputFloat3("posit", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

					const ndVector delta(position[0], position[1], position[2], ndFloat32(0.0f));
					matrix.m_posit += matrix.RotateVector(delta);
					ApplyNodeTransform(matrix, entNode);

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				}

				euler[0] = ndReal(0.0f);
				euler[1] = ndReal(0.0f);
				euler[2] = ndReal(0.0f);
				if (ImGui::InputFloat3("rotation", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

					const ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * matrix);
					ApplyNodeTransform(newMatrix, entNode);

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				}
			}
		}

		// show geometry node matrix
		if (*m_currentSelection->GetGeometry())
		{
			ImGui::SeparatorText("geomtry transform");
			if (m_showTransformValues)
			{
				ndMatrix matrix(m_currentSelection->GetGeometryMatrix());
				ndReal position[3];
				position[0] = ndReal(matrix.m_posit.m_x);
				position[1] = ndReal(matrix.m_posit.m_y);
				position[2] = ndReal(matrix.m_posit.m_z);
				if (ImGui::InputFloat3("posit##1", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

					matrix.m_posit.m_x = position[0];
					matrix.m_posit.m_y = position[1];
					matrix.m_posit.m_z = position[2];
					m_currentSelection->SetGeometryMatrix(matrix);
					entNode->SetPrimitiveMatrix(matrix);

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				}

				ndReal euler[3];
				ndVector tmp;
				ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

				euler[0] = ndReal(radians[0]);
				euler[1] = ndReal(radians[1]);
				euler[2] = ndReal(radians[2]);
				if (ImGui::InputFloat3("rotation##1", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

					ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
					newMatrix.m_posit = matrix.m_posit;
					m_currentSelection->SetGeometryMatrix(newMatrix);
					entNode->SetPrimitiveMatrix(matrix);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				}
			}
			else
			{
				ndMatrix matrix(m_currentSelection->GetGeometryMatrix());
				ndReal position[3];
				position[0] = ndReal(0.0f);
				position[1] = ndReal(0.0f);
				position[2] = ndReal(0.0f);
				if (ImGui::InputFloat3("posit##1", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

					const ndVector delta(position[0], position[1], position[2], ndFloat32(0.0f));
					matrix.m_posit += matrix.RotateVector(delta);
					m_currentSelection->SetGeometryMatrix(matrix);
					entNode->SetPrimitiveMatrix(matrix);

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				};

				ndReal euler[3];
				euler[0] = ndReal(0.0f);
				euler[1] = ndReal(0.0f);
				euler[2] = ndReal(0.0f);
				if (ImGui::InputFloat3("rotation##1", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

					const ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * matrix);
					m_currentSelection->SetGeometryMatrix(newMatrix);
					entNode->SetPrimitiveMatrix(newMatrix);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				}
			}
		}
	}
}

