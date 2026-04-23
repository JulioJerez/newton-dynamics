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

class ndUndoRedoMeshNode : public ndUndoRedoCommand
{
	public:
	ndUndoRedoMeshNode(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
		,m_name(mesh->GetName())
		,m_matrix(mesh->GetMatrix())
		,m_geometryMatrix(mesh->GetGeometryMatrix())
	{
	}

	virtual class ndUndoRedoMeshNode* GetAsUndoRedoMeshNode() const override
	{ 
		return (ndUndoRedoMeshNode*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoMeshNode* const other = command.GetAsUndoRedoMeshNode();
			if (other)
			{
				bool test = m_name == other->m_name;
				test = test && (m_matrix * other->m_matrix.OrthoInverse()).TestIdentity();
				test = test && (m_geometryMatrix * other->m_geometryMatrix.OrthoInverse()).TestIdentity();
				if (test)
				{
					return false;
				}
			}
		}

		return true;
	}
	
	virtual void Undo() override
	{
		ndRenderSceneNode* const entNode = GetSceneNode();
		ndAssert(entNode);

		m_mesh->SetName(m_name);
		m_mesh->SetMatrix(m_matrix);
		m_mesh->SetGeometryMatrix(m_geometryMatrix);

		entNode->m_name = m_name;
		entNode->SetTransform(m_matrix);
		entNode->SetTransform(m_matrix);
		entNode->SetPrimitiveMatrix(m_geometryMatrix);
	}

	ndString m_name;
	ndMatrix m_matrix;
	ndMatrix m_geometryMatrix;
};

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
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, m_currentSelection)));
				ndString newName(nodeName);
				while (m_mesh->FindByName(newName))
				{
					newName += "_1";
				}
				ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());

				m_currentSelection->SetName(newName);
				entNode->m_name = newName;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, m_currentSelection)));
			}
		}

		// show node matrix
		{
			if (m_showPreTransform)
			{
				ImGui::SeparatorText("parent relative Tranform");
			}
			else
			{
				ImGui::SeparatorText("child relative Tranform");
			}

			ndReal position[3];
			ndMatrix matrix(m_currentSelection->GetMatrix());

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			if (m_showPreTransform)
			{
				position[0] = ndReal(matrix.m_posit.m_x);
				position[1] = ndReal(matrix.m_posit.m_y);
				position[2] = ndReal(matrix.m_posit.m_z);
				if (ImGui::InputFloat3("posit", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, m_currentSelection)));
				
					matrix.m_posit = ndVector(position[0], position[1], position[2], ndFloat32(1.0f));
					m_currentSelection->SetMatrix(matrix);
					entNode->SetTransform(matrix);
					entNode->SetTransform(matrix);
				
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, m_currentSelection)));
				}
				
				euler[0] = ndReal(radians[0]);
				euler[1] = ndReal(radians[1]);
				euler[2] = ndReal(radians[2]);
				if (ImGui::InputFloat3("rotation", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, m_currentSelection)));
				
					ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
					newMatrix.m_posit = matrix.m_posit;
					m_currentSelection->SetMatrix(newMatrix);
					entNode->SetTransform(newMatrix);
					entNode->SetTransform(newMatrix);
				
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, m_currentSelection)));
				}
			}
			else
			{
				position[0] = ndReal(0.0f);
				position[1] = ndReal(0.0f);
				position[2] = ndReal(0.0f);

				// this is really nice but creates lots of issues with undo/redo
				if (ImGui::InputFloat3("posit", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, m_currentSelection)));

					const ndVector delta(position[0], position[1], position[2], ndFloat32(0.0f));
					matrix.m_posit += matrix.RotateVector(delta);
					m_currentSelection->SetMatrix(matrix);
					entNode->SetTransform(matrix);
					entNode->SetTransform(matrix);

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, m_currentSelection)));
				}

				euler[0] = ndReal(0.0f);
				euler[1] = ndReal(0.0f);
				euler[2] = ndReal(0.0f);
				if (ImGui::InputFloat3("rotation", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, m_currentSelection)));

					const ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * matrix);
					m_currentSelection->SetMatrix(newMatrix);
					entNode->SetTransform(newMatrix);
					entNode->SetTransform(newMatrix);

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, m_currentSelection)));
				}
			}
		}

		// show geometry node matrix
		if (*m_currentSelection->GetGeometry())
		{
			ImGui::SeparatorText("geomtry transform");
			ndMatrix matrix(m_currentSelection->GetGeometryMatrix());
			ndReal position[3];
			position[0] = ndReal(matrix.m_posit.m_x);
			position[1] = ndReal(matrix.m_posit.m_y);
			position[2] = ndReal(matrix.m_posit.m_z);
			if (ImGui::InputFloat3("posit##1", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
				ndAssert(entNode);
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, m_currentSelection)));

				matrix.m_posit.m_x = position[0];
				matrix.m_posit.m_y = position[1];
				matrix.m_posit.m_z = position[2];
				m_currentSelection->SetGeometryMatrix(matrix);
				entNode->SetPrimitiveMatrix(matrix);

				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, m_currentSelection)));
			};

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
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, m_currentSelection)));

				ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
				newMatrix.m_posit = matrix.m_posit;
				m_currentSelection->SetGeometryMatrix(newMatrix);
				entNode->SetPrimitiveMatrix(matrix);
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, m_currentSelection)));
			};
		}
	}
}

