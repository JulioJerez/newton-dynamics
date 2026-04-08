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

class ndUndoRedoName : public ndUndoRedoCommand
{
	public:
	ndUndoRedoName(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
		,m_name(mesh->GetName())
	{
	}

	virtual class ndUndoRedoName* GetAsUndoRedoName() const override
	{ 
		return (ndUndoRedoName*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoName* const other = command.GetAsUndoRedoName();
			if (other)
			{
				if (m_name == other->m_name)
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
		entNode->m_name = m_name;
	}
	ndString m_name;
};

class ndUndoRedoTransform : public ndUndoRedoCommand
{
	public:
	ndUndoRedoTransform(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
		,m_matrix(mesh->GetMatrix())
	{
	}

	virtual class ndUndoRedoTransform* GetAsUndoRedoTransform() const override
	{
		return (ndUndoRedoTransform*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoTransform* const other = command.GetAsUndoRedoTransform();
			if (other)
			{
				ndMatrix matrix(m_matrix * other->m_matrix.OrthoInverse());
				if (matrix.TestIdentity())
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

		m_mesh->SetMatrix(m_matrix);
		entNode->SetTransform(m_matrix);
		entNode->SetTransform(m_matrix);
	}

	ndMatrix m_matrix;
};

class ndUndoRedoGeometryTransform : public ndUndoRedoCommand
{
	public:
	ndUndoRedoGeometryTransform(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
		,m_matrix(mesh->GetGeometryMatrix())
	{
	}

	virtual class ndUndoRedoGeometryTransform* GetAsUndoRedoGeometryTransform() const override
	{
		return (ndUndoRedoGeometryTransform*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoGeometryTransform* const other = command.GetAsUndoRedoGeometryTransform();
			if (other)
			{
				ndMatrix matrix(m_matrix * other->m_matrix.OrthoInverse());
				if (matrix.TestIdentity())
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
		m_mesh->SetGeometryMatrix(m_matrix);
		entNode->SetPrimitiveMatrix(m_matrix);
	}

	ndMatrix m_matrix;
};


void ndAssetEditor::ShowPropertiesMeshInfo()
{
	if (ImGui::CollapsingHeader("mesh node"))
	{
		char nodeName[256];
		snprintf(nodeName, sizeof(nodeName) - 1, "%s", m_currentSelection->GetName().GetStr());
		if (ImGui::InputText("Name", nodeName, sizeof(nodeName) - 1, ImGuiInputTextFlags_EnterReturnsTrue))
		{
			if (strcmp(m_currentSelection->GetName().GetStr(), nodeName))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoName(this, m_currentSelection)));
				ndString newName(nodeName);
				while (m_mesh->FindByName(newName))
				{
					newName += "_1";
				}
				ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());

				m_currentSelection->SetName(newName);
				entNode->m_name = newName;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoName(this, m_currentSelection)));
			}
		}

		// show node matrix
		{
			ImGui::SeparatorText("Transform");

			ndReal position[3];
			ndMatrix matrix(m_currentSelection->GetMatrix());

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			if (m_relativeTransformProperties)
			{
				position[0] = ndReal(0.0f);
				position[1] = ndReal(0.0f);
				position[2] = ndReal(0.0f);

				// this is really nice but creates lots of issues with undo/redo
				//if (ImGui::DragFloat3("position", position, 0.01f))
				if (ImGui::InputFloat3("rel posit", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoTransform(this, m_currentSelection)));

					const ndVector delta(position[0], position[1], position[2], ndFloat32(1.0f));
					matrix.m_posit += matrix.RotateVector(delta);
					m_currentSelection->SetMatrix(matrix);
					entNode->SetTransform(matrix);
					entNode->SetTransform(matrix);

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoTransform(this, m_currentSelection)));
				}

				euler[0] = ndReal(0.0f);
				euler[1] = ndReal(0.0f);
				euler[2] = ndReal(0.0f);
				//if (ImGui::DragFloat3("rel rotation", euler))
				if (ImGui::InputFloat3("rel rotation", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoTransform(this, m_currentSelection)));

					const ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * matrix);
					m_currentSelection->SetMatrix(newMatrix);
					entNode->SetTransform(newMatrix);
					entNode->SetTransform(newMatrix);

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoTransform(this, m_currentSelection)));
				};
			}
			else
			{
				position[0] = ndReal(matrix.m_posit.m_x);
				position[1] = ndReal(matrix.m_posit.m_y);
				position[2] = ndReal(matrix.m_posit.m_z);
				//if (ImGui::DragFloat3("posit", position, 0.01f))
				if (ImGui::InputFloat3("posit", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoTransform(this, m_currentSelection)));

					matrix.m_posit = ndVector (position[0], position[1], position[2], ndFloat32(1.0f));
					m_currentSelection->SetMatrix(matrix);
					entNode->SetTransform(matrix);
					entNode->SetTransform(matrix);

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoTransform(this, m_currentSelection)));
				}

				euler[0] = ndReal(radians[0]);
				euler[1] = ndReal(radians[1]);
				euler[2] = ndReal(radians[2]);
				//if (ImGui::DragFloat3("rotation", euler))
				if (ImGui::InputFloat3("rotation", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoTransform(this, m_currentSelection)));

					ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
					newMatrix.m_posit = matrix.m_posit;
					m_currentSelection->SetMatrix(newMatrix);
					entNode->SetTransform(newMatrix);
					entNode->SetTransform(newMatrix);

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoTransform(this, m_currentSelection)));
				};
			}
		}

		// show geometry node matrix
		if (*m_currentSelection->GetMesh())
		{
			ImGui::SeparatorText("geomtry transform");
			ndMatrix matrix(m_currentSelection->GetGeometryMatrix());
			ndReal position[3];
			position[0] = ndReal(matrix.m_posit.m_x);
			position[1] = ndReal(matrix.m_posit.m_y);
			position[2] = ndReal(matrix.m_posit.m_z);
			//if (ImGui::DragFloat3("position##1", position))
			if (ImGui::InputFloat3("posit##1", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
				ndAssert(entNode);
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoGeometryTransform(this, m_currentSelection)));

				matrix.m_posit.m_x = position[0];
				matrix.m_posit.m_y = position[1];
				matrix.m_posit.m_z = position[2];
				m_currentSelection->SetGeometryMatrix(matrix);
				entNode->SetPrimitiveMatrix(matrix);

				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoGeometryTransform(this, m_currentSelection)));
			};

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			euler[0] = ndReal(radians[0]);
			euler[1] = ndReal(radians[1]);
			euler[2] = ndReal(radians[2]);
			//if (ImGui::DragFloat3("rotation##1", euler))
			if (ImGui::InputFloat3("rotation##1", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
				ndAssert(entNode);
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoGeometryTransform(this, m_currentSelection)));

				ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
				newMatrix.m_posit = matrix.m_posit;
				m_currentSelection->SetGeometryMatrix(newMatrix);
				entNode->SetPrimitiveMatrix(matrix);
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoGeometryTransform(this, m_currentSelection)));
			};
		}
	}
}

