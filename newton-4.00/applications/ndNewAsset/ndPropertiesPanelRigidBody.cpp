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

class ndUndoRedoRigidBody : public ndUndoRedoCommand
{
	public:
	ndUndoRedoRigidBody(ndAssetEditor* const editor, const ndMesh* const selectedNode)
		:ndUndoRedoCommand(editor, selectedNode)
		,m_body(ndSharedPtr<ndMeshBody>(new ndMeshBodyDynamic(*((ndMeshBodyDynamic*)selectedNode->GetRigidBody()->Duplicate()))))
	{
	}

	virtual class ndUndoRedoRigidBody* GetAsUndoRedoRigidBody() const override
	{
		return (ndUndoRedoRigidBody*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_selectedNode == *command.m_selectedNode)
		{
			const ndUndoRedoRigidBody* const other = command.GetAsUndoRedoRigidBody();
			if (other)
			{
				const ndMeshBody* const self = *m_body;
				const ndMeshBody* const otherSelf = *other->m_body;
	
				bool test = (*self == *otherSelf);
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
		ndAssert(m_selectedNode == m_editor->m_currentSelection);
		m_selectedNode->SetRigidBody(m_body);
	}

	ndSharedPtr<ndMeshBody> m_body;
};

void ndAssetEditor::ShowPropertiesRigidBodyInfo()
{
	if (ImGui::CollapsingHeader("Rigid body"))
	{
		ndSharedPtr<ndMeshBody> body(m_currentSelection->GetRigidBody());
		ndAssert(body->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const rigidBody = (ndMeshBodyDynamic*)*body;

		// body mass
		{
			ndReal scalar = ndReal(ndFloat32(1.0f) / rigidBody->m_invMass.m_w);
			if (ImGui::InputFloat("mass", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
				scalar = ndMax(scalar, ndReal(0.001f));
				rigidBody->m_invMass.m_w = ndFloat32(1.0f) / scalar;

				ndMeshBodyKinematic* const kinBody = (ndMeshBodyKinematic*)*body;
				ndSharedPtr<ndShapeInstance> instance(kinBody->m_shapeInstance.CreateObject());
				ndMatrix inertia (instance->CalculateInertia());
				ndMatrix diagMass(ndGetIdentityMatrix());
				for (ndInt32 i = 0; i < 3; ++i)
				{
					diagMass[i][i] = scalar;
				}
				inertia = diagMass * inertia;

				ndVector eigenValues(inertia.EigenVectors());
				rigidBody->m_invMass.m_x = ndFloat32(1.0f) / eigenValues[0];
				rigidBody->m_invMass.m_y = ndFloat32(1.0f) / eigenValues[1];
				rigidBody->m_invMass.m_z = ndFloat32(1.0f) / eigenValues[2];

				ndVector tmp;
				ndVector angles(inertia.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));
				angles.m_w = ndFloat32(0.0f);
				rigidBody->m_inertiaPrincipalAxis = angles;

				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
			};

			scalar = ndReal(rigidBody->m_massVolumeWeigh);
			if (ImGui::InputFloat("mass weigh", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
				scalar = ndMax(scalar, ndReal(0.001f));
				rigidBody->m_massVolumeWeigh = scalar;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
			};
		}

		// body max linear integration step 
		{
			ndReal scalar = ndReal(rigidBody->m_maxLinearStep);
			if (ImGui::InputFloat("linear step", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
				scalar = ndClamp(scalar, ndReal(0.1f), ndReal(30.0f));
				rigidBody->m_maxLinearStep = scalar;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
			};
		}

		// body max angular integration step 
		{
			ndReal scalar = ndReal(rigidBody->m_maxAngleStep);
			if (ImGui::InputFloat("angle step", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
				scalar = ndClamp(scalar, ndReal(10.0f), ndReal(180.0f));
				rigidBody->m_maxAngleStep = scalar;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
			};
		}

		// body intrinsic linear damp 
		{
			ndReal scalar = ndReal(rigidBody->m_intrinsicDamping.m_w);
			if (ImGui::InputFloat("linear damp", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
				scalar = ndClamp(scalar, ndReal(0.0f), ndReal(1.0f));
				rigidBody->m_intrinsicDamping.m_w = scalar;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
			};
		}

		// body intrinsic angular damp
		{
			ndVector vector(rigidBody->m_intrinsicDamping);
			ndReal real[3];
			real[0] = ndReal(vector.m_x);
			real[1] = ndReal(vector.m_y);
			real[2] = ndReal(vector.m_z);
			if (ImGui::InputFloat3("angular damp", real, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
				vector.m_x = real[0];
				vector.m_y = real[1];
				vector.m_z = real[2];
				rigidBody->m_intrinsicDamping = vector;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
			};
		}

		// body center of mass
		{
			ndVector vector(rigidBody->m_localCentreOfMass);
			ndReal real[3];
			real[0] = ndReal(vector.m_x);
			real[1] = ndReal(vector.m_y);
			real[2] = ndReal(vector.m_z);
			if (ImGui::InputFloat3("com", real, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
				vector.m_x = real[0];
				vector.m_y = real[1];
				vector.m_z = real[2];
				rigidBody->m_localCentreOfMass = vector;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
			};
		}

		// body inertia principal inertia
		{
			ndVector vector(rigidBody->m_invMass);
			ndReal real[3];
			real[0] = ndReal(ndFloat32(1.0f) / vector.m_x);
			real[1] = ndReal(ndFloat32(1.0f) / vector.m_y);
			real[2] = ndReal(ndFloat32(1.0f) / vector.m_z);

			if (ImGui::InputFloat3("principal inertia", real, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
				vector.m_x = ndFloat32(1.0f) / real[0];
				vector.m_y = ndFloat32(1.0f) / real[1];
				vector.m_z = ndFloat32(1.0f) / real[2];
				rigidBody->m_invMass = vector;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
			};
		}

		// body principal axis of inertia
		{
			ndVector vector(rigidBody->m_inertiaPrincipalAxis);
			ndReal real[3];
			real[0] = ndReal(vector.m_x);
			real[1] = ndReal(vector.m_y);
			real[2] = ndReal(vector.m_z);

			if (ImGui::InputFloat3("inertia axis", real, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
				vector.m_x = real[0];
				vector.m_y = real[1];
				vector.m_z = real[2];
				vector.m_w = ndReal(0.0f);
				rigidBody->m_inertiaPrincipalAxis = vector;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, *m_currentSelection)));
			};
		}

		EditLoopJoints();
		EditCollidingPair();
		ImGui::Separator();
	}
}

void ndAssetEditor::EditCollidingPair()
{
	ImGui::Separator();
	if (m_subSelection != m_collidingPair)
	{
		if (ImGui::Button("enter colliding pairs"))
		{
			m_subSelection = m_collidingPair;
		}
	}
	else
	{
		if (ImGui::Button("exit colliding pairs"))
		{
			m_subSelection = m_none;
			m_currentSubSelection = ndWeakPtr<ndMesh>(nullptr);
		}

		ImGui::SameLine();
		if (ImGui::Button("add pair"))
		{
			if (m_currentSubSelection)
			{
				AddCollidingPair();
			}
		}
	}
}

void ndAssetEditor::EditLoopJoints()
{
	ImGui::Separator();
	if (m_subSelection != m_loopJoint)
	{
		if (ImGui::Button("enter loop joint"))
		{
			m_subSelection = m_loopJoint;
		}
	}
	else
	{
		if (ImGui::Button("exit loop joints"))
		{
			m_subSelection = m_none;
			m_currentSubSelection = ndWeakPtr<ndMesh>(nullptr);
		}

		ImGui::SameLine();
		if (ImGui::Button("add loop joint"))
		{
			if (m_currentSubSelection)
			{
				AddLoopJoint();
				m_subSelection = m_none;
				m_currentSubSelection = ndWeakPtr<ndMesh>(nullptr);
			}
		}
	}
}

