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
	ndUndoRedoRigidBody(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());

		m_inverseMass = body->m_invMass;
		m_angleStep = body->m_maxAngleStep;
		m_linearStep = body->m_maxLinearStep;
		m_intrisicDamp = body->m_intrinsicDamping;
		m_centerOfMass = body->m_localCentreOfMass;
		m_massVolumeWeigh = body->m_massVolumeWeigh;
		m_inertiaPrincipalAxis = body->m_inertiaPrincipalAxis;
	}

	virtual class ndUndoRedoRigidBody* GetAsUndoRedoRigidBody() const override
	{
		return (ndUndoRedoRigidBody*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoRigidBody* const other = command.GetAsUndoRedoRigidBody();
			if (other)
			{
				bool test = (m_inverseMass - other->m_inverseMass).DotProduct(m_inverseMass - other->m_inverseMass).GetScalar() < 0.0001f;
				test = test && (m_intrisicDamp - other->m_intrisicDamp).DotProduct(m_intrisicDamp - other->m_intrisicDamp).GetScalar() < 0.0001f;
				test = test && (m_centerOfMass - other->m_centerOfMass).DotProduct(m_centerOfMass - other->m_centerOfMass).GetScalar() < 0.0001f;
				test = test && (m_inertiaPrincipalAxis - other->m_inertiaPrincipalAxis).DotProduct(m_inertiaPrincipalAxis - other->m_inertiaPrincipalAxis).GetScalar() < 0.0001f;
				test = test && (m_angleStep == other->m_angleStep);
				test = test && (m_linearStep == other->m_linearStep);
				test = test && (m_massVolumeWeigh == other->m_massVolumeWeigh);

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
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());

		body->m_invMass = m_inverseMass;
		body->m_intrinsicDamping = m_intrisicDamp;
		body->m_localCentreOfMass = m_centerOfMass;
		body->m_inertiaPrincipalAxis = m_inertiaPrincipalAxis;
		body->m_maxAngleStep = m_angleStep;
		body->m_maxLinearStep = m_linearStep;
		body->m_massVolumeWeigh = m_massVolumeWeigh;
	}

	ndVector m_inverseMass;
	ndVector m_intrisicDamp;
	ndVector m_centerOfMass;
	ndVector m_inertiaPrincipalAxis;
	ndFloat32 m_angleStep;
	ndFloat32 m_linearStep;
	ndFloat32 m_massVolumeWeigh;
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
			//if (ImGui::DragFloat("mass", &scalar))
			if (ImGui::InputFloat("mass", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
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

				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
			};

			scalar = ndReal(rigidBody->m_massVolumeWeigh);
			if (ImGui::InputFloat("mass weigh", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
				scalar = ndMax(scalar, ndReal(0.001f));
				rigidBody->m_massVolumeWeigh = scalar;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
			};
		}

		// body max linear integration step 
		{
			ndReal scalar = ndReal(rigidBody->m_maxLinearStep);
			//if (ImGui::DragFloat("linear step", &scalar))
			if (ImGui::InputFloat("linear step", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
				scalar = ndClamp(scalar, ndReal(0.1f), ndReal(30.0f));
				rigidBody->m_maxLinearStep = scalar;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
			};
		}

		// body max angular integration step 
		{
			ndReal scalar = ndReal(rigidBody->m_maxAngleStep);
			//if (ImGui::DragFloat("angle step", &scalar))
			if (ImGui::InputFloat("angle step", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
				scalar = ndClamp(scalar, ndReal(10.0f), ndReal(180.0f));
				rigidBody->m_maxAngleStep = scalar;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
			};
		}

		// body intrinsic linear damp 
		{
			ndReal scalar = ndReal(rigidBody->m_intrinsicDamping.m_w);
			//if (ImGui::DragFloat("linear Damp", &scalar))
			if (ImGui::InputFloat("linear damp", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
				scalar = ndClamp(scalar, ndReal(0.0f), ndReal(1.0f));
				rigidBody->m_intrinsicDamping.m_w = scalar;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
			};
		}

		// body intrinsic angular damp
		{
			ndVector vector(rigidBody->m_intrinsicDamping);
			ndReal real[3];
			real[0] = ndReal(vector.m_x);
			real[1] = ndReal(vector.m_y);
			real[2] = ndReal(vector.m_z);
			//if (ImGui::DragFloat3("angular damp", real))
			if (ImGui::InputFloat3("angular damp", real, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
				vector.m_x = real[0];
				vector.m_y = real[1];
				vector.m_z = real[2];
				rigidBody->m_intrinsicDamping = vector;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
			};
		}

		// body center of mass
		{
			ndVector vector(rigidBody->m_localCentreOfMass);
			ndReal real[3];
			real[0] = ndReal(vector.m_x);
			real[1] = ndReal(vector.m_y);
			real[2] = ndReal(vector.m_z);
			//if (ImGui::DragFloat3("com", real))
			if (ImGui::InputFloat3("com", real, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
				vector.m_x = real[0];
				vector.m_y = real[1];
				vector.m_z = real[2];
				rigidBody->m_localCentreOfMass = vector;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
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
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
				vector.m_x = ndFloat32(1.0f) / real[0];
				vector.m_y = ndFloat32(1.0f) / real[1];
				vector.m_z = ndFloat32(1.0f) / real[2];
				rigidBody->m_invMass = vector;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
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
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
				vector.m_x = real[0];
				vector.m_y = real[1];
				vector.m_z = real[2];
				vector.m_w = ndReal(0.0f);
				rigidBody->m_inertiaPrincipalAxis = vector;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoRigidBody(this, m_currentSelection)));
			};
		}
	}
}

