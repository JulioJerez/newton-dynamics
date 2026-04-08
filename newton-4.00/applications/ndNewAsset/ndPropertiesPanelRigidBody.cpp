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

class ndUndoRedoMass : public ndUndoRedoCommand
{
	public:
	ndUndoRedoMass(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());

		m_invMass = body->m_invMass.m_w;
	}

	virtual class ndUndoRedoMass* GetAsUndoRedoMass() const override
	{
		return (ndUndoRedoMass*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoMass* const other = command.GetAsUndoRedoMass();
			if (other)
			{
				if (m_invMass == other->m_invMass)
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

		body->m_invMass = m_invMass;
	}

	ndFloat32 m_invMass;
};

class ndUndoRedoLinearStep : public ndUndoRedoCommand
{
	public:
	ndUndoRedoLinearStep(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());

		m_linearStep = body->m_maxLinearStep;
	}

	virtual class ndUndoRedoLinearStep* GetAsUndoRedoLinearStep() const override
	{
		return (ndUndoRedoLinearStep*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoLinearStep* const other = command.GetAsUndoRedoLinearStep();
			if (other)
			{
				if (m_linearStep == other->m_linearStep)
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

		body->m_maxLinearStep = m_linearStep;
	}

	ndFloat32 m_linearStep;
};

class ndUndoRedoAngleStep : public ndUndoRedoCommand
{
	public:
	ndUndoRedoAngleStep(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());

		m_angleStep = body->m_maxAngleStep;
	}

	virtual class ndUndoRedoAngleStep* GetAsUndoRedoAngleStep() const override
	{
		return (ndUndoRedoAngleStep*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoAngleStep* const other = command.GetAsUndoRedoAngleStep();
			if (other)
			{
				if (m_angleStep == other->m_angleStep)
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

		body->m_maxAngleStep = m_angleStep;
	}

	ndFloat32 m_angleStep;
};

class ndUndoRedoLinearDamp : public ndUndoRedoCommand
{
	public:
	ndUndoRedoLinearDamp(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());

		m_linearDamp = body->m_intrinsicDamping.m_w;
	}

	virtual class ndUndoRedoLinearDamp* GetAsUndoRedoLinearDamp() const override
	{
		return (ndUndoRedoLinearDamp*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoLinearDamp* const other = command.GetAsUndoRedoLinearDamp();
			if (other)
			{
				if (m_linearDamp == other->m_linearDamp)
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

		body->m_intrinsicDamping.m_w = m_linearDamp;
	}

	ndFloat32 m_linearDamp;
};

class ndUndoRedoCenterOfMass : public ndUndoRedoCommand
{
	public:
	ndUndoRedoCenterOfMass(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
		,m_centerOfMass(mesh->GetRigidBody()->m_localCentreOfMass)
	{
	}

	virtual class ndUndoRedoCenterOfMass* GetAsUndoRedoCenterOfMass() const override
	{
		return (ndUndoRedoCenterOfMass*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoCenterOfMass* const other = command.GetAsUndoRedoCenterOfMass();
			if (other)
			{
				const ndVector comDiff(m_centerOfMass - other->m_centerOfMass);
				if (comDiff.DotProduct(comDiff).GetScalar() < ndFloat32(1.0e-6f))
				{
					return false;
				}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		m_mesh->GetRigidBody()->m_localCentreOfMass = m_centerOfMass;
	}

	ndVector m_centerOfMass;
};

class ndUndoRedoAngularDamp : public ndUndoRedoCommand
{
	public:
	ndUndoRedoAngularDamp(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());

		m_angularDamp = body->m_intrinsicDamping;
	}

	virtual ndUndoRedoAngularDamp* GetAsUndoRedoAngularDamp() const override
	{
		return (ndUndoRedoAngularDamp*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoAngularDamp* const other = command.GetAsUndoRedoAngularDamp();
			if (other)
			{
				const ndVector comDiff(ndVector::m_triplexMask & (m_angularDamp - other->m_angularDamp));
				if (comDiff.DotProduct(comDiff).GetScalar() < ndFloat32(1.0e-6f))
				{
					return false;
				}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		m_angularDamp.m_w = m_mesh->GetRigidBody()->m_localCentreOfMass.m_w;
		m_mesh->GetRigidBody()->m_localCentreOfMass = m_angularDamp;
	}

	ndVector m_angularDamp;
};

class ndUndoRedoInertiaAxis : public ndUndoRedoCommand
{
	public:
	ndUndoRedoInertiaAxis(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());

		m_inertiaPrincipalAxis = body->m_inertiaPrincipalAxis;
	}

	virtual ndUndoRedoInertiaAxis* GetAsUndoRedoInertiaAxis() const override
	{
		return (ndUndoRedoInertiaAxis*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoInertiaAxis* const other = command.GetAsUndoRedoInertiaAxis();
			if (other)
			{
				const ndVector comDiff(ndVector::m_triplexMask & (m_inertiaPrincipalAxis - other->m_inertiaPrincipalAxis));
				if (comDiff.DotProduct(comDiff).GetScalar() < ndFloat32(1.0e-6f))
				{
					return false;
				}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());
		body->m_inertiaPrincipalAxis = m_inertiaPrincipalAxis;
	}

	ndVector m_inertiaPrincipalAxis;
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
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMass(this, m_currentSelection)));
				scalar = ndMax(scalar, ndReal(0.001f));
				rigidBody->m_invMass.m_w = ndFloat32(1.0f) / scalar;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMass(this, m_currentSelection)));
			};
		}

		// body max linear integration step 
		{
			ndReal scalar = ndReal(rigidBody->m_maxLinearStep);
			//if (ImGui::DragFloat("linear step", &scalar))
			if (ImGui::InputFloat("linear step", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLinearStep(this, m_currentSelection)));
				scalar = ndClamp(scalar, ndReal(0.1f), ndReal(30.0f));
				rigidBody->m_maxLinearStep = scalar;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLinearStep(this, m_currentSelection)));
			};
		}

		// body max angular integration step 
		{
			ndReal scalar = ndReal(rigidBody->m_maxAngleStep);
			//if (ImGui::DragFloat("angle step", &scalar))
			if (ImGui::InputFloat("angle step", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoAngleStep(this, m_currentSelection)));
				scalar = ndClamp(scalar, ndReal(10.0f), ndReal(180.0f));
				rigidBody->m_maxAngleStep = scalar;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoAngleStep(this, m_currentSelection)));
			};
		}

		// body intrinsic linear damp 
		{
			ndReal scalar = ndReal(rigidBody->m_intrinsicDamping.m_w);
			//if (ImGui::DragFloat("linear Damp", &scalar))
			if (ImGui::InputFloat("linear damp", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLinearDamp(this, m_currentSelection)));
				scalar = ndClamp(scalar, ndReal(0.0f), ndReal(1.0f));
				rigidBody->m_intrinsicDamping.m_w = scalar;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLinearDamp(this, m_currentSelection)));
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
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoAngularDamp(this, m_currentSelection)));
				vector.m_x = real[0];
				vector.m_y = real[1];
				vector.m_z = real[2];
				rigidBody->m_intrinsicDamping = vector;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoAngularDamp(this, m_currentSelection)));
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
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoCenterOfMass(this, m_currentSelection)));
				vector.m_x = real[0];
				vector.m_y = real[1];
				vector.m_z = real[2];
				rigidBody->m_localCentreOfMass = vector;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoCenterOfMass(this, m_currentSelection)));
			};
		}

		// body principal axis of inertia
		{
			ndVector vector(rigidBody->m_inertiaPrincipalAxis);
			ndReal real[3];
			real[0] = ndReal(vector.m_x);
			real[1] = ndReal(vector.m_y);
			real[2] = ndReal(vector.m_z);

			//if (ImGui::DragFloat3("inertia axis", euler))
			if (ImGui::InputFloat3("inertia axis", real, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoInertiaAxis(this, m_currentSelection)));
				vector.m_x = real[0];
				vector.m_y = real[1];
				vector.m_z = real[2];
				vector.m_w = ndReal(0.0f);
				rigidBody->m_inertiaPrincipalAxis = vector;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoInertiaAxis(this, m_currentSelection)));
			};
		}
	}
}



