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

class ndUndoRedoStructuralJoint : public ndUndoRedoCommand
{
	public:
	ndUndoRedoStructuralJoint(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
		,m_joint(ndSharedPtr<ndMeshJoint>(mesh->GetJoint()->Duplicate()))
	{
	}

	virtual class ndUndoRedoStructuralJoint* GetAsUndoRedoStructuralJoint() const override
	{
		return (ndUndoRedoStructuralJoint*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoStructuralJoint* const other = command.GetAsUndoRedoStructuralJoint();
			if (other)
			{
				const ndMeshJoint* const self = *m_joint;
				const ndMeshJoint* const otherSelf = *other->m_joint;
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
		ndAssert(m_editor->m_currentSelection);
		m_editor->m_currentSelection->SetJoint(m_joint);
	}
	ndSharedPtr<ndMeshJoint> m_joint;
};

void ndAssetEditor::ShowJointGlobalMatrix()
{
	ndSharedPtr<ndMeshJoint> joint(m_currentSelection->GetJoint());

	if (m_showPreTransform)
	{
		ndAssert(0);
	}
	else
	{
		ImGui::SeparatorText("global frame");
		ndReal position[3];
		position[0] = ndReal(0.0f);
		position[1] = ndReal(0.0f);
		position[2] = ndReal(0.0f);
		if (ImGui::InputFloat3("rel position##2", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
			ndMatrix localFrame0(joint->m_localFrame0);
			const ndVector delta(position[0], position[1], position[2], ndFloat32(0.0f));
			localFrame0.m_posit += localFrame0.RotateVector(delta);

			ndMatrix globalMatrix(localFrame0 * m_currentSelection->CalculateGlobalMatrix());
			ndMatrix localFrame1(globalMatrix * m_currentSelection->GetParent()->CalculateGlobalMatrix().OrthoInverse());

			joint->m_localFrame0 = localFrame0;
			joint->m_localFrame1 = localFrame1;

			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		}

		ndReal euler[3];
		euler[0] = ndReal(0.0f);
		euler[1] = ndReal(0.0f);
		euler[2] = ndReal(0.0f);
		if (ImGui::InputFloat3("rel rotation##2", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
			ndMatrix localMatrix0(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * joint->m_localFrame0);

			ndMatrix globalMatrix(localMatrix0 * m_currentSelection->CalculateGlobalMatrix());
			ndMatrix localMatrix1(globalMatrix * m_currentSelection->GetParent()->CalculateGlobalMatrix().OrthoInverse());

			joint->m_localFrame0 = localMatrix0;
			joint->m_localFrame1 = localMatrix1;

			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		}
	}
}

void ndAssetEditor::ShowPropertiesJointInfo()
{
	if (ImGui::CollapsingHeader("Constraint joint"))
	{
		ndSharedPtr<ndMeshJoint> joint (m_currentSelection->GetJoint());
		if (ImGui::BeginCombo("joint type", joint->m_constructor.GetStr()))
		{
			auto SetDropdownList = [this, &joint](const char* const name)
			{
				bool selected = strcmp(name, joint->m_constructor.GetStr()) ? false : true;
				if (ImGui::Selectable(name, selected))
				{
					auto InitNewGlobalJoint = [this, &joint](ndSharedPtr<ndJointBilateralConstraint>& newJoint)
					{
						const ndMatrix localFrame0(joint->m_localFrame0);
						const ndMatrix globalMatrix(localFrame0 * m_currentSelection->CalculateGlobalMatrix());
						const ndMatrix localFrame1(globalMatrix * m_currentSelection->GetParent()->CalculateGlobalMatrix().OrthoInverse());

						newJoint->SetLocalMatrix0(localFrame0);
						newJoint->SetLocalMatrix1(localFrame1);

						m_currentSelection->SetJoint(ndSharedPtr<ndMeshJoint>(newJoint->GetMeshJoint(*joint->m_owner)));
						joint = m_currentSelection->GetJoint();
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
					};

					if (strcmp(name, ndJointFix6dof::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointFix6dof());
						InitNewGlobalJoint(newJoint);
					}
					else if (strcmp(name, ndJointHinge::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointHinge());
						InitNewGlobalJoint(newJoint);
					}
					else if (strcmp(name, ndJointSlider::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointSlider());
						InitNewGlobalJoint(newJoint);
					}
					else if (strcmp(name, ndJointPlane::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointPlane());
						InitNewGlobalJoint(newJoint);
					}

					else
					{
						ndAssert(0);
					}
				}
			};
			SetDropdownList(ndJointFix6dof::StaticClassName());
			SetDropdownList(ndJointHinge::StaticClassName());
			SetDropdownList(ndJointSlider::StaticClassName());
			SetDropdownList(ndJointPlane::StaticClassName());
			SetDropdownList(ndJointRoller::StaticClassName());
			SetDropdownList(ndJointCylinder::StaticClassName());
			SetDropdownList(ndJointDoubleHinge::StaticClassName());
			SetDropdownList(ndJointWheel::StaticClassName());
			SetDropdownList(ndJointSpherical::StaticClassName());
			

			ImGui::EndCombo();
		}

		if (strcmp(joint->m_constructor.GetStr(), ndJointFix6dof::StaticClassName()) == 0)
		{
			JointsEditFix6dof();

		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointHinge::StaticClassName()) == 0)
		{
			JointsEditHingeJoint();
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointSlider::StaticClassName()) == 0)
		{
			JointsEditSliderJoint();
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointPlane::StaticClassName()) == 0)
		{
			JointsEditPlaneJoint();
		}

		else
		{
			ndAssert(0);
		}
	}
}

void ndAssetEditor::JointsEditFix6dof()
{
	ShowLoopJointGlobalMatrix();

	ndMeshJointFix6dof* const joint = (ndMeshJointFix6dof*)*m_currentSelection->GetJoint();

	ndReal value = joint->m_softness;
	if (ImGui::InputFloat("softness", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		joint->m_softness = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
	}

	value = joint->m_maxForce;
	if (ImGui::InputFloat("max force", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		joint->m_maxForce = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
	}

	value = joint->m_maxTorque;
	if (ImGui::InputFloat("max torque", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		joint->m_maxTorque = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
	}
}

void ndAssetEditor::JointsEditHingeJoint()
{
	ShowJointGlobalMatrix();

	ndMeshJointHinge* const joint = (ndMeshJointHinge*)*m_currentSelection->GetJoint();

	ImGui::SeparatorText("actuator params");
	ndReal value = joint->m_axis.m_springK;
	if (ImGui::InputFloat("spring const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		joint->m_axis.m_springK = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
	}
	value = joint->m_axis.m_damperC;
	if (ImGui::InputFloat("damper const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		joint->m_axis.m_damperC = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
	}
	value = joint->m_axis.m_springDamperRegularizer;
	if (ImGui::InputFloat("regularizer", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		joint->m_axis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
	}

	char enableLimist[64];
	if (joint->m_axis.m_limitState)
	{
		snprintf(enableLimist, sizeof(enableLimist) - 1, "true");
	}
	else
	{
		snprintf(enableLimist, sizeof(enableLimist) - 1, "false");
	}

	if (ImGui::BeginCombo("limits on##10", enableLimist))
	{
		if (ImGui::Selectable("true", joint->m_axis.m_limitState))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
			joint->m_axis.m_limitState = true;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		}
		if (ImGui::Selectable("false", !joint->m_axis.m_limitState))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
			joint->m_axis.m_limitState = false;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		}
		ImGui::EndCombo();
	}

	if (joint->m_axis.m_limitState)
	{
		value = joint->m_axis.m_minLimit;
		if (ImGui::InputFloat("min limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
			joint->m_axis.m_minLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		}
		value = joint->m_axis.m_maxLimit;
		if (ImGui::InputFloat("max limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
			joint->m_axis.m_maxLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		}
	}
}

void ndAssetEditor::JointsEditSliderJoint()
{
	ShowJointGlobalMatrix();

	ndMeshJointSlider* const joint = (ndMeshJointSlider*)*m_currentSelection->GetJoint();

	ImGui::SeparatorText("actuator params");
	ndReal value = joint->m_axis.m_springK;
	if (ImGui::InputFloat("spring const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		joint->m_axis.m_springK = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
	}
	value = joint->m_axis.m_damperC;
	if (ImGui::InputFloat("damper const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		joint->m_axis.m_damperC = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
	}
	value = joint->m_axis.m_springDamperRegularizer;
	if (ImGui::InputFloat("regularizer", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		joint->m_axis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
	}

	char enableLimist[64];
	if (joint->m_axis.m_limitState)
	{
		snprintf(enableLimist, sizeof(enableLimist) - 1, "true");
	}
	else
	{
		snprintf(enableLimist, sizeof(enableLimist) - 1, "false");
	}

	if (ImGui::BeginCombo("limits on##10", enableLimist))
	{
		if (ImGui::Selectable("true", joint->m_axis.m_limitState))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
			joint->m_axis.m_limitState = true;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		}
		if (ImGui::Selectable("false", !joint->m_axis.m_limitState))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
			joint->m_axis.m_limitState = false;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		}
		ImGui::EndCombo();
	}

	if (joint->m_axis.m_limitState)
	{
		value = joint->m_axis.m_minLimit;
		if (ImGui::InputFloat("min limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
			joint->m_axis.m_minLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		}
		value = joint->m_axis.m_maxLimit;
		if (ImGui::InputFloat("max limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
			joint->m_axis.m_maxLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		}
	}
}

void ndAssetEditor::JointsEditPlaneJoint()
{
	ShowLoopJointGlobalMatrix();

	ndMeshJointPlane* const joint = (ndMeshJointPlane*)*m_currentSelection->GetJoint();

	char enableLimist[64];
	if (joint->m_controlRotation)
	{
		snprintf(enableLimist, sizeof(enableLimist) - 1, "true");
	}
	else
	{
		snprintf(enableLimist, sizeof(enableLimist) - 1, "false");
	}

	if (ImGui::BeginCombo("lock rotation##10", enableLimist))
	{
		if (ImGui::Selectable("true", joint->m_controlRotation))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
			joint->m_controlRotation = true;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		}
		if (ImGui::Selectable("false", !joint->m_controlRotation))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
			joint->m_controlRotation = false;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoStructuralJoint(this, m_currentSelection)));
		}
		ImGui::EndCombo();
	}
}