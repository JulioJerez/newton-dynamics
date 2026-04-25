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

#if 0
		if (strcmp(joint->m_constructor.GetStr(), ndJointFix6dof::StaticClassName()) == 0)
		{
			ndMeshJointFix6dof* const subJoint = (ndMeshJointFix6dof*)*joint;
			ndReal value = subJoint->m_softness;
			if (ImGui::InputFloat("softness", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointFix6dof(this, m_currentSelection)));
				subJoint->m_softness = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointFix6dof(this, m_currentSelection)));
			}
			value = subJoint->m_maxForce;
			if (ImGui::InputFloat("max Force", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointFix6dof(this, m_currentSelection)));
				subJoint->m_maxForce = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointFix6dof(this, m_currentSelection)));
			}
			value = subJoint->m_maxTorque;
			if (ImGui::InputFloat("max_torque", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointFix6dof(this, m_currentSelection)));
				subJoint->m_maxTorque = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointFix6dof(this, m_currentSelection)));
			}
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointHinge::StaticClassName()) == 0)
		{
			ndMeshJointHinge* const subJoint = (ndMeshJointHinge*)*joint;
			ndReal value = subJoint->m_axis.m_springK;
			if (ImGui::InputFloat("spring const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_springK = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_damperC;
			if (ImGui::InputFloat("damper const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_damperC = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_springDamperRegularizer;
			if (ImGui::InputFloat("regularizer", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_minLimit;
			if (ImGui::InputFloat("min limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_minLimit = ndMin(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_maxLimit;
			if (ImGui::InputFloat("max limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_maxLimit = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			bool limitState = subJoint->m_axis.m_limitState ? true : false;
			if (ImGui::Checkbox("limit State", &limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_limitState = m_showSelectedNode ? 1 : 0;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointSlider::StaticClassName()) == 0)
		{
			ndMeshJointSlider* const subJoint = (ndMeshJointSlider*)*joint;
			ndReal value = subJoint->m_axis.m_springK;
			if (ImGui::InputFloat("spring const##1", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_springK = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_damperC;
			if (ImGui::InputFloat("damper const##1", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_damperC = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_springDamperRegularizer;
			if (ImGui::InputFloat("regularizer##1", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_minLimit;
			if (ImGui::InputFloat("min limit##1", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_minLimit = ndMin(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_maxLimit;
			if (ImGui::InputFloat("max limit##1", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_maxLimit = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			bool limitState = subJoint->m_axis.m_limitState ? true : false;
			if (ImGui::Checkbox("limit State##1", &limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_limitState = m_showSelectedNode ? 1 : 0;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointDoubleHinge::StaticClassName()) == 0)
		{
			ndMeshJointDoubleHinge* const subJoint = (ndMeshJointDoubleHinge*)*joint;
			ImGui::SeparatorText("child pin");
			{
				ndReal value = subJoint->m_axis0.m_springK;
				if (ImGui::InputFloat("spring const##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis0.m_springK = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				value = subJoint->m_axis0.m_damperC;
				if (ImGui::InputFloat("damper const##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis0.m_damperC = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				value = subJoint->m_axis0.m_springDamperRegularizer;
				if (ImGui::InputFloat("regularizer##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis0.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				value = subJoint->m_axis0.m_minLimit;
				if (ImGui::InputFloat("min limit##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis0.m_minLimit = ndMin(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				value = subJoint->m_axis0.m_maxLimit;
				if (ImGui::InputFloat("max limit##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis0.m_maxLimit = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				bool limitState = subJoint->m_axis0.m_limitState ? true : false;
				if (ImGui::Checkbox("limit State##2", &limitState))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis0.m_limitState = m_showSelectedNode ? 1 : 0;
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
			}
			ImGui::SeparatorText("parent pin");
			{
				ndReal value = subJoint->m_axis1.m_springK;
				if (ImGui::InputFloat("spring const##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis1.m_springK = ndMax (value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				value = subJoint->m_axis1.m_damperC;
				if (ImGui::InputFloat("damper const##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis1.m_damperC = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				value = subJoint->m_axis1.m_springDamperRegularizer;
				if (ImGui::InputFloat("regularizer##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis1.m_springDamperRegularizer = ndMax (value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				value = subJoint->m_axis1.m_minLimit;
				if (ImGui::InputFloat("min limit##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis1.m_minLimit = ndMin(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				value = subJoint->m_axis1.m_maxLimit;
				if (ImGui::InputFloat("max limit##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis1.m_maxLimit = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				bool limitState = subJoint->m_axis1.m_limitState ? true : false;
				if (ImGui::Checkbox("limit State##3", &limitState))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis1.m_limitState = m_showSelectedNode ? 1 : 0;
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
			}
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointRoller::StaticClassName()) == 0)
		{
			ndAssert(0);
			//ndMeshJointRoller* const subJoint = (ndMeshJointRoller*)*joint;
			//ImGui::SeparatorText("child pin");
			//{
			//	ndReal value = subJoint->m_positAxis.m_springK;
			//	if (ImGui::InputFloat("spring const##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//	{
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//		subJoint->m_positAxis.m_springK = ndMax(value, ndReal(0.0f));
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//	}
			//	value = subJoint->m_positAxis.m_damperC;
			//	if (ImGui::InputFloat("damper const##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//	{
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//		subJoint->m_positAxis.m_damperC = ndMax(value, ndReal(0.0f));
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//	}
			//	value = subJoint->m_positAxis.m_springDamperRegularizer;
			//	if (ImGui::InputFloat("regularizer##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//	{
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//		subJoint->m_positAxis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//	}
			//	value = subJoint->m_positAxis.m_minLimit;
			//	if (ImGui::InputFloat("min limit##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//	{
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//		subJoint->m_positAxis.m_minLimit = ndMin(value, ndReal(0.0f));
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//	}
			//	value = subJoint->m_positAxis.m_maxLimit;
			//	if (ImGui::InputFloat("max limit##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//	{
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//		subJoint->m_positAxis.m_maxLimit = ndMax(value, ndReal(0.0f));
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//	}
			//	bool limitState = subJoint->m_positAxis.m_limitState ? true : false;
			//	if (ImGui::Checkbox("limit State##3", &limitState))
			//	{
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//		subJoint->m_positAxis.m_limitState = m_showSelectedNode ? 1 : 0;
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//	}
			//}
			//ImGui::SeparatorText("parent pin");
			//{
			//	ndReal value = subJoint->m_angleAxis.m_springK;
			//	if (ImGui::InputFloat("spring const##4", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//	{
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//		subJoint->m_angleAxis.m_springK = ndMax(value, ndReal(0.0f));
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//	}
			//	value = subJoint->m_angleAxis.m_damperC;
			//	if (ImGui::InputFloat("damper const##4", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//	{
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//		subJoint->m_angleAxis.m_damperC = ndMax(value, ndReal(0.0f));
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//	}
			//	value = subJoint->m_angleAxis.m_springDamperRegularizer;
			//	if (ImGui::InputFloat("regularizer##4", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//	{
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//		subJoint->m_angleAxis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//	}
			//	value = subJoint->m_angleAxis.m_minLimit;
			//	if (ImGui::InputFloat("min limit##4", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//	{
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//		subJoint->m_angleAxis.m_minLimit = ndMin(value, ndReal(0.0f));
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//	}
			//	value = subJoint->m_angleAxis.m_maxLimit;
			//	if (ImGui::InputFloat("max limit##4", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//	{
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//		subJoint->m_angleAxis.m_maxLimit = ndMax(value, ndReal(0.0f));
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//	}
			//	bool limitState = subJoint->m_angleAxis.m_limitState ? true : false;
			//	if (ImGui::Checkbox("limit State##4", &limitState))
			//	{
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//		subJoint->m_angleAxis.m_limitState = m_showSelectedNode ? 1 : 0;
			//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
			//	}
			//}
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointCylinder::StaticClassName()) == 0)
		{
			ndAssert(0);
			//ndMeshJointCylinder* const subJoint = (ndMeshJointCylinder*)*joint;
			ImGui::SeparatorText("child pin");
			{
				ndAssert(0);
				//ndReal value = subJoint->m_axis0.m_springK;
				//if (ImGui::InputFloat("spring const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				//{
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//	subJoint->m_axis0.m_springK = ndMax(value, ndReal(0.0f));
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//}
				//value = subJoint->m_axis0.m_damperC;
				//if (ImGui::InputFloat("damper const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				//{
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//	subJoint->m_axis0.m_damperC = ndMax(value, ndReal(0.0f));
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//}
				//value = subJoint->m_axis0.m_springDamperRegularizer;
				//if (ImGui::InputFloat("regularizer##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				//{
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//	subJoint->m_axis0.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//}
				//value = subJoint->m_axis0.m_minLimit;
				//if (ImGui::InputFloat("min limit##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				//{
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//	subJoint->m_axis0.m_minLimit = ndMin(value, ndReal(0.0f));
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//}
				//value = subJoint->m_axis0.m_maxLimit;
				//if (ImGui::InputFloat("max limit##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				//{
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//	subJoint->m_axis0.m_maxLimit = ndMax(value, ndReal(0.0f));
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//}
				//bool limitState = subJoint->m_axis0.m_limitState ? true : false;
				//if (ImGui::Checkbox("limit State##5", &limitState))
				//{
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//	subJoint->m_axis0.m_limitState = m_showSelectedNode ? 1 : 0;
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//}
			}
			ImGui::SeparatorText("parent pin");
			{
				ndAssert(0);
				//ndReal value = subJoint->m_axis1.m_springK;
				//if (ImGui::InputFloat("spring const##6", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				//{
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//	subJoint->m_axis1.m_springK = ndMax(value, ndReal(0.0f));
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//}
				//value = subJoint->m_axis1.m_damperC;
				//if (ImGui::InputFloat("damper const##6", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				//{
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//	subJoint->m_axis1.m_damperC = ndMax(value, ndReal(0.0f));
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//}
				//value = subJoint->m_axis1.m_springDamperRegularizer;
				//if (ImGui::InputFloat("regularizer##6", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				//{
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//	subJoint->m_axis1.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//}
				//value = subJoint->m_axis1.m_minLimit;
				//if (ImGui::InputFloat("min limit##6", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				//{
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//	subJoint->m_axis1.m_minLimit = ndMin(value, ndReal(0.0f));
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//}
				//value = subJoint->m_axis1.m_maxLimit;
				//if (ImGui::InputFloat("max limit##6", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				//{
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//	subJoint->m_axis1.m_maxLimit = ndMax(value, ndReal(0.0f));
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//}
				//bool limitState = subJoint->m_axis1.m_limitState ? true : false;
				//if (ImGui::Checkbox("limit State##6", &limitState))
				//{
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//	subJoint->m_axis1.m_limitState = m_showSelectedNode ? 1 : 0;
				//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				//}
			}
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointWheel::StaticClassName()) == 0)
		{
			ndAssert(0);
			//ndMeshJointWheel* const subJoint = (ndMeshJointWheel*)*joint;
			//ImGui::SeparatorText("baseFrame");
			//
			//ndReal value = subJoint->m_axis.m_springK;
			//if (ImGui::InputFloat("suspension spring", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//{
			//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			//	subJoint->m_axis.m_springK = ndMax(value, ndReal(0.0f));
			//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			//}
			//value = subJoint->m_axis.m_damperC;
			//if (ImGui::InputFloat("suspension const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//{
			//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			//	subJoint->m_axis.m_damperC = ndMax(value, ndReal(0.0f));
			//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			//}
			//value = subJoint->m_axis.m_springDamperRegularizer;
			//if (ImGui::InputFloat("suspension regularizer", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//{
			//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			//	subJoint->m_axis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
			//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			//}
			//value = subJoint->m_axis.m_maxLimit;
			//if (ImGui::InputFloat("lower stop", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//{
			//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			//	subJoint->m_axis.m_maxLimit = ndMax(value, ndReal(0.0f));
			//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			//}
			//value = subJoint->m_axis.m_minLimit;
			//if (ImGui::InputFloat("upper stop", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//{
			//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			//	subJoint->m_axis.m_minLimit = ndMin(value, ndReal(0.0f));
			//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			//}
			//value = subJoint->m_steeringAngle;
			//if (ImGui::InputFloat("steering angle", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//{
			//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			//	subJoint->m_steeringAngle = ndMax(value, ndReal(0.0f));
			//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			//}
			//
			//value = subJoint->m_brakeTorque;
			//if (ImGui::InputFloat("brake torque", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//{
			//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			//	subJoint->m_brakeTorque = ndMax(value, ndReal(0.0f));
			//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			//}
			//value = subJoint->m_handBrakeTorque;
			//if (ImGui::InputFloat("hand brake torque", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			//{
			//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			//	subJoint->m_handBrakeTorque = ndMax(value, ndReal(0.0f));
			//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			//}
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointSpherical::StaticClassName()) == 0)
		{
			ndMeshJointSpherical* const subJoint = (ndMeshJointSpherical*)*joint;
			ndReal value = subJoint->m_axis.m_springK;
			if (ImGui::InputFloat("spring const##8", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
				subJoint->m_axis.m_springK = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_damperC;
			if (ImGui::InputFloat("damper const##8", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
				subJoint->m_axis.m_damperC = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_springDamperRegularizer;
			if (ImGui::InputFloat("regularizer##8", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
				subJoint->m_axis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
			}
			value = subJoint->m_maxConeAngle;
			if (ImGui::InputFloat("max cone angle", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
				subJoint->m_maxConeAngle = ndClamp (value, ndReal(0.0), ndReal(180.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
			}
			bool limitState = subJoint->m_coneAngleState ? true : false;;
			if (ImGui::Checkbox("cone limit state", &limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
				subJoint->m_coneAngleState = limitState;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
			}

			value = subJoint->m_axis.m_minLimit;
			if (ImGui::InputFloat("min twist", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
				subJoint->m_axis.m_minLimit = ndMin(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_maxLimit;
			if (ImGui::InputFloat("max twist", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
				subJoint->m_axis.m_maxLimit = ndMax (value, ndReal (0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
			}
			limitState = subJoint->m_axis.m_limitState ? true : false;
			if (ImGui::Checkbox("twist limit state", &limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
				subJoint->m_axis.m_limitState = m_showSelectedNode ? 1 : 0;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
			}
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointPlane::StaticClassName()) == 0)
		{
			ndMeshJointPlane* const subJoint = (ndMeshJointPlane*)*joint;
			bool limitState = subJoint->m_controlRotation ? true : false;
			if (ImGui::Checkbox("control rotation", &limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointPlane(this, m_currentSelection)));
				subJoint->m_controlRotation = limitState ? 1 : 0;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointPlane(this, m_currentSelection)));
			}
		}
		else
		{
			ndAssert(0);
		}

		// child local frame
		{
			ImGui::SeparatorText("child local Frame");

			if (m_showPreTransform)
			{
				const ndMatrix matrix(joint->m_localFrame0);
				ndReal position[3];
				position[0] = ndReal(matrix.m_posit.m_x);
				position[1] = ndReal(matrix.m_posit.m_y);
				position[2] = ndReal(matrix.m_posit.m_z);
				if (ImGui::InputFloat3("position##2", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJoint(this, m_currentSelection)));
					ndMatrix localFrame0(joint->m_localFrame0);
					localFrame0.m_posit.m_x = position[0];
					localFrame0.m_posit.m_y = position[1];
					localFrame0.m_posit.m_z = position[2];

					ndMatrix globalMatrix(localFrame0 * m_currentSelection->CalculateGlobalMatrix());
					ndMatrix localFrame1(globalMatrix * m_currentSelection->GetParent()->CalculateGlobalMatrix().OrthoInverse());

					joint->m_localFrame0 = localFrame0;
					joint->m_localFrame1 = localFrame1;

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJoint(this, m_currentSelection)));
				}

				ndReal euler[3];
				ndVector tmp;
				ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));
				euler[0] = ndReal(radians[0]);
				euler[1] = ndReal(radians[1]);
				euler[2] = ndReal(radians[2]);

				if (ImGui::InputFloat3("rotation##2", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJoint(this, m_currentSelection)));
					ndMatrix localMatrix0(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
					localMatrix0.m_posit = joint->m_localFrame0.m_posit;
					ndMatrix globalMatrix(localMatrix0 * m_currentSelection->CalculateGlobalMatrix());
					ndMatrix localMatrix1(globalMatrix * m_currentSelection->GetParent()->CalculateGlobalMatrix().OrthoInverse());
					localMatrix0.m_posit = joint->m_localFrame0.m_posit;

					joint->m_localFrame0 = localMatrix0;
					joint->m_localFrame1 = localMatrix1;

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJoint(this, m_currentSelection)));
				}
			}
			else
			{
				ndReal position[3];
				position[0] = ndReal(0.0f);
				position[1] = ndReal(0.0f);
				position[2] = ndReal(0.0f);
				if (ImGui::InputFloat3("rel position##2", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJoint(this, m_currentSelection)));
					ndMatrix localFrame0(joint->m_localFrame0);
					const ndVector delta(position[0], position[1], position[2], ndFloat32(0.0f));
					localFrame0.m_posit += localFrame0.RotateVector(delta);

					ndMatrix globalMatrix(localFrame0 * m_currentSelection->CalculateGlobalMatrix());
					ndMatrix localFrame1(globalMatrix * m_currentSelection->GetParent()->CalculateGlobalMatrix().OrthoInverse());

					joint->m_localFrame0 = localFrame0;
					joint->m_localFrame1 = localFrame1;

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJoint(this, m_currentSelection)));
				}

				ndReal euler[3];
				euler[0] = ndReal(0.0f);
				euler[1] = ndReal(0.0f);
				euler[2] = ndReal(0.0f);

				if (ImGui::InputFloat3("rel rotation##2", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJoint(this, m_currentSelection)));
					ndMatrix localMatrix0(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * joint->m_localFrame0);
					ndMatrix globalMatrix(localMatrix0 * m_currentSelection->CalculateGlobalMatrix());
					ndMatrix localMatrix1(globalMatrix * m_currentSelection->GetParent()->CalculateGlobalMatrix().OrthoInverse());
					localMatrix0.m_posit = joint->m_localFrame0.m_posit;

					joint->m_localFrame0 = localMatrix0;
					joint->m_localFrame1 = localMatrix1;

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJoint(this, m_currentSelection)));
				}
			}
		}
#endif

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