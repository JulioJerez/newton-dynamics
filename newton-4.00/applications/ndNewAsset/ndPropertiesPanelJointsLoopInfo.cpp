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

class ndUndoRedoLoopJoint : public ndUndoRedoCommand
{
	public:
	ndUndoRedoLoopJoint(ndAssetEditor* const editor)
		:ndUndoRedoCommand(editor, ndSharedPtr<ndMesh>(nullptr))
		,m_loopJoint(editor->m_currentLoopJointSelection)
		,m_name(editor->m_currentLoopJointSelection->m_name)
		,m_joint(editor->m_currentLoopJointSelection->m_joint)
	{
	}

	virtual class ndUndoRedoLoopJoint* GetAsUndoRedoLoopJoint() const override
	{
		return (ndUndoRedoLoopJoint*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoLoopJoint* const other = command.GetAsUndoRedoLoopJoint();
			if (other)
			{
				bool test = m_name == other->m_name;
				test = test && (m_joint->m_constructor == other->m_joint->m_constructor);
				test = test && (m_joint->m_localFrame0 * other->m_joint->m_localFrame0.OrthoInverse()).TestIdentity();
				test = test && (m_joint->m_localFrame1 * other->m_joint->m_localFrame1.OrthoInverse()).TestIdentity();
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
		ndAssert(m_editor->m_currentLoopJointSelection);
		m_editor->m_currentLoopJointSelection = m_loopJoint;
		ndMeshLoopJoint* const loopJoint = *m_loopJoint;

		loopJoint->m_name = m_name;
		loopJoint->m_joint = m_joint;
	}

	ndSharedPtr<ndMeshLoopJoint> m_loopJoint;
	ndString m_name;
	ndSharedPtr<ndMeshJoint> m_joint;
};

void ndAssetEditor::ShowPropertiesJointsLoopInfo()
{
	if (ImGui::CollapsingHeader("Loop joint"))
	{
		char nodeName[256];
		snprintf(nodeName, sizeof(nodeName) - 1, "%s", m_currentLoopJointSelection->m_name.GetStr());
		if (ImGui::InputText("Name", nodeName, sizeof(nodeName) - 1, ImGuiInputTextFlags_EnterReturnsTrue))
		{
			if (strcmp(m_currentLoopJointSelection->m_name.GetStr(), nodeName))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				ndString newName(nodeName);
				while (m_mesh->FindByName(newName))
				{
					newName += "_1";
				}
				m_currentLoopJointSelection->m_name = newName;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
		}

		ndSharedPtr<ndMeshJoint> joint(m_currentLoopJointSelection->m_joint);
		if (ImGui::BeginCombo("joint type", joint->m_constructor.GetStr()))
		{
			auto SetDropdownList = [this, &joint](const char* const name)
			{
				bool selected = strcmp(name, joint->m_constructor.GetStr()) ? false : true;
				if (ImGui::Selectable(name, selected))
				{
					auto InitNewJoint = [this, &joint](ndSharedPtr<ndJointBilateralConstraint>& newJoint)
					{
						newJoint->SetLocalMatrix0(joint->m_localFrame0);
						newJoint->SetLocalMatrix1(joint->m_localFrame1);
						m_currentLoopJointSelection->m_joint = newJoint->GetMeshJoint(*joint->m_owner);
						joint = m_currentLoopJointSelection->m_joint;
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
					};
					if (strcmp(name, ndJointHinge::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointHinge());
						InitNewJoint(newJoint);
					}
					else if (strcmp(name, ndJointSlider::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointSlider());
						InitNewJoint(newJoint);
					}
					else if (strcmp(name, ndJointPlane::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointPlane());
						InitNewJoint(newJoint);
					}
					else if (strcmp(name, ndJointDoubleHinge::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointDoubleHinge());
						InitNewJoint(newJoint);
					}
					else if (strcmp(name, ndJointSpherical::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointSpherical());
						InitNewJoint(newJoint);
					}
					else if (strcmp(name, ndJointFix6dof::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointFix6dof());
						InitNewJoint(newJoint);
					}
					else if (strcmp(name, ndJointRoller::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointRoller());
						InitNewJoint(newJoint);
					}
					else if (strcmp(name, ndJointCylinder::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointCylinder());
						InitNewJoint(newJoint);
					}
					else if (strcmp(name, ndJointWheel::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointWheel());
						InitNewJoint(newJoint);
					}
					else
					{
						ndAssert(0);
					}
				}
			};
			SetDropdownList(ndJointHinge::StaticClassName());
			SetDropdownList(ndJointSlider::StaticClassName());
			SetDropdownList(ndJointPlane::StaticClassName());
			SetDropdownList(ndJointRoller::StaticClassName());
			SetDropdownList(ndJointCylinder::StaticClassName());
			SetDropdownList(ndJointDoubleHinge::StaticClassName());
			SetDropdownList(ndJointWheel::StaticClassName());
			SetDropdownList(ndJointSpherical::StaticClassName());
			SetDropdownList(ndJointFix6dof::StaticClassName());
			SetDropdownList(ndJointGear::StaticClassName());
			SetDropdownList(ndIkSwivelPositionEffector::StaticClassName());
			SetDropdownList(ndMultiBodyVehicleDifferentialAxle::StaticClassName());

			ImGui::EndCombo();
		}

		ImGui::SeparatorText("");
		if (ImGui::Button("pickBody0"))
		{
			ndTrace(("select body 0\n"));
		}
		ImGui::Text(m_currentLoopJointSelection->m_childNode->GetName().GetStr());

		ImGui::SeparatorText("");
		if (ImGui::Button("pickBody1"))
		{
			ndTrace(("select body 1\n"));
		}
		ImGui::Text(m_currentLoopJointSelection->m_parentNode->GetName().GetStr());

		if (strcmp(joint->m_constructor.GetStr(), ndIkSwivelPositionEffector::StaticClassName()) == 0)
		{
			JointsLoopEditSwivelPositionEffector();
		}
		else
		{
			ndAssert(0);
		}
	}
}

void ndAssetEditor::JointsLoopEditSwivelPositionEffector()
{
	ndMeshJointIkSwivelPositionEffector* const joint = (ndMeshJointIkSwivelPositionEffector*)*m_currentLoopJointSelection->m_joint;
	
	ImGui::SeparatorText("child local Frame");
	{
		ndReal position[3];
		position[0] = ndReal(0.0f);
		position[1] = ndReal(0.0f);
		position[2] = ndReal(0.0f);
		if (ImGui::InputFloat3("rel position##0", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			ndMatrix localFrame0(joint->m_localFrame0);
			const ndVector delta(position[0], position[1], position[2], ndFloat32(0.0f));
			localFrame0.m_posit += localFrame0.RotateVector(delta);

			//ndMatrix globalMatrix(localFrame0 * m_currentLoopJointSelection->m_childNode->CalculateGlobalMatrix());
			//ndMatrix localFrame1(globalMatrix * m_currentLoopJointSelection->m_childNode->GetParent()->CalculateGlobalMatrix().OrthoInverse());
			joint->m_localFrame0 = localFrame0;
			//joint->m_localFrame1 = localFrame1;

			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}

		ndReal euler[3];
		euler[0] = ndReal(0.0f);
		euler[1] = ndReal(0.0f);
		euler[2] = ndReal(0.0f);
		if (ImGui::InputFloat3("rel rotation##0", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			ndMatrix localMatrix0(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * joint->m_localFrame0);
			//ndMatrix globalMatrix(localMatrix0 * m_currentLoopJointSelection->m_childNode->CalculateGlobalMatrix());
			//ndMatrix localMatrix1(globalMatrix * m_currentLoopJointSelection->m_childNode->GetParent()->CalculateGlobalMatrix().OrthoInverse());
			//localMatrix0.m_posit = joint->m_localFrame0.m_posit;

			joint->m_localFrame0 = localMatrix0;
			//joint->m_localFrame1 = localMatrix1;

			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
	}

	ImGui::SeparatorText("parent local Frame");
	{
		ndReal position[3];
		position[0] = ndReal(0.0f);
		position[1] = ndReal(0.0f);
		position[2] = ndReal(0.0f);
		if (ImGui::InputFloat3("rel position##1", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			ndMatrix localFrame1(joint->m_localFrame1);
			const ndVector delta(position[0], position[1], position[2], ndFloat32(0.0f));
			localFrame1.m_posit += localFrame1.RotateVector(delta);

			//ndMatrix globalMatrix(localFrame0 * m_currentLoopJointSelection->m_childNode->CalculateGlobalMatrix());
			//ndMatrix localFrame1(globalMatrix * m_currentLoopJointSelection->m_childNode->GetParent()->CalculateGlobalMatrix().OrthoInverse());
			joint->m_localFrame1 = localFrame1;
			//joint->m_localFrame1 = localFrame1;

			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}

		ndReal euler[3];
		euler[0] = ndReal(0.0f);
		euler[1] = ndReal(0.0f);
		euler[2] = ndReal(0.0f);
		if (ImGui::InputFloat3("rel rotation##1", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			//ndMatrix localMatrix0(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * joint->m_localFrame0);
			//ndMatrix globalMatrix(localMatrix0 * m_currentLoopJointSelection->m_parentNode->CalculateGlobalMatrix());
			//ndMatrix localMatrix1(globalMatrix * m_currentLoopJointSelection->m_parentNode->GetParent()->CalculateGlobalMatrix().OrthoInverse());
			//localMatrix0.m_posit = joint->m_localFrame0.m_posit;
			//
			//joint->m_localFrame0 = localMatrix0;
			//joint->m_localFrame1 = localMatrix1;

			ndMatrix localMatrix1(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * joint->m_localFrame1);
			joint->m_localFrame1 = localMatrix1;

			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
	}

	ImGui::SeparatorText("linear actuator");
	{
		ndReal value = joint->m_linearSpring;
		if (ImGui::InputFloat("spring const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_linearSpring = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_linearDamper;
		if (ImGui::InputFloat("damper const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_linearDamper = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_linearRegularizer;
		if (ImGui::InputFloat("regularizer", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_linearRegularizer = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_linearMaxForce;
		if (ImGui::InputFloat("max force", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_linearMaxForce = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		//value = joint->m_axis0.m_maxLimit;
		//if (ImGui::InputFloat("max limit##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		//{
		//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		//	joint->m_axis0.m_maxLimit = ndMax(value, ndReal(0.0f));
		//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		//}
		//bool limitState = joint->m_axis0.m_limitState ? true : false;
		//if (ImGui::Checkbox("limit State##5", &limitState))
		//{
		//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		//	joint->m_axis0.m_limitState = m_showSelectedNode ? 1 : 0;
		//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		//}
	}

}