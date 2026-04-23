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
		,m_loopJoint(ndSharedPtr<ndMeshLoopJoint>(new ndMeshLoopJoint(**editor->m_currentLoopJointSelection)))
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
				const ndMeshLoopJoint* const self = *m_loopJoint;
				const ndMeshLoopJoint* const otherSelf = *other->m_loopJoint;
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
		ndAssert(m_editor->m_currentLoopJointSelection);
		ndCloseLoopConstraints* const loopContainer = m_editor->m_mesh->GetLoopJoints();
		ndAssert(loopContainer);
		for (ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* node = loopContainer->m_loopJoints.GetFirst(); node; node = node->GetNext())
		{
			const ndSharedPtr<ndMeshLoopJoint>& loopJoint = node->GetInfo();
			if (loopJoint == m_editor->m_currentLoopJointSelection)
			{
				ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* const newNode = loopContainer->m_loopJoints.Append(m_loopJoint);
				loopContainer->m_loopJoints.InsertAfter(node, newNode);
				loopContainer->m_loopJoints.Remove(node);
				break;
			}
		}
		m_editor->m_currentLoopJointSelection = m_loopJoint;
	}

	ndSharedPtr<ndMeshLoopJoint> m_loopJoint;
};

void ndAssetEditor::ShowLoopJointLocalMatrix()
{
	ndSharedPtr<ndMeshJoint> joint(m_currentLoopJointSelection->m_joint);

	if (m_showPreTransform)
	{
		ndAssert(0);
	}
	else
	{
		ImGui::SeparatorText("child local frame");
		{
			ndReal position[3];
			position[0] = ndReal(0.0f);
			position[1] = ndReal(0.0f);
			position[2] = ndReal(0.0f);
			//snprintf(name, sizeof(name) - 1, "rel position%s", labelTag);
			if (ImGui::InputFloat3("rel position##0", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				ndMatrix localFrame0(joint->m_localFrame0);
				const ndVector delta(position[0], position[1], position[2], ndFloat32(0.0f));
				localFrame0.m_posit += localFrame0.RotateVector(delta);
				joint->m_localFrame0 = localFrame0;
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
				joint->m_localFrame0 = localMatrix0;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
		}

		ImGui::SeparatorText("parent local frame");
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
				joint->m_localFrame1 = localFrame1;

				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}

			ndReal euler[3];
			euler[0] = ndReal(0.0f);
			euler[1] = ndReal(0.0f);
			euler[2] = ndReal(0.0f);
			if (ImGui::InputFloat3("rel rotation##1", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				ndMatrix localMatrix1(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * joint->m_localFrame1);
				joint->m_localFrame1 = localMatrix1;

				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
		}
	}
}

void ndAssetEditor::ShowLoopJointGlobalMatrix()
{
	ndSharedPtr<ndMeshJoint> joint(m_currentLoopJointSelection->m_joint);

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
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			ndMatrix localFrame0(joint->m_localFrame0);
			const ndVector delta(position[0], position[1], position[2], ndFloat32(0.0f));
			localFrame0.m_posit += localFrame0.RotateVector(delta);

			ndMatrix globalMatrix(localFrame0 * m_currentLoopJointSelection->m_childNode->CalculateGlobalMatrix());
			ndMatrix localFrame1(globalMatrix * m_currentLoopJointSelection->m_parentNode->CalculateGlobalMatrix().OrthoInverse());

			joint->m_localFrame0 = localFrame0;
			joint->m_localFrame1 = localFrame1;

			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}

		ndReal euler[3];
		euler[0] = ndReal(0.0f);
		euler[1] = ndReal(0.0f);
		euler[2] = ndReal(0.0f);
		if (ImGui::InputFloat3("rel rotation##2", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			ndMatrix localMatrix0(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * joint->m_localFrame0);

			ndMatrix globalMatrix(localMatrix0 * m_currentLoopJointSelection->m_childNode->CalculateGlobalMatrix());
			ndMatrix localMatrix1(globalMatrix * m_currentLoopJointSelection->m_parentNode->CalculateGlobalMatrix().OrthoInverse());

			joint->m_localFrame0 = localMatrix0;
			joint->m_localFrame1 = localMatrix1;

			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
	}
}

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

		if (strcmp(joint->m_constructor.GetStr(), ndJointGear::StaticClassName()) == 0)
		{
			JointsLoopEditGearJoint();
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointHinge::StaticClassName()) == 0)
		{
			JointsLoopEditHingeJoint();
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndMultiBodyVehicleDifferentialAxle::StaticClassName()) == 0)
		{
			JointsLoopEditDifferentialAxle();
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndIkSwivelPositionEffector::StaticClassName()) == 0)
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
	ShowLoopJointLocalMatrix();

	ndMeshJointIkSwivelPositionEffector* const joint = (ndMeshJointIkSwivelPositionEffector*)*m_currentLoopJointSelection->m_joint;
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
	}

	ImGui::SeparatorText("rotation order");
	{
		char rotationOrder[64];
		if (joint->m_rotationOrder == ndIkSwivelPositionEffector::m_pitchRollYaw)
		{
			snprintf(rotationOrder, sizeof(rotationOrder) - 1, "pitchRollYaw");
		}
		else
		{
			snprintf(rotationOrder, sizeof(rotationOrder) - 1, "pitchYawRoll");
		}

		if (ImGui::BeginCombo("##11", rotationOrder))
		{
			bool param = (joint->m_rotationOrder == ndIkSwivelPositionEffector::m_pitchRollYaw);
			if (ImGui::Selectable("pitchRollYaw", param))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_rotationOrder = ndIkSwivelPositionEffector::m_pitchRollYaw;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			if (ImGui::Selectable("pitchYawRoll", !param))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_rotationOrder = ndIkSwivelPositionEffector::m_pitchYawRoll;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			ImGui::EndCombo();
		}
	}

	ImGui::SeparatorText("work space constraint");
	{
		ndReal value = joint->m_minWorkSpaceRadio;
		if (ImGui::InputFloat("min radios", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_minWorkSpaceRadio = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}

		value = joint->m_maxWorkSpaceRadio;
		if (ImGui::InputFloat("max radios", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_maxWorkSpaceRadio = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
	}

	ImGui::SeparatorText("swivel actuator mode");
	{
		char swivelMode[64];
		if (joint->m_enableSwivelControl)
		{
			snprintf(swivelMode, sizeof(swivelMode) - 1, "true");
		}
		else
		{
			snprintf(swivelMode, sizeof(swivelMode) - 1, "false");
		}

		if (ImGui::BeginCombo("##10", swivelMode))
		{
			if (ImGui::Selectable("true", joint->m_enableSwivelControl))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_enableSwivelControl = true;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			if (ImGui::Selectable("false", !joint->m_enableSwivelControl))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_enableSwivelControl = false;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}

			ImGui::EndCombo();
		}

		if (joint->m_enableSwivelControl)
		{
			ndReal value = joint->m_angularSpring;
			if (ImGui::InputFloat("spring const##1", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_angularSpring = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			value = joint->m_angularDamper;
			if (ImGui::InputFloat("damper const##1", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_angularSpring = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			value = joint->m_angularRegularizer;
			if (ImGui::InputFloat("regularizer#1", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_angularRegularizer = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			value = joint->m_angularMaxTorque;
			if (ImGui::InputFloat("max torque", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_angularMaxTorque = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
		}
	}
}

void ndAssetEditor::JointsLoopEditGearJoint()
{
	ShowLoopJointLocalMatrix();

	ndMeshJointGear* const joint = (ndMeshJointGear*)*m_currentLoopJointSelection->m_joint;
	ndReal value = joint->m_ratio;
	if (ImGui::InputFloat("gear ratio", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_ratio = ndMax(value, ndReal(0.01f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
}

void ndAssetEditor::JointsLoopEditDifferentialAxle()
{
	ShowLoopJointLocalMatrix();

	ndMeshJointDifferentialAxle* const joint = (ndMeshJointDifferentialAxle*)*m_currentLoopJointSelection->m_joint;
	ndReal value = joint->m_gearRatio;
	if (ImGui::InputFloat("gear ratio", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_gearRatio = ndMax(value, ndReal(0.01f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
}

void ndAssetEditor::JointsLoopEditHingeJoint()
{
	ShowLoopJointGlobalMatrix();

	ndMeshJointHinge* const joint = (ndMeshJointHinge*)*m_currentLoopJointSelection->m_joint;
	ImGui::SeparatorText("actuator params");
}