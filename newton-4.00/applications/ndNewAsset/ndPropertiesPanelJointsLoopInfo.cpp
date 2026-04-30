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
		//,m_loopJoint(ndSharedPtr<ndMeshLoopJoint>(new ndMeshLoopJoint(**editor->m_currentSelection->GetAsCloseLoopConstraints())))
	{
		ndAssert(0);
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
		ndAssert(0);
		//ndAssert(m_editor->m_currentLoopJointSelection);
		//ndCloseLoopConstraints* const loopContainer = m_editor->m_mesh->GetLoopJoints();
		//ndAssert(loopContainer);
		//for (ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* node = loopContainer->m_loopJoints.GetFirst(); node; node = node->GetNext())
		//{
		//	const ndSharedPtr<ndMeshLoopJoint>& loopJoint = node->GetInfo();
		//	if (loopJoint == m_editor->m_currentLoopJointSelection)
		//	{
		//		ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* const newNode = loopContainer->m_loopJoints.Append(m_loopJoint);
		//		loopContainer->m_loopJoints.InsertAfter(node, newNode);
		//		loopContainer->m_loopJoints.Remove(node);
		//		break;
		//	}
		//}
		//m_editor->m_currentLoopJointSelection = m_loopJoint;
	}

	ndSharedPtr<ndMeshLoopJoint> m_loopJoint;
};

void ndAssetEditor::EditLoopJointLocalMatrix(ndSharedPtr<ndMeshLoopJoint>& loopJoint)
{
	ndSharedPtr<ndMeshJoint> joint(loopJoint->m_joint);

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

void ndAssetEditor::EditLoopJointGlobalMatrix(ndSharedPtr<ndMeshLoopJoint>& loopJoint)
{
	ndSharedPtr<ndMeshJoint> joint(loopJoint->m_joint);

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

			ndMatrix globalMatrix(localFrame0 * loopJoint->m_childNode->CalculateGlobalMatrix());
			ndMatrix localFrame1(globalMatrix * loopJoint->m_parentNode->CalculateGlobalMatrix().OrthoInverse());

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

			ndMatrix globalMatrix(localMatrix0 * loopJoint->m_childNode->CalculateGlobalMatrix());
			ndMatrix localMatrix1(globalMatrix * loopJoint->m_parentNode->CalculateGlobalMatrix().OrthoInverse());

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
#if 0
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
	}
}
#endif

		ndCloseLoopConstraints* const loops = m_currentSelection->GetAsCloseLoopConstraints();

		auto FindJoint = [this, loops]()
		{
			ndInt32 i = 0;
			for (ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* ptr = loops->m_loopJoints.GetFirst(); ptr; ptr = ptr->GetNext())
			{
				if (i == m_closeLoopIndex)
				{
					return ptr;
					//m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoCollidingPairs(this)));
					//collidingPairs->m_collidingPairs.Remove(ptr);
					//m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoCollidingPairs(this)));
					//break;
				}
				i++;
			}
			return (ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode*)nullptr;
		};

		ndFixSizeArray<const char*, 1024> names;
		for (ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* ptr = loops->m_loopJoints.GetFirst(); ptr; ptr = ptr->GetNext())
		{
			ndSharedPtr<ndMeshLoopJoint>& loopJoint = ptr->GetInfo();
			names.PushBack(loopJoint->m_name.GetStr());
		}
		if (names.GetCount())
		{
			ImGui::ListBox(" ##11", &m_closeLoopIndex, &names[0], names.GetCount(), 12);

			ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* loopJointNode = FindJoint();
			if (loopJointNode)
			{
				ndSharedPtr<ndMeshLoopJoint>& loopJoint = loopJointNode->GetInfo();

				ImGui::NewLine();
				//ImGui::Separator();
				ImGui::Text(loopJoint->m_name.GetStr());

				if (ImGui::Button("remove selected"))
				{
					ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* ptr = FindJoint();
					if (ptr)
					{
						ndAssert(0);
						//m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoCollidingPairs(this)));
						//collidingPairs->m_collidingPairs.Remove(ptr);
						//m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoCollidingPairs(this)));
					}

					m_closeLoopIndex = 0;
				}
			}

			loopJointNode = FindJoint();
			if (loopJointNode)
			{
				ndSharedPtr<ndMeshLoopJoint>& loopJoint = loopJointNode->GetInfo();
				if (ImGui::BeginCombo("joint type", loopJoint->m_joint->m_constructor.GetStr()))
				{
					auto SetDropdownList = [this, &loopJoint](const char* const name)
					{
						bool selected = strcmp(name, loopJoint->m_joint->m_constructor.GetStr()) ? false : true;
						if (ImGui::Selectable(name, selected))
						{
							auto InitNewLocalJoint = [this, &loopJoint](ndSharedPtr<ndJointBilateralConstraint>& newJoint)
							{
								ndMatrix localMatrix0(ndGetIdentityMatrix());
								ndMatrix localMatrix1(ndGetIdentityMatrix());
								localMatrix0.m_posit = loopJoint->m_childNode->GetRigidBody()->m_localCentreOfMass;
								localMatrix1.m_posit = loopJoint->m_parentNode->GetRigidBody()->m_localCentreOfMass;
								newJoint->SetLocalMatrix0(localMatrix0);
								newJoint->SetLocalMatrix1(localMatrix1);
			
								//loopJoint->m_joint = newJoint->GetMeshJoint(*joint->m_owner);
								//joint = m_currentLoopJointSelection->m_joint;
								//m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
							};
							auto InitNewGlobalJoint = [this, &loopJoint](ndSharedPtr<ndJointBilateralConstraint>& newJoint)
							{
								const ndMatrix localFrame0(loopJoint->m_joint->m_localFrame0);
								const ndMatrix globalMatrix(localFrame0 * loopJoint->m_childNode->CalculateGlobalMatrix());
								const ndMatrix localFrame1(globalMatrix * loopJoint->m_parentNode->CalculateGlobalMatrix().OrthoInverse());
			
								newJoint->SetLocalMatrix0(localFrame0);
								newJoint->SetLocalMatrix1(localFrame1);
			
								//loopJoint->m_joint = newJoint->GetMeshJoint(*joint->m_owner);
								//joint = m_currentLoopJointSelection->m_joint;
								//m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
							};
			
							if (strcmp(name, ndJointFix6dof::StaticClassName()) == 0)
							{
								m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
								ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointFix6dof());
								InitNewGlobalJoint(newJoint);
							}
							else if (strcmp(name, ndJointHinge::StaticClassName()) == 0)
							{
								m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
								ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointHinge());
								InitNewGlobalJoint(newJoint);
							}
							else if (strcmp(name, ndJointSlider::StaticClassName()) == 0)
							{
								m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
								ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointSlider());
								InitNewGlobalJoint(newJoint);
							}
							else if (strcmp(name, ndJointPlane::StaticClassName()) == 0)
							{
								m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
								ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointPlane());
								InitNewGlobalJoint(newJoint);
							}
							else if (strcmp(name, ndJointGear::StaticClassName()) == 0)
							{
								m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
								ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointGear());
								InitNewLocalJoint(newJoint);
							}
							else if (strcmp(name, ndMultiBodyVehicleDifferentialAxle::StaticClassName()) == 0)
							{
								m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
								ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndMultiBodyVehicleDifferentialAxle());
								InitNewLocalJoint(newJoint);
							}
							else if (strcmp(name, ndJointDoubleHinge::StaticClassName()) == 0)
							{
								m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
								ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointDoubleHinge());
								InitNewGlobalJoint(newJoint);
							}
							else if (strcmp(name, ndJointRoller::StaticClassName()) == 0)
							{
								m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
								ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointRoller());
								InitNewGlobalJoint(newJoint);
							}
							else if (strcmp(name, ndJointCylinder::StaticClassName()) == 0)
							{
								m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
								ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointCylinder());
								InitNewGlobalJoint(newJoint);
							}
							else if (strcmp(name, ndJointWheel::StaticClassName()) == 0)
							{
								m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
								ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointWheel());
								InitNewGlobalJoint(newJoint);
							}
							else if (strcmp(name, ndIkSwivelPositionEffector::StaticClassName()) == 0)
							{
								m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
								ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndIkSwivelPositionEffector());
								InitNewLocalJoint(newJoint);
							}
							else if (strcmp(name, ndJointSpherical::StaticClassName()) == 0)
							{
								m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
								ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointSpherical());
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
					SetDropdownList(ndJointGear::StaticClassName());
					SetDropdownList(ndIkSwivelPositionEffector::StaticClassName());
					SetDropdownList(ndMultiBodyVehicleDifferentialAxle::StaticClassName());
			
					ImGui::EndCombo();
				}
			
				ndSharedPtr<ndMeshJoint>& joint = loopJoint->m_joint;
				if (strcmp(joint->m_constructor.GetStr(), ndJointFix6dof::StaticClassName()) == 0)
				{
					EditFix6dofLoopJoint(loopJoint);
				}
				else if (strcmp(joint->m_constructor.GetStr(), ndJointHinge::StaticClassName()) == 0)
				{
					EditHingeLoopJoint(loopJoint);
				}
				else if (strcmp(joint->m_constructor.GetStr(), ndJointSlider::StaticClassName()) == 0)
				{
					EditSliderLoopJoint(loopJoint);
				}
				else if (strcmp(joint->m_constructor.GetStr(), ndJointGear::StaticClassName()) == 0)
				{
					EditGearLoopJoint(loopJoint);
				}
				else if (strcmp(joint->m_constructor.GetStr(), ndJointPlane::StaticClassName()) == 0)
				{
					EditPlaneLoopJoint(loopJoint);
				}
				else if (strcmp(joint->m_constructor.GetStr(), ndMultiBodyVehicleDifferentialAxle::StaticClassName()) == 0)
				{
					EditDifferentialAxleLoopJoint(loopJoint);
				}
				else if (strcmp(joint->m_constructor.GetStr(), ndJointRoller::StaticClassName()) == 0)
				{
					EditRollerLoopJoint(loopJoint);
				}
				else if (strcmp(joint->m_constructor.GetStr(), ndJointCylinder::StaticClassName()) == 0)
				{
					EditCylinderLoopJoint(loopJoint);
				}
				else if (strcmp(joint->m_constructor.GetStr(), ndJointDoubleHinge::StaticClassName()) == 0)
				{
					EditDoubleHingeLoopJoint(loopJoint);
				}
				else if (strcmp(joint->m_constructor.GetStr(), ndJointWheel::StaticClassName()) == 0)
				{
					EditWheelLoopJoint(loopJoint);
				}
				else if (strcmp(joint->m_constructor.GetStr(), ndJointSpherical::StaticClassName()) == 0)
				{
					EditSphericalLoopJoint(loopJoint);
				}
				else if (strcmp(joint->m_constructor.GetStr(), ndIkSwivelPositionEffector::StaticClassName()) == 0)
				{
					EditSwivelPositionEffectorLoopJoint(loopJoint);
				}
				else
				{
					ndAssert(0);
				}
			}
		}
	}
}

void ndAssetEditor::EditHingeLoopJoint(ndSharedPtr<ndMeshLoopJoint>& loopJoint)
{
	EditLoopJointGlobalMatrix(loopJoint);

	ndMeshJointHinge* const joint = (ndMeshJointHinge*)*loopJoint->m_joint;

	ImGui::SeparatorText("actuator params");
	ndReal value = joint->m_axis.m_springK;
	if (ImGui::InputFloat("spring const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_axis.m_springK = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
	value = joint->m_axis.m_damperC;
	if (ImGui::InputFloat("damper const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_axis.m_damperC = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
	value = joint->m_axis.m_springDamperRegularizer;
	if (ImGui::InputFloat("regularizer", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_axis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
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
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_axis.m_limitState = true;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		if (ImGui::Selectable("false", !joint->m_axis.m_limitState))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_axis.m_limitState = false;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		ImGui::EndCombo();
	}

	if (joint->m_axis.m_limitState)
	{
		value = joint->m_axis.m_minLimit;
		if (ImGui::InputFloat("min limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_axis.m_minLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_axis.m_maxLimit;
		if (ImGui::InputFloat("max limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_axis.m_maxLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
	}
}

void ndAssetEditor::EditDifferentialAxleLoopJoint(ndSharedPtr<ndMeshLoopJoint>& loopJoint)
{
	EditLoopJointLocalMatrix(loopJoint);

	ndMeshJointDifferentialAxle* const joint = (ndMeshJointDifferentialAxle*)*loopJoint->m_joint;

	ndReal value = joint->m_gearRatio;
	if (ImGui::InputFloat("gear ratio", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_gearRatio = ndMax(value, ndReal(0.01f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
}

void ndAssetEditor::EditSliderLoopJoint(ndSharedPtr<ndMeshLoopJoint>& loopJoint)
{
	EditLoopJointGlobalMatrix(loopJoint);

	ndMeshJointSlider* const joint = (ndMeshJointSlider*)*loopJoint->m_joint;

	ImGui::SeparatorText("actuator params");
	ndReal value = joint->m_axis.m_springK;
	if (ImGui::InputFloat("spring const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_axis.m_springK = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
	value = joint->m_axis.m_damperC;
	if (ImGui::InputFloat("damper const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_axis.m_damperC = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
	value = joint->m_axis.m_springDamperRegularizer;
	if (ImGui::InputFloat("regularizer", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_axis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
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
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_axis.m_limitState = true;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		if (ImGui::Selectable("false", !joint->m_axis.m_limitState))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_axis.m_limitState = false;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		ImGui::EndCombo();
	}

	if (joint->m_axis.m_limitState)
	{
		value = joint->m_axis.m_minLimit;
		if (ImGui::InputFloat("min limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_axis.m_minLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_axis.m_maxLimit;
		if (ImGui::InputFloat("max limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_axis.m_maxLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
	}
}

void ndAssetEditor::EditPlaneLoopJoint(ndSharedPtr<ndMeshLoopJoint>& loopJoint)
{
	EditLoopJointGlobalMatrix(loopJoint);

	ndMeshJointPlane* const joint = (ndMeshJointPlane*)*loopJoint->m_joint;

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
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_controlRotation = true;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		if (ImGui::Selectable("false", !joint->m_controlRotation))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_controlRotation = false;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		ImGui::EndCombo();
	}
}

void ndAssetEditor::EditRollerLoopJoint(ndSharedPtr<ndMeshLoopJoint>& loopJoint)
{
	EditLoopJointGlobalMatrix(loopJoint);

	ndMeshJointRoller* const joint = (ndMeshJointRoller*)*loopJoint->m_joint;

	{
		ImGui::SeparatorText("linear actuator params");
		ndReal value = joint->m_linearAxis.m_springK;
		if (ImGui::InputFloat("spring const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_linearAxis.m_springK = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_linearAxis.m_damperC;
		if (ImGui::InputFloat("damper const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_linearAxis.m_damperC = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_linearAxis.m_springDamperRegularizer;
		if (ImGui::InputFloat("regularizer", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_linearAxis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}

		char enableLimist[64];
		if (joint->m_linearAxis.m_limitState)
		{
			snprintf(enableLimist, sizeof(enableLimist) - 1, "true");
		}
		else
		{
			snprintf(enableLimist, sizeof(enableLimist) - 1, "false");
		}

		if (ImGui::BeginCombo("limits on##10", enableLimist))
		{
			if (ImGui::Selectable("true", joint->m_linearAxis.m_limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_linearAxis.m_limitState = true;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			if (ImGui::Selectable("false", !joint->m_linearAxis.m_limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_linearAxis.m_limitState = false;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			ImGui::EndCombo();
		}

		if (joint->m_linearAxis.m_limitState)
		{
			value = joint->m_linearAxis.m_minLimit;
			if (ImGui::InputFloat("min limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_linearAxis.m_minLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			value = joint->m_linearAxis.m_maxLimit;
			if (ImGui::InputFloat("max limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_linearAxis.m_maxLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
		}
	}

	{
		ImGui::SeparatorText("angular actuator params");
		ndReal value = joint->m_angularAxis.m_springK;
		if (ImGui::InputFloat("spring const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_angularAxis.m_springK = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_angularAxis.m_damperC;
		if (ImGui::InputFloat("damper const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_angularAxis.m_damperC = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_angularAxis.m_springDamperRegularizer;
		if (ImGui::InputFloat("regularizer##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_angularAxis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}

		char enableLimist[64];
		if (joint->m_angularAxis.m_limitState)
		{
			snprintf(enableLimist, sizeof(enableLimist) - 1, "true");
		}
		else
		{
			snprintf(enableLimist, sizeof(enableLimist) - 1, "false");
		}

		if (ImGui::BeginCombo("limits on##5", enableLimist))
		{
			if (ImGui::Selectable("true", joint->m_angularAxis.m_limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_angularAxis.m_limitState = true;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			if (ImGui::Selectable("false", !joint->m_angularAxis.m_limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_angularAxis.m_limitState = false;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			ImGui::EndCombo();
		}

		if (joint->m_angularAxis.m_limitState)
		{
			value = joint->m_angularAxis.m_minLimit;
			if (ImGui::InputFloat("min limit##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_angularAxis.m_minLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			value = joint->m_angularAxis.m_maxLimit;
			if (ImGui::InputFloat("max limit##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_angularAxis.m_maxLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
		}
	}
}

void ndAssetEditor::EditCylinderLoopJoint(ndSharedPtr<ndMeshLoopJoint>& loopJoint)
{
	EditLoopJointGlobalMatrix(loopJoint);

	ndMeshJointCylinder* const joint = (ndMeshJointCylinder*)*loopJoint->m_joint;

	{
		ImGui::SeparatorText("linear actuator params");
		ndReal value = joint->m_linearAxis.m_springK;
		if (ImGui::InputFloat("spring const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_linearAxis.m_springK = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_linearAxis.m_damperC;
		if (ImGui::InputFloat("damper const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_linearAxis.m_damperC = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_linearAxis.m_springDamperRegularizer;
		if (ImGui::InputFloat("regularizer", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_linearAxis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}

		char enableLimist[64];
		if (joint->m_linearAxis.m_limitState)
		{
			snprintf(enableLimist, sizeof(enableLimist) - 1, "true");
		}
		else
		{
			snprintf(enableLimist, sizeof(enableLimist) - 1, "false");
		}

		if (ImGui::BeginCombo("limits on##10", enableLimist))
		{
			if (ImGui::Selectable("true", joint->m_linearAxis.m_limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_linearAxis.m_limitState = true;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			if (ImGui::Selectable("false", !joint->m_linearAxis.m_limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_linearAxis.m_limitState = false;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			ImGui::EndCombo();
		}

		if (joint->m_linearAxis.m_limitState)
		{
			value = joint->m_linearAxis.m_minLimit;
			if (ImGui::InputFloat("min limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_linearAxis.m_minLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			value = joint->m_linearAxis.m_maxLimit;
			if (ImGui::InputFloat("max limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_linearAxis.m_maxLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
		}
	}

	{
		ImGui::SeparatorText("angular actuator params");
		ndReal value = joint->m_angularAxis.m_springK;
		if (ImGui::InputFloat("spring const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_angularAxis.m_springK = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_angularAxis.m_damperC;
		if (ImGui::InputFloat("damper const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_angularAxis.m_damperC = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_angularAxis.m_springDamperRegularizer;
		if (ImGui::InputFloat("regularizer##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_angularAxis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}

		char enableLimist[64];
		if (joint->m_angularAxis.m_limitState)
		{
			snprintf(enableLimist, sizeof(enableLimist) - 1, "true");
		}
		else
		{
			snprintf(enableLimist, sizeof(enableLimist) - 1, "false");
		}

		if (ImGui::BeginCombo("limits on##5", enableLimist))
		{
			if (ImGui::Selectable("true", joint->m_angularAxis.m_limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_angularAxis.m_limitState = true;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			if (ImGui::Selectable("false", !joint->m_angularAxis.m_limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_angularAxis.m_limitState = false;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			ImGui::EndCombo();
		}

		if (joint->m_angularAxis.m_limitState)
		{
			value = joint->m_angularAxis.m_minLimit;
			if (ImGui::InputFloat("min limit##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_angularAxis.m_minLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			value = joint->m_angularAxis.m_maxLimit;
			if (ImGui::InputFloat("max limit##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_angularAxis.m_maxLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
		}
	}
}

void ndAssetEditor::EditDoubleHingeLoopJoint(ndSharedPtr<ndMeshLoopJoint>& loopJoint)
{
	EditLoopJointGlobalMatrix(loopJoint);

	ndMeshJointDoubleHinge* const joint = (ndMeshJointDoubleHinge*)*loopJoint->m_joint;

	{
		ImGui::SeparatorText("actuator0 params");
		ndReal value = joint->m_axis0.m_springK;
		if (ImGui::InputFloat("spring const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_axis0.m_springK = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_axis0.m_damperC;
		if (ImGui::InputFloat("damper const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_axis0.m_damperC = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_axis0.m_springDamperRegularizer;
		if (ImGui::InputFloat("regularizer", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_axis0.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}

		char enableLimist[64];
		if (joint->m_axis0.m_limitState)
		{
			snprintf(enableLimist, sizeof(enableLimist) - 1, "true");
		}
		else
		{
			snprintf(enableLimist, sizeof(enableLimist) - 1, "false");
		}

		if (ImGui::BeginCombo("limits on##10", enableLimist))
		{
			if (ImGui::Selectable("true", joint->m_axis0.m_limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_axis0.m_limitState = true;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			if (ImGui::Selectable("false", !joint->m_axis0.m_limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_axis0.m_limitState = false;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			ImGui::EndCombo();
		}

		if (joint->m_axis0.m_limitState)
		{
			value = joint->m_axis0.m_minLimit;
			if (ImGui::InputFloat("min limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_axis0.m_minLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			value = joint->m_axis0.m_maxLimit;
			if (ImGui::InputFloat("max limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_axis0.m_maxLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
		}
	}

	{
		ImGui::SeparatorText("angular actuator params");
		ndReal value = joint->m_axis1.m_springK;
		if (ImGui::InputFloat("spring const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_axis1.m_springK = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_axis1.m_damperC;
		if (ImGui::InputFloat("damper const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_axis1.m_damperC = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		value = joint->m_axis1.m_springDamperRegularizer;
		if (ImGui::InputFloat("regularizer##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_axis1.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}

		char enableLimist[64];
		if (joint->m_axis1.m_limitState)
		{
			snprintf(enableLimist, sizeof(enableLimist) - 1, "true");
		}
		else
		{
			snprintf(enableLimist, sizeof(enableLimist) - 1, "false");
		}

		if (ImGui::BeginCombo("limits on##5", enableLimist))
		{
			if (ImGui::Selectable("true", joint->m_axis1.m_limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_axis1.m_limitState = true;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			if (ImGui::Selectable("false", !joint->m_axis1.m_limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_axis1.m_limitState = false;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			ImGui::EndCombo();
		}

		if (joint->m_axis1.m_limitState)
		{
			value = joint->m_axis1.m_minLimit;
			if (ImGui::InputFloat("min limit##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_axis1.m_minLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
			value = joint->m_axis1.m_maxLimit;
			if (ImGui::InputFloat("max limit##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
				joint->m_axis1.m_maxLimit = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			}
		}
	}
}

void ndAssetEditor::EditWheelLoopJoint(ndSharedPtr<ndMeshLoopJoint>& loopJoint)
{
	EditLoopJointGlobalMatrix(loopJoint);

	ndMeshJointWheel* const joint = (ndMeshJointWheel*)*loopJoint->m_joint;

	ImGui::SeparatorText("actuator params");
	ndReal value = joint->m_desc->m_springK;
	if (ImGui::InputFloat("spring const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_desc->m_springK = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
	value = joint->m_desc->m_damperC;
	if (ImGui::InputFloat("damper const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_desc->m_damperC = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
	value = joint->m_desc->m_regularizer;
	if (ImGui::InputFloat("regularizer", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_desc->m_regularizer = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}

	value = joint->m_desc->m_upperStop;
	if (ImGui::InputFloat("upper stop", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_desc->m_upperStop = ndClamp(value, ndReal(0.0f), ndReal(10.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
	value = joint->m_desc->m_lowerStop;
	if (ImGui::InputFloat("lower stop", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_desc->m_lowerStop = ndClamp(value, ndReal(-10.0f), ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}

	value = joint->m_desc->m_steeringAngle;
	if (ImGui::InputFloat("steering angle", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_desc->m_steeringAngle = ndClamp(value, ndReal(0.0f), ndReal(45.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}

	value = joint->m_desc->m_brakeTorque;
	if (ImGui::InputFloat("brake torque", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_desc->m_brakeTorque = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}

	value = joint->m_desc->m_handBrakeTorque;
	if (ImGui::InputFloat("hand brake torque", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_desc->m_handBrakeTorque = ndClamp(value, ndReal(0.0f), ndReal(D_LCP_MAX_VALUE));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
}

void ndAssetEditor::EditSphericalLoopJoint(ndSharedPtr<ndMeshLoopJoint>& loopJoint)
{
	EditLoopJointGlobalMatrix(loopJoint);

	ndMeshJointSpherical* const joint = (ndMeshJointSpherical*)*loopJoint->m_joint;

	ImGui::SeparatorText("actuator params");
	ndReal value = joint->m_axis.m_springK;
	if (ImGui::InputFloat("spring const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_axis.m_springK = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
	value = joint->m_axis.m_damperC;
	if (ImGui::InputFloat("damper const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_axis.m_damperC = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
	value = joint->m_axis.m_springDamperRegularizer;
	if (ImGui::InputFloat("regularizer", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_axis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
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

	if (ImGui::BeginCombo("twist limits on##10", enableLimist))
	{
		if (ImGui::Selectable("true", joint->m_axis.m_limitState))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_axis.m_limitState = true;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		if (ImGui::Selectable("false", !joint->m_axis.m_limitState))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_axis.m_limitState = false;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		ImGui::EndCombo();
	}

	value = joint->m_axis.m_maxLimit;
	if (ImGui::InputFloat("max twist", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_axis.m_maxLimit = ndClamp(value, ndReal(0.0f), ndReal(180.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
	value = joint->m_axis.m_minLimit;
	if (ImGui::InputFloat("min twist", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_axis.m_minLimit = ndClamp(value, ndReal(-180.0f), ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}


	if (joint->m_coneAngleState)
	{
		snprintf(enableLimist, sizeof(enableLimist) - 1, "true");
	}
	else
	{
		snprintf(enableLimist, sizeof(enableLimist) - 1, "false");
	}
	if (ImGui::BeginCombo("cone limits on##10", enableLimist))
	{
		if (ImGui::Selectable("true", joint->m_coneAngleState))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_coneAngleState = true;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		if (ImGui::Selectable("false", !joint->m_coneAngleState))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
			joint->m_coneAngleState = false;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		}
		ImGui::EndCombo();
	}

	value = joint->m_maxConeAngle;
	if (ImGui::InputFloat("cone angle", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_maxConeAngle = ndClamp(value, ndReal(0.0f), ndReal(180.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
}

void ndAssetEditor::EditGearLoopJoint(ndSharedPtr<ndMeshLoopJoint>& loopJoint)
{
	EditLoopJointLocalMatrix(loopJoint);

	ndMeshJointGear* const joint = (ndMeshJointGear*)*loopJoint->m_joint;

	ndReal value = joint->m_ratio;
	if (ImGui::InputFloat("gear ratio", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_ratio = ndMax(value, ndReal(0.01f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
}

void ndAssetEditor::EditFix6dofLoopJoint(ndSharedPtr<ndMeshLoopJoint>& loopJoint)
{
	EditLoopJointGlobalMatrix(loopJoint);

	ndMeshJointFix6dof* const joint = (ndMeshJointFix6dof*)*loopJoint->m_joint;

	ndReal value = joint->m_softness;
	if (ImGui::InputFloat("softness", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_softness = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}

	value = joint->m_maxForce;
	if (ImGui::InputFloat("max force", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_maxForce = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}

	value = joint->m_maxTorque;
	if (ImGui::InputFloat("max torque", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		joint->m_maxTorque = ndMax(value, ndReal(0.0f));
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
	}
}

void ndAssetEditor::EditSwivelPositionEffectorLoopJoint(ndSharedPtr<ndMeshLoopJoint>& loopJoint)
{
	EditLoopJointLocalMatrix(loopJoint);

	ndMeshJointIkSwivelPositionEffector* const joint = (ndMeshJointIkSwivelPositionEffector*)*loopJoint->m_joint;

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

		if (ImGui::BeginCombo("order mode##11", rotationOrder))
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

		if (ImGui::BeginCombo("swivel on##10", swivelMode))
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
