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

class ndUndoRedoJoint : public ndUndoRedoCommand
{
	public:
	ndUndoRedoJoint(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
		,m_localFrame(m_mesh->GetJoint()->m_localFrame0)
	{
	}

	virtual ndUndoRedoJoint* GetAsUndoRedoJoint() const override
	{
		return (ndUndoRedoJoint*)this;
	}
	
	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoJoint* const other = command.GetAsUndoRedoJoint();
			if (other)
			{
				ndMatrix matrix(m_localFrame * other->m_localFrame.OrthoInverse());
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
		ndMeshJoint* const joint = *m_mesh->GetJoint();
		joint->m_localFrame0 = m_localFrame;

		ndMatrix globalMatrix(m_localFrame * m_mesh->CalculateGlobalMatrix());
		joint->m_localFrame1 = globalMatrix * m_mesh->GetParent()->CalculateGlobalMatrix().OrthoInverse();
	}

	ndMatrix m_localFrame;
};

class ndUndoRedoJointChange : public ndUndoRedoCommand
{
	public:
	ndUndoRedoJointChange(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
		,m_joint(m_mesh->GetJoint())
	{
	}

	virtual ndUndoRedoJointChange* GetAsUndoRedoJointChange() const override
	{
		return (ndUndoRedoJointChange*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoJointChange* const other = command.GetAsUndoRedoJointChange();
			if (other)
			{
				if (m_joint->m_constructor == other->m_joint->m_constructor)
				{
					return false;
				}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		m_mesh->SetJoint(m_joint);
	}

	ndSharedPtr<ndMeshJoint> m_joint;
};


void ndAssetEditor::ShowPropertiesJointInfo()
{
	if (ImGui::CollapsingHeader("Constraint joint"))
	{
		ndSharedPtr<ndMeshJoint> joint (m_currentSelection->GetJoint());

		// child local frame
		{
			ImGui::SeparatorText("child local Frame");

			const ndMatrix matrix(joint->m_localFrame0);
			ndReal position[3];
			position[0] = ndReal(matrix.m_posit.m_x);
			position[1] = ndReal(matrix.m_posit.m_y);
			position[2] = ndReal(matrix.m_posit.m_z);
			if (ImGui::DragFloat3("position##2", position))
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
			};

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
			};
		}

		if (ImGui::BeginCombo("joints", joint->m_constructor.GetStr()))
		{
			auto SetDropdownList = [this, &joint](const char* const name)
			{
				bool selected = strcmp(name, joint->m_constructor.GetStr()) ? false : true;
				if (ImGui::Selectable(name, selected))
				{
					if (strcmp(name, ndJointHinge::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointHinge());
						newJoint->SetLocalMatrix0(joint->m_localFrame0);
						newJoint->SetLocalMatrix1(joint->m_localFrame1);
						m_currentSelection->SetJoint(newJoint->GetMeshJoint());
						joint = m_currentSelection->GetJoint();
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
					}
					else if (strcmp(name, ndJointSlider::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointSlider());
						newJoint->SetLocalMatrix0(joint->m_localFrame0);
						newJoint->SetLocalMatrix1(joint->m_localFrame1);
						m_currentSelection->SetJoint(newJoint->GetMeshJoint());
						joint = m_currentSelection->GetJoint();
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
					}
					else if (strcmp(name, ndJointDoubleHinge::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointDoubleHinge());
						newJoint->SetLocalMatrix0(joint->m_localFrame0);
						newJoint->SetLocalMatrix1(joint->m_localFrame1);
						m_currentSelection->SetJoint(newJoint->GetMeshJoint());
						joint = m_currentSelection->GetJoint();
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
					}
					else if (strcmp(name, ndJointSpherical::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointSpherical());
						newJoint->SetLocalMatrix0(joint->m_localFrame0);
						newJoint->SetLocalMatrix1(joint->m_localFrame1);
						m_currentSelection->SetJoint(newJoint->GetMeshJoint());
						joint = m_currentSelection->GetJoint();
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
					}
					else if (strcmp(name, ndJointFix6dof::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointFix6dof());
						newJoint->SetLocalMatrix0(joint->m_localFrame0);
						newJoint->SetLocalMatrix1(joint->m_localFrame1);
						m_currentSelection->SetJoint(newJoint->GetMeshJoint());
						joint = m_currentSelection->GetJoint();
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
					}
					else
					{
						ndAssert(0);
					}
				}
			};
			SetDropdownList(ndJointFix6dof::StaticClassName());
			SetDropdownList(ndJointSlider::StaticClassName());
			SetDropdownList(ndJointHinge::StaticClassName());
			SetDropdownList(ndJointDoubleHinge::StaticClassName());
			SetDropdownList(ndJointSpherical::StaticClassName());
			SetDropdownList(ndJointWheel::StaticClassName());

			ImGui::EndCombo();
		}

		if (strcmp(joint->m_constructor.GetStr(), ndJointHinge::StaticClassName()) == 0)
		{
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointSlider::StaticClassName()) == 0)
		{
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointDoubleHinge::StaticClassName()) == 0)
		{
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointSpherical::StaticClassName()) == 0)
		{
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointFix6dof::StaticClassName()) == 0)
		{
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointWheel::StaticClassName()) == 0)
		{
		}
		else
		{
			ndAssert(0);
		}
	}
}

