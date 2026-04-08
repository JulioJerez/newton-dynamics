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

void ndAssetEditor::ShowPropertiesJointInfo()
{
	if (ImGui::CollapsingHeader("Constraint joint"))
	{
		ndSharedPtr<ndMeshJoint> joint (m_currentSelection->GetJoint());
		if (ImGui::BeginCombo("joints", joint->m_constructor.GetStr()))
		{
			auto SetDropdownList = [this, &joint](const char* const name)
			{
				bool selected = strcmp(name, joint->m_constructor.GetStr()) ? false : true;
				if (ImGui::Selectable(name, selected))
				{
					if (strcmp(name, ndJointHinge::StaticClassName()) == 0)
					{
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointHinge());
						newJoint->SetLocalMatrix0(joint->m_localFrame0);
						newJoint->SetLocalMatrix1(joint->m_localFrame1);
						m_currentSelection->SetJoint(newJoint->GetMeshJoint());
						joint = m_currentSelection->GetJoint();
					}
					else if (strcmp(name, ndJointSlider::StaticClassName()) == 0)
					{
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointSlider());
						newJoint->SetLocalMatrix0(joint->m_localFrame0);
						newJoint->SetLocalMatrix1(joint->m_localFrame1);
						m_currentSelection->SetJoint(newJoint->GetMeshJoint());
						joint = m_currentSelection->GetJoint();
					}
					else if (strcmp(name, ndJointDoubleHinge::StaticClassName()) == 0)
					{
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointDoubleHinge());
						newJoint->SetLocalMatrix0(joint->m_localFrame0);
						newJoint->SetLocalMatrix1(joint->m_localFrame1);
						m_currentSelection->SetJoint(newJoint->GetMeshJoint());
						joint = m_currentSelection->GetJoint();
					}
					else if (strcmp(name, ndJointSpherical::StaticClassName()) == 0)
					{
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointSpherical());
						newJoint->SetLocalMatrix0(joint->m_localFrame0);
						newJoint->SetLocalMatrix1(joint->m_localFrame1);
						m_currentSelection->SetJoint(newJoint->GetMeshJoint());
						joint = m_currentSelection->GetJoint();
					}
					else if (strcmp(name, ndJointFix6dof::StaticClassName()) == 0)
					{
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointFix6dof());
						newJoint->SetLocalMatrix0(joint->m_localFrame0);
						newJoint->SetLocalMatrix1(joint->m_localFrame1);
						m_currentSelection->SetJoint(newJoint->GetMeshJoint());
						joint = m_currentSelection->GetJoint();
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
			SetDropdownList(ndJointDoubleHinge::StaticClassName());
			SetDropdownList(ndJointSpherical::StaticClassName());
			SetDropdownList(ndJointWheel::StaticClassName());

			ImGui::EndCombo();
		}

		// child local frame
		{
			ImGui::SeparatorText("child local Frame");
			ndMatrix matrix(joint->m_localFrame0);
			ndReal position[3];
			position[0] = ndReal(matrix.m_posit.m_x);
			position[1] = ndReal(matrix.m_posit.m_y);
			position[2] = ndReal(matrix.m_posit.m_z);
			if (ImGui::DragFloat3("position##2", position))
			{
				matrix.m_posit.m_x = position[0];
				matrix.m_posit.m_y = position[1];
				matrix.m_posit.m_z = position[2];
				joint->m_localFrame0 = matrix;
			};

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			euler[0] = ndReal(radians[0]);
			euler[1] = ndReal(radians[1]);
			euler[2] = ndReal(radians[2]);
			if (ImGui::DragFloat3("rotation##2", euler))
			{
				ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
				newMatrix.m_posit = matrix.m_posit;
				joint->m_localFrame0 = newMatrix;
			};
		}

		// parent local frame
		{
			ImGui::SeparatorText("parent local Frame");
			ndMatrix matrix(joint->m_localFrame1);
			ndReal position[3];
			position[0] = ndReal(matrix.m_posit.m_x);
			position[1] = ndReal(matrix.m_posit.m_y);
			position[2] = ndReal(matrix.m_posit.m_z);
			if (ImGui::DragFloat3("position##3", position))
			{
				matrix.m_posit.m_x = position[0];
				matrix.m_posit.m_y = position[1];
				matrix.m_posit.m_z = position[2];
				joint->m_localFrame1 = matrix;
			};

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			euler[0] = ndReal(radians[0]);
			euler[1] = ndReal(radians[1]);
			euler[2] = ndReal(radians[2]);
			if (ImGui::DragFloat3("rotation##3", euler))
			{
				ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
				newMatrix.m_posit = matrix.m_posit;
				joint->m_localFrame1 = newMatrix;
			};
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

