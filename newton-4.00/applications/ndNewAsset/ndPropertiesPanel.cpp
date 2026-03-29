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
#include "ndAssetEditor.h"

void ndAssetEditor::ShowPropertiesPanel()
{
	ImGui::Begin("Properties Panel");

	WindowFrame frame;
	frame.m_posit = ImGui::GetWindowPos();
	frame.m_size = ImGui::GetWindowSize();
	m_windowSizes.PushBack(frame);

	if (*m_currentSelection)
	{
		ShowPropertiesMeshInfo();
		if (*m_currentSelection->GetRigidBody())
		{
			ShowPropertiesRigidBodyInfo();
		}
	}
	
	ImGui::End();
}

void ndAssetEditor::ShowPropertiesMeshInfo()
{
	if (ImGui::CollapsingHeader("Transforms"))
	{
		char nodeName[256];
		snprintf(nodeName, sizeof(nodeName) - 1, "%s", m_currentSelection->GetName().GetStr());
		if (ImGui::InputText("node Name1", nodeName, sizeof(nodeName) - 1))
		{
			if (strcmp(m_currentSelection->GetName().GetStr(), nodeName))
			{
				m_currentSelection->SetName(ndString(nodeName));
			}
		}

		// show node matrix
		{
			ImGui::SeparatorText("Transform");
			ndMatrix matrix(m_currentSelection->GetMatrix());
			ndReal position[3];
			position[0] = matrix.m_posit.m_x;
			position[1] = matrix.m_posit.m_y;
			position[2] = matrix.m_posit.m_z;
			if (ImGui::DragFloat3("position", position))
			{
				matrix.m_posit.m_x = position[0];
				matrix.m_posit.m_y = position[1];
				matrix.m_posit.m_z = position[2];
				m_currentSelection->SetMatrix(matrix);
			};

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			euler[0] = radians[0];
			euler[1] = radians[1];
			euler[2] = radians[2];
			if (ImGui::DragFloat3("rotation", euler))
			{
				ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
				newMatrix.m_posit = matrix.m_posit;
				m_currentSelection->SetMatrix(newMatrix);
			};
		}

		// show geometry node matrix
		if (*m_currentSelection->GetMesh())
		{
			ImGui::SeparatorText("geomtry transform");
			ndMatrix matrix(m_currentSelection->GetGeometryMatrix());
			ndReal position[3];
			position[0] = matrix.m_posit.m_x;
			position[1] = matrix.m_posit.m_y;
			position[2] = matrix.m_posit.m_z;
			if (ImGui::DragFloat3("position##1", position))
			{
				matrix.m_posit.m_x = position[0];
				matrix.m_posit.m_y = position[1];
				matrix.m_posit.m_z = position[2];
				m_currentSelection->SetGeometryMatrix(matrix);
			};

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			euler[0] = radians[0];
			euler[1] = radians[1];
			euler[2] = radians[2];
			if (ImGui::DragFloat3("rotation##1", euler))
			{
				ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
				newMatrix.m_posit = matrix.m_posit;
				m_currentSelection->SetGeometryMatrix(newMatrix);
			};
		}
	}
}

void ndAssetEditor::ShowPropertiesRigidBodyInfo()
{
	if (ImGui::CollapsingHeader("Rigid body"))
	{
		ndSharedPtr<ndMeshBody> body (m_currentSelection->GetRigidBody());
		ndMeshBodyDynamic* const rigidBody = (ndMeshBodyDynamic*)*body;

		// body mass
		{
			ndReal scalar = ndReal (ndFloat32 (1.0f) / rigidBody->m_invMass.m_w);
			if (ImGui::DragFloat("mass", &scalar))
			{
				scalar = ndMax(scalar, ndReal(0.001f));
				rigidBody->m_invMass.m_w = ndFloat32(1.0f) / scalar;
			};
		}

		// body max angular integration step 
		{
			ndReal scalar = ndReal(rigidBody->m_maxAngleStep);
			if (ImGui::DragFloat("angle Step", &scalar))
			{
				scalar = ndClamp(scalar, ndReal(10.0f), ndReal(180.0f));
				rigidBody->m_maxAngleStep = scalar;
			};
		}

		// body max linear integration step 
		{
			ndReal scalar = ndReal(rigidBody->m_maxLinearStep);
			if (ImGui::DragFloat("linear Step", &scalar))
			{
				scalar = ndClamp(scalar, ndReal(0.1f), ndReal(30.0f));
				rigidBody->m_maxLinearStep = scalar;
			};
		}

		// body intrinsic linear damp 
		{
			ndReal scalar = ndReal(rigidBody->m_intrinsicDamping.m_w);
			if (ImGui::DragFloat("linear Damp", &scalar))
			{
				scalar = ndClamp(scalar, ndReal(0.0f), ndReal(1.0f));
				rigidBody->m_intrinsicDamping.m_w = scalar;
			};
		}

		// body center of mass
		{
			ndVector vector(rigidBody->m_localCentreOfMass);
			ndReal real[3];
			real[0] = vector.m_x;
			real[1] = vector.m_y;
			real[2] = vector.m_z;
			if (ImGui::DragFloat3("com", real))
			{
				vector.m_x = real[0];
				vector.m_y = real[1];
				vector.m_z = real[2];
				rigidBody->m_localCentreOfMass = vector;
			};
		}

		// body initial linear velocity
		{
			ndVector vector(rigidBody->m_veloc);
			ndReal real[3];
			real[0] = vector.m_x;
			real[1] = vector.m_y;
			real[2] = vector.m_z;
			if (ImGui::DragFloat3("veloc", real))
			{
				vector.m_x = real[0];
				vector.m_y = real[1];
				vector.m_z = real[2];
				rigidBody->m_veloc = vector;
			};
		}

		// body initial angular velocity
		{
			ndVector vector(rigidBody->m_omega);
			ndReal real[3];
			real[0] = vector.m_x;
			real[1] = vector.m_y;
			real[2] = vector.m_z;
			if (ImGui::DragFloat3("omega", real))
			{
				vector.m_x = real[0];
				vector.m_y = real[1];
				vector.m_z = real[2];
				rigidBody->m_omega = vector;
			};
		}

		// body intrinsic angulat damp
		{
			ndVector vector(rigidBody->m_intrinsicDamping);
			ndReal real[3];
			real[0] = vector.m_x;
			real[1] = vector.m_y;
			real[2] = vector.m_z;
			if (ImGui::DragFloat3("angle Damp", real))
			{
				vector.m_x = real[0];
				vector.m_y = real[1];
				vector.m_z = real[2];
				rigidBody->m_intrinsicDamping = vector;
			};
		}
	}
}