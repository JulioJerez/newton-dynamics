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
			ShowPropertiesCollisionInfo();

			if (*m_currentSelection->GetJoint())
			{
				ShowPropertiesJointInfo();
			}
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

		// body intrinsic angular damp
		{
			ndVector vector(rigidBody->m_intrinsicDamping);
			ndReal real[3];
			real[0] = vector.m_x;
			real[1] = vector.m_y;
			real[2] = vector.m_z;
			if (ImGui::DragFloat3("angular damp", real))
			{
				vector.m_x = real[0];
				vector.m_y = real[1];
				vector.m_z = real[2];
				rigidBody->m_intrinsicDamping = vector;
			};
		}

		// body principal axis of inertia
		{
			ndReal euler[3];
			ndVector tmp;
			ndMatrix matrix(rigidBody->m_inertiaPrincipalAxis);
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			euler[0] = radians[0];
			euler[1] = radians[1];
			euler[2] = radians[2];
			if (ImGui::DragFloat3("inertia axis", euler))
			{
				const ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
				rigidBody->m_inertiaPrincipalAxis = newMatrix;
			};
		}
	}
}

void ndAssetEditor::ShowPropertiesCollisionInfo()
{
	if (ImGui::CollapsingHeader("Collision shape"))
	{
		ndSharedPtr<ndMeshBody> body(m_currentSelection->GetRigidBody());
		ndMeshBodyKinematic* const rigidBody = (ndMeshBodyKinematic*)*body;
		ndMeshShapeInstance& shapeInstance = rigidBody->m_shapeInstance;

		ndSharedPtr<ndShape> shape(shapeInstance.m_shape->CreateObject());
		if (ImGui::BeginCombo("shapes", shape->ClassName()))
		{
			auto SetDropdownList = [this, rigidBody, &shape, &shapeInstance](const char* name)
			{
				bool selected = strcmp(name, shape->ClassName()) ? false : true;
				if (ImGui::Selectable(name, selected))
				{
					if (strcmp(name, shape->ClassName()))
					{
						if (strcmp(name, ndShapeBox::StaticClassName()) == 0)
						{
							ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionBox());
							instance->Serialize(&shapeInstance);
							shape = ndSharedPtr<ndShape>(shapeInstance.m_shape->CreateObject());
						}
						else if (strcmp(name, ndShapeSphere::StaticClassName()) == 0)
						{
							ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionSphere());
							instance->Serialize(&shapeInstance);
							shape = ndSharedPtr<ndShape>(shapeInstance.m_shape->CreateObject());
						}
						else if (strcmp(name, ndShapeCapsule::StaticClassName()) == 0)
						{
							ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionCapsule());
							instance->Serialize(&shapeInstance);
							shape = ndSharedPtr<ndShape>(shapeInstance.m_shape->CreateObject());
						}
						else if (strcmp(name, ndShapeConvexHull::StaticClassName()) == 0)
						{
							ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionConvex());
							instance->Serialize(&shapeInstance);
							shape = ndSharedPtr<ndShape>(shapeInstance.m_shape->CreateObject());
						}
						else if (strcmp(name, ndShapeChamferCylinder::StaticClassName()) == 0)
						{
							ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionChamferCylinder());
							instance->Serialize(&shapeInstance);
							shape = ndSharedPtr<ndShape>(shapeInstance.m_shape->CreateObject());
						}
						else
						{
							ndAssert(0);
						}
					}
				}
			};
			SetDropdownList(ndShapeBox::StaticClassName());
			SetDropdownList(ndShapeSphere::StaticClassName());
			SetDropdownList(ndShapeCapsule::StaticClassName());
			SetDropdownList(ndShapeConvexHull::StaticClassName());
			SetDropdownList(ndShapeChamferCylinder::StaticClassName());

			ImGui::EndCombo();
		}

		if (strcmp(shape->ClassName(), ndShapeBox::StaticClassName()) == 0)
		{
			ndReal size[3];
			size[0] = 1;
			size[1] = 2;
			size[2] = 3;
			if (ImGui::DragFloat3("size##1", size))
			{

			}
		}
		else if (strcmp(shape->ClassName(), ndShapeSphere::StaticClassName()) == 0)
		{
			ndReal size = ndReal(1.0f);
			if (ImGui::DragFloat("radio0##1", &size))
			{

			}
		}
		else if (strcmp(shape->ClassName(), ndShapeCapsule::StaticClassName()) == 0)
		{
			ndReal radio0 = ndReal(1.0f);
			if (ImGui::DragFloat("radio0##1", &radio0))
			{

			}

			ndReal radio1 = ndReal(2.0f);
			if (ImGui::DragFloat("radio1##1", &radio1))
			{

			}
		}
		else if (strcmp(shape->ClassName(), ndShapeConvexHull::StaticClassName()) == 0)
		{
			ndInt32 points = 100;
			if (ImGui::DragInt("max point count##1", &points))
			{

			}
		}
		else if (strcmp(shape->ClassName(), ndShapeChamferCylinder::StaticClassName()) == 0)
		{
			ndReal size[3];
			size[0] = 1;
			size[1] = 2;
			size[2] = 3;
			if (ImGui::DragFloat3("size##1", size))
			{

			}
		}

		else
		{
			ndAssert(0);
		}

		// shaw shape scale
		{
			ndVector vector(shapeInstance.m_scale);
			ndReal real[3];
			real[0] = vector.m_x;
			real[1] = vector.m_y;
			real[2] = vector.m_z;
			if (ImGui::DragFloat3("scale##1", real))
			{
				vector.m_x = ndMax(real[0], ndReal(0.01f));
				vector.m_y = ndMax(real[1], ndReal(0.01f));
				vector.m_z = ndMax(real[2], ndReal(0.01f));
				shapeInstance.m_scale = vector;
			};
		}

		// show local transform
		{
			//ImGui::SeparatorText("local transform");
			ndMatrix matrix(shapeInstance.m_localMatrix);
			ndReal position[3];
			position[0] = matrix.m_posit.m_x;
			position[1] = matrix.m_posit.m_y;
			position[2] = matrix.m_posit.m_z;
			if (ImGui::DragFloat3("position##2", position))
			{
				matrix.m_posit.m_x = position[0];
				matrix.m_posit.m_y = position[1];
				matrix.m_posit.m_z = position[2];
				shapeInstance.m_localMatrix = matrix;
			};

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			euler[0] = radians[0];
			euler[1] = radians[1];
			euler[2] = radians[2];
			if (ImGui::DragFloat3("rotation##2", euler))
			{
				ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
				newMatrix.m_posit = matrix.m_posit;
				shapeInstance.m_localMatrix = newMatrix;
			};
		}
	}
}

void ndAssetEditor::ShowPropertiesJointInfo()
{
	if (ImGui::CollapsingHeader("Constraint joint"))
	{
		ndSharedPtr<ndMeshJoint>& joint (m_currentSelection->GetJoint());

		if (ImGui::BeginCombo("joints", joint->m_constructor.GetStr()))
		{
			auto SetDropdownList = [this, &joint](const char* name)
			{
				bool selected = strcmp(name, joint->m_constructor.GetStr()) ? false : true;
				if (ImGui::Selectable(name, selected))
				{
					//if (strcmp(name, shape->ClassName()))
					//{
					//	if (strcmp(name, ndShapeBox::StaticClassName()) == 0)
					//	{
					//		ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionBox());
					//		instance->Serialize(&shapeInstance);
					//		shape = ndSharedPtr<ndShape>(shapeInstance.m_shape->CreateObject());
					//	}
					//	else if (strcmp(name, ndShapeSphere::StaticClassName()) == 0)
					//	{
					//		ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionSphere());
					//		instance->Serialize(&shapeInstance);
					//		shape = ndSharedPtr<ndShape>(shapeInstance.m_shape->CreateObject());
					//	}
					//	else if (strcmp(name, ndShapeCapsule::StaticClassName()) == 0)
					//	{
					//		ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionCapsule());
					//		instance->Serialize(&shapeInstance);
					//		shape = ndSharedPtr<ndShape>(shapeInstance.m_shape->CreateObject());
					//	}
					//	else
					//	{
					//		ndAssert(0);
					//	}
					//}
				}
			};
			SetDropdownList(ndJointFix6dof::StaticClassName());
			SetDropdownList(ndJointHinge::StaticClassName());
			SetDropdownList(ndJointSlider::StaticClassName());
			SetDropdownList(ndJointDoubleHinge::StaticClassName());
			SetDropdownList(ndJointSpherical::StaticClassName());

			ImGui::EndCombo();
		}

		// child local frame
		{
			ImGui::SeparatorText("child local Frame");
			ndMatrix matrix(joint->m_locatFrame0);
			ndReal position[3];
			position[0] = matrix.m_posit.m_x;
			position[1] = matrix.m_posit.m_y;
			position[2] = matrix.m_posit.m_z;
			if (ImGui::DragFloat3("position##2", position))
			{
				matrix.m_posit.m_x = position[0];
				matrix.m_posit.m_y = position[1];
				matrix.m_posit.m_z = position[2];
				joint->m_locatFrame0 = matrix;
			};

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			euler[0] = radians[0];
			euler[1] = radians[1];
			euler[2] = radians[2];
			if (ImGui::DragFloat3("rotation##2", euler))
			{
				ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
				newMatrix.m_posit = matrix.m_posit;
				joint->m_locatFrame0 = newMatrix;
			};
		}

		// parent local frame
		{
			ImGui::SeparatorText("parent local Frame");
			ndMatrix matrix(joint->m_locatFrame1);
			ndReal position[3];
			position[0] = matrix.m_posit.m_x;
			position[1] = matrix.m_posit.m_y;
			position[2] = matrix.m_posit.m_z;
			if (ImGui::DragFloat3("position##3", position))
			{
				matrix.m_posit.m_x = position[0];
				matrix.m_posit.m_y = position[1];
				matrix.m_posit.m_z = position[2];
				joint->m_locatFrame1 = matrix;
			};

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			euler[0] = radians[0];
			euler[1] = radians[1];
			euler[2] = radians[2];
			if (ImGui::DragFloat3("rotation##3", euler))
			{
				ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
				newMatrix.m_posit = matrix.m_posit;
				joint->m_locatFrame1 = newMatrix;
			};
		}
	}
}