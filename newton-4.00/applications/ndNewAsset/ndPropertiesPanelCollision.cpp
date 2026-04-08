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

void ndAssetEditor::ShowPropertiesCollisionInfo()
{
	if (ImGui::CollapsingHeader("Collision shape"))
	{
		ndSharedPtr<ndMeshBody> body(m_currentSelection->GetRigidBody());
		ndMeshBodyKinematic* const rigidBody = (ndMeshBodyKinematic*)*body;
		ndMeshShapeInstance& shapeInstance = rigidBody->m_shapeInstance;

		ndSharedPtr<ndShape> shape(shapeInstance.m_shape->CreateObject());
		const char* const className = shape->ClassName();
		if (ImGui::BeginCombo("shapes", className))
		{
			auto SetDropdownList = [this, rigidBody, &shape, &shapeInstance, &className](const char* const name)
			{
				bool selected = strcmp(name, className) ? false : true;
				if (ImGui::Selectable(name, selected))
				{
					if (strcmp(name, className))
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
						else if (strcmp(name, ndShapeCylinder::StaticClassName()) == 0)
						{
							ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionCylinder());
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
						else if (strcmp(name, ndShapeNull::StaticClassName()) == 0)
						{
							ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionNull());
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
			SetDropdownList(ndShapeNull::StaticClassName());
			SetDropdownList(ndShapeBox::StaticClassName());
			SetDropdownList(ndShapeSphere::StaticClassName());
			SetDropdownList(ndShapeCapsule::StaticClassName());
			SetDropdownList(ndShapeCylinder::StaticClassName());
			SetDropdownList(ndShapeConvexHull::StaticClassName());
			SetDropdownList(ndShapeChamferCylinder::StaticClassName());
			SetDropdownList(ndShapeCompound::StaticClassName());

			ImGui::EndCombo();
		}
		
		if (strcmp(className, ndShapeBox::StaticClassName()) == 0)
		{
			ndReal size[3];
			size[0] = 1;
			size[1] = 2;
			size[2] = 3;
			if (ImGui::DragFloat3("size##1", size))
			{

			}
		}
		else if (strcmp(className, ndShapeSphere::StaticClassName()) == 0)
		{
			ndReal size = ndReal(1.0f);
			if (ImGui::DragFloat("radio0##1", &size))
			{

			}
		}
		else if (strcmp(className, ndShapeCapsule::StaticClassName()) == 0)
		{
			ndReal radio0 = ndReal(1.0f);
			if (ImGui::DragFloat("radio0##1", &radio0))
			{
			}

			ndReal radio1 = ndReal(2.0f);
			if (ImGui::DragFloat("radio1##1", &radio1))
			{
			}

			ndReal length = ndReal(2.0f);
			if (ImGui::DragFloat("lenght##1", &length))
			{
			}

		}
		else if (strcmp(className, ndShapeCylinder::StaticClassName()) == 0)
		{
			ndReal radio0 = ndReal(1.0f);
			if (ImGui::DragFloat("radio0##2", &radio0))
			{
			}

			ndReal radio1 = ndReal(2.0f);
			if (ImGui::DragFloat("radio1##2", &radio1))
			{
			}

			ndReal length = ndReal(2.0f);
			if (ImGui::DragFloat("lenght##2", &length))
			{
			}
		}

		else if (strcmp(className, ndShapeConvexHull::StaticClassName()) == 0)
		{
			ndInt32 points = 100;
			if (ImGui::DragInt("max point count##1", &points))
			{

			}
		}
		else if (strcmp(className, ndShapeChamferCylinder::StaticClassName()) == 0)
		{
			ndReal size[3];
			size[0] = 1;
			size[1] = 2;
			size[2] = 3;
			if (ImGui::DragFloat3("size##1", size))
			{

			}
		}
		else if (strcmp(className, ndShapeCompound::StaticClassName()) == 0)
		{
			ndReal size = 1;
			if (ImGui::DragFloat("size##1", &size))
			{

			}
		}
		else if (strcmp(className, ndShapeNull::StaticClassName()) == 0)
		{
			ndReal size = 1;
			if (ImGui::DragFloat("size##1", &size))
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
			real[0] = ndReal(vector.m_x);
			real[1] = ndReal(vector.m_y);
			real[2] = ndReal(vector.m_z);
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
			position[0] = ndReal(matrix.m_posit.m_x);
			position[1] = ndReal(matrix.m_posit.m_y);
			position[2] = ndReal(matrix.m_posit.m_z);
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

			euler[0] = ndReal(radians[0]);
			euler[1] = ndReal(radians[1]);
			euler[2] = ndReal(radians[2]);
			if (ImGui::DragFloat3("rotation##2", euler))
			{
				ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
				newMatrix.m_posit = matrix.m_posit;
				shapeInstance.m_localMatrix = newMatrix;
			};
		}
	}
}
