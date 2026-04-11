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

class ndUndoRedoShape : public ndUndoRedoCommand
{
	public:
	ndUndoRedoShape(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());

		m_scale = body->m_shapeInstance.m_scale;
		m_localFrame = body->m_shapeInstance.m_localMatrix;
	}

	virtual ndUndoRedoShape* GetAsUndoRedoShape() const override
	{
		return (ndUndoRedoShape*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoShape* const other = command.GetAsUndoRedoShape();
			if (other)
			{
				bool test = (m_localFrame * other->m_localFrame.OrthoInverse()).TestIdentity();
				test = test && ((m_scale - other->m_scale).DotProduct(m_scale - other->m_scale).GetScalar() < ndFloat32(-0.001f));
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
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());

		body->m_shapeInstance.m_scale = m_scale;
		body->m_shapeInstance.m_localMatrix = m_localFrame;
	}

	ndMatrix m_localFrame;
	ndVector m_scale;
};

class ndUndoRedoShapeChange : public ndUndoRedoCommand
{
	public:
	ndUndoRedoShapeChange(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());
		m_shape = body->m_shapeInstance.m_shape;
	}

	virtual ndUndoRedoShapeChange* GetAsUndoRedoShapeChange() const override
	{
		return (ndUndoRedoShapeChange*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoShapeChange* const other = command.GetAsUndoRedoShapeChange();
			if (other)
			{
				if (m_shape->m_constructor == other->m_shape->m_constructor)
				{
					return false;
				}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());
		body->m_shapeInstance.m_shape = m_shape;
	}

	ndSharedPtr<ndMeshCollisionShape> m_shape;
};

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
					auto InitNewShape = [this, &shape, &shapeInstance](ndSharedPtr<ndShapeInstance>& instance)
					{
						//newJoint->SetLocalMatrix0(joint->m_localFrame0);
						//newJoint->SetLocalMatrix1(joint->m_localFrame1);
						//m_currentSelection->SetJoint(newJoint->GetMeshJoint());
						//joint = m_currentSelection->GetJoint();
						//m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
						instance->Serialize(&shapeInstance);
						shape = ndSharedPtr<ndShape>(shapeInstance.m_shape->CreateObject());
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeChange(this, m_currentSelection)));
					};

					if (strcmp(name, className))
					{
						if (strcmp(name, ndShapeBox::StaticClassName()) == 0)
						{
							m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeChange(this, m_currentSelection)));
							ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionBox());
							InitNewShape(instance);
						}
						else if (strcmp(name, ndShapeSphere::StaticClassName()) == 0)
						{
							m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeChange(this, m_currentSelection)));
							ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionSphere());
							InitNewShape(instance);
						}
						else if (strcmp(name, ndShapeCapsule::StaticClassName()) == 0)
						{
							m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeChange(this, m_currentSelection)));
							ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionCapsule());
							InitNewShape(instance);
						}
						else if (strcmp(name, ndShapeCylinder::StaticClassName()) == 0)
						{
							m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeChange(this, m_currentSelection)));
							ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionCylinder());
							InitNewShape(instance);
						}
						else if (strcmp(name, ndShapeConvexHull::StaticClassName()) == 0)
						{
							m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeChange(this, m_currentSelection)));
							ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionConvex());
							InitNewShape(instance);
						}
						else if (strcmp(name, ndShapeChamferCylinder::StaticClassName()) == 0)
						{
							m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeChange(this, m_currentSelection)));
							ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionChamferCylinder());
							InitNewShape(instance);
						}
						else if (strcmp(name, ndShapeNull::StaticClassName()) == 0)
						{
							m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeChange(this, m_currentSelection)));
							ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionNull());
							InitNewShape(instance);
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

		// show local scaled transform
		{
			ImGui::SeparatorText("local transform");
			ndMatrix matrix(shapeInstance.m_localMatrix);
			ndReal position[3];
			position[0] = ndReal(matrix.m_posit.m_x);
			position[1] = ndReal(matrix.m_posit.m_y);
			position[2] = ndReal(matrix.m_posit.m_z);
			if (ImGui::InputFloat3("position##2", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, m_currentSelection)));
				matrix.m_posit.m_x = position[0];
				matrix.m_posit.m_y = position[1];
				matrix.m_posit.m_z = position[2];
				shapeInstance.m_localMatrix = matrix;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, m_currentSelection)));
			};

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			euler[0] = ndReal(radians[0]);
			euler[1] = ndReal(radians[1]);
			euler[2] = ndReal(radians[2]);
			if (ImGui::InputFloat3("rotation##2", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, m_currentSelection)));
				ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
				newMatrix.m_posit = matrix.m_posit;
				shapeInstance.m_localMatrix = newMatrix;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, m_currentSelection)));
			};

			ndVector vector(shapeInstance.m_scale);
			ndReal real[3];
			real[0] = ndReal(vector.m_x);
			real[1] = ndReal(vector.m_y);
			real[2] = ndReal(vector.m_z);
			if (ImGui::InputFloat3("scale##1", real, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, m_currentSelection)));
				vector.m_x = ndMax(real[0], ndReal(0.001f));
				vector.m_y = ndMax(real[1], ndReal(0.001f));
				vector.m_z = ndMax(real[2], ndReal(0.001f));
				shapeInstance.m_scale = vector;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, m_currentSelection)));
			};
		}
	}
}
