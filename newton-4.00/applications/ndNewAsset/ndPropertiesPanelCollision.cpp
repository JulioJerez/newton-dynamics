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
#include "ndDebugDisplayRenderPass.h"

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
		m_editor->GetDebugDisplay()->RebuildDebugCollision();
	}

	ndSharedPtr<ndMeshCollisionShape> m_shape;
};

class ndUndoRedoShapeModified : public ndUndoRedoCommand
{
	public:
	ndUndoRedoShapeModified(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
		,m_shapeParam(ndVector::m_zero)
	{
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());
		const ndMeshCollisionShape* const shape = *body->m_shapeInstance.m_shape;
		if (strcmp(shape->m_constructor.GetStr(), ndShapeBox::StaticClassName()) == 0)
		{
			const ndMeshCollisionShapeBox* const subJoint = (ndMeshCollisionShapeBox*)shape;
			m_shapeParam.m_x = subJoint->m_x;
			m_shapeParam.m_y = subJoint->m_y;
			m_shapeParam.m_z = subJoint->m_z;
		}
		else if (strcmp(shape->m_constructor.GetStr(), ndShapeSphere::StaticClassName()) == 0)
		{
			const ndMeshCollisionShapeSphere* const subJoint = (ndMeshCollisionShapeSphere*)shape;
			m_shapeParam.m_x = subJoint->m_radius;
		}
		else if (strcmp(shape->m_constructor.GetStr(), ndShapeCapsule::StaticClassName()) == 0)
		{
			const ndMeshCollisionShapeCapsule* const subJoint = (ndMeshCollisionShapeCapsule*)shape;
			m_shapeParam.m_x = subJoint->m_height;
			m_shapeParam.m_y = subJoint->m_radius0;
			m_shapeParam.m_z = subJoint->m_radius1;
		}
		else if (strcmp(shape->m_constructor.GetStr(), ndShapeCylinder::StaticClassName()) == 0)
		{
			const ndMeshCollisionShapeCylinder* const subJoint = (ndMeshCollisionShapeCylinder*)shape;
			m_shapeParam.m_x = subJoint->m_height;
			m_shapeParam.m_y = subJoint->m_radius0;
			m_shapeParam.m_z = subJoint->m_radius1;
		}
		else if (strcmp(shape->m_constructor.GetStr(), ndShapeChamferCylinder::StaticClassName()) == 0)
		{
			const ndMeshCollisionShapeChamferCylinder* const subJoint = (ndMeshCollisionShapeChamferCylinder*)shape;
			m_shapeParam.m_x = subJoint->m_height;
			m_shapeParam.m_y = subJoint->m_radius;
		}
		else if (strcmp(shape->m_constructor.GetStr(), ndShapeConvexHull::StaticClassName()) == 0)
		{
			const ndMeshCollisionShapeConvexHull* const subJoint = (ndMeshCollisionShapeConvexHull*)shape;
			m_shapeParam.m_x = ndFloat32 (subJoint->m_maxPointCount);
		}
		else
		{
			ndAssert(0);
		}
	}

	virtual ndUndoRedoShapeModified* GetAsUndoRedoShapeModified() const override
	{
		return (ndUndoRedoShapeModified*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoShapeModified* const other = command.GetAsUndoRedoShapeModified();
			if (other)
			{
				bool test = (m_shapeParam - other->m_shapeParam).DotProduct(m_shapeParam - other->m_shapeParam).GetScalar() < ndFloat32(0.0001f);
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
		ndMeshCollisionShape* const shape = *body->m_shapeInstance.m_shape;
		if (strcmp(shape->m_constructor.GetStr(), ndShapeBox::StaticClassName()) == 0)
		{
			ndMeshCollisionShapeBox* const subJoint = (ndMeshCollisionShapeBox*)shape;
			subJoint->m_x = m_shapeParam.m_x;
			subJoint->m_y = m_shapeParam.m_y;
			subJoint->m_z = m_shapeParam.m_z;
		}
		else if (strcmp(shape->m_constructor.GetStr(), ndShapeSphere::StaticClassName()) == 0)
		{
			ndMeshCollisionShapeSphere* const subJoint = (ndMeshCollisionShapeSphere*)shape;
			subJoint->m_radius = m_shapeParam.m_x;
		}
		else if (strcmp(shape->m_constructor.GetStr(), ndShapeCapsule::StaticClassName()) == 0)
		{
			ndMeshCollisionShapeCapsule* const subJoint = (ndMeshCollisionShapeCapsule*)shape;
			subJoint->m_height = m_shapeParam.m_x;
			subJoint->m_radius0 = m_shapeParam.m_y;
			subJoint->m_radius1 = m_shapeParam.m_z;
		}
		else if (strcmp(shape->m_constructor.GetStr(), ndShapeCylinder::StaticClassName()) == 0)
		{
			ndMeshCollisionShapeCylinder* const subJoint = (ndMeshCollisionShapeCylinder*)shape;
			subJoint->m_height = m_shapeParam.m_x;
			subJoint->m_radius0 = m_shapeParam.m_y;
			subJoint->m_radius1 = m_shapeParam.m_z;
		}
		else if (strcmp(shape->m_constructor.GetStr(), ndShapeChamferCylinder::StaticClassName()) == 0)
		{
			ndMeshCollisionShapeChamferCylinder* const subJoint = (ndMeshCollisionShapeChamferCylinder*)shape;
			subJoint->m_height = m_shapeParam.m_x;
			subJoint->m_radius = m_shapeParam.m_y;
		}
		else if (strcmp(shape->m_constructor.GetStr(), ndShapeConvexHull::StaticClassName()) == 0)
		{
			ndMeshCollisionShapeConvexHull* const subJoint = (ndMeshCollisionShapeConvexHull*)shape;
			subJoint->m_maxPointCount = ndInt32 (m_shapeParam.m_x);
		}
		else
		{
			ndAssert(0);
		}
		m_editor->GetDebugDisplay()->RebuildDebugCollision();
	}

	ndVector m_shapeParam;
};


void ndAssetEditor::ShowPropertiesCollisionInfo()
{
	if (ImGui::CollapsingHeader("Collision shape"))
	{
		ndSharedPtr<ndMeshBody> body(m_currentSelection->GetRigidBody());
		ndMeshBodyKinematic* const rigidBody = (ndMeshBodyKinematic*)*body;
		ndMeshShapeInstance& shapeInstance = rigidBody->m_shapeInstance;

		const char* const className = shapeInstance.m_shape->m_constructor.GetStr();
		if (ImGui::BeginCombo("shapes", className))
		{
			auto SetDropdownList = [this, rigidBody, &shapeInstance, &className](const char* const name)
			{
				bool selected = strcmp(name, className) ? false : true;
				if (ImGui::Selectable(name, selected))
				{
					auto InitNewShape = [this, &shapeInstance](ndSharedPtr<ndShapeInstance>& instance)
					{
						instance->Serialize(&shapeInstance);
						GetDebugDisplay()->RebuildDebugCollision();
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
			SetDropdownList(ndShapeChamferCylinder::StaticClassName());
			SetDropdownList(ndShapeConvexHull::StaticClassName());
			SetDropdownList(ndShapeCompound::StaticClassName());

			ImGui::EndCombo();
		}

		if (strcmp(className, ndShapeNull::StaticClassName()) == 0)
		{
			// null shape does nothing
		}
		else if (strcmp(className, ndShapeBox::StaticClassName()) == 0)
		{
			ndReal value;
			ndMeshCollisionShapeBox* const subJoint = (ndMeshCollisionShapeBox*)*shapeInstance.m_shape;

			value = subJoint->m_x;
			if (ImGui::InputFloat("x##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
				subJoint->m_x = value;
				GetDebugDisplay()->RebuildDebugCollision();
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
			}
			value = subJoint->m_y;
			if (ImGui::InputFloat("y##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
				subJoint->m_y = value;
				GetDebugDisplay()->RebuildDebugCollision();
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
			}
			value = subJoint->m_z;
			if (ImGui::InputFloat("z##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
				subJoint->m_z = value;
				GetDebugDisplay()->RebuildDebugCollision();
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
			}
		}
		else if (strcmp(className, ndShapeSphere::StaticClassName()) == 0)
		{
			ndReal value;
			ndMeshCollisionShapeSphere* const subJoint = (ndMeshCollisionShapeSphere*)*shapeInstance.m_shape;

			value = subJoint->m_radius;
			if (ImGui::InputFloat("radius##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
				subJoint->m_radius = value;
				GetDebugDisplay()->RebuildDebugCollision();
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
			}
		}
		else if (strcmp(className, ndShapeCapsule::StaticClassName()) == 0)
		{
			ndReal value;
			ndMeshCollisionShapeCapsule* const subJoint = (ndMeshCollisionShapeCapsule*)*shapeInstance.m_shape;

			value = subJoint->m_radius0;
			if (ImGui::InputFloat("radius1##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
				subJoint->m_radius0 = value;
				GetDebugDisplay()->RebuildDebugCollision();
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
			}
			value = subJoint->m_radius1;
			if (ImGui::InputFloat("radius2##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
				subJoint->m_radius1 = value;
				GetDebugDisplay()->RebuildDebugCollision();
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
			}
			value = subJoint->m_height;
			if (ImGui::InputFloat("height##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
				subJoint->m_height = value;
				GetDebugDisplay()->RebuildDebugCollision();
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
			}
		}
		else if (strcmp(className, ndShapeCylinder::StaticClassName()) == 0)
		{
			ndReal value;
			ndMeshCollisionShapeCylinder* const subJoint = (ndMeshCollisionShapeCylinder*)*shapeInstance.m_shape;

			value = subJoint->m_radius0;
			if (ImGui::InputFloat("radius1##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
				subJoint->m_radius0 = value;
				GetDebugDisplay()->RebuildDebugCollision();
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
			}
			value = subJoint->m_radius1;
			if (ImGui::InputFloat("radius2##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
				subJoint->m_radius1 = value;
				GetDebugDisplay()->RebuildDebugCollision();
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
			}
			value = subJoint->m_height;
			if (ImGui::InputFloat("height##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
				subJoint->m_height = value;
				GetDebugDisplay()->RebuildDebugCollision();
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
			}
		}
		else if (strcmp(className, ndShapeChamferCylinder::StaticClassName()) == 0)
		{
			ndReal value;
			ndMeshCollisionShapeChamferCylinder* const subJoint = (ndMeshCollisionShapeChamferCylinder*)*shapeInstance.m_shape;

			value = subJoint->m_radius;
			if (ImGui::InputFloat("radius##4", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
				subJoint->m_radius = value;
				GetDebugDisplay()->RebuildDebugCollision();
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
			}
			value = subJoint->m_height;
			if (ImGui::InputFloat("height##4", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
				subJoint->m_height = value;
				GetDebugDisplay()->RebuildDebugCollision();
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
			}
		}
		else if (strcmp(className, ndShapeConvexHull::StaticClassName()) == 0)
		{
			ndInt32 value;
			ndMeshCollisionShapeConvexHull* const subJoint = (ndMeshCollisionShapeConvexHull*)*shapeInstance.m_shape;

			value = subJoint->m_maxPointCount;
			if (ImGui::InputInt("pointCount", &value, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
				subJoint->m_maxPointCount = value;
				GetDebugDisplay()->RebuildDebugCollision();
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeModified(this, m_currentSelection)));
			}
		}
		else if (strcmp(className, ndShapeCompound::StaticClassName()) == 0)
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
