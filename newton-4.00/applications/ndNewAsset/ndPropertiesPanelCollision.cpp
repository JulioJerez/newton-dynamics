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
	ndUndoRedoShape(ndAssetEditor* const editor, const ndMesh* const selectedNode)
		:ndUndoRedoCommand(editor, selectedNode)
		,m_shapeInstance(((ndMeshBodyKinematic*)*editor->m_currentSelection->GetRigidBody())->m_shapeInstance)
	{
	}

	virtual ndUndoRedoShape* GetAsUndoRedoShape() const override
	{
		return (ndUndoRedoShape*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_selectedNode == *command.m_selectedNode)
		{
			const ndUndoRedoShape* const other = command.GetAsUndoRedoShape();
			if (other)
			{
				bool test = (m_shapeInstance == other->m_shapeInstance);
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
		ndAssert(m_selectedNode->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_selectedNode->GetRigidBody());
		body->m_shapeInstance = m_shapeInstance;
		m_editor->GetDebugDisplay()->RebuildDebugCollision();
	}

	ndMeshShapeInstance m_shapeInstance;
};

void ndAssetEditor::EditShapeTransform()
{
	ndSharedPtr<ndMeshBody> body(m_currentSelection->GetRigidBody());
	ndMeshBodyKinematic* const rigidBody = (ndMeshBodyKinematic*)*body;
	ndMeshShapeInstance& shapeInstance = rigidBody->m_shapeInstance;

	if (m_showTransformValues)
	{
		ImGui::SeparatorText("local transform");
		ndReal position[3];
		position[0] = ndReal(shapeInstance.m_localMatrix.m_posit.m_x);
		position[1] = ndReal(shapeInstance.m_localMatrix.m_posit.m_y);
		position[2] = ndReal(shapeInstance.m_localMatrix.m_posit.m_z);
		if (ImGui::InputFloat3("position##2", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
			const ndVector delta(position[0], position[1], position[2], ndFloat32(0.0f));
			shapeInstance.m_localMatrix.m_posit = delta;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		};

		ndReal euler[3];
		ndVector tmp;
		const ndVector angle(shapeInstance.m_localMatrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

		euler[0] = ndReal(angle.m_x);
		euler[1] = ndReal(angle.m_y);
		euler[2] = ndReal(angle.m_z);
		if (ImGui::InputFloat3("rotation##2", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
			ndMatrix matrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
			matrix.m_posit = shapeInstance.m_localMatrix.m_posit;
			shapeInstance.m_localMatrix = matrix;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		};

		ndVector vector(shapeInstance.m_scale);
		ndReal real[3];
		real[0] = ndReal(vector.m_x);
		real[1] = ndReal(vector.m_y);
		real[2] = ndReal(vector.m_z);
		if (ImGui::InputFloat3("scale##1", real, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
			vector.m_x = ndMax(real[0], ndReal(0.001f));
			vector.m_y = ndMax(real[1], ndReal(0.001f));
			vector.m_z = ndMax(real[2], ndReal(0.001f));
			shapeInstance.m_scale = vector;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		};

	}
	else
	{
		ImGui::SeparatorText("local transform");
		ndReal position[3];
		position[0] = ndReal(0.0f);
		position[1] = ndReal(0.0f);
		position[2] = ndReal(0.0f);
		if (ImGui::InputFloat3("position##2", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
			const ndVector delta(position[0], position[1], position[2], ndFloat32(0.0f));
			shapeInstance.m_localMatrix.m_posit += shapeInstance.m_localMatrix.RotateVector(delta);
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		};

		ndReal euler[3];
		ndVector tmp;

		euler[0] = ndReal(0.0f);
		euler[1] = ndReal(0.0f);
		euler[2] = ndReal(0.0f);
		if (ImGui::InputFloat3("rotation##2", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
			shapeInstance.m_localMatrix = ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * shapeInstance.m_localMatrix;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		};

		ndVector vector(shapeInstance.m_scale);
		ndReal real[3];
		real[0] = ndReal(vector.m_x);
		real[1] = ndReal(vector.m_y);
		real[2] = ndReal(vector.m_z);
		if (ImGui::InputFloat3("scale##1", real, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
			vector.m_x = ndMax(real[0], ndReal(0.001f));
			vector.m_y = ndMax(real[1], ndReal(0.001f));
			vector.m_z = ndMax(real[2], ndReal(0.001f));
			shapeInstance.m_scale = vector;
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		};
	}
}

void ndAssetEditor::EditCollisionBox()
{
	ndSharedPtr<ndMeshBody> body(m_currentSelection->GetRigidBody());
	ndMeshBodyKinematic* const rigidBody = (ndMeshBodyKinematic*)*body;
	ndMeshShapeInstance& shapeInstance = rigidBody->m_shapeInstance;

	ndReal value;
	ndMeshCollisionShapeBox* const subJoint = (ndMeshCollisionShapeBox*)*shapeInstance.m_shape;

	value = ndReal(subJoint->m_x);
	if (ImGui::InputFloat("x##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		subJoint->m_x = ndMax (value, ndReal(0.01f));
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	}
	value = ndReal(subJoint->m_y);
	if (ImGui::InputFloat("y##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		subJoint->m_y = ndMax(value, ndReal(0.01f));
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	}
	value = ndReal(subJoint->m_z);
	if (ImGui::InputFloat("z##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		subJoint->m_z = ndMax(value, ndReal(0.01f));
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	}
}

void ndAssetEditor::EditCollisionSphere()
{
	ndSharedPtr<ndMeshBody> body(m_currentSelection->GetRigidBody());
	ndMeshBodyKinematic* const rigidBody = (ndMeshBodyKinematic*)*body;
	ndMeshShapeInstance& shapeInstance = rigidBody->m_shapeInstance;

	ndReal value;
	ndMeshCollisionShapeSphere* const subJoint = (ndMeshCollisionShapeSphere*)*shapeInstance.m_shape;

	value = ndReal(subJoint->m_radius);
	if (ImGui::InputFloat("radios", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		subJoint->m_radius = ndMax(value, ndReal(0.01f));
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	}
}

void ndAssetEditor::EditCollisionCapsule()
{
	ndSharedPtr<ndMeshBody> body(m_currentSelection->GetRigidBody());
	ndMeshBodyKinematic* const rigidBody = (ndMeshBodyKinematic*)*body;
	ndMeshShapeInstance& shapeInstance = rigidBody->m_shapeInstance;

	ndReal value;
	ndMeshCollisionShapeCapsule* const subJoint = (ndMeshCollisionShapeCapsule*)*shapeInstance.m_shape;

	value = ndReal(subJoint->m_radius0);
	if (ImGui::InputFloat("radios0", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		subJoint->m_radius0 = ndMax(value, ndReal(0.01f));
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	}
	value = ndReal(subJoint->m_radius1);
	if (ImGui::InputFloat("radios1", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		subJoint->m_radius1 = ndMax(value, ndReal(0.01f));
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	}
	value = ndReal(subJoint->m_height);
	if (ImGui::InputFloat("height", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		subJoint->m_height = ndMax(value, ndReal(0.01f));
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	}
}

void ndAssetEditor::EditCollisionCylinder()
{
	ndSharedPtr<ndMeshBody> body(m_currentSelection->GetRigidBody());
	ndMeshBodyKinematic* const rigidBody = (ndMeshBodyKinematic*)*body;
	ndMeshShapeInstance& shapeInstance = rigidBody->m_shapeInstance;

	ndReal value;
	ndMeshCollisionShapeCylinder* const subJoint = (ndMeshCollisionShapeCylinder*)*shapeInstance.m_shape;

	value = ndReal(subJoint->m_radius0);
	if (ImGui::InputFloat("radios0", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		subJoint->m_radius0 = ndMax(value, ndReal(0.01f));
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	}
	value = ndReal(subJoint->m_radius1);
	if (ImGui::InputFloat("radios1", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		subJoint->m_radius1 = ndMax(value, ndReal(0.01f));
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	}
	value = ndReal(subJoint->m_height);
	if (ImGui::InputFloat("height", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		subJoint->m_height = ndMax(value, ndReal(0.01f));
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	}
}

void ndAssetEditor::EditCollisionChamferCylinder()
{
	ndSharedPtr<ndMeshBody> body(m_currentSelection->GetRigidBody());
	ndMeshBodyKinematic* const rigidBody = (ndMeshBodyKinematic*)*body;
	ndMeshShapeInstance& shapeInstance = rigidBody->m_shapeInstance;

	ndReal value;
	ndMeshCollisionShapeChamferCylinder* const subJoint = (ndMeshCollisionShapeChamferCylinder*)*shapeInstance.m_shape;

	value = ndReal(subJoint->m_radius);
	if (ImGui::InputFloat("radios", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		subJoint->m_radius = ndMax(value, ndReal(0.01f));
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	}
	value = ndReal(subJoint->m_height);
	if (ImGui::InputFloat("height", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		subJoint->m_height = ndMax(value, ndReal(0.01f));
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	}
}

void ndAssetEditor::EditCollisionWheel()
{
	// do nothing
}

void ndAssetEditor::EditCollisionConvexHull()
{
	ndSharedPtr<ndMeshBody> body(m_currentSelection->GetRigidBody());
	ndMeshBodyKinematic* const rigidBody = (ndMeshBodyKinematic*)*body;
	ndMeshShapeInstance& shapeInstance = rigidBody->m_shapeInstance;

	//ndReal value;
	ndMeshCollisionShapeConvexHull* const subJoint = (ndMeshCollisionShapeConvexHull*)*shapeInstance.m_shape;

	if (ImGui::Button("Recalculate"))
	{
		ndTrace(("To Do\n"));
	}

	ndReal value = ndReal(subJoint->m_tolarence);
	if (ImGui::InputFloat("radios", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		subJoint->m_tolarence = ndMax(value, ndReal(0.1f));
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	}

	ndInt32 points = subJoint->m_maxPointCount;
	if (ImGui::InputInt("max points", &points, 1, ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		subJoint->m_maxPointCount = ndMax(points, 16);
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	}
}

void ndAssetEditor::EditCollisionCompound()
{
	ndSharedPtr<ndMeshBody> body(m_currentSelection->GetRigidBody());
	ndMeshBodyKinematic* const rigidBody = (ndMeshBodyKinematic*)*body;
	ndMeshShapeInstance& shapeInstance = rigidBody->m_shapeInstance;

	//ndReal value;
	ndMeshCollisionShapeCompound* const subJoint = (ndMeshCollisionShapeCompound*)*shapeInstance.m_shape;

	if (ImGui::Button("Recalculate"))
	{
		ndTrace(("To Do\n"));
	}

	ndInt32 count = subJoint->m_subShapes.GetCount();
	if (ImGui::InputInt("num of subshapes", &count, 0, ImGuiInputTextFlags_EnterReturnsTrue))
	{
		ndTrace(("To Do\n"));
		//m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
		//subJoint->m_maxPointCount = ndMax(points, 16);
		//GetDebugDisplay()->RebuildDebugCollision();
		//m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	}

	//ndReal value = subJoint->m_tolarence;
	//if (ImGui::InputFloat("radios", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	//{
	//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	//	subJoint->m_tolarence = ndMax(value, ndReal(0.1f));
	//	GetDebugDisplay()->RebuildDebugCollision();
	//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	//}
}

void ndAssetEditor::ShowPropertiesCollisionInfo()
{
	if (ImGui::CollapsingHeader("Collision shape"))
	{
		ndSharedPtr<ndMeshBody> body(m_currentSelection->GetRigidBody());
		ndMeshBodyKinematic* const rigidBody = (ndMeshBodyKinematic*)*body;
		ndMeshShapeInstance& shapeInstance = rigidBody->m_shapeInstance;

		if (ImGui::BeginCombo("shapes", shapeInstance.m_shape->m_constructor.GetStr()))
		{
			const char* const className = shapeInstance.m_shape->m_constructor.GetStr();
			auto SetDropdownList = [this, rigidBody, &shapeInstance, &className](const char* const name)
			{
				bool selected = strcmp(name, className) ? false : true;
				if (ImGui::Selectable(name, selected))
				{
					auto InitNewShape = [this, rigidBody, &shapeInstance](ndSharedPtr<ndShapeInstance>& instance)
					{
						ndMeshShapeInstance newInstance(**instance);
						shapeInstance = newInstance;
						rigidBody->m_shapeInstance = shapeInstance;
						GetDebugDisplay()->RebuildDebugCollision();
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
					};

					if (strcmp(name, ndShapeNull::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
						ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionNull());
						InitNewShape(instance);
					}
					else if (strcmp(name, ndShapeBox::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
						ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionBox());
						InitNewShape(instance);
					}
					else if (strcmp(name, ndShapeSphere::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
						ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionSphere());
						InitNewShape(instance);
					}
					else if (strcmp(name, ndShapeCapsule::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
						ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionCapsule());
						InitNewShape(instance);
					}
					else if (strcmp(name, ndShapeCylinder::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
						ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionCylinder());
						InitNewShape(instance);
					}
					else if (strcmp(name, ndShapeChamferCylinder::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
						ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionChamferCylinder());
						InitNewShape(instance);
					}
					else if (strcmp(name, ndShapeWheel::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
						ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionWheel());
						InitNewShape(instance);
					}
					else if (strcmp(name, ndShapeConvexHull::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
						ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionConvex());
						InitNewShape(instance);
					}
					else if (strcmp(name, ndShapeCompound::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
						ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionFromChildren());
						InitNewShape(instance);
					}
					else
					{
						ndAssert(0);
					}
				}
			};
			SetDropdownList(ndShapeNull::StaticClassName());
			SetDropdownList(ndShapeBox::StaticClassName());
			SetDropdownList(ndShapeSphere::StaticClassName());
			SetDropdownList(ndShapeCapsule::StaticClassName());
			SetDropdownList(ndShapeCylinder::StaticClassName());
			SetDropdownList(ndShapeChamferCylinder::StaticClassName());
			SetDropdownList(ndShapeWheel::StaticClassName());
			SetDropdownList(ndShapeConvexHull::StaticClassName());
			SetDropdownList(ndShapeCompound::StaticClassName());

			ImGui::EndCombo();
		}

		MakeVisualGeometry();
		EditShapeTransform();
		const ndString& contructor = shapeInstance.m_shape->m_constructor;
		//const char* const className = shapeInstance.m_shape->m_constructor.GetStr();

		if (strcmp(contructor.GetStr(), ndShapeNull::StaticClassName()) == 0)
		{
			// null shape does nothing
		}
		else if (strcmp(contructor.GetStr(), ndShapeBox::StaticClassName()) == 0)
		{
			EditCollisionBox();
		}
		else if (strcmp(contructor.GetStr(), ndShapeSphere::StaticClassName()) == 0)
		{
			EditCollisionSphere();
		}
		else if (strcmp(contructor.GetStr(), ndShapeCapsule::StaticClassName()) == 0)
		{
			EditCollisionCapsule();
		}
		else if (strcmp(contructor.GetStr(), ndShapeCylinder::StaticClassName()) == 0)
		{
			EditCollisionCylinder();
		}
		else if (strcmp(contructor.GetStr(), ndShapeChamferCylinder::StaticClassName()) == 0)
		{
			EditCollisionChamferCylinder();
		}
		else if (strcmp(contructor.GetStr(), ndShapeWheel::StaticClassName()) == 0)
		{
			EditCollisionWheel();
		}
		else if (strcmp(contructor.GetStr(), ndShapeConvexHull::StaticClassName()) == 0)
		{
			EditCollisionConvexHull();
		}
		else if (strcmp(contructor.GetStr(), ndShapeCompound::StaticClassName()) == 0)
		{
			EditCollisionCompound();
		}
		else
		{
			//ndAssert(0);
		}
	}
}

void ndAssetEditor::MakeVisualGeometry()
{
	ndSharedPtr<ndMeshBody> body(m_currentSelection->GetRigidBody());
	ndAssert(body->m_classConstructor == ndBodyDynamic::StaticClassName());
	ndMeshBodyDynamic* const rigidBody = (ndMeshBodyDynamic*)*body;
	const ndMeshShapeInstance& shapeInstance = rigidBody->m_shapeInstance;
	if (strcmp (shapeInstance.m_shape->m_constructor.GetStr(), ndShapeNull::StaticClassName()) == 0)
	{
		return;
	}

	ImGui::Separator();
	if (ImGui::Button("build visual mesh"))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));

		ndSharedPtr<ndShapeInstance> instance (shapeInstance.CreateObject());
		ndSharedPtr<ndMesh> tmpMesh(new ndMesh(**instance));
		m_currentSelection->SetGeometry(tmpMesh->GetGeometry());
		m_currentSelection->SetGeometryMatrix(tmpMesh->GetGeometryMatrix());

		ndRenderSceneNode* const visualMesh = m_entity->FindByName(m_currentSelection->GetName());
		ndSharedPtr<ndRenderSceneNode> tmpVisualMesh(ndRenderMeshLoader::CreateRenderSceneMesh(*m_renderer, *tmpMesh, ndGetWorkingFileName("")));
		visualMesh->SetPrimitive(tmpVisualMesh->GetPrimitive());
		visualMesh->SetPrimitiveMatrix(tmpVisualMesh->GetPrimitiveMatrix());
		
		GetDebugDisplay()->RebuildVisualDebugMesh();

		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, *m_currentSelection)));
	}
}