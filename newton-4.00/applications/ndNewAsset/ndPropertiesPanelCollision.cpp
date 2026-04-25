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
		,m_shapeInstance(((ndMeshBodyKinematic*)*editor->m_currentSelection->GetRigidBody())->m_shapeInstance)
	{
		ndAssert(0);
		//ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		//ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());
		//
		//m_scale = body->m_shapeInstance.m_scale;
		//m_localFrame = body->m_shapeInstance.m_localMatrix;
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
				ndAssert(0);
				//bool test = (m_localFrame * other->m_localFrame.OrthoInverse()).TestIdentity();
				//test = test && ((m_scale - other->m_scale).DotProduct(m_scale - other->m_scale).GetScalar() < ndFloat32(-0.001f));
				//if (test)
				//{
				//	return false;
				//}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		ndAssert(0);
		//ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		//ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());
		//
		//body->m_shapeInstance.m_scale = m_scale;
		//body->m_shapeInstance.m_localMatrix = m_localFrame;
	}

	//ndMatrix m_localFrame;
	//ndVector m_scale;
	ndMeshShapeInstance m_shapeInstance;
};

void ndAssetEditor::ShowShapeTransform()
{
	ndSharedPtr<ndMeshBody> body(m_currentSelection->GetRigidBody());
	ndMeshBodyKinematic* const rigidBody = (ndMeshBodyKinematic*)*body;
	ndMeshShapeInstance& shapeInstance = rigidBody->m_shapeInstance;

	if (m_showPreTransform)
	{
		ndAssert(0);
	}
	else
	{
		//ndReal euler[3];
		//euler[0] = ndReal(0.0f);
		//euler[1] = ndReal(0.0f);
		//euler[2] = ndReal(0.0f);
		//if (ImGui::InputFloat3("rel rotation##2", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		//{
		//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		//	ndMatrix localMatrix0(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * joint->m_localFrame0);
		//
		//	ndMatrix globalMatrix(localMatrix0 * m_currentLoopJointSelection->m_childNode->CalculateGlobalMatrix());
		//	ndMatrix localMatrix1(globalMatrix * m_currentLoopJointSelection->m_parentNode->CalculateGlobalMatrix().OrthoInverse());
		//
		//	joint->m_localFrame0 = localMatrix0;
		//	joint->m_localFrame1 = localMatrix1;
		//
		//	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLoopJoint(this)));
		//}

		ImGui::SeparatorText("local transform");
		//ndMatrix matrix(shapeInstance.m_localMatrix);
		ndReal position[3];
		position[0] = ndReal(0.0f);
		position[1] = ndReal(0.0f);
		position[2] = ndReal(0.0f);
		if (ImGui::InputFloat3("position##2", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, m_currentSelection)));
			const ndVector delta(position[0], position[1], position[2], ndFloat32(0.0f));
			shapeInstance.m_localMatrix.m_posit += shapeInstance.m_localMatrix.RotateVector(delta);
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, m_currentSelection)));
		};

		ndReal euler[3];
		ndVector tmp;
		//ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

		euler[0] = ndReal(0.0f);
		euler[1] = ndReal(0.0f);
		euler[2] = ndReal(0.0f);
		if (ImGui::InputFloat3("rotation##2", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, m_currentSelection)));
			shapeInstance.m_localMatrix = ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * shapeInstance.m_localMatrix;
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
						ndAssert(0);
						//instance->Serialize(&shapeInstance);
						//GetDebugDisplay()->RebuildDebugCollision();
						//m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeChange(this, m_currentSelection)));
					};

					//if (strcmp(name, className))
					//{
					//	if (strcmp(name, ndShapeBox::StaticClassName()) == 0)
					//	{
					//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeChange(this, m_currentSelection)));
					//		ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionBox());
					//		InitNewShape(instance);
					//	}
					//	else if (strcmp(name, ndShapeSphere::StaticClassName()) == 0)
					//	{
					//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeChange(this, m_currentSelection)));
					//		ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionSphere());
					//		InitNewShape(instance);
					//	}
					//	else if (strcmp(name, ndShapeCapsule::StaticClassName()) == 0)
					//	{
					//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeChange(this, m_currentSelection)));
					//		ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionCapsule());
					//		InitNewShape(instance);
					//	}
					//	else if (strcmp(name, ndShapeCylinder::StaticClassName()) == 0)
					//	{
					//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeChange(this, m_currentSelection)));
					//		ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionCylinder());
					//		InitNewShape(instance);
					//	}
					//	else if (strcmp(name, ndShapeConvexHull::StaticClassName()) == 0)
					//	{
					//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeChange(this, m_currentSelection)));
					//		ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionConvex());
					//		InitNewShape(instance);
					//	}
					//	else if (strcmp(name, ndShapeChamferCylinder::StaticClassName()) == 0)
					//	{
					//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeChange(this, m_currentSelection)));
					//		ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionChamferCylinder());
					//		InitNewShape(instance);
					//	}
					//	else if (strcmp(name, ndShapeNull::StaticClassName()) == 0)
					//	{
					//		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShapeChange(this, m_currentSelection)));
					//		ndSharedPtr<ndShapeInstance> instance(m_currentSelection->CreateCollisionNull());
					//		InitNewShape(instance);
					//	}
					//	else
					//	{
					//		ndAssert(0);
					//	}
					//}
					ndAssert(0);
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

#if 0
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
			if (ImGui::InputFloat("mass", &size, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{

			}
		}
		else
		{
			ndAssert(0);
		}
#endif

		ShowShapeTransform();
		const ndString& contructor = shapeInstance.m_shape->m_constructor;

		if (strcmp(className, ndShapeNull::StaticClassName()) == 0)
		{
			// null shape does nothing
		}
		else if (strcmp(contructor.GetStr(), ndShapeBox::StaticClassName()) == 0)
		{
			ShowCollisionBox();
		}
		else
		{
			//ndAssert(0);
		}
	}
}

void ndAssetEditor::ShowCollisionBox()
{
	ndSharedPtr<ndMeshBody> body(m_currentSelection->GetRigidBody());
	ndMeshBodyKinematic* const rigidBody = (ndMeshBodyKinematic*)*body;
	ndMeshShapeInstance& shapeInstance = rigidBody->m_shapeInstance;

	ndReal value;
	ndMeshCollisionShapeBox* const subJoint = (ndMeshCollisionShapeBox*)*shapeInstance.m_shape;

	value = subJoint->m_x;
	if (ImGui::InputFloat("x##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, m_currentSelection)));
		subJoint->m_x = value;
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, m_currentSelection)));
	}
	value = subJoint->m_y;
	if (ImGui::InputFloat("y##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, m_currentSelection)));
		subJoint->m_y = value;
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, m_currentSelection)));
	}
	value = subJoint->m_z;
	if (ImGui::InputFloat("z##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, m_currentSelection)));
		subJoint->m_z = value;
		GetDebugDisplay()->RebuildDebugCollision();
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoShape(this, m_currentSelection)));
	}
}