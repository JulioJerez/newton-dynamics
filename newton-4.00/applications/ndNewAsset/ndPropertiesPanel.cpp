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

class ndUndoRedoName : public ndUndoRedoCommand
{
	public:
	ndUndoRedoName(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
		,m_name(mesh->GetName())
	{
	}

	virtual class ndUndoRedoName* GetAsUndoRedoName() const override
	{ 
		return (ndUndoRedoName*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoName* const other = command.GetAsUndoRedoName();
			if (other)
			{
				if (m_name == other->m_name)
				{
					return false;
				}
			}
		}

		return true;
	}
	
	virtual void Undo() override
	{
		ndRenderSceneNode* const entNode = GetSceneNode();
		ndAssert(entNode);
		m_mesh->SetName(m_name);
		entNode->m_name = m_name;
	}
	ndString m_name;
};

class ndUndoRedoMass : public ndUndoRedoCommand
{
	public:
	ndUndoRedoMass(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());

		m_invMass = body->m_invMass.m_w;
	}

	virtual class ndUndoRedoMass* GetAsUndoRedoMass() const override
	{
		return (ndUndoRedoMass*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoMass* const other = command.GetAsUndoRedoMass();
			if (other)
			{
				if (m_invMass == other->m_invMass)
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

		body->m_invMass = m_invMass;
	}

	ndFloat32 m_invMass;
};

class ndUndoRedoLinearStep : public ndUndoRedoCommand
{
	public:
	ndUndoRedoLinearStep(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());

		m_linearStep = body->m_maxLinearStep;
	}

	virtual class ndUndoRedoLinearStep* GetAsUndoRedoLinearStep() const override
	{
		return (ndUndoRedoLinearStep*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoLinearStep* const other = command.GetAsUndoRedoLinearStep();
			if (other)
			{
				if (m_linearStep == other->m_linearStep)
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

		body->m_maxLinearStep = m_linearStep;
	}

	ndFloat32 m_linearStep;
};

class ndUndoRedoAngleStep : public ndUndoRedoCommand
{
	public:
	ndUndoRedoAngleStep(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());

		m_angleStep = body->m_maxAngleStep;
	}

	virtual class ndUndoRedoAngleStep* GetAsUndoRedoAngleStep() const override
	{
		return (ndUndoRedoAngleStep*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoAngleStep* const other = command.GetAsUndoRedoAngleStep();
			if (other)
			{
				if (m_angleStep == other->m_angleStep)
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

		body->m_maxAngleStep = m_angleStep;
	}

	ndFloat32 m_angleStep;
};

class ndUndoRedoLinearDamp : public ndUndoRedoCommand
{
	public:
	ndUndoRedoLinearDamp(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndAssert(m_mesh->GetRigidBody()->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const body = ((ndMeshBodyDynamic*)*m_mesh->GetRigidBody());

		m_linearDamp = body->m_intrinsicDamping.m_w;
	}

	virtual class ndUndoRedoLinearDamp* GetAsUndoRedoLinearDamp() const override
	{
		return (ndUndoRedoLinearDamp*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoLinearDamp* const other = command.GetAsUndoRedoLinearDamp();
			if (other)
			{
				if (m_linearDamp == other->m_linearDamp)
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

		body->m_intrinsicDamping.m_w = m_linearDamp;
	}

	ndFloat32 m_linearDamp;
};

class ndUndoRedoTransform : public ndUndoRedoCommand
{
	public:
	ndUndoRedoTransform(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
		,m_matrix(mesh->GetMatrix())
	{
	}

	virtual class ndUndoRedoTransform* GetAsUndoRedoTransform() const override
	{
		return (ndUndoRedoTransform*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoTransform* const other = command.GetAsUndoRedoTransform();
			if (other)
			{
				ndMatrix matrix(m_matrix * other->m_matrix.OrthoInverse());
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
		ndRenderSceneNode* const entNode = GetSceneNode();
		ndAssert(entNode);

		m_mesh->SetMatrix(m_matrix);
		entNode->SetTransform(m_matrix);
		entNode->SetTransform(m_matrix);
	}

	ndMatrix m_matrix;
};


class ndUndoRedoGeometryTransform : public ndUndoRedoCommand
{
	public:
	ndUndoRedoGeometryTransform(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
		,m_matrix(mesh->GetGeometryMatrix())
	{
	}

	virtual class ndUndoRedoGeometryTransform* GetAsUndoRedoGeometryTransform() const override
	{
		return (ndUndoRedoGeometryTransform*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			ndUndoRedoGeometryTransform* const other = command.GetAsUndoRedoGeometryTransform();
			if (other)
			{
				ndMatrix matrix(m_matrix * other->m_matrix.OrthoInverse());
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
		ndRenderSceneNode* const entNode = GetSceneNode();
		ndAssert(entNode);
		m_mesh->SetGeometryMatrix(m_matrix);
		entNode->SetPrimitiveMatrix(m_matrix);
	}

	ndMatrix m_matrix;
};

void ndAssetEditor::ShowPropertiesMeshInfo()
{
	if (ImGui::CollapsingHeader("mesh node"))
	{
		char nodeName[256];
		snprintf(nodeName, sizeof(nodeName) - 1, "%s", m_currentSelection->GetName().GetStr());
		if (ImGui::InputText("Name", nodeName, sizeof(nodeName) - 1, ImGuiInputTextFlags_EnterReturnsTrue))
		{
			if (strcmp(m_currentSelection->GetName().GetStr(), nodeName))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoName(this, m_currentSelection)));
				ndString newName(nodeName);
				while (m_mesh->FindByName(newName))
				{
					newName += "_1";
				}
				ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());

				m_currentSelection->SetName(newName);
				entNode->m_name = newName;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoName(this, m_currentSelection)));
			}
		}

		// show node matrix
		{
			ImGui::SeparatorText("Transform");

			ndReal position[3];
			ndMatrix matrix(m_currentSelection->GetMatrix());

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			if (m_relativeTransformProperties)
			{
				position[0] = ndReal(0.0f);
				position[1] = ndReal(0.0f);
				position[2] = ndReal(0.0f);

				// this is really nice but creates lots of issues with undo/redo
				//if (ImGui::DragFloat3("position", position, 0.01f))
				if (ImGui::InputFloat3("rel posit", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoTransform(this, m_currentSelection)));

					const ndVector delta(position[0], position[1], position[2], ndFloat32(1.0f));
					matrix.m_posit += matrix.RotateVector(delta);
					m_currentSelection->SetMatrix(matrix);
					entNode->SetTransform(matrix);
					entNode->SetTransform(matrix);

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoTransform(this, m_currentSelection)));
				}

				euler[0] = ndReal(0.0f);
				euler[1] = ndReal(0.0f);
				euler[2] = ndReal(0.0f);
				//if (ImGui::DragFloat3("rel rotation", euler))
				if (ImGui::InputFloat3("rel rotation", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoTransform(this, m_currentSelection)));

					const ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * matrix);
					m_currentSelection->SetMatrix(newMatrix);
					entNode->SetTransform(newMatrix);
					entNode->SetTransform(newMatrix);

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoTransform(this, m_currentSelection)));
				};
			}
			else
			{
				position[0] = ndReal(matrix.m_posit.m_x);
				position[1] = ndReal(matrix.m_posit.m_y);
				position[2] = ndReal(matrix.m_posit.m_z);
				//if (ImGui::DragFloat3("posit", position, 0.01f))
				if (ImGui::InputFloat3("posit", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoTransform(this, m_currentSelection)));

					matrix.m_posit = ndVector (position[0], position[1], position[2], ndFloat32(1.0f));
					m_currentSelection->SetMatrix(matrix);
					entNode->SetTransform(matrix);
					entNode->SetTransform(matrix);

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoTransform(this, m_currentSelection)));
				}

				euler[0] = ndReal(radians[0]);
				euler[1] = ndReal(radians[1]);
				euler[2] = ndReal(radians[2]);
				//if (ImGui::DragFloat3("rotation", euler))
				if (ImGui::InputFloat3("rotation", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoTransform(this, m_currentSelection)));

					ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
					newMatrix.m_posit = matrix.m_posit;
					m_currentSelection->SetMatrix(newMatrix);
					entNode->SetTransform(newMatrix);
					entNode->SetTransform(newMatrix);

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoTransform(this, m_currentSelection)));
				};
			}
		}

		// show geometry node matrix
		if (*m_currentSelection->GetMesh())
		{
			ImGui::SeparatorText("geomtry transform");
			ndMatrix matrix(m_currentSelection->GetGeometryMatrix());
			ndReal position[3];
			position[0] = ndReal(matrix.m_posit.m_x);
			position[1] = ndReal(matrix.m_posit.m_y);
			position[2] = ndReal(matrix.m_posit.m_z);
			//if (ImGui::DragFloat3("position##1", position))
			if (ImGui::InputFloat3("posit##1", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
				ndAssert(entNode);
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoGeometryTransform(this, m_currentSelection)));

				matrix.m_posit.m_x = position[0];
				matrix.m_posit.m_y = position[1];
				matrix.m_posit.m_z = position[2];
				m_currentSelection->SetGeometryMatrix(matrix);
				entNode->SetPrimitiveMatrix(matrix);

				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoGeometryTransform(this, m_currentSelection)));
			};

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			euler[0] = ndReal(radians[0]);
			euler[1] = ndReal(radians[1]);
			euler[2] = ndReal(radians[2]);
			//if (ImGui::DragFloat3("rotation##1", euler))
			if (ImGui::InputFloat3("rotation##1", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
				ndAssert(entNode);
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoGeometryTransform(this, m_currentSelection)));

				ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
				newMatrix.m_posit = matrix.m_posit;
				m_currentSelection->SetGeometryMatrix(newMatrix);
				entNode->SetPrimitiveMatrix(matrix);
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoGeometryTransform(this, m_currentSelection)));
			};
		}
	}
}

void ndAssetEditor::ShowPropertiesRigidBodyInfo()
{
	if (ImGui::CollapsingHeader("Rigid body"))
	{
		ndSharedPtr<ndMeshBody> body (m_currentSelection->GetRigidBody());
		ndAssert(body->m_classConstructor == ndBodyDynamic::StaticClassName());
		ndMeshBodyDynamic* const rigidBody = (ndMeshBodyDynamic*)*body;

		// body mass
		{
			ndReal scalar = ndReal (ndFloat32 (1.0f) / rigidBody->m_invMass.m_w);
			//if (ImGui::DragFloat("mass", &scalar))
			if (ImGui::InputFloat("mass", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMass(this, m_currentSelection)));
				scalar = ndMax(scalar, ndReal(0.001f));
				rigidBody->m_invMass.m_w = ndFloat32(1.0f) / scalar;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMass(this, m_currentSelection)));
			};
		}

		// body max angular integration step 
		{
			ndReal scalar = ndReal(rigidBody->m_maxAngleStep);
			//if (ImGui::DragFloat("angle step", &scalar))
			if (ImGui::InputFloat("angle step", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoAngleStep(this, m_currentSelection)));
				scalar = ndClamp(scalar, ndReal(10.0f), ndReal(180.0f));
				rigidBody->m_maxAngleStep = scalar;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoAngleStep(this, m_currentSelection)));
			};
		}

		// body max linear integration step 
		{
			ndReal scalar = ndReal(rigidBody->m_maxLinearStep);
			//if (ImGui::DragFloat("linear step", &scalar))
			if (ImGui::InputFloat("linear step", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLinearStep(this, m_currentSelection)));
				scalar = ndClamp(scalar, ndReal(0.1f), ndReal(30.0f));
				rigidBody->m_maxLinearStep = scalar;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLinearStep(this, m_currentSelection)));
			};
		}

		// body intrinsic linear damp 
		{
			ndReal scalar = ndReal(rigidBody->m_intrinsicDamping.m_w);
			//if (ImGui::DragFloat("linear Damp", &scalar))
			if (ImGui::InputFloat("linear damp", &scalar, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLinearDamp(this, m_currentSelection)));
				scalar = ndClamp(scalar, ndReal(0.0f), ndReal(1.0f));
				rigidBody->m_intrinsicDamping.m_w = scalar;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoLinearDamp(this, m_currentSelection)));
			};
		}

		// body center of mass
		{
			ndVector vector(rigidBody->m_localCentreOfMass);
			ndReal real[3];
			real[0] = ndReal(vector.m_x);
			real[1] = ndReal(vector.m_y);
			real[2] = ndReal(vector.m_z);
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
			real[0] = ndReal(vector.m_x);
			real[1] = ndReal(vector.m_y);
			real[2] = ndReal(vector.m_z);
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
			real[0] = ndReal(vector.m_x);
			real[1] = ndReal(vector.m_y);
			real[2] = ndReal(vector.m_z);
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
			real[0] = ndReal(vector.m_x);
			real[1] = ndReal(vector.m_y);
			real[2] = ndReal(vector.m_z);
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

			euler[0] = ndReal(radians[0]);
			euler[1] = ndReal(radians[1]);
			euler[2] = ndReal(radians[2]);
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

void ndAssetEditor::ShowPropertiesPanel()
{
	ImGui::Begin("Properties Panel");

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