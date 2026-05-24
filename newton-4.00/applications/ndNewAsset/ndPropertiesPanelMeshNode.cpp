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

ndUndoRedoMeshNode::ndUndoRedoMeshNode(ndAssetEditor* const editor, const ndMesh* const selectedNode)
	:ndUndoRedoCommand(editor, selectedNode)
	,m_copy(editor->m_mesh->CreateClone())
{
	m_selectedNode = m_copy->FindByName(m_selectedNodeName);
}

ndUndoRedoMeshNode* ndUndoRedoMeshNode::GetAsUndoRedoMeshNode() const
{ 
	return (ndUndoRedoMeshNode*)this;
}

bool ndUndoRedoMeshNode::operator!=(const ndUndoRedoCommand& command) const
{
	if (*m_selectedNode == *command.m_selectedNode)
	{
		ndUndoRedoMeshNode* const other = command.GetAsUndoRedoMeshNode();
		if (other)
		{
			bool test = true;
			auto CompareNodes = [this, &test, other](ndMesh* node)
			{
				const ndMesh* const otherNode = other->m_copy->FindByName(node->GetName());
				test = test && (otherNode ? true : false);
				test = test && (*node == *otherNode);
			};
			m_copy->NodeIterator(CompareNodes);
			
			if (test)
			{
				return false;
			}
		}
	}

	return true;
}
	
void ndUndoRedoMeshNode::Undo()
{
	ndSharedPtr<ndRenderSceneNode> visualMesh(ndRenderMeshLoader::CreateRenderSceneMesh(*m_editor->GetRenderer(), *m_copy, ndGetPath(m_editor->GetPath())));

	m_editor->m_initCamera = false;
	m_editor->SetVisualScene(m_copy, visualMesh);
	m_editor->m_currentSelection = m_selectedNode;
	if (m_editor->m_currentSelection)
	{
		ndSharedPtr<ndMeshTransformModifier> modifier(m_editor->m_currentSelection->GetModifier());
		if (modifier)
		{
			if (modifier->m_target)
			{
				m_editor->m_subSelection = ndAssetEditor::m_transformModifier;
			}
			else
			{
				m_editor->m_subSelection = ndAssetEditor::m_none;
			}
		}
	}
}

void ndAssetEditor::ApplyNodeTransform(const ndMatrix& matrix, ndRenderSceneNode* const entNode)
{
	const ndMatrix localMatrix(m_currentSelection->GetMatrix() * matrix.OrthoInverse());
	ndSharedPtr<ndMeshJoint> joint(m_currentSelection->GetJoint());
	if (joint)
	{
		const ndMatrix parentMatrix(m_currentSelection->GetParent()->GetMatrix());
		joint->m_localFrame1 =
			joint->m_localFrame1 * parentMatrix *
			m_currentSelection->GetMatrix().OrthoInverse() *
			matrix * parentMatrix.OrthoInverse();
	}

	m_currentSelection->SetMatrix(matrix);
	entNode->SetTransform(matrix);
	entNode->SetTransform(matrix);
	if (m_transformPivotOnly)
	{
		const ndMatrix geoMatrix(m_currentSelection->GetGeometryMatrix() * localMatrix);
		entNode->SetPrimitiveMatrix(geoMatrix);
		m_currentSelection->SetGeometryMatrix(geoMatrix);

		if (m_currentSelection->GetRigidBody())
		{
			ndMeshBodyKinematic* const body = (ndMeshBodyKinematic*)*m_currentSelection->GetRigidBody();
			ndMeshShapeInstance& shapeInstance = body->m_shapeInstance;
			shapeInstance.m_localMatrix = shapeInstance.m_localMatrix * localMatrix;
		}

		for (ndList<ndSharedPtr<ndMesh>>::ndNode* childPtr = m_currentSelection->GetChildren().GetFirst(); childPtr; childPtr = childPtr->GetNext())
		{
			ndMesh* const child = *childPtr->GetInfo();
			child->SetMatrix(child->GetMatrix() * localMatrix);
		}

		for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* childPtr = entNode->GetChildren().GetFirst(); childPtr; childPtr = childPtr->GetNext())
		{
			ndRenderSceneNode* const child = *childPtr->GetInfo();
			const ndMatrix counterMatrix(child->GetMatrix() * localMatrix);
			child->SetTransform(counterMatrix);
			child->SetTransform(counterMatrix);
		}
	}
}

void ndAssetEditor::ShowPropertiesMeshInfo()
{
	if (ImGui::CollapsingHeader("Mesh node"))
	{
		char nodeName[256];
		snprintf(nodeName, sizeof(nodeName) - 1, "%s", m_currentSelection->GetName().GetStr());
		if (ImGui::InputText("Name", nodeName, sizeof(nodeName) - 1, ImGuiInputTextFlags_EnterReturnsTrue))
		{
			if (strcmp(m_currentSelection->GetName().GetStr(), nodeName))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				ndString newName(nodeName);
				while (m_mesh->FindByName(newName))
				{
					newName += "_1";
				}
				ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());

				m_currentSelection->SetName(newName);
				entNode->m_name = newName;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
			}
		}
		ImGui::Checkbox("parent space transform", &m_parentSpaceTransform);
		ImGui::Checkbox("transform pivot only", &m_transformPivotOnly);

		if (ImGui::Button("add node"))
		{
			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

			ndSharedPtr<ndMesh> childMesh(new ndMesh());
			ndInt32 i = 1;
			ndString name("unnamed");
			while (m_mesh->FindByName(name))
			{
				name += "_";
				name += i;
				i++;
			}
			childMesh->SetName(name);
			m_currentSelection->AddChild(childMesh);

			ndSharedPtr<ndRenderSceneNode> childSceneNode(new ndRenderSceneNode(ndGetIdentityMatrix()));
			childSceneNode->m_name = name;
			ndRenderSceneNode* const parentSceneNode = m_entity->FindByName(m_currentSelection->GetName());
			parentSceneNode->AddChild(childSceneNode);

			m_currentSelection = *childMesh;

			m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
		}

		ImGui::SameLine();
		if (ImGui::Button("delete node"))
		{
			ndTrace(("xxxx1\n"));
		}

		if (m_currentSelection->GetParent())
		{
			if (ImGui::Button("clone node"))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				ndSharedPtr<ndMesh> clone(m_currentSelection->CreateClone());

				ndRenderSceneNode* const selectedVisualMesh = m_entity->FindByName(m_currentSelection->GetName());
				ndAssert(selectedVisualMesh);
				ndSharedPtr<ndRenderSceneNode> cloneVisualMesh(selectedVisualMesh->Clone());

				auto Rename = [this, &cloneVisualMesh](ndMesh* const node)
				{
					ndString name(node->GetName());
					ndRenderSceneNode* const cloneMesh = cloneVisualMesh->FindByName(name);
					while (m_mesh->FindByName(name))
					{
						name += "_";
					}
					node->SetName(name);
					cloneMesh->m_name = name;
				};
				clone->NodeIterator(Rename);
				m_currentSelection->GetParent()->AddChild(clone);
				selectedVisualMesh->GetParent()->AddChild(cloneVisualMesh);

				m_debugDisplayRenderPass->ResetScene();
				m_currentSelection = *clone;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
			}
		}

		if (!m_currentSelection->GetRigidBody())
		{
			if (ImGui::Button("add body"))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				AddRigidBody();
				m_debugDisplayRenderPass->ResetScene();
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
			}
		}
		else
		{
			if (ImGui::Button("delete body"))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				m_currentSelection->SetJoint(ndSharedPtr<ndMeshJoint>(nullptr));
				m_currentSelection->SetRigidBody(ndSharedPtr<ndMeshBody>(nullptr));
				m_debugDisplayRenderPass->ResetScene();
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
			}
		}

		// show node matrix
		{
			ndReal position[3];
			ndMatrix matrix(m_currentSelection->GetMatrix());

			ndReal euler[3];
			ndVector tmp;
			ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

			ImGui::SeparatorText("mode transform");
			if (m_parentSpaceTransform)
			{
				position[0] = ndReal(matrix.m_posit.m_x);
				position[1] = ndReal(matrix.m_posit.m_y);
				position[2] = ndReal(matrix.m_posit.m_z);
				if (ImGui::InputFloat3("posit", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				
					matrix.m_posit = ndVector(position[0], position[1], position[2], ndFloat32(1.0f));
					ApplyNodeTransform(matrix, entNode);
				
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				}

				euler[0] = ndReal(radians[0]);
				euler[1] = ndReal(radians[1]);
				euler[2] = ndReal(radians[2]);
				if (ImGui::InputFloat3("rotation", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				
					ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
					newMatrix.m_posit = matrix.m_posit;
					ApplyNodeTransform(newMatrix, entNode);
				
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				}
			}
			else
			{
				position[0] = ndReal(0.0f);
				position[1] = ndReal(0.0f);
				position[2] = ndReal(0.0f);
				if (ImGui::InputFloat3("posit", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

					const ndVector delta(position[0], position[1], position[2], ndFloat32(0.0f));
					matrix.m_posit += matrix.RotateVector(delta);
					ApplyNodeTransform(matrix, entNode);

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				}

				euler[0] = ndReal(0.0f);
				euler[1] = ndReal(0.0f);
				euler[2] = ndReal(0.0f);
				if (ImGui::InputFloat3("rotation", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

					const ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * matrix);
					ApplyNodeTransform(newMatrix, entNode);

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				}
			}

			if (m_subSelection == m_none)
			{
				if (ImGui::Button("align to target"))
				{
					m_subSelection = m_alignToTarget;
				}
			}
			else if (m_subSelection == m_alignToTarget)
			{
				if (ImGui::Button("pick to target"))
				{
					m_subSelection = m_none;

					ndMatrix selectionMatrix(m_currentSelection->CalculateGlobalMatrix());
					ndMatrix targetMatrix(m_currentSubSelection->CalculateGlobalMatrix());
					ndVector dir(targetMatrix.m_posit - selectionMatrix.m_posit);
					ndMatrix alignMatrix(ndGramSchmidtMatrix(dir));
					ndMatrix localRotation(alignMatrix * selectionMatrix.OrthoInverse() * m_currentSelection->GetMatrix());
					localRotation.m_posit = m_currentSelection->GetMatrix().m_posit;
					
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
					ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
					ndAssert(entNode);
					bool savedState = m_transformPivotOnly;
					m_transformPivotOnly = true;
					ApplyNodeTransform(localRotation, entNode);
					m_transformPivotOnly = savedState;
					m_currentSubSelection = ndWeakPtr<ndMesh>(nullptr);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				}
			}
		}

		// show geometry matrix
		{
			if (*m_currentSelection->GetGeometry())
			{
				ImGui::SeparatorText("geomtry transform");
				if (m_parentSpaceTransform)
				{
					ndMatrix matrix(m_currentSelection->GetGeometryMatrix());
					ndReal position[3];
					position[0] = ndReal(matrix.m_posit.m_x);
					position[1] = ndReal(matrix.m_posit.m_y);
					position[2] = ndReal(matrix.m_posit.m_z);
					if (ImGui::InputFloat3("posit##1", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
					{
						ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
						ndAssert(entNode);
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

						matrix.m_posit.m_x = position[0];
						matrix.m_posit.m_y = position[1];
						matrix.m_posit.m_z = position[2];
						m_currentSelection->SetGeometryMatrix(matrix);
						entNode->SetPrimitiveMatrix(matrix);

						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
					}

					ndReal euler[3];
					ndVector tmp;
					ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));

					euler[0] = ndReal(radians[0]);
					euler[1] = ndReal(radians[1]);
					euler[2] = ndReal(radians[2]);
					if (ImGui::InputFloat3("rotation##1", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
					{
						ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
						ndAssert(entNode);
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

						ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
						newMatrix.m_posit = matrix.m_posit;
						m_currentSelection->SetGeometryMatrix(newMatrix);
						entNode->SetPrimitiveMatrix(matrix);
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
					}
				}
				else
				{
					ndMatrix matrix(m_currentSelection->GetGeometryMatrix());
					ndReal position[3];
					position[0] = ndReal(0.0f);
					position[1] = ndReal(0.0f);
					position[2] = ndReal(0.0f);
					if (ImGui::InputFloat3("posit##1", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
					{
						ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
						ndAssert(entNode);
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

						const ndVector delta(position[0], position[1], position[2], ndFloat32(0.0f));
						matrix.m_posit += matrix.RotateVector(delta);
						m_currentSelection->SetGeometryMatrix(matrix);
						entNode->SetPrimitiveMatrix(matrix);

						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
					};

					ndReal euler[3];
					euler[0] = ndReal(0.0f);
					euler[1] = ndReal(0.0f);
					euler[2] = ndReal(0.0f);
					if (ImGui::InputFloat3("rotation##1", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
					{
						ndRenderSceneNode* const entNode = m_entity->FindByName(m_currentSelection->GetName());
						ndAssert(entNode);
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));

						const ndMatrix newMatrix(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * matrix);
						m_currentSelection->SetGeometryMatrix(newMatrix);
						entNode->SetPrimitiveMatrix(newMatrix);
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
					}
				}
			}
		}

		if (!m_currentSelection->GetRigidBody())
		{ 
			ndSharedPtr<ndMeshTransformModifier> modifier(m_currentSelection->GetModifier());
			char name[256];
			snprintf(name, sizeof(name) - 1, "none");
			if (modifier)
			{
				snprintf(name, sizeof(name) - 1, "%s", modifier->ClassName());
			}

			ImGui::SeparatorText("modifier type");
			if (ImGui::BeginCombo(" ##10", name))
			{
				auto SetDropdownList = [this, &modifier](const char* const name)
				{
					const char* const selectedName = modifier ? modifier->ClassName() : "none";
					bool selected = strcmp(name, selectedName) ? false : true;
					if (ImGui::Selectable(name, selected))
					{
						if (strcmp(name, "none") == 0)
						{
							m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
							m_subSelection = m_none;
							modifier = ndSharedPtr<ndMeshTransformModifier>(nullptr);
							m_currentSelection->SetModifier(modifier);
							m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
						}
						else if (strcmp(name, ndMeshTransformModifierUserDefined::StaticClassName()) == 0)
						{
							m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
							m_subSelection = m_transformModifier;
							modifier = ndSharedPtr<ndMeshTransformModifier>(new ndMeshTransformModifierUserDefined(*m_currentSelection));
							m_currentSelection->SetModifier(modifier);
							m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
						}
						else if (strcmp(name, ndMeshTransformModifierLookAt::StaticClassName()) == 0)
						{
							m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
							m_subSelection = m_transformModifier;
							modifier = ndSharedPtr<ndMeshTransformModifier>(new ndMeshTransformModifierLookAt(*m_currentSelection, nullptr));
							m_currentSelection->SetModifier(modifier);
							m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
						}
						else if (strcmp(name, ndMeshTransformModifierTwoLinksIK::StaticClassName()) == 0)
						{
							m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
							m_subSelection = m_transformModifier;
							modifier = ndSharedPtr<ndMeshTransformModifier>(new ndMeshTransformModifierTwoLinksIK(*m_currentSelection, nullptr));
							m_currentSelection->SetModifier(modifier);
							m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
						}
						else
						{
							ndAssert(0);
						}
					}
				};

				SetDropdownList("none");
				SetDropdownList(ndMeshTransformModifierLookAt::StaticClassName());
				if (m_currentSelection->GetChildren().GetCount())
				{
					SetDropdownList(ndMeshTransformModifierTwoLinksIK::StaticClassName());
				}
				SetDropdownList(ndMeshTransformModifierUserDefined::StaticClassName());

				ImGui::EndCombo();
			}

			char targetName[256];
			snprintf(targetName, sizeof(nodeName) - 1, "no target");
			if (modifier && modifier->m_target)
			{
				snprintf(targetName, sizeof(nodeName) - 1, modifier->m_target->GetName().GetStr());
			}
			ImGui::Text(targetName);

			if (m_subSelection == m_transformModifier)
			{
				if (modifier)
				{
					if (strcmp(modifier->ClassName(), ndMeshTransformModifierUserDefined::StaticClassName()) != 0)
					{
						if (ImGui::Button("exit select target"))
						{
							if (modifier->m_target)
							{
								m_subSelection = m_none;
							}
						}
					}
				}
			}

			if (modifier)
			{
				if (strcmp(modifier->ClassName(), ndMeshTransformModifierTwoLinksIK::StaticClassName()) == 0)
				{
					EditMeshTransformModifierTwoLinksIK();
				}
				else if (strcmp(modifier->ClassName(), ndMeshTransformModifierUserDefined::StaticClassName()) == 0)
				{
					EditMeshTransformModifierUserDefined();
				}
			}
		}
	}
}

void ndAssetEditor::EditMeshTransformModifierUserDefined()
{
	ndMeshTransformModifierUserDefined* const modifier = (ndMeshTransformModifierUserDefined*)*m_currentSelection->GetModifier();

	char constructor[256];
	snprintf(constructor, sizeof(constructor) - 1, "%s", modifier->m_userConstructor.GetStr());
	if (ImGui::InputText("constructor", constructor, sizeof(constructor) - 1, ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
		modifier->m_userConstructor = ndString(constructor);
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
	}
}

void ndAssetEditor::EditMeshTransformModifierTwoLinksIK()
{
	ndMeshTransformModifierTwoLinksIK* const modifier = (ndMeshTransformModifierTwoLinksIK*)*m_currentSelection->GetModifier();

	ndReal sign = ndReal(modifier->m_solutionSign);
	if (ImGui::InputFloat("solution sign", &sign, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
		modifier->m_solutionSign = sign;
		m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
	}

	if (ImGui::BeginCombo("select child Link", modifier->m_childLink->GetName().GetStr()))
	{
		for (ndList<ndSharedPtr<ndMesh>>::ndNode* ptr = m_currentSelection->GetChildren().GetFirst(); ptr; ptr = ptr->GetNext())
		{
			const ndMesh* const child = *ptr->GetInfo();
			bool selected = (child->GetName() == modifier->m_childLink->GetName());
			if (ImGui::Selectable(child->GetName().GetStr(), selected))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				modifier->m_childLink = child;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoMeshNode(this, *m_currentSelection)));
				break;
			}
		}

		ImGui::EndCombo();
	}
}

void ndAssetEditor::AddRigidBody()
{
	ndMeshBodyDynamic* const rigidBody = new ndMeshBodyDynamic(*m_currentSelection);
	rigidBody->m_invMass = ndFloat32(1.0f);
	rigidBody->m_shapeInstance.m_shape = ndSharedPtr<ndMeshCollisionShape>(new ndMeshCollisionShapeNull());
	
	m_currentSelection->SetRigidBody(ndSharedPtr<ndMeshBody>(rigidBody));
	
	const ndMesh* parentBody = m_currentSelection->GetParent();
	if (parentBody && !parentBody->GetRigidBody())
	{
		parentBody = parentBody->GetParent();
	}
	if (parentBody)
	{
		ndMeshJointFix6dof* const joint = new ndMeshJointFix6dof(*m_currentSelection);
		m_currentSelection->SetJoint(ndSharedPtr<ndMeshJoint>(joint));
	}
}