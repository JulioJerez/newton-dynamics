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

class ndUndoRedoCollidingPairs : public ndUndoRedoCommand
{
	public:
	ndUndoRedoCollidingPairs(ndAssetEditor* const editor)
		:ndUndoRedoCommand(editor, *editor->GetMesh()->GetCollingPairs()->GetSharedPtr())
	{
		ndCollidingPairs* const collidingPairs = m_selectedNode->GetAsCollidingPairs();
		ndAssert(collidingPairs);
		for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* ptr = collidingPairs->m_collidingPairs.GetFirst(); ptr; ptr = ptr->GetNext())
		{
			m_collidingPairs.Append(ptr->GetInfo());
		}
	}

	virtual class ndUndoRedoCollidingPairs* GetAsUndoRedoCollidingPairs() const override
	{
		return (ndUndoRedoCollidingPairs*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_selectedNode == *command.m_selectedNode)
		{
			ndUndoRedoCollidingPairs* const other = command.GetAsUndoRedoCollidingPairs();
			if (other)
			{
				bool test = (m_collidingPairs.GetCount() == other->m_collidingPairs.GetCount());
				if (test)
				{
					ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* otherPtr = other->m_collidingPairs.GetFirst();
					for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* ptr = m_collidingPairs.GetFirst(); test && ptr; ptr = ptr->GetNext())
					{
						test = test && (ptr->GetInfo() == otherPtr->GetInfo());
						otherPtr = otherPtr->GetNext();
					}
				}
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
		ndCollidingPairs* const collidingPairs = m_selectedNode->GetCollingPairs();
		collidingPairs->m_collidingPairs.RemoveAll();
		for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* ptr = m_collidingPairs.GetFirst(); ptr; ptr = ptr->GetNext())
		{
			collidingPairs->m_collidingPairs.Append(ptr->GetInfo());
		}
	}

	ndList<ndSharedPtr<ndMeshCollidingPair>> m_collidingPairs;
};

void ndAssetEditor::ShowPropertiesCollidingPairs()
{
	if (ImGui::CollapsingHeader("Colliding Pairs"))
	{
		ndCollidingPairs* const collidingPairs = m_currentSelection->GetAsCollidingPairs();
		if (ImGui::Button("remove colliding pair"))
		{ 
			ndInt32 i = 0; 
			for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* ptr = collidingPairs->m_collidingPairs.GetFirst(); ptr; ptr = ptr->GetNext())
			{
				if (i == m_collidingPairIndex)
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoCollidingPairs(this)));
					collidingPairs->m_collidingPairs.Remove(ptr);
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoCollidingPairs(this)));
					break;
				}
				i++;
			}
			m_collidingPairIndex = 0;
		}

		ndList<ndString> nameList;
		ndFixSizeArray<const char*, 1024> names;
		for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* ptr = collidingPairs->m_collidingPairs.GetFirst(); ptr; ptr = ptr->GetNext())
		{
			char name[256];
			ndSharedPtr<ndMeshCollidingPair>& pair = ptr->GetInfo();
			snprintf(name, sizeof(name) - 1, "%s_%s", pair->m_parentNode->GetName().GetStr(), pair->m_childNode->GetName().GetStr());
			ndList<ndString>::ndNode* const nameNode = nameList.Append(ndString(name));
			names.PushBack(nameNode->GetInfo().GetStr());
		}
		if (names.GetCount())
		{
			ImGui::ListBox(" ##10", &m_collidingPairIndex, &names[0], names.GetCount(), 24);
		}
	}
}

void ndAssetEditor::SetCollidingSubSelection(const ndMesh* const subSelection)
{
	if (subSelection == *m_currentSelection)
	{
		return;
	}

	const ndMesh* const selection = *m_currentSelection;
	ndCollidingPairs* const collidingPairs = m_mesh->GetCollingPairs();
	for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* ptr = collidingPairs->m_collidingPairs.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		const ndSharedPtr<ndMeshCollidingPair>& pair = ptr->GetInfo();
		if ((*pair->m_childNode == selection) || (*pair->m_parentNode == selection))
		{
			const ndMesh* const subMeshSelection = (*pair->m_childNode == selection) ? *pair->m_parentNode : *pair->m_childNode;
			ndAssert(subMeshSelection);
			if (subMeshSelection == subSelection)
			{
				return;
			}
		}
	}

	m_currentSubSelection = ndWeakPtr<ndMesh>((ndMesh*)subSelection);
}

void ndAssetEditor::SetModifierSubSelection(const ndMesh* const subSelection)
{
	if (subSelection == *m_currentSelection)
	{
		return;
	}
	m_currentSubSelection = ndWeakPtr<ndMesh>((ndMesh*)subSelection);
	ndSharedPtr<ndMeshTransformModifier> modifier(m_currentSelection->GetModifier());
	ndAssert(modifier);
	modifier->m_target = ndWeakPtr<const ndMesh>((ndMesh*)subSelection);
}

void ndAssetEditor::SetLoopJointSelection(const ndMesh* const subSelection)
{
	if (subSelection == *m_currentSelection)
	{
		return;
	}

	const ndMesh* parent = m_currentSelection->GetParent();
	while (parent && !parent->GetRigidBody())
	{
		parent = parent->GetParent();
	}
	if (subSelection == parent)
	{
		return;
	}

	parent = subSelection->GetParent();
	while (parent && !parent->GetRigidBody())
	{
		parent = parent->GetParent();
	}

	if (*m_currentSelection == parent)
	{
		return;
	}

	ndCloseLoopConstraints* const loops = m_mesh->GetLoopJoints();
	for (ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* ptr = loops->m_loopJoints.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		const ndSharedPtr<ndMeshLoopJoint>& loop = ptr->GetInfo();

		//bool test = (*loop->m_childNode == *m_currentSelection) && (*loop->m_parentNode == subSelection);
		//test = test || (*loop->m_parentNode == *m_currentSelection) && (*loop->m_childNode == subSelection);

		bool test0 = (*loop->m_childNode == *m_currentSelection);
		bool test1 = (*loop->m_parentNode == *m_currentSelection);
		test0 = test0 && (*loop->m_parentNode == subSelection);
		test1 = test1 && (*loop->m_childNode == subSelection);
		bool test = test0 || test1;

		if (test)
		{
			return;
		}
	}

	m_currentSubSelection = ndWeakPtr<ndMesh>((ndMesh*)subSelection);
}

void ndAssetEditor::SetCustomPropertySelection(const ndMesh* const subSelection)
{
	if (subSelection == *m_currentSelection)
	{
		return;
	}
	m_currentSubSelection = ndWeakPtr<ndMesh>((ndMesh*)subSelection);
}

void ndAssetEditor::AddCollidingPair()
{
	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoCollidingPairs(this)));

	ndCollidingPairs* const collidingPairs = m_mesh->GetCollingPairs();
	ndSharedPtr<ndMeshCollidingPair> pair(new ndMeshCollidingPair(*m_currentSelection, *m_currentSubSelection));
	collidingPairs->m_collidingPairs.Append(pair);
	m_currentSubSelection = ndWeakPtr<ndMesh>(nullptr);

	m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoCollidingPairs(this)));
}
