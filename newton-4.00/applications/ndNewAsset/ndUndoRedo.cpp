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

ndUndoRedoCommand::ndUndoRedoCommand(ndAssetEditor* const editor, const ndMesh* const selectedNode)
	:m_selectedNodeName(selectedNode ? selectedNode->GetName(): "___noSelection___")
	,m_selectedNode((ndMesh*)selectedNode)
	,m_editor(editor)
{
}

ndUndoRedoCommand::~ndUndoRedoCommand()
{
}

ndRenderSceneNode* ndUndoRedoCommand::GetSceneNode() const
{
	return m_editor->m_entity->FindByName(m_selectedNodeName);
}

ndUndoRedo::ndUndoRedo()
	:ndList<ndSharedPtr<ndUndoRedoCommand>>()
	,m_currentCommand(nullptr)
{
}

ndUndoRedo::~ndUndoRedo()
{
	Clear();
}

void ndUndoRedo::Clear()
{
	m_currentCommand = ndWeakPtr<ndNode>(nullptr);
	RemoveAll();
}

void ndUndoRedo::Push(const ndSharedPtr<ndUndoRedoCommand>& command)
{
	if (GetCount() == 0)
	{
		m_currentCommand = Append(command);
	}
	else if (**m_currentCommand->GetInfo() != **command)
	{
		ndAssert(m_currentCommand);
		while (GetLast() != *m_currentCommand)
		{
			Remove(GetLast());
		}
		m_currentCommand = Append(command);
	}
}

void ndUndoRedo::Redo(ndAssetEditor* const owner)
{
	if (GetCount())
	{
		ndAssert(*m_currentCommand);
		if (m_currentCommand->GetNext())
		{
			m_currentCommand = m_currentCommand->GetNext();

			ndUndoRedoCommand* const command = *m_currentCommand->GetInfo();
			command->Undo();
			owner->m_currentSelection = command->m_selectedNode ? command->m_selectedNode->GetRoot()->FindByName(m_currentCommand->GetInfo()->m_selectedNodeName) : ndWeakPtr<ndMesh>(nullptr);
		}
	}
}

void ndUndoRedo::Undo(ndAssetEditor* const owner)
{
	if (GetCount())
	{
		ndAssert(*m_currentCommand);
		if (m_currentCommand->GetPrev())
		{
			m_currentCommand = m_currentCommand->GetPrev();
			ndUndoRedoCommand* const command = *m_currentCommand->GetInfo();
			command->Undo();
			owner->m_currentSelection = command->m_selectedNode ? command->m_selectedNode->GetRoot()->FindByName(m_currentCommand->GetInfo()->m_selectedNodeName) : ndWeakPtr<ndMesh>(nullptr);
		}
	}
}

