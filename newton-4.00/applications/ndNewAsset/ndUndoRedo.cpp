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

ndUndoRedoCommand::ndUndoRedoCommand(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
	:m_mesh(mesh)
	,m_editor(editor)
{
}

ndUndoRedoCommand::~ndUndoRedoCommand()
{
}

ndRenderSceneNode* ndUndoRedoCommand::GetSceneNode() const
{
	return m_editor->m_entity->FindByName(m_mesh->GetName());
}

ndUndoRedo::ndUndoRedo()
	:ndList<ndSharedPtr<ndUndoRedoCommand>>()
	,m_currentCommand(nullptr)
{
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
			m_currentCommand->GetInfo()->Undo();
			owner->m_currentSelection = m_currentCommand->GetInfo()->m_mesh;
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
			m_currentCommand->GetInfo()->Undo();
			owner->m_currentSelection = m_currentCommand->GetInfo()->m_mesh;
		}
	}
}

