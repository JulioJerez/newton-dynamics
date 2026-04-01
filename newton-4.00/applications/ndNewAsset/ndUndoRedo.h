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
#include "ndAssetEditor.h"

#ifndef ND_UNDO_REDO_H
#define ND_UNDO_REDO_H

class ndAssetEditor;
class ndUndoRedoCommand : public ndClassAlloc
{
	public:
	ndUndoRedoCommand(const ndSharedPtr<ndMesh>& mesh);
	virtual ~ndUndoRedoCommand();

	virtual void Undo() = 0;

	ndSharedPtr<ndMesh> m_mesh;
};

class ndUndoRedo: public ndList<ndSharedPtr<ndUndoRedoCommand>>
{
	public:
	ndUndoRedo();

	void Clear();
	void Undo(ndAssetEditor* const owner);
	void Redo(ndAssetEditor* const owner);
	
	void Push(const ndSharedPtr<ndUndoRedoCommand>& command);
	
	//ndSharedPtr<ndUndoRedo::ndNode> m_currentCommand;
	ndWeakPtr<ndNode> m_currentCommand;
};

#endif

