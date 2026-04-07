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
	ndUndoRedoCommand(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh);
	virtual ~ndUndoRedoCommand();

	ndRenderSceneNode* GetSceneNode() const;

	virtual void Undo() = 0;
	virtual bool operator!=(const ndUndoRedoCommand& command) const = 0;

	virtual class ndUndoRedoMass* GetAsUndoRedoMass() const { return nullptr; }
	virtual class ndUndoRedoName* GetAsUndoRedoName() const { return nullptr; }
	virtual class ndUndoRedoTransform* GetAsUndoRedoTransform() const { return nullptr; }
	virtual class ndUndoRedoAngleStep* GetAsUndoRedoAngleStep() const { return nullptr; }
	virtual class ndUndoRedoLinearStep* GetAsUndoRedoLinearStep() const { return nullptr; }
	virtual class ndUndoRedoLinearDamp* GetAsUndoRedoLinearDamp() const { return nullptr; }
	virtual class ndUndoRedoCenterOfMass* GetAsUndoRedoCenterOfMass() const { return nullptr; }
	virtual class ndUndoRedoGeometryTransform* GetAsUndoRedoGeometryTransform() const { return nullptr; }

	ndSharedPtr<ndMesh> m_mesh;
	ndWeakPtr<ndAssetEditor> m_editor;
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

