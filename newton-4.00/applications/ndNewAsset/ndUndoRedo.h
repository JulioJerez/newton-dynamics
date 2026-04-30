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

	virtual class ndUndoRedoShape* GetAsUndoRedoShape() const { return nullptr; }
	virtual class ndUndoRedoRigidBody* GetAsUndoRedoRigidBody() const { return nullptr; }
	virtual class ndUndoRedoMeshNode* GetAsUndoRedoMeshNode() const { return nullptr; }

	virtual class ndUndoRedoLoopJoint* GetAsUndoRedoLoopJoint() const { return nullptr; }
	virtual class ndUndoRedoStructuralJoint* GetAsUndoRedoStructuralJoint() const { return nullptr; }
	virtual class ndUndoRedoCollidingPairs* GetAsUndoRedoCollidingPairs() const { return nullptr; }

	virtual class ndUndoRedoResizeMesh* GetAsUndoRedoResizeMesh() const { return nullptr; }
	virtual class ndUndoRedoRotateMesh* GetAsUndoRedoResizeRotateMesh() const { return nullptr; }
	virtual class ndUndoRedoNormalizeMass* GetAsUndoRedoResizeNormalizeMass() const { return nullptr; }

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

