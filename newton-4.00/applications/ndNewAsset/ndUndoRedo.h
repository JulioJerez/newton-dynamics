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
	ndUndoRedoCommand(ndAssetEditor* const editor, const ndMesh* const selectedNode);
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

	ndString m_selectedNodeName;
	ndWeakPtr<ndMesh> m_selectedNode;
	ndWeakPtr<ndAssetEditor> m_editor;
};

class ndUndoRedoMeshNode : public ndUndoRedoCommand
{
	public:
	ndUndoRedoMeshNode(ndAssetEditor* const editor, const ndMesh* const selectedNode);
	class ndUndoRedoMeshNode* GetAsUndoRedoMeshNode() const override;
	virtual bool operator!=(const ndUndoRedoCommand& command) const override;
	virtual void Undo() override;

	ndSharedPtr<ndMesh> m_copy;
};

class ndUndoRedo: public ndList<ndSharedPtr<ndUndoRedoCommand>>
{
	public:
	ndUndoRedo();
	~ndUndoRedo();

	void Clear();
	void Undo(ndAssetEditor* const owner);
	void Redo(ndAssetEditor* const owner);
	void Push(const ndSharedPtr<ndUndoRedoCommand>& command);

	ndWeakPtr<ndNode> m_currentCommand;
};

#endif

