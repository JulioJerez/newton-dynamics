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

#ifndef __D_ROTATE_MESH_H__
#define __D_ROTATE_MESH_H__

class ndRotateMesh : public ndAssetEditor::ndAssetTool
{
	public:
	ndRotateMesh(ndAssetEditor* const owner);

	void ApplyRotation();

	virtual void Execute() override;
	ndReal m_angles[3];
};

class ndRotatePivots : public ndAssetEditor::ndAssetTool
{
	public:
	ndRotatePivots(ndAssetEditor* const owner);

	void ApplyRotation();

	virtual void Execute() override;
	ndReal m_angles[3];
};

class ndRotateBones : public ndAssetEditor::ndAssetTool
{
	public:
	ndRotateBones(ndAssetEditor* const owner);

	void ApplyRotation();

	virtual void Execute() override;
	ndReal m_angles[3];
};

#endif