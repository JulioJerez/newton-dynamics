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

#ifndef _ND_MESH_LOADER_H_
#define _ND_MESH_LOADER_H_

#include "ndNewtonStdafx.h"

class ndMesh;
class ndString;

class ndMeshLoader : public ndClassAlloc
{
	public:
	D_NEWTON_API ndMeshLoader();
	D_NEWTON_API ndMeshLoader(const ndSharedPtr<ndMesh>& mesh);
	D_NEWTON_API virtual ~ndMeshLoader();

	D_NEWTON_API virtual bool LoadMesh(const ndString& pathMeshName);
	D_NEWTON_API virtual void SaveMesh(const ndString& pathMeshName) const;

	public:
	ndSharedPtr<ndMesh> m_mesh;
};

#endif