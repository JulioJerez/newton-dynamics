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

#ifndef _ND_FBX_EXPORT_H_
#define _ND_FBX_EXPORT_H_

#include "ndNewAssetStdafx.h"

class ndFbxExport : public ndRenderMeshLoader
{
	public:
	ndFbxExport(ndRender* const renderer);
	
	bool Export(const ndString& ndPath, const ndString& fbxOuputPath);
};

#endif