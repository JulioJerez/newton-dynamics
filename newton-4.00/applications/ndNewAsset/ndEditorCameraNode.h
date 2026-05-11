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

#ifndef __CAMERA_NODE_H__
#define __CAMERA_NODE_H__

#include "ndNewAssetStdafx.h"

class ndEditorCameraNode: public ndRenderSceneNode
{
	public:
	ndEditorCameraNode(ndRender* const owner);

	ndRenderSceneCamera* GetCamera() const;
	virtual void TickUpdate(ndFloat32 timestep) = 0;
	virtual ndRenderSceneNode* Clone() const;
	
	protected:
	bool m_prevMouseState;
	ndWeakPtr<ndRenderSceneCamera> m_camera;
};

#endif 
