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
#include "ndEditorCameraNode.h"

ndEditorCameraNode::ndEditorCameraNode(ndRender* const owner)
	:ndRenderSceneNode(ndGetIdentityMatrix())
	,m_prevMouseState(false)
{
	m_owner = owner;
	m_name = ndString("__PlayerCamera__");
	ndSharedPtr<ndRenderSceneNode> camera(new ndRenderSceneCamera(owner));
	m_camera = camera->GetAsCamera();
	AddChild(camera);
}

ndRenderSceneNode* ndEditorCameraNode::Clone() const
{
	ndAssert(0);
	return nullptr;
}

ndRenderSceneCamera* ndEditorCameraNode::GetCamera() const
{
	return (ndRenderSceneCamera*)*m_camera;
}
