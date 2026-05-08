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
#ifndef __ND_RENDER_TRANSFORM_MODIOFIER_H__
#define __ND_RENDER_TRANSFORM_MODIOFIER_H__

#include "ndRenderStdafx.h"

class ndRenderSceneNode;

class ndRenderTransformModifier: public ndClassAlloc
{
	public:
	ndRenderTransformModifier(ndRenderSceneNode* const owner);
	virtual ~ndRenderTransformModifier();
	virtual void Update() = 0;

	ndWeakPtr<ndRenderSceneNode> m_owner;
};

class ndRenderTransformModifierLockAtNode: public ndRenderTransformModifier
{
	public:
	ndRenderTransformModifierLockAtNode(ndRenderSceneNode* const owner, ndRenderSceneNode* const target);

	virtual void Update() override;

	ndWeakPtr<ndRenderSceneNode> m_target;
};

#endif