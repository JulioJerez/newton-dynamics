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

#include "ndRenderStdafx.h"
#include "ndRenderSceneNode.h"
#include "ndRenderTransformModifier.h"

class ndRenderSceneNode;

ndRenderTransformModifier::ndRenderTransformModifier(ndRenderSceneNode* const owner)
	:ndClassAlloc()
	,m_owner(owner)
{
}

ndRenderTransformModifier::~ndRenderTransformModifier()
{
}


ndRenderTransformModifierLockAtNode::ndRenderTransformModifierLockAtNode(ndRenderSceneNode* const owner, ndRenderSceneNode* const target)
	:ndRenderTransformModifier(owner)
	,m_target(target)
{
}

void ndRenderTransformModifierLockAtNode::Update()
{
	const ndMatrix frameMatrix(m_owner->m_matrix * m_owner->GetParent()->m_globalMatrix);
	const ndVector point(frameMatrix.UntransformVector(m_target->m_globalMatrix.m_posit) & ndVector::m_triplexMask);
	const ndVector dir (point.Normalize());

	const ndMatrix yawMatrix(ndYawMatrix(ndAsin(-dir.m_z)));
	const ndMatrix rollMatrix(ndRollMatrix(ndAtan2(dir.m_y, dir.m_x)));

	m_owner->m_globalMatrix = yawMatrix * rollMatrix * frameMatrix;
}
