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

ndRenderTransformModifierLookAtNode::ndRenderTransformModifierLookAtNode(ndRenderSceneNode* const owner, ndRenderSceneNode* const target)
	:ndRenderTransformModifier(owner)
	,m_target(target)
{
}

void ndRenderTransformModifierLookAtNode::Update()
{
	const ndMatrix frameMatrix(m_owner->m_matrix * m_owner->GetParent()->m_globalMatrix);
	const ndVector targetPoint(frameMatrix.UntransformVector(m_target->m_globalMatrix.m_posit) & ndVector::m_triplexMask);
	const ndVector targetDir(targetPoint.Normalize());

	const ndMatrix yawMatrix(ndYawMatrix(ndAsin(-targetDir.m_z)));
	const ndMatrix rollMatrix(ndRollMatrix(ndAtan2(targetDir.m_y, targetDir.m_x)));
	m_owner->m_globalMatrix = yawMatrix * rollMatrix * frameMatrix;
}

ndRenderTransformModifierTwoLinksIK::ndRenderTransformModifierTwoLinksIK(ndRenderSceneNode* const owner, ndRenderSceneNode* const linkChild, ndRenderSceneNode* const target, ndFloat32 solutionSign)
	:ndRenderTransformModifier(owner)
	,m_target(target)
	,m_linkChild(linkChild)
	,m_sign(solutionSign)
{
	const ndMatrix targetMatrix(m_target->CalculateGlobalMatrix());
	const ndMatrix linkMatrix(m_linkChild->CalculateGlobalMatrix());

	const ndMatrix frameMatrix(m_owner->m_matrix * m_owner->GetParent()->CalculateGlobalMatrix());
	const ndVector targetPoint(frameMatrix.UntransformVector(targetMatrix.m_posit) & ndVector::m_triplexMask);

	ndVector segL0(frameMatrix.UntransformVector(linkMatrix.m_posit) & ndVector::m_triplexMask);
	ndVector segL1(targetPoint - segL0);

	segL0.m_z = ndFloat32(0.0f);
	segL1.m_z = ndFloat32(0.0f);
	m_l0 = ndSqrt(segL0.DotProduct(segL0).GetScalar());
	m_l1 = ndSqrt(segL1.DotProduct(segL1).GetScalar());
	m_initialAngle = TwoLinksIK(targetPoint.m_x, targetPoint.m_y, m_l0, m_l1, m_sign);
}

void ndRenderTransformModifierTwoLinksIK::Update()
{
	const ndMatrix frameMatrix(m_owner->m_matrix * m_owner->GetParent()->m_globalMatrix);
	const ndVector targetPoint(frameMatrix.UntransformVector(m_target->m_globalMatrix.m_posit) & ndVector::m_triplexMask);
	const ndVector linkPoint(frameMatrix.UntransformVector(m_linkChild->m_globalMatrix.m_posit) & ndVector::m_triplexMask);
	
	//const ndVector targetDir(targetPoint.Normalize());
	//const ndMatrix yawMatrix(ndYawMatrix(ndAsin(-targetDir.m_z)));
	//const ndMatrix rollMatrix(ndRollMatrix(ndAtan2(targetDir.m_y, targetDir.m_x)));
	const TwoLinksIK ik2d(targetPoint.m_x, targetPoint.m_y, m_l0, m_l1, m_sign);

	//const ndMatrix yawMatrix(ndGetIdentityMatrix());
	const ndMatrix rollMatrix0(ndRollMatrix(ik2d.m_angle0 - m_initialAngle.m_angle0));
	m_owner->m_globalMatrix = rollMatrix0 * frameMatrix;

	ndMatrix rollMatrix1(m_linkChild->m_matrix * ndRollMatrix(ik2d.m_angle1 - m_initialAngle.m_angle1));
	rollMatrix1.m_posit = m_linkChild->m_matrix.m_posit;
	m_linkChild->m_globalMatrix = rollMatrix1 * m_owner->m_globalMatrix;
}