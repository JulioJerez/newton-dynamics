/* Copyright (c) <2003-2021> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "newtonStdafx.h"
#include "NewtonUtils.h"
#include "newtonWorld.h"
#include "newtonUserJointClass.h"

NewtonJointUserData::NewtonJointUserData(ndJointBilateralConstraint* const owner)
	:ndJointUserData(owner)
	,m_userData(nullptr)
{
}


NewtonUserJoint::NewtonUserJoint(ndInt32 maxDof, ndBodyKinematic* const body0, ndBodyKinematic* const body1, const ndMatrix& globalMatrix, NewtonUserBilateralCallback jacobianCallback)
	:ndJointBilateralConstraint(maxDof, body0, body1, globalMatrix)
	,m_onDestropy(nullptr)
	,m_onSubmitJacobians(jacobianCallback)
{
}

NewtonUserJoint::~NewtonUserJoint()
{
	if (m_onDestropy)
	{
		m_onDestropy(PointerCast<NewtonJoint, NewtonUserJoint>(this));
	}
}

void NewtonUserJoint::UpdateParameters()
{
	// do nothing
}

void NewtonUserJoint::JacobianDerivative(ndConstraintDescritor& desc)
{
	if (m_onSubmitJacobians)
	{
		m_param = &desc;
		m_onSubmitJacobians(PointerCast<NewtonJoint, NewtonUserJoint>(this), desc.m_timestep, 0);
	}
}
