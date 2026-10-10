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

#ifndef D_NEWTON_USER_JOINT_CLASS_H_
#define D_NEWTON_USER_JOINT_CLASS_H_

#include "newtonStdafx.h"

class NewtonJoint;


class NewtonJointUserData : public ndJointUserData
{
	public:
	NewtonJointUserData(ndJointBilateralConstraint* const owner);

	ndWeakPtr<void> m_userData;
};


class NewtonUserJoint : public ndJointBilateralConstraint
{
	public:
	typedef void (*NewtonConstraintDestructor) (const NewtonJoint* const me);
	typedef void (*NewtonUserBilateralCallback) (const NewtonJoint* const userJoint, ndFloat32 timestep, int threadIndex);

	D_CLASS_REFLECTION(NewtonUserJoint, ndJointBilateralConstraint)

	NewtonUserJoint(ndInt32 maxDof, ndBodyKinematic* const body0, ndBodyKinematic* const body1, const ndMatrix& globalMatrix, NewtonUserBilateralCallback jacobianCallback);

	virtual ~NewtonUserJoint() override;
	virtual void JacobianDerivative(ndConstraintDescritor& desc) override;

	void UpdateParameters() override;

	ndWeakPtr<ndConstraintDescritor> m_param;
	NewtonConstraintDestructor m_onDestropy;
	NewtonUserBilateralCallback m_onSubmitJacobians;
};

#endif