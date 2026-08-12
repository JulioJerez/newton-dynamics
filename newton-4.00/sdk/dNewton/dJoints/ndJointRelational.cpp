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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointRelational.h"

ndJointRelational::ndJointRelational()
	:ndJointBilateralConstraint()
	//,m_angle(ndFloat32(0.0f))
	//,m_omega(ndFloat32(0.0f))
	,m_gearRatio(ndFloat32(1.0f))
	,m_regularizer(ndFloat32(0.0f))
{
	m_maxDof = 1;
}

ndJointRelational::ndJointRelational(ndFloat32 gearRatio,
	const ndVector& parentPin, ndBodyKinematic* const parent,
	const ndVector& childPin, ndBodyKinematic* const child)
	:ndJointBilateralConstraint(1, child, parent, ndGetIdentityMatrix())
	,m_gearRatio(gearRatio)
	,m_regularizer(ndFloat32(0.0f))
{
	// calculate the two local matrix of the pivot point
	ndMatrix dommyMatrix;

	// calculate the local matrix for body body0
	const ndMatrix pinAndPivotChild(ndGramSchmidtMatrix(childPin));
	CalculateLocalMatrix(pinAndPivotChild, m_localMatrix0, dommyMatrix);
	m_localMatrix0.m_posit = ndVector::m_wOne;

	// calculate the local matrix for body body1  
	const ndMatrix pinAndPivotParent(ndGramSchmidtMatrix(parentPin));
	CalculateLocalMatrix(pinAndPivotParent, dommyMatrix, m_localMatrix1);
	m_localMatrix1.m_posit = ndVector::m_wOne;

	// set as kinematic loop
	SetSolverModel(m_jointkinematicOpenLoop);
}

ndJointRelational::~ndJointRelational()
{
}

ndFloat32 ndJointRelational::GetRatio() const
{
	return m_gearRatio;
}

void ndJointRelational::SetRatio(ndFloat32 ratio)
{
	m_gearRatio = ratio;
}

ndFloat32 ndJointRelational::GetRegularizer() const
{
	return m_regularizer;
}

void ndJointRelational::SetRegularizer(ndFloat32 regularizer)
{
	m_regularizer = ndClamp(regularizer, ndFloat32(0.0f), ndFloat32(0.5f));
}

void ndJointRelational::UpdateParameters()
{
	ndAssert(0);
}

void ndJointRelational::JacobianDerivative(ndConstraintDescritor&)
{
	ndAssert(0);
	//ndMatrix matrix0;
	//ndMatrix matrix1;
	//
	//if (ndAbs(m_gearRatio) > ndFloat32(1.0e-3f))
	//{
	//	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	//	CalculateGlobalMatrix(matrix0, matrix1);
	//
	//	AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
	//
	//	ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
	//	ndJacobian& jacobian1 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
	//
	//	jacobian0.m_angular = matrix0.m_front;
	//	jacobian1.m_angular = matrix1.m_front.Scale(m_gearRatio);
	//
	//	const ndVector& omega0 = m_body0->GetOmega();
	//	const ndVector& omega1 = m_body1->GetOmega();
	//
	//	const ndVector relOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
	//	const ndFloat32 w = relOmega.AddHorizontal().GetScalar();
	//
	//	SetDiagonalRegularizer(desc, m_regularizer);
	//	SetMotorAcceleration(desc, -w * desc.m_invTimestep);
	//}
}

