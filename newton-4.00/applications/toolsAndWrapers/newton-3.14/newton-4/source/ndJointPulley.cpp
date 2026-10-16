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
#include "ndJointPulley.h"

ndJointPulley::ndJointPulley()
	:ndJointRelational()
	,m_posit(ndFloat32 (0.0f))
	,m_speed(ndFloat32(0.0f))
{
}

ndJointPulley::ndJointPulley(ndFloat32 gearRatio,
	const ndVector& parentPin, ndBodyKinematic* const parent,
	const ndVector& childPin, ndBodyKinematic* const child)
	:ndJointRelational(gearRatio, childPin, child, parentPin, parent)
	,m_posit(ndFloat32(0.0f))
	,m_speed(ndFloat32(0.0f))
{
}

void ndJointPulley::UpdateParameters()
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	
	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);
	
	const ndVector jacobian0(matrix0.m_front.Scale(m_gearRatio));
	const ndVector jacobian1(matrix1.m_front);
	
	const ndVector& veloc0 = m_body0->GetVelocity();
	const ndVector& veloc1 = m_body1->GetVelocity();
	
	const ndVector relSpeed(veloc0 * jacobian0 + veloc1 * jacobian1);
	m_speed = relSpeed.AddHorizontal().GetScalar();
	
	const ndVector relPosit(matrix0.m_posit * jacobian0 + matrix1.m_posit * jacobian1);
	m_posit = relPosit.AddHorizontal().GetScalar();
}

void ndJointPulley::JacobianDerivative(ndConstraintDescritor& desc)
{
	if (ndAbs(m_gearRatio) > ndFloat32(1.0e-3f))
	{
		ndMatrix matrix0;
		ndMatrix matrix1;

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix(matrix0, matrix1);

		AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));

		ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
		ndJacobian& jacobian1 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;

		jacobian0.m_angular = ndVector::m_zero;
		jacobian1.m_angular = ndVector::m_zero;
		jacobian0.m_linear = matrix0.m_front.Scale(m_gearRatio);
		jacobian1.m_linear = matrix1.m_front;

		const ndVector& veloc0 = m_body0->GetVelocity();
		const ndVector& veloc1 = m_body1->GetVelocity();

		const ndVector relVeloc(veloc0 * jacobian0.m_linear + veloc1 * jacobian1.m_linear);
		const ndFloat32 w = relVeloc.AddHorizontal().GetScalar();

		SetDiagonalRegularizer(desc, m_regularizer);
		SetMotorAcceleration(desc, -w * desc.m_invTimestep);
	}
}

