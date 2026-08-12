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
#include "ndJointWormGear.h"

ndJointWormGear::ndJointWormGear()
	:ndJointRelational()
{
}

ndJointWormGear::ndJointWormGear(ndFloat32 gearRatio,
	const ndVector& parentPin, ndBodyKinematic* const parent,
	const ndVector& childPin, ndBodyKinematic* const child)
	:ndJointRelational(gearRatio, childPin, child, parentPin, parent)
{
}

void ndJointWormGear::UpdateParameters()
{
	// do nothing for now
}

void ndJointWormGear::JacobianDerivative(ndConstraintDescritor& desc)
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

		jacobian0.m_linear = ndVector::m_zero;
		jacobian0.m_angular = matrix0.m_front.Scale(m_gearRatio);

		jacobian1.m_linear = matrix1.m_front;
		jacobian1.m_angular = ndVector::m_zero;

		const ndVector omega(m_body0->GetOmega());
		const ndVector veloc(m_body1->GetVelocity());
		const ndVector relVeloc(omega * jacobian0.m_angular + veloc * jacobian1.m_linear);
		const ndFloat32 w = relVeloc.AddHorizontal().GetScalar();

		SetDiagonalRegularizer(desc, m_regularizer);
		SetMotorAcceleration(desc, -w * desc.m_invTimestep);
	}
}

