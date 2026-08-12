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
#include "ndJointGear.h"
#include "ndMeshComponents.h"

ndJointGear::ndJointGear()
	:ndJointRelational()
	,m_angle(ndFloat32(0.0f))
	,m_omega(ndFloat32(0.0f))
{
}

ndJointGear::ndJointGear(ndFloat32 gearRatio,
	const ndVector& parentPin, ndBodyKinematic* const parent,
	const ndVector& childPin, ndBodyKinematic* const child)
	:ndJointRelational(gearRatio, childPin, child, parentPin, parent)
	,m_angle(ndFloat32(0.0f))
	,m_omega(ndFloat32(0.0f))
{
}

ndSharedPtr<ndMeshJoint> ndJointGear::GetMeshJoint(const ndMesh* const owner) const
{
	ndMeshJointGear* const joint = new ndMeshJointGear(owner, this);
	return ndSharedPtr<ndMeshJoint>(joint);
}

void ndJointGear::UpdateParameters()
{
	ndMatrix matrix0;
	ndMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	const ndVector jacobian0(matrix0.m_front.Scale(m_gearRatio));
	const ndVector jacobian1(matrix1.m_front);

	const ndVector& omega0 = m_body0->GetOmega();
	const ndVector& omega1 = m_body1->GetOmega();

	const ndVector relOmega(omega0 * jacobian0 + omega1 * jacobian1);
	m_omega = relOmega.AddHorizontal().GetScalar();

	const ndFloat32 deltaAngle = ndAnglesAdd(-CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front), -m_angle);
	m_angle += deltaAngle;
}

void ndJointGear::JacobianDerivative(ndConstraintDescritor& desc)
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

		jacobian0.m_angular = matrix0.m_front;
		jacobian1.m_angular = matrix1.m_front.Scale(m_gearRatio);

		const ndVector& omega0 = m_body0->GetOmega();
		const ndVector& omega1 = m_body1->GetOmega();

		const ndVector relOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
		const ndFloat32 w = relOmega.AddHorizontal().GetScalar();

		SetDiagonalRegularizer(desc, m_regularizer);
		SetMotorAcceleration(desc, -w * desc.m_invTimestep);
	}
}

