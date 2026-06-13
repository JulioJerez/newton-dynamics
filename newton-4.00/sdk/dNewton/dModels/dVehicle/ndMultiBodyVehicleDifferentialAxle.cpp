/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndMeshComponents.h"
#include "ndMultiBodyVehicleDifferentialAxle.h"

ndMultiBodyVehicleDifferentialAxle::ndMultiBodyVehicleDifferentialAxle()
	:ndJointBilateralConstraint()
{
	m_maxDof = 1;
}

ndMultiBodyVehicleDifferentialAxle::ndMultiBodyVehicleDifferentialAxle(
	const ndVector& differentialPin, const ndVector& differentialUpPin, ndBodyKinematic* const differentialBody,
	const ndVector& parentPin, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(1, differentialBody, parent, ndGetIdentityMatrix())
	,m_omega(ndFloat32 (0.0f))
{
	ndMatrix temp;
	ndMatrix matrix0(differentialPin, differentialUpPin, differentialPin.CrossProduct(differentialUpPin), ndVector::m_wOne);
	ndMatrix matrix1(ndGramSchmidtMatrix(parentPin));
	matrix0.m_posit = differentialBody->GetMatrix().m_posit;
	matrix1.m_posit = parent->GetMatrix().m_posit;

	ndAssert(matrix0.TestOrthogonal());
	ndAssert(matrix1.TestOrthogonal());

	CalculateLocalMatrix(matrix0, m_localMatrix0, temp);
	CalculateLocalMatrix(matrix1, temp, m_localMatrix1);
	SetSolverModel(m_jointkinematicCloseLoop);
}

ndSharedPtr<ndMeshJoint> ndMultiBodyVehicleDifferentialAxle::GetMeshJoint(const ndMesh* const owner) const
{
	ndMeshJointVehicleDifferentialAxle* const joint = new ndMeshJointVehicleDifferentialAxle(owner, this);
	return ndSharedPtr<ndMeshJoint>(joint);
}

void ndMultiBodyVehicleDifferentialAxle::UpdateParameters()
{
	// do nothing;
	//ndMatrix matrix0;
	//ndMatrix matrix1;
	//CalculateGlobalMatrix(matrix0, matrix1);
	//
	//const ndVector jacobian0(matrix0.m_front);
	//const ndVector jacobian1(matrix1.m_front + matrix1.m_up);
	//const ndVector& omega0 = m_body0->GetOmega();
	//const ndVector& omega1 = m_body1->GetOmega();
	//
	//const ndVector relOmega(omega0 * jacobian0 + omega1 * jacobian1);
	//m_omega = relOmega.AddHorizontal().GetScalar();
}

ndFloat32 ndMultiBodyVehicleDifferentialAxle::GetGearOmega() const
{
	//return m_omega;
	return 0.0f;
}

void ndMultiBodyVehicleDifferentialAxle::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	//AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
	AddAngularRowJacobian(desc, matrix1.m_right, ndFloat32(0.0f));

	ndJacobian& jacobian1 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
	ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;

	jacobian1.m_angular = matrix1.m_front;
	jacobian0.m_angular = matrix0.m_front + matrix0.m_up;

	const ndVector& omega0 = m_body0->GetOmega();
	const ndVector& omega1 = m_body1->GetOmega();

	const ndVector relOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
	const ndFloat32 w = relOmega.AddHorizontal().GetScalar() * ndFloat32(0.5f);
	SetMotorAcceleration(desc, -w * desc.m_invTimestep);
}

