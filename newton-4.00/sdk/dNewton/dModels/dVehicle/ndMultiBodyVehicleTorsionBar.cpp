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
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndMeshComponents.h"
#include "ndMultiBodyVehicle.h"
#include "ndMultiBodyVehicleMotor.h"
#include "ndMultiBodyVehicleTorsionBar.h"

ndMultiBodyVehicleTorsionBar::ndMultiBodyVehicleTorsionBar()
	:ndJointBilateralConstraint()
	,m_axis()
{
	m_maxDof = 1;
	m_axis.m_damperC = ndFloat32(20.0f);
	m_axis.m_springK = ndFloat32(2000.0f);
	m_axis.m_springDamperRegularizer = ndFloat32(0.01f);

	SetSolverModel(m_jointkinematicCloseLoop);
}

ndMultiBodyVehicleTorsionBar::ndMultiBodyVehicleTorsionBar(const ndMatrix& matrix0, ndBodyDynamic* const body0,
	const ndMatrix& matrix1, ndBodyDynamic* const body1)
	:ndJointBilateralConstraint(1, body0, body1, matrix0, matrix1)
	,m_axis()
{
	m_axis.m_damperC = ndFloat32(20.0f);
	m_axis.m_springK = ndFloat32(2000.0f);
	m_axis.m_springDamperRegularizer = ndFloat32(0.01f);
	SetSolverModel(m_jointkinematicCloseLoop);
}

ndSharedPtr<ndMeshJoint> ndMultiBodyVehicleTorsionBar::GetMeshJoint(const ndMesh* const owner) const
{
	ndMeshJointVehicleTorsionBar* const joint = new ndMeshJointVehicleTorsionBar(owner, this);

	// align the transform
	const ndMatrix alignmnt(ndRollMatrix(ndFloat32(90.0f) * ndDegreeToRad));
	const ndMatrix localTrasform0(alignmnt * GetLocalMatrix0());
	const ndMatrix localTrasform1(alignmnt * GetLocalMatrix1());
	joint->m_localFrame0 = localTrasform0;
	joint->m_localFrame1 = localTrasform1;

	return ndSharedPtr<ndMeshJoint>(joint);
}

void ndMultiBodyVehicleTorsionBar::UpdateParameters()
{
	// do nothing for now;
}

void ndMultiBodyVehicleTorsionBar::SetTorsionTorque(ndFloat32 springK, ndFloat32 damperC, ndFloat32 springDamperRegularizer)
{
	m_axis.m_springK = ndAbs(springK);
	m_axis.m_damperC = ndAbs(damperC);
	m_axis.m_springDamperRegularizer = ndClamp (springDamperRegularizer, ND_SPRING_DAMP_MIN_REG, ndFloat32(0.99f));
}

void ndMultiBodyVehicleTorsionBar::GetTorsionTorque(ndFloat32& springK, ndFloat32& damperC, ndFloat32& springDamperRegularizer) const
{
	springK = m_axis.m_springK;
	damperC = m_axis.m_damperC;
	springDamperRegularizer = m_axis.m_springDamperRegularizer;
}

void ndMultiBodyVehicleTorsionBar::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	
	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	const ndVector p0(matrix0.m_posit);
	const ndVector p1(matrix1.m_posit);
	AddLinearRowJacobian(desc, p0, p1, matrix0.m_front);
	SetMassSpringDamperAcceleration(desc, m_axis.m_springDamperRegularizer, m_axis.m_springK, m_axis.m_damperC);
}
