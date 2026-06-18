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
#include "ndJointWheel.h"
#include "ndBodyDynamic.h"
#include "ndMeshComponents.h"
#include "ndMultiBodyVehicle.h"
#include "ndMultiBodyVehicleMotor.h"
#include "ndMultiBodyVehicleGearBox.h"

ndMultiBodyVehicleMotor::ndEngineTorqueCurve::ndEngineTorqueCurve()
{
	// take from the data sheet of a 2005 dodge viper, 
	// some values are missing so I have to improvise them
	//ndFloat32 idleTorquePoundFoot = ndFloat32(100.0f);
	//ndFloat32 idleRmp = ndFloat32(800.0f);
	//ndFloat32 horsePower = ndFloat32(400.0f);
	//ndFloat32 rpm0 = ndFloat32(5000.0f);
	//ndFloat32 rpm1 = ndFloat32(6200.0f);
	//ndFloat32 horsePowerAtRedLine = ndFloat32(100.0f);
	//ndFloat32 redLineRpm = ndFloat32(8000.0f);

	ndFloat32 idleTorquePoundFoot = ndFloat32(100.0f);
	ndFloat32 idleRmp = ndFloat32(900.0f);
	ndFloat32 horsePower = ndFloat32(400.0f);
	ndFloat32 rpm0 = ndFloat32(5000.0f);
	ndFloat32 rpm1 = ndFloat32(6200.0f);
	ndFloat32 horsePowerAtRedLine = ndFloat32(100.0f);
	ndFloat32 redLineRpm = ndFloat32(8000.0f);

	Init(idleTorquePoundFoot, idleRmp,
		horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);
}

void ndMultiBodyVehicleMotor::ndEngineTorqueCurve::Init(
	ndFloat32 idleTorquePoundFoot, ndFloat32 idleRmp,
	ndFloat32 horsePower, ndFloat32 rpm0, ndFloat32 rpm1,
	ndFloat32 horsePowerAtRedLine, ndFloat32 redLineRpm)
{
	m_torqueCurve.SetCount(5);
	m_torqueCurve[0] = ndTorqueTap(ndFloat32(0.0f), idleTorquePoundFoot);
	m_torqueCurve[1] = ndTorqueTap(idleRmp, idleTorquePoundFoot);

	ndFloat32 power = horsePower * ndFloat32(746.0f);
	ndFloat32 omegaInRadPerSec = rpm0 * ndFloat32(0.105f);
	ndFloat32 torqueInPoundFood = (power / omegaInRadPerSec) / ndFloat32(1.36f);
	m_torqueCurve[2] = ndTorqueTap(rpm0, torqueInPoundFood);

	power = horsePower * ndFloat32(746.0f);
	omegaInRadPerSec = rpm1 * ndFloat32(0.105f);
	torqueInPoundFood = (power / omegaInRadPerSec) / ndFloat32(1.36f);
	m_torqueCurve[3] = ndTorqueTap(rpm1, torqueInPoundFood);

	power = horsePowerAtRedLine * ndFloat32(746.0f);
	omegaInRadPerSec = redLineRpm * ndFloat32(0.105f);
	torqueInPoundFood = (power / omegaInRadPerSec) / ndFloat32(1.36f);
	m_torqueCurve[4] = ndTorqueTap(redLineRpm, torqueInPoundFood);

	m_omegaStep = ndFloat32(8.0f);
	m_frictionLoss = GetTorque(ndFloat32(0.0f)) * ndFloat32(0.5f);
}

void ndMultiBodyVehicleMotor::ndEngineTorqueCurve::SetOmegaAccel(ndFloat32 rpmStep)
{
	m_omegaStep = ndAbs(rpmStep / ndRadPerSecToRpm);
}

ndFloat32 ndMultiBodyVehicleMotor::ndEngineTorqueCurve::GetIdleRadPerSec() const
{
	return m_torqueCurve[1].m_radPerSeconds;
}

ndFloat32 ndMultiBodyVehicleMotor::ndEngineTorqueCurve::GetLowGearShiftRadPerSec() const
{
	return m_torqueCurve[2].m_radPerSeconds;
}

ndFloat32 ndMultiBodyVehicleMotor::ndEngineTorqueCurve::GetHighGearShiftRadPerSec() const
{
	return m_torqueCurve[3].m_radPerSeconds;
}

ndFloat32 ndMultiBodyVehicleMotor::ndEngineTorqueCurve::GetRedLineRadPerSec() const
{
	const ndFloat32 omega = m_torqueCurve[m_torqueCurve.GetCount() - 1].m_radPerSeconds;
	return omega;
}

ndFloat32 ndMultiBodyVehicleMotor::ndEngineTorqueCurve::GetTorque(ndFloat32 omegaInRadPerSeconds) const
{
	omegaInRadPerSeconds = ndClamp(omegaInRadPerSeconds, ndFloat32(0.0f), m_torqueCurve[m_torqueCurve.GetCount() - 1].m_radPerSeconds);
	for (ndInt32 i = 1; i < m_torqueCurve.GetCount(); ++i)
	{
		if (omegaInRadPerSeconds <= m_torqueCurve[i].m_radPerSeconds)
		{
			const ndFloat32 omega0 = m_torqueCurve[i - 0].m_radPerSeconds;
			const ndFloat32 omega1 = m_torqueCurve[i - 1].m_radPerSeconds;

			const ndFloat32 torque0 = m_torqueCurve[i - 0].m_torqueInNewtonMeters;
			const ndFloat32 torque1 = m_torqueCurve[i - 1].m_torqueInNewtonMeters;

			const ndFloat32 torque = torque0 + (omegaInRadPerSeconds - omega0) * (torque1 - torque0) / (omega1 - omega0);
			return torque;
		}
	}
	return m_torqueCurve[m_torqueCurve.GetCount() - 1].m_torqueInNewtonMeters;
}


ndMultiBodyVehicleMotor::ndMultiBodyVehicleMotor()
	:ndJointBilateralConstraint()
	,m_vehicle(nullptr)
	,m_engineCurve()
	,m_omega(ndFloat32(0.0f))
	,m_targetOmega(ndFloat32(0.0f))
	,m_engineTorque(ndFloat32(0.0f))
{
	m_maxDof = 3;
}

ndMultiBodyVehicleMotor::ndMultiBodyVehicleMotor(ndBodyKinematic* const motor, ndMultiBodyVehicle* const vehicelModel)
	:ndJointBilateralConstraint(3, motor, *vehicelModel->m_chassis, motor->GetMatrix())
	,m_vehicle(vehicelModel)
	,m_omega(ndFloat32(0.0f))
	,m_targetOmega(ndFloat32(0.0f))
	,m_engineTorque(ndFloat32(0.0f))
{
}

ndMultiBodyVehicleMotor::ndMultiBodyVehicleMotor(ndBodyKinematic* const motor, ndBodyKinematic* const chassis)
	:ndJointBilateralConstraint(3, motor, chassis, motor->GetMatrix())
	,m_vehicle(nullptr)
	,m_omega(ndFloat32(0.0f))
	,m_targetOmega(ndFloat32(0.0f))
	,m_engineTorque(ndFloat32(0.0f))
{
}

ndSharedPtr<ndMeshJoint> ndMultiBodyVehicleMotor::GetMeshJoint(const ndMesh* const owner) const
{
	ndMeshJointVehicleMotor* const joint = new ndMeshJointVehicleMotor(owner, this);
	ndAssert(0);
	//joint->m_maxOmega = ndReal(m_maxOmega);
	return ndSharedPtr<ndMeshJoint>(joint);
}

void ndMultiBodyVehicleMotor::SetVehicleOwner(ndMultiBodyVehicle* const vehicle)
{
	m_vehicle = vehicle;
}

void ndMultiBodyVehicleMotor::UpdateParameters()
{
	//ndTrace(("do nothing for now\n"));
}

void ndMultiBodyVehicleMotor::AlignMatrix()
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	m_body0->SetMatrixNoSleep(matrix1);
	m_body0->SetVelocityNoSleep(m_body1->GetVelocity());

	const ndVector omega0(m_body0->GetOmega());
	const ndVector omega1(m_body1->GetOmega());

	const ndVector wx(matrix1.m_front.Scale(matrix1.m_front.DotProduct(omega0).GetScalar()));
	const ndVector wy(matrix1.m_up.Scale(matrix1.m_up.DotProduct(omega1).GetScalar()));
	const ndVector wz(matrix1.m_right.Scale (matrix1.m_right.DotProduct(omega1).GetScalar()));
	const ndVector omega(wx + wy + wz);
	m_body0->SetOmegaNoSleep(omega);
}

//void ndMultiBodyVehicleMotor::SetFrictionLoss(ndFloat32 newtonMeters)
//{
//	m_internalFriction = ndAbs(newtonMeters);
//}
//void ndMultiBodyVehicleMotor::SetMaxRpm(ndFloat32 redLineRpm)
//{
//	m_maxOmega = ndMax(redLineRpm / ndRadPerSecToRpm, ndFloat32 (0.0f));
//}
//void ndMultiBodyVehicleMotor::SetOmegaAccel(ndFloat32 rpmStep)
//{
//	m_omegaStep = ndAbs(rpmStep / ndRadPerSecToRpm);
//}

ndFloat32 ndMultiBodyVehicleMotor::GetMaxRpm() const
{
	return m_engineCurve.m_torqueCurve[m_engineCurve.m_torqueCurve.GetCount() - 1].m_radPerSeconds * ndRadPerSecToRpm;
}

const ndMultiBodyVehicleMotor::ndEngineTorqueCurve& ndMultiBodyVehicleMotor::GetCurve() const
{
	return m_engineCurve;
}

void ndMultiBodyVehicleMotor::SetCurve(const ndEngineTorqueCurve& curve)
{
	m_engineCurve = curve;
}

void ndMultiBodyVehicleMotor::SetTorqueAndRpm(ndFloat32 newtonMeters, ndFloat32 rpm)
{
	m_engineTorque = ndMax(newtonMeters, ndFloat32(0.0f));
	m_targetOmega = ndClamp(rpm, ndFloat32(0.0f), GetMaxRpm()) / ndRadPerSecToRpm;
}

ndFloat32 ndMultiBodyVehicleMotor::GetRpm() const
{
	return m_omega * ndRadPerSecToRpm;
}

ndFloat32 ndMultiBodyVehicleMotor::CalculateAcceleration(ndConstraintDescritor& desc)
{
	const ndVector& motorOmega = m_body0->GetOmega();
	const ndJacobian& motorJacobian = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
	const ndVector relOmega(motorOmega * motorJacobian.m_angular);

	ndFloat32 currentOmega = relOmega.AddHorizontal().GetScalar();
	if (currentOmega < ndFloat32(0.0f))
	{
		const ndVector clippedOmega(motorOmega - motorJacobian.m_angular * relOmega);
		m_body0->SetOmega(clippedOmega);
		currentOmega = 0;
	}

	m_omega = currentOmega;
	ndFloat32 gasStep = m_engineCurve.m_omegaStep;
	ndFloat32 omegaStep = ndClamp(m_targetOmega - m_omega, -gasStep, gasStep);
	ndFloat32 accel = omegaStep * desc.m_invTimestep;
	return accel;
}

void ndMultiBodyVehicleMotor::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	
	// two rows to restrict rotation around around the parent coordinate system
	const ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	const ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	
	AddAngularRowJacobian(desc, matrix1.m_up, angle0);
	AddAngularRowJacobian(desc, matrix1.m_right, angle1);
	
	// add rotor joint acceleration
	AddAngularRowJacobian(desc, matrix0.m_front * ndVector::m_negOne, ndFloat32(0.0f));

	// de coupling chassis.
	ndJacobian& chassisJacobian = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
	chassisJacobian.m_angular = ndVector::m_zero;

	const ndFloat32 accel = CalculateAcceleration(desc);
	const ndFloat32 torque = ndMax(m_engineTorque, m_engineCurve.m_frictionLoss);
	SetMotorAcceleration(desc, accel);
	SetHighFriction(desc, torque);
	SetLowerFriction(desc, -m_engineCurve.m_frictionLoss);
	SetDiagonalRegularizer(desc, ndFloat32(0.001f));
}
