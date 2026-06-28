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
	m_rpms.SetCount(5);
	m_torques.SetCount(5);

	m_rpms[0] = ndReal(0.0f);
	m_torques[0] = ndReal(idleTorquePoundFoot);

	m_rpms[1] = ndReal(idleRmp);
	m_torques[1] = ndReal(idleTorquePoundFoot);

	ndFloat32 power = horsePower * ndFloat32(746.0f);
	ndFloat32 omegaInRadPerSec = rpm0 * ndFloat32(0.105f);
	ndFloat32 torqueInPoundFood = (power / omegaInRadPerSec) / ndFloat32(1.36f);

	m_rpms[2] = ndReal(rpm0);
	m_torques[2] = ndReal (torqueInPoundFood);

	power = horsePower * ndFloat32(746.0f);
	omegaInRadPerSec = rpm1 / ndRadPerSecToRpm;
	torqueInPoundFood = (power / omegaInRadPerSec) / ndFloat32(1.36f);
	m_rpms[3] = ndReal(rpm1);
	m_torques[3] = ndReal(torqueInPoundFood);

	power = horsePowerAtRedLine * ndFloat32(746.0f);
	omegaInRadPerSec = redLineRpm / ndRadPerSecToRpm;
	torqueInPoundFood = (power / omegaInRadPerSec) / ndFloat32(1.36f);
	m_rpms[4] = ndReal(redLineRpm);
	m_torques[4] = ndReal(torqueInPoundFood);
	m_torques[4] = ndReal(torqueInPoundFood);

	m_omegaStep = ndFloat32(8.0f);
	m_frictionLoss = ndReal(GetTorque(ndFloat32(0.0f)) * ndFloat32(0.5f));
}

bool ndMultiBodyVehicleMotor::ndEngineTorqueCurve::operator==(const ndEngineTorqueCurve& other) const
{
	bool test = m_omegaStep == other.m_omegaStep;
	test = test && (m_frictionLoss == other.m_frictionLoss);
	test = test && (m_rpms.GetCount() == other.m_rpms.GetCount());
	test = test && (m_torques.GetCount() == other.m_torques.GetCount());

	for (ndInt32 i = 0; test && i < m_rpms.GetCount(); ++i)
	{
		test = test && (m_rpms[i] == other.m_rpms[i]);
		test = test && (m_torques[i] == other.m_torques[i]);
	}
	return test;
}

void ndMultiBodyVehicleMotor::ndEngineTorqueCurve::SetOmegaAccel(ndFloat32 rpmStep)
{
	m_omegaStep = ndReal (ndAbs(rpmStep / ndRadPerSecToRpm));
}

ndFloat32 ndMultiBodyVehicleMotor::ndEngineTorqueCurve::GetIdleRpm() const
{
	return m_rpms[1];
}

ndFloat32 ndMultiBodyVehicleMotor::ndEngineTorqueCurve::GetLowGearShiftRpm() const
{
	return m_rpms[2];
}

ndFloat32 ndMultiBodyVehicleMotor::ndEngineTorqueCurve::GetHighGearShiftRpm() const
{
	return m_rpms[3];
}

ndFloat32 ndMultiBodyVehicleMotor::ndEngineTorqueCurve::GetRedLineRpm() const
{
	const ndFloat32 rpm = m_rpms[m_rpms.GetCount() - 1];
	return rpm;
}

ndFloat32 ndMultiBodyVehicleMotor::ndEngineTorqueCurve::GetTorque(ndFloat32 rpm) const
{
	rpm = ndClamp(rpm, ndFloat32(0.0f), ndFloat32(m_rpms[m_rpms.GetCount() - 1]));
	for (ndInt32 i = 1; i < m_rpms.GetCount(); ++i)
	{
		if (rpm <= m_rpms[i])
		{
			const ndFloat32 rpm0 = m_rpms[i - 0];
			const ndFloat32 rpm1 = m_rpms[i - 1];

			const ndFloat32 torque0 = m_torques[i - 0];
			const ndFloat32 torque1 = m_torques[i - 1];

			const ndFloat32 torque = torque0 + (rpm - rpm0) * (torque1 - torque0) / (rpm1 - rpm0);
			return torque;
		}
	}
	return m_torques[m_torques.GetCount() - 1];
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

ndFloat32 ndMultiBodyVehicleMotor::GetMaxRpm() const
{
	//return m_engineCurve.m_omega[m_engineCurve.m_omega.GetCount() - 1] * ndRadPerSecToRpm;
	return m_engineCurve.m_rpms[m_engineCurve.m_rpms.GetCount() - 1];
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

void ndMultiBodyVehicleMotor::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	debugCallback.DrawFrame(matrix0);
	debugCallback.DrawFrame(matrix1);
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
	ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
	jacobian0.m_angular = ndVector::m_zero;

	AddAngularRowJacobian(desc, matrix1.m_right, angle1);
	ndJacobian& jacobian1 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
	jacobian1.m_angular = ndVector::m_zero;
	
	// add rotor joint acceleration
	AddAngularRowJacobian(desc, matrix0.m_front * ndVector::m_negOne, ndFloat32(0.0f));

	// de coupling chassis.
	ndJacobian& chassisJacobian = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
	chassisJacobian.m_angular = ndVector::m_zero;

	const ndFloat32 accel = CalculateAcceleration(desc);
	const ndFloat32 torque = m_engineTorque;
	SetMotorAcceleration(desc, accel);
	SetHighFriction(desc, torque);
	SetLowerFriction(desc, -m_engineCurve.m_frictionLoss);
	SetDiagonalRegularizer(desc, ndFloat32(0.001f));
}
