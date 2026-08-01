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
	ndFloat32 idleRmp = ndFloat32(1000.0f);
	ndFloat32 horsePower = ndFloat32(400.0f);
	ndFloat32 rpm0 = ndFloat32(5000.0f);
	ndFloat32 rpm1 = ndFloat32(6000.0f);
	ndFloat32 horsePowerAtRedLine = horsePower * ndFloat32 (0.3f);
	ndFloat32 redLineRpm = ndFloat32(7000.0f);

	Init(idleTorquePoundFoot, idleRmp,
		horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);
}

void ndMultiBodyVehicleMotor::ndEngineTorqueCurve::Init(
	ndFloat32 idleTorquePoundFoot, ndFloat32 idleRmp,
	ndFloat32 horsePower, ndFloat32 rpm0, ndFloat32 rpm1,
	ndFloat32 horsePowerAtRedLine, ndFloat32 redLineRpm)
{
	//m_engineOff,
	//m_engineIdle,
	//m_engineMaxTorque,
	//m_engineMaxPower,
	//m_engineRedLine,

	m_rpms.SetCount(m_engineRMPSize);
	m_torques.SetCount(m_engineRMPSize);

	ndFloat32 idleTorqueTorqueNewtonMeter = idleTorquePoundFoot * ndFloat32(1.356f);
	m_rpms[m_engineOff] = ndReal(0.0f);
	m_torques[m_engineOff] = ndReal(idleTorqueTorqueNewtonMeter);

	m_rpms[m_engineIdle] = ndReal(idleRmp);
	m_torques[m_engineIdle] = ndReal(idleTorqueTorqueNewtonMeter);

	ndFloat32 powerInWatts = horsePower * ndFloat32(746.0f);
	ndFloat32 pickOmegaInRadPerSec = rpm0 * ndRpmToRadPerSec;
	ndFloat32 pickToqueInNewtonMetters = powerInWatts / pickOmegaInRadPerSec;
	m_rpms[m_enginePickTorque] = ndReal(rpm0);
	m_torques[m_enginePickTorque] = ndReal (pickToqueInNewtonMetters);

	ndFloat32 omegaAtPickPower = rpm1 * ndRpmToRadPerSec;
	ndFloat32 torqueAtPickPower = powerInWatts / omegaAtPickPower;
	m_rpms[m_enginePickPower] = ndReal(rpm1);
	m_torques[m_enginePickPower] = ndReal(torqueAtPickPower);

	ndFloat32 redLineOmega = redLineRpm * ndRpmToRadPerSec;
	ndFloat32 redLinePowerInWatts = horsePowerAtRedLine * ndFloat32(746.0f);
	ndFloat32 torqueAtRedLine = redLinePowerInWatts / redLineOmega;
	m_rpms[m_engineRedLine] = ndReal(redLineRpm);
	m_torques[m_engineRedLine] = ndReal(torqueAtRedLine);

	m_omegaStep = ndFloat32(200.0f);
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
	return m_rpms[m_engineIdle];
}

ndFloat32 ndMultiBodyVehicleMotor::ndEngineTorqueCurve::GetLowGearShiftRpm() const
{
	const ndFloat32 range = (m_rpms[m_enginePickPower] - m_rpms[m_enginePickTorque]);
	const ndFloat32 minRmp = m_rpms[m_enginePickTorque];
	const ndFloat32 maxRmp = m_rpms[m_enginePickPower];
	const ndFloat32 midRpm = ndFloat32(0.5f) * (minRmp + maxRmp);
	return midRpm - range * ndFloat32(0.5f) * ndFloat32(0.25f);
}

ndFloat32 ndMultiBodyVehicleMotor::ndEngineTorqueCurve::GetHighGearShiftRpm() const
{
	const ndFloat32 range = (m_rpms[m_enginePickPower] - m_rpms[m_enginePickTorque]);
	const ndFloat32 minRmp = m_rpms[m_enginePickTorque];
	const ndFloat32 maxRmp = m_rpms[m_enginePickPower];
	const ndFloat32 midRpm = ndFloat32(0.5f) * (minRmp + maxRmp);
	return midRpm + range * ndFloat32(0.5f) * ndFloat32(0.25f);
}

ndFloat32 ndMultiBodyVehicleMotor::ndEngineTorqueCurve::GetRedLineRpm() const
{
	const ndFloat32 rpm = m_rpms[m_engineRedLine];
	return rpm;
}

ndFloat32 ndMultiBodyVehicleMotor::ndEngineTorqueCurve::GetPickPowerRpm() const
{
	const ndFloat32 rpm = m_rpms[m_enginePickPower];
	return rpm;
}

ndFloat32 ndMultiBodyVehicleMotor::ndEngineTorqueCurve::GetPickTorqueRpm() const
{
	const ndFloat32 rpm = m_rpms[m_enginePickTorque];
	return rpm;
}

ndFloat32 ndMultiBodyVehicleMotor::ndEngineTorqueCurve::GetTorque(ndFloat32 rpm) const
{
	rpm = ndClamp(rpm, ndFloat32(0.0f), ndFloat32(m_rpms[m_engineRedLine]));
	for (ndInt32 i = 1; i < m_rpms.GetCount(); ++i)
	{
		if (rpm <= m_rpms[i])
		{
			const ndFloat32 rpm0 = m_rpms[i - 1];
			const ndFloat32 rpm1 = m_rpms[i - 0];

			const ndFloat32 torque0 = m_torques[i - 1];
			const ndFloat32 torque1 = m_torques[i - 0];

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
	,m_topSpeed(ndFloat32(40.0f))
	,m_targetOmega(ndFloat32(0.0f))
	,m_engineTorque(ndFloat32(0.0f))
{
	m_maxDof = 3;
}

ndMultiBodyVehicleMotor::ndMultiBodyVehicleMotor(ndBodyKinematic* const motor, ndMultiBodyVehicle* const vehicelModel)
	:ndJointBilateralConstraint(3, motor, *vehicelModel->m_chassis, motor->GetMatrix())
	,m_vehicle(vehicelModel)
	,m_omega(ndFloat32(0.0f))
	,m_topSpeed(ndFloat32(40.0f))
	,m_targetOmega(ndFloat32(0.0f))
	,m_engineTorque(ndFloat32(0.0f))
{
}

ndMultiBodyVehicleMotor::ndMultiBodyVehicleMotor(ndBodyKinematic* const motor, ndBodyKinematic* const chassis)
	:ndJointBilateralConstraint(3, motor, chassis, motor->GetMatrix())
	,m_vehicle(nullptr)
	,m_omega(ndFloat32(0.0f))
	,m_topSpeed(ndFloat32(40.0f))
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
	return m_engineCurve.GetRedLineRpm();
}

ndFloat32 ndMultiBodyVehicleMotor::GetTopSpeed() const
{
	return m_topSpeed;
}

void ndMultiBodyVehicleMotor::SetTopSpeed(ndFloat32 topSpeed)
{
	m_topSpeed = ndMax (ndAbs(topSpeed), ndFloat32(10.0f));
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
	m_targetOmega = ndRpmToRadPerSec * ndClamp(rpm, ndFloat32(0.0f), GetMaxRpm()) ;
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

//ndTrace(("%f\n", currentOmega * ndRadPerSecToRpm));

	ndFloat32 gasScale = ndFloat32 (1.0f);
	if (currentOmega < m_engineCurve.GetPickPowerRpm())
	{
		ndFloat32 t0 = m_engineCurve.GetIdleRpm();
		ndFloat32 t1 = m_engineCurve.GetPickTorqueRpm();
		ndFloat32 t2 = m_engineCurve.GetPickPowerRpm();
		gasScale = (t1 - t0) / (t2 - t1);
	}

	m_omega = currentOmega;
	ndFloat32 gasStep = ndRpmToRadPerSec * gasScale * m_engineCurve.m_omegaStep;
	
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
