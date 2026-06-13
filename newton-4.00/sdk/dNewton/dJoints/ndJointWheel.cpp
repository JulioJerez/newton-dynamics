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
#include "ndJointWheel.h"
#include "ndMeshComponents.h"

static ndTireFrictionModel::ndPacejkaTireModel pacejkaSportLateral(ndFloat32(0.2f), ndFloat32(1.5f), ndFloat32(1000.0f), ndFloat32(-0.1f), ndFloat32(0.0f), ndFloat32(0.01f));
static ndTireFrictionModel::ndPacejkaTireModel pacejkaSportLongitudinal(ndFloat32(0.5f), ndFloat32(1.65f), ndFloat32(1000.0f), ndFloat32(0.8f), ndFloat32(0.0f), ndFloat32(0.0f));

static ndTireFrictionModel::ndPacejkaTireModel pacejkaUtilityLateral(ndFloat32(0.14f), ndFloat32(1.35f), ndFloat32(1000.0f), ndFloat32(-0.5f), ndFloat32(0.0f), ndFloat32(0.01f));
static ndTireFrictionModel::ndPacejkaTireModel pacejkaUtilityLongitudinal(ndFloat32(0.2f), ndFloat32(1.4), ndFloat32(1000.0f), ndFloat32(-9.8f), ndFloat32(0.0f), ndFloat32(0.0f));

static ndTireFrictionModel::ndPacejkaTireModel pacejkaTruckLateral(ndFloat32(0.01f), ndFloat32(2.85f), ndFloat32(1000.0f), ndFloat32(1.42f), ndFloat32(0.0f), ndFloat32(0.01f));
static ndTireFrictionModel::ndPacejkaTireModel pacejkaTruckLongitudinal(ndFloat32(0.5f), ndFloat32(1.65f), ndFloat32(1000.0f), ndFloat32(0.8f), ndFloat32(0.0f), ndFloat32(0.0f));

ndTireFrictionModel::ndTireFrictionModel()
	:m_frictionModel(m_pacejkaUtility)
{
	SetPacejkaCurves(m_pacejkaUtility);
}

void ndTireFrictionModel::SetPacejkaCurves(const ndPacejkaTireModel& longitudinal, const ndPacejkaTireModel& lateral)
{
	m_lateralPacejka = lateral;
	m_frictionModel = m_pacejkaCustom;
	m_longitudinalPacejka = longitudinal;
}

void ndTireFrictionModel::SetPacejkaCurves(ndFrictionModel pacejkaStockModel)
{
	switch (pacejkaStockModel)
	{
		case m_pacejkaSport:
			SetPacejkaCurves(pacejkaSportLongitudinal, pacejkaSportLateral);
			m_frictionModel = pacejkaStockModel;
			break;

		case ndTireFrictionModel::m_pacejkaTruck:
			SetPacejkaCurves(pacejkaTruckLongitudinal, pacejkaTruckLateral);
			m_frictionModel = pacejkaStockModel;
			break;

		case ndTireFrictionModel::m_pacejkaUtility:
		default:
			SetPacejkaCurves(pacejkaUtilityLongitudinal, pacejkaUtilityLateral);
			m_frictionModel = pacejkaStockModel;
			break;
	}
}

void ndTireFrictionModel::GetPacejkaCurves(ndFrictionModel pacejkaStockModel, ndPacejkaTireModel& longitudinal, ndPacejkaTireModel& lateral) const
{
	switch (pacejkaStockModel)
	{
		case m_pacejkaSport:
			lateral = pacejkaSportLateral;
			longitudinal = pacejkaSportLongitudinal;
			break;

		case ndTireFrictionModel::m_pacejkaTruck:
			lateral = pacejkaTruckLateral;
			longitudinal = pacejkaTruckLongitudinal;
			break;

		case ndTireFrictionModel::m_pacejkaUtility:
		default:
			lateral = pacejkaUtilityLateral;
			longitudinal = pacejkaUtilityLongitudinal;
			break;
	}
}

void ndTireFrictionModel::PlotPacejkaCurves(const char* const name) const
{
	FILE* outFile;
	char nameExt[256];

	// write as execcl format
	snprintf(nameExt, size_t(nameExt) - 1, "%s.csv", name);

	outFile = fopen(nameExt, "wb");
	fprintf(outFile, "fx; fz; phi\n");
	for (ndFloat32 x = -20.0f; x < 20.0f; x += 0.01f)
	{
		ndFloat32 fx = m_longitudinalPacejka.Evaluate(x, ndFloat32(1.0f));
		ndFloat32 fz = m_lateralPacejka.Evaluate(x, ndFloat32(1.0f));
		fprintf(outFile, "%g; %g; %g\n", fx, fz, x);
	}
	fclose(outFile);
}

ndTireFrictionModel::ndPacejkaTireModel::ndPacejkaTireModel()
	:m_b(ndFloat32(0.5f))
	,m_c(ndFloat32(1.65f))
	,m_d(ndFloat32(1.0f))
	,m_e(ndFloat32(0.8f))
	,m_sv(ndFloat32(0.0f))
	,m_sh(ndFloat32(0.0f))
	,m_normalizingPhi(ndFloat32(1.0f))
	,m_norminalNormalForce(ndFloat32(1000.0f))
{
	// set some defualt values, longitudinal force for a classic tire form Giancalr Genta book.
	CalculateMaxPhi();
}

ndTireFrictionModel::ndPacejkaTireModel::ndPacejkaTireModel(ndFloat32 B, ndFloat32 C, ndFloat32 D, ndFloat32 E, ndFloat32 Sv, ndFloat32 Sh)
	:m_b(B)
	,m_c(C)
	,m_d(1.0f)
	,m_e(E)
	,m_sv(Sv)
	,m_sh(Sh)
	,m_normalizingPhi(ndFloat32(1.0f))
	,m_norminalNormalForce(D)
{
	CalculateMaxPhi();
}

void ndTireFrictionModel::ndPacejkaTireModel::CalculateMaxPhi()
{
	// claculate Max sizeSlipParam
	ndFloat32 maxPhi = ndFloat32(0.0f);
	ndFloat32 maxForce = ndFloat32(0.0f);
	for (ndFloat32 phi = ndFloat32(0.0f); phi < ndFloat32(20.0f); phi += ndFloat32(0.001f))
	{
		ndFloat32 force = Evaluate(phi, ndFloat32(1.0f));
		if (force >= maxForce)
		{
			maxPhi = phi;
			maxForce = force;
		}
	}
	m_normalizingPhi = maxPhi;
}

ndFloat32 ndTireFrictionModel::ndPacejkaTireModel::Evaluate(ndFloat32 phi, ndFloat32 frictionCoefficient) const
{
	ndFloat32 displacedPhi = phi + m_sh;
	ndFloat32 EaTang = m_e * ndAtan(m_b * displacedPhi);
	ndFloat32 BEarg = m_b * (ndFloat32(1.0f) - m_e) * displacedPhi;
	ndFloat32 Carg = m_c * ndAtan(BEarg + EaTang);
	ndFloat32 f = m_d * ndSin(Carg) + m_sv;
	return frictionCoefficient * m_norminalNormalForce * f;
}

ndJointWheel::ndJointWheel()
	:ndJointBilateralConstraint()
	,m_baseFrame(m_localMatrix1)
	,m_info()
	,m_posit(ndFloat32(0.0f))
	,m_speed(ndFloat32(0.0f))
	,m_normalizedBrake(ndFloat32(0.0f))
	,m_normalizedSteering(ndFloat32(0.0f))
	,m_normalizedSteering0(ndFloat32(0.0f))
	,m_normalizedHandBrake(ndFloat32(0.0f))
	,m_variableRateRegularizer(m_info.m_regularizer)
	,m_isApplyingBrakes(false)
{
	m_maxDof = 7;
}

ndJointWheel::ndJointWheel(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent, const ndWheelDescriptor& info)
	:ndJointBilateralConstraint(7, child, parent, pinAndPivotFrame)
	,m_baseFrame(m_localMatrix1)
	,m_info(info)
	,m_posit(ndFloat32 (0.0f))
	,m_speed(ndFloat32(0.0f))
	,m_normalizedBrake(ndFloat32(0.0f))
	,m_normalizedSteering(ndFloat32(0.0f))
	,m_normalizedSteering0(ndFloat32(0.0f))
	,m_normalizedHandBrake(ndFloat32(0.0f))
	,m_variableRateRegularizer(info.m_regularizer)
	,m_isApplyingBrakes(false)
{
}

ndJointWheel::ndJointWheel(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent, const ndWheelDescriptor& desc)
	:ndJointBilateralConstraint(7, child, parent, pinAndPivotInChild, pinAndPivotInParent)
	,m_baseFrame(m_localMatrix1)
	,m_info(desc)
	,m_posit(ndFloat32(0.0f))
	,m_speed(ndFloat32(0.0f))
	,m_normalizedBrake(ndFloat32(0.0f))
	,m_normalizedSteering(ndFloat32(0.0f))
	,m_normalizedSteering0(ndFloat32(0.0f))
	,m_normalizedHandBrake(ndFloat32(0.0f))
	,m_variableRateRegularizer(desc.m_regularizer)
	,m_isApplyingBrakes(false)
{
}

ndJointWheel::ndJointWheel(const ndJointWheel& wheel)
	:ndJointBilateralConstraint(wheel)
	,m_baseFrame(wheel.m_baseFrame)
	,m_info(wheel.m_info)
	,m_posit(wheel.m_posit)
	,m_speed(wheel.m_speed)
	,m_normalizedBrake(wheel.m_normalizedBrake)
	,m_normalizedSteering(wheel.m_normalizedSteering)
	,m_normalizedSteering0(wheel.m_normalizedSteering0)
	,m_normalizedHandBrake(wheel.m_normalizedHandBrake)
	,m_variableRateRegularizer(wheel.m_variableRateRegularizer)
	,m_isApplyingBrakes(wheel.m_isApplyingBrakes)
{
}

ndJointWheel::~ndJointWheel()
{
}

ndSharedPtr<ndMeshJoint> ndJointWheel::GetMeshJoint(const ndMesh* const owner) const
{
	ndMeshJointWheel* const joint = new ndMeshJointWheel(owner, this);
	return ndSharedPtr<ndMeshJoint>(joint);
}

const ndWheelDescriptor& ndJointWheel::GetInfo() const
{
	return m_info;
}

void ndJointWheel::SetInfo(const ndWheelDescriptor& info)
{
	m_info = info;
}

ndFloat32 ndJointWheel::GetPosit() const
{
	return m_posit;
}

ndFloat32 ndJointWheel::SetSpeed() const
{
	return m_speed;
}

ndFloat32 ndJointWheel::GetBreak() const
{
	return m_normalizedBrake;
}

ndFloat32 ndJointWheel::GetSteering() const
{
	return m_normalizedSteering;
}

ndFloat32 ndJointWheel::GetHandBreak() const
{
	return m_normalizedHandBrake;
}

void ndJointWheel::SetBreak(ndFloat32 normalizedBrake)
{
	m_normalizedBrake = ndClamp (normalizedBrake, ndFloat32 (0.0f), ndFloat32 (1.0f));
}

void ndJointWheel::SetHandBreak(ndFloat32 normalizedBrake)
{
	m_normalizedHandBrake = ndClamp(normalizedBrake, ndFloat32(0.0f), ndFloat32(1.0f));
}

void ndJointWheel::SetSteering(ndFloat32 normalidedSteering)
{
	m_normalizedSteering = ndClamp(normalidedSteering, ndFloat32(-1.0f), ndFloat32(1.0f));
}

void ndJointWheel::UpdateTireSteeringAngleMatrix()
{
	ndMatrix tireMatrix;
	ndMatrix chassisMatrix;
	m_localMatrix1 = ndYawMatrix(m_normalizedSteering * m_info.m_steeringAngle) * m_baseFrame;

	CalculateGlobalMatrix(tireMatrix, chassisMatrix);
	const ndVector localRelPosit(chassisMatrix.UntransformVector(tireMatrix.m_posit));
	const ndFloat32 distance = ndClamp(localRelPosit.m_y, m_info.m_lowerStop, m_info.m_upperStop);

	const ndFloat32 spinAngle = -CalculateAngle(tireMatrix.m_up, chassisMatrix.m_up, chassisMatrix.m_front);
	ndMatrix newTireMatrix(ndPitchMatrix(spinAngle) * chassisMatrix);
	newTireMatrix.m_posit = chassisMatrix.m_posit + chassisMatrix.m_up.Scale(distance);

	const ndMatrix tireBodyMatrix(m_localMatrix0.OrthoInverse() * newTireMatrix);
	m_body0->SetMatrix(tireBodyMatrix);
}

ndMatrix ndJointWheel::CalculateBaseFrame() const
{
	return m_localMatrix1 * m_body1->GetMatrix();
}

ndMatrix ndJointWheel::CalculateUpperBumperMatrix() const
{
	ndMatrix matrix(m_localMatrix1 * m_body1->GetMatrix());
	matrix.m_posit += matrix.m_up.Scale(m_info.m_upperStop);
	return matrix;
}

void ndJointWheel::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	//draw reference frame
	debugCallback.DrawFrame(matrix0);

	//draw reference frame
	debugCallback.DrawFrame(matrix1);

	if (m_info.m_steeringAngle > ndFloat32 (0.01f))
	{
		// show yaw angle limits
		const int subdiv = 12;
		ndVector arch[subdiv + 1];
		const ndFloat32 radius = debugCallback.m_debugScale;
		ndVector point(ndFloat32(radius), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));

		ndFloat32 minAngle = -m_info.m_steeringAngle - ndPi * 0.5f;
		ndFloat32 maxAngle = m_info.m_steeringAngle - ndPi * 0.5f;

		ndFloat32 angleStep = (maxAngle - minAngle) / subdiv;
		ndFloat32 angle0 = minAngle;

		ndMatrix steeringMatrix(m_baseFrame * m_body1->GetMatrix());

		ndVector color(ndVector(0.0f, 0.5f, 0.0f, 0.0f));
		for (ndInt32 i = 0; i <= subdiv; ++i)
		{
			arch[i] = steeringMatrix.TransformVector(ndYawMatrix(angle0).RotateVector(point));
			debugCallback.DrawLine(steeringMatrix.m_posit, arch[i], color);
			angle0 += angleStep;
		}

		for (ndInt32 i = 0; i < subdiv; ++i)
		{
			debugCallback.DrawLine(arch[i], arch[i + 1], color);
		}
	}

	// draw bumper limits
	{
		const ndVector lineColor(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(1.0f));
	
		ndVector p0(matrix1.m_posit + matrix1.m_up.Scale(m_info.m_lowerStop));
		ndVector p1(matrix1.m_posit + matrix1.m_up.Scale(m_info.m_upperStop));
		debugCallback.DrawLine(p0, p1, lineColor);

		ndMatrix arrowMatrix(ndRollMatrix(ndFloat32(90.0f) * ndDegreeToRad) * matrix1);
		arrowMatrix.m_posit = p0;
		debugCallback.DrawArrow(arrowMatrix, m_arrowDebugColor, ndFloat32(0.125f));

		arrowMatrix.m_posit = p1;
		debugCallback.DrawArrow(arrowMatrix, m_arrowDebugColor, ndFloat32(-0.125f));
	}
}

void ndJointWheel::UpdateParameters()
{
	// for now do nothing
}

void ndJointWheel::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	// calculate position and speed	
	const ndVector veloc0(m_body0->GetVelocityAtPoint(matrix0.m_posit));
	const ndVector veloc1(m_body1->GetVelocityAtPoint(matrix1.m_posit));

	const ndVector& pin = matrix1[0];
	const ndVector& p0 = matrix0.m_posit;
	const ndVector& p1 = matrix1.m_posit;
	const ndVector prel(p0 - p1);
	const ndVector vrel(veloc0 - veloc1);

	m_speed = vrel.DotProduct(matrix1.m_up).GetScalar();
	m_posit = prel.DotProduct(matrix1.m_up).GetScalar();
	const ndVector projectedPoint = p1 + pin.Scale(pin.DotProduct(prel).GetScalar());

	const ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	const ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);

	AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[0]);
	AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[2]);
	AddAngularRowJacobian(desc, matrix1.m_up, angle0);
	AddAngularRowJacobian(desc, matrix1.m_right, angle1);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_up);
	SetMassSpringDamperAcceleration(desc, m_variableRateRegularizer, m_info.m_springK, m_info.m_damperC);

	m_isApplyingBrakes = false;
	const ndFloat32 brakeFrictionTorque = ndMax(m_normalizedBrake * m_info.m_brakeTorque, m_normalizedHandBrake * m_info.m_handBrakeTorque);
	if (brakeFrictionTorque > ndFloat32(0.0f))
	{
		m_isApplyingBrakes = true;
		const ndFloat32 brakesToChassisInfluence = ndFloat32 (0.125f);

		AddAngularRowJacobian(desc, matrix1.m_front, ndFloat32(0.0f));
		const ndVector tireOmega(m_body0->GetOmega());
		const ndVector chassisOmega(m_body1->GetOmega());

		ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
		ndJacobian& jacobian1 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
		jacobian1.m_angular = jacobian1.m_angular.Scale(brakesToChassisInfluence);

		ndFloat32 w0 = tireOmega.DotProduct(jacobian0.m_angular).GetScalar();
		ndFloat32 w1 = chassisOmega.DotProduct(jacobian1.m_angular).GetScalar();
		ndFloat32 wRel = (w0 + w1) * ndFloat32 (0.35f);
		//ndTrace(("(%d: %f)\n", m_body0->GetId(), wRel));
		SetMotorAcceleration(desc, -wRel * desc.m_invTimestep);
		SetHighFriction(desc, brakeFrictionTorque);
		SetLowerFriction(desc, -brakeFrictionTorque);
	}

	// add suspension limits alone the vertical axis 
	const ndFloat32 x = m_posit + m_speed * desc.m_timestep;
	if (x >= m_info.m_upperStop)
	{
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_up);
		const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		SetMotorAcceleration(desc, stopAccel);
		SetHighFriction(desc, ndFloat32(0.0f));
	}
	else if (x <= m_info.m_lowerStop)
	{
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_up);
		const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		SetMotorAcceleration(desc, stopAccel);
		SetLowerFriction(desc, ndFloat32(0.0f));
	}
}

