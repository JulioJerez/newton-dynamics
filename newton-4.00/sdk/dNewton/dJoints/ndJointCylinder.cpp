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
#include "ndJointCylinder.h"

#define D_MAX_SLIDER_RECOVERY_SPEED	ndFloat32 (0.5f)
#define D_MAX_SLIDER_PENETRATION	ndFloat32 (0.05f)

#define D_MAX_HINGE_RECOVERY_SPEED	ndFloat32 (0.25f)
#define D_MAX_HINGE_PENETRATION		(ndFloat32 (4.0f) * ndDegreeToRad)

ndJointCylinder::ndJointCylinder()
	:ndJointBilateralConstraint()
	,m_rotationAxis()
	,m_posit(ndFloat32(0.0f))
	,m_speed(ndFloat32(0.0f))
	,m_springKPosit(ndFloat32(0.0f))
	,m_damperCPosit(ndFloat32(0.0f))
	,m_minLimitPosit(ndFloat32(-1.0e10f))
	,m_maxLimitPosit(ndFloat32(1.0e10f))
	,m_offsetPosit(ndFloat32(0.0f))
	,m_springDamperRegularizerPosit(ndFloat32(0.1f))
	,m_limitStatePosit(0)
{
	m_maxDof = 8;
}

ndJointCylinder::ndJointCylinder(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(8, child, parent, pinAndPivotFrame)
	,m_rotationAxis()
	,m_posit(ndFloat32(0.0f))
	,m_speed(ndFloat32(0.0f))
	,m_springKPosit(ndFloat32(0.0f))
	,m_damperCPosit(ndFloat32(0.0f))
	,m_minLimitPosit(ndFloat32(-1.0e10f))
	,m_maxLimitPosit(ndFloat32(1.0e10f))
	,m_offsetPosit(ndFloat32(0.0f))
	,m_springDamperRegularizerPosit(ndFloat32(0.1f))
	,m_limitStatePosit(0)
{
}

ndJointCylinder::ndJointCylinder(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(8, child, parent, pinAndPivotInChild)
	,m_rotationAxis()
	,m_posit(ndFloat32(0.0f))
	,m_speed(ndFloat32(0.0f))
	,m_springKPosit(ndFloat32(0.0f))
	,m_damperCPosit(ndFloat32(0.0f))
	,m_minLimitPosit(ndFloat32(-1.0e10f))
	,m_maxLimitPosit(ndFloat32(1.0e10f))
	,m_offsetPosit(ndFloat32(0.0f))
	,m_springDamperRegularizerPosit(ndFloat32(0.1f))
	,m_limitStatePosit(0)
{
	ndMatrix tmp;
	CalculateLocalMatrix(pinAndPivotInChild, m_localMatrix0, tmp);
	CalculateLocalMatrix(pinAndPivotInParent, tmp, m_localMatrix1);
}

ndJointCylinder::~ndJointCylinder()
{
}

ndFloat32 ndJointCylinder::GetAngle() const
{
	return m_rotationAxis.m_angle;
}

ndFloat32 ndJointCylinder::GetOmega() const
{
	return m_rotationAxis.m_omega;
}

bool ndJointCylinder::GetLimitStateAngle() const
{
	return m_rotationAxis.m_limitState;
}

void ndJointCylinder::SetLimitStateAngle(bool state)
{
	m_rotationAxis.m_limitState = state;
	if (m_rotationAxis.m_limitState)
	{
		SetLimitsAngle(m_rotationAxis.m_minLimit, m_rotationAxis.m_maxLimit);
	}
}

void ndJointCylinder::SetLimitsAngle(ndFloat32 minLimit, ndFloat32 maxLimit)
{
#ifdef _DEBUG
	if (minLimit > 0.0f)
	{
		ndTrace(("warning: %s minLimit %f larger than zero\n", __FUNCTION__, minLimit))
	}
	if (maxLimit < 0.0f)
	{
		ndTrace(("warning: %s m_maxLimit %f smaller than zero\n", __FUNCTION__, maxLimit))
	}
#endif

	m_rotationAxis.m_minLimit = minLimit;
	m_rotationAxis.m_maxLimit = maxLimit;

	if (m_rotationAxis.m_angle > m_rotationAxis.m_maxLimit)
	{
		//const ndFloat32 deltaAngle = ndAnglesAdd(m_angle, -m_maxLimitAngle);
		//m_angle = m_maxLimitAngle + deltaAngle;
		m_rotationAxis.m_angle = m_rotationAxis.m_maxLimit;
	} 
	else if (m_rotationAxis.m_angle < m_rotationAxis.m_minLimit)
	{
		//const ndFloat32 deltaAngle = ndAnglesAdd(m_angle, -m_minLimitAngle);
		//m_angle = m_minLimitAngle + deltaAngle;
		m_rotationAxis.m_angle = m_rotationAxis.m_minLimit;
	}
}

void ndJointCylinder::GetLimitsAngle(ndFloat32& minLimit, ndFloat32& maxLimit) const
{
	minLimit = m_rotationAxis.m_minLimit;
	maxLimit = m_rotationAxis.m_maxLimit;
}

ndFloat32 ndJointCylinder::GetOffsetAngle() const
{
	return m_rotationAxis.m_targetAngle;
}

void ndJointCylinder::SetOffsetAngle(ndFloat32 angle)
{
	m_rotationAxis.m_targetAngle = angle;
}

void ndJointCylinder::SetAsSpringDamperAngle(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_rotationAxis.m_springK = ndAbs(spring);
	m_rotationAxis.m_damperC = ndAbs(damper);
	m_rotationAxis.m_springDamperRegularizer = ndClamp(regularizer, ndFloat32(1.0e-2f), ndFloat32(0.99f));
}

void ndJointCylinder::GetSpringDamperAngle(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const
{
	spring = m_rotationAxis.m_springK;
	damper = m_rotationAxis.m_damperC;
	regularizer = m_rotationAxis.m_springDamperRegularizer;
}

ndFloat32 ndJointCylinder::GetPosit() const
{
	return m_posit;
}

ndFloat32 ndJointCylinder::GetTargetPosit() const
{
	return m_offsetPosit;
}

void ndJointCylinder::SetTargetPosit(ndFloat32 offset)
{
	m_offsetPosit = offset;
}

bool ndJointCylinder::GetLimitStatePosit() const
{
	return m_limitStatePosit ? true : false;
}

void ndJointCylinder::SetLimitStatePosit(bool state)
{
	m_limitStatePosit = state ? 1 : 0;
	if (m_limitStatePosit)
	{
		SetLimitsPosit(m_minLimitPosit, m_maxLimitPosit);
	}
}

void ndJointCylinder::SetLimitsPosit(ndFloat32 minLimit, ndFloat32 maxLimit)
{
	ndAssert(minLimit <= 0.0f);
	ndAssert(maxLimit >= 0.0f);

#ifdef _DEBUG
	if (minLimit > 0.0f)
	{
		ndTrace(("warning: %s minLimit %f larger than zero\n", __FUNCTION__, minLimit))
	}
	if (maxLimit < 0.0f)
	{
		ndTrace(("warning: %s m_maxLimit %f smaller than zero\n", __FUNCTION__, maxLimit))
	}
#endif

	m_minLimitPosit = minLimit;
	m_maxLimitPosit = maxLimit;
	if (m_posit > m_maxLimitPosit)
	{
		m_posit = m_maxLimitPosit;
	}
	else if (m_posit < m_minLimitPosit)
	{
		m_posit = m_minLimitPosit;
	}
}

void ndJointCylinder::GetLimitsPosit(ndFloat32& minLimit, ndFloat32& maxLimit) const
{
	minLimit = m_minLimitPosit;
	maxLimit = m_maxLimitPosit;
}

void ndJointCylinder::SetAsSpringDamperPosit(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_springKPosit = ndAbs(spring);
	m_damperCPosit = ndAbs(damper);
	m_springDamperRegularizerPosit = ndClamp(regularizer, ndFloat32(1.0e-2f), ndFloat32(0.99f));
}

void ndJointCylinder::GetSpringDamperPosit(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const
{
	spring = m_springKPosit;
	damper = m_damperCPosit;
	regularizer = m_springDamperRegularizerPosit;
}

void ndJointCylinder::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	debugCallback.DrawFrame(matrix0);
	debugCallback.DrawFrame(matrix1);

	const ndInt32 subdiv = 8;
	const ndFloat32 radius = debugCallback.m_debugScale;
	ndVector arch[subdiv + 1];

	ndFloat32 deltaTwist = m_rotationAxis.m_maxLimit - m_rotationAxis.m_minLimit;
	if ((deltaTwist > ndFloat32(1.0e-3f)) && (deltaTwist <= ndFloat32(2.0f) * ndPi))
	{
		ndMatrix pitchMatrix(matrix1);
		pitchMatrix.m_posit = matrix1.m_posit;

		ndVector point(ndFloat32(0.0f), ndFloat32(radius), ndFloat32(0.0f), ndFloat32(0.0f));

		ndFloat32 angleStep = ndMin(deltaTwist, ndFloat32(2.0f * ndPi)) / subdiv;
		ndFloat32 angle0 = m_rotationAxis.m_minLimit;

		ndVector color(ndFloat32(0.4f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		for (ndInt32 i = 0; i <= subdiv; ++i)
		{
			arch[i] = pitchMatrix.TransformVector(ndPitchMatrix(angle0).RotateVector(point));
			debugCallback.DrawLine(pitchMatrix.m_posit, arch[i], color);
			angle0 += angleStep;
		}

		for (ndInt32 i = 0; i < subdiv; ++i)
		{
			debugCallback.DrawLine(arch[i], arch[i + 1], color);
		}
	}
}

void ndJointCylinder::SubmitSpringDamperAngle(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& )
{
	// add spring damper row
	AddAngularRowJacobian(desc, matrix0.m_front, m_rotationAxis.m_targetAngle - m_rotationAxis.m_angle);
	SetMassSpringDamperAcceleration(desc, m_rotationAxis.m_springDamperRegularizer, m_rotationAxis.m_springK, m_rotationAxis.m_damperC);
}

void ndJointCylinder::SubmitSpringDamperPosit(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	// add spring damper row
	const ndVector p1(matrix1.m_posit + matrix1.m_front.Scale(m_offsetPosit));
	AddLinearRowJacobian(desc, matrix0.m_posit, p1, matrix1.m_front);
	SetMassSpringDamperAcceleration(desc, m_springDamperRegularizerPosit, m_springKPosit, m_damperCPosit);
}

void ndJointCylinder::ApplyBaseRows(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	const ndVector veloc0(m_body0->GetVelocityAtPoint(matrix0.m_posit));
	const ndVector veloc1(m_body1->GetVelocityAtPoint(matrix1.m_posit));

	const ndVector& pin = matrix1[0];
	const ndVector& p0 = matrix0.m_posit;
	const ndVector& p1 = matrix1.m_posit;
	const ndVector prel(p0 - p1);
	const ndVector vrel(veloc0 - veloc1);

	m_speed = vrel.DotProduct(matrix1.m_front).GetScalar();
	m_posit = prel.DotProduct(matrix1.m_front).GetScalar();
	const ndVector projectedPoint = p1 + pin.Scale(pin.DotProduct(prel).GetScalar());

	AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[1]);
	AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[2]);

	const ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	AddAngularRowJacobian(desc, matrix1.m_up, angle1);

	const ndFloat32 angle2 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	AddAngularRowJacobian(desc, matrix1.m_right, angle2);
	
	// save the current joint Omega
	const ndVector omega0(m_body0->GetOmega());
	const ndVector omega1(m_body1->GetOmega());
	
	//// the joint angle can be determined by getting the angle between any two non parallel vectors
	//const ndFloat32 deltaAngle = ndAnglesAdd(-CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front), -m_angle);
	//m_angle += deltaAngle;
	//m_omega = matrix1.m_front.DotProduct(omega0 - omega1).GetScalar();
}

ndFloat32 ndJointCylinder::PenetrationOmega(ndFloat32 penetration) const
{
	ndFloat32 param = ndClamp(penetration, ndFloat32(0.0f), D_MAX_HINGE_PENETRATION) / D_MAX_HINGE_PENETRATION;
	ndFloat32 omega = D_MAX_HINGE_RECOVERY_SPEED * param;
	return omega;
}

void ndJointCylinder::SubmitLimitsAngle(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	if (m_rotationAxis.m_limitState)
	{
		if ((m_rotationAxis.m_minLimit > (ndFloat32(-1.0f) * ndDegreeToRad)) && (m_rotationAxis.m_maxLimit < (ndFloat32(1.0f) * ndDegreeToRad)))
		{
			AddAngularRowJacobian(desc, &matrix1.m_front[0], -m_rotationAxis.m_angle);
		}
		else
		{
			const ndFloat32 angle = m_rotationAxis.m_angle + m_rotationAxis.m_omega * desc.m_timestep;
			if (angle < m_rotationAxis.m_minLimit)
			{
				AddAngularRowJacobian(desc, &matrix0.m_front[0], ndFloat32(0.0f));
				const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
				const ndFloat32 penetration = angle - m_rotationAxis.m_minLimit;
				const ndFloat32 recoveringAceel = -desc.m_invTimestep * PenetrationOmega(-penetration);
				SetMotorAcceleration(desc, stopAccel - recoveringAceel);
				SetLowerFriction(desc, ndFloat32(0.0f));
			}
			else if (angle > m_rotationAxis.m_maxLimit)
			{
				AddAngularRowJacobian(desc, &matrix0.m_front[0], ndFloat32(0.0f));
				const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
				const ndFloat32 penetration = angle - m_rotationAxis.m_maxLimit;
				const ndFloat32 recoveringAceel = desc.m_invTimestep * PenetrationOmega(penetration);
				SetMotorAcceleration(desc, stopAccel - recoveringAceel);
				SetHighFriction(desc, ndFloat32(0.0f));
			}
		}
	}
}

ndFloat32 ndJointCylinder::PenetrationSpeed(ndFloat32 penetration) const
{
	ndFloat32 param = ndClamp(penetration, ndFloat32(0.0f), D_MAX_SLIDER_PENETRATION) / D_MAX_SLIDER_PENETRATION;
	ndFloat32 speed = D_MAX_SLIDER_RECOVERY_SPEED * param;
	return speed;
}

void ndJointCylinder::ClearMemory()
{
	ndJointBilateralConstraint::ClearMemory();

	UpdateParameters();
	m_offsetPosit = m_posit;
	m_rotationAxis.m_targetAngle = m_rotationAxis.m_angle;
}

void ndJointCylinder::UpdateParameters()
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	const ndVector veloc0(m_body0->GetVelocityAtPoint(matrix0.m_posit));
	const ndVector veloc1(m_body1->GetVelocityAtPoint(matrix1.m_posit));

	const ndVector& p0 = matrix0.m_posit;
	const ndVector& p1 = matrix1.m_posit;
	const ndVector prel(p0 - p1);
	const ndVector vrel(veloc0 - veloc1);

	m_speed = vrel.DotProduct(matrix1.m_front).GetScalar();
	m_posit = prel.DotProduct(matrix1.m_front).GetScalar();

	const ndVector omega0(m_body0->GetOmega());
	const ndVector omega1(m_body1->GetOmega());

	// the joint angle can be determined by getting the angle between any two non parallel vectors
	const ndFloat32 deltaAngle = ndAnglesAdd(-CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front), -m_rotationAxis.m_angle);
	m_rotationAxis.m_angle += deltaAngle;
	m_rotationAxis.m_omega = matrix1.m_front.DotProduct(omega0 - omega1).GetScalar();
}

void ndJointCylinder::SubmitLimitsPosit(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	if (m_limitStatePosit)
	{
		if ((m_minLimitPosit == ndFloat32(0.0f)) && (m_maxLimitPosit == ndFloat32(0.0f)))
		{
			AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_front);
		}
		else
		{
			ndFloat32 x = m_posit + m_speed * desc.m_timestep;
			if (x < m_minLimitPosit)
			{
				ndVector p1(matrix1.m_posit + matrix1.m_front.Scale(m_minLimitPosit));
				AddLinearRowJacobian(desc, matrix0.m_posit, p1, matrix1.m_front);
				const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
				const ndFloat32 penetration = x - m_minLimitPosit;
				const ndFloat32 recoveringAceel = -desc.m_invTimestep * PenetrationSpeed(-penetration);
				SetMotorAcceleration(desc, stopAccel - recoveringAceel);
				SetLowerFriction(desc, ndFloat32(0.0f));
			}
			else if (x > m_maxLimitPosit)
			{
				AddLinearRowJacobian(desc, matrix0.m_posit, matrix0.m_posit, matrix1.m_front);
				const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
				const ndFloat32 penetration = x - m_maxLimitPosit;
				const ndFloat32 recoveringAceel = desc.m_invTimestep * PenetrationSpeed(penetration);
				SetMotorAcceleration(desc, stopAccel - recoveringAceel);
				SetHighFriction(desc, ndFloat32(0.0f));
			}
		}
	}
}

void ndJointCylinder::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	ApplyBaseRows(desc, matrix0, matrix1);

	if (m_rotationAxis.m_springDamperRegularizer && ((m_rotationAxis.m_springK > ndFloat32(0.0f)) || (m_rotationAxis.m_damperC > ndFloat32(0.0f))))
	{
		// spring damper with limits
		SubmitSpringDamperAngle(desc, matrix0, matrix1);
	}

	if (m_springDamperRegularizerPosit && ((m_springKPosit > ndFloat32(0.0f)) || (m_damperCPosit > ndFloat32(0.0f))))
	{
		// spring damper with limits
		SubmitSpringDamperPosit(desc, matrix0, matrix1);
	}

	SubmitLimitsPosit(desc, matrix0, matrix1);
	SubmitLimitsAngle(desc, matrix0, matrix1);
}