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
#include "ndMeshComponents.h"
#include "ndMultiBodyVehicle.h"
#include "ndMultiBodyVehicleMotor.h"
#include "ndMultiBodyVehicleGearBox.h"


ndMultiBodyVehicleGearBox::ndGearBox::ndGearBox()
	:m_manual(false)
{
	m_crownGearRatio = ndFloat32(10.0f);
	m_torqueConverter = ndFloat32(2000.0f);
	m_idleClutchTorque = ndFloat32(200.0f);
	m_lockedClutchTorque = ndFloat32(1.0e6f);
	m_gearShiftDelayTicks = 180;

	m_gearRatios.SetCount(m_firstGear);
	m_gearRatios[m_revertGear] = ndReal(-3.0f);
	m_gearRatios[m_neutralGear] = ndReal(0.0f);
	m_gearRatios.PushBack(ndReal(3.0f));
	m_gearRatios.PushBack(ndReal(1.5f));
	m_gearRatios.PushBack(ndReal(1.1f));
	m_gearRatios.PushBack(ndReal(0.8f));
}

bool ndMultiBodyVehicleGearBox::ndGearBox::operator==(const ndGearBox& other) const
{
	bool test = m_crownGearRatio == other.m_crownGearRatio;
	test = test && (m_idleClutchTorque == other.m_idleClutchTorque);
	test = test && (m_lockedClutchTorque == other.m_lockedClutchTorque);
	test = test && (m_torqueConverter == other.m_torqueConverter);
	test = test && (m_gearShiftDelayTicks == other.m_gearShiftDelayTicks);
	test = test && (m_manual == other.m_manual);
	test = test && (m_gearRatios.GetCount() == other.m_gearRatios.GetCount());
	for (ndInt32 i = 0; test && (i < m_gearRatios.GetCount()); ++i)
	{
		test = test && (m_gearRatios[i] == other.m_gearRatios[i]);
	}

	return test;
}

ndMultiBodyVehicleGearBox::ndMultiBodyVehicleGearBox()
	:ndJointGear()
	,m_gearBox()
	,m_idleOmega(ndFloat32(1.0f))
	,m_clutchTorque(ndFloat32(1.0e5f))
	,m_driveTrainResistanceTorque(ndFloat32(1000.0f))
{
}

ndMultiBodyVehicleGearBox::ndMultiBodyVehicleGearBox(ndFloat32 gearRatio,
	const ndVector& motorPin, ndBodyKinematic* const motor,
	const ndVector& differentialPin, ndBodyKinematic* const differential)
	:ndJointGear(gearRatio, differentialPin, differential, motorPin, motor)
	,m_gearBox()
	,m_idleOmega(ndFloat32(1.0f))
	,m_clutchTorque(ndFloat32(1.0e5f))
	,m_driveTrainResistanceTorque(ndFloat32(1000.0f))
{
	SetRatio(ndFloat32(0.0f));
	SetSolverModel(m_jointkinematicCloseLoop);
}

ndSharedPtr<ndMeshJoint> ndMultiBodyVehicleGearBox::GetMeshJoint(const ndMesh* const owner) const
{
	ndMeshJointVehicleGearBox* const joint = new ndMeshJointVehicleGearBox(owner, this);
	return ndSharedPtr<ndMeshJoint>(joint);
}

ndMultiBodyVehicleGearBox::ndGearBox& ndMultiBodyVehicleGearBox::GetGearBox()
{
	return m_gearBox;
}

const ndMultiBodyVehicleGearBox::ndGearBox& ndMultiBodyVehicleGearBox::GetGearBox() const
{
	return m_gearBox;
}

void ndMultiBodyVehicleGearBox::SetGearBox(const ndGearBox& gearBox)
{
	m_gearBox = gearBox;
}

void ndMultiBodyVehicleGearBox::SetIdleOmega(ndFloat32 rpm)
{
	ndAssert(0);
	m_idleOmega = ndMax(rpm / ndRadPerSecToRpm, ndFloat32(0.0f));
}

void ndMultiBodyVehicleGearBox::SetClutchTorque(ndFloat32 torqueInNewtonMeters)
{
	m_clutchTorque = ndAbs(torqueInNewtonMeters);
}

void ndMultiBodyVehicleGearBox::SetInternalTorqueLoss(ndFloat32 torqueInNewtonMeters)
{
	m_driveTrainResistanceTorque = ndAbs(torqueInNewtonMeters);
}

ndFloat32 ndMultiBodyVehicleGearBox::GetIdleOmega() const
{
	return m_idleOmega;
}

ndFloat32 ndMultiBodyVehicleGearBox::GetClutchTorque() const
{
	return m_clutchTorque;
}

ndFloat32 ndMultiBodyVehicleGearBox::GetInternalTorqueLoss() const
{
	return m_driveTrainResistanceTorque;
}

void ndMultiBodyVehicleGearBox::JacobianDerivative(ndConstraintDescritor& desc)
{
	if (ndAbs(m_gearRatio) > ndFloat32(1.0e-2f))
	{
		ndMatrix matrix0;
		ndMatrix matrix1;
		
		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix(matrix0, matrix1);
		
		AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));

		const ndFloat32 gearRatio = m_gearRatio;
		ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
		ndJacobian& jacobian1 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
		jacobian0.m_angular = matrix0.m_front;
		jacobian1.m_angular = matrix1.m_front.Scale(gearRatio);
		
		const ndVector& omega0 = m_body0->GetOmega();
		const ndVector& omega1 = m_body1->GetOmega();
		const ndFloat32 idleOmega = m_idleOmega * gearRatio * ndFloat32(0.95f);

		const ndFloat32 w0 = omega0.DotProduct(jacobian0.m_angular).GetScalar();
		const ndFloat32 w1 = omega1.DotProduct(jacobian1.m_angular).GetScalar() + idleOmega;
		const ndFloat32 w = w0 + w1;
		SetMotorAcceleration(desc, -w * desc.m_invTimestep);
		
		if (m_gearBox.m_crownGearRatio > 0)
		{
			ndAssert(0);
			if (m_gearRatio > ndFloat32(0.0f))
			{
				SetHighFriction(desc, m_clutchTorque);
				SetLowerFriction(desc, -m_driveTrainResistanceTorque);
			}
			else
			{
				//SetHighFriction(desc, m_driveTrainResistanceTorque);
				//SetLowerFriction(desc, -m_clutchTorque);

				SetHighFriction(desc, m_clutchTorque);
				//SetLowerFriction(desc, -0.01f);
			}
		}
		else
		{
			if (m_gearRatio > ndFloat32(0.0f))
			{
				SetHighFriction(desc, m_driveTrainResistanceTorque);
				SetLowerFriction(desc, -m_clutchTorque);
			}
			else
			{
				SetHighFriction(desc, m_clutchTorque);
				SetLowerFriction(desc, -m_driveTrainResistanceTorque);
			}
		}
	}
}
