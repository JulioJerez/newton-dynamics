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

#include "ndSandboxStdafx.h"
#include "ndVehicleCommon.h"
#include "ndPhysicsWorld.h"
#include "ndGameControllerInputs.h"

ndVehicleCommonNotify::ndVehicleCommonNotify(ndMultiBodyVehicle* const vehicle)
	:ndModelNotify()
	,m_currentGear(ndMultiBodyVehicleGearBox::ndGearBox::m_neutralGear)
	,m_autoGearShiftTimer(0)
	,m_driverState(m_parked)
	,m_isPlayer(false)
{
	SetModel(vehicle);
}

bool ndVehicleCommonNotify::GetPlayerState() const
{
	return m_isPlayer;
}

void ndVehicleCommonNotify::SetAsPlayer(bool state)
{
	m_isPlayer = state;
}

void ndVehicleCommonNotify::Update(ndFloat32 timestep)
{
	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();
	if (m_isPlayer || (vehicle && !vehicle->IsSleeping()))
	{
		vehicle->Update(timestep);
	}
}

void ndVehicleCommonNotify::PostUpdate(ndFloat32 timestep)
{
	ndModelNotify::PostUpdate(timestep);
	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();
	if (vehicle)
	{
		vehicle->PostUpdate(timestep);
	}
}

void ndVehicleCommonNotify::PostTransformUpdate(ndFloat32 timestep)
{
	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();

	if (m_isPlayer || (vehicle && !vehicle->IsSleeping()))
	{
		ApplyInputs(timestep);
	}
}

void ndVehicleCommonNotify::Debug(ndConstraintDebugCallback& callback) const
{
	ndModelNotify::Debug(callback);
	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();
	if (vehicle)
	{
		vehicle->Debug(callback);
	}
}

void ndVehicleCommonNotify::ApplyInputs(ndFloat32)
{
	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();
	ndMultiBodyVehicleMotor* const motor = vehicle->GetMotor();
	ndMultiBodyVehicleGearBox* const gearJoint = vehicle->GetGearBox();

	if (!(motor && gearJoint))
	{
		return;
	}

	ndPhysicsWorld* const world = (ndPhysicsWorld*)vehicle->GetWorld();
	ndDemoEntityManager* const scene = world->GetManager();

	const ndSharedPtr<ndGameControllerInputs>& gameController = scene->GetGameController();
	const ndFixSizeArray<ndFloat32, 8>& axis = gameController->GetAxis();
	const ndFixSizeArray<bool, 32>& buttons = gameController->GetButtons();

	const ndVehicleDectriptor& desc = vehicle->GetDescriptor();
	auto ApplyControls = [this, vehicle, &desc, &axis, &buttons]()
	{
		ndFloat32 throttle = axis[ndGameControllerInputs::m_gasPedal];
		
		ndMultiBodyVehicleMotor* const motor = vehicle->GetMotor();
		const ndMultiBodyVehicleMotor::ndEngineTorqueCurve& engineCurve = motor->GetCurve();
		ndFloat32 currentRpm = motor->GetRpm();
		ndFloat32 desiredRpm = ndMax(engineCurve.GetIdleRpm(), throttle * engineCurve.GetRedLineRpm());
		ndFloat32 torqueFromCurve = engineCurve.GetTorque(currentRpm);
		ndFloat32 brake = axis[ndGameControllerInputs::m_brakePedal];
		ndFloat32 steerAngle = axis[ndGameControllerInputs::m_steeringWheel];
		ndFloat32 handBrake = buttons[ndGameControllerInputs::m_handBreakButton] ? ndFloat32(1.0f) : ndFloat32(0.0f);

		if (!m_isPlayer)
		{
			brake = ndFloat32(1.0f);
			handBrake = ndFloat32(1.0f);
			desiredRpm = ndFloat32(0.0f);
			torqueFromCurve = ndFloat32(0.0f);

			m_driverState = m_parked;
			ndMultiBodyVehicleGearBox* const gearJoint = vehicle->GetGearBox();
			gearJoint->SetRatio(ndFloat32(0.0f));
			m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_neutralGear;
		}

		ndMultiBodyVehicleGearBox* const gearJoint = vehicle->GetGearBox();
		ndMultiBodyVehicleGearBox::ndGearBox& gearBox = gearJoint->GetGearBox();
		gearJoint->SetClutchTorque(gearBox.m_lockedClutchTorque);
		if ((handBrake > ndFloat32(0.1f)) || (brake > ndFloat32(0.1f)))
		{
			// apply clucth or torque converted here
			// for now just ignore the torque
			gearJoint->SetClutchTorque(gearBox.m_torqueConverter);
		}
		motor->SetTorqueAndRpm(torqueFromCurve, desiredRpm);

		for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = vehicle->GetTireList().GetFirst(); node; node = node->GetNext())
		{
			ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
			tire->SetBrake(brake);
			tire->SetSteering(steerAngle);
			tire->SetHandBrake(handBrake);
		}

		bool aweakeVehicle = false;
		for (ndInt32 i = 0; i < axis.GetCount(); i ++)
		{
			aweakeVehicle = aweakeVehicle || (ndAbs(axis[i]) > ndFloat32(0.01f));
		}
		for (ndInt32 i = 0; i < buttons.GetCount(); i++)
		{
			aweakeVehicle = aweakeVehicle || buttons[i];
		}
		if (aweakeVehicle)
		{
			vehicle->GetRoot()->m_body->GetAsBodyDynamic()->SetSleepState(false);
		}
	};
	ApplyControls();

	if (!m_isPlayer)
	{
		return;
	}

	const ndMultiBodyVehicleGearBox::ndGearBox& gearBox = gearJoint->GetGearBox();
	switch (m_driverState)
	{
		case m_parked:
		{
			gearJoint->SetRatio(0.0f);
			motor->SetTorqueAndRpm(0.0f, 0.0f);
			for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = vehicle->GetTireList().GetFirst(); node; node = node->GetNext())
			{
				ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
				tire->SetBrake(ndFloat32(1.0f));
				tire->SetHandBrake(ndFloat32(1.0f));
			}

			if (m_ignition.Update(buttons[ndGameControllerInputs::m_ignitionButton] ? true : false))
			{
				m_driverState = m_idle;
			}
			break;
		}

		case m_idle:
		{
			if (m_ignition.Update(buttons[ndGameControllerInputs::m_ignitionButton] ? true : false))
			{
				m_driverState = m_parked;
			}

			if (m_forwardGearUp.Update(buttons[ndGameControllerInputs::m_upGearButton] ? true : false))
			{
				// set neutral gear
				gearJoint->SetRatio(0.0f);
				m_driverState = m_driveForward;
				//m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_neutralGear;
			}

			if (m_forwardGearUp.Update(buttons[ndGameControllerInputs::m_downGearButton] ? true : false))
			{
				// set neutral gear
				gearJoint->SetRatio(0.0f);
				m_driverState = m_driveForward;
				//m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_neutralGear;
			}

			else if (m_reverseGear.Update(buttons[ndGameControllerInputs::m_reverseGearButton] ? true : false))
			{
				m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_revertGear;
				ndFloat32 reverseGearRatio = gearBox.m_gearRatios[m_currentGear];
				ndFloat32 gearGain = gearBox.m_crownGearRatio * reverseGearRatio;
				gearJoint->SetRatio(gearGain);
				m_driverState = m_driveReverse;
			}
			m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_neutralGear;

			break;
		}

		case m_driveReverse:
		{
			if (m_ignition.Update(buttons[ndGameControllerInputs::m_ignitionButton] ? true : false))
			{
				gearJoint->SetRatio(0.0f);
				m_driverState = m_idle;
			}
			if (m_reverseGear.Update(buttons[ndGameControllerInputs::m_neutralGearButton] ? true : false))
			{
				gearJoint->SetRatio(0.0f);
				m_driverState = m_idle;
			}
			break;
		}

		case m_driveForward:
		{
			if (m_ignition.Update(buttons[ndGameControllerInputs::m_ignitionButton] ? true : false))
			{
				gearJoint->SetRatio(0.0f);
				m_driverState = m_idle;
			}
			if (m_reverseGear.Update(buttons[ndGameControllerInputs::m_reverseGearButton] ? true : false))
			{
				m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_revertGear;
				ndFloat32 reverseGearRatio = gearBox.m_gearRatios[m_currentGear];
				ndFloat32 gearGain = gearBox.m_crownGearRatio * reverseGearRatio;
				gearJoint->SetRatio(gearGain);
				m_driverState = m_driveReverseFromForward;
			}
			if (m_reverseGear.Update(buttons[ndGameControllerInputs::m_neutralGearButton] ? true : false))
			{
				gearJoint->SetRatio(0.0f);
				m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_neutralGear;
				m_driverState = m_driveForwardGearDelay;
			}

			if (m_forwardGearUp.Update(buttons[ndGameControllerInputs::m_upGearButton] ? true : false))
			{
				m_driverState = m_driveShitGearUp;
			}
			if (m_forwardGearUp.Update(buttons[ndGameControllerInputs::m_downGearButton] ? true : false))
			{
				m_driverState = m_driveShitGearDown;
			}
			break;
		}

		case m_driveReverseFromForward:
		{
			ndFloat32 gearRatio = gearJoint->GetRatio();
			if (gearRatio == ndFloat32(0.0f))
			{
				m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_revertGear;
				ndFloat32 reverseGearRatio = gearBox.m_gearRatios[m_currentGear];
				ndFloat32 gearGain = gearBox.m_crownGearRatio * reverseGearRatio;
				gearJoint->SetRatio(gearGain);
				m_driverState = m_driveReverse;
			}
			else
			{
				m_driverState = m_driveForward;
			}
			break;
		}

		case m_driveShitGearUp:
		{
			ndInt32 neutralGearIndex = ndMultiBodyVehicleGearBox::ndGearBox::m_neutralGear;
			if (m_currentGear == neutralGearIndex)
			{
				m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear;
			}
			else
			{
				m_currentGear++;
				if (m_currentGear >= gearBox.m_gearRatios.GetCount())
				{
					m_currentGear = gearBox.m_gearRatios.GetCount() - 1;
				}
			}
			ndFloat32 gearGain = gearBox.m_crownGearRatio * gearBox.m_gearRatios[m_currentGear];
			gearJoint->SetRatio(gearGain);
			
			m_driverState = m_driveForwardGearDelay;
			m_autoGearShiftTimer = gearBox.m_gearShiftDelayTicks;
			break;
		}

		case m_driveShitGearDown:
		{
			ndInt32 neutralGearIndex = ndMultiBodyVehicleGearBox::ndGearBox::m_neutralGear;
			if (m_currentGear == neutralGearIndex)
			{
				m_currentGear = 0;
				m_currentGear = gearBox.m_gearRatios.GetCount() - 1;
			}
			else
			{
				m_currentGear--;
				if (m_currentGear <= ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear)
				{
					m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear;
				}
			}
			ndFloat32 gearGain = gearBox.m_crownGearRatio * gearBox.m_gearRatios[m_currentGear];
			gearJoint->SetRatio(gearGain);
			
			m_driverState = m_driveForwardGearDelay;
			m_autoGearShiftTimer = gearBox.m_gearShiftDelayTicks;
			break;
		}

		case m_driveForwardGearDelay:
		{
			m_autoGearShiftTimer--;
			if (m_autoGearShiftTimer <= 0)
			{
				m_driverState = m_driveForward;
			}
			break;
		}

		default:
		{
			ndAssert(0);
		}
	}
}
