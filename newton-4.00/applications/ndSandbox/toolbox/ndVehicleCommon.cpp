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
#include "ndDemoEntityManager.h"
#include "ndGameControllerInputs.h"

ndVehicleCommonNotify::ndVehicleCommonNotify(ndMultiBodyVehicle* const vehicle)
	:ndModelNotify()
	,m_currentGear(ndMultiBodyVehicleGearBox::ndGearBox::m_neutralGear)
	,m_autoGearShiftTimer(0)
	,m_driverState(m_parked)
	,m_transmission(m_manual)
	,m_isPlayer(false)
	,m_engineOn(false)
{
	SetModel(vehicle);
}

bool ndVehicleCommonNotify::GetPlayerState() const
{
	return m_isPlayer;
}

bool ndVehicleCommonNotify::EngineOn() const
{
	return m_engineOn;
}

void ndVehicleCommonNotify::SetAsPlayer(bool state)
{
	m_isPlayer = state;
}

void ndVehicleCommonNotify::Update(ndFloat32 timestep, ndInt32 threadId)
{
	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();
	if (vehicle)
	{
		vehicle->Update(timestep, threadId);
	}
}

void ndVehicleCommonNotify::PostUpdate(ndFloat32 timestep, ndInt32 threadId)
{
	ndModelNotify::PostUpdate(timestep, threadId);
	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();
	if (vehicle)
	{
		vehicle->PostUpdate(timestep, threadId);
	}
}

void ndVehicleCommonNotify::PostTransformUpdate(ndFloat32 timestep, ndInt32)
{
	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();
	if (m_isPlayer || vehicle)
	{
		ApplyInputs(timestep);
	}
	vehicle->TransformUpdate(timestep);
}

void ndVehicleCommonNotify::Debug(ndConstraintDebugCallback& callback) const
{
	ndModelNotify::Debug(callback);
	ndMultiBodyVehicle* const vehicle = (ndMultiBodyVehicle*)GetModel();
	if (m_isPlayer && vehicle)
	{
		vehicle->Debug(callback);
	}
}

void ndVehicleCommonNotify::ApplyInputs(ndFloat32)
{
	ndMultiBodyVehicle* const vehicle = GetModel()->GetAsMultiBodyVehicle();
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

	auto ApplyControls = [this, vehicle, &axis, &buttons]()
	{
		ndFloat32 throttle = axis[ndGameControllerInputs::m_gasPedal];

		ndMultiBodyVehicleMotor* const motor = vehicle->GetMotor();
		const ndMultiBodyVehicleMotor::ndEngineTorqueCurve& engineCurve = motor->GetCurve();
		ndFloat32 currentRpm = motor->GetRpm();
		ndFloat32 idleRpm = engineCurve.GetIdleRpm();
		ndFloat32 desiredRpm = ndMax(idleRpm, throttle * engineCurve.GetRedLineRpm());
		ndFloat32 torqueFromCurve = engineCurve.GetTorque(currentRpm);
		ndFloat32 brake = axis[ndGameControllerInputs::m_brakePedal];
		ndFloat32 steerAngle = axis[ndGameControllerInputs::m_steeringWheel];
		ndFloat32 handBrake = buttons[ndGameControllerInputs::m_handBreakButton] ? ndFloat32(1.0f) : ndFloat32(0.0f);

		if (!m_isPlayer)
		{
			brake = ndFloat32(1.0f);
			handBrake = ndFloat32(1.0f);
			desiredRpm = ndFloat32(0.0f);
			steerAngle = ndFloat32(0.0f);
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
			gearJoint->SetClutchTorque(0.0f);
		}
		else 
		{
			gearJoint->SetClutchTorque(gearBox.m_torqueConverter);
		}
		motor->SetTorqueAndRpm(torqueFromCurve, desiredRpm);

		for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = vehicle->GetTireList().GetFirst(); node; node = node->GetNext())
		{
			ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
			tire->SetBrake(brake);
			tire->SetHandBrake(handBrake);
			ndFloat32 steerError = tire->GetSteering() - steerAngle;
			if (ndAbs(steerError) > ndFloat32(1.0e-2f))
			{
				tire->GetBody0()->SetSleepState(false);
			}
			tire->SetSteering(steerAngle);
		}

		if (m_isPlayer)
		{
			bool aweakeVehicle = ndAbs(currentRpm - idleRpm) > ndFloat32(1.0f);
			for (ndInt32 i = 0; !aweakeVehicle && (i < ndGameControllerInputs::m_usedAxisCount); i++)
			{
				aweakeVehicle = aweakeVehicle || (ndAbs(axis[i]) > ndFloat32(0.01f));
			}
			for (ndInt32 i = 0; !aweakeVehicle && (i < buttons.GetCount()); i++)
			{
				aweakeVehicle = aweakeVehicle || buttons[i];
			}
			
			if (aweakeVehicle)
			{
				vehicle->GetRoot()->m_body->GetAsBodyDynamic()->SetSleepState(false);
			}
		}
	};
	ApplyControls();

	if (!m_isPlayer)
	{
		return;
	}

	const ndMultiBodyVehicleGearBox::ndGearBox& gearBox = gearJoint->GetGearBox();
	const ndMultiBodyVehicleMotor::ndEngineTorqueCurve& engineCurve = motor->GetCurve();

//m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear;
//m_driverState = m_driveForwardGearDelay;
//m_autoGearShiftTimer = gearBox.m_gearShiftDelayTicks;
//ndFloat32 gearGain = gearBox.m_crownGearRatio * gearBox.m_gearRatios[m_currentGear];
//gearJoint->SetRatio(gearGain);
//return;


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
				m_engineOn = true;
				m_driverState = m_idle;
			}
			break;
		}

		case m_idle:
		{
			gearJoint->SetRatio(0.0f);
			m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_neutralGear;

			if (m_ignition.Update(buttons[ndGameControllerInputs::m_ignitionButton] ? true : false))
			{
				m_engineOn = false;
				m_driverState = m_parked;
			}

			if (m_manualTransmission.Update(buttons[ndGameControllerInputs::m_automaticGearBoxButton] ? true : false))
			{
				m_transmission = m_automatic;
				m_driverState = m_driveAutoGear;
				m_autoGearShiftTimer = 0;
				m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear;
				ndFloat32 gearGain = gearBox.m_crownGearRatio * gearBox.m_gearRatios[m_currentGear];
				gearJoint->SetRatio(gearGain);
			}

			if (m_forwardGearUp.Update(buttons[ndGameControllerInputs::m_upGearButton] ? true : false))
			{
				m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear;
				m_driverState = m_driveForwardGearDelay;
				m_autoGearShiftTimer = gearBox.m_gearShiftDelayTicks;
				ndFloat32 gearGain = gearBox.m_crownGearRatio * gearBox.m_gearRatios[m_currentGear];
				gearJoint->SetRatio(gearGain);
			}

			if (m_reverseGear.Update(buttons[ndGameControllerInputs::m_reverseGearButton] ? true : false))
			{
				m_driverState = m_driveReverse;
				m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_revertGear;
				ndFloat32 gearGain = gearBox.m_crownGearRatio * gearBox.m_gearRatios[m_currentGear];
				gearJoint->SetRatio(gearGain);
			}
			break;
		}

		// automatic transmission state machine
		case m_driveAutoGear:
		{
			if (m_ignition.Update(buttons[ndGameControllerInputs::m_ignitionButton] ? true : false))
			{
				gearJoint->SetRatio(0.0f);
				m_driverState = m_idle;
			}

			if (m_neutralGear.Update(buttons[ndGameControllerInputs::m_neutralGearButton] ? true : false))
			{
				gearJoint->SetRatio(0.0f);
				m_driverState = m_idle;
			}

			if (m_manualTransmission.Update(buttons[ndGameControllerInputs::m_automaticGearBoxButton] ? true : false))
			{
				m_transmission = m_manual;
				m_driverState = m_driveForward;
			}

			if (m_reverseGear.Update(buttons[ndGameControllerInputs::m_reverseGearButton] ? true : false))
			{
				if (ndAbs(vehicle->GetSpeed()) < ndFloat32 (1.0f))
				{
					m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_revertGear;
					ndFloat32 reverseGearRatio = gearBox.m_gearRatios[m_currentGear];
					ndFloat32 gearGain = gearBox.m_crownGearRatio * reverseGearRatio;
					gearJoint->SetRatio(gearGain);
					m_driverState = m_driveAutoReverse;
				}
			}

			if (m_forwardGearUp.Update(buttons[ndGameControllerInputs::m_upGearButton] ? true : false))
			{
				if (m_currentGear < (gearBox.m_gearRatios.GetCount() - 1))
				{
					m_currentGear++;
					m_driverState = m_driveAutoShiftGearUp;
					m_autoGearShiftTimer = gearBox.m_gearShiftDelayTicks * 1;
					ndFloat32 gearGain = gearBox.m_crownGearRatio * gearBox.m_gearRatios[m_currentGear];
					gearJoint->SetRatio(gearGain);
				}
			}
			else if (m_forwardGearDown.Update(buttons[ndGameControllerInputs::m_downGearButton] ? true : false))
			{
				if (m_currentGear > ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear)
				{
					m_currentGear--;
					m_driverState = m_driveAutoShiftGearDown;
					m_autoGearShiftTimer = gearBox.m_gearShiftDelayTicks * 1;
					ndFloat32 gearGain = gearBox.m_crownGearRatio * gearBox.m_gearRatios[m_currentGear];
					gearJoint->SetRatio(gearGain);
				}
			}
			else if (m_autoGearShiftTimer <= 0)
			{
				ndFloat32 rmp = motor->GetRpm();
				ndFloat32 lowRpm = engineCurve.GetLowGearShiftRpm();
				ndFloat32 highRpm = engineCurve.GetHighGearShiftRpm();
				if (rmp >= highRpm)
				{
					if (m_currentGear < (gearBox.m_gearRatios.GetCount() - 1))
					{
						m_currentGear++;
						m_driverState = m_driveAutoShiftGearUp;
						m_autoGearShiftTimer = gearBox.m_gearShiftDelayTicks * 1;
						ndFloat32 gearGain = gearBox.m_crownGearRatio * gearBox.m_gearRatios[m_currentGear];
						gearJoint->SetRatio(gearGain);
					}
				}
				else if (rmp <= lowRpm)
				{
					if (m_currentGear > ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear)
					{
						m_currentGear--;
						m_driverState = m_driveAutoShiftGearDown;
						m_autoGearShiftTimer = gearBox.m_gearShiftDelayTicks * 1;
						ndFloat32 gearGain = gearBox.m_crownGearRatio * gearBox.m_gearRatios[m_currentGear];
						gearJoint->SetRatio(gearGain);
					}
				}
			}
			break;
		}

		case m_driveAutoReverse:
		{
			if (m_ignition.Update(buttons[ndGameControllerInputs::m_ignitionButton] ? true : false))
			{
				gearJoint->SetRatio(0.0f);
				m_driverState = m_idle;
			}
			if (m_neutralGear.Update(buttons[ndGameControllerInputs::m_neutralGearButton] ? true : false))
			{
				gearJoint->SetRatio(0.0f);
				m_driverState = m_idle;
			}
			if (m_manualTransmission.Update(buttons[ndGameControllerInputs::m_automaticGearBoxButton] ? true : false))
			{
				if (ndAbs(vehicle->GetSpeed()) < ndFloat32(1.0f))
				{
					m_transmission = m_automatic;
					m_driverState = m_driveAutoGear;
					m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear;
					ndFloat32 gearGain = gearBox.m_crownGearRatio * gearBox.m_gearRatios[m_currentGear];
					gearJoint->SetRatio(gearGain);
				}
			}
			break;
		}

		case m_driveAutoShiftGearUp:
		{
			if (m_manualTransmission.Update(buttons[ndGameControllerInputs::m_automaticGearBoxButton] ? true : false))
			{
				m_transmission = m_manual;
				m_driverState = m_driveForward;
				break;
			}
			if (m_ignition.Update(buttons[ndGameControllerInputs::m_ignitionButton] ? true : false))
			{
				gearJoint->SetRatio(0.0f);
				m_driverState = m_idle;
				break;
			}
		
			m_autoGearShiftTimer--;
			if (m_autoGearShiftTimer <= 0)
			{
				// apply hysteresis. 
				ndFloat32 rmp = motor->GetRpm();
				ndFloat32 lowRpm = engineCurve.GetLowGearShiftRpm();
				if ((m_autoGearShiftTimer < -120) || (rmp > lowRpm))
				{
					m_driverState = m_driveAutoGear;
				}
			}
			break;
		}

		case m_driveAutoShiftGearDown:
		{
			if (m_manualTransmission.Update(buttons[ndGameControllerInputs::m_automaticGearBoxButton] ? true : false))
			{
				m_transmission = m_manual;
				m_driverState = m_driveForward;
				break;
			}
			if (m_ignition.Update(buttons[ndGameControllerInputs::m_ignitionButton] ? true : false))
			{
				gearJoint->SetRatio(0.0f);
				m_driverState = m_idle;
				break;
			}

			m_autoGearShiftTimer--;
			if (m_autoGearShiftTimer <= 0)
			{
				// apply hysteresis. 
				ndFloat32 rmp = motor->GetRpm();
				ndFloat32 highRpm = engineCurve.GetHighGearShiftRpm();
				if ((m_autoGearShiftTimer < -120) || (rmp < highRpm))
				{
					m_driverState = m_driveAutoGear;
				}
			}
			break;
		}

		// manual transmission state machine
		case m_driveForward:
		{
			if (m_ignition.Update(buttons[ndGameControllerInputs::m_ignitionButton] ? true : false))
			{
				gearJoint->SetRatio(0.0f);
				m_driverState = m_idle;
			}

			if (m_neutralGear.Update(buttons[ndGameControllerInputs::m_neutralGearButton] ? true : false))
			{
				gearJoint->SetRatio(0.0f);
				m_driverState = m_idle;
			}

			if (m_manualTransmission.Update(buttons[ndGameControllerInputs::m_automaticGearBoxButton] ? true : false))
			{
				m_transmission = m_automatic;
				m_driverState = m_driveAutoGear;
				m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear;
				ndFloat32 gearGain = gearBox.m_crownGearRatio * gearBox.m_gearRatios[m_currentGear];
				gearJoint->SetRatio(gearGain);
			}

			if (m_forwardGearUp.Update(buttons[ndGameControllerInputs::m_upGearButton] ? true : false))
			{
				if (m_currentGear < (gearBox.m_gearRatios.GetCount() - 1))
				{
					m_currentGear++;
					m_driverState = m_driveForwardGearDelay;
					m_autoGearShiftTimer = gearBox.m_gearShiftDelayTicks;
					ndFloat32 gearGain = gearBox.m_crownGearRatio * gearBox.m_gearRatios[m_currentGear];
					gearJoint->SetRatio(gearGain);
				}
			}
			if (m_forwardGearDown.Update(buttons[ndGameControllerInputs::m_downGearButton] ? true : false))
			{
				//m_driverState = m_driveShitGearDown;
				//m_autoGearShiftTimer = gearBox.m_gearShiftDelayTicks;

				if (m_currentGear > ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear)
				{
					m_currentGear--;
					m_driverState = m_driveForwardGearDelay;
					m_autoGearShiftTimer = gearBox.m_gearShiftDelayTicks;
					ndFloat32 gearGain = gearBox.m_crownGearRatio * gearBox.m_gearRatios[m_currentGear];
					gearJoint->SetRatio(gearGain);
				}
			}

			if (m_reverseGear.Update(buttons[ndGameControllerInputs::m_reverseGearButton] ? true : false))
			{
				if (ndAbs(vehicle->GetSpeed()) < ndFloat32(1.0f))
				{
					m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_revertGear;
					ndFloat32 reverseGearRatio = gearBox.m_gearRatios[m_currentGear];
					ndFloat32 gearGain = gearBox.m_crownGearRatio * reverseGearRatio;
					gearJoint->SetRatio(gearGain);
					m_driverState = m_driveReverse;
				}
			}
			break;
		}

		case m_driveReverse:
		{
			if (m_ignition.Update(buttons[ndGameControllerInputs::m_ignitionButton] ? true : false))
			{
				gearJoint->SetRatio(0.0f);
				m_driverState = m_idle;
			}
			if (m_neutralGear.Update(buttons[ndGameControllerInputs::m_neutralGearButton] ? true : false))
			{
				gearJoint->SetRatio(0.0f);
				m_driverState = m_idle;
			}
			if (m_forwardGearUp.Update(buttons[ndGameControllerInputs::m_upGearButton] ? true : false))
			{
				if (ndAbs(vehicle->GetSpeed()) < ndFloat32(1.0f))
				{
					ndAssert(0);
					//m_driverState = m_driveShitGearUp;
					//m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear;
					//ndFloat32 gearGain = gearBox.m_crownGearRatio * gearBox.m_gearRatios[m_currentGear];
					//m_autoGearShiftTimer = gearBox.m_gearShiftDelayTicks - 1;
					//gearJoint->SetRatio(gearGain);
				}
			}
			if (m_manualTransmission.Update(buttons[ndGameControllerInputs::m_automaticGearBoxButton] ? true : false))
			{
				if (ndAbs(vehicle->GetSpeed()) < ndFloat32(1.0f))
				{
					m_transmission = m_automatic;
					m_driverState = m_driveAutoGear;
					m_currentGear = ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear;
					ndFloat32 gearGain = gearBox.m_crownGearRatio * gearBox.m_gearRatios[m_currentGear];
					m_autoGearShiftTimer = gearBox.m_gearShiftDelayTicks - 1;
					gearJoint->SetRatio(gearGain);
				}
			}
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
