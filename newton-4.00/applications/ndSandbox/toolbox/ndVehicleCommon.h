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

#ifndef __ND_VEHICLE_COMMON_UTIL__
#define __ND_VEHICLE_COMMON_UTIL__

#include "ndSandboxStdafx.h"
#include "ndDemoEntityNotify.h"

class ndDemoEntityManager;

class ndVehicleCommonNotify : public ndModelNotify
{
	public:
	enum ndDriveState
	{
		m_parked,
		m_idle,
		m_driveReverse,
		m_driveForward,
		m_driveAutoGear,
		m_driveAutoReverse,
		m_driveAutoShitGearUp,
		m_driveAutoShitGearDown,
		m_driveShitGearUp,
		m_driveShitGearDown,
		m_driveForwardGearDelay,
		m_driveReverseFromForward,
	};

	enum ndTransmissionMode
	{
		m_manual,
		m_automatic
	};

	ndVehicleCommonNotify(ndMultiBodyVehicle* const vehicle);

	virtual void ApplyInputs(ndFloat32 timestep);
	
	void Update(ndFloat32 timestep) override;
	void PostUpdate(ndFloat32 timestep) override;
	void PostTransformUpdate(ndFloat32 timestep) override;
	void Debug(ndConstraintDebugCallback&) const override;

	bool GetPlayerState() const;
	void SetAsPlayer(bool state);

	ndInt32 m_currentGear;
	ndInt32 m_autoGearShiftTimer;
	ndDemoEntityManager::ndKeyTrigger m_parking;
	ndDemoEntityManager::ndKeyTrigger m_ignition;
	ndDemoEntityManager::ndKeyTrigger m_neutralGear;
	ndDemoEntityManager::ndKeyTrigger m_reverseGear;
	ndDemoEntityManager::ndKeyTrigger m_forwardGearUp;
	ndDemoEntityManager::ndKeyTrigger m_forwardGearDown;
	ndDemoEntityManager::ndKeyTrigger m_manualTransmission;
	ndDriveState m_driverState;
	ndTransmissionMode m_transmission;
	bool m_isPlayer;
};

#endif