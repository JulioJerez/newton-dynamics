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

#ifndef __ND_GAME_CONTROLLER_INPUTS__
#define __ND_GAME_CONTROLLER_INPUTS__

#include "ndSandboxStdafx.h"
class ndDemoEntityManager;

class ndGameControllerInputs
{
	public:
	enum ndInputAxis
	{
		m_steeringWheel,
		m_gasPedal,
		m_brakePedal,
		m_clutch,
		m_usedAxisCount,
	};

	enum ndInputButtons
	{
		m_ignitionButton,
		m_upGearButton,
		m_downGearButton,
		m_handBreakButton,
		m_neutralGearButton,
		m_reverseGearButton,
		m_parkGearButton,
		m_automaticGearBoxButton,
		m_changeCamera,
		m_changePlayer,
	};

	static const char* m_buttonNames[];

	ndGameControllerInputs();
	~ndGameControllerInputs();
	void Update(ndDemoEntityManager* const scene);

	const ndFixSizeArray<bool, 32>& GetButtons() const
	{
		return m_buttons;
	}

	const ndFixSizeArray<ndFloat32, 8>& GetAxis() const
	{
		return m_axis;
	}

	private:
	bool GetKeyboardInputs(ndDemoEntityManager* const scene);
	void GetJoystickInputs(ndDemoEntityManager* const scene);
	void GetXboxJoystickInputs(ndDemoEntityManager* const scene);
	void GetWheelJoystickInputs(ndDemoEntityManager* const scene);

	ndFixSizeArray<bool, 32> m_buttons;
	ndFixSizeArray<ndFloat32, 8> m_axis;
	ndFloat32 m_keyBoardSteerAngle;

	//friend class ndVehicleCommonNotify;
};


#endif