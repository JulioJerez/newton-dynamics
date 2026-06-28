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
#include "ndDemoEntityManager.h"
#include "ndGameControllerInputs.h"

const char* ndGameControllerInputs::m_buttonNames[] =
{
	{"ignitionButton"},
	{"upGearButton"},
	{"downGearButton"},
	{"handBreakButton"},
	{"neutralGearButton"},
	{"reverseGearButton"},
	{"parkGearButton"},
	{"automaticGearBoxButton"},
	{"changeCamera"},
	{"changePlayer"}
};

ndGameControllerInputs::ndGameControllerInputs()
	:m_keyBoardSteerAngle(0.0f)
{
	for (ndInt32 i = 0; i < m_buttons.GetCapacity(); ++i)
	{
		m_buttons.PushBack(0);
	}
	for (ndInt32 i = 0; i < m_axis.GetCapacity(); ++i)
	{
		m_axis.PushBack(ndFloat32 (0.0f));
	}
}

ndGameControllerInputs::~ndGameControllerInputs()
{
}

bool ndGameControllerInputs::GetKeyboardInputs(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndRender>& render = scene->GetRenderer();
	if (render->HasGameController() && !scene->AnyKeyDown())
	{
		return false;
	}
	m_buttons[ndGameControllerInputs::m_handBreakButton] = scene->GetKeyState(' ');
	m_buttons[ndGameControllerInputs::m_upGearButton] = scene->GetKeyState('>') || scene->GetKeyState('.');
	m_buttons[ndGameControllerInputs::m_downGearButton] = scene->GetKeyState('<') || scene->GetKeyState(',');
	m_buttons[ndGameControllerInputs::m_neutralGearButton] = scene->GetKeyState('N');
	m_buttons[ndGameControllerInputs::m_ignitionButton] = scene->GetKeyState('I');
	m_buttons[ndGameControllerInputs::m_reverseGearButton] = scene->GetKeyState('R');
	m_buttons[ndGameControllerInputs::m_automaticGearBoxButton] = scene->GetKeyState('?') || scene->GetKeyState('/');
	m_buttons[ndGameControllerInputs::m_parkGearButton] = scene->GetKeyState('P');
	m_buttons[ndGameControllerInputs::m_changeCamera] = scene->GetKeyState('C');
	m_buttons[ndGameControllerInputs::m_changePlayer] = scene->GetKeyState('K');
	
	ndFloat32 steerAngle = ndFloat32(scene->GetKeyState('A')) - ndFloat32(scene->GetKeyState('D'));
	m_keyBoardSteerAngle += (steerAngle - m_keyBoardSteerAngle) * 0.10f;
	m_keyBoardSteerAngle = (m_keyBoardSteerAngle < (1.0e-4f)) ? (m_keyBoardSteerAngle > (-1.0e-4f) ? 0.0f : m_keyBoardSteerAngle) : m_keyBoardSteerAngle;

	m_axis[ndGameControllerInputs::m_steeringWheel] = m_keyBoardSteerAngle;
	m_axis[ndGameControllerInputs::m_gasPedal] = ndFloat32(scene->GetKeyState('W')) ? 1.0f : 0.0f;
	m_axis[ndGameControllerInputs::m_brakePedal] = ndFloat32(scene->GetKeyState('S') ? 1.0f : 0.0f);
	m_axis[ndGameControllerInputs::m_clutch] = ndFloat32(0.0f);
	
	bool ret = false;
	for (ndInt32 i = 0; i < m_buttons.GetCount(); ++i)
	{
		ret = ret | m_buttons[i];
	}
	for (ndInt32 i = 0; i < m_axis.GetCount(); ++i)
	{
		ret = ret | ((m_axis[i] != 0.0f) ? 1 : 0);
	}

	return ret ? true : false;
}

void ndGameControllerInputs::Update(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndRender>& render = scene->GetRenderer();
	if (render->HasGameController())
	{
#if !defined (__APPLE__)
		char joystickName[256];
		
		strcpy(&joystickName[0], render->GameControllerName());
		strtolwr(joystickName);
		if (strstr(joystickName, "wheel"))
		{
			GetWheelJoystickInputs(scene);
		}
		else if (strstr(joystickName, "xbox"))
		{
			GetXboxJoystickInputs(scene);
		}
		else
		{
			GetJoystickInputs(scene);
		}
#endif
	}
	GetKeyboardInputs(scene);
}

//void ndGameControllerInputs::GetJoystickInputs(ndDemoEntityManager* const scene)
void ndGameControllerInputs::GetJoystickInputs(ndDemoEntityManager* const)
{
	ndAssert(0);
}

void ndGameControllerInputs::GetWheelJoystickInputs(ndDemoEntityManager* const scene)
{
	// logitech g920 mapping
	ndSharedPtr<ndRender>& render = scene->GetRenderer();
	{
		// remap buttons
		ndFixSizeArray<int, 32> buttonMapping;
		const ndFixSizeArray<ndInt8, 32>& unmappedButtons = render->GameControllerButtons();
		for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
		{
			buttonMapping.PushBack(buttonMapping.GetCapacity() - 1);
		}
		
		buttonMapping[2] = ndGameControllerInputs::ndGameControllerInputs::m_changeCamera;
		buttonMapping[3] = ndGameControllerInputs::ndGameControllerInputs::m_changePlayer;
		buttonMapping[7] = ndGameControllerInputs::ndGameControllerInputs::m_reverseGearButton;
		buttonMapping[8] = ndGameControllerInputs::ndGameControllerInputs::m_handBreakButton;
		buttonMapping[9] = ndGameControllerInputs::ndGameControllerInputs::m_neutralGearButton;
		buttonMapping[4] = ndGameControllerInputs::ndGameControllerInputs::m_upGearButton;
		buttonMapping[18] = ndGameControllerInputs::ndGameControllerInputs::m_upGearButton;
		buttonMapping[5] = ndGameControllerInputs::ndGameControllerInputs::m_downGearButton;
		buttonMapping[20] = ndGameControllerInputs::ndGameControllerInputs::m_downGearButton;
		buttonMapping[6] = ndGameControllerInputs::ndGameControllerInputs::m_ignitionButton;

		for (ndInt32 i = 0; i < m_buttons.GetCount(); ++i)
		{
			m_buttons[i] = false;
		}
		
		for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
		{
			ndInt32 index = buttonMapping[i];
			m_buttons[index] = m_buttons[index] || (unmappedButtons[i] ? true : false);
		}
	}

	{
		// remap game pad axis
		ndFixSizeArray<int, 8> axisMapping;
		const ndFixSizeArray<ndFloat32, 8>& unmappedAxis = render->GameControllerAxis();
		for (ndInt32 i = 0; i < unmappedAxis.GetCount(); i++)
		{
			axisMapping.PushBack(unmappedAxis.GetCapacity() - 1);
		}
		axisMapping[0] = m_steeringWheel;
		axisMapping[1] = m_gasPedal;
		axisMapping[2] = m_brakePedal;
		axisMapping[3] = m_clutch;

		for (ndInt32 i = 0; i < unmappedAxis.GetCount(); i++)
		{
			ndInt32 index = axisMapping[i];
			m_axis[index] = unmappedAxis[i];
		}

		m_axis[m_steeringWheel] = -m_axis[m_steeringWheel] * ndFloat32 (2.0f);
		m_axis[m_gasPedal] = (ndFloat32(1.0f) - m_axis[m_gasPedal]) * ndFloat32 (0.5f);
		m_axis[m_brakePedal] = ndFloat32 (1.0f) - ndClamp(m_axis[m_brakePedal], ndFloat32(0.0f), ndFloat32(1.0f));
		m_axis[m_clutch] = ndFloat32(1.0f) - ndClamp(m_axis[m_clutch], ndFloat32(0.0f), ndFloat32(1.0f));
	}
}

void ndGameControllerInputs::GetXboxJoystickInputs(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndRender>& render = scene->GetRenderer();
	{
		// remap buttons
		ndFixSizeArray<int, 32> buttonMapping;
		const ndFixSizeArray<ndInt8, 32>& unmappedButtons = render->GameControllerButtons();
		for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
		{
			buttonMapping.PushBack(buttonMapping.GetCapacity() - 1);
		}

		buttonMapping[0] = ndGameControllerInputs::ndGameControllerInputs::m_automaticGearBoxButton;
		buttonMapping[1] = ndGameControllerInputs::ndGameControllerInputs::m_parkGearButton;
		buttonMapping[2] = ndGameControllerInputs::ndGameControllerInputs::m_changeCamera;
		buttonMapping[3] = ndGameControllerInputs::ndGameControllerInputs::m_changePlayer;
		buttonMapping[4] = ndGameControllerInputs::ndGameControllerInputs::m_neutralGearButton;
		buttonMapping[5] = ndGameControllerInputs::ndGameControllerInputs::m_handBreakButton;
		buttonMapping[6] = ndGameControllerInputs::ndGameControllerInputs::m_reverseGearButton;
		buttonMapping[7] = ndGameControllerInputs::ndGameControllerInputs::m_ignitionButton;
		buttonMapping[10] = ndGameControllerInputs::ndGameControllerInputs::m_upGearButton;
		buttonMapping[12] = ndGameControllerInputs::ndGameControllerInputs::m_downGearButton;

		for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
		{
			ndInt32 index = buttonMapping[i];
			m_buttons[index] = unmappedButtons[i] ? true : false;
			//if (m_buttons[index])
			//{
			//	ndTrace(("%d\n", i));
			//}
		}
	}

	{
		// remap game pad axis
		ndFixSizeArray<int, 8> axisMapping;
		const ndFixSizeArray<ndFloat32, 8>& unmappedAxis = render->GameControllerAxis();
		for (ndInt32 i = 0; i < unmappedAxis.GetCount(); i++)
		{
			axisMapping.PushBack(unmappedAxis.GetCapacity() - 1);
		}
		axisMapping[0] = m_steeringWheel;
		axisMapping[4] = m_brakePedal;
		axisMapping[5] = m_gasPedal;

		ndFloat32 savedSteering = m_axis[m_steeringWheel];
		for (ndInt32 i = 0; i < unmappedAxis.GetCount(); i++)
		{
			ndInt32 index = axisMapping[i];
			m_axis[index] = unmappedAxis[i];
		}

		const ndFloat32 steering2 = m_axis[m_steeringWheel] * m_axis[m_steeringWheel];
		const ndFloat32 newSteering = -m_axis[m_steeringWheel] * steering2;
		if ((newSteering > ndAbs(savedSteering)) || (newSteering < -ndAbs(savedSteering)))
		{
			m_axis[m_steeringWheel] = savedSteering + (newSteering - savedSteering) * ndFloat32(0.02f);
		}
		else
		{
			m_axis[m_steeringWheel] = savedSteering + (newSteering - savedSteering) * ndFloat32(0.1f);
		}

		ndFloat32 gas = (m_axis[m_gasPedal] + ndFloat32(1.0f)) * ndFloat32(0.5f);
		m_axis[m_gasPedal] = gas * gas;

		ndFloat32 brake = (m_axis[m_brakePedal] + ndFloat32(1.0f)) * ndFloat32(0.5f);
		m_axis[m_brakePedal] = brake * brake;
		m_axis[m_clutch] = ndFloat32(0.0f);
	}
}