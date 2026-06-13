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
	//m_buttons.SetCount(m_buttonCount);
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
	
	//m_axis.SetCount(m_axisCount);
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
	//static ndFixSizeArray<int, 8> axisMapping;
	//static ndFixSizeArray<int, 32> buttonMapping;
	//
	//ndFixSizeArray<char, 32> unmappedButtons;
	//ndFixSizeArray<ndFloat32, 8> unmappedAxis;
	//
	//scene->GetJoystickAxis(unmappedAxis);
	//scene->GetJoystickButtons(unmappedButtons);
	//
	//if (!buttonMapping.GetCount())
	//{
	//	for (ndInt32 i = 0; i < buttonMapping.GetCapacity(); ++i)
	//	{
	//		buttonMapping.PushBack(m_buttonCount);
	//	}
	//
	//	buttonMapping[0] = m_button_00;		//m_handBreakButton
	//	buttonMapping[3] = m_button_01;		//m_upGearButton
	//	buttonMapping[2] = m_button_02;		//m_downGearButton
	//	buttonMapping[5] = m_button_03;		//m_neutralGearButton
	//	buttonMapping[10] = m_button_04;	//m_ignitionButton
	//	buttonMapping[4] = m_button_05;		//m_reverseGearButton
	//	buttonMapping[11] = m_button_06;	//m_automaticGearBoxButton
	//	buttonMapping[1] = m_button_07;		//m_parkingButton
	//	buttonMapping[8] = m_button_08;		//m_playerButton
	//}
	//
	//m_buttons.SetCount(m_buttonCount);
	//for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
	//{
	//	ndInt32 j = buttonMapping[i];
	//	m_buttons[j] = unmappedButtons[i];
	//}
	//
	//if (!axisMapping.GetCount())
	//{
	//	for (ndInt32 i = 0; i < axisMapping.GetCapacity(); ++i)
	//	{
	//		axisMapping.PushBack(m_axisCount);
	//	}
	//	axisMapping[0] = m_azis_00;
	//	axisMapping[1] = m_azis_01;
	//}
	//
	//m_axis.SetCount(m_axisCount);
	//for (ndInt32 i = 0; i < axisMapping.GetCount(); i++)
	//{
	//	m_axis[axisMapping[i]] = unmappedAxis[i];
	//}
	//
	//m_axis[m_azis_00] = -m_axis[m_azis_00] * m_axis[m_azis_00] * m_axis[m_azis_00];
	//
	//ndFloat32 gas = ndMin (m_axis[m_azis_01], ndFloat32 (0.0f));
	//ndFloat32 brake = ndMax(m_axis[m_azis_01], ndFloat32(0.0f));
	//
	//m_axis[m_azis_01] = gas * gas;
	//m_axis[m_azis_02] = brake * brake;
}

//void ndGameControllerInputs::GetWheelJoystickInputs(ndDemoEntityManager* const scene)
void ndGameControllerInputs::GetWheelJoystickInputs(ndDemoEntityManager* const)
{
	ndAssert(0);
	//// logitech g920 mapping
	//ndFixSizeArray<char, 32> unmappedButtons;
	//ndFixSizeArray<ndFloat32, 8> unmappedAxis;
	//static ndFixSizeArray<int, 8> axisMapping;
	//static ndFixSizeArray<int, 32> buttonMapping;
	//
	//scene->GetJoystickAxis(unmappedAxis);
	//scene->GetJoystickButtons(unmappedButtons);
	//
	//if (!buttonMapping.GetCount())
	//{
	//	for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
	//	{
	//		buttonMapping.PushBack(m_buttonCount);
	//	}
	//
	//	buttonMapping[2] = m_button_00;		//m_ignitionButton
	//	buttonMapping[4] = m_button_01;		//m_upGearButton
	//	buttonMapping[5] = m_button_02;		//m_downGearButton
	//	buttonMapping[0] = m_button_03;		//m_handBreakButton
	//	buttonMapping[1] = m_button_03;		//m_handBreakButton
	//	buttonMapping[6] = m_button_04;		//m_neutralGearButton
	//	buttonMapping[7] = m_button_08;		//m_playerButton
	//	buttonMapping[8] = m_button_05;		//m_reverseGearButton
	//	buttonMapping[18] = m_button_01;	//m_upGearButton paddle on streeng wheel
	//	buttonMapping[20] = m_button_02;	//m_downGearButton paddle on streeng wheel
	//}
	//
	//m_buttons.SetCount(m_buttonCount+1);
	//ndMemSet(&m_buttons[0], char(0), m_buttons.GetCount());
	//for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
	//{
	//	if (unmappedButtons[i])
	//	{
	//		ndInt32 buttonIndex = buttonMapping[i];
	//		m_buttons[buttonIndex] = unmappedButtons[i];
	//		//ndTrace(("(%d %d)\n", buttonIndex, unmappedButtons[i]));
	//	}
	//}
	//
	//if (!axisMapping.GetCount())
	//{
	//	for (ndInt32 i = 0; i < unmappedAxis.GetCount(); ++i)
	//	{
	//		axisMapping.PushBack(m_axisCount);
	//	}
	//	axisMapping[0] = m_azis_00;
	//	axisMapping[1] = m_azis_01;
	//	axisMapping[2] = m_azis_02;
	//	axisMapping[3] = m_azis_03;
	//}
	//
	//m_axis.SetCount(m_axisCount + 1);
	//ndMemSet(&m_axis[0], ndFloat32(0.0f), m_axis.GetCount());
	//for (ndInt32 i = 0; i < unmappedAxis.GetCount(); i++)
	//{
	//	ndInt32 axisIndex = axisMapping[i];
	//	m_axis[axisIndex] = unmappedAxis[i];
	//
	//	//if (((i == 0) && (ndAbs(unmappedAxis[0]) > 0.1f)) || ((i > 0) && ndAbs(unmappedAxis[i]) < 0.9f))
	//	//ndTrace(("(%d %f)\n", axisIndex, unmappedAxis[i]));
	//}
	//m_axis[m_azis_00] = -m_axis[m_azis_00] * 2.0f; 
	//m_axis[m_azis_01] = (1.0f - m_axis[m_azis_01]) * 0.5f;
	//m_axis[m_azis_02] = ndFloat32 (1.0f) - ndClamp(m_axis[m_azis_02], ndFloat32(0.0f), ndFloat32(1.0f));
	//m_axis[m_azis_03] = ndFloat32(0.0f);
}

void ndGameControllerInputs::GetXboxJoystickInputs(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndRender>& render = scene->GetRenderer();
	// remap buttons
	{
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
		buttonMapping[4] = ndGameControllerInputs::ndGameControllerInputs::m_handBreakButton;
		buttonMapping[5] = ndGameControllerInputs::ndGameControllerInputs::m_reverseGearButton;
		buttonMapping[6] = ndGameControllerInputs::ndGameControllerInputs::m_neutralGearButton;
		buttonMapping[7] = ndGameControllerInputs::ndGameControllerInputs::m_ignitionButton;
		buttonMapping[10] = ndGameControllerInputs::ndGameControllerInputs::m_upGearButton;
		buttonMapping[12] = ndGameControllerInputs::ndGameControllerInputs::m_downGearButton;

		for (ndInt32 i = 0; i < unmappedButtons.GetCount(); ++i)
		{
			ndInt32 index = buttonMapping[i];
			//if (m_buttons[index] != unmappedButtons[i])
			//{
			//	ndTrace(("%d %d\n", index, i));
			//	index *= 1;
			//}
			m_buttons[index] = unmappedButtons[i] ? true : false;
		}
	}

	// remap game pad axis
	{
		ndFixSizeArray<int, 8> axisMapping;
		const ndFixSizeArray<ndFloat32, 8>& unmappedAxis = render->GameControllerAxis();
		for (ndInt32 i = 0; i < unmappedAxis.GetCount(); i++)
		{
			axisMapping.PushBack(unmappedAxis.GetCapacity() - 1);
		}
		axisMapping[0] = m_steeringWheel;
		axisMapping[4] = m_brakePedal;
		axisMapping[5] = m_gasPedal;

		for (ndInt32 i = 0; i < unmappedAxis.GetCount(); i++)
		{
			ndInt32 index = axisMapping[i];
			m_axis[index] = unmappedAxis[i];
		}

		m_axis[m_steeringWheel] = -m_axis[m_steeringWheel] * m_axis[m_steeringWheel] * m_axis[m_steeringWheel];
		ndFloat32 gas = (m_axis[m_gasPedal] + ndFloat32(1.0f)) * ndFloat32(0.5f);
		m_axis[m_gasPedal] = gas * gas;

		ndFloat32 brake = (m_axis[m_brakePedal] + ndFloat32(1.0f)) * ndFloat32(0.5f);
		m_axis[m_brakePedal] = brake * brake;
		m_axis[m_clutch] = ndFloat32(0.0f);
	}
}