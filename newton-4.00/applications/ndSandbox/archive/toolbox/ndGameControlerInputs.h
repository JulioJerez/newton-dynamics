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
	ndFixSizeArray<char, 32> m_buttons;
	ndFixSizeArray<ndFloat32, 8> m_axis;

	ndGameControllerInputs();
	~ndGameControllerInputs();
	void Update(ndDemoEntityManager* const scene);

	private:
	bool GetKeyboardInputs(ndDemoEntityManager* const scene);
	void GetJoystickInputs(ndDemoEntityManager* const scene);
	void GetXboxJoystickInputs(ndDemoEntityManager* const scene);
	void GetWheelJoystickInputs(ndDemoEntityManager* const scene);

	ndFloat32 m_keyBoardSteerAngle;
};


#endif