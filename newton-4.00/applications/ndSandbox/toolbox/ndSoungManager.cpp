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
#include "ndSoundManager.h"

#if !defined (ND_OPEN_AL)

class ndSoundManager::Implementation : public ndClassAlloc
{
	public:
	Implementation()
		:ndClassAlloc()
	{
	}

	virtual ~Implementation()
	{
	}
};

#else
#include "AL/al.h"
#include "AL/alext.h"

class ndSoundManager::Implementation : public ndClassAlloc
{
	public:
	Implementation()
		:ndClassAlloc()
	{
		m_device = alcOpenDevice(nullptr);
	}

	virtual ~Implementation()
	{
		if (m_device)
		{
			alcCloseDevice(m_device);
		}
	}

	ALCdevice* m_device;
};
#endif


ndSoundManager::ndSoundManager()
	:ndClassAlloc()
	,m_implementation(new Implementation())
{
}

ndSoundManager::~ndSoundManager()
{
	delete m_implementation;
}