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

		if (m_device)
		{
			m_context = alcCreateContext(m_device, NULL);
			alcMakeContextCurrent(m_context);
		}
		// Check for EAX 2.0 support
		m_extension = alIsExtensionPresent("EAX2.0");
		// Generate Buffers
		alGetError(); // clear error code
	}

	virtual ~Implementation()
	{
		if (m_device)
		{
			ndAssert(m_context == alcGetCurrentContext());
			ndAssert(m_device = alcGetContextsDevice(m_context));
			//Device = alcGetContextsDevice(Context);
			//ndAssert(m_context == alcGetCurrentContext())
			alcMakeContextCurrent(nullptr);
			alcDestroyContext(m_context);
			alcCloseDevice(m_device);
		}
	}

	ALCdevice* m_device;
	ALCcontext* m_context;
	ALboolean m_extension;
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