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

#ifndef __ND_SOUND_MANAGER_UTIL__
#define __ND_SOUND_MANAGER_UTIL__

#include "ndSandboxStdafx.h"

class ndSoundManager;

class ndSoundBuffer : public ndClassAlloc
{
	class Implementation;
	public:
	virtual ~ndSoundBuffer();

	void Play();
	void Stop();
	bool IsLooping() const;
	void SetLooping(bool state);

	void SetPosition(const ndVector& posit);
	void SetVelocity(const ndVector& veloc);

	private:
	ndSoundBuffer(ndSoundManager* const manager, const char* const waveFileName);
	Implementation* m_implementation;
	friend class ndSoundManager;
};

class ndSoundManager : public ndClassAlloc
{
	class Implementation;
	public:

	ndSoundManager();
	virtual ~ndSoundManager();

	void ClearSounds();
	void RemoveSound(ndSharedPtr<ndSoundBuffer>& sound);
	ndSharedPtr<ndSoundBuffer> AddSound(const char* const waveFileName);

	void Update(const ndMatrix& listenerPosit, const ndVector& veloc);

	private:
	Implementation* m_implementation;
	friend class ndSoundBuffer;
};

#endif