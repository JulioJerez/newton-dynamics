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

class ndSoundSource;
class ndSoundManager;
class ndDemoEntityManager;

class ndSoundSourceNotify : public ndClassAlloc
{
	public:
	ndSoundSourceNotify()
		:ndClassAlloc()
	{
	}

	virtual ~ndSoundSourceNotify()
	{
	}

	virtual void Update(ndSoundSource* const source) = 0;
};

class ndSoundSource : public ndClassAlloc
{
	public:
	class Implementation;
	virtual ~ndSoundSource();

	void Play();
	void Stop();
	bool IsLooping() const;
	void SetLooping(bool state);

	ndVector GetPosition() const;
	ndVector GetVelocity() const;
	void SetPosition(const ndVector& posit);
	void SetVelocity(const ndVector& veloc);

	ndSharedPtr<ndSoundSourceNotify> GetNotify() const;
	void SetNotify(ndSharedPtr<ndSoundSourceNotify> notify);

	protected:
	ndSoundSource(ndSharedPtr<ndSoundManager>& owner, const char* const waveFileName);
	Implementation* m_implementation;
	friend class ndSoundManager;
};

class ndSoundManager : public ndClassAlloc
{
	public:
	class Implementation;
	ndSoundManager(ndDemoEntityManager* const owner);
	virtual ~ndSoundManager();

	void ClearSounds();
	void RemoveSound(ndSharedPtr<ndSoundSource>& sound);
	ndSharedPtr<ndSoundSource> AddSound(const char* const waveFileName);

	void Update(const ndMatrix& listenerMatrix, const ndVector& veloc);

	protected:
	Implementation* m_implementation;
	friend class ndSoundSource;
	friend class ndOpenAlSource;
};

#endif