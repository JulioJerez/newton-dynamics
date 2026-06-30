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

class ndSoundBuffer::Implementation : public ndClassAlloc
{
	public:
	Implementation(ndSoundManager* const owner, const char* const)
		:ndClassAlloc()
		,m_manager(owner)
		,m_sceneNode(nullptr)
		,m_isLooping(false)
	{
	}

	virtual ~Implementation()
	{
	}

	void SetLooping(bool state)
	{
		m_isLooping = state;
	}

	bool IsLooping() const
	{
		return m_isLooping;
	}

	void Play()
	{
	}

	void Stop()
	{
	}

	void SetPosition(const ndVector&)
	{
	}

	void SetVelocity(const ndVector&)
	{
	}

	ndWeakPtr<ndSoundManager> m_manager;
	ndList<ndSharedPtr<ndSoundBuffer>>::ndNode* m_sceneNode;
	bool m_isLooping;
};

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

	void SetLooping(bool)
	{
	}

	bool IsLooping() const
	{
		return false;
	}

	void Play()
	{
	}

	void Stop()
	{
	}

	ndSharedPtr<ndSoundBuffer> AddSound(ndSoundManager* const manager, const char* const waveFileName)
	{
		ndSharedPtr<ndSoundBuffer> sound(new ndSoundBuffer(manager, waveFileName));
		sound->m_implementation->m_sceneNode = m_soundScene.Append(sound);
		return sound;
	}

	void RemoveSound(ndSharedPtr<ndSoundBuffer>& sound)
	{
		sound->Stop();
		ndAssert(sound->m_implementation->m_sceneNode);
		m_soundScene.Remove(sound->m_implementation->m_sceneNode);
	}

	void ClearSounds()
	{
		while (m_soundScene.GetCount())
		{
			ndSoundManager* const manager = *m_soundScene.GetLast()->GetInfo()->m_implementation->m_manager;
			ndAssert(manager);
			manager->RemoveSound(m_soundScene.GetLast()->GetInfo());
		}
	}

	void Update(const ndMatrix&, const ndVector&)
	{
	}

	ndList<ndSharedPtr<ndSoundBuffer>> m_soundScene;


};

#else
#include "AL/al.h"
#include "AL/alext.h"

class ndSoundManager::Implementation: public ndClassAlloc
{
	public:
	Implementation();
	virtual ~Implementation();
	bool LoadWaveFile(ALuint buffer, const char* const waveFileName) const;
	void Update(const ndMatrix& listenerPosit, const ndVector& listenerVeloc);

	ndSharedPtr<ndSoundBuffer> AddSound(ndSoundManager* const manager, const char* const waveFileName);
	void RemoveSound(ndSharedPtr<ndSoundBuffer>& sound);
	void ClearSounds();

	ALCdevice* m_device;
	ALCcontext* m_context;
	ALboolean m_extension;
	ndTree<ALuint, ndString> m_buffersCache;
	ndList<ndSharedPtr<ndSoundBuffer>> m_soundScene;
	static ndMatrix m_coodinateSystem;
};

class ndSoundBuffer::Implementation : public ndClassAlloc
{
	public:
	Implementation(ndSoundManager* const owner, const char* const waveFileName);
	virtual ~Implementation();

	void Play();
	void Stop();
	void Update();

	bool IsLooping() const;
	void SetLooping(bool state);
	void SetPosition(const ndVector& posit);
	void SetVelocity(const ndVector& veloc);

	ndWeakPtr<ndSoundManager> m_manager;
	ndList<ndSharedPtr<ndSoundBuffer>>::ndNode* m_sceneNode;
	ALuint m_source;
	bool m_isLooping;
};

ndMatrix ndSoundManager::Implementation::m_coodinateSystem(ndYawMatrix(ndFloat32(90.0f)* ndDegreeToRad));

ndSoundBuffer::Implementation::Implementation(ndSoundManager* const owner, const char* const waveFileName)
//Implementation(ndSoundManager* const owner, ALuint buffer)
	:ndClassAlloc()
	, m_manager(owner)
	, m_sceneNode(nullptr)
	, m_isLooping(false)
{
	ndTree<ALuint, ndString>::ndNode* const resource = m_manager->m_implementation->m_buffersCache.Find(waveFileName);
	ndAssert(resource);
	ALuint buffer = resource->GetInfo();
	//// generate buffer
	//alGenBuffers(1, &m_buffer);
	//ndAssert(alGetError() == AL_NO_ERROR);
	//LoadWaveFile(waveFileName);

	// Generate Source
	alGenSources(1, &m_source);
	ndAssert(alGetError() == AL_NO_ERROR);

	// bind buffer to source
	alSourcei(m_source, AL_BUFFER, ALint(buffer));
	ndAssert(alGetError() == AL_NO_ERROR);

	// Explicitly ensure the sound is relative to the world, not the listener
	alSourcei(m_source, AL_SOURCE_RELATIVE, AL_FALSE);
}

ndSoundBuffer::Implementation::~Implementation()
{
	// delete source
	alDeleteSources(1, &m_source);
	ndAssert(alGetError() == AL_NO_ERROR);
}

void ndSoundBuffer::Implementation::SetLooping(bool state)
{
	m_isLooping = state;
	alSourcei(m_source, AL_LOOPING, m_isLooping ? AL_TRUE : AL_FALSE);
}

bool ndSoundBuffer::Implementation::IsLooping() const
{
	return m_isLooping;
}

void ndSoundBuffer::Implementation::Play()
{
	if (m_source)
	{
		alSourcePlay(m_source);
	}
}

void ndSoundBuffer::Implementation::Stop()
{
	if (m_source)
	{
		alSourceStop(m_source);
	}
}

void ndSoundBuffer::Implementation::Update()
{

}

void ndSoundBuffer::Implementation::SetPosition(const ndVector& posit)
{
	if (m_source)
	{
		const ndVector alPosit(ndSoundManager::Implementation::m_coodinateSystem.RotateVector(posit));
		alSource3f(m_source, AL_POSITION, ALfloat(alPosit.m_x), ALfloat(alPosit.m_y), ALfloat(alPosit.m_z));
	}
}

void ndSoundBuffer::Implementation::SetVelocity(const ndVector& veloc)
{
	if (m_source)
	{
		const ndVector alVeloc(ndSoundManager::Implementation::m_coodinateSystem.RotateVector(veloc));
		alSource3f(m_source, AL_POSITION, ALfloat(alVeloc.m_x), ALfloat(alVeloc.m_y), ALfloat(alVeloc.m_z));
	}
}

ndSoundManager::Implementation::Implementation()
	:ndClassAlloc()
	,m_device(nullptr)
	,m_context(nullptr)
	,m_extension(false)
{
	m_device = alcOpenDevice(nullptr);

	if (m_device)
	{
		m_context = alcCreateContext(m_device, NULL);
		alcMakeContextCurrent(m_context);
	}
	// Check for EAX 2.0 support
	m_extension = alIsExtensionPresent("EAX2.0");
	ndAssert(alGetError() == AL_NO_ERROR);
}

ndSoundManager::Implementation::~Implementation()
{
	if (m_device)
	{
		ndAssert(m_context == alcGetCurrentContext());
		ndAssert(m_device = alcGetContextsDevice(m_context));
		alcMakeContextCurrent(nullptr);
		alcDestroyContext(m_context);
		alcCloseDevice(m_device);
	}
}

bool ndSoundManager::Implementation::LoadWaveFile(ALuint buffer, const char* const waveFileName) const
{
	struct WavHeader
	{
		char riffId[4];         // "RIFF"
		uint32_t fileSize;      // Total file size minus 8 bytes
		char waveId[4];         // "WAVE"
		char fmtId[4];          // "fmt "
		uint32_t fmtSize;       // Size of format chunk (typically 16)
		uint16_t audioFormat;   // 1 for uncompressed PCM
		uint16_t channels;      // 1 = Mono, 2 = Stereo
		uint32_t sampleRate;    // e.g., 44100
		uint32_t byteRate;      // sampleRate * channels * (bitsPerSample/8)
		uint16_t blockAlign;    // channels * (bitsPerSample/8)
		uint16_t bitsPerSample; // 8 or 16 bits
		char dataId[4];         // "data"
		uint32_t dataSize;      // Total bytes of raw audio data
	};

	const ndString fileName(ndGetWorkingFileName(waveFileName));
	FILE* const file = fopen(fileName.GetStr(), "rb");
	if (!file)
	{
		ndTrace(("sound file %s : not found\n", fileName.GetStr()));
		return false;
	}

	WavHeader header;
	fread(&header, sizeof(WavHeader), 1, file);

	// Verify this is a proper standard WAV file container
	if (ndString(header.riffId, 4) != "RIFF" || ndString(header.waveId, 4) != "WAVE")
	{
		ndTrace(("sound file %s : Invalid WAVE file format\n", fileName.GetStr()));
		return false;
	}

	// Allocate memory buffer to hold the raw audio data on CPU
	ndArray<char> pcmData(header.dataSize);
	pcmData.SetCount(header.dataSize);
	fread(&pcmData[0], 1, header.dataSize, file);

	// Identify corresponding OpenAL format identifier
	auto GetOpenAlFormat = [](uint16_t channels, uint16_t bitsPerSample)
	{
		if (channels == 1)
		{
			return (bitsPerSample == 8) ? AL_FORMAT_MONO8 : AL_FORMAT_MONO16;
		}
		else if (channels == 2)
		{
			return (bitsPerSample == 8) ? AL_FORMAT_STEREO8 : AL_FORMAT_STEREO16;
		}
		return 0; // Unsupported format configuration
	};
	ALenum format = GetOpenAlFormat(header.channels, header.bitsPerSample);
	if (format == 0)
	{
		ndTrace(("sound file %s : Unsupported channel/bit depth combination.\n", fileName.GetStr()));
		return false;
	}

	// Clear previous errors and upload native data to the OpenAL driver
	alGetError();
	alBufferData(buffer, format, &pcmData[0], ALsizei(header.dataSize), ALsizei(header.sampleRate));
	ndAssert(alGetError() == AL_NO_ERROR);
	return true;
}

void ndSoundManager::Implementation::Update(const ndMatrix& listenerPosit, const ndVector& listenerVeloc)
{
	ALfloat posit[3];
	ALfloat veloc[3];
	ALfloat orientation[6];
	const ndMatrix alMatrix(m_coodinateSystem * listenerPosit);
	const ndVector alVeloc(m_coodinateSystem.RotateVector(listenerVeloc));

	veloc[0] = ALfloat(alVeloc.m_x);
	veloc[1] = ALfloat(alVeloc.m_y);
	veloc[2] = ALfloat(alVeloc.m_z);
	posit[0] = ALfloat(alMatrix.m_posit.m_x);
	posit[1] = ALfloat(alMatrix.m_posit.m_y);
	posit[2] = ALfloat(alMatrix.m_posit.m_z);

	orientation[0] = ALfloat(alMatrix.m_front.m_x);
	orientation[1] = ALfloat(alMatrix.m_front.m_y);
	orientation[2] = ALfloat(alMatrix.m_front.m_z);
	orientation[3] = ALfloat(alMatrix.m_up.m_x);
	orientation[4] = ALfloat(alMatrix.m_up.m_y);
	orientation[5] = ALfloat(alMatrix.m_up.m_z);

	// Position ...
	alListenerfv(AL_POSITION, posit);
	ndAssert(alGetError() == AL_NO_ERROR);

	// Velocity ...
	alListenerfv(AL_VELOCITY, veloc);
	ndAssert(alGetError() == AL_NO_ERROR);

	// Orientation ...
	alListenerfv(AL_ORIENTATION, orientation);
	ndAssert(alGetError() == AL_NO_ERROR);

	// update all positionalsounds in teh scene
	for (ndList<ndSharedPtr<ndSoundBuffer>>::ndNode* node = m_soundScene.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndSoundBuffer>& sound = node->GetInfo();
		sound->m_implementation->Update();
	}
}

ndSharedPtr<ndSoundBuffer> ndSoundManager::Implementation::AddSound(ndSoundManager* const manager, const char* const waveFileName)
{
	const ndString name(waveFileName);
	ndTree<ALuint, ndString>::ndNode* resource = m_buffersCache.Find(name);
	if (!resource)
	{
		// generate buffer
		ALuint buffer;
		alGenBuffers(1, &buffer);
		ndAssert(alGetError() == AL_NO_ERROR);
		LoadWaveFile(buffer, waveFileName);
		resource = m_buffersCache.Insert(buffer, name);
	}

	ndSharedPtr<ndSoundBuffer> sound(new ndSoundBuffer(manager, waveFileName));
	sound->m_implementation->m_sceneNode = m_soundScene.Append(sound);
	return sound;
}

void ndSoundManager::Implementation::RemoveSound(ndSharedPtr<ndSoundBuffer>& sound)
{
	sound->Stop();
	ndAssert(sound->m_implementation->m_sceneNode);
	m_soundScene.Remove(sound->m_implementation->m_sceneNode);
}

void ndSoundManager::Implementation::ClearSounds()
{
	while (m_soundScene.GetCount())
	{
		ndSoundManager* const manager = *m_soundScene.GetLast()->GetInfo()->m_implementation->m_manager;
		ndAssert(manager);
		manager->RemoveSound(m_soundScene.GetLast()->GetInfo());
	}

	while (m_buffersCache.GetCount())
	{
		// delete buffer
		ndTree<ALuint, ndString>::ndNode* const node = m_buffersCache.GetRoot();;
		ALuint buffer = node->GetInfo();
		alDeleteBuffers(1, &buffer);
		ndAssert(alGetError() == AL_NO_ERROR);
		m_buffersCache.Remove(node);
	}
}

#endif

ndSoundBuffer::ndSoundBuffer(ndSoundManager* const manager, const char* const waveFileName)
	:ndClassAlloc()
	,m_implementation(new Implementation(manager, waveFileName))
{
}

ndSoundBuffer::~ndSoundBuffer()
{
	delete m_implementation;
};

void ndSoundBuffer::Play()
{
	m_implementation->Play();
}

void ndSoundBuffer::Stop()
{
	m_implementation->Stop();
}

void ndSoundBuffer::SetLooping(bool state)
{
	m_implementation->SetLooping(state);
}

bool ndSoundBuffer::IsLooping() const
{
	return m_implementation->IsLooping();
}

void ndSoundBuffer::SetPosition(const ndVector& posit)
{
	m_implementation->SetPosition(posit);
}

void ndSoundBuffer::SetVelocity(const ndVector& veloc)
{
	m_implementation->SetVelocity(veloc);
}

ndSoundManager::ndSoundManager()
	:ndClassAlloc()
	,m_implementation(new Implementation())
{
	//ndSharedPtr<ndSoundBuffer> test0(AddSound("diesel_engine.wav"));
	//ndSharedPtr<ndSoundBuffer> test(AddSound("diesel_engine.wav"));
	//test->SetLooping(true);
	//test->Play();
	//Sleep(10000);
	//test->Stop();
	//RemoveSound(test0);
}

ndSoundManager::~ndSoundManager()
{
	ClearSounds();
	delete m_implementation;
}

void ndSoundManager::Update(const ndMatrix& listenerPosit, const ndVector& veloc)
{
	m_implementation->Update(listenerPosit, veloc);
}

ndSharedPtr<ndSoundBuffer> ndSoundManager::AddSound(const char* const waveFileName)
{
	ndSharedPtr<ndSoundBuffer> sound (m_implementation->AddSound(this, waveFileName));
	return sound;
}

void ndSoundManager::RemoveSound(ndSharedPtr<ndSoundBuffer>& sound)
{
	m_implementation->RemoveSound(sound);
}

void ndSoundManager::ClearSounds()
{
	m_implementation->ClearSounds();
}

