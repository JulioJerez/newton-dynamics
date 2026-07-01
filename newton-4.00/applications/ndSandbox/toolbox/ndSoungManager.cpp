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
#include "ndDemoEntityManager.h"

class ndSoundManager::Implementation : public ndClassAlloc
{
	public:
	Implementation(ndDemoEntityManager* const owner)
		:ndClassAlloc()
		,m_owner(owner)
	{
	}

	virtual ~Implementation() {}

	virtual void Update(const ndMatrix&, const ndVector&);
	virtual ndSharedPtr<ndSoundSource> AddSound(const char* const waveFileName);
	virtual void RemoveSound(ndSharedPtr<ndSoundSource>&) { ndAssert(0); }
	virtual void ClearSounds()
	{
		ndAssert(m_soundScene.GetCount() == 0);
		while (m_soundScene.GetCount())
		{
			ndAssert(0);
			//	ndList<ndWeakPtr<ndSoundSource>>::ndNode* const node = m_soundScene.GetLast();
			//	ndSoundSource* const sound = *node->GetInfo();
			//	sound
			//
			//	//manager->RemoveSound(m_soundScene.GetLast()->GetInfo());
		}
	}

	ndWeakPtr<ndDemoEntityManager> m_owner;
	ndList<ndWeakPtr<ndSoundSource>> m_soundScene;
};

class ndSoundSource::Implementation : public ndClassAlloc
{
	public:
	Implementation(ndSharedPtr<ndSoundManager>& owner)
		:ndClassAlloc()
		,m_posit(ndVector::m_wOne)
		,m_veloc(ndVector::m_zero)
		,m_manager(owner)
		,m_notify(nullptr)
		,m_sceneNode(nullptr)
		,m_isLooping(false)
	{
	}

	virtual ~Implementation() 
	{ 
		if (m_sceneNode)
		{
			for (ndList<ndWeakPtr<ndSoundSource>>::ndNode* node = m_manager->m_implementation->m_soundScene.GetFirst(); node; node = node->GetNext())
			{
				if (node == m_sceneNode)
				{
					m_manager->m_implementation->m_soundScene.Remove(node);
					m_sceneNode = nullptr;
					break;
				}
			}
		}
	}

	virtual void Play() 
	{ 
		m_isPlayig = true;
	}

	virtual void Stop() 
	{
		m_isPlayig = false;
	}

	virtual bool IsPlaying() const
	{
		return m_isPlayig;
	}

	virtual bool IsLooping() const 
	{ 
		return m_isLooping;
	}

	virtual void SetLooping(bool state) 
	{ 
		m_isLooping = state; 
	}

	virtual void Update() 
	{ 
		if (m_notify)
		{
			ndAssert(m_sceneNode);
			m_notify->Update(*m_sceneNode->GetInfo());
		}
	}

	ndVector GetPosition() const 
	{
		return m_posit;
	}

	ndVector GetVelocity() const 
	{ 
		return m_veloc;
	}

	virtual void SetPosition(const ndVector& posit) 
	{ 
		m_posit = posit;
	}

	virtual void SetVelocity(const ndVector& veloc) 
	{ 
		m_veloc = veloc;
	}

	ndFloat32 GetVolume() const
	{
		return m_volume;
	}

	virtual void SetVolume(ndFloat32 volume)
	{
		m_volume = ndClamp(volume, ndFloat32(0.0f), ndFloat32(1.0f));
	}

	virtual ndFloat32 GetPitch() const
	{
		return m_pitch;
	}

	virtual void SetPitch(ndFloat32 ptich)
	{
		m_pitch = ndAbs(ptich);
	}

	ndSharedPtr<ndSoundSourceNotify> GetNotify() const
	{
		return m_notify;
	}

	void SetNotify(ndSharedPtr<ndSoundSourceNotify> notify)
	{
		m_notify = notify;
	}

	ndVector m_posit;
	ndVector m_veloc;
	ndSharedPtr<ndSoundManager> m_manager;
	ndSharedPtr<ndSoundSourceNotify> m_notify;
	ndList<ndWeakPtr<ndSoundSource>>::ndNode* m_sceneNode;

	ndFloat32 m_pitch;
	ndFloat32 m_volume;
	bool m_isPlayig;
	bool m_isLooping;
};

ndSharedPtr<ndSoundSource> ndSoundManager::Implementation::AddSound(const char* const waveFileName)
{
	ndSharedPtr<ndSoundManager> manager(m_owner->GetSoundManager());
	ndAssert(*manager);
	ndSharedPtr<ndSoundSource> sound(new ndSoundSource(manager, waveFileName));
	sound->m_implementation->m_sceneNode = m_soundScene.Append(*sound);
	return sound;
}

void ndSoundManager::Implementation::Update(const ndMatrix&, const ndVector&) 
{ 
	// update all positional sounds in the scene
	for (ndList<ndWeakPtr<ndSoundSource>>::ndNode* node = m_soundScene.GetFirst(); node; node = node->GetNext())
	{
		ndWeakPtr<ndSoundSource>& sound = node->GetInfo();
		sound->m_implementation->Update();
	}
}

#if defined (ND_OPEN_AL)
#include "AL/al.h"
#include "AL/alext.h"

class ndOpenAlManager: public ndSoundManager::Implementation
{
	public:
	ndOpenAlManager(ndDemoEntityManager* const owner);
	virtual ~ndOpenAlManager();

	virtual void Update(const ndMatrix& listenerPosit, const ndVector& listenerVeloc) override;
	virtual ndSharedPtr<ndSoundSource> AddSound(const char* const waveFileName) override;
	virtual void RemoveSound(ndSharedPtr<ndSoundSource>& sound) override;
	virtual void ClearSounds() override;

	bool LoadWaveFile(ALuint buffer, const char* const waveFileName) const;
	ALCdevice* m_device;
	ALCcontext* m_context;
	ALboolean m_extension;
	ndTree<ALuint, ndString> m_buffersCache;
	static ndMatrix m_newtonToOpenAl;
	static ndMatrix m_openAlToNewton;
};

class ndOpenAlSource: public ndSoundSource::Implementation
{
	public:
	ndOpenAlSource(ndSharedPtr<ndSoundManager>& owner, const char* const waveFileName);
	virtual ~ndOpenAlSource();

	virtual void Play() override;
	virtual void Stop() override;
	virtual bool IsPlaying() const override;

	virtual bool IsLooping() const override;
	virtual void SetLooping(bool state) override;
	virtual void SetPitch(ndFloat32 pitch) override;
	virtual void SetVolume(ndFloat32 volume) override;
	virtual void SetPosition(const ndVector& posit) override;
	virtual void SetVelocity(const ndVector& veloc) override;

	ALuint m_source;
};

ndMatrix ndOpenAlManager::m_newtonToOpenAl(ndYawMatrix(ndFloat32(90.0f)* ndDegreeToRad));
ndMatrix ndOpenAlManager::m_openAlToNewton(ndYawMatrix(ndFloat32(-90.0f)* ndDegreeToRad));

ndOpenAlSource::ndOpenAlSource(ndSharedPtr<ndSoundManager>& owner, const char* const waveFileName)
	:ndSoundSource::Implementation(owner)
{
	ndOpenAlManager* const manager = (ndOpenAlManager*)m_manager->m_implementation;
	ndTree<ALuint, ndString>::ndNode* const resource = manager->m_buffersCache.Find(waveFileName);
	ndAssert(resource);
	ALuint buffer = resource->GetInfo();
	
	// Generate Source
	alGenSources(1, &m_source);
	ndAssert(alGetError() == AL_NO_ERROR);
	
	// bind buffer to source
	alSourcei(m_source, AL_BUFFER, ALint(buffer));
	ndAssert(alGetError() == AL_NO_ERROR);
	
	// Explicitly ensure the sound is relative to the world, not the listener
	alSourcei(m_source, AL_SOURCE_RELATIVE, AL_FALSE);

	// set the volume
	SetVolume(ndFloat32(1.0f));
}

ndOpenAlSource::~ndOpenAlSource()
{
	alDeleteSources(1, &m_source);
	ndAssert(alGetError() == AL_NO_ERROR);
}

void ndOpenAlSource::SetLooping(bool state)
{
	m_isLooping = state;
	alSourcei(m_source, AL_LOOPING, m_isLooping ? AL_TRUE : AL_FALSE);
}

bool ndOpenAlSource::IsLooping() const
{
	return m_isLooping;
}

void ndOpenAlSource::Play()
{
	Implementation::Play();
	if (m_source)
	{
		alSourcePlay(m_source);
	}
}

void ndOpenAlSource::Stop()
{
	Implementation::Play();
	if (m_source)
	{
		alSourceStop(m_source);
	}
}

bool ndOpenAlSource::IsPlaying() const
{ 
	bool playing = Implementation::IsPlaying();
	if (m_source)
	{
		ALint sourceState;
		alGetSourcei(m_source, AL_SOURCE_STATE, &sourceState);
		playing = sourceState ? true : false;
	}
	return playing;
}

void ndOpenAlSource::SetPosition(const ndVector& posit)
{
	Implementation::SetPosition(posit);
	if (m_source)
	{
		const ndVector alPosit(ndOpenAlManager::m_newtonToOpenAl.RotateVector(posit));
		alSource3f(m_source, AL_POSITION, ALfloat(alPosit.m_x), ALfloat(alPosit.m_y), ALfloat(alPosit.m_z));
	}
}

void ndOpenAlSource::SetVelocity(const ndVector& veloc)
{
	Implementation::SetVelocity(veloc);
	if (m_source)
	{
		const ndVector alVeloc(ndOpenAlManager::m_newtonToOpenAl.RotateVector(veloc));
		alSource3f(m_source, AL_VELOCITY, ALfloat(alVeloc.m_x), ALfloat(alVeloc.m_y), ALfloat(alVeloc.m_z));
	}
}

void ndOpenAlSource::SetVolume(ndFloat32 volume)
{
	Implementation::SetVolume(volume);
	if (m_source)
	{
		alSourcef(m_source, AL_GAIN, ALfloat(m_volume));
	}
}

void ndOpenAlSource::SetPitch(ndFloat32 pitch)
{
	Implementation::SetVolume(pitch);
	if (m_source)
	{
		alSourcef(m_source, AL_PITCH, m_pitch);
	}
}

ndOpenAlManager::ndOpenAlManager(ndDemoEntityManager* const owner)
	:Implementation(owner)
	,m_device(nullptr)
	,m_context(nullptr)
	,m_extension(false)
{
	m_device = alcOpenDevice(nullptr);

	if (m_device)
	{
		m_context = alcCreateContext(m_device, nullptr);
		alcMakeContextCurrent(m_context);
	}
	// Check for EAX 2.0 support
	m_extension = alIsExtensionPresent("EAX2.0");
	ndAssert(alGetError() == AL_NO_ERROR);
}

ndOpenAlManager::~ndOpenAlManager()
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

bool ndOpenAlManager::LoadWaveFile(ALuint buffer, const char* const waveFileName) const
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

void ndOpenAlManager::Update(const ndMatrix& listenerMatrix, const ndVector& listenerVeloc)
{
	ALfloat posit[3];
	ALfloat veloc[3];
	ALfloat orientation[6];
	const ndMatrix alMatrix(listenerMatrix * m_newtonToOpenAl);
	const ndVector alVeloc(m_newtonToOpenAl.RotateVector(listenerVeloc));

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
	Implementation::Update(listenerMatrix, listenerVeloc);
}

ndSharedPtr<ndSoundSource> ndOpenAlManager::AddSound(const char* const waveFileName)
{
	const ndString name(waveFileName);
	const ndTree<ALuint, ndString>::ndNode* const resource = m_buffersCache.Find(name);
	if (!resource)
	{
		// generate buffer
		ALuint buffer;
		alGenBuffers(1, &buffer);
		ndAssert(alGetError() == AL_NO_ERROR);
		LoadWaveFile(buffer, waveFileName);
		m_buffersCache.Insert(buffer, name);
	}
	return Implementation::AddSound(waveFileName);
}

void ndOpenAlManager::RemoveSound(ndSharedPtr<ndSoundSource>& sound)
{
	ndAssert(0);
	//sound->Stop();
	//ndAssert(sound->m_implementation->m_sceneNode);
	//m_soundScene.Remove(sound->m_implementation->m_sceneNode);
}

void ndOpenAlManager::ClearSounds()
{
	//ndAssert(0);
	//ndAssert(m_soundScene.GetCount() == 0);
	//while (m_soundScene.GetCount())
	//{
	//	ndList<ndWeakPtr<ndSoundSource>>::ndNode* const node = m_soundScene.GetLast();
	//	ndSoundSource* const sound = *node->GetInfo();
	//	sound
	//
	//	//manager->RemoveSound(m_soundScene.GetLast()->GetInfo());
	//}
	Implementation::ClearSounds();
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

ndSoundSource::ndSoundSource(ndSharedPtr<ndSoundManager>& owner, const char* const waveFileName)
	:ndClassAlloc()
	,m_implementation(nullptr)
{
	#if defined (ND_OPEN_AL)
	{
		m_implementation = new ndOpenAlSource(owner, waveFileName);
	}
	#else
		m_implementation = new Implementation(owner, waveFileName);
	#endif
}

ndSoundSource::~ndSoundSource()
{
	delete m_implementation;
};

void ndSoundSource::Play()
{
	m_implementation->Play();
}

void ndSoundSource::Stop()
{
	m_implementation->Stop();
}

bool ndSoundSource::IsPlaying() const
{
	return m_implementation->IsPlaying();
}

void ndSoundSource::SetLooping(bool state)
{
	m_implementation->SetLooping(state);
}

bool ndSoundSource::IsLooping() const
{
	return m_implementation->IsLooping();
}

void ndSoundSource::SetPosition(const ndVector& posit)
{
	m_implementation->SetPosition(posit);
}

void ndSoundSource::SetVelocity(const ndVector& veloc)
{
	m_implementation->SetVelocity(veloc);
}

ndVector ndSoundSource::GetPosition() const
{
	return m_implementation->GetPosition();
}

ndVector ndSoundSource::GetVelocity() const
{
	return m_implementation->GetVelocity();
}

ndSharedPtr<ndSoundSourceNotify> ndSoundSource::GetNotify() const
{
	return m_implementation->GetNotify();
}

void ndSoundSource::SetNotify(ndSharedPtr<ndSoundSourceNotify> notify)
{
	m_implementation->SetNotify(notify);
}

ndFloat32 ndSoundSource::GetVolume() const
{
	return m_implementation->GetVolume();
}

void ndSoundSource::SetVolume(ndFloat32 volume)
{
	m_implementation->SetVolume(volume);
}

ndFloat32 ndSoundSource::GetPitch() const
{
	return m_implementation->GetPitch();
}

void ndSoundSource::SetPitch(ndFloat32 pitch)
{
	m_implementation->SetPitch(pitch);
}

ndSoundManager::ndSoundManager(ndDemoEntityManager* const owner)
	:ndClassAlloc()
	,m_implementation(nullptr)
{
	#if defined (ND_OPEN_AL)
	{
		m_implementation = new ndOpenAlManager(owner);
	}
	#else
		m_implementation = new Implementation(owner);
	#endif
}

ndSoundManager::~ndSoundManager()
{
	ClearSounds();
	delete m_implementation;
}

void ndSoundManager::Update(const ndMatrix& listenerMatrix, const ndVector& listenerVeloc)
{
	m_implementation->Update(listenerMatrix, listenerVeloc);
}

ndSharedPtr<ndSoundSource> ndSoundManager::AddSound(const char* const waveFileName)
{
	ndSharedPtr<ndSoundSource> sound (m_implementation->AddSound(waveFileName));
	return sound;
}

void ndSoundManager::RemoveSound(ndSharedPtr<ndSoundSource>& sound)
{
	m_implementation->RemoveSound(sound);
}

void ndSoundManager::ClearSounds()
{
	m_implementation->ClearSounds();
}

