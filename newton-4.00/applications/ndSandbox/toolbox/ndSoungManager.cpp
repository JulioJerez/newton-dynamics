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
	Implementation(const char* const)
		:ndClassAlloc()
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
};

#else
#include "AL/al.h"
#include "AL/alext.h"

class ndSoundBuffer::Implementation: public ndClassAlloc
{
	public:
	Implementation(const char* const waveFileName)
		:ndClassAlloc()
	{
		// generate buffer
		alGenBuffers(1, &m_buffer);
		ndAssert(alGetError() == AL_NO_ERROR);
		LoadWaveFile(waveFileName);

		// Generate Source
		alGenSources(1, &m_source);
		ndAssert(alGetError() == AL_NO_ERROR);

		// bind buffer to source
		alSourcei(m_source, AL_BUFFER, ALint(m_buffer));
		ndAssert(alGetError() == AL_NO_ERROR);
	}

	~Implementation()
	{
		// delete source
		alDeleteSources(1, &m_source);
		ndAssert(alGetError() == AL_NO_ERROR);

		// delete buffer
		alDeleteBuffers(1, &m_buffer);
		ndAssert(alGetError() == AL_NO_ERROR);
	}

	void SetLooping(bool state)
	{
		m_isLooping = state;
		alSourcei(m_source, AL_LOOPING, m_isLooping ? AL_TRUE : AL_FALSE);
	}

	bool IsLooping() const
	{
		return m_isLooping;
	}

	void Play()
	{
		if (m_source && m_buffer)
		{
			alSourcePlay(m_source);
		}
	}

	void Stop()
	{
		if (m_source && m_buffer)
		{
			alSourceStop(m_source);
		}
	}

	bool LoadWaveFile(const char* const waveFileName) const
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
		alBufferData(m_buffer, format, &pcmData[0], ALsizei(header.dataSize), ALsizei(header.sampleRate));
		ndAssert(alGetError() == AL_NO_ERROR);
		return true;
	}


	ALuint m_buffer;
	ALuint m_source;
	bool m_isLooping;
};

class ndSoundManager::Implementation: public ndClassAlloc
{
	public:
	Implementation()
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

	virtual ~Implementation()
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

	ALCdevice* m_device;
	ALCcontext* m_context;
	ALboolean m_extension;
};
#endif


ndSoundManager::ndSoundManager()
	:ndClassAlloc()
	,m_implementation(new Implementation())
{
	//ndSharedPtr<ndSoundBuffer> test(new ndSoundBuffer("diesel_engine.wav"));
	//test->SetLooping(true);
	//test->Play();
	//Sleep(10000);
	//test->Stop();
}

ndSoundManager::~ndSoundManager()
{
	delete m_implementation;
}

ndSoundBuffer::ndSoundBuffer(const char* const waveFileName)
	:ndClassAlloc()
	,m_implementation(new Implementation(waveFileName))
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
