/* Copyright (c) <2018-2018> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __D_TRACY_PROFILER_H__
#define __D_TRACY_PROFILER_H__


#ifdef D_PROFILER_EXPORTS
#define D_PROFILER_API __declspec(dllexport)
#else
#define D_PROFILER_API __declspec(dllimport)
#endif

struct ndProfilerSourceLocation
{
	const char* name;
	const char* function;
	const char* file;
	long long line;
	long long color;
};

#ifdef D_PROFILER

D_PROFILER_API void ndProfilerFamerMarker();
D_PROFILER_API void ndProfilerStartSampling();
D_PROFILER_API void ndProfilerSetThreadName(const char* const threadName);

class ndPropfileZone
{
	public:
	ndPropfileZone(const char* const name, const char* const file, unsigned lineNumber)
	{
		m_tracyConcat.name = nullptr;
		m_tracyConcat.function = name;
		m_tracyConcat.file = file;
		m_tracyConcat.line = lineNumber;
		m_tracyConcat.color = 0;
	}

	~ndPropfileZone()
	{
	}

	D_PROFILER_API void TraceSample();

	ndProfilerSourceLocation m_tracyConcat;
};

#define ndProfilerScopedZone(name)									\
	static ndPropfileZone __hotSpot__ (name, __FILE__, __LINE__);	\
	__hotSpot__.TraceSample();

#define ndProfilerFrameMarker() 

#else

#define ndProfilerFamerMarker()
#define ndProfilerStartSampling()
#define ndProfilerScopedZone(name)
#define ndProfilerSetThreadName(threadName)

#endif

#endif