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

#include "ndTracyProfiler.h"

//#if !defined (WIN32) || (_MSC_VER >= 1900)
#if defined (WIN32)

	#include "Tracy.hpp"
	#include "common\TracySystem.hpp"
	#include "client\TracyProfiler.hpp"

	using namespace tracy;

	static bool profileOn = false;

	void ndPropfileZone::TraceSample()
	{
		tracy::SourceLocationData* const contact = (tracy::SourceLocationData*)&m_tracyConcat;
		tracy::ScopedZone varname(contact, profileOn);
	}

	void ndProfilerFamerMarker()
	{
		if (profileOn)
		{
			FrameMark;
		}
	}

	void ndProfilerStartSampling()
	{
		profileOn = !profileOn;
	}

	void ndProfilerSetThreadName(const char* const threadName)
	{
		tracy::SetThreadName(threadName);
	}

#else

	void dProfilerEnableProlingLow(int mode)
	{
		mode = 0;
	}

	long long dProfilerStartTraceLow(const dProfilerSourceLocation* const)
	{
		return 0;
	}

	void dProfilerEndTraceLow(int long long)
	{
	}

	void dProfilerSetTrackNameLow(const char* const)
	{
	}

#endif
