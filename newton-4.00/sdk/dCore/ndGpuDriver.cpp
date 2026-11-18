/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndCoreStdafx.h"
#include "ndGpuDriver.h"


#if !defined (ND_USE_GPU_PHYSICS)

class ndGpuDriver::ndInternal : public ndClassAlloc
{
	public:
	ndInternal()
		:ndClassAlloc()
	{
	}

	~ndInternal()
	{
	}
};

ndGpuDriver::ndGpuDriver()
	:ndClassAlloc()
	,m_implementation(nullptr)
{
}

ndGpuDriver::~ndGpuDriver()
{
}

bool ndGpuDriver::IsValid() const
{
	return false;
}

ndGpuDriver* ndGpuDriver::Create()
{
	return new ndGpuDriver();
}

#else
//#ifdef D_USE_OPENCL_SDK
//	#define CL_HPP_TARGET_OPENCL_VERSION 300
//	#include <CL/opencl.hpp>
//#endif

class ndGpuDriver::ndInternal : public ndClassAlloc
{
	public:
	ndInternal()
		:ndClassAlloc()
		,m_isValid(true)
	{
		//std::vector<cl::Device> cl_devices;
		//{
		//	std::vector<cl::Platform> cl_platforms;
		//	cl::Platform::get(&cl_platforms);
		//	ndExpandTraceMessage("\n");
		//	ndExpandTraceMessage("platforms found:\n");
		//	for (size_t i = 0; i < cl_platforms.size(); i++)
		//	{
		//		std::vector<cl::Device> cl_devices_available;
		//		cl_platforms[i].getDevices(ND_OPENCL_SELECTION_TYPE, &cl_devices_available);
		//		for (size_t j = 0; j < cl_devices_available.size(); j++)
		//		{
		//			const std::string name(cl_platforms[i].getInfo<CL_PLATFORM_NAME>());
		//			ndExpandTraceMessage("opencl platform: %s\n", name.c_str());
		//			cl_devices.push_back(cl_devices_available[j]);
		//		}
		//	}
		//}
		//
		//// select fastest available device
		//if (cl_devices.size())
		//{
		//	size_t bestScore = 0;
		//	size_t bestDeviceIndex = 0;
		//	for (size_t i = 0; i < cl_devices.size(); ++i)
		//	{
		//		size_t score = GetDeviceScore(cl_devices[i]);
		//		if (score > bestScore)
		//		{
		//			bestScore = score;
		//			bestDeviceIndex = i;
		//		}
		//	}
		//	//bestDeviceIndex = 4;
		//
		//	m_device = ndSharedPtr<cl::Device>(new cl::Device(cl_devices[bestDeviceIndex]));
		//	const std::string name(m_device->getInfo<CL_DEVICE_NAME>());
		//	const std::string version(m_device->getInfo<CL_DEVICE_VERSION>());
		//	size_t localMemorySize = m_device->getInfo<CL_DEVICE_LOCAL_MEM_SIZE>();
		//	size_t computeUnits = m_device->getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>();
		//	m_maxBufferSize = m_device->getInfo<CL_DEVICE_MAX_MEM_ALLOC_SIZE>();
		//
		//	ndExpandTraceMessage("\n");
		//	ndExpandTraceMessage("selecting:\n");
		//	ndExpandTraceMessage("open cl device name: %s\n", name.c_str());
		//	ndExpandTraceMessage("open cl device version: %s\n", version.c_str());
		//
		//	ndExpandTraceMessage("open cl device compute units: %d\n", computeUnits);
		//	ndExpandTraceMessage("open cl device local memory: %d\n", localMemorySize);
		//	ndExpandTraceMessage("\n");
		//
		//	cl_int error = 0;
		//	m_context = ndSharedPtr<cl::Context>(new cl::Context(**m_device, nullptr, clNotification, this, &error));
		//	ndAssert(error == CL_SUCCESS);
		//
		//	m_emptyBuffer = cl::Buffer(**m_context, CL_MEM_READ_WRITE, 256);
		//
		//	CreateQueue();
		//	CreateKerners();
		//	CreateCopyCommands();
		//}
	}

	~ndInternal()
	{
		ndAssert(0);
	}

	bool IsValid() const
	{
		ndAssert(0);
		return false;
	}

	bool m_isValid;
};

ndGpuDriver::ndGpuDriver()
	:ndClassAlloc()
	,m_implementation(nullptr)
{
}

ndGpuDriver::~ndGpuDriver()
{
	ndAssert(0);
}

bool ndGpuDriver::IsValid() const
{
	return true;
}

ndGpuDriver* ndGpuDriver::Create()
{
	ndGpuDriver* const driver = new ndGpuDriver();
	if (driver->IsValid())
	{
		return driver;
	}
	delete driver;
	return nullptr;
}

#endif