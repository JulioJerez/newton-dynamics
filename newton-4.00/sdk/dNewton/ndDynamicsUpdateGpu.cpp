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
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndSkeletonList.h"
#include "ndDynamicsUpdateGpu.h"
#include "ndJointBilateralConstraint.h"


ndDynamicsUpdateGpu::ndDynamicsUpdateGpu(ndWorld* const world)
	:ndDynamicsUpdate(ndDynamicsUpdate(world))
	,m_context(nullptr)
{
	ndSharedPtr<ndGpuDriver> driver (ndGpuDriver::Create());
	if (driver->IsValid())
	{
		ndScene* const scene = world->GetScene();
		scene->SetGpuContext(driver);
		m_context = ndWeakPtr<ndGpuDriver>(*driver);
	}
}

ndDynamicsUpdateGpu::~ndDynamicsUpdateGpu()
{
}


void ndDynamicsUpdateGpu::Update()
{
	if (m_context)
	{
		UpdateGpu();
	}
	else
	{
		// if not GPU found, default to cpu 
		ndDynamicsUpdate::Update();
	}
}

void ndDynamicsUpdateGpu::UpdateGpu()
{
	ndDynamicsUpdate::Update();
}

