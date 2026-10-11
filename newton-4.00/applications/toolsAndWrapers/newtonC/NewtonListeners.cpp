/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#include "newtonStdafx.h"
#include "Newton.h"
#include "newtonWorld.h"
#include "newtonMaterial.h"
#include "newtonBodyNotify.h"

void* NewtonWorldGetListener(const NewtonWorld* const newtonWorld, const char* const nameId)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return world->FindListener(nameId);
	ndAssert(0);
	return 0;
}

void* NewtonWorldGetListenerUserData(const NewtonWorld* const newtonWorld, void* const listener)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return world->GetListenerUserData(listener);
	ndAssert(0);
	return 0;
}

NewtonWorldListenerBodyDestroyCallback NewtonWorldListenerGetBodyDestroyCallback(const NewtonWorld* const newtonWorld, void* const listener)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return (NewtonWorldListenerBodyDestroyCallback)world->GetListenerBodyDestroyCallback(listener);
	ndAssert(0);
	return 0;
}

void NewtonWorldListenerSetBodyDestroyCallback(const NewtonWorld* const newtonWorld, void* const listener, NewtonWorldListenerBodyDestroyCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//world->SetListenerBodyDestroyCallback(listener, (dgWorld::OnListenerBodyDestroyCallback)callback);
	ndAssert(0);
}

void NewtonWorldListenerSetDestructorCallback(const NewtonWorld* const newtonWorld, void* const listener, NewtonWorldDestroyListenerCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return world->ListenerSetDestroyCallback(listener, (dgWorld::OnListenerDestroyCallback)callback);
	ndAssert(0);
}

void NewtonWorldListenerSetPreUpdateCallback(const NewtonWorld* const newtonWorld, void* const listener, NewtonWorldUpdateListenerCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return world->ListenerSetPreUpdate(listener, (dgWorld::OnListenerUpdateCallback)callback);
	ndAssert(0);
}

void NewtonWorldListenerSetPostUpdateCallback(const NewtonWorld* const newtonWorld, void* const listener, NewtonWorldUpdateListenerCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return world->ListenerSetPostUpdate(listener, (dgWorld::OnListenerUpdateCallback)callback);
	ndAssert(0);
}

void NewtonWorldListenerSetPostStepCallback(const NewtonWorld* const newtonWorld, void* const listener, NewtonWorldUpdateListenerCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return world->ListenerSetPostStep(listener, (dgWorld::OnListenerUpdateCallback)callback);
	ndAssert(0);
}

void NewtonWorldListenerSetDebugCallback(const NewtonWorld* const newtonWorld, void* const listener, NewtonWorldListenerDebugCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return world->SetListenerBodyDebugCallback(listener, (dgWorld::OnListenerDebugCallback)callback);
	ndAssert(0);
}

void NewtonWorldListenerDebug(const NewtonWorld* const newtonWorld, void* const context)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return world->ListenersDebug(context);
	ndAssert(0);
}


void* NewtonWorldAddListener(const NewtonWorld* const newtonWorld, const char* const nameId, void* const listenerUserData)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return world->AddListener(nameId, listenerUserData);
	ndAssert(0);
	return 0;
}
