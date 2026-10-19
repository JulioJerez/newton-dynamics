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
#include "NewtonUtils.h"
#include "newtonWorld.h"
#include "newtonMaterial.h"
#include "newtonBodyNotify.h"

class NewtonListenerNotify : public ndModelNotify
{
	public:
	D_CLASS_REFLECTION(NewtonListenerNotify, ndModelNotify)

	typedef void (*NewtonWorldDestroyListenerCallback) (const NewtonWorld* const world, void* const listenerUserData);
	typedef void (*NewtonWorldListenerDebugCallback) (const NewtonWorld* const world, void* const listener, void* const debugContext);
	typedef void (*NewtonWorldUpdateListenerCallback) (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep);
	typedef void (*NewtonWorldListenerBodyDestroyCallback) (const NewtonWorld* const world, void* const listenerUserData, NewtonBody* const body);

	NewtonListenerNotify(void* const userdata)
		:ndModelNotify()
		,m_userData(userdata)
		,m_onDebug(nullptr)
		,m_onDestroy(nullptr)
		,m_onPostStep(nullptr)
		,m_onPreUpdate(nullptr)
		,m_onPostUpdate(nullptr)
		,m_onBodyDestroy(nullptr)
	{
	}

	~NewtonListenerNotify() override
	{
		if (m_onDestroy)
		{
			ndModel* const model = GetModel();
			ndWorld* const world = model->GetWorld();
			m_onDestroy(PointerCast<NewtonWorld, ndWorld>(world), *m_userData);
		}
	}

	virtual bool OnContactGeneration(const ndBodyKinematic* const body0, const ndBodyKinematic* const body1) override
	{
		return true;
	}

	virtual void Debug(ndConstraintDebugCallback& context) const override
	{
	}

	// mapping this call to the update after the collision update
	virtual void Update(ndFloat32 timestep, ndInt32 threadId) override
	{
		if (m_onPreUpdate)
		{
			ndModel* const model = GetModel();
			ndWorld* const world = model->GetWorld();
			m_onPreUpdate(PointerCast<NewtonWorld, ndWorld>(world), *m_userData, timestep);
		}
	}

	virtual void PostUpdate(ndFloat32 timestep, ndInt32 threadId) override
	{
		if (m_onPostUpdate)
		{
			ndModel* const model = GetModel();
			ndWorld* const world = model->GetWorld();
			m_onPostUpdate(PointerCast<NewtonWorld, ndWorld>(world), *m_userData, timestep);
		}
	}

	virtual void PostTransformUpdate(ndFloat32 timestep, ndInt32 threadId) override
	{
		if (m_onPostStep)
		{
			ndModel* const model = GetModel();
			ndWorld* const world = model->GetWorld();
			m_onPostStep(PointerCast<NewtonWorld, ndWorld>(world), *m_userData, timestep);
		}
	}

	ndString m_name;
	ndWeakPtr<void> m_userData;
	NewtonWorldListenerDebugCallback m_onDebug;
	NewtonWorldDestroyListenerCallback m_onDestroy;
	NewtonWorldUpdateListenerCallback m_onPostStep;
	NewtonWorldUpdateListenerCallback m_onPreUpdate;
	NewtonWorldUpdateListenerCallback m_onPostUpdate;
	NewtonWorldListenerBodyDestroyCallback m_onBodyDestroy;
};

class NewtonListener : public ndModel
{
	public:
	D_CLASS_REFLECTION(NewtonListener, ndModel)

	NewtonListener(const char* const name, void* const userData)
		:ndModel()
	{
		ndSharedPtr<ndModelNotify> notify (new NewtonListenerNotify(userData));
		SetNotifyCallback(notify);
		NewtonListenerNotify* const litener = PointerCast<NewtonListenerNotify, ndModelNotify>(*notify);
		litener->m_name = name;
	}

	~NewtonListener() override
	{
	}
};

NewtonWorldListenerBodyDestroyCallback NewtonWorldListenerGetBodyDestroyCallback(const NewtonWorld* const newtonWorld, void* const listener)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return (NewtonWorldListenerBodyDestroyCallback)world->GetListenerBodyDestroyCallback(listener);
	ndAssert(0);
	return 0;
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
	ndNewtonWorld* const world = const_cast<ndNewtonWorld*>(reinterpret_cast<const ndNewtonWorld*>(newtonWorld));

	ndSharedPtr<ndModel> listener(new NewtonListener(nameId, listenerUserData));
	world->AddModel(listener);

	return *listener;
}

// new for newton 3.15
void NewtonWorldRemoveListener(const void* const listenerPtr)
{
	NewtonListener* const listener = PointerCast<NewtonListener, void>(listenerPtr);
	ndAssert(listener->IsType(listener->StaticClassName()));
	ndWorld* const world = listener->GetWorld();
	world->RemoveModel(listener);
}

void NewtonWorldListenerSetDebugCallback(const NewtonWorld* const, void* const listenerPtr, NewtonWorldListenerDebugCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonListener* const listener = PointerCast<NewtonListener, void>(listenerPtr);
	ndAssert(listener->IsType(listener->StaticClassName()));

	ndSharedPtr<ndModelNotify>& baseNofify = listener->GetNotifyCallback();
	NewtonListenerNotify* const notify = PointerCast<NewtonListenerNotify, ndModelNotify>(*baseNofify);
	notify->m_onDebug = callback;
}

void NewtonWorldListenerSetDestructorCallback(const NewtonWorld* const, void* const listenerPtr, NewtonWorldDestroyListenerCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonListener* const listener = PointerCast<NewtonListener, void>(listenerPtr);
	ndAssert(listener->IsType(listener->StaticClassName()));

	ndSharedPtr<ndModelNotify>& baseNofify = listener->GetNotifyCallback();
	NewtonListenerNotify* const notify = PointerCast<NewtonListenerNotify, ndModelNotify>(*baseNofify);

	notify->m_onDestroy = callback;
}

void NewtonWorldListenerSetPostStepCallback(const NewtonWorld* const, void* const listenerPtr, NewtonWorldUpdateListenerCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonListener* const listener = PointerCast<NewtonListener, void>(listenerPtr);
	ndAssert(listener->IsType(listener->StaticClassName()));

	ndSharedPtr<ndModelNotify>& baseNofify = listener->GetNotifyCallback();
	NewtonListenerNotify* const notify = PointerCast<NewtonListenerNotify, ndModelNotify>(*baseNofify);

	notify->m_onPostStep = callback;
}

void NewtonWorldListenerSetPreUpdateCallback(const NewtonWorld* const, void* const listenerPtr, NewtonWorldUpdateListenerCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonListener* const listener = PointerCast<NewtonListener, void>(listenerPtr);
	ndAssert(listener->IsType(listener->StaticClassName()));

	ndSharedPtr<ndModelNotify>& baseNofify = listener->GetNotifyCallback();
	NewtonListenerNotify* const notify = PointerCast<NewtonListenerNotify, ndModelNotify>(*baseNofify);
	notify->m_onPreUpdate = callback;
}

void NewtonWorldListenerSetPostUpdateCallback(const NewtonWorld* const, void* const listenerPtr, NewtonWorldUpdateListenerCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonListener* const listener = PointerCast<NewtonListener, void>(listenerPtr);
	ndAssert(listener->IsType(listener->StaticClassName()));

	ndSharedPtr<ndModelNotify>& baseNofify = listener->GetNotifyCallback();
	NewtonListenerNotify* const notify = PointerCast<NewtonListenerNotify, ndModelNotify>(*baseNofify);
	notify->m_onPostUpdate = callback;
}

void NewtonWorldListenerSetBodyDestroyCallback(const NewtonWorld* const, void* const listenerPtr, NewtonWorldListenerBodyDestroyCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonListener* const listener = PointerCast<NewtonListener, void>(listenerPtr);
	ndAssert(listener->IsType(listener->StaticClassName()));

	ndSharedPtr<ndModelNotify>& baseNofify = listener->GetNotifyCallback();
	NewtonListenerNotify* const notify = PointerCast<NewtonListenerNotify, ndModelNotify>(*baseNofify);

	notify->m_onBodyDestroy = callback;
}

void* NewtonWorldGetListenerUserData(const NewtonWorld* const, void* const listenerPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	NewtonListener* const listener = PointerCast<NewtonListener, void>(listenerPtr);
	ndAssert(listener->IsType(listener->StaticClassName()));

	ndSharedPtr<ndModelNotify>& baseNofify = listener->GetNotifyCallback();
	NewtonListenerNotify* const notify = PointerCast<NewtonListenerNotify, ndModelNotify>(*baseNofify);
	return *notify->m_userData;
}


void* NewtonWorldGetListener(const NewtonWorld* const newtonWorld, const char* const nameId)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);

	const ndModelList& modelList = world->GetModelList();
	for (ndModelList::ndNode* node = modelList.GetFirst(); node; node = node->GetNext())
	{
		ndModel* model = *node->GetInfo();
		NewtonListenerNotify* const notify = PointerCast<NewtonListenerNotify, ndModelNotify>(*model->GetNotifyCallback());
		if (strcmp(notify->m_name.GetStr(), nameId) == 0)
		{
			return model;
		}
	}
	return nullptr;
}
