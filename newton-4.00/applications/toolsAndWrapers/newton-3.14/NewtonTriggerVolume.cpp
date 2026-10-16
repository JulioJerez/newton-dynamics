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

class NewtonTriggerVolume : public ndBodyTriggerVolume
{
	public:
	D_CLASS_REFLECTION(NewtonTriggerVolume, ndBodyTriggerVolume)

	typedef void (*NewtonTriggerEnterVolume)(const NewtonBody* const trigger, const NewtonBody* const body, dFloat timestep);
	typedef void (*NewtonTriggerExitVolume)(const NewtonBody* const trigger, const NewtonBody* const body, dFloat timestep);
	typedef void (*NewtonTriggerInVolume)(const NewtonBody* const trigger, const NewtonBody* const body, const NewtonJoint* const joint, dFloat timestep);

	NewtonTriggerVolume()
		:ndBodyTriggerVolume()
	{
	}

	virtual void OnTrigger(const ndContact* const contact, ndFloat32 timestep) override
	{
		if (m_onInVolume)
		{
			ndBody* const body = (contact->GetBody0() != this) ? contact->GetBody0() : contact->GetBody1();
			m_onInVolume(PointerCast<NewtonBody, NewtonTriggerVolume>(this), PointerCast<NewtonBody, ndBody>(body), PointerCast<NewtonJoint, ndContact>(contact), timestep);
		}
	}

	virtual void OnTriggerEnter(ndBodyKinematic* const body, ndFloat32 timestep) override
	{
		if (m_onEnterVolume)
		{
			m_onEnterVolume(PointerCast<NewtonBody, NewtonTriggerVolume>(this), PointerCast<NewtonBody, ndBodyKinematic>(body), timestep);
		}
	}

	virtual void OnTriggerExit(ndBodyKinematic* const body, ndFloat32 timestep) override
	{
		if (m_onExitVolume)
		{
			m_onExitVolume(PointerCast<NewtonBody, NewtonTriggerVolume>(this), PointerCast<NewtonBody, ndBodyKinematic>(body), timestep);
		}
	}

	NewtonTriggerInVolume m_onInVolume;
	NewtonTriggerExitVolume m_onExitVolume;
	NewtonTriggerEnterVolume m_onEnterVolume;
};

NewtonBody* NewtonCreateTriggerVolume(const NewtonWorld* const newtonWorld, const NewtonCollision* const collision, const dFloat* const matrixPtr,
	NewtonTriggerEnterVolume enterVolume, NewtonTriggerExitVolume onExit, NewtonTriggerInVolume onTrigger)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(GetMatrix(matrixPtr));

	matrix.m_front.m_w = ndFloat32(0.0f);
	matrix.m_up.m_w = ndFloat32(0.0f);
	matrix.m_right.m_w = ndFloat32(0.0f);
	matrix.m_posit.m_w = ndFloat32(1.0f);

	ndSharedPtr<ndBody> body(new NewtonTriggerVolume());
	NewtonTriggerVolume* const trigger = PointerCast<NewtonTriggerVolume, ndBodyTriggerVolume>(body->GetAsBodyTriggerVolume());

	trigger->m_onExitVolume = onExit;
	trigger->m_onInVolume = onTrigger;
	trigger->m_onEnterVolume = enterVolume;

	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndSharedPtr<ndBodyNotify> bodyNotify(new ndNewtonBodyNotify(world, reinterpret_cast<NewtonBody*>(*body)));

	ndShapeInstance instance(nullptr);
	if (collision)
	{
		instance = *PointerCast<ndShapeInstance, NewtonCollision>(collision);
	}
	else
	{
		instance.SetShape(new ndShapeNull());
	}

	trigger->SetMatrix(matrix);
	trigger->SetCollisionShape(instance);
	trigger->SetNotifyCallback(bodyNotify);
	world->AddBody(body);
	return PointerCast<NewtonBody, NewtonTriggerVolume>(trigger);
}

