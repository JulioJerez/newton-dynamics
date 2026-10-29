/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef D_NEWTON_BOBY_H_
#define D_NEWTON_BOBY_H_

#include "newtonStdafx.h"
#include "Newton.h"
#include "NewtonUtils.h"
#include "newtonWorld.h"

struct NewtonBody;
struct NewtonJoint;
struct NewtonWorld;

class ndNewtonBodyNotify : public ndModelBodyNotify
{
	public:
	D_CLASS_REFLECTION(ndNewtonBodyNotify, ndModelBodyNotify)

	typedef void (*NewtonBodyDestructor) (const NewtonBody* const body);
	typedef void (*NewtonApplyForceAndTorque) (const NewtonBody* const body);
	typedef void (*NewtonBodyActivationState) (const NewtonBody* body, unsigned state);
	typedef void (*NewtonSetTransform) (const NewtonBody* const body, const ndFloat32* const matrix);

	ndNewtonBodyNotify(ndNewtonWorld* const world, NewtonBody* const owner);
	ndNewtonBodyNotify(const ndNewtonBodyNotify& notify);
	virtual ~ndNewtonBodyNotify() override;

	ndBodyNotify* Clone() const override;
	virtual void OnBodyAddedToWorld();
	virtual void OnBodyRemovedFromWorld();
	virtual void OnTransform(ndFloat32 timestep, const ndMatrix& matrix) override;

	virtual void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep) override;

	ndWeakPtr<void> m_userData;
	ndWeakPtr<NewtonBody> m_owner;
	ndWeakPtr<ndNewtonWorld> m_world;
	ndInt32 m_materialGoupId;
	ndFloat32 m_capSpeed;
	ndFloat32 m_capOmega;

	bool m_bodyIsInWorld;
	bool m_collideWithConnectedBodies;

	NewtonBodyDestructor m_onDestroy;
	NewtonSetTransform m_applyTransform;
	NewtonApplyForceAndTorque m_onForceAndTorque;
};

#endif