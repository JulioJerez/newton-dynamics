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

#ifndef D_NEWTON_VEHICLE_H_
#define D_NEWTON_VEHICLE_H_

#include "newtonStdafx.h"
#include "Newton.h"

class NewtonConvexCastVehicle : public ndConvexCastVehicle
{
	public:
	D_CLASS_REFLECTION(NewtonConvexCastVehicle, ndConvexCastVehicle)

	typedef void (*NewtonDrawDebugLine) (const dFloat* const p0, const dFloat* const p1, const dFloat* const color);

	NewtonConvexCastVehicle(const ndVector& gravityDir);

	virtual ~NewtonConvexCastVehicle() override;

	virtual void OnAddToWorld() override;
	virtual void Update(ndFloat32 timestep, ndInt32 threadId) override;
	virtual void Debug(ndConstraintDebugCallback& context) const override;

	ndVector m_gravityDir;
	NewtonDrawDebugLine m_onDebugDisplay;
};

#endif