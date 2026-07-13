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

#ifndef __ND_CONVEX_CAST_VEHICLE_H__
#define __ND_CONVEX_CAST_VEHICLE_H__

#include "ndNewtonStdafx.h"
#include "ndIkSolver.h"
#include "ndMultiBodyVehicle.h"

class ndSkeletonContainer;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndConvexCastVehicle : public ndMultiBodyVehicle
{
	public:
	D_CLASS_REFLECTION(ndConvexCastVehicle, ndMultiBodyVehicle )
	D_NEWTON_API ndConvexCastVehicle(ndFloat32 gravityMagnitud = ndFloat32 (10.0f));

	D_NEWTON_API virtual void ConvertToMotorVehicle() override;
	D_NEWTON_API virtual void Update(ndFloat32 timestep, ndInt32 threadId) override;
	D_NEWTON_API virtual void PostUpdate(ndFloat32 timestep, ndInt32 threadId) override;
	D_NEWTON_API virtual void TransformUpdate(ndFloat32 timestep) override;

	private:
	virtual void OnAddToWorld() override;
	virtual void OnRemoveFromWorld() override;
	void CalculateContacts(ndFixSizeArray<ndConstraint*, 32>& contacts, ndInt32 threadId);

	ndIkSolver m_solver;
	ndFixSizeArray<ndBodyDynamic*, 32> m_savedBody;
	ndFixSizeArray<ndJacobian, 32> m_savedForceTorque;
	ndSharedPtr<ndSkeletonContainer> m_skeleton;
	ndSharedPtr<ndJointBilateralConstraint> m_castGearBox;
	ndList<ndSharedPtr<ndJointBilateralConstraint>> m_castDifferentialAxelList;
	ndInt32 m_sleepCounter;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif