/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __ND_JOINT_WORMGEAR_H__
#define __ND_JOINT_WORMGEAR_H__

#include "ndNewtonStdafx.h"
#include "ndJointRelational.h"

D_MSV_NEWTON_CLASS_ALIGN_32
class ndJointWormGear: public ndJointRelational
{
	public:
	D_CLASS_REFLECTION(ndJointWormGear, ndJointRelational)

	D_NEWTON_API ndJointWormGear();
	D_NEWTON_API ndJointWormGear(ndFloat32 gearRatio,
		const ndVector& body0Pin, ndBodyKinematic* const body0,
		const ndVector& body1Pin, ndBodyKinematic* const body1);

	protected:
	D_NEWTON_API void UpdateParameters() override;
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc) override;

} D_GCC_NEWTON_CLASS_ALIGN_32;


#endif 

