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

#ifndef __ND_JOINT_RELATIONAL_H__
#define __ND_JOINT_RELATIONAL_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

D_MSV_NEWTON_CLASS_ALIGN_32
class ndJointRelational: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointRelational, ndJointBilateralConstraint)

	D_NEWTON_API ndJointRelational();
	D_NEWTON_API ndJointRelational(ndFloat32 gearRatio,
		const ndVector& parentPin, ndBodyKinematic* const parent,
		const ndVector& childPin, ndBodyKinematic* const child);
	D_NEWTON_API virtual ~ndJointRelational();

	D_NEWTON_API ndFloat32 GetRatio() const;
	D_NEWTON_API void SetRatio(ndFloat32 ratio);

	D_NEWTON_API ndFloat32 GetRegularizer() const;
	D_NEWTON_API void SetRegularizer(ndFloat32 regularizer);

	protected:
	D_NEWTON_API void UpdateParameters() override;
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc) override;

	//ndFloat32 m_angle;
	//ndFloat32 m_omega;
	ndFloat32 m_gearRatio;
	ndFloat32 m_regularizer;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif 

