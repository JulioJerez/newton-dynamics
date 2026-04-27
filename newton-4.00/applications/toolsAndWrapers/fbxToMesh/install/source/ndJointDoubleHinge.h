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

#ifndef __ND_JOINT_DOUBLE_HINGE_H__
#define __ND_JOINT_DOUBLE_HINGE_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"


D_MSV_NEWTON_CLASS_ALIGN_32
class ndJointDoubleHinge: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointDoubleHinge, ndJointBilateralConstraint)

	D_NEWTON_API ndJointDoubleHinge();
	D_NEWTON_API ndJointDoubleHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API ndJointDoubleHinge(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointDoubleHinge();

	D_NEWTON_API ndFloat32 GetAngle0() const;
	D_NEWTON_API ndFloat32 GetOmega0() const;
	D_NEWTON_API ndFloat32 GetTargetAngle0() const;
	D_NEWTON_API void SetTargetAngle0(ndFloat32 angle);
	D_NEWTON_API bool GetLimitState0() const;
	D_NEWTON_API void SetLimitState0(bool state);
	D_NEWTON_API void SetLimits0(ndFloat32 minLimit, ndFloat32 maxLimit);
	D_NEWTON_API void GetLimits0(ndFloat32& minLimit, ndFloat32& maxLimit) const;
	D_NEWTON_API void SetAsSpringDamper0(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);
	D_NEWTON_API void GetSpringDamper0(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const;

	D_NEWTON_API ndFloat32 GetAngle1() const;
	D_NEWTON_API ndFloat32 GetOmega1() const;
	D_NEWTON_API ndFloat32 GetTargetAngle1() const;
	D_NEWTON_API void SetTargetAngle1(ndFloat32 angle);
	D_NEWTON_API bool GetLimitState1() const;
	D_NEWTON_API void SetLimitState1(bool state);
	D_NEWTON_API void SetLimits1(ndFloat32 minLimit, ndFloat32 maxLimit);
	D_NEWTON_API void GetLimits1(ndFloat32& minLimit, ndFloat32& maxLimit) const;
	D_NEWTON_API void SetAsSpringDamper1(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);
	D_NEWTON_API void GetSpringDamper1(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const;
	D_NEWTON_API void ClearMemory() override;

	protected:
	D_NEWTON_API void UpdateParameters() override;
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc) override;
	D_NEWTON_API ndFloat32 PenetrationOmega(ndFloat32 penetartion) const;
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const override;
	D_NEWTON_API void ApplyBaseRows(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	D_NEWTON_API void SubmitLimits(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	D_NEWTON_API void SubmitSpringDamper0(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	D_NEWTON_API void SubmitSpringDamper1(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	D_NEWTON_API virtual ndSharedPtr<ndMeshJoint> GetMeshJoint(const ndMesh* const owner) const override;

	ndAxisParam m_axis0;
	ndAxisParam m_axis1;
} D_GCC_NEWTON_CLASS_ALIGN_32;


#endif 

