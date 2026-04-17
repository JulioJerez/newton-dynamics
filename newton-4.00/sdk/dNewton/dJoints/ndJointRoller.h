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

#ifndef __ND_JOINT_ROLLER_H__
#define __ND_JOINT_ROLLER_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

D_MSV_NEWTON_CLASS_ALIGN_32
class ndJointRoller: public ndJointBilateralConstraint
{
	public:
	D_CLASS_REFLECTION(ndJointRoller, ndJointBilateralConstraint)

	D_NEWTON_API ndJointRoller();
	D_NEWTON_API ndJointRoller(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API ndJointRoller(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent);
	D_NEWTON_API virtual ~ndJointRoller();

	D_NEWTON_API ndFloat32 GetAngle() const;
	D_NEWTON_API ndFloat32 GetOmega() const;
	D_NEWTON_API ndFloat32 GetOffsetAngle() const;
	D_NEWTON_API void SetOffsetAngle(ndFloat32 angle);
	D_NEWTON_API bool GetLimitStateAngle() const;
	D_NEWTON_API void SetLimitStateAngle(bool state);
	D_NEWTON_API void SetLimitsAngle(ndFloat32 minLimit, ndFloat32 maxLimit);
	D_NEWTON_API void GetLimitsAngle(ndFloat32& minLimit, ndFloat32& maxLimit) const;
	D_NEWTON_API void SetAsSpringDamperAngle(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);
	D_NEWTON_API void GetSpringDamperAngle(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const;

	D_NEWTON_API ndFloat32 GetPosit() const;
	D_NEWTON_API ndFloat32 GetTargetPosit() const;
	D_NEWTON_API void SetTargetPosit(ndFloat32 offset);
	D_NEWTON_API bool GetLimitStatePosit() const;
	D_NEWTON_API void SetLimitStatePosit(bool state);
	D_NEWTON_API void SetLimitsPosit(ndFloat32 minLimit, ndFloat32 maxLimit);
	D_NEWTON_API void GetLimitsPosit(ndFloat32& minLimit, ndFloat32& maxLimit) const;
	D_NEWTON_API void SetAsSpringDamperPosit(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);
	D_NEWTON_API void GetSpringDamperPosit(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const;
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& debugCallback) const override;

	D_NEWTON_API void ClearMemory() override;

	protected:
	D_NEWTON_API void UpdateParameters() override;
	D_NEWTON_API ndFloat32 PenetrationOmega(ndFloat32 penetartion) const;
	D_NEWTON_API void SubmitLimitsAngle(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	D_NEWTON_API void SubmitSpringDamperAngle(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	D_NEWTON_API ndFloat32 PenetrationSpeed(ndFloat32 penetration) const;
	D_NEWTON_API void SubmitLimitsPosit(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);
	D_NEWTON_API void SubmitSpringDamperPosit(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc) override;
	D_NEWTON_API void ApplyBaseRows(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1);

	D_NEWTON_API virtual ndSharedPtr<ndMeshJoint> GetMeshJoint(const ndMesh* const owner) const override;

	ndAxisParam m_rotationAxis;
	ndAxisParam m_positionAxis;
} D_GCC_NEWTON_CLASS_ALIGN_32;


#endif 

