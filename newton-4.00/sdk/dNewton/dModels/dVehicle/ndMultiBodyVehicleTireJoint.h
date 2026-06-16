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

#ifndef __ND_MULTIBODY_VEHICLE_TIRE_JOINT_H__
#define __ND_MULTIBODY_VEHICLE_TIRE_JOINT_H__

#include "ndNewtonStdafx.h"
#include "ndJointWheel.h"

D_MSV_NEWTON_CLASS_ALIGN_32
class ndMultiBodyVehicleTireJoint: public ndJointWheel
{
	public:
	D_CLASS_REFLECTION(ndMultiBodyVehicleTireJoint, ndJointWheel)

	D_NEWTON_API ndMultiBodyVehicleTireJoint();
	D_NEWTON_API ndMultiBodyVehicleTireJoint(const ndMultiBodyVehicleTireJoint& joint);
	D_NEWTON_API ndMultiBodyVehicleTireJoint(const ndJointWheel* const joint, ndMultiBodyVehicle* const owner);
	D_NEWTON_API ndMultiBodyVehicleTireJoint(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent, const ndWheelDescriptor& desc, ndMultiBodyVehicle* const owner);
	D_NEWTON_API virtual ~ndMultiBodyVehicleTireJoint();

	D_NEWTON_API ndFloat32 GetSideSlip() const;
	D_NEWTON_API ndFloat32 GetLongitudinalSlip() const;

	D_NEWTON_API const ndTireFrictionModel& GetFrictionModel() const;
	D_NEWTON_API void SetVehicleOwner(ndMultiBodyVehicle* const vehicle);

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc) override;

	ndWeakPtr<ndMultiBodyVehicle> m_vehicle;
	ndTireFrictionModel m_frictionModel;
	ndFloat32 m_lateralSlip;
	ndFloat32 m_longitudinalSlip;
	ndFloat32 m_normalizedAligningTorque;

	friend class ndMultiBodyVehicle;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif 

