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

#ifndef __ND_MULTIBODY_VEHICLE_GEAR_BOX_H__
#define __ND_MULTIBODY_VEHICLE_GEAR_BOX_H__

#include "ndNewtonStdafx.h"
#include "ndJointGear.h"

class ndMultiBodyVehicle;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndMultiBodyVehicleGearBox : public ndJointGear
{
	public: 
	class ndGearBox
	{
		public:
		enum Gear
		{
			m_revertGear,
			m_neutralGear,
			m_firstGear,
		};

		D_NEWTON_API ndGearBox();
		D_NEWTON_API bool operator==(const ndGearBox& other) const;

		ndFixSizeArray<ndReal, 8> m_gearRatios;
		ndReal m_crownGearRatio;
		ndReal m_idleClutchTorque;
		ndReal m_lockedClutchTorque;
		ndReal m_torqueConverter;
		ndInt32 m_gearShiftDelayTicks;
		bool m_manual;
	};

	D_CLASS_REFLECTION(ndMultiBodyVehicleGearBox, ndJointGear)

	D_NEWTON_API ndMultiBodyVehicleGearBox();
	D_NEWTON_API ndMultiBodyVehicleGearBox(ndFloat32 gearRatio,
		const ndVector& motorPin, ndBodyKinematic* const motor,
		const ndVector& differentialPin, ndBodyKinematic* const differential);

	D_NEWTON_API ndGearBox& GetGearBox();
	D_NEWTON_API const ndGearBox& GetGearBox() const;
	D_NEWTON_API void SetGearBox(const ndGearBox& gearBox);

	D_NEWTON_API void SetIdleOmega(ndFloat32 rpm);
	D_NEWTON_API void SetClutchTorque(ndFloat32 torqueInNewtonMeters);
	D_NEWTON_API void SetInternalTorqueLoss(ndFloat32 torqueInNewtonMeters);

	D_NEWTON_API ndFloat32 GetIdleOmega() const;
	D_NEWTON_API ndFloat32 GetClutchTorque() const;
	D_NEWTON_API ndFloat32 GetInternalTorqueLoss() const;
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback&) const override {}

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc) override;
	D_NEWTON_API virtual ndSharedPtr<ndMeshJoint> GetMeshJoint(const ndMesh* const owner) const override;

	ndGearBox m_gearBox;
	ndFloat32 m_idleOmega;
	ndFloat32 m_clutchTorque;
	ndFloat32 m_driveTrainResistanceTorque;

	friend class ndMultiBodyVehicle;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif