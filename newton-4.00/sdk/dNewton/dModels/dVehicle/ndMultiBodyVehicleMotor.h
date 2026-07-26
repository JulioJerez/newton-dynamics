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

#ifndef __ND_MULTIBODY_VEHICLE_MOTOR_H__
#define __ND_MULTIBODY_VEHICLE_MOTOR_H__

#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"

class ndMultiBodyVehicle;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndMultiBodyVehicleMotor: public ndJointBilateralConstraint
{
	public:
	class ndEngineTorqueCurve
	{
		public:
		enum rpmName
		{
			m_engineOff,
			m_engineIdle,
			m_enginePickTorque,
			m_enginePickPower,
			m_engineRedLine,
			m_engineRMPSize
		};

		D_NEWTON_API ndEngineTorqueCurve();
		D_NEWTON_API void Init(ndFloat32 idleTorquePoundFoot, ndFloat32 idleRmp,
			ndFloat32 horsePower, ndFloat32 rpm0, ndFloat32 rpm1,
			ndFloat32 horsePowerAtRedLine, ndFloat32 redLineRpm);

		D_NEWTON_API ndFloat32 GetIdleRpm() const;
		D_NEWTON_API ndFloat32 GetRedLineRpm() const;
		D_NEWTON_API ndFloat32 GetPickPowerRpm() const;
		D_NEWTON_API ndFloat32 GetPickTorqueRpm() const;

		D_NEWTON_API ndFloat32 GetLowGearShiftRpm() const;
		D_NEWTON_API ndFloat32 GetHighGearShiftRpm() const;
		D_NEWTON_API ndFloat32 GetTorque(ndFloat32 rpm) const;
		D_NEWTON_API void SetOmegaAccel(ndFloat32 rpmStep);

		D_NEWTON_API bool operator==(const ndEngineTorqueCurve& other) const;

		ndFixSizeArray<ndReal, m_engineRMPSize> m_rpms;
		ndFixSizeArray<ndReal, m_engineRMPSize> m_torques;
		ndReal m_omegaStep;
		ndReal m_frictionLoss;
	};

	D_CLASS_REFLECTION(ndMultiBodyVehicleMotor, ndJointBilateralConstraint)

	D_NEWTON_API ndMultiBodyVehicleMotor();
	D_NEWTON_API ndMultiBodyVehicleMotor(ndBodyKinematic* const motor, ndBodyKinematic* const chassis);
	D_NEWTON_API ndMultiBodyVehicleMotor(ndBodyKinematic* const motor, ndMultiBodyVehicle* const vehicelModel);

	D_NEWTON_API void SetVehicleOwner(ndMultiBodyVehicle* const vehicle);

	D_NEWTON_API ndFloat32 GetRpm() const;
	D_NEWTON_API ndFloat32 GetMaxRpm() const;
	D_NEWTON_API ndFloat32 GetTopSpeed() const;
	D_NEWTON_API void SetTopSpeed(ndFloat32 topSpeed);

	D_NEWTON_API const ndEngineTorqueCurve& GetCurve() const;
	D_NEWTON_API void SetCurve(const ndEngineTorqueCurve& curve);

	D_NEWTON_API virtual void SetTorqueAndRpm(ndFloat32 newtonMeters, ndFloat32 rpm);

	private:
	void AlignMatrix();
	void UpdateParameters() override;
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc) override;
	D_NEWTON_API ndFloat32 CalculateAcceleration(ndConstraintDescritor& desc);
	D_NEWTON_API void DebugJoint(ndConstraintDebugCallback& callback) const override;
	D_NEWTON_API virtual ndSharedPtr<ndMeshJoint> GetMeshJoint(const ndMesh* const owner) const override;

	protected:
	ndWeakPtr<ndMultiBodyVehicle> m_vehicle;
	ndEngineTorqueCurve m_engineCurve;
	ndFloat32 m_omega;
	ndFloat32 m_topSpeed;
	ndFloat32 m_targetOmega;
	ndFloat32 m_engineTorque;

	friend class ndMultiBodyVehicle;
	friend class ndMultiBodyVehicleGearBox;
} D_GCC_NEWTON_CLASS_ALIGN_32;
#endif