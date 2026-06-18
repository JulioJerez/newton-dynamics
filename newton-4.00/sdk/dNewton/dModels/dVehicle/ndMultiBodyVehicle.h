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

#ifndef __ND_MULTIBODY_VEHICLE_H__
#define __ND_MULTIBODY_VEHICLE_H__

#include "ndNewtonStdafx.h"
#include "ndModel.h"
#include "ndIkSolver.h"
#include "ndJointWheel.h"
#include "ndModelArticulation.h"
#include "ndMultiBodyVehicleTireJoint.h"

class ndWorld;
class ndBodyDynamic;
class ndMultiBodyVehicleMotor;
class ndMultiBodyVehicleGearBox;
class ndMultiBodyVehicleTireJoint;
class ndMultiBodyVehicleTorsionBar;
class ndMultiBodyVehicleDifferential;
class ndMultiBodyVehicleDifferentialAxle;

class ndVehicleDectriptor: public ndClassAlloc
{
	public:
	class ndTorqueTap
	{
		public:
		ndTorqueTap() {}
		ndTorqueTap(ndFloat32 rpm, ndFloat32 torqueInPoundFeet)
			:m_radPerSeconds(rpm * ndFloat32 (0.105f))
			,m_torqueInNewtonMeters(torqueInPoundFeet * ndFloat32 (1.36f))
		{
		}
		ndFloat32 m_radPerSeconds;
		ndFloat32 m_torqueInNewtonMeters;
	};

	class ndEngineTorqueCurve
	{
		public:
		D_NEWTON_API ndEngineTorqueCurve();

		D_NEWTON_API void Init(ndFloat32 idleTorquePoundFoot, ndFloat32 idleRmp,
			ndFloat32 horsePower, ndFloat32 rpm0, ndFloat32 rpm1,
			ndFloat32 horsePowerAtRedLine, ndFloat32 redLineRpm);

		D_NEWTON_API ndFloat32 GetIdleRadPerSec() const;
		D_NEWTON_API ndFloat32 GetRedLineRadPerSec() const;
		D_NEWTON_API ndFloat32 GetLowGearShiftRadPerSec() const;
		D_NEWTON_API ndFloat32 GetHighGearShiftRadPerSec() const;
		D_NEWTON_API ndFloat32 GetTorque(ndFloat32 omegaInRadPerSeconds) const;

		ndTorqueTap m_torqueCurve[5];
	};

	class ndGearBox
	{
		public:
		ndGearBox()
			:m_gearsCount(4)
			,m_manual(false)
		{
			m_neutral = ndFloat32 (0.0f);
			m_reverseRatio = ndFloat32(-3.0f);
			m_crownGearRatio = ndFloat32(5.0f);

			m_forwardRatios[0] = ndFloat32 (3.0f);
			m_forwardRatios[1] = ndFloat32 (1.5f);
			m_forwardRatios[2] = ndFloat32 (1.1f);
			m_forwardRatios[3] = ndFloat32 (0.8f);

			m_torqueConverter = ndFloat32(2000.0f);
			m_idleClutchTorque = ndFloat32(200.0f);
			m_lockedClutchTorque = ndFloat32(1.0e6f);
			m_gearShiftDelayTicks = 300;
		}

		union
		{
			struct
			{
				ndFloat32 m_forwardRatios[5];
				ndFloat32 m_reverseRatio;
				ndFloat32 m_neutral;
			};
			ndFloat32 m_ratios[8];
		};

		ndFloat32 m_idleClutchTorque;
		ndFloat32 m_lockedClutchTorque;
		ndFloat32 m_crownGearRatio;
		ndFloat32 m_torqueConverter;
		ndInt32 m_gearsCount;
		ndInt32 m_gearShiftDelayTicks;
		bool m_manual;
	};

	enum ndDifferentialType
	{
		m_rearWheelDrive,
		m_frontWheelDrive,
		m_fourWheeldrive,
		m_eightWheeldrive,
	};

	enum ndTorsionBarType
	{
		m_noWheelAxle,
		m_rearWheelAxle,
		m_frontWheelAxle,
		m_fourWheelAxle,
	};

	D_NEWTON_API ndVehicleDectriptor();

	ndString m_name;
	ndFloat32 m_chassisAngularDrag;
	ndEngineTorqueCurve m_engine;
	ndGearBox m_transmission;
	//ndTireFrictionModel m_tireFrictionModel;
	//ndFloat32 m_motorMass;
	//ndFloat32 m_motorRadius;
	//ndFloat32 m_differentialMass;
	//ndFloat32 m_differentialRadius;

	ndFloat32 m_slipDifferentialRmpLock;
	ndDifferentialType m_differentialType;

	ndFloat32 m_torsionBarSpringK;
	ndFloat32 m_torsionBarDamperC;
	ndFloat32 m_torsionBarRegularizer;
	ndTorsionBarType m_torsionBarType;
};

D_MSV_NEWTON_CLASS_ALIGN_32
class ndMultiBodyVehicle : public ndModelArticulation
{
	public:
	class ndTireContactPair
	{
		public:
		ndContact* m_contact;
		ndMultiBodyVehicleTireJoint* m_tireJoint;
	};
	
	class ndDownForce
	{
		public:
		class ndSpeedForcePair
		{
			public:
			ndFloat32 m_speed;
			ndFloat32 m_forceFactor;
			ndFloat32 m_aerodynamicDownforceConstant;
		};
	
		ndDownForce();
		ndFloat32 GetDownforceFactor(ndFloat32 speed) const;
	
		private:
		ndFloat32 CalculateFactor(const ndSpeedForcePair* const entry) const;
	
		ndFloat32 m_suspensionStiffnessModifier;
		ndSpeedForcePair m_downForceTable[5];
		friend class ndMultiBodyVehicle;
		friend class ndMultiBodyVehicleTireJoint;
	};

	D_CLASS_REFLECTION(ndMultiBodyVehicle, ndModelArticulation)

	D_NEWTON_API ndMultiBodyVehicle(ndFloat32 gravityMagnitud = ndFloat32 (10.0f));
	D_NEWTON_API void ConvertToMotorVehicle(const ndVehicleDectriptor& vehicleDescritor);

	D_NEWTON_API ndVehicleDectriptor& GetDescriptor();

	D_NEWTON_API const ndMatrix& GetLocalFrame() const;
	D_NEWTON_API void SetLocalFrame(const ndMatrix& localframe);

	D_NEWTON_API ndMultiBodyVehicle* GetAsMultiBodyVehicle() override;

	D_NEWTON_API ndBodyDynamic* GetChassis() const;
	D_NEWTON_API ndMultiBodyVehicleMotor* GetMotor() const;
	D_NEWTON_API ndMultiBodyVehicleGearBox* GetGearBox() const;
	
	D_NEWTON_API const ndList<ndMultiBodyVehicleTireJoint*>& GetTireList() const;

	D_NEWTON_API bool IsSleeping() const;
	D_NEWTON_API ndFloat32 GetSpeed() const;
	D_NEWTON_API void AddChassis(const ndSharedPtr<ndBody>& chassis);
	D_NEWTON_API ndMultiBodyVehicleMotor* AddMotor(ndFloat32 mass, ndFloat32 radius);
	D_NEWTON_API ndShapeInstance CreateTireShape(ndFloat32 radius, ndFloat32 width) const;
	D_NEWTON_API ndMultiBodyVehicleGearBox* AddGearBox(ndMultiBodyVehicleDifferential* const differential);

	D_NEWTON_API void AddTire(const ndSharedPtr<ndBody>& tireBody, const ndSharedPtr<ndJointBilateralConstraint>& tireJoint);
	D_NEWTON_API void AddMotor(const ndSharedPtr<ndBody>& motorBody, const ndSharedPtr<ndJointBilateralConstraint>& motorJoint);
	D_NEWTON_API void AddDifferential(const ndSharedPtr<ndBody>& differentialBody, const ndSharedPtr<ndJointBilateralConstraint>& differentialJoint);

	D_NEWTON_API void AddGearBox(const ndSharedPtr<ndJointBilateralConstraint>& gearBoxJoint);
	D_NEWTON_API void AddDifferentialAxle(const ndSharedPtr<ndJointBilateralConstraint>& differentialAxleJoint);

	D_NEWTON_API ndMultiBodyVehicleTireJoint* AddTire(const ndWheelDescriptor& desc, const ndSharedPtr<ndBody>& tire);
	//D_NEWTON_API ndMultiBodyVehicleTireJoint* AddAxleTire(const ndMultiBodyVehicleTireJointInfo& desc, const ndSharedPtr<ndBody>& tire, const ndSharedPtr<ndBody>& axleBody);
	D_NEWTON_API ndMultiBodyVehicleDifferential* AddDifferential(ndFloat32 mass, ndFloat32 radius, ndMultiBodyVehicleTireJoint* const leftTire, ndMultiBodyVehicleTireJoint* const rightTire, ndFloat32 slipOmegaLock);
	D_NEWTON_API ndMultiBodyVehicleDifferential* AddDifferential(ndFloat32 mass, ndFloat32 radius, ndMultiBodyVehicleDifferential* const leftDifferential, ndMultiBodyVehicleDifferential* const rightDifferential, ndFloat32 slipOmegaLock);

	D_NEWTON_API virtual void Update(ndFloat32 timestep);
	D_NEWTON_API virtual void PostUpdate(ndFloat32 timestep);
	D_NEWTON_API virtual void Debug(ndConstraintDebugCallback& context) const;

#if 0
	D_NEWTON_API ndMultiBodyVehicleTorsionBar* AddTorsionBar(ndBodyKinematic* const sentinel);
	D_NEWTON_API ndMultiBodyVehicle* GetAsMultiBodyVehicle();
#endif

	private:
	void ApplyTireModel();
	void ApplyStabilityControl();
	void CalculateSprungWeight();
	void ApplyAlignmentAndBalancing();
	void ApplyAerodynamics(ndFloat32 timestep);
	void ApplyTireModel(ndFixSizeArray<ndTireContactPair, 128>& tireContacts);
	ndBodyKinematic* CreateInternalBodyPart(ndFloat32 mass, ndFloat32 radius) const;

	bool CoulombTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint) const;
	bool PacejkaTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint) const;
	bool CalculateNormalizedAlgniningTorque(ndMultiBodyVehicleTireJoint* const tire, ndFloat32 sideSlipTangent) const;
	bool CoulombFrictionCircleTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint) const;

	ndMatrix m_localFrame;
	ndWeakPtr<ndBodyDynamic> m_chassis;
	ndWeakPtr<ndMultiBodyVehicleMotor> m_motor;
	ndSharedPtr<ndShapeWheel> m_tireShape;
	ndWeakPtr<ndMultiBodyVehicleGearBox> m_gearBox;
	ndWeakPtr<ndMultiBodyVehicleTorsionBar> m_torsionBar;
	ndList<ndMultiBodyVehicleTireJoint*> m_tireList;
	ndList<ndMultiBodyVehicleDifferential*> m_differentialList;

	ndDownForce m_downForce;
	ndVehicleDectriptor m_descriptor;
	ndFloat32 m_steeringRate;
	ndFloat32 m_maxSideslipRate;
	ndFloat32 m_maxSideslipAngle;
	ndFloat32 m_gravityMagnitud;
	bool m_initialized;

	friend class ndMultiBodyVehicleMotor;
	friend class ndMultiBodyVehicleTireJoint;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif