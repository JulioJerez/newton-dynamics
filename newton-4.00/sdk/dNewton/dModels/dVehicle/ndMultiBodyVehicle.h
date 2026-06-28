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
#include "ndMultiBodyVehicleMotor.h"
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

	enum DebugFlags
	{
		m_wheel = 1<<0,
		m_torsionBar = 1 << 1,
	};

	class ndComponentNotify;

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
	
	D_NEWTON_API DebugFlags GetDebugFlags() const;
	D_NEWTON_API void SetDebugFlags(DebugFlags flags);
	
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
	D_NEWTON_API void AddTorsionBar(const ndSharedPtr<ndJointBilateralConstraint>& torsionBar);
	D_NEWTON_API void AddDifferentialAxle(const ndSharedPtr<ndJointBilateralConstraint>& differentialAxleJoint);

	D_NEWTON_API ndMultiBodyVehicleTireJoint* AddTire(const ndWheelDescriptor& desc, const ndSharedPtr<ndBody>& tire);
	//D_NEWTON_API ndMultiBodyVehicleTireJoint* AddAxleTire(const ndMultiBodyVehicleTireJointInfo& desc, const ndSharedPtr<ndBody>& tire, const ndSharedPtr<ndBody>& axleBody);
	D_NEWTON_API ndMultiBodyVehicleDifferential* AddDifferential(ndFloat32 mass, ndFloat32 radius, ndMultiBodyVehicleTireJoint* const leftTire, ndMultiBodyVehicleTireJoint* const rightTire, ndFloat32 slipOmegaLock);
	D_NEWTON_API ndMultiBodyVehicleDifferential* AddDifferential(ndFloat32 mass, ndFloat32 radius, ndMultiBodyVehicleDifferential* const leftDifferential, ndMultiBodyVehicleDifferential* const rightDifferential, ndFloat32 slipOmegaLock);

	D_NEWTON_API virtual void Update(ndFloat32 timestep);
	D_NEWTON_API virtual void PostUpdate(ndFloat32 timestep);
	D_NEWTON_API virtual void Debug(ndConstraintDebugCallback& context) const;

	private:
	void ApplyTireModel();
	void ApplyStabilityControl();
	void CalculateRestSprungWeight();
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
	ndList<ndMultiBodyVehicleTireJoint*> m_tireList;
	ndList<ndMultiBodyVehicleDifferential*> m_differentialList;

	ndDownForce m_downForce;
	ndVehicleDectriptor m_descriptor;
	ndFloat32 m_steeringRate;
	ndFloat32 m_maxSideslipRate;
	ndFloat32 m_maxSideslipAngle;
	ndFloat32 m_gravityMagnitud;

	DebugFlags m_debugFlags;
	bool m_initialized;

	friend class ndMultiBodyVehicleMotor;
	friend class ndMultiBodyVehicleTireJoint;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif