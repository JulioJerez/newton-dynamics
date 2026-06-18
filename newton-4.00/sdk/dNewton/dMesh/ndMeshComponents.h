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

#ifndef __ND_MESH_COMPONENTS_H__
#define __ND_MESH_COMPONENTS_H__

#include "ndNewtonStdafx.h"

class ndBody;
class ndMeshEffect;
class ndShapeInstance;
class ndWheelDescriptor;
class ndCloseLoopConstraints;
class ndJointBilateralConstraint;

class ndMeshBodyDynamic : public ndMeshBodyKinematic
{
	public:
	D_NEWTON_API ndMeshBodyDynamic(const ndMesh* const owner);
	D_NEWTON_API ndMeshBodyDynamic(const ndMeshBodyDynamic& other);

	D_NEWTON_API virtual ndMeshBody* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshBody& other) const override;

	D_NEWTON_API virtual ndBody* CreateObject() const override;
	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndVector m_intrinsicDamping;
};

class ndMeshLoopJoint : public ndClassAlloc
{
	public:
	D_NEWTON_API ndMeshLoopJoint(const ndMeshLoopJoint& other);
	D_NEWTON_API ndMeshLoopJoint(const ndCloseLoopConstraints* const owner);
	D_NEWTON_API ndMeshLoopJoint(const ndCloseLoopConstraints* const owner, const ndSharedPtr<ndMeshJoint>& joint, ndMesh* const childReference, ndMesh* const parentReference);
	D_NEWTON_API virtual ~ndMeshLoopJoint();
	
	D_NEWTON_API void UpdateName();
	D_NEWTON_API bool operator==(const ndMeshLoopJoint& other) const;

	//D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const;
	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent);

	ndString m_name;
	ndWeakPtr<ndMesh> m_childNode;
	ndWeakPtr<ndMesh> m_parentNode;
	ndSharedPtr<ndMeshJoint> m_joint;
	ndWeakPtr<const ndCloseLoopConstraints> m_owner;
};

class ndMeshJointFix6dof : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointFix6dof(const ndMesh* const owner);
	D_NEWTON_API ndMeshJointFix6dof(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);

	D_NEWTON_API ndMeshJointFix6dof(const ndMeshJointFix6dof& other);
	D_NEWTON_API virtual ndMeshJoint* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshJoint& other) const override;

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	ndFloat32 m_softness;
	ndFloat32 m_maxForce;
	ndFloat32 m_maxTorque;
};

class ndMeshJointDoubleHinge : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointDoubleHinge(const ndMesh* const owner);
	D_NEWTON_API ndMeshJointDoubleHinge(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);

	D_NEWTON_API ndMeshJointDoubleHinge(const ndMeshJointDoubleHinge& other);
	D_NEWTON_API virtual ndMeshJoint* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshJoint& other) const override;

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	ndAxis m_axis0;
	ndAxis m_axis1;
};

class ndMeshJointRoller : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointRoller(const ndMesh* const owner);
	D_NEWTON_API ndMeshJointRoller(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);

	D_NEWTON_API ndMeshJointRoller(const ndMeshJointRoller& other);
	D_NEWTON_API virtual ndMeshJoint* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshJoint& other) const override;

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	ndAxis m_linearAxis;
	ndAxis m_angularAxis;
};

class ndMeshJointCylinder : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointCylinder(const ndMesh* const owner);
	D_NEWTON_API ndMeshJointCylinder(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);

	D_NEWTON_API ndMeshJointCylinder(const ndMeshJointCylinder& other);
	D_NEWTON_API virtual ndMeshJoint* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshJoint& other) const override;

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	ndAxis m_linearAxis;
	ndAxis m_angularAxis;
};

class ndMeshJointPlane : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointPlane(const ndMesh* const owner);
	D_NEWTON_API ndMeshJointPlane(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);

	D_NEWTON_API ndMeshJointPlane(const ndMeshJointPlane& other);
	D_NEWTON_API virtual ndMeshJoint* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshJoint& other) const override;

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	bool m_controlRotation;
};

class ndMeshJointIkSwivelPositionEffector : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointIkSwivelPositionEffector(const ndMesh* const owner);
	D_NEWTON_API ndMeshJointIkSwivelPositionEffector(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);

	D_NEWTON_API ndMeshJointIkSwivelPositionEffector(const ndMeshJointIkSwivelPositionEffector& other);
	D_NEWTON_API virtual ndMeshJoint* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshJoint& other) const override;

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	ndVector m_restPosition;
	ndFloat32 m_angularSpring;
	ndFloat32 m_angularDamper;
	ndFloat32 m_angularMaxTorque;
	ndFloat32 m_angularRegularizer;

	ndFloat32 m_linearSpring;
	ndFloat32 m_linearDamper;
	ndFloat32 m_linearMaxForce;
	ndFloat32 m_linearRegularizer;

	ndFloat32 m_minWorkSpaceRadio;
	ndFloat32 m_maxWorkSpaceRadio;
	ndInt32 m_rotationOrder;
	bool m_enableSwivelControl;
};

class ndMeshJointGear : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointGear(const ndMesh* const owner);
	D_NEWTON_API ndMeshJointGear(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);

	D_NEWTON_API ndMeshJointGear(const ndMeshJointGear& other);
	D_NEWTON_API virtual ndMeshJoint* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshJoint& other) const override;

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	ndFloat32 m_ratio;
};

class ndMeshJointHinge : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointHinge(const ndMesh* const owner);
	D_NEWTON_API ndMeshJointHinge(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);

	D_NEWTON_API ndMeshJointHinge(const ndMeshJointHinge& other);
	D_NEWTON_API virtual ndMeshJoint* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshJoint& other) const override;

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	ndAxis m_axis;
};

class ndMeshJointSlider : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointSlider(const ndMesh* const owner);
	D_NEWTON_API ndMeshJointSlider(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);

	D_NEWTON_API ndMeshJointSlider(const ndMeshJointSlider& other);
	D_NEWTON_API virtual ndMeshJoint* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshJoint& other) const override;

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	ndAxis m_axis;
};

class ndMeshJointWheel : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointWheel(const ndMesh* const owner);
	D_NEWTON_API ndMeshJointWheel(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);

	D_NEWTON_API ndMeshJointWheel(const ndMeshJointWheel& other);
	D_NEWTON_API virtual ndMeshJoint* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshJoint& other) const override;

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	ndSharedPtr<ndWheelDescriptor> m_desc;
};

class ndMeshJointSpherical : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointSpherical(const ndMesh* const owner);
	D_NEWTON_API ndMeshJointSpherical(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);

	D_NEWTON_API ndMeshJointSpherical(const ndMeshJointSpherical& other);
	D_NEWTON_API virtual ndMeshJoint* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshJoint& other) const override;

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	ndAxis m_axis;
	ndFloat32 m_maxConeAngle;
	bool m_coneAngleState;
};

class ndMeshJointVehicleDifferential : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointVehicleDifferential(const ndMesh* const owner);
	D_NEWTON_API ndMeshJointVehicleDifferential(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);
	D_NEWTON_API ndMeshJointVehicleDifferential(const ndMeshJointVehicleDifferential& other);

	D_NEWTON_API virtual ndMeshJoint* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshJoint& other) const override;

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	ndReal m_limitedSlipOmega;
};

class ndMeshJointVehicleGearBox : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointVehicleGearBox(const ndMesh* const owner);
	D_NEWTON_API ndMeshJointVehicleGearBox(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);
	D_NEWTON_API ndMeshJointVehicleGearBox(const ndMeshJointVehicleGearBox& other);

	D_NEWTON_API virtual ndMeshJoint* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshJoint& other) const override;

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	ndReal m_idleOmega;
	ndReal m_clutchTorque;
	ndReal m_driveTrainResistanceTorque;
};

class ndMeshJointVehicleMotor : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointVehicleMotor(const ndMesh* const owner);
	D_NEWTON_API ndMeshJointVehicleMotor(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);
	D_NEWTON_API ndMeshJointVehicleMotor(const ndMeshJointVehicleMotor& other);

	D_NEWTON_API virtual ndMeshJoint* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshJoint& other) const override;

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	//ndReal m_omega;
	ndReal m_maxOmega;
	//ndReal m_omegaStep;
	//ndReal m_targetOmega;
	//ndReal m_engineTorque;
	//ndReal m_internalFriction;
};

class ndMeshJointVehicleDifferentialAxle : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointVehicleDifferentialAxle(const ndMesh* const owner);
	D_NEWTON_API ndMeshJointVehicleDifferentialAxle(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);

	D_NEWTON_API ndMeshJointVehicleDifferentialAxle(const ndMeshJointVehicleDifferentialAxle& other);
	D_NEWTON_API virtual ndMeshJoint* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshJoint& other) const override;

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	ndReal m_gearRatio;
};

class ndMeshJointVehicleTireJoint : public ndMeshJointWheel
{
	public:
	enum ndFrictionModel
	{
		m_coulomb,
		m_pacejkaSport,
		m_pacejkaTruck,
		m_pacejkaUtility,
		m_pacejkaCustom,
		m_coulombCicleOfFriction,
	};

	D_NEWTON_API ndMeshJointVehicleTireJoint(const ndMesh* const owner);
	D_NEWTON_API ndMeshJointVehicleTireJoint(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);
	D_NEWTON_API ndMeshJointVehicleTireJoint(const ndMeshJointVehicleTireJoint& other);

	D_NEWTON_API virtual ndMeshJoint* Duplicate() const override;
	D_NEWTON_API virtual bool operator==(const ndMeshJoint& other) const override;

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	ndFrictionModel m_frictionModel;
};
#endif

