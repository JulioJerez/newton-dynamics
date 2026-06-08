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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndMesh.h"
#include "ndJointGear.h"
#include "ndJointPlane.h"
#include "ndJointHinge.h"
#include "ndJointWheel.h"
#include "ndJointRoller.h"
#include "ndJointSlider.h"
#include "ndBodyDynamic.h"
#include "ndJointFix6dof.h"
#include "ndJointCylinder.h"
#include "ndJointSpherical.h"
#include "ndMeshComponents.h"
#include "ndJointDoubleHinge.h"
#include "ndIkSwivelPositionEffector.h"
#include "ndMultiBodyVehicleDifferential.h"
#include "ndMultiBodyVehicleDifferentialAxle.h"

ndMeshBodyDynamic::ndMeshBodyDynamic(const ndMesh* const owner)
	:ndMeshBodyKinematic(owner)
	,m_intrinsicDamping(ndVector::m_zero)
{
	m_classConstructor = ndString("ndBodyDynamic");
}

ndMeshBodyDynamic::ndMeshBodyDynamic(const ndMeshBodyDynamic& other)
	:ndMeshBodyKinematic(other)
	,m_intrinsicDamping(other.m_intrinsicDamping)
{
}

ndMeshBody* ndMeshBodyDynamic::Duplicate() const
{
	return new ndMeshBodyDynamic(*this);
}

bool ndMeshBodyDynamic::operator==(const ndMeshBody& other) const
{
	bool test = ndMeshBodyKinematic::operator==(other);

	const ndMeshBodyDynamic* const otherBody = (ndMeshBodyDynamic*)&other;

	ndVector diff(m_intrinsicDamping - otherBody->m_intrinsicDamping);
	test = test && diff.DotProduct(diff).GetScalar() < ndFloat32(1.0e-6f);
	return test;
}

void ndMeshBodyDynamic::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshBodyKinematic::SerializeToXml(parent);

	xmlSaveParam(parent, "intrinsicLinearDamping", m_intrinsicDamping.m_w);
	xmlSaveParam(parent, "intrinsicAngularDamping", m_intrinsicDamping);
}

void ndMeshBodyDynamic::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshBodyKinematic::DeserializeFromXml(parent);

	m_intrinsicDamping = xmlGetVector3(parent, "intrinsicAngularDamping");
	m_intrinsicDamping.m_w = xmlGetFloat(parent, "intrinsicLinearDamping");
}

ndBody* ndMeshBodyDynamic::CreateObject() const
{
	ndBodyDynamic* const body = new ndBodyDynamic();
	body->Deserialize(this);
	return body;
}

ndMeshLoopJoint::ndMeshLoopJoint(const ndCloseLoopConstraints* const owner)
	:ndClassAlloc()
	,m_owner(ndWeakPtr<const ndCloseLoopConstraints>(owner))
{
}

ndMeshLoopJoint::ndMeshLoopJoint(const ndMeshLoopJoint& other)
	:ndClassAlloc()
	,m_name(other.m_name)
	,m_childNode(other.m_childNode)
	,m_parentNode(other.m_parentNode)
	,m_joint(ndSharedPtr<ndMeshJoint>(other.m_joint->Duplicate()))
	,m_owner(other.m_owner)
{
}

ndMeshLoopJoint::ndMeshLoopJoint(
	const ndCloseLoopConstraints* const owner,
	const ndSharedPtr<ndMeshJoint>& joint, 
	ndMesh* const childReference, ndMesh* const parentReference)
	:ndClassAlloc()
	//,m_name(parentReference->GetName() + "_" + childReference->GetName())
	,m_name()
	,m_childNode(ndWeakPtr<ndMesh>(childReference))
	,m_parentNode(ndWeakPtr<ndMesh>(parentReference))
	,m_joint(joint)
	,m_owner(ndWeakPtr<const ndCloseLoopConstraints>(owner))
{
	UpdateName();
}

ndMeshLoopJoint::~ndMeshLoopJoint()
{
}

void ndMeshLoopJoint::UpdateName()
{
	m_name = ndString (m_parentNode->GetName() + "_" + m_childNode->GetName());
}

bool ndMeshLoopJoint::operator==(const ndMeshLoopJoint& other) const
{
	bool test = (m_name == other.m_name);
	test = test && (m_childNode->GetName() == other.m_childNode->GetName());
	test = test && (m_parentNode->GetName() == other.m_parentNode->GetName());
	const ndMeshJoint* const self = *m_joint;
	const ndMeshJoint* const otherSelf = *other.m_joint;
	test = test && (*self == *otherSelf);
	return test;
}

void ndMeshLoopJoint::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "name", m_name.GetStr());
	xmlSaveParam(parent, "childReference", m_childNode->GetName().GetStr());
	xmlSaveParam(parent, "parentReference", m_parentNode->GetName().GetStr());

	nd::TiXmlElement* const loopJoint = new nd::TiXmlElement("joint");
	parent->LinkEndChild(loopJoint);
	m_joint->SerializeToXml(loopJoint);
}

void ndMeshLoopJoint::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	const ndString childName (xmlGetString(parent, "childReference"));
	const ndString parentName (xmlGetString(parent, "parentReference"));

	if (xmlHasAttribute(parent, "name"))
	{
		m_name = xmlGetString(parent, "name");
	}
	else
	{
		m_name = parentName + " " + childName;
	}

	m_childNode = ndWeakPtr<ndMesh>(m_owner->GetRoot()->FindByName(childName));
	m_parentNode = ndWeakPtr<ndMesh>(m_owner->GetRoot()->FindByName(parentName));

	const nd::TiXmlElement* const xmlJoint = (nd::TiXmlElement*)parent->FirstChild("joint");
	ndAssert(xmlJoint);
	m_joint = m_owner->LoadJoint(xmlJoint);
}

ndMeshJointFix6dof::ndMeshJointFix6dof(const ndMesh* const owner)
	:ndMeshJoint(owner)
	,m_softness(ndFloat32(0.0f))
	,m_maxForce(D_MAX_BOUND)
	,m_maxTorque(D_MAX_BOUND)
{
}

ndMeshJointFix6dof::ndMeshJointFix6dof(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
	const ndJointFix6dof* const fixJoint = (ndJointFix6dof*)joint;
	m_softness = fixJoint->GetRegularizer();
	m_maxForce = fixJoint->GetMaxForce();
	m_maxTorque = fixJoint->GetMaxTorque();
}

ndMeshJointFix6dof::ndMeshJointFix6dof(const ndMeshJointFix6dof& other)
	:ndMeshJoint(other)
	,m_softness(other.m_softness)
	,m_maxForce(other.m_maxForce)
	,m_maxTorque(other.m_maxTorque)
{
}

ndMeshJoint* ndMeshJointFix6dof::Duplicate() const
{
	return new ndMeshJointFix6dof(*this);
}

bool ndMeshJointFix6dof::operator==(const ndMeshJoint& other) const
{
	bool test = ndMeshJoint::operator==(other);

	if (test)
	{
		const ndMeshJointFix6dof* const otherJoint = (ndMeshJointFix6dof*)&other;
		test = test && (m_softness == otherJoint->m_softness);
		test = test && (m_maxForce == otherJoint->m_maxForce);
		test = test && (m_maxTorque == otherJoint->m_maxTorque);
	}
	return test;
}

void ndMeshJointFix6dof::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);

	xmlSaveParam(parent, "softness", m_softness);
	xmlSaveParam(parent, "maxForce", m_maxForce);
	xmlSaveParam(parent, "maxTorque", m_maxTorque);
}

void ndMeshJointFix6dof::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	m_softness = xmlGetFloat(parent, "softness");
	m_maxForce = xmlGetFloat(parent, "maxForce");
	m_maxTorque = xmlGetFloat(parent, "maxTorque");
}

ndJointBilateralConstraint* ndMeshJointFix6dof::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_localFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_localFrame1 * parent->GetMatrix());
	ndJointFix6dof* const joint = new ndJointFix6dof(child, parent, pinAndPivotInChild, pinAndPivotInParent);
	joint->SetMaxForce(m_maxForce);
	joint->SetMaxTorque(m_maxTorque);
	joint->SetRegularizer(m_softness);
	return joint;
}

ndMeshJointIkSwivelPositionEffector::ndMeshJointIkSwivelPositionEffector(const ndMesh* const owner)
	:ndMeshJoint(owner)
{
}

ndMeshJointIkSwivelPositionEffector::ndMeshJointIkSwivelPositionEffector(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
	const ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)joint;
	m_restPosition = effector->GetRestPosit();

	m_angularMaxTorque = effector->GetMaxTorque();
	effector->GetAngularSpringDamper(m_angularRegularizer, m_angularSpring, m_angularDamper);
	
	m_linearMaxForce = effector->GetMaxForce();
	effector->GetLinearSpringDamper(m_linearRegularizer, m_linearSpring, m_linearDamper);

	effector->GetWorkSpaceConstraints(m_minWorkSpaceRadio, m_maxWorkSpaceRadio);
	m_rotationOrder = effector->GetRotationOrder();
	m_enableSwivelControl = effector->GetSwivelMode();
}

ndMeshJointIkSwivelPositionEffector::ndMeshJointIkSwivelPositionEffector(const ndMeshJointIkSwivelPositionEffector& other)
	:ndMeshJoint(other)
	,m_restPosition(other.m_restPosition)
	,m_angularSpring(other.m_angularSpring)
	,m_angularDamper(other.m_angularDamper)
	,m_angularMaxTorque(other.m_angularMaxTorque)
	,m_angularRegularizer(other.m_angularRegularizer)
	,m_linearSpring(other.m_linearSpring)
	,m_linearDamper(other.m_linearDamper)
	,m_linearMaxForce(other.m_linearMaxForce)
	,m_linearRegularizer(other.m_linearRegularizer)
	,m_minWorkSpaceRadio(other.m_minWorkSpaceRadio)
	,m_maxWorkSpaceRadio(other.m_maxWorkSpaceRadio)
	,m_rotationOrder(other.m_rotationOrder)
	,m_enableSwivelControl(other.m_enableSwivelControl)
{
}

ndMeshJoint* ndMeshJointIkSwivelPositionEffector::Duplicate() const
{
	return new ndMeshJointIkSwivelPositionEffector(*this);
}

bool ndMeshJointIkSwivelPositionEffector::operator==(const ndMeshJoint& other) const
{
	bool test = ndMeshJoint::operator==(other);

	if (test)
	{
		const ndMeshJointIkSwivelPositionEffector* const otherJoint = (ndMeshJointIkSwivelPositionEffector*)&other;
		test = test && (m_angularSpring == otherJoint->m_angularSpring);
		test = test && (m_angularDamper == otherJoint->m_angularDamper);
		test = test && (m_angularMaxTorque == otherJoint->m_angularMaxTorque);
		test = test && (m_angularRegularizer == otherJoint->m_angularRegularizer);

		test = test && (m_linearSpring == otherJoint->m_linearSpring);
		test = test && (m_linearDamper == otherJoint->m_linearDamper);
		test = test && (m_linearMaxForce == otherJoint->m_linearMaxForce);
		test = test && (m_linearRegularizer == otherJoint->m_linearRegularizer);

		test = test && (m_rotationOrder == otherJoint->m_rotationOrder);
		test = test && (m_minWorkSpaceRadio == otherJoint->m_minWorkSpaceRadio);
		test = test && (m_maxWorkSpaceRadio == otherJoint->m_maxWorkSpaceRadio);
		test = test && (m_enableSwivelControl == otherJoint->m_enableSwivelControl);
	}

	return test;
}

void ndMeshJointIkSwivelPositionEffector::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);

	xmlSaveParam(parent, "restPosition", m_restPosition);
	xmlSaveParam(parent, "angularSpring", m_angularSpring);
	xmlSaveParam(parent, "angularDamper", m_angularDamper);
	xmlSaveParam(parent, "angularMaxTorque", m_angularMaxTorque);
	xmlSaveParam(parent, "angularRegularizer", m_angularRegularizer);

	xmlSaveParam(parent, "linearSpring", m_linearSpring);
	xmlSaveParam(parent, "linearDamper", m_linearDamper);
	xmlSaveParam(parent, "linearMaxForce", m_linearMaxForce);
	xmlSaveParam(parent, "linearRegularizer", m_linearRegularizer);
	xmlSaveParam(parent, "minWorkSpaceRadio", m_minWorkSpaceRadio);
	xmlSaveParam(parent, "maxWorkSpaceRadio", m_maxWorkSpaceRadio);

	xmlSaveParam(parent, "rotationOrder", m_rotationOrder);
	xmlSaveParam(parent, "enableSwivelControl", m_enableSwivelControl);
}

void ndMeshJointIkSwivelPositionEffector::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);

	m_restPosition = xmlGetVector3(parent, "restPosition");
	m_angularSpring = xmlGetFloat(parent, "angularSpring");
	m_angularDamper = xmlGetFloat(parent, "angularDamper");
	m_angularMaxTorque = xmlGetFloat(parent, "angularMaxTorque");
	m_angularRegularizer = xmlGetFloat(parent, "angularRegularizer");
	
	m_linearSpring = xmlGetFloat(parent, "linearSpring");
	m_linearDamper = xmlGetFloat(parent, "linearDamper");
	m_linearMaxForce = xmlGetFloat(parent, "linearMaxForce");
	m_linearRegularizer = xmlGetFloat(parent, "linearRegularizer");
	m_minWorkSpaceRadio = xmlGetFloat(parent, "minWorkSpaceRadio");
	m_maxWorkSpaceRadio = xmlGetFloat(parent, "maxWorkSpaceRadio");
	
	m_rotationOrder = xmlGetInt(parent, "rotationOrder");
	m_enableSwivelControl = xmlGetInt(parent, "enableSwivelControl") ? true: false;
}

ndJointBilateralConstraint* ndMeshJointIkSwivelPositionEffector::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_localFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_localFrame1 * parent->GetMatrix());
	ndIkSwivelPositionEffector* const joint = new ndIkSwivelPositionEffector(
		pinAndPivotInParent, parent, pinAndPivotInChild.m_posit, child);

	joint->SetMaxForce(m_linearMaxForce);
	joint->SetMaxTorque(m_angularMaxTorque);
	joint->SetSwivelMode(m_enableSwivelControl);
	joint->SetWorkSpaceConstraints(m_minWorkSpaceRadio, m_maxWorkSpaceRadio);
	joint->SetLinearSpringDamper(m_linearRegularizer, m_linearSpring, m_linearDamper);
	joint->SetAngularSpringDamper(m_angularRegularizer, m_angularSpring, m_angularDamper);

	return joint;
}

ndMeshJointPlane::ndMeshJointPlane(const ndMesh* const owner)
	:ndMeshJoint(owner)
{
}

ndMeshJointPlane::ndMeshJointPlane(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
	const ndJointPlane* const subJoint = (ndJointPlane*)joint;

	m_controlRotation = subJoint->GetEnableControlRotation();
}

ndMeshJointPlane::ndMeshJointPlane(const ndMeshJointPlane& other)
	:ndMeshJoint(other)
	,m_controlRotation(other.m_controlRotation)
{
}

ndMeshJoint* ndMeshJointPlane::Duplicate() const
{
	return new ndMeshJointPlane(*this);
}

bool ndMeshJointPlane::operator==(const ndMeshJoint& other) const
{
	bool test = ndMeshJoint::operator==(other);

	if (test)
	{
		const ndMeshJointPlane* const otherJoint = (ndMeshJointPlane*)&other;
		test = test && (m_controlRotation == otherJoint->m_controlRotation);
	}
	return test;
}

void ndMeshJointPlane::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);
	xmlSaveParam(parent, "controlRotation", m_controlRotation ? 1 : 0);
}

void ndMeshJointPlane::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);
	m_controlRotation = xmlGetInt(parent, "controlRotation") ? true : false;
}

ndJointBilateralConstraint* ndMeshJointPlane::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_localFrame0 * child->GetMatrix());
	ndJointPlane* const joint = new ndJointPlane(pinAndPivotInChild.m_posit, pinAndPivotInChild.m_front, child, parent);

	joint->EnableControlRotation(m_controlRotation ? true : false);
	return joint;
}

ndMeshJointGear::ndMeshJointGear(const ndMesh* const owner)
	:ndMeshJoint(owner)
{
}

ndMeshJointGear::ndMeshJointGear(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
	const ndJointGear* const subJoint = (ndJointGear*)joint;
	m_ratio = subJoint->GetRatio();
}

ndMeshJointGear::ndMeshJointGear(const ndMeshJointGear& other)
	:ndMeshJoint(other)
	,m_ratio (other.m_ratio)
{
}

ndMeshJoint* ndMeshJointGear::Duplicate() const
{
	return new ndMeshJointGear(*this);
}

bool ndMeshJointGear::operator==(const ndMeshJoint& other) const
{
	bool test = ndMeshJoint::operator==(other);

	if (test)
	{
		const ndMeshJointGear* const otherJoint = (ndMeshJointGear*)&other;
		test = test && (m_ratio == otherJoint->m_ratio);
	}
	return test;
}

void ndMeshJointGear::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);
	xmlSaveParam(parent, "ratio", m_ratio);
}

void ndMeshJointGear::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);
	m_ratio = xmlGetFloat(parent, "ratio");
}

ndJointBilateralConstraint* ndMeshJointGear::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_localFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_localFrame1 * parent->GetMatrix());
	ndJointGear* const joint = new ndJointGear(m_ratio, 
		pinAndPivotInChild.m_front, child, 
		pinAndPivotInParent.m_front, parent);
	return joint;
}

ndMeshJointDifferential::ndMeshJointDifferential(const ndMesh* const owner)
	:ndMeshJoint(owner)
{
}

ndMeshJointDifferential::ndMeshJointDifferential(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
	const ndMultiBodyVehicleDifferential* const subJoint = (ndMultiBodyVehicleDifferential*)joint;
	m_limitedSlipOmega = subJoint->GetSlipOmega();
}

ndMeshJointDifferential::ndMeshJointDifferential(const ndMeshJointDifferential& other)
	:ndMeshJoint(other)
	,m_limitedSlipOmega(other.m_limitedSlipOmega)
{
}

ndMeshJoint* ndMeshJointDifferential::Duplicate() const
{
	return new ndMeshJointDifferential(*this);
}

bool ndMeshJointDifferential::operator==(const ndMeshJoint& other) const
{
	bool test = ndMeshJoint::operator==(other);

	if (test)
	{
		const ndMeshJointDifferential* const otherJoint = (ndMeshJointDifferential*)&other;
		test = test && (m_limitedSlipOmega == otherJoint->m_limitedSlipOmega);
	}
	return test;
}

void ndMeshJointDifferential::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);
	xmlSaveParam(parent, "slipOmega", m_limitedSlipOmega);
}

void ndMeshJointDifferential::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);
	m_limitedSlipOmega = xmlGetFloat(parent, "slipOmega");
}

ndJointBilateralConstraint* ndMeshJointDifferential::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_localFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_localFrame1 * parent->GetMatrix());
	ndMultiBodyVehicleDifferential* const joint = new ndMultiBodyVehicleDifferential(child, parent, m_limitedSlipOmega);
	return joint;
}

ndMeshJointDifferentialAxle::ndMeshJointDifferentialAxle(const ndMesh* const owner)
	:ndMeshJoint(owner)
{
}

ndMeshJointDifferentialAxle::ndMeshJointDifferentialAxle(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
	const ndMultiBodyVehicleDifferentialAxle* const subJoint = (ndMultiBodyVehicleDifferentialAxle*)joint;
	m_gearRatio = subJoint->GetGearRatio();
}

ndMeshJointDifferentialAxle::ndMeshJointDifferentialAxle(const ndMeshJointDifferentialAxle& other)
	:ndMeshJoint(other)
	, m_gearRatio(other.m_gearRatio)
{
}

ndMeshJoint* ndMeshJointDifferentialAxle::Duplicate() const
{
	return new ndMeshJointDifferentialAxle(*this);
}

bool ndMeshJointDifferentialAxle::operator==(const ndMeshJoint& other) const
{
	bool test = ndMeshJoint::operator==(other);

	if (test)
	{
		const ndMeshJointDifferentialAxle* const otherJoint = (ndMeshJointDifferentialAxle*)&other;
		test = test && (m_gearRatio == otherJoint->m_gearRatio);
	}
	return test;
}

void ndMeshJointDifferentialAxle::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);
	xmlSaveParam(parent, "ratio", m_gearRatio);
}

void ndMeshJointDifferentialAxle::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);
	m_gearRatio = xmlGetFloat(parent, "ratio");
}

ndJointBilateralConstraint* ndMeshJointDifferentialAxle::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_localFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_localFrame1 * parent->GetMatrix());

	ndMultiBodyVehicleDifferentialAxle* const joint = new ndMultiBodyVehicleDifferentialAxle(
		pinAndPivotInParent.m_front, pinAndPivotInParent.m_up, parent,
		pinAndPivotInChild.m_front.Scale(m_gearRatio), child);

	return joint;
}


ndMeshJointSlider::ndMeshJointSlider(const ndMesh* const owner)
	:ndMeshJoint(owner)
{
}

ndMeshJointSlider::ndMeshJointSlider(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
	const ndJointSlider* const subJoint = (ndJointSlider*)joint;
	subJoint->GetSpringDamper(m_axis.m_springDamperRegularizer, m_axis.m_springK, m_axis.m_damperC);
	subJoint->GetLimits(m_axis.m_minLimit, m_axis.m_maxLimit);
	m_axis.m_limitState = subJoint->GetLimitState();
}

ndMeshJointSlider::ndMeshJointSlider(const ndMeshJointSlider& other)
	:ndMeshJoint(other)
	,m_axis(other.m_axis)
{
}

ndMeshJoint* ndMeshJointSlider::Duplicate() const
{
	return new ndMeshJointSlider(*this);
}

bool ndMeshJointSlider::operator == (const ndMeshJoint& other) const
{
	bool test = ndMeshJoint::operator==(other);

	if (test)
	{
		const ndMeshJointSlider* const otherJoint = (ndMeshJointSlider*)&other;
		test = test && (m_axis == otherJoint->m_axis);
	}
	return test;
}

void ndMeshJointSlider::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);
	xmlSaveParam(parent, "springK", m_axis.m_springK);
	xmlSaveParam(parent, "damperC", m_axis.m_damperC);
	xmlSaveParam(parent, "limitState", m_axis.m_limitState);
	xmlSaveParam(parent, "minLimit", m_axis.m_minLimit);
	xmlSaveParam(parent, "maxLimit", m_axis.m_maxLimit);
	xmlSaveParam(parent, "springDamperRegularizer", m_axis.m_springDamperRegularizer);
}

void ndMeshJointSlider::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);
	m_axis.m_springK = xmlGetFloat(parent, "springK");
	m_axis.m_damperC = xmlGetFloat(parent, "damperC");
	m_axis.m_minLimit = xmlGetFloat(parent, "minLimit");
	m_axis.m_maxLimit = xmlGetFloat(parent, "maxLimit");
	m_axis.m_springDamperRegularizer = xmlGetFloat(parent, "springDamperRegularizer");
	m_axis.m_limitState = xmlGetInt(parent, "limitState") ? true : false;
}

ndJointBilateralConstraint* ndMeshJointSlider::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_localFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_localFrame1 * parent->GetMatrix());
	ndJointSlider* const joint = new ndJointSlider(pinAndPivotInChild, pinAndPivotInParent, child, parent);
	joint->SetAsSpringDamper(m_axis.m_springDamperRegularizer, m_axis.m_springK, m_axis.m_damperC);
	joint->SetLimits(m_axis.m_minLimit, m_axis.m_maxLimit);
	joint->SetLimitState(m_axis.m_limitState ? true : false);
	return joint;
}

ndMeshJointHinge::ndMeshJointHinge(const ndMesh* const owner)
	:ndMeshJoint(owner)
{
}

ndMeshJointHinge::ndMeshJointHinge(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
	const ndJointHinge* const subJoint = (ndJointHinge*)joint;
	subJoint->GetSpringDamper(m_axis.m_springDamperRegularizer, m_axis.m_springK, m_axis.m_damperC);
	subJoint->GetLimits(m_axis.m_minLimit, m_axis.m_maxLimit);
	m_axis.m_limitState = subJoint->GetLimitState();
	m_axis.m_minLimit *= ndRadToDegree;
	m_axis.m_maxLimit *= ndRadToDegree;
}

ndMeshJointHinge::ndMeshJointHinge(const ndMeshJointHinge& other)
	:ndMeshJoint(other)
	,m_axis(other.m_axis)
{
}

ndMeshJoint* ndMeshJointHinge::Duplicate() const
{
	return new ndMeshJointHinge(*this);
}

bool ndMeshJointHinge::operator == (const ndMeshJoint& other) const
{
	bool test = ndMeshJoint::operator==(other);

	if (test)
	{
		const ndMeshJointHinge* const otherJoint = (ndMeshJointHinge*)&other;
		test = test && (m_axis == otherJoint->m_axis);
	}
	return test;
}

void ndMeshJointHinge::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);
	xmlSaveParam(parent, "springK", m_axis.m_springK);
	xmlSaveParam(parent, "damperC", m_axis.m_damperC);
	xmlSaveParam(parent, "limitState", m_axis.m_limitState);
	xmlSaveParam(parent, "minLimit", m_axis.m_minLimit);
	xmlSaveParam(parent, "maxLimit", m_axis.m_maxLimit);
	xmlSaveParam(parent, "springDamperRegularizer", m_axis.m_springDamperRegularizer);
}

void ndMeshJointHinge::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);
	m_axis.m_springK = xmlGetFloat(parent, "springK");
	m_axis.m_damperC = xmlGetFloat(parent, "damperC");
	m_axis.m_minLimit = xmlGetFloat(parent, "minLimit");
	m_axis.m_maxLimit = xmlGetFloat(parent, "maxLimit");
	m_axis.m_springDamperRegularizer = xmlGetFloat(parent, "springDamperRegularizer");
	m_axis.m_limitState = xmlGetInt(parent, "limitState") ? true : false;
}

ndJointBilateralConstraint* ndMeshJointHinge::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_localFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_localFrame1 * parent->GetMatrix());
	ndJointHinge* const joint = new ndJointHinge(pinAndPivotInChild, pinAndPivotInParent, child, parent);

	joint->SetAsSpringDamper(m_axis.m_springDamperRegularizer, m_axis.m_springK, m_axis.m_damperC);
	joint->SetLimits(m_axis.m_minLimit * ndDegreeToRad, m_axis.m_maxLimit * ndDegreeToRad);
	joint->SetLimitState(m_axis.m_limitState ? true : false);
	return joint;
}

ndMeshJointDoubleHinge::ndMeshJointDoubleHinge(const ndMesh* const owner)
	:ndMeshJoint(owner)
{
}

ndMeshJointDoubleHinge::ndMeshJointDoubleHinge(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
	const ndJointDoubleHinge* const subJoint = (ndJointDoubleHinge*)joint;

	subJoint->GetSpringDamper0(m_axis0.m_springDamperRegularizer, m_axis0.m_springK, m_axis0.m_damperC);
	subJoint->GetLimits0(m_axis0.m_minLimit, m_axis0.m_maxLimit);
	m_axis0.m_limitState = subJoint->GetLimitState0();
	m_axis0.m_minLimit *= ndRadToDegree;
	m_axis0.m_maxLimit *= ndRadToDegree;

	subJoint->GetSpringDamper1(m_axis1.m_springDamperRegularizer, m_axis1.m_springK, m_axis1.m_damperC);
	subJoint->GetLimits1(m_axis1.m_minLimit, m_axis1.m_maxLimit);
	m_axis1.m_limitState = subJoint->GetLimitState1();
	m_axis1.m_minLimit *= ndRadToDegree;
	m_axis1.m_maxLimit *= ndRadToDegree;
}

ndMeshJointDoubleHinge::ndMeshJointDoubleHinge(const ndMeshJointDoubleHinge& other)
	:ndMeshJoint(other)
	,m_axis0(other.m_axis0)
	,m_axis1(other.m_axis1)
{
}

ndMeshJoint* ndMeshJointDoubleHinge::Duplicate() const
{
	return new ndMeshJointDoubleHinge(*this);
}

bool ndMeshJointDoubleHinge::operator == (const ndMeshJoint& other) const
{
	bool test = ndMeshJoint::operator==(other);

	if (test)
	{
		const ndMeshJointDoubleHinge* const otherJoint = (ndMeshJointDoubleHinge*)&other;
		test = test && (m_axis0 == otherJoint->m_axis0);
		test = test && (m_axis1 == otherJoint->m_axis1);
	}
	return test;
}

void ndMeshJointDoubleHinge::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);

	nd::TiXmlElement* const axis0 = new nd::TiXmlElement("axis0");
	parent->LinkEndChild(axis0);
	xmlSaveParam(axis0, "springK", m_axis0.m_springK);
	xmlSaveParam(axis0, "damperC", m_axis0.m_damperC);
	xmlSaveParam(axis0, "limitState", m_axis0.m_limitState);
	xmlSaveParam(axis0, "minLimit", m_axis0.m_minLimit);
	xmlSaveParam(axis0, "maxLimit", m_axis0.m_maxLimit);
	xmlSaveParam(axis0, "springDamperRegularizer", m_axis0.m_springDamperRegularizer);

	nd::TiXmlElement* const axis1 = new nd::TiXmlElement("axis1");
	parent->LinkEndChild(axis1);
	xmlSaveParam(axis1, "springK", m_axis1.m_springK);
	xmlSaveParam(axis1, "damperC", m_axis1.m_damperC);
	xmlSaveParam(axis1, "limitState", m_axis1.m_limitState);
	xmlSaveParam(axis1, "minLimit", m_axis1.m_minLimit);
	xmlSaveParam(axis1, "maxLimit", m_axis1.m_maxLimit);
	xmlSaveParam(axis1, "springDamperRegularizer", m_axis1.m_springDamperRegularizer);
}

void ndMeshJointDoubleHinge::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);

	const nd::TiXmlElement* const xmlAxis0 = (nd::TiXmlElement*)parent->FirstChild("axis0");
	m_axis0.m_springK = xmlGetFloat(xmlAxis0, "springK");
	m_axis0.m_damperC = xmlGetFloat(xmlAxis0, "damperC");
	m_axis0.m_minLimit = xmlGetFloat(xmlAxis0, "minLimit");
	m_axis0.m_maxLimit = xmlGetFloat(xmlAxis0, "maxLimit");
	m_axis0.m_springDamperRegularizer = xmlGetFloat(xmlAxis0, "springDamperRegularizer");
	m_axis0.m_limitState = xmlGetInt(xmlAxis0, "limitState") ? true : false;

	const nd::TiXmlElement* const xmlAxis1 = (nd::TiXmlElement*)parent->FirstChild("axis1");
	m_axis1.m_springK = xmlGetFloat(xmlAxis1, "springK");
	m_axis1.m_damperC = xmlGetFloat(xmlAxis1, "damperC");
	m_axis1.m_minLimit = xmlGetFloat(xmlAxis1, "minLimit");
	m_axis1.m_maxLimit = xmlGetFloat(xmlAxis1, "maxLimit");
	m_axis1.m_springDamperRegularizer = xmlGetFloat(xmlAxis1, "springDamperRegularizer");
	m_axis1.m_limitState = xmlGetInt(xmlAxis1, "limitState") ? true : false;
}

ndJointBilateralConstraint* ndMeshJointDoubleHinge::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_localFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_localFrame1 * parent->GetMatrix());
	ndJointDoubleHinge* const joint = new ndJointDoubleHinge(pinAndPivotInChild, pinAndPivotInParent, child, parent);

	joint->SetLimitState0(m_axis0.m_limitState ? true : false);
	joint->SetLimits0(m_axis0.m_minLimit * ndDegreeToRad, m_axis0.m_maxLimit * ndDegreeToRad);
	joint->SetAsSpringDamper0(m_axis0.m_springDamperRegularizer, m_axis0.m_springK, m_axis0.m_damperC);

	joint->SetLimitState1(m_axis1.m_limitState ? true : false);
	joint->SetLimits1(m_axis1.m_minLimit * ndDegreeToRad, m_axis1.m_maxLimit * ndDegreeToRad);
	joint->SetAsSpringDamper1(m_axis1.m_springDamperRegularizer, m_axis1.m_springK, m_axis1.m_damperC);

	return joint;
}

ndMeshJointCylinder::ndMeshJointCylinder(const ndMesh* const owner)
	:ndMeshJoint(owner)
{
}

ndMeshJointCylinder::ndMeshJointCylinder(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
	const ndJointCylinder* const subJoint = (ndJointCylinder*)joint;

	subJoint->GetSpringDamperPosit(m_linearAxis.m_springDamperRegularizer, m_linearAxis.m_springK, m_linearAxis.m_damperC);
	subJoint->GetLimitsPosit(m_linearAxis.m_minLimit, m_linearAxis.m_maxLimit);
	m_linearAxis.m_limitState = subJoint->GetLimitStatePosit();

	subJoint->GetSpringDamperAngle(m_angularAxis.m_springDamperRegularizer, m_angularAxis.m_springK, m_angularAxis.m_damperC);
	subJoint->GetLimitsAngle(m_angularAxis.m_minLimit, m_angularAxis.m_maxLimit);
	m_angularAxis.m_limitState = subJoint->GetLimitStateAngle();
	m_angularAxis.m_minLimit *= ndRadToDegree;
	m_angularAxis.m_maxLimit *= ndRadToDegree;
}

ndMeshJointCylinder::ndMeshJointCylinder(const ndMeshJointCylinder& other)
	:ndMeshJoint(other)
	,m_linearAxis(other.m_linearAxis)
	,m_angularAxis(other.m_angularAxis)
{
}

ndMeshJoint* ndMeshJointCylinder::Duplicate() const
{
	return new ndMeshJointCylinder(*this);
}

bool ndMeshJointCylinder::operator == (const ndMeshJoint& other) const
{
	bool test = ndMeshJoint::operator==(other);

	if (test)
	{
		const ndMeshJointCylinder* const otherJoint = (ndMeshJointCylinder*)&other;
		test = test && (m_linearAxis == otherJoint->m_linearAxis);
		test = test && (m_angularAxis == otherJoint->m_angularAxis);
	}
	return test;
}

void ndMeshJointCylinder::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);

	nd::TiXmlElement* const axis0 = new nd::TiXmlElement("axis0");
	parent->LinkEndChild(axis0);
	xmlSaveParam(axis0, "linearSpringK", m_linearAxis.m_springK);
	xmlSaveParam(axis0, "linearDamperC", m_linearAxis.m_damperC);
	xmlSaveParam(axis0, "linearLimitState", m_linearAxis.m_limitState);
	xmlSaveParam(axis0, "linearMinLimit", m_linearAxis.m_minLimit);
	xmlSaveParam(axis0, "linearMaxLimit", m_linearAxis.m_maxLimit);
	xmlSaveParam(axis0, "linearSpringDamperRegularizer", m_linearAxis.m_springDamperRegularizer);

	nd::TiXmlElement* const axis1 = new nd::TiXmlElement("axis1");
	parent->LinkEndChild(axis1);
	xmlSaveParam(axis1, "angularSpringK", m_angularAxis.m_springK);
	xmlSaveParam(axis1, "angularDamperC", m_angularAxis.m_damperC);
	xmlSaveParam(axis1, "angularLimitState", m_angularAxis.m_limitState);
	xmlSaveParam(axis1, "angularMinLimit", m_angularAxis.m_minLimit);
	xmlSaveParam(axis1, "angularMaxLimit", m_angularAxis.m_maxLimit);
	xmlSaveParam(axis1, "angularSpringDamperRegularizer", m_angularAxis.m_springDamperRegularizer);
}

void ndMeshJointCylinder::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);

	const nd::TiXmlElement* const xmlAxis0 = (nd::TiXmlElement*)parent->FirstChild("axis0");
	m_linearAxis.m_springK = xmlGetFloat(xmlAxis0, "linearSpringK");
	m_linearAxis.m_damperC = xmlGetFloat(xmlAxis0, "linearDamperC");
	m_linearAxis.m_minLimit = xmlGetFloat(xmlAxis0, "linearMinLimit");
	m_linearAxis.m_maxLimit = xmlGetFloat(xmlAxis0, "linearMaxLimit");
	m_linearAxis.m_springDamperRegularizer = xmlGetFloat(xmlAxis0, "linearSpringDamperRegularizer");
	m_linearAxis.m_limitState = xmlGetInt(xmlAxis0, "linearLimitState") ? true : false;

	const nd::TiXmlElement* const xmlAxis1 = (nd::TiXmlElement*)parent->FirstChild("axis1");
	m_angularAxis.m_springK = xmlGetFloat(xmlAxis1, "angularSpringK");
	m_angularAxis.m_damperC = xmlGetFloat(xmlAxis1, "angularDamperC");
	m_angularAxis.m_minLimit = xmlGetFloat(xmlAxis1, "angularMinLimit");
	m_angularAxis.m_maxLimit = xmlGetFloat(xmlAxis1, "angularMaxLimit");
	m_angularAxis.m_springDamperRegularizer = xmlGetFloat(xmlAxis1, "angularSpringDamperRegularizer");
	m_angularAxis.m_limitState = xmlGetInt(xmlAxis1, "angularLimitState") ? true : false;
}

ndJointBilateralConstraint* ndMeshJointCylinder::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_localFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_localFrame1 * parent->GetMatrix());
	ndJointCylinder* const joint = new ndJointCylinder(pinAndPivotInChild, pinAndPivotInParent, child, parent);

	joint->SetLimitStateAngle(m_linearAxis.m_limitState ? true : false);
	joint->SetLimitsAngle(m_linearAxis.m_minLimit, m_linearAxis.m_maxLimit);
	joint->SetAsSpringDamperAngle(m_linearAxis.m_springDamperRegularizer, m_linearAxis.m_springK, m_linearAxis.m_damperC);

	joint->SetLimitStatePosit(m_angularAxis.m_limitState ? true : false);
	joint->SetLimitsPosit(m_angularAxis.m_minLimit * ndDegreeToRad, m_angularAxis.m_maxLimit * ndDegreeToRad);
	joint->SetAsSpringDamperPosit(m_angularAxis.m_springDamperRegularizer, m_angularAxis.m_springK, m_angularAxis.m_damperC);

	return joint;
}

ndMeshJointRoller::ndMeshJointRoller(const ndMesh* const owner)
	:ndMeshJoint(owner)
{
}

ndMeshJointRoller::ndMeshJointRoller(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
	const ndJointRoller* const subJoint = (ndJointRoller*)joint;

	subJoint->GetSpringDamperPosit(m_linearAxis.m_springDamperRegularizer, m_linearAxis.m_springK, m_linearAxis.m_damperC);
	subJoint->GetLimitsPosit(m_linearAxis.m_minLimit, m_linearAxis.m_maxLimit);
	m_linearAxis.m_limitState = subJoint->GetLimitStatePosit();

	subJoint->GetSpringDamperAngle(m_angularAxis.m_springDamperRegularizer, m_angularAxis.m_springK, m_angularAxis.m_damperC);
	subJoint->GetLimitsPosit(m_angularAxis.m_minLimit, m_angularAxis.m_maxLimit);
	m_angularAxis.m_limitState = subJoint->GetLimitStateAngle();
	m_angularAxis.m_minLimit *= ndRadToDegree;
	m_angularAxis.m_maxLimit *= ndRadToDegree;
}

ndMeshJointRoller::ndMeshJointRoller(const ndMeshJointRoller& other)
	:ndMeshJoint(other)
	,m_linearAxis(other.m_linearAxis)
	,m_angularAxis(other.m_angularAxis)
{
}

ndMeshJoint* ndMeshJointRoller::Duplicate() const
{
	return new ndMeshJointRoller(*this);
}

bool ndMeshJointRoller::operator == (const ndMeshJoint& other) const
{
	bool test = ndMeshJoint::operator==(other);

	if (test)
	{
		const ndMeshJointRoller* const otherJoint = (ndMeshJointRoller*)&other;
		test = test && (m_linearAxis == otherJoint->m_linearAxis);
		test = test && (m_angularAxis == otherJoint->m_angularAxis);
	}
	return test;
}

void ndMeshJointRoller::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);

	nd::TiXmlElement* const axis0 = new nd::TiXmlElement("positAxis");
	parent->LinkEndChild(axis0);
	xmlSaveParam(axis0, "linearSpringK", m_linearAxis.m_springK);
	xmlSaveParam(axis0, "linearDamperC", m_linearAxis.m_damperC);
	xmlSaveParam(axis0, "linearLimitState", m_linearAxis.m_limitState);
	xmlSaveParam(axis0, "linearMinLimit", m_linearAxis.m_minLimit);
	xmlSaveParam(axis0, "linearMaxLimit", m_linearAxis.m_maxLimit);
	xmlSaveParam(axis0, "linearSpringDamperRegularizer", m_linearAxis.m_springDamperRegularizer);

	nd::TiXmlElement* const axis1 = new nd::TiXmlElement("angleAxis");
	parent->LinkEndChild(axis1);
	xmlSaveParam(axis1, "angularSpringK", m_angularAxis.m_springK);
	xmlSaveParam(axis1, "angularDamperC", m_angularAxis.m_damperC);
	xmlSaveParam(axis1, "angularLimitState", m_angularAxis.m_limitState);
	xmlSaveParam(axis1, "angularMinLimit", m_angularAxis.m_minLimit);
	xmlSaveParam(axis1, "angularMaxLimit", m_angularAxis.m_maxLimit);
	xmlSaveParam(axis1, "angularSpringDamperRegularizer", m_angularAxis.m_springDamperRegularizer);
}

void ndMeshJointRoller::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);

	const nd::TiXmlElement* const xmlAxis0 = (nd::TiXmlElement*)parent->FirstChild("positAxis");
	m_linearAxis.m_springK = xmlGetFloat(xmlAxis0, "linearSpringK");
	m_linearAxis.m_damperC = xmlGetFloat(xmlAxis0, "linearDamperC");
	m_linearAxis.m_minLimit = xmlGetFloat(xmlAxis0, "linearMinLimit");
	m_linearAxis.m_maxLimit = xmlGetFloat(xmlAxis0, "linearMaxLimit");
	m_linearAxis.m_springDamperRegularizer = xmlGetFloat(xmlAxis0, "linearSpringDamperRegularizer");
	m_linearAxis.m_limitState = xmlGetInt(xmlAxis0, "linearLimitState") ? true : false;

	const nd::TiXmlElement* const xmlAxis1 = (nd::TiXmlElement*)parent->FirstChild("angleAxis");
	m_angularAxis.m_springK = xmlGetFloat(xmlAxis1, "angularSpringK");
	m_angularAxis.m_damperC = xmlGetFloat(xmlAxis1, "angularDamperC");
	m_angularAxis.m_minLimit = xmlGetFloat(xmlAxis1, "angularMinLimit");
	m_angularAxis.m_maxLimit = xmlGetFloat(xmlAxis1, "angularMaxLimit");
	m_angularAxis.m_springDamperRegularizer = xmlGetFloat(xmlAxis1, "angularSpringDamperRegularizer");
	m_angularAxis.m_limitState = xmlGetInt(xmlAxis1, "angularLimitState") ? true : false;
}

ndJointBilateralConstraint* ndMeshJointRoller::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_localFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_localFrame1 * parent->GetMatrix());
	ndJointRoller* const joint = new ndJointRoller(pinAndPivotInChild, pinAndPivotInParent, child, parent);

	joint->SetLimitStatePosit(m_linearAxis.m_limitState ? true : false);
	joint->SetLimitsPosit(m_linearAxis.m_minLimit, m_linearAxis.m_maxLimit);
	joint->SetAsSpringDamperPosit(m_linearAxis.m_springDamperRegularizer, m_linearAxis.m_springK, m_linearAxis.m_damperC);

	joint->SetLimitStateAngle(m_angularAxis.m_limitState ? true : false);
	joint->SetLimitsAngle(m_angularAxis.m_minLimit * ndDegreeToRad, m_angularAxis.m_maxLimit * ndDegreeToRad);
	joint->SetAsSpringDamperAngle(m_angularAxis.m_springDamperRegularizer, m_angularAxis.m_springK, m_angularAxis.m_damperC);
	return joint;
}

ndMeshJointWheel::ndMeshJointWheel(const ndMesh* const owner)
	:ndMeshJoint(owner)
	,m_desc(new ndWheelDescriptor)
{
}

ndMeshJointWheel::ndMeshJointWheel(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
	,m_desc(new ndWheelDescriptor)
{
	const ndJointWheel* const subJoint = (ndJointWheel*)joint;
	**m_desc = subJoint->GetInfo();
}

ndMeshJointWheel::ndMeshJointWheel(const ndMeshJointWheel& other)
	:ndMeshJoint(other)
	,m_desc(new ndWheelDescriptor(**other.m_desc))
{
}

ndMeshJoint* ndMeshJointWheel::Duplicate() const
{
	return new ndMeshJointWheel(*this);
}

bool ndMeshJointWheel::operator == (const ndMeshJoint& other) const
{
	bool test = ndMeshJoint::operator==(other);

	if (test)
	{
		const ndMeshJointWheel* const otherJoint = (ndMeshJointWheel*)&other;
		test = test && (m_desc->m_springK == otherJoint->m_desc->m_springK);
		test = test && (m_desc->m_damperC == otherJoint->m_desc->m_damperC);
		test = test && (m_desc->m_upperStop == otherJoint->m_desc->m_upperStop);
		test = test && (m_desc->m_lowerStop == otherJoint->m_desc->m_lowerStop);
		test = test && (m_desc->m_regularizer == otherJoint->m_desc->m_regularizer);
		test = test && (m_desc->m_brakeTorque == otherJoint->m_desc->m_brakeTorque);
		test = test && (m_desc->m_steeringAngle == otherJoint->m_desc->m_steeringAngle);
		test = test && (m_desc->m_handBrakeTorque == otherJoint->m_desc->m_handBrakeTorque);
	}
	return test;
}

void ndMeshJointWheel::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);

	xmlSaveParam(parent, "springK", m_desc->m_springK);
	xmlSaveParam(parent, "damperC", m_desc->m_damperC);
	xmlSaveParam(parent, "upperStop", m_desc->m_upperStop);
	xmlSaveParam(parent, "lowerStop", m_desc->m_lowerStop);
	xmlSaveParam(parent, "brakeTorque", m_desc->m_brakeTorque);
	xmlSaveParam(parent, "handBrakeTorque", m_desc->m_handBrakeTorque);
	xmlSaveParam(parent, "steeringAngle", m_desc->m_steeringAngle);
	xmlSaveParam(parent, "springDamperRegularizer", m_desc->m_regularizer);
}

void ndMeshJointWheel::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);

	m_desc->m_springK = xmlGetFloat(parent, "springK");
	m_desc->m_damperC = xmlGetFloat(parent, "damperC");
	m_desc->m_upperStop = xmlGetFloat(parent, "upperStop");
	m_desc->m_lowerStop = xmlGetFloat(parent, "lowerStop");
	m_desc->m_brakeTorque = xmlGetFloat(parent, "brakeTorque");
	m_desc->m_handBrakeTorque = xmlGetFloat(parent, "handBrakeTorque");
	m_desc->m_steeringAngle = xmlGetFloat(parent, "steeringAngle");
	m_desc->m_regularizer = xmlGetFloat(parent, "springDamperRegularizer");
}

ndJointBilateralConstraint* ndMeshJointWheel::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_localFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_localFrame1 * parent->GetMatrix());
	ndJointWheel* const joint = new ndJointWheel(pinAndPivotInChild, pinAndPivotInParent, child, parent, **m_desc);
	return joint;
}

ndMeshJointSpherical::ndMeshJointSpherical(const ndMesh* const owner)
	:ndMeshJoint(owner)
	,m_maxConeAngle(0.0f)
	,m_coneAngleState(false)
{
}

ndMeshJointSpherical::ndMeshJointSpherical(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
	const ndJointSpherical* const subJoint = (ndJointSpherical*)joint;
	subJoint->GetSpringDamper(m_axis.m_springDamperRegularizer, m_axis.m_springK, m_axis.m_damperC);
	subJoint->GetTwistLimits(m_axis.m_minLimit, m_axis.m_maxLimit);
	m_axis.m_limitState = subJoint->GetTwistLimitState();
	m_axis.m_minLimit *= ndRadToDegree;
	m_axis.m_maxLimit *= ndRadToDegree;

	m_coneAngleState = subJoint->GetConeLimitState();
	m_maxConeAngle = subJoint->GetConeLimit() * ndRadToDegree;
}

ndMeshJointSpherical::ndMeshJointSpherical(const ndMeshJointSpherical& other)
	:ndMeshJoint(other)
	,m_axis(other.m_axis)
	,m_maxConeAngle(other.m_maxConeAngle)
	,m_coneAngleState(other.m_coneAngleState)
{
}

ndMeshJoint* ndMeshJointSpherical::Duplicate() const
{
	return new ndMeshJointSpherical(*this);
}

bool ndMeshJointSpherical::operator == (const ndMeshJoint& other) const
{
	bool test = ndMeshJoint::operator==(other);

	if (test)
	{
		const ndMeshJointSpherical* const otherJoint = (ndMeshJointSpherical*)&other;
		test = test && (m_axis == otherJoint->m_axis);
		test = test && (m_maxConeAngle == otherJoint->m_maxConeAngle);
		test = test && (m_coneAngleState == otherJoint->m_coneAngleState);
	}
	return test;
}

void ndMeshJointSpherical::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);

	xmlSaveParam(parent, "springK", m_axis.m_springK);
	xmlSaveParam(parent, "damperC", m_axis.m_damperC);
	xmlSaveParam(parent, "maxConeAngle", m_maxConeAngle);
	xmlSaveParam(parent, "coneLimitState", m_coneAngleState);
	xmlSaveParam(parent, "minTwistAngle", m_axis.m_minLimit);
	xmlSaveParam(parent, "maxTwistAngle", m_axis.m_maxLimit);
	xmlSaveParam(parent, "twistLimitState", m_axis.m_limitState);
	xmlSaveParam(parent, "springDamperRegularizer", m_axis.m_springDamperRegularizer);
}

void ndMeshJointSpherical::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);

	m_axis.m_springK = xmlGetFloat(parent, "springK");
	m_axis.m_damperC = xmlGetFloat(parent, "damperC");
	m_maxConeAngle = xmlGetFloat(parent, "maxConeAngle");
	m_axis.m_minLimit = xmlGetFloat(parent, "minTwistAngle");
	m_axis.m_maxLimit = xmlGetFloat(parent, "maxTwistAngle");

	if (xmlHasAttribute(parent, "coneLimitState"))
	{
		m_coneAngleState = xmlGetInt(parent, "coneLimitState") ? true : false;
		m_axis.m_limitState = xmlGetInt(parent, "twistLimitState") ? true : false;
		m_axis.m_springDamperRegularizer = xmlGetFloat(parent, "springDamperRegularizer");
	}
}

ndJointBilateralConstraint* ndMeshJointSpherical::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_localFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_localFrame1 * parent->GetMatrix());
	ndJointSpherical* const joint = new ndJointSpherical(pinAndPivotInChild, pinAndPivotInParent, child, parent);
	joint->SetConeLimitState(m_coneAngleState ? true : false);
	joint->SetTwistLimitState(m_axis.m_limitState ? true : false);
	joint->SetAsSpringDamper(m_axis.m_springDamperRegularizer, m_axis.m_springK, m_axis.m_damperC);
	joint->SetConeLimit(m_maxConeAngle * ndDegreeToRad);
	joint->SetTwistLimits(m_axis.m_minLimit * ndDegreeToRad, m_axis.m_maxLimit * ndDegreeToRad);
	return joint;
}
