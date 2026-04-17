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

ndMeshBodyDynamic::ndMeshBodyDynamic(const ndMesh* const owner)
	:ndMeshBodyKinematic(owner)
	,m_intrinsicDamping(ndVector::m_zero)
{
	m_classConstructor = ndString("ndBodyDynamic");
}

void ndMeshBodyDynamic::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshBodyKinematic::SerializeToXml(parent);

	xmlSaveParam(parent, "intrinsicLinearDamping", m_intrinsicDamping.m_w);
	xmlSaveParam(parent, "intrinsicAngularDamping", m_intrinsicDamping);

	nd::TiXmlElement* const collidingPair = new nd::TiXmlElement("collidingPairs");
	parent->LinkEndChild(collidingPair);
	for (ndInt32 i = 0; i < m_collidingPair.GetCount(); ++i)
	{
		const ndMesh* const node = *m_collidingPair[i];
		xmlSaveParam(collidingPair, "otherBody", node->GetName().GetStr());
	}
}

void ndMeshBodyDynamic::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshBodyKinematic::DeserializeFromXml(parent);

	m_intrinsicDamping = xmlGetVector3(parent, "intrinsicAngularDamping");
	m_intrinsicDamping.m_w = xmlGetFloat(parent, "intrinsicLinearDamping");

	if (xmlHasAttribute(parent, "collidingPairs"))
	{
		const ndMesh* const root = m_owner->GetRoot();
		const nd::TiXmlElement* const collidingBodies = (nd::TiXmlElement*)parent->FirstChild("collidingPairs");
		for (const nd::TiXmlNode* node = collidingBodies->FirstChild("otherBody"); node; node = node->NextSibling("otherBody"))
		{
			const char* const name = xmlGetNameAttribute((nd::TiXmlElement*)node, "string");
			ndMesh* const otherNode = root->FindByName(name);
			ndAssert(otherNode);
			m_collidingPair.PushBack(ndWeakPtr<ndMesh>(otherNode));
		}
	}
}

ndBody* ndMeshBodyDynamic::CreateObject() const
{
	ndBodyDynamic* const body = new ndBodyDynamic();
	body->Deserialize(this);
	return body;
}

ndMeshLoopJoint::ndMeshLoopJoint()
	:ndClassAlloc()
{
}

ndMeshLoopJoint::ndMeshLoopJoint(const ndSharedPtr<ndMeshJoint>& joint, ndMesh* const otherNode)
	:ndClassAlloc()
	,m_joint(joint)
	,m_otherNode(otherNode)
{
}

ndMeshLoopJoint::~ndMeshLoopJoint()
{
}

//ndJointBilateralConstraint* ndMeshLoopJoint::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
ndJointBilateralConstraint* ndMeshLoopJoint::CreateObject(ndBodyKinematic* const, ndBodyKinematic* const) const
{
	ndAssert(0);
	return nullptr;
}

void ndMeshLoopJoint::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "otherNode", m_otherNode->GetName().GetStr());

	nd::TiXmlElement* const loopJoint = new nd::TiXmlElement("joint");
	parent->LinkEndChild(loopJoint);
	m_joint->SerializeToXml(loopJoint);
}

//void ndMeshLoopJoint::DeserializeFromXml(const nd::TiXmlElement* const parent)
void ndMeshLoopJoint::DeserializeFromXml(const nd::TiXmlElement* const)
{
	ndAssert(0);
}

ndMeshJointFix6dof::ndMeshJointFix6dof(const ndMesh* const owner)
	:ndMeshJoint(owner)
{
}

ndMeshJointFix6dof::ndMeshJointFix6dof(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
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

	if (m_axis.m_limitState)
	{
		m_axis.m_minLimit *= ndRadToDegree;
		m_axis.m_maxLimit *= ndRadToDegree;
	}
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
	m_axis.m_limitState = ndInt8(xmlGetInt(parent, "limitState"));
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

ndMeshJointSlider::ndMeshJointSlider(const ndMesh* const owner)
	:ndMeshJoint(owner)
{
}

ndMeshJointSlider::ndMeshJointSlider(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
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
	m_axis.m_limitState = ndInt8(xmlGetInt(parent, "limitState"));
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

ndMeshJointDoubleHinge::ndMeshJointDoubleHinge(const ndMesh* const owner)
	:ndMeshJoint(owner)
{
}

ndMeshJointDoubleHinge::ndMeshJointDoubleHinge(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
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
	m_axis0.m_limitState = ndInt8(xmlGetInt(xmlAxis0, "limitState"));

	const nd::TiXmlElement* const xmlAxis1 = (nd::TiXmlElement*)parent->FirstChild("axis1");
	m_axis1.m_springK = xmlGetFloat(xmlAxis1, "springK");
	m_axis1.m_damperC = xmlGetFloat(xmlAxis1, "damperC");
	m_axis1.m_minLimit = xmlGetFloat(xmlAxis1, "minLimit");
	m_axis1.m_maxLimit = xmlGetFloat(xmlAxis1, "maxLimit");
	m_axis1.m_springDamperRegularizer = xmlGetFloat(xmlAxis1, "springDamperRegularizer");
	m_axis1.m_limitState = ndInt8(xmlGetInt(xmlAxis1, "limitState"));
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

ndMeshJointSpherical::ndMeshJointSpherical(const ndMesh* const owner)
	:ndMeshJoint(owner)
	,m_maxConeAngle(0.0f)
	,m_coneAngleState(false)
{
}

ndMeshJointSpherical::ndMeshJointSpherical(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
	,m_maxConeAngle(0.0f)
	,m_coneAngleState(false)
{
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
		m_coneAngleState = ndInt8(xmlGetInt(parent, "coneLimitState"));
		m_axis.m_limitState = ndInt8(xmlGetInt(parent, "twistLimitState"));
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

ndMeshJointWheel::ndMeshJointWheel(const ndMesh* const owner)
	:ndMeshJoint(owner)
{
}

ndMeshJointWheel::ndMeshJointWheel(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
}

void ndMeshJointWheel::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);

	xmlSaveParam(parent, "springK", m_axis.m_springK);
	xmlSaveParam(parent, "damperC", m_axis.m_damperC);
	xmlSaveParam(parent, "upperStop", m_axis.m_minLimit);
	xmlSaveParam(parent, "lowerStop", m_axis.m_maxLimit);
	xmlSaveParam(parent, "brakeTorque", m_brakeTorque);
	xmlSaveParam(parent, "handBrakeTorque", m_handBrakeTorque);
	xmlSaveParam(parent, "steeringAngle", m_steeringAngle);
	xmlSaveParam(parent, "springDamperRegularizer", m_axis.m_springDamperRegularizer);
}

void ndMeshJointWheel::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);

	m_axis.m_springK = xmlGetFloat(parent, "springK");
	m_axis.m_damperC = xmlGetFloat(parent, "damperC");
	m_axis.m_minLimit = xmlGetFloat(parent, "upperStop");
	m_axis.m_maxLimit = xmlGetFloat(parent, "lowerStop");
	m_brakeTorque = xmlGetFloat(parent, "brakeTorque");
	m_handBrakeTorque = xmlGetFloat(parent, "handBrakeTorque");
	m_steeringAngle = xmlGetFloat(parent, "steeringAngle");
	m_axis.m_springDamperRegularizer = xmlGetFloat(parent, "springDamperRegularizer");
}

ndJointBilateralConstraint* ndMeshJointWheel::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_localFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_localFrame1 * parent->GetMatrix());

	ndWheelDescriptor desc;
	desc.m_springK = m_axis.m_springK;
	desc.m_damperC = m_axis.m_damperC;
	desc.m_upperStop = m_axis.m_minLimit;
	desc.m_lowerStop = m_axis.m_maxLimit;
	desc.m_regularizer = m_axis.m_springDamperRegularizer;
	desc.m_brakeTorque = m_brakeTorque;
	desc.m_steeringAngle = m_steeringAngle * ndDegreeToRad;
	desc.m_handBrakeTorque = m_handBrakeTorque;
	ndJointWheel* const joint = new ndJointWheel(pinAndPivotInChild, pinAndPivotInParent, child, parent, desc);
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

	subJoint->GetSpringDamperPosit(m_positAxis.m_springDamperRegularizer, m_positAxis.m_springK, m_positAxis.m_damperC);
	subJoint->GetLimitsPosit(m_positAxis.m_minLimit, m_positAxis.m_maxLimit);
	m_positAxis.m_limitState = subJoint->GetLimitStatePosit();

	subJoint->GetSpringDamperAngle(m_angleAxis.m_springDamperRegularizer, m_angleAxis.m_springK, m_angleAxis.m_damperC);
	subJoint->GetLimitsPosit(m_angleAxis.m_minLimit, m_angleAxis.m_maxLimit);
	m_angleAxis.m_limitState = subJoint->GetLimitStateAngle();

	if (m_angleAxis.m_limitState)
	{
		m_angleAxis.m_minLimit *= ndRadToDegree;
		m_angleAxis.m_maxLimit *= ndRadToDegree;
	}
}

void ndMeshJointRoller::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);

	nd::TiXmlElement* const axis0 = new nd::TiXmlElement("positAxis");
	parent->LinkEndChild(axis0);
	xmlSaveParam(axis0, "springK", m_positAxis.m_springK);
	xmlSaveParam(axis0, "damperC", m_positAxis.m_damperC);
	xmlSaveParam(axis0, "limitState", m_positAxis.m_limitState);
	xmlSaveParam(axis0, "minLimit", m_positAxis.m_minLimit);
	xmlSaveParam(axis0, "maxLimit", m_positAxis.m_maxLimit);
	xmlSaveParam(axis0, "springDamperRegularizer", m_positAxis.m_springDamperRegularizer);

	nd::TiXmlElement* const axis1 = new nd::TiXmlElement("angleAxis");
	parent->LinkEndChild(axis1);
	xmlSaveParam(axis1, "springK", m_angleAxis.m_springK);
	xmlSaveParam(axis1, "damperC", m_angleAxis.m_damperC);
	xmlSaveParam(axis1, "limitState", m_angleAxis.m_limitState);
	xmlSaveParam(axis1, "minLimit", m_angleAxis.m_minLimit);
	xmlSaveParam(axis1, "maxLimit", m_angleAxis.m_maxLimit);
	xmlSaveParam(axis1, "springDamperRegularizer", m_angleAxis.m_springDamperRegularizer);
}

void ndMeshJointRoller::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);

	const nd::TiXmlElement* const xmlAxis0 = (nd::TiXmlElement*)parent->FirstChild("positAxis");
	m_positAxis.m_springK = xmlGetFloat(xmlAxis0, "springK");
	m_positAxis.m_damperC = xmlGetFloat(xmlAxis0, "damperC");
	m_positAxis.m_minLimit = xmlGetFloat(xmlAxis0, "minLimit");
	m_positAxis.m_maxLimit = xmlGetFloat(xmlAxis0, "maxLimit");
	m_positAxis.m_springDamperRegularizer = xmlGetFloat(xmlAxis0, "springDamperRegularizer");
	m_positAxis.m_limitState = ndInt8(xmlGetInt(xmlAxis0, "limitState"));

	const nd::TiXmlElement* const xmlAxis1 = (nd::TiXmlElement*)parent->FirstChild("angleAxis");
	m_angleAxis.m_springK = xmlGetFloat(xmlAxis1, "springK");
	m_angleAxis.m_damperC = xmlGetFloat(xmlAxis1, "damperC");
	m_angleAxis.m_minLimit = xmlGetFloat(xmlAxis1, "minLimit");
	m_angleAxis.m_maxLimit = xmlGetFloat(xmlAxis1, "maxLimit");
	m_angleAxis.m_springDamperRegularizer = xmlGetFloat(xmlAxis1, "springDamperRegularizer");
	m_angleAxis.m_limitState = ndInt8(xmlGetInt(xmlAxis1, "limitState"));
}

ndJointBilateralConstraint* ndMeshJointRoller::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_localFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_localFrame1 * parent->GetMatrix());
	ndJointRoller* const joint = new ndJointRoller(pinAndPivotInChild, pinAndPivotInParent, child, parent);

	joint->SetLimitStatePosit(m_positAxis.m_limitState ? true : false);
	joint->SetLimitsPosit(m_positAxis.m_minLimit * ndDegreeToRad, m_positAxis.m_maxLimit * ndDegreeToRad);
	joint->SetAsSpringDamperPosit(m_positAxis.m_springDamperRegularizer, m_positAxis.m_springK, m_positAxis.m_damperC);
	
	joint->SetLimitStateAngle(m_angleAxis.m_limitState ? true : false);
	joint->SetLimitsAngle(m_angleAxis.m_minLimit * ndDegreeToRad, m_angleAxis.m_maxLimit * ndDegreeToRad);
	joint->SetAsSpringDamperAngle(m_angleAxis.m_springDamperRegularizer, m_angleAxis.m_springK, m_angleAxis.m_damperC);
	return joint;
}

ndMeshJointCylinder::ndMeshJointCylinder(const ndMesh* const owner)
	:ndMeshJoint(owner)
{
}

ndMeshJointCylinder::ndMeshJointCylinder(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(owner, joint)
{
}

void ndMeshJointCylinder::SerializeToXml(nd::TiXmlElement* const parent) const
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

void ndMeshJointCylinder::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);

	const nd::TiXmlElement* const xmlAxis0 = (nd::TiXmlElement*)parent->FirstChild("axis0");
	m_axis0.m_springK = xmlGetFloat(xmlAxis0, "springK");
	m_axis0.m_damperC = xmlGetFloat(xmlAxis0, "damperC");
	m_axis0.m_minLimit = xmlGetFloat(xmlAxis0, "minLimit");
	m_axis0.m_maxLimit = xmlGetFloat(xmlAxis0, "maxLimit");
	m_axis0.m_springDamperRegularizer = xmlGetFloat(xmlAxis0, "springDamperRegularizer");
	m_axis0.m_limitState = ndInt8(xmlGetInt(xmlAxis0, "limitState"));

	const nd::TiXmlElement* const xmlAxis1 = (nd::TiXmlElement*)parent->FirstChild("axis1");
	m_axis1.m_springK = xmlGetFloat(xmlAxis1, "springK");
	m_axis1.m_damperC = xmlGetFloat(xmlAxis1, "damperC");
	m_axis1.m_minLimit = xmlGetFloat(xmlAxis1, "minLimit");
	m_axis1.m_maxLimit = xmlGetFloat(xmlAxis1, "maxLimit");
	m_axis1.m_springDamperRegularizer = xmlGetFloat(xmlAxis1, "springDamperRegularizer");
	m_axis1.m_limitState = ndInt8(xmlGetInt(xmlAxis1, "limitState"));
}

ndJointBilateralConstraint* ndMeshJointCylinder::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_localFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_localFrame1 * parent->GetMatrix());
	ndJointCylinder* const joint = new ndJointCylinder(pinAndPivotInChild, pinAndPivotInParent, child, parent);

	joint->SetLimitStateAngle(m_axis0.m_limitState ? true : false);
	joint->SetLimitsAngle(m_axis0.m_minLimit * ndDegreeToRad, m_axis0.m_maxLimit * ndDegreeToRad);
	joint->SetAsSpringDamperAngle(m_axis0.m_springDamperRegularizer, m_axis0.m_springK, m_axis0.m_damperC);

	joint->SetLimitStatePosit(m_axis1.m_limitState ? true : false);
	joint->SetLimitsPosit(m_axis1.m_minLimit * ndDegreeToRad, m_axis1.m_maxLimit * ndDegreeToRad);
	joint->SetAsSpringDamperPosit(m_axis1.m_springDamperRegularizer, m_axis1.m_springK, m_axis1.m_damperC);

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
}

void ndMeshJointPlane::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);
	xmlSaveParam(parent, "controlRotation", m_controlRotation ? 1 : 0);
}

void ndMeshJointPlane::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);

	m_controlRotation = ndInt8(xmlGetInt(parent, "limitState"));
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
