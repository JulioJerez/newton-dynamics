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
#include "ndJointHinge.h"
#include "ndJointWheel.h"
#include "ndBodyDynamic.h"
#include "ndJointSpherical.h"
#include "ndMeshComponents.h"

ndMeshBodyDynamic::ndMeshBodyDynamic()
	:ndMeshBodyKinematic()
	,m_intrinsicDamping(ndVector::m_zero)
{
	m_classConstructor = ndString("ndBodyDynamic");
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

ndMeshJointFix6dof::ndMeshJointFix6dof()
	:ndMeshJoint()
{
}

ndMeshJointFix6dof::ndMeshJointFix6dof(const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(joint)
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
	ndAssert(0);
}

ndJointBilateralConstraint* ndMeshJointFix6dof::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	ndAssert(0);
	return nullptr;
}

ndMeshJointHinge::ndMeshJointHinge()
	:ndMeshJoint()
{
}

ndMeshJointHinge::ndMeshJointHinge(const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(joint)
{
}

void ndMeshJointHinge::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);
	xmlSaveParam(parent, "springK", m_springK);
	xmlSaveParam(parent, "damperC", m_damperC);
	xmlSaveParam(parent, "limitState", m_limitState);
	xmlSaveParam(parent, "minLimit", m_minLimit);
	xmlSaveParam(parent, "maxLimit", m_maxLimit);
	xmlSaveParam(parent, "springDamperRegularizer", m_springDamperRegularizer);
}

void ndMeshJointHinge::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);
	m_springK = xmlGetFloat(parent, "springK");
	m_damperC = xmlGetFloat(parent, "damperC");
	m_minLimit = xmlGetFloat(parent, "minLimit");
	m_maxLimit = xmlGetFloat(parent, "maxLimit");
	m_springDamperRegularizer = xmlGetFloat(parent, "springDamperRegularizer");
	m_limitState = ndInt8(xmlGetInt(parent, "limitState"));
}

ndJointBilateralConstraint* ndMeshJointHinge::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_locatFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_locatFrame1 * parent->GetMatrix());
	ndJointHinge* const joint = new ndJointHinge(pinAndPivotInChild, pinAndPivotInParent, child, parent);
	joint->SetAsSpringDamper(m_springDamperRegularizer, m_springK, m_damperC);
	joint->SetLimits(m_minLimit * ndDegreeToRad, m_maxLimit * ndDegreeToRad);
	joint->SetLimitState(m_limitState ? true : false);
	return joint;
}

ndMeshJointSlider::ndMeshJointSlider()
	:ndMeshJoint()
{
}

ndMeshJointSlider::ndMeshJointSlider(const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(joint)
{
}

void ndMeshJointSlider::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);
	xmlSaveParam(parent, "springK", m_springK);
	xmlSaveParam(parent, "damperC", m_damperC);
	xmlSaveParam(parent, "limitState", m_limitState);
	xmlSaveParam(parent, "minLimit", m_minLimit);
	xmlSaveParam(parent, "maxLimit", m_maxLimit);
	xmlSaveParam(parent, "springDamperRegularizer", m_springDamperRegularizer);
}

void ndMeshJointSlider::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndAssert(0);
}

ndMeshJointDoubleHinge::ndMeshJointDoubleHinge()
	:ndMeshJoint()
{
}

ndMeshJointDoubleHinge::ndMeshJointDoubleHinge(const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(joint)
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
	ndAssert(0);
}

ndMeshJointSpherical::ndMeshJointSpherical()
	:ndMeshJoint()
{
}

ndMeshJointSpherical::ndMeshJointSpherical(const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(joint)
{
}

void ndMeshJointSpherical::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);

	xmlSaveParam(parent, "rotation", m_rotation);
	xmlSaveParam(parent, "springK", m_springK);
	xmlSaveParam(parent, "damperC", m_damperC);
	xmlSaveParam(parent, "maxConeAngle", m_maxConeAngle);
	xmlSaveParam(parent, "minTwistAngle", m_minTwistAngle);
	xmlSaveParam(parent, "maxTwistAngle", m_maxTwistAngle);
	xmlSaveParam(parent, "springDamperRegularizer", m_springDamperRegularizer);
}

void ndMeshJointSpherical::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);

	m_rotation = xmlGetMatrix(parent, "rotation");
	m_springK = xmlGetFloat(parent, "springK");
	m_damperC = xmlGetFloat(parent, "damperC");
	m_maxConeAngle = xmlGetFloat(parent, "maxConeAngle");
	m_minTwistAngle = xmlGetFloat(parent, "minTwistAngle");
	m_maxTwistAngle = xmlGetFloat(parent, "maxTwistAngle");
	m_springDamperRegularizer = xmlGetFloat(parent, "springDamperRegularizer");
}

ndJointBilateralConstraint* ndMeshJointSpherical::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_locatFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_locatFrame1 * parent->GetMatrix());
	ndJointSpherical* const joint = new ndJointSpherical(pinAndPivotInChild, pinAndPivotInParent, child, parent);
	joint->SetOffsetRotation(m_rotation);
	joint->SetAsSpringDamper(m_springDamperRegularizer, m_springK, m_damperC);
	joint->SetConeLimit(m_maxConeAngle * ndDegreeToRad);
	joint->SetTwistLimits(m_minTwistAngle * ndDegreeToRad, m_maxTwistAngle * ndDegreeToRad);
	return joint;
}

ndMeshJointWheel::ndMeshJointWheel()
	:ndMeshJoint()
{
}

ndMeshJointWheel::ndMeshJointWheel(const ndJointBilateralConstraint* const joint)
	:ndMeshJoint(joint)
{
}

void ndMeshJointWheel::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshJoint::SerializeToXml(parent);

	xmlSaveParam(parent, "baseFrame", m_baseFrame);
	xmlSaveParam(parent, "springK", m_springK);
	xmlSaveParam(parent, "damperC", m_damperC);
	xmlSaveParam(parent, "upperStop", m_upperStop);
	xmlSaveParam(parent, "lowerStop", m_lowerStop);
	xmlSaveParam(parent, "brakeTorque", m_brakeTorque);
	xmlSaveParam(parent, "handBrakeTorque", m_handBrakeTorque);
	xmlSaveParam(parent, "steeringAngle", m_steeringAngle);
	xmlSaveParam(parent, "springDamperRegularizer", m_regularizer);
}

void ndMeshJointWheel::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshJoint::DeserializeFromXml(parent);

	m_baseFrame = xmlGetMatrix(parent, "baseFrame");
	m_springK = xmlGetFloat(parent, "springK");
	m_damperC = xmlGetFloat(parent, "damperC");
	m_upperStop = xmlGetFloat(parent, "upperStop");
	m_lowerStop = xmlGetFloat(parent, "lowerStop");
	m_brakeTorque = xmlGetFloat(parent, "brakeTorque");
	m_handBrakeTorque = xmlGetFloat(parent, "handBrakeTorque");
	m_steeringAngle = xmlGetFloat(parent, "steeringAngle");
	m_regularizer = xmlGetFloat(parent, "springDamperRegularizer");
}

ndJointBilateralConstraint* ndMeshJointWheel::CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const
{
	const ndMatrix pinAndPivotInChild(m_locatFrame0 * child->GetMatrix());
	const ndMatrix pinAndPivotInParent(m_locatFrame1 * parent->GetMatrix());

	ndWheelDescriptor desc;
	desc.m_springK = m_springK;
	desc.m_damperC = m_damperC;
	desc.m_upperStop = m_upperStop;
	desc.m_lowerStop = m_lowerStop;
	desc.m_regularizer = m_regularizer;
	desc.m_brakeTorque = m_brakeTorque;
	desc.m_steeringAngle = m_steeringAngle;
	desc.m_handBrakeTorque = m_handBrakeTorque;
	ndJointWheel* const joint = new ndJointWheel(pinAndPivotInChild, pinAndPivotInParent, child, parent, desc);

	//joint->SetOffsetRotation(m_rotation);
	//joint->SetAsSpringDamper(m_springDamperRegularizer, m_springK, m_damperC);
	//joint->SetConeLimit(m_maxConeAngle * ndDegreeToRad);
	//joint->SetTwistLimits(m_minTwistAngle * ndDegreeToRad, m_maxTwistAngle * ndDegreeToRad);
	return joint;

	return nullptr;
}
