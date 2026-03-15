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
#include "ndCollisionStdafx.h"
#include "VHACD.h"
#include "ndMesh.h"
#include "ndBody.h"
#include "ndCollision.h"
#include "ndMeshComponents.h"
#include "ndJointBilateralConstraint.h"

ndMeshCollisionShape::ndMeshCollisionShape()
	:ndClassAlloc()
{
}

ndMeshCollisionShape::~ndMeshCollisionShape()
{
}

void ndMeshCollisionShapeNull::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "constructor", ndShapeNull::StaticClassName());
}

void ndMeshCollisionShapeNull::DeserializeFromXml(const nd::TiXmlElement* const)
{
	// do nothing
}

void ndMeshCollisionShapeBox::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "constructor", ndShapeBox::StaticClassName());
	xmlSaveParam(parent, "x", m_x);
	xmlSaveParam(parent, "y", m_y);
	xmlSaveParam(parent, "z", m_z);
}

void ndMeshCollisionShapeBox::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	m_x = xmlGetFloat(parent, "x");
	m_y = xmlGetFloat(parent, "y");
	m_z = xmlGetFloat(parent, "z");
}

void ndMeshCollisionShapeCapsule::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "constructor", ndShapeCapsule::StaticClassName());
	xmlSaveParam(parent, "radio0", m_radius0);
	xmlSaveParam(parent, "radio1", m_radius1);
	xmlSaveParam(parent, "heigh", m_height);
}

void ndMeshCollisionShapeCapsule::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	m_radius0 = xmlGetFloat(parent, "radio0");
	m_radius1 = xmlGetFloat(parent, "radio0");
	m_height = xmlGetFloat(parent, "heigh");
}

ndMeshShapeInstance::ndMeshShapeInstance()
	:ndClassAlloc()
	,m_localMatrix(ndGetIdentityMatrix())
	,m_alignmentMatrix(ndGetIdentityMatrix())
	,m_scale(ndVector::m_one)
	,m_shape(nullptr)
{
}

ndMeshShapeInstance::ndMeshShapeInstance(const ndShapeInstance& instance)
	:ndClassAlloc()
	,m_localMatrix(instance.GetLocalMatrix())
	,m_alignmentMatrix(instance.GetAlignmentMatrix())
	,m_scale(instance.GetScale())
	,m_shape(instance.GetShape()->GetMeshShape())
{
}

void ndMeshShapeInstance::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "localMatrix", m_localMatrix);
	xmlSaveParam(parent, "alignmentMatrix", m_alignmentMatrix);
	xmlSaveParam(parent, "scale", m_scale);

	nd::TiXmlElement* const shapeNode = new nd::TiXmlElement("shape");
	parent->LinkEndChild(shapeNode);
	ndAssert(*m_shape);
	m_shape->SerializeToXml(shapeNode);
}

void ndMeshShapeInstance::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	m_localMatrix = xmlGetMatrix(parent, "localMatrix");
	m_alignmentMatrix = xmlGetMatrix(parent, "alignmentMatrix");
	m_scale = xmlGetVector3(parent, "scale");

	const nd::TiXmlElement* const xmlShape = (nd::TiXmlElement*)parent->FirstChild("shape");
	ndAssert(xmlShape);
	const char* const constructor = xmlGetString(xmlShape, "constructor");
	if (strcmp(constructor, "ndShapeBox") == 0)
	{
		m_shape = ndSharedPtr<ndMeshCollisionShape>(new ndMeshCollisionShapeBox());
		m_shape->DeserializeFromXml(xmlShape);
	}
	else if (strcmp(constructor, "ndShapeCapsule") == 0)
	{
		m_shape = ndSharedPtr<ndMeshCollisionShape>(new ndMeshCollisionShapeCapsule());
		m_shape->DeserializeFromXml(xmlShape);
	}
	else
	{
		ndAssert(0);
		m_shape = ndSharedPtr<ndMeshCollisionShape>(new ndMeshCollisionShapeNull());
		m_shape->DeserializeFromXml(xmlShape);
	}
}

ndMeshBody::ndMeshBody()
	:ndClassAlloc()
	,m_veloc(ndVector::m_zero)
	,m_omega(ndVector::m_zero)
	,m_localCentreOfMass(ndVector::m_zero)
{
	m_classConstructor = ndString("ndBody");
}

void ndMeshBody::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "veloc", m_veloc);
	xmlSaveParam(parent, "omega", m_omega);
	xmlSaveParam(parent, "com", m_localCentreOfMass);
}

void ndMeshBody::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	m_veloc = xmlGetVector3(parent, "veloc");
	m_omega = xmlGetVector3(parent, "omega");
	m_localCentreOfMass = xmlGetVector3(parent, "com");
}

ndMeshBodyKinematic::ndMeshBodyKinematic()
	:ndMeshBody()
	,m_shapeInstance()
	,m_invMass(ndVector::m_zero)
	,m_inertiaPrincipalAxis(ndGetIdentityMatrix())
	,m_maxAngleStep(ndFloat32 (45.0f))
	,m_maxLinearStep(ndFloat32(2.0f))
{
	m_classConstructor = ndString("ndBodyKinematic");
}

void ndMeshBodyKinematic::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshBody::SerializeToXml(parent);

	ndVector euler;
	ndVector axisOfInertia(m_inertiaPrincipalAxis.CalcPitchYawRoll(euler));
	axisOfInertia = axisOfInertia.Scale(ndRadToDegree);

	xmlSaveParam(parent, "inverseMass", m_invMass.m_w);
	xmlSaveParam(parent, "inverseDiagonalInertia", m_invMass);
	xmlSaveParam(parent, "principalAxis", axisOfInertia);
	xmlSaveParam(parent, "maxAngleStep", m_maxAngleStep);
	xmlSaveParam(parent, "maxLinearStep", m_maxLinearStep);

	nd::TiXmlElement* const collisionInstance = new nd::TiXmlElement("collisionInstance");
	parent->LinkEndChild(collisionInstance);
	m_shapeInstance.SerializeToXml(collisionInstance);
}

void ndMeshBodyKinematic::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshBody::DeserializeFromXml(parent);

	m_invMass = xmlGetVector3(parent, "inverseDiagonalInertia");
	m_invMass.m_w = xmlGetFloat(parent, "inverseMass");

	ndVector euler(xmlGetVector3(parent, "principalAxis"));
	euler.Scale(ndDegreeToRad);
	m_inertiaPrincipalAxis = ndPitchMatrix(euler.m_x) * ndYawMatrix(euler.m_y) * ndRollMatrix(euler.m_z);

	m_maxAngleStep = xmlGetFloat(parent, "maxAngleStep");
	m_maxLinearStep = xmlGetFloat(parent, "maxLinearStep");

	const nd::TiXmlElement* const xmlShape = (nd::TiXmlElement*)parent->FirstChild("collisionInstance");
	ndAssert(xmlShape);
	m_shapeInstance.DeserializeFromXml(xmlShape);
}

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

ndMeshBody::~ndMeshBody()
{
}

ndMeshJoint::ndMeshJoint()
	:ndClassAlloc()
	,m_locatFrame0(ndGetIdentityMatrix())
	,m_locatFrame1(ndGetIdentityMatrix())
	,m_constructor("ndJointFix6dof")
{
}

ndMeshJoint::ndMeshJoint(const ndJointBilateralConstraint* const joint)
	:ndClassAlloc()
	,m_locatFrame0(joint->GetLocalMatrix0())
	,m_locatFrame1(joint->GetLocalMatrix1())
	,m_constructor(joint->ClassName())
{
}

ndMeshJoint::~ndMeshJoint()
{
}

void ndMeshJoint::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "constructor", m_constructor.GetStr());
	xmlSaveParam(parent, "localFrame0", m_locatFrame0);
	xmlSaveParam(parent, "localFrame1", m_locatFrame1);
}

void ndMeshJoint::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndAssert(0);
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
	//xmlSaveParam(parent, "constructor", m_con);
	//ndAssert(0);
}

void ndMeshJointHinge::DeserializeFromXml(const nd::TiXmlElement* const parent)
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
	//ndAssert(0);
}

void ndMeshJointDoubleHinge::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndAssert(0);
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
	//ndAssert(0);
}

void ndMeshJointSlider::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndAssert(0);
}
