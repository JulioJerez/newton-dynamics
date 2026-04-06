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
#include "ndBody.h"
#include "ndShapeBox.h"
#include "ndCollision.h"
#include "ndShapeSphere.h"
#include "ndShapeCapsule.h"
#include "ndShapeCylinder.h"
#include "ndShapeCompound.h"
#include "ndShapeConvexHull.h"
#include "ndMeshBaseComponents.h"
#include "ndShapeChamferCylinder.h"
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

ndShape* ndMeshCollisionShapeNull::CreateObject() const
{
	return new ndShapeNull();
}

void ndMeshCollisionShapeSphere::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "constructor", ndShapeSphere::StaticClassName());
	xmlSaveParam(parent, "radius", m_radius);
}

void ndMeshCollisionShapeSphere::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	m_radius = xmlGetFloat(parent, "radius");
}

ndShape* ndMeshCollisionShapeSphere::CreateObject() const
{
	return new ndShapeSphere(m_radius);
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

ndShape* ndMeshCollisionShapeBox::CreateObject() const
{
	return new ndShapeBox(m_x, m_y, m_z);
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

ndShape* ndMeshCollisionShapeCapsule::CreateObject() const
{
	return new ndShapeCapsule(m_radius0, m_radius1, m_height);
}

void ndMeshCollisionShapeCylinder::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "constructor", ndShapeCapsule::StaticClassName());
	xmlSaveParam(parent, "radio0", m_radius0);
	xmlSaveParam(parent, "radio1", m_radius1);
	xmlSaveParam(parent, "heigh", m_height);
}

void ndMeshCollisionShapeCylinder::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	m_radius0 = xmlGetFloat(parent, "radio0");
	m_radius1 = xmlGetFloat(parent, "radio0");
	m_height = xmlGetFloat(parent, "heigh");
}

ndShape* ndMeshCollisionShapeCylinder::CreateObject() const
{
	return new ndShapeCapsule(m_radius0, m_radius1, m_height);
}

void ndMeshCollisionShapeChamferCylinder::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "constructor", ndShapeChamferCylinder::StaticClassName());
	xmlSaveParam(parent, "radio", m_radius);
	xmlSaveParam(parent, "heigh", m_height);
}

void ndMeshCollisionShapeChamferCylinder::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	m_height = xmlGetFloat(parent, "heigh");
	m_radius = xmlGetFloat(parent, "radio");
}

ndShape* ndMeshCollisionShapeChamferCylinder::CreateObject() const
{
	return new ndShapeChamferCylinder(m_radius, m_height);
}

void ndMeshCollisionShapeConvexHull::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "constructor", ndShapeConvexHull::StaticClassName());
	xmlSaveParam(parent, "pointcloud", m_points);
}

void ndMeshCollisionShapeConvexHull::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	m_points.SetCount(0);
	xmlGetFloatArray3(parent, "pointcloud", m_points);
}

ndShape* ndMeshCollisionShapeConvexHull::CreateObject() const
{
	return new ndShapeConvexHull(ndInt32(m_points.GetCount()), sizeof(ndVector), ndFloat32(0.0f), &m_points[0].m_x);
}

ndShape* ndMeshCollisionShapeCompound::CreateObject() const
{
	ndShapeCompound* const compoundShape = new ndShapeCompound();
	compoundShape->BeginAddRemove();
	for (ndList<ndSharedPtr<ndMeshShapeInstance>>::ndNode* node = m_subShapes.GetFirst(); node; node = node->GetNext())
	{
		const ndSharedPtr<ndMeshShapeInstance>& subMeshInstancePtr = node->GetInfo();
		const ndMeshShapeInstance* subMeshInstance = *subMeshInstancePtr;
		compoundShape->AddCollision(subMeshInstance->CreateObject());
	}
	compoundShape->EndAddRemove();

	return compoundShape;
}

void ndMeshCollisionShapeCompound::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndAssert(0);
}

void ndMeshCollisionShapeCompound::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndAssert(0);
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
	if (strcmp(constructor, ndShapeBox::StaticClassName()) == 0)
	{
		m_shape = ndSharedPtr<ndMeshCollisionShape>(new ndMeshCollisionShapeBox());
		m_shape->DeserializeFromXml(xmlShape);
	}
	else if (strcmp(constructor, ndShapeSphere::StaticClassName()) == 0)
	{
		m_shape = ndSharedPtr<ndMeshCollisionShape>(new ndMeshCollisionShapeSphere());
		m_shape->DeserializeFromXml(xmlShape);
	}
	else if (strcmp(constructor, ndShapeCapsule::StaticClassName()) == 0)
	{
		m_shape = ndSharedPtr<ndMeshCollisionShape>(new ndMeshCollisionShapeCapsule());
		m_shape->DeserializeFromXml(xmlShape);
	}
	else if (strcmp(constructor, ndShapeChamferCylinder::StaticClassName()) == 0)
	{
		m_shape = ndSharedPtr<ndMeshCollisionShape>(new ndMeshCollisionShapeChamferCylinder());
		m_shape->DeserializeFromXml(xmlShape);
	}
	else if (strcmp(constructor, ndShapeConvexHull::StaticClassName()) == 0)
	{
		m_shape = ndSharedPtr<ndMeshCollisionShape>(new ndMeshCollisionShapeConvexHull());
		m_shape->DeserializeFromXml(xmlShape);
	}
	else
	{
		ndExpandTraceMessage("warning ndMesh has a null shape\n");
		m_shape = ndSharedPtr<ndMeshCollisionShape>(new ndMeshCollisionShapeNull());
		m_shape->DeserializeFromXml(xmlShape);
	}
}

ndShapeInstance* ndMeshShapeInstance::CreateObject() const
{
	ndShapeInstance* const shapeInstance = new ndShapeInstance(m_shape->CreateObject());
	shapeInstance->SetLocalMatrix(m_localMatrix);
	shapeInstance->SetScale(m_scale);
	shapeInstance->m_alignmentMatrix = m_alignmentMatrix;
	return shapeInstance;
}

ndMeshBody::ndMeshBody()
	:ndClassAlloc()
	,m_veloc(ndVector::m_zero)
	,m_omega(ndVector::m_zero)
	,m_localCentreOfMass(ndVector::m_zero)
{
	m_classConstructor = ndString("ndBody");
}

ndMeshBody::~ndMeshBody()
{
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

ndBody* ndMeshBody::CreateObject() const
{
	ndAssert(0);
	return nullptr;
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

ndBody* ndMeshBodyKinematic::CreateObject() const
{
	ndAssert(0);
	return nullptr;
}

ndMeshJoint::ndMeshJoint()
	:ndClassAlloc()
	,m_localFrame0(ndGetIdentityMatrix())
	,m_localFrame1(ndGetIdentityMatrix())
	,m_constructor("ndJointFix6dof")
{
}

ndMeshJoint::ndMeshJoint(const ndJointBilateralConstraint* const joint)
	:ndClassAlloc()
	,m_localFrame0(joint->GetLocalMatrix0())
	,m_localFrame1(joint->GetLocalMatrix1())
	,m_constructor(joint->ClassName())
{
}

ndMeshJoint::~ndMeshJoint()
{
}

ndJointBilateralConstraint* ndMeshJoint::CreateObject(ndBodyKinematic* const, ndBodyKinematic* const) const
{
	ndExpandTraceMessage("ndMesh joint: %s serialization not Implemented", m_constructor.GetStr());
	ndAssert(0);
	return nullptr;
}

void ndMeshJoint::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "constructor", m_constructor.GetStr());
	xmlSaveParam(parent, "localFrame0", m_localFrame0);
	xmlSaveParam(parent, "localFrame1", m_localFrame1);
}

void ndMeshJoint::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	m_constructor = ndString(xmlGetString(parent, "constructor"));
	m_localFrame0 = xmlGetMatrix(parent, "localFrame0");
	m_localFrame1 = xmlGetMatrix(parent, "localFrame1");
}
