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
#include "ndMesh.h"
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

ndMeshCollisionShape::ndMeshCollisionShape(const char* const constructor)
	:ndClassAlloc()
	,m_constructor(constructor)
{
}

ndMeshCollisionShape::ndMeshCollisionShape(const ndMeshCollisionShape& other)
	:ndClassAlloc()
	,m_constructor(other.m_constructor)
{
}

ndMeshCollisionShape* ndMeshCollisionShape::Duplicate() const
{
	ndAssert(0);
	return nullptr;
}

bool ndMeshCollisionShape::operator==(const ndMeshCollisionShape& other) const
{
	return m_constructor == other.m_constructor;
}

ndMeshCollisionShape::~ndMeshCollisionShape()
{
}

ndMeshCollisionShapeNull::ndMeshCollisionShapeNull()
	:ndMeshCollisionShape(ndShapeNull::StaticClassName())
{
}

ndMeshCollisionShapeNull::ndMeshCollisionShapeNull(const ndMeshCollisionShapeNull& other)
	:ndMeshCollisionShape(other)
{
}

ndMeshCollisionShape* ndMeshCollisionShapeNull::Duplicate() const
{
	return new ndMeshCollisionShapeNull(*this);
}

bool ndMeshCollisionShapeNull::operator==(const ndMeshCollisionShape& other) const
{
	bool test = ndMeshCollisionShape::operator==(other);
	return test;
}

void ndMeshCollisionShapeNull::ApplyScale(ndFloat32)
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

ndMeshCollisionShapeSphere::ndMeshCollisionShapeSphere()
	:ndMeshCollisionShape(ndShapeSphere::StaticClassName())
{
}

ndMeshCollisionShapeSphere::ndMeshCollisionShapeSphere(const ndMeshCollisionShapeSphere& other)
	:ndMeshCollisionShape(other)
	,m_radius(other.m_radius)
{
}

ndMeshCollisionShape* ndMeshCollisionShapeSphere::Duplicate() const
{
	return new ndMeshCollisionShapeSphere(*this);
}

bool ndMeshCollisionShapeSphere::operator==(const ndMeshCollisionShape& other) const
{
	bool test = ndMeshCollisionShape::operator==(other);
	if (test)
	{
		const ndMeshCollisionShapeSphere* const otherShape = (ndMeshCollisionShapeSphere*)&other;
		test = test && (m_radius == otherShape->m_radius);
	}
	return false;
}

void ndMeshCollisionShapeSphere::ApplyScale(ndFloat32 scale)
{
	m_radius *= scale;
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

ndMeshCollisionShapeBox::ndMeshCollisionShapeBox()
	:ndMeshCollisionShape(ndShapeBox::StaticClassName())
{
}

ndMeshCollisionShapeBox::ndMeshCollisionShapeBox(const ndMeshCollisionShapeBox& other)
	:ndMeshCollisionShape(other)
	,m_x(other.m_x)
	,m_y(other.m_y) 
	,m_z(other.m_z)
{
}

ndMeshCollisionShape* ndMeshCollisionShapeBox::Duplicate() const
{
	return new ndMeshCollisionShapeBox(*this);
}

bool ndMeshCollisionShapeBox::operator==(const ndMeshCollisionShape& other) const
{
	bool test = ndMeshCollisionShape::operator==(other);
	if (test)
	{
		const ndMeshCollisionShapeBox* const otherShape = (ndMeshCollisionShapeBox*)&other;
		test = test && (m_x == otherShape->m_x);
		test = test && (m_y == otherShape->m_y);
		test = test && (m_z == otherShape->m_z);
	}
	return false;
}

void ndMeshCollisionShapeBox::ApplyScale(ndFloat32 scale)
{
	m_x *= scale;
	m_y *= scale;
	m_z *= scale;
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

ndMeshCollisionShapeCapsule::ndMeshCollisionShapeCapsule()
	:ndMeshCollisionShape(ndShapeCapsule::StaticClassName())
{
}

ndMeshCollisionShapeCapsule::ndMeshCollisionShapeCapsule(const ndMeshCollisionShapeCapsule& other)
	:ndMeshCollisionShape(other)
	,m_height(other.m_height)
	,m_radius0(other.m_radius0)
	,m_radius1(other.m_radius1)
{
}

ndMeshCollisionShape* ndMeshCollisionShapeCapsule::Duplicate() const
{
	return new ndMeshCollisionShapeCapsule(*this);
}

bool ndMeshCollisionShapeCapsule::operator==(const ndMeshCollisionShape& other) const
{
	bool test = ndMeshCollisionShape::operator==(other);
	if (test)
	{
		const ndMeshCollisionShapeCapsule* const otherShape = (ndMeshCollisionShapeCapsule*)&other;
		test = test && (m_height == otherShape->m_height);
		test = test && (m_radius0 == otherShape->m_radius0);
		test = test && (m_radius1 == otherShape->m_radius1);
	}
	return false;
}

void ndMeshCollisionShapeCapsule::ApplyScale(ndFloat32 scale)
{
	m_radius0 *= scale;
	m_radius1 *= scale;
	m_height *= scale;
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

ndMeshCollisionShapeCylinder::ndMeshCollisionShapeCylinder()
	:ndMeshCollisionShape(ndShapeCylinder::StaticClassName())
{
}

ndMeshCollisionShapeCylinder::ndMeshCollisionShapeCylinder(const ndMeshCollisionShapeCylinder& other)
	:ndMeshCollisionShape(other)
	,m_height(other.m_height)
	,m_radius0(other.m_radius0)
	,m_radius1(other.m_radius1)
{
}

ndMeshCollisionShape* ndMeshCollisionShapeCylinder::Duplicate() const
{
	return new ndMeshCollisionShapeCylinder(*this);
}

bool ndMeshCollisionShapeCylinder::operator==(const ndMeshCollisionShape& other) const
{
	bool test = ndMeshCollisionShape::operator==(other);
	if (test)
	{
		const ndMeshCollisionShapeCylinder* const otherShape = (ndMeshCollisionShapeCylinder*)&other;
		test = test && (m_height == otherShape->m_height);
		test = test && (m_radius0 == otherShape->m_radius0);
		test = test && (m_radius1 == otherShape->m_radius1);
	}
	return false;
}

void ndMeshCollisionShapeCylinder::ApplyScale(ndFloat32 scale)
{
	m_radius0 *= scale;
	m_radius1 *= scale;
	m_height *= scale;
}

void ndMeshCollisionShapeCylinder::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "constructor", ndShapeCylinder::StaticClassName());
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
	return new ndShapeCylinder(m_radius0, m_radius1, m_height);
}

ndMeshCollisionShapeChamferCylinder::ndMeshCollisionShapeChamferCylinder()
	:ndMeshCollisionShape(ndShapeChamferCylinder::StaticClassName())
{
}

ndMeshCollisionShapeChamferCylinder::ndMeshCollisionShapeChamferCylinder(const ndMeshCollisionShapeChamferCylinder& other)
	:ndMeshCollisionShape(other)
	,m_height(other.m_height)
	,m_radius(other.m_radius)
{
}

ndMeshCollisionShape* ndMeshCollisionShapeChamferCylinder::Duplicate() const
{
	return new ndMeshCollisionShapeChamferCylinder(*this);
}

bool ndMeshCollisionShapeChamferCylinder::operator==(const ndMeshCollisionShape& other) const
{
	bool test = ndMeshCollisionShape::operator==(other);
	if (test)
	{
		const ndMeshCollisionShapeChamferCylinder* const otherShape = (ndMeshCollisionShapeChamferCylinder*)&other;
		test = test && (m_height == otherShape->m_height);
		test = test && (m_radius == otherShape->m_radius);
	}
	return false;
}

void ndMeshCollisionShapeChamferCylinder::ApplyScale(ndFloat32 scale)
{
	m_height *= scale;
	m_radius *= scale;
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

ndMeshCollisionShapeConvexHull::ndMeshCollisionShapeConvexHull()
	:ndMeshCollisionShape(ndShapeConvexHull::StaticClassName())
	,m_points()
	,m_tolarence(ndFloat32 (0.0f))
	,m_maxPointCount(256)
{
}

ndMeshCollisionShapeConvexHull::ndMeshCollisionShapeConvexHull(const ndMeshCollisionShapeConvexHull& other)
	:ndMeshCollisionShape(other)
	,m_tolarence(other.m_tolarence)
	,m_maxPointCount(other.m_maxPointCount)
{
	for (ndInt32 i = 0; i < other.m_points.GetCount(); ++i)
	{
		m_points.PushBack(other.m_points[i]);
	}
}

ndMeshCollisionShape* ndMeshCollisionShapeConvexHull::Duplicate() const
{
	return new ndMeshCollisionShapeConvexHull(*this);
}

bool ndMeshCollisionShapeConvexHull::operator==(const ndMeshCollisionShape& other) const
{
	bool test = ndMeshCollisionShape::operator==(other);
	if (test)
	{
		const ndMeshCollisionShapeConvexHull* const otherShape = (ndMeshCollisionShapeConvexHull*)&other;
		test = test && (m_tolarence == otherShape->m_tolarence);
		test = test && (m_maxPointCount == otherShape->m_maxPointCount);
		for (ndInt32 i = 0; test && (i < m_points.GetCount()); ++i)
		{
			const ndVector diff(m_points[i] - otherShape->m_points[i]);
			ndFloat32 err2 = diff.DotProduct(diff & ndVector::m_triplexMask).GetScalar();
			test = test && (err2 < ndFloat32 (1.0e-6f));
		}
	}
	return false;
}

void ndMeshCollisionShapeConvexHull::ApplyScale(ndFloat32 scale)
{
	for (ndInt32 i = 0; i < m_points.GetCount(); ++i)
	{
		m_points[i] = m_points[i].Scale(scale);
		m_points[i].m_w = ndFloat32(1.0f);
	}
}

void ndMeshCollisionShapeConvexHull::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "constructor", ndShapeConvexHull::StaticClassName());
	xmlSaveParam(parent, "pointcloud", m_points);
	xmlSaveParam(parent, "tolerance", m_tolarence);
	xmlSaveParam(parent, "maxPoints", m_maxPointCount);
}

void ndMeshCollisionShapeConvexHull::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	m_points.SetCount(0);
	xmlGetFloatArray3(parent, "pointcloud", m_points);
	m_tolarence = ndFloat32(0.0f);
	m_maxPointCount = ndInt32 (m_points.GetCount());
	if (xmlHasAttribute(parent, "tolerance"))
	{
		m_maxPointCount = xmlGetInt(parent, "maxPoints");
		m_tolarence = xmlGetFloat(parent, "tolerance");
	}
}

ndShape* ndMeshCollisionShapeConvexHull::CreateObject() const
{
	ndShape* const hull = new ndShapeConvexHull(ndInt32(m_points.GetCount()), sizeof(ndVector), m_tolarence, &m_points[0].m_x, m_maxPointCount);
	ndShapeInfo info(hull->GetShapeInfo());
	//ndMeshCollisionShapeConvexHull* const self = (ndMeshCollisionShapeConvexHull*)this;
	//self->m_maxPointCount = info.m_convexhull.m_vertexCount;
	return hull;
}

ndMeshCollisionShapeCompound::ndMeshCollisionShapeCompound()
	:ndMeshCollisionShape(ndShapeCompound::StaticClassName())
{
}

ndMeshCollisionShapeCompound::ndMeshCollisionShapeCompound(const ndMeshCollisionShapeCompound& other)
	:ndMeshCollisionShape(other)
{
	for (ndList<ndSharedPtr<ndMeshShapeInstance>>::ndNode* node = other.m_subShapes.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndMeshShapeInstance>& otherInstance = node->GetInfo();
		ndSharedPtr<ndMeshShapeInstance> instance (new ndMeshShapeInstance(**otherInstance));
		m_subShapes.Append(instance);
	}
}

ndMeshCollisionShape* ndMeshCollisionShapeCompound::Duplicate() const
{
	return new ndMeshCollisionShapeCompound(*this);
}

bool ndMeshCollisionShapeCompound::operator==(const ndMeshCollisionShape& other) const
{
	bool test = ndMeshCollisionShape::operator==(other);
	if (test)
	{
		const ndMeshCollisionShapeCompound* const otherShape = (ndMeshCollisionShapeCompound*)&other;
		test = test && (m_subShapes.GetCount() == otherShape->m_subShapes.GetCount());
		ndList<ndSharedPtr<ndMeshShapeInstance>>::ndNode* selftNode = m_subShapes.GetFirst();
		for (ndList<ndSharedPtr<ndMeshShapeInstance>>::ndNode* node = otherShape->m_subShapes.GetFirst(); node; node = node->GetNext())
		{
			const ndSharedPtr<ndMeshShapeInstance>& otherInstance = node->GetInfo();
			const ndSharedPtr<ndMeshShapeInstance>& selfInstance = selftNode->GetInfo();
			test = test && (**selfInstance == **otherInstance);
			selftNode = selftNode->GetNext();
		}
	}
	return false;
}

//void ndMeshCollisionShapeCompound::ApplyScale(ndFloat32 scale)
void ndMeshCollisionShapeCompound::ApplyScale(ndFloat32)
{
	ndAssert(0);
}

ndShape* ndMeshCollisionShapeCompound::CreateObject() const
{
	ndShapeCompound* const compoundShape = new ndShapeCompound();
	compoundShape->BeginAddRemove();
	for (ndList<ndSharedPtr<ndMeshShapeInstance>>::ndNode* node = m_subShapes.GetFirst(); node; node = node->GetNext())
	{
		const ndSharedPtr<ndMeshShapeInstance>& subMeshInstancePtr = node->GetInfo();
		const ndMeshShapeInstance* subMeshInstance = *subMeshInstancePtr;
		ndSharedPtr<ndShapeInstance> subIntance(subMeshInstance->CreateObject());
		compoundShape->AddCollision(*subIntance);
	}
	compoundShape->EndAddRemove();

	return compoundShape;
}

void ndMeshCollisionShapeCompound::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "constructor", ndShapeCompound::StaticClassName());
	for (ndList<ndSharedPtr<ndMeshShapeInstance>>::ndNode* node = m_subShapes.GetFirst(); node; node = node->GetNext())
	{
		nd::TiXmlElement* const subInstanceNode = new nd::TiXmlElement("collisionInstance");
		parent->LinkEndChild(subInstanceNode);
		ndSharedPtr<ndMeshShapeInstance>& subInstance = node->GetInfo();
		subInstance->SerializeToXml(subInstanceNode);
	}
}

void ndMeshCollisionShapeCompound::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	for (const nd::TiXmlNode* node = parent->FirstChild("collisionInstance"); node; node = node->NextSibling("collisionInstance"))
	{
		ndSharedPtr<ndMeshShapeInstance> instance(new ndMeshShapeInstance);
		instance->DeserializeFromXml((nd::TiXmlElement*)node);
		m_subShapes.Append(instance);
	}
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

ndMeshShapeInstance::ndMeshShapeInstance(const ndMeshShapeInstance& other)
	:ndClassAlloc()
	,m_localMatrix(other.m_localMatrix)
	,m_alignmentMatrix(other.m_alignmentMatrix)
	,m_scale(other.m_scale)
	,m_shape(ndSharedPtr<ndMeshCollisionShape>(other.m_shape->Duplicate()))
{
}

ndMeshShapeInstance& ndMeshShapeInstance::operator=(const ndMeshShapeInstance& other)
{
	m_scale = other.m_scale;
	m_localMatrix = other.m_localMatrix;
	m_alignmentMatrix = other.m_alignmentMatrix;
	m_shape = other.m_shape;
	return *this;
}

bool ndMeshShapeInstance::operator == (const ndMeshShapeInstance& other) const
{
	bool test = (m_localMatrix * other.m_localMatrix.OrthoInverse()).TestIdentity(ndFloat32 (1.0e-5f));
	test = test && (m_alignmentMatrix * other.m_alignmentMatrix.OrthoInverse()).TestIdentity(ndFloat32(1.0e-5f));
	test = test && ((m_scale - other.m_scale).DotProduct(m_scale - other.m_scale)).GetScalar() < ndFloat32 (1.0e-6f);
	test = test && (**m_shape == **other.m_shape);
	return false;
}

void ndMeshShapeInstance::ApplyScale(const ndMatrix& scaleMatrix)
{
	ndMatrix scale(ndGetIdentityMatrix());
	scale[0][0] = m_scale[0];
	scale[1][1] = m_scale[1];
	scale[2][2] = m_scale[2];

	const ndMatrix matrix(m_alignmentMatrix * scale * m_localMatrix * scaleMatrix);

	ndVector localScale;
	ndMatrix stretchAxis;
	matrix.PolarDecomposition(m_localMatrix, localScale, stretchAxis);
	if (stretchAxis.TestIdentity())
	{
		m_shape->ApplyScale(localScale[0]);
		m_scale = ndVector::m_one & ndVector::m_triplexMask;
	}
	else
	{
		m_scale = localScale & ndVector::m_triplexMask;
		m_alignmentMatrix = stretchAxis;
	}
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
	else if (strcmp(constructor, ndShapeCylinder::StaticClassName()) == 0)
	{
		m_shape = ndSharedPtr<ndMeshCollisionShape>(new ndMeshCollisionShapeCylinder());
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
	else if (strcmp(constructor, ndShapeCompound::StaticClassName()) == 0)
	{
		m_shape = ndSharedPtr<ndMeshCollisionShape>(new ndMeshCollisionShapeCompound());
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

ndMeshBody::ndMeshBody(const ndMesh* const owner)
	:ndClassAlloc()
	,m_veloc(ndVector::m_zero)
	,m_omega(ndVector::m_zero)
	,m_localCentreOfMass(ndVector::m_wOne)
	,m_owner(owner)
{
	m_classConstructor = ndString("ndBody");
}

ndMeshBody::ndMeshBody(const ndMeshBody& other)
	:ndClassAlloc()
	,m_veloc(other.m_veloc)
	,m_omega(other.m_omega)
	,m_localCentreOfMass(other.m_localCentreOfMass)
	,m_classConstructor(other.m_classConstructor)
	,m_owner(other.m_owner)
{
}

ndMeshBody* ndMeshBody::Duplicate() const
{
	ndAssert(0);
	return nullptr;
}

void ndMeshBody::DuplicateFixDependencies(const ndMesh* const otherRoot)
{
	m_owner = otherRoot->FindByName(m_owner->GetName());
	ndAssert(m_owner);
}

bool ndMeshBody::operator==(const ndMeshBody& other) const
{
	bool test = (m_classConstructor == other.m_classConstructor);

	ndVector diff(m_localCentreOfMass - other.m_localCentreOfMass);
	test = test && diff.DotProduct(diff & ndVector::m_triplexMask).GetScalar() < ndFloat32(1.0e-6f);
	return test;
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
	m_localCentreOfMass.m_w = ndFloat32(1.0f);
}

ndBody* ndMeshBody::CreateObject() const
{
	ndAssert(0);
	return nullptr;
}

ndMeshBodyKinematic::ndMeshBodyKinematic(const ndMesh* const owner)
	:ndMeshBody(owner)
	,m_shapeInstance()
	,m_invMass(ndVector::m_zero)
	,m_inertiaPrincipalAxis(ndVector::m_zero)
	,m_maxAngleStep(ndFloat32 (45.0f))
	,m_maxLinearStep(ndFloat32(2.0f))
	,m_massVolumeWeigh(ndFloat32(1.0f))
{
	m_classConstructor = ndString("ndBodyKinematic");
}

ndMeshBodyKinematic::ndMeshBodyKinematic(const ndMeshBodyKinematic& other)
	:ndMeshBody(other)
	,m_shapeInstance(other.m_shapeInstance)
	,m_invMass(other.m_invMass)
	,m_inertiaPrincipalAxis(other.m_inertiaPrincipalAxis)
	,m_maxAngleStep(other.m_maxAngleStep)
	,m_maxLinearStep(other.m_maxLinearStep)
	,m_massVolumeWeigh(other.m_massVolumeWeigh)
{
}

ndMeshBody* ndMeshBodyKinematic::Duplicate() const
{
	return new ndMeshBodyKinematic(*this);
}

bool ndMeshBodyKinematic::operator==(const ndMeshBody& other) const
{
	bool test = ndMeshBody::operator==(other);

	auto Compare = [](const ndVector& a, const ndVector& b)
	{
		ndVector diff(a - b);
		return diff.DotProduct(diff).GetScalar() < ndFloat32(1.0e-6f);
	};

	const ndMeshBodyKinematic* const otherBody = (ndMeshBodyKinematic*)&other;
	test = test && (m_shapeInstance == otherBody->m_shapeInstance);
	test = test && Compare(m_invMass, otherBody->m_invMass);
	test = test && Compare(m_inertiaPrincipalAxis & ndVector::m_triplexMask, otherBody->m_inertiaPrincipalAxis & ndVector::m_triplexMask);
	test = test && (m_maxAngleStep == otherBody->m_maxAngleStep);
	test = test && (m_maxLinearStep == otherBody->m_maxLinearStep);
	test = test && (m_massVolumeWeigh == otherBody->m_massVolumeWeigh);
	return test;
}


void ndMeshBodyKinematic::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndMeshBody::SerializeToXml(parent);

	xmlSaveParam(parent, "inverseMass", m_invMass.m_w);
	xmlSaveParam(parent, "inverseDiagonalInertia", m_invMass);
	xmlSaveParam(parent, "principalAxis", m_inertiaPrincipalAxis);
	xmlSaveParam(parent, "maxAngleStep", m_maxAngleStep);
	xmlSaveParam(parent, "maxLinearStep", m_maxLinearStep);
	xmlSaveParam(parent, "massVolumeWeigh", m_massVolumeWeigh);

	nd::TiXmlElement* const collisionInstance = new nd::TiXmlElement("collisionInstance");
	parent->LinkEndChild(collisionInstance);
	m_shapeInstance.SerializeToXml(collisionInstance);
}

void ndMeshBodyKinematic::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndMeshBody::DeserializeFromXml(parent);

	m_invMass = xmlGetVector3(parent, "inverseDiagonalInertia");
	m_invMass.m_w = xmlGetFloat(parent, "inverseMass");
	m_inertiaPrincipalAxis = xmlGetFloat(parent, "principalAxis");
	m_maxAngleStep = xmlGetFloat(parent, "maxAngleStep");
	m_maxLinearStep = xmlGetFloat(parent, "maxLinearStep");

	m_massVolumeWeigh = ndFloat32(1.0f);
	if (xmlHasAttribute(parent, "massVolumeWeigh"))
	{
		m_massVolumeWeigh = xmlGetFloat(parent, "massVolumeWeigh");
	}

	const nd::TiXmlElement* const xmlShape = (nd::TiXmlElement*)parent->FirstChild("collisionInstance");
	ndAssert(xmlShape);
	m_shapeInstance.DeserializeFromXml(xmlShape);
}

ndBody* ndMeshBodyKinematic::CreateObject() const
{
	ndAssert(0);
	return nullptr;
}

ndMeshJoint::ndMeshJoint(const ndMesh* const owner)
	:ndClassAlloc()
	,m_localFrame0(ndGetIdentityMatrix())
	,m_localFrame1(ndGetIdentityMatrix())
	,m_constructor("ndJointFix6dof")
	,m_owner(ndWeakPtr<const ndMesh>(owner))
	,m_surrogateParent(ndWeakPtr<const ndMesh>(nullptr))
{
}

ndMeshJoint::ndMeshJoint(const ndMesh* const owner, const ndJointBilateralConstraint* const joint)
	:ndClassAlloc()
	,m_localFrame0(joint->GetLocalMatrix0())
	,m_localFrame1(joint->GetLocalMatrix1())
	,m_constructor(joint->ClassName())
	,m_owner(ndWeakPtr<const ndMesh>(owner))
	,m_surrogateParent(ndWeakPtr<const ndMesh>(nullptr))
{
}

ndMeshJoint::ndMeshJoint(const ndMeshJoint& other)
	:ndClassAlloc()
	,m_localFrame0(other.m_localFrame0)
	,m_localFrame1(other.m_localFrame1)
	,m_constructor(other.m_constructor)
	,m_owner(ndWeakPtr<const ndMesh>(other.m_owner))
	,m_surrogateParent(ndWeakPtr<const ndMesh>(other.m_surrogateParent))
{
}

ndMeshJoint::~ndMeshJoint()
{
}

ndMeshJoint* ndMeshJoint::Duplicate() const
{
	ndAssert(0);
	return nullptr;
}

void ndMeshJoint::DuplicateFixDependencies(const ndMesh* const otherRoot)
{
	m_owner = otherRoot->FindByName(m_owner->GetName());
	ndAssert(m_owner);

	if (m_surrogateParent)
	{
		const ndMesh* const root = m_owner->GetRoot();
		ndMesh* const surrogateParent = root->FindByName(m_surrogateParent->GetName());
		ndAssert(surrogateParent);
		m_surrogateParent = ndWeakPtr<const ndMesh>(surrogateParent);
	}
}

bool ndMeshJoint::operator==(const ndMeshJoint& other) const
{
	bool test = true;
	test = test && (m_constructor == other.m_constructor);
	test = test && (m_localFrame0 * other.m_localFrame0.OrthoInverse()).TestIdentity(ndFloat32(1.0e-6f));
	test = test && (m_localFrame1 * other.m_localFrame1.OrthoInverse()).TestIdentity(ndFloat32(1.0e-6f));

	return test;
}

void ndMeshJoint::ApplyTransform(const ndMatrix& transform)
{
	ndMatrix matrix0(m_localFrame0 * transform);
	ndMatrix matrix1(m_localFrame1 * transform);

	ndVector scale;
	ndMatrix stretchAxis;
	ndMatrix transformMatrix;

	matrix0.PolarDecomposition(transformMatrix, scale, stretchAxis);
	m_localFrame0 = transformMatrix;

	matrix1.PolarDecomposition(transformMatrix, scale, stretchAxis);
	m_localFrame1 = transformMatrix;
}

const ndMesh* ndMeshJoint::GetSurrogateParent() const
{
	return *m_surrogateParent;
}

void ndMeshJoint::SetSurrogateParent(const ndMesh* const surrodateParent)
{
	m_surrogateParent = ndWeakPtr<const ndMesh>(surrodateParent);
}

void ndMeshJoint::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "constructor", m_constructor.GetStr());
	xmlSaveParam(parent, "localFrame0", m_localFrame0);
	xmlSaveParam(parent, "localFrame1", m_localFrame1);

	if (m_surrogateParent)
	{
		xmlSaveParam(parent, "surrogateParent", m_surrogateParent->GetName().GetStr());
	}
}

void ndMeshJoint::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	m_constructor = ndString(xmlGetString(parent, "constructor"));
	m_localFrame0 = xmlGetMatrix(parent, "localFrame0");
	m_localFrame1 = xmlGetMatrix(parent, "localFrame1");

	if (xmlHasAttribute(parent, "surrogateParent"))
	{
		const char* const name = xmlGetString(parent, "surrogateParent");
		m_surrogateParent = ndWeakPtr<const ndMesh> (m_owner->GetRoot()->FindByName(name));
	}
}

ndJointBilateralConstraint* ndMeshJoint::CreateObject(ndBodyKinematic* const, ndBodyKinematic* const) const
{
	ndExpandTraceMessage("ndMesh joint: %s serialization not Implemented", m_constructor.GetStr());
	ndAssert(0);
	return nullptr;
}

ndMeshCollidingPair::ndMeshCollidingPair()
	:ndClassAlloc()
{
	ndAssert(0);
}

ndMeshCollidingPair::ndMeshCollidingPair(const ndMesh* const node0, const ndMesh* const node1)
	:ndClassAlloc()
	,m_childNode((node0->GetName() < node1->GetName()) ? node0 : node1)
	,m_parentNode((node0->GetName() < node1->GetName()) ? node1 : node0)
{
}

ndMeshCollidingPair::~ndMeshCollidingPair()
{
}

void ndMeshCollidingPair::SerializeToXml(nd::TiXmlElement* const parent) const
{
	xmlSaveParam(parent, "reference0", m_childNode->GetName().GetStr());
	xmlSaveParam(parent, "reference1", m_parentNode->GetName().GetStr());
}

//void ndMeshCollidingPair::DeserializeFromXml(const nd::TiXmlElement* const parent)
void ndMeshCollidingPair::DeserializeFromXml(const nd::TiXmlElement* const)
{
	ndAssert(0);
}

ndMeshTransformModifier::ndMeshTransformModifier(const ndMesh* const owner, const ndMesh* const target)
	:ndClassAlloc()
	,m_owner(owner)
	,m_target(target)
{
}

ndMeshTransformModifier::ndMeshTransformModifier(const ndMeshTransformModifier& other)
	:ndClassAlloc()
	,m_owner(other.m_owner)
	,m_target(other.m_target)
{
}

ndMeshTransformModifier::~ndMeshTransformModifier()
{
}

ndMeshTransformModifier* ndMeshTransformModifier::Duplicate() const
{
	ndAssert(0);
	return nullptr;
}

void ndMeshTransformModifier::DuplicateFixDependencies(const ndMesh* const otherRoot)
{
	m_owner = otherRoot->FindByName(m_owner->GetName());
	ndAssert(m_owner);

	if (m_target)
	{
		m_target = otherRoot->FindByName(m_target->GetName());
		ndAssert(m_target);
	}
}

void ndMeshTransformModifier::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndAssert(0);
}

void ndMeshTransformModifier::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndAssert(0);
}

bool ndMeshTransformModifier::operator==(const ndMeshTransformModifier& other) const
{
	ndAssert(0);
	return false;
}

ndMeshTransformModifierLookAt::ndMeshTransformModifierLookAt(const ndMesh* const owner, const ndMesh* const target)
	:ndMeshTransformModifier(owner, target)
{
}

ndMeshTransformModifierLookAt::ndMeshTransformModifierLookAt(const ndMeshTransformModifierLookAt& other)
	:ndMeshTransformModifier(other)
{
}

ndMeshTransformModifier* ndMeshTransformModifierLookAt::Duplicate() const
{
	return new ndMeshTransformModifierLookAt(*this);
}

void ndMeshTransformModifierLookAt::SerializeToXml(nd::TiXmlElement* const parent) const
{
	ndAssert(0);
}

void ndMeshTransformModifierLookAt::DeserializeFromXml(const nd::TiXmlElement* const parent)
{
	ndAssert(0);
}

bool ndMeshTransformModifierLookAt::operator==(const ndMeshTransformModifier& other) const
{
	ndAssert(0);
	return false;
}
