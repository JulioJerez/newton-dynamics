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

#ifndef __ND_MESH_BASE_COMPONENTS_H__
#define __ND_MESH_BASE_COMPONENTS_H__

#include "ndCore.h"

class ndBody;
class ndMesh;
class ndMeshEffect;
class ndShapeInstance;
class ndBodyKinematic;
class ndJointBilateralConstraint;

#define ND_MESH_COLLIDING_PAIRS		"__collidingPairs__"
#define ND_MESH_CONSTRAINT_LOOPS	"__constraintLoops__"

class ndMeshCustomProperty : public ndClassAlloc
{
	public:
	D_BASE_CLASS_REFLECTION(ndMeshCustomProperty);

	D_COLLISION_API ndMeshCustomProperty(ndMesh* const owner);
	D_COLLISION_API ndMeshCustomProperty(ndMesh* const owner, const ndMeshCustomProperty& other);
	D_COLLISION_API virtual ~ndMeshCustomProperty();

	D_COLLISION_API virtual ndMeshCustomProperty* Duplicate(ndMesh* const owner) const = 0;
	D_COLLISION_API virtual bool operator==(const ndMeshCustomProperty& other) const;

	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent);

	ndString m_name;
	ndWeakPtr<ndMesh> m_owner;
};

class ndMeshCustomPropertyFloat : public ndMeshCustomProperty
{
	public:
	D_CLASS_REFLECTION(ndMeshCustomPropertyFloat, ndMeshCustomProperty)

	D_COLLISION_API ndMeshCustomPropertyFloat(ndMesh* const owner);
	D_COLLISION_API ndMeshCustomPropertyFloat(ndMesh* const owner, const ndMeshCustomPropertyFloat& other);

	D_COLLISION_API virtual ndMeshCustomProperty* Duplicate(ndMesh* const owner) const override;
	D_COLLISION_API virtual bool operator==(const ndMeshCustomProperty& other) const override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndReal m_value;
};

class ndMeshCustomPropertyString : public ndMeshCustomProperty
{
	public:
	D_CLASS_REFLECTION(ndMeshCustomPropertyString, ndMeshCustomProperty)

	D_COLLISION_API ndMeshCustomPropertyString(ndMesh* const owner);
	D_COLLISION_API ndMeshCustomPropertyString(ndMesh* const owner, const ndMeshCustomPropertyString& other);

	D_COLLISION_API virtual ndMeshCustomProperty* Duplicate(ndMesh* const owner) const override;
	D_COLLISION_API virtual bool operator==(const ndMeshCustomProperty& other) const override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndString m_value;
};

class ndMeshCustomPropertyNode : public ndMeshCustomProperty
{
	public:
	D_CLASS_REFLECTION(ndMeshCustomPropertyNode, ndMeshCustomProperty)

	D_COLLISION_API ndMeshCustomPropertyNode(ndMesh* const owner);
	D_COLLISION_API ndMeshCustomPropertyNode(ndMesh* const owner, const ndMeshCustomPropertyNode& other);

	D_COLLISION_API virtual ndMeshCustomProperty* Duplicate(ndMesh* const owner) const override;
	D_COLLISION_API virtual bool operator==(const ndMeshCustomProperty& other) const override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndWeakPtr<ndMesh> m_value;
};

class ndMeshCollisionShape : public ndClassAlloc
{
	public:
	D_COLLISION_API ndMeshCollisionShape(const char* const constructor);
	D_COLLISION_API ndMeshCollisionShape(const ndMeshCollisionShape& other);

	D_COLLISION_API virtual ndMeshCollisionShape* Duplicate() const;
	D_COLLISION_API virtual bool operator==(const ndMeshCollisionShape& other) const;

	D_COLLISION_API virtual ~ndMeshCollisionShape();
	D_COLLISION_API virtual ndShape* CreateObject() const = 0;
	D_COLLISION_API virtual void ApplyScale(ndFloat32 scale) = 0;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent);

	ndString m_constructor;
};

class ndMeshCollisionShapeNull : public ndMeshCollisionShape
{
	public:
	D_COLLISION_API ndMeshCollisionShapeNull();

	D_COLLISION_API ndMeshCollisionShapeNull(const ndMeshCollisionShapeNull& other);
	D_COLLISION_API virtual ndMeshCollisionShape* Duplicate() const override;
	D_COLLISION_API virtual bool operator==(const ndMeshCollisionShape& other) const override;

	D_COLLISION_API virtual ndShape* CreateObject() const override;
	D_COLLISION_API virtual void ApplyScale(ndFloat32 scale) override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
};

class ndMeshCollisionShapeSphere : public ndMeshCollisionShape
{
	public:
	D_COLLISION_API ndMeshCollisionShapeSphere();

	D_COLLISION_API ndMeshCollisionShapeSphere(const ndMeshCollisionShapeSphere& other);
	D_COLLISION_API virtual ndMeshCollisionShape* Duplicate() const override;
	D_COLLISION_API virtual bool operator==(const ndMeshCollisionShape& other) const override;

	D_COLLISION_API virtual void ApplyScale(ndFloat32 scale) override;
	D_COLLISION_API virtual ndShape* CreateObject() const override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndFloat32 m_radius;
};

class ndMeshCollisionShapeBox : public ndMeshCollisionShape
{
	public:
	D_COLLISION_API ndMeshCollisionShapeBox();

	D_COLLISION_API ndMeshCollisionShapeBox(const ndMeshCollisionShapeBox& other);
	D_COLLISION_API virtual ndMeshCollisionShape* Duplicate() const override;
	D_COLLISION_API virtual bool operator==(const ndMeshCollisionShape& other) const override;

	D_COLLISION_API virtual ndShape* CreateObject() const override;
	D_COLLISION_API virtual void ApplyScale(ndFloat32 scale) override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndFloat32 m_x;
	ndFloat32 m_y;
	ndFloat32 m_z;
};

class ndMeshCollisionShapeCapsule : public ndMeshCollisionShape
{
	public:
	D_COLLISION_API ndMeshCollisionShapeCapsule();

	D_COLLISION_API ndMeshCollisionShapeCapsule(const ndMeshCollisionShapeCapsule& other);
	D_COLLISION_API virtual ndMeshCollisionShape* Duplicate() const override;
	D_COLLISION_API virtual bool operator==(const ndMeshCollisionShape& other) const override;

	D_COLLISION_API virtual ndShape* CreateObject() const override;
	D_COLLISION_API virtual void ApplyScale(ndFloat32 scale) override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndFloat32 m_height;
	ndFloat32 m_radius0;
	ndFloat32 m_radius1;
};

class ndMeshCollisionShapeCylinder : public ndMeshCollisionShape
{
	public:
	D_COLLISION_API ndMeshCollisionShapeCylinder();

	D_COLLISION_API ndMeshCollisionShapeCylinder(const ndMeshCollisionShapeCylinder& other);
	D_COLLISION_API virtual ndMeshCollisionShape* Duplicate() const override;
	D_COLLISION_API virtual bool operator==(const ndMeshCollisionShape& other) const override;

	D_COLLISION_API virtual ndShape* CreateObject() const override;
	D_COLLISION_API virtual void ApplyScale(ndFloat32 scale) override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndFloat32 m_height;
	ndFloat32 m_radius0;
	ndFloat32 m_radius1;
};

class ndMeshCollisionShapeChamferCylinder : public ndMeshCollisionShape
{
	public:
	D_COLLISION_API ndMeshCollisionShapeChamferCylinder();

	D_COLLISION_API ndMeshCollisionShapeChamferCylinder(const ndMeshCollisionShapeChamferCylinder& other);
	D_COLLISION_API virtual ndMeshCollisionShape* Duplicate() const override;
	D_COLLISION_API virtual bool operator==(const ndMeshCollisionShape& other) const override;

	D_COLLISION_API virtual ndShape* CreateObject() const override;
	D_COLLISION_API virtual void ApplyScale(ndFloat32 scale) override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndFloat32 m_height;
	ndFloat32 m_radius;
};

class ndMeshCollisionShapeWheel : public ndMeshCollisionShape
{
	public:
	D_COLLISION_API ndMeshCollisionShapeWheel();

	D_COLLISION_API ndMeshCollisionShapeWheel(const ndMeshCollisionShapeWheel& other);
	D_COLLISION_API virtual ndMeshCollisionShape* Duplicate() const override;
	D_COLLISION_API virtual bool operator==(const ndMeshCollisionShape& other) const override;

	D_COLLISION_API virtual ndShape* CreateObject() const override;
	D_COLLISION_API virtual void ApplyScale(ndFloat32 scale) override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
};

class ndMeshCollisionShapeConvexHull : public ndMeshCollisionShape
{
	public:
	D_COLLISION_API ndMeshCollisionShapeConvexHull();

	D_COLLISION_API ndMeshCollisionShapeConvexHull(const ndMeshCollisionShapeConvexHull& other);
	D_COLLISION_API virtual ndMeshCollisionShape* Duplicate() const override;
	D_COLLISION_API virtual bool operator==(const ndMeshCollisionShape& other) const override;

	D_COLLISION_API virtual ndShape* CreateObject() const override;
	D_COLLISION_API virtual void ApplyScale(ndFloat32 scale) override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndArray<ndVector> m_points;
	ndFloat32 m_tolarence;
	ndInt32 m_maxPointCount;
};

class ndMeshCollisionShapeCompound : public ndMeshCollisionShape
{
	public:
	D_COLLISION_API ndMeshCollisionShapeCompound();

	D_COLLISION_API ndMeshCollisionShapeCompound(const ndMeshCollisionShapeCompound& other);
	D_COLLISION_API virtual ndMeshCollisionShape* Duplicate() const override;
	D_COLLISION_API virtual bool operator==(const ndMeshCollisionShape& other) const override;

	D_COLLISION_API virtual ndShape* CreateObject() const override;
	D_COLLISION_API virtual void ApplyScale(ndFloat32 scale) override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndList<ndSharedPtr<ndMeshShapeInstance>> m_subShapes;
};

class ndMeshCollisionShapeTree : public ndMeshCollisionShape
{
	public:
	D_COLLISION_API ndMeshCollisionShapeTree();

	D_COLLISION_API ndMeshCollisionShapeTree(const ndMeshCollisionShapeTree& other);
	D_COLLISION_API virtual ndMeshCollisionShape* Duplicate() const override;
	D_COLLISION_API virtual bool operator==(const ndMeshCollisionShape& other) const override;

	D_COLLISION_API virtual ndShape* CreateObject() const override;
	D_COLLISION_API virtual void ApplyScale(ndFloat32 scale) override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndArray<ndVector> m_points;
	ndArray<ndInt32> m_indices;
	ndArray<ndAabbPolygonSoup::ndNode> m_nodes;
};

class ndMeshShapeInstance : public ndClassAlloc
{
	public:
	D_COLLISION_API ndMeshShapeInstance();
	D_COLLISION_API ndMeshShapeInstance(const ndShapeInstance& instance);

	D_COLLISION_API ndMeshShapeInstance(const ndMeshShapeInstance& other);

	D_COLLISION_API bool operator==(const ndMeshShapeInstance& other) const;
	D_COLLISION_API ndMeshShapeInstance& operator=(const ndMeshShapeInstance& other);

	D_COLLISION_API ndShapeInstance* CreateObject() const;
	D_COLLISION_API void ApplyScale(const ndMatrix& scaleMatrix);
	D_COLLISION_API void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_COLLISION_API void DeserializeFromXml(const nd::TiXmlElement* const parent);

	ndMatrix m_localMatrix;
	ndMatrix m_alignmentMatrix;
	ndVector m_scale;
	ndSharedPtr<ndMeshCollisionShape> m_shape;
	bool m_collidable;
};

class ndMeshBody : public ndClassAlloc
{
	public:
	D_COLLISION_API ndMeshBody(const ndMesh* const owner);
	D_COLLISION_API ndMeshBody(const ndMeshBody& other);
	D_COLLISION_API virtual ~ndMeshBody();

	D_COLLISION_API virtual ndMeshBody* Duplicate() const;
	D_COLLISION_API virtual void DuplicateFixDependencies(const ndMesh* const otherRoot);
	D_COLLISION_API virtual bool operator==(const ndMeshBody& other) const;

	D_COLLISION_API virtual ndBody* CreateObject() const;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent);

	ndVector m_localCentreOfMass;
	ndString m_classConstructor;
	ndWeakPtr<const ndMesh> m_owner;
};

class ndMeshBodyKinematic : public ndMeshBody
{
	public:
	D_COLLISION_API ndMeshBodyKinematic(const ndMesh* const owner);
	D_COLLISION_API ndMeshBodyKinematic(const ndMeshBodyKinematic& other);

	D_COLLISION_API virtual ndMeshBody* Duplicate() const override;
	D_COLLISION_API virtual bool operator==(const ndMeshBody& other) const override;

	D_COLLISION_API virtual ndBody* CreateObject() const override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndMeshShapeInstance m_shapeInstance;
	ndVector m_invMass;
	ndVector m_inertiaPrincipalAxis;
	ndFloat32 m_maxAngleStep;
	ndFloat32 m_maxLinearStep;
	ndFloat32 m_massVolumeWeigh;
};

class ndMeshCollidingPair : public ndClassAlloc
{
	public:
	D_COLLISION_API ndMeshCollidingPair();
	D_COLLISION_API ndMeshCollidingPair(const ndMesh* const node0, const ndMesh* const node1);
	D_COLLISION_API virtual ~ndMeshCollidingPair();

	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent);

	ndWeakPtr<const ndMesh> m_childNode;
	ndWeakPtr<const ndMesh> m_parentNode;
};

class ndMeshJoint : public ndClassAlloc
{
	public:
	class ndAxis
	{
		public:
		ndAxis()
			:m_springK(ndFloat32(0.0f))
			,m_damperC(ndFloat32(0.0f))
			,m_minLimit(ndFloat32(-1.0e10f))
			,m_maxLimit(ndFloat32(1.0e10f))
			,m_springDamperRegularizer(ndFloat32(0.0f))
			,m_limitState(0)
		{
		}

		bool operator==(const ndAxis& other) const
		{
			bool test = (m_springK == other.m_springK);
			test = test && (m_damperC == other.m_damperC);
			test = test && (m_minLimit == other.m_minLimit);
			test = test && (m_maxLimit == other.m_maxLimit);
			test = test && (m_springDamperRegularizer == other.m_springDamperRegularizer);
			test = test && (m_limitState == other.m_limitState);
			return test;
		}

		ndFloat32 m_springK;
		ndFloat32 m_damperC;
		ndFloat32 m_minLimit;
		ndFloat32 m_maxLimit;
		ndFloat32 m_springDamperRegularizer;
		bool m_limitState;
	};

	D_COLLISION_API ndMeshJoint(const ndMeshJoint& other);
	D_COLLISION_API ndMeshJoint(const ndMesh* const owner);
	D_COLLISION_API ndMeshJoint(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);
	D_COLLISION_API virtual ~ndMeshJoint();

	D_COLLISION_API virtual ndMeshJoint* Duplicate() const;
	D_COLLISION_API virtual void DuplicateFixDependencies(const ndMesh* const otherRoot);

	D_COLLISION_API const ndMesh* GetSurrogateParent() const;
	D_COLLISION_API void SetSurrogateParent(const ndMesh* const surrodateParent);

	D_COLLISION_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent);

	D_COLLISION_API virtual void ApplyTransform(const ndMatrix& tranform);

	D_COLLISION_API virtual bool operator==(const ndMeshJoint& other) const;

	ndMatrix m_localFrame0;
	ndMatrix m_localFrame1;
	ndString m_constructor;
	ndWeakPtr<const ndMesh> m_owner;
	ndWeakPtr<const ndMesh> m_surrogateParent;
};

class ndMeshTransformModifier : public ndClassAlloc
{
	public:
	D_COLLISION_API ndMeshTransformModifier(const ndMesh* const owner, const ndMesh* const target);
	D_COLLISION_API ndMeshTransformModifier(const ndMeshTransformModifier& other);
	D_COLLISION_API virtual ~ndMeshTransformModifier();

	D_COLLISION_API virtual ndMeshTransformModifier* Duplicate() const;
	D_COLLISION_API virtual void DuplicateFixDependencies(const ndMesh* const otherRoot);

	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent);
	D_COLLISION_API virtual ndFixSizeArray<const ndMesh*, 256> GetAffectedNodes() const;

	D_COLLISION_API virtual bool operator==(const ndMeshTransformModifier& other) const;

	ndWeakPtr<const ndMesh> m_owner;
	ndWeakPtr<const ndMesh> m_target;

	D_BASE_CLASS_REFLECTION(ndMeshTransformModifier);
};

class ndMeshTransformModifierLookAt : public ndMeshTransformModifier
{
	public:
	D_COLLISION_API ndMeshTransformModifierLookAt(const ndMesh* const owner, const ndMesh* const target);
	D_COLLISION_API ndMeshTransformModifierLookAt(const ndMeshTransformModifierLookAt& other);
	D_COLLISION_API virtual ndMeshTransformModifier* Duplicate() const override;
	D_COLLISION_API virtual bool operator==(const ndMeshTransformModifier& other) const override;

	D_CLASS_REFLECTION(ndMeshTransformModifierLookAt, ndMeshTransformModifier)
};

class ndMeshTransformModifierTwoLinksIK : public ndMeshTransformModifier
{
	public:
	D_COLLISION_API ndMeshTransformModifierTwoLinksIK(const ndMesh* const owner, const ndMesh* const target);
	D_COLLISION_API ndMeshTransformModifierTwoLinksIK(const ndMeshTransformModifierTwoLinksIK& other);
	D_COLLISION_API virtual ndMeshTransformModifier* Duplicate() const override;

	D_COLLISION_API virtual void DuplicateFixDependencies(const ndMesh* const otherRoot) override;
	D_COLLISION_API virtual ndFixSizeArray<const ndMesh*, 256> GetAffectedNodes() const override;
	D_COLLISION_API virtual bool operator==(const ndMeshTransformModifier& other) const override;

	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	D_CLASS_REFLECTION(ndMeshTransformModifierTwoLinksIK, ndMeshTransformModifier)

	ndWeakPtr<const ndMesh> m_childLink;
	ndFloat32 m_solutionSign;
};

class ndMeshTransformModifierUserDefined : public ndMeshTransformModifier
{
	public:
	D_COLLISION_API ndMeshTransformModifierUserDefined(const ndMesh* const owner);
	D_COLLISION_API ndMeshTransformModifierUserDefined(const ndMeshTransformModifierUserDefined& other);
	D_COLLISION_API virtual ndMeshTransformModifier* Duplicate() const override;

	D_COLLISION_API virtual bool operator==(const ndMeshTransformModifier& other) const override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	D_CLASS_REFLECTION(ndMeshTransformModifierUserDefined, ndMeshTransformModifier)

	ndString m_userConstructor;
};

#endif
