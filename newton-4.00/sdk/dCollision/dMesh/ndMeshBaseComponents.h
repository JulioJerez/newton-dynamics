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

class ndMeshCollisionShape : public ndClassAlloc
{
	public:
	D_COLLISION_API ndMeshCollisionShape(const char* const constructor);
	D_COLLISION_API virtual ~ndMeshCollisionShape();

	D_COLLISION_API virtual ndShape* CreateObject() const = 0;
	D_COLLISION_API virtual void ApplyScale(ndFloat32 scale) = 0;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const = 0;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) = 0;

	ndString m_constructor;
};

class ndMeshCollisionShapeNull : public ndMeshCollisionShape
{
	public:
	D_COLLISION_API ndMeshCollisionShapeNull();
	D_COLLISION_API virtual ndShape* CreateObject() const override;
	D_COLLISION_API virtual void ApplyScale(ndFloat32 scale) override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
};

class ndMeshCollisionShapeSphere : public ndMeshCollisionShape
{
	public:
	D_COLLISION_API ndMeshCollisionShapeSphere();
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
	D_COLLISION_API virtual ndShape* CreateObject() const override;
	D_COLLISION_API virtual void ApplyScale(ndFloat32 scale) override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndFloat32 m_height;
	ndFloat32 m_radius;
};

class ndMeshCollisionShapeConvexHull : public ndMeshCollisionShape
{
	public:
	D_COLLISION_API ndMeshCollisionShapeConvexHull();
	D_COLLISION_API virtual ndShape* CreateObject() const override;
	D_COLLISION_API virtual void ApplyScale(ndFloat32 scale) override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndArray<ndVector> m_points;
	ndInt32 m_maxPointCount;
};

class ndMeshCollisionShapeCompound : public ndMeshCollisionShape
{
	public:
	D_COLLISION_API ndMeshCollisionShapeCompound();
	D_COLLISION_API virtual ndShape* CreateObject() const override;
	D_COLLISION_API virtual void ApplyScale(ndFloat32 scale) override;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndList<ndSharedPtr<ndMeshShapeInstance>> m_subShapes;
};

class ndMeshShapeInstance : public ndClassAlloc
{
	public:
	D_COLLISION_API ndMeshShapeInstance();
	D_COLLISION_API ndMeshShapeInstance(const ndShapeInstance& instance);

	D_COLLISION_API ndShapeInstance* CreateObject() const;
	D_COLLISION_API void ApplyScale(const ndMatrix& scaleMatrix);
	D_COLLISION_API void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_COLLISION_API void DeserializeFromXml(const nd::TiXmlElement* const parent);

	ndMatrix m_localMatrix;
	ndMatrix m_alignmentMatrix;
	ndVector m_scale;
	ndSharedPtr<ndMeshCollisionShape> m_shape;
};

class ndMeshBody : public ndClassAlloc
{
	public:
	D_COLLISION_API ndMeshBody(const ndMesh* const owner);
	D_COLLISION_API virtual ~ndMeshBody();

	D_COLLISION_API virtual ndBody* CreateObject() const;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent);

	ndVector m_veloc;
	ndVector m_omega;
	ndVector m_localCentreOfMass;
	ndString m_classConstructor;
	ndWeakPtr<const ndMesh> m_owner;
};

class ndMeshBodyKinematic : public ndMeshBody
{
	public:
	D_COLLISION_API ndMeshBodyKinematic(const ndMesh* const owner);

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

		ndFloat32 m_springK;
		ndFloat32 m_damperC;
		ndFloat32 m_minLimit;
		ndFloat32 m_maxLimit;
		ndFloat32 m_springDamperRegularizer;
		ndInt8 m_limitState;
	};

	D_COLLISION_API ndMeshJoint(const ndMesh* const owner);
	D_COLLISION_API ndMeshJoint(const ndMesh* const owner, const ndJointBilateralConstraint* const joint);
	D_COLLISION_API virtual ~ndMeshJoint();

	D_COLLISION_API const ndMesh* GetSurrogateParent() const;
	D_COLLISION_API void SetSurrogateParent(const ndMesh* const surrodateParent);

	D_COLLISION_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const;
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent);

	D_COLLISION_API virtual void ApplyTransform(const ndMatrix& tranform);

	ndMatrix m_localFrame0;
	ndMatrix m_localFrame1;
	ndString m_constructor;
	ndWeakPtr<const ndMesh> m_owner;
	ndWeakPtr<const ndMesh> m_surrogateParent;
};

#endif

