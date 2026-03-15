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

#ifndef __ND_MESH_H__
#define __ND_MESH_H__

//#include "ndCore.h"
#include "ndCollisionStdafx.h"


//class ndBody;
class ndMeshBody;
class ndMeshEffect;
class ndShapeInstance;
class ndMeshShapeInstance;

#if 0
class ndMeshCollisionShape : public ndClassAlloc
{
	public:
	D_COLLISION_API ndMeshCollisionShape();
	D_COLLISION_API virtual ~ndMeshCollisionShape();
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const = 0;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) = 0;
};

class ndMeshCollisionShapeNull : public ndMeshCollisionShape
{
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
};

class ndMeshCollisionShapeBox : public ndMeshCollisionShape
{
	public:
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndFloat32 m_x;
	ndFloat32 m_y;
	ndFloat32 m_z;
};

class ndMeshCollisionShapeCapsule : public ndMeshCollisionShape
{
	public:
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndFloat32 m_height;
	ndFloat32 m_radius0;
	ndFloat32 m_radius1;
};

class ndMeshShapeInstance
{
	public:
	D_COLLISION_API ndMeshShapeInstance();
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
	D_COLLISION_API ndMeshBody();
	D_COLLISION_API virtual ~ndMeshBody();
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent);

	ndVector m_veloc;
	ndVector m_omega;
	ndVector m_localCentreOfMass;
	ndString m_classConstructor;
};

class ndMeshBodyKinematic : public ndMeshBody
{
	public:
	D_COLLISION_API ndMeshBodyKinematic();
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;

	ndMeshShapeInstance m_shapeInstance;
	ndVector m_invMass;
	ndMatrix m_inertiaPrincipalAxis;
	ndFloat32 m_maxAngleStep;
	ndFloat32 m_maxLinearStep;
};

class ndMeshBodyDynamic : public ndMeshBodyKinematic
{
	public:
	D_COLLISION_API ndMeshBodyDynamic();
	D_COLLISION_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_COLLISION_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	ndVector m_intrinsicDamping;
};
#endif

class ndMesh : public ndClassAlloc
{
	public:
	enum ndNodeType
	{
		m_node,
		m_bone,
		m_boneEnd,
		m_collisionShape,
	};
	
	class ndCurveValue
	{
		public:
		ndReal m_x;
		ndReal m_y;
		ndReal m_z;
		ndReal m_time;
	};

	enum ndUvMapingMode
	{
		m_box,
		m_capsule,
		m_spherical,
		m_cylindrical
	};

	class ndCurve: public ndList<ndCurveValue>
	{
		public:
		ndCurve()
			:ndList<ndCurveValue>()
			,m_lenght(ndFloat32 (0.0f))
		{
		}
		ndReal m_lenght;
	};

	D_COLLISION_API ndMesh();
	D_COLLISION_API ndMesh(const ndMesh& src);
	D_COLLISION_API ndMesh(const ndShapeInstance& src, ndUvMapingMode mapping = m_box);

	D_COLLISION_API virtual ~ndMesh();
	D_COLLISION_API ndMesh* CreateClone() const;

	ndMatrix GetMatrix() const;
	void SetMatrix(const ndMatrix& matrix);

	ndMatrix GetGeometryMatrix() const;
	void SetGeometryMatrix(const ndMatrix& matrix);

	D_COLLISION_API void AddChild(const ndSharedPtr<ndMesh>& child);
	D_COLLISION_API void RemoveChild(const ndSharedPtr<ndMesh>& child);

	D_COLLISION_API ndMesh* GetParent();
	D_COLLISION_API const ndMesh* GetParent() const;

	D_COLLISION_API ndList<ndSharedPtr<ndMesh>>& GetChildren();
	D_COLLISION_API const ndList<ndSharedPtr<ndMesh>>& GetChildren() const;

	D_COLLISION_API ndMesh* IteratorFirst();
	D_COLLISION_API ndMesh* IteratorNext(const ndMesh* const root);

	D_COLLISION_API ndMesh* FindByName(const ndString& name) const;
	D_COLLISION_API ndMesh* FindByClosestMatch(const ndString& name) const;

	D_COLLISION_API ndSharedPtr<ndMesh> GetSharedPtr() const;

	D_COLLISION_API ndSharedPtr<ndMeshEffect>& GetMesh();
	D_COLLISION_API const ndSharedPtr<ndMeshEffect>& GetMesh() const;
	D_COLLISION_API void SetMesh(const ndSharedPtr<ndMeshEffect>& mesh);

	D_COLLISION_API ndSharedPtr<ndMeshShapeInstance>& GetPrimitive();
	D_COLLISION_API const ndSharedPtr<ndMeshShapeInstance>& GetPrimitive() const;
	D_COLLISION_API void SetPrimitive(const ndSharedPtr<ndMeshShapeInstance>& primitive);

	D_COLLISION_API const ndString& GetName() const;
	D_COLLISION_API void SetName(const ndString& name);

	D_COLLISION_API ndNodeType GetNodeType() const;
	D_COLLISION_API void SetNodeType(ndNodeType type);

	D_COLLISION_API ndVector GetBoneTarget() const;
	D_COLLISION_API void SetBoneTarget(const ndVector& target);

	D_COLLISION_API ndCurve& GetScaleCurve();
	D_COLLISION_API ndCurve& GetPositCurve();
	D_COLLISION_API ndCurve& GetRotationCurve();

	D_COLLISION_API const ndCurve& GetScaleCurve() const;
	D_COLLISION_API const ndCurve& GetPositCurve() const;
	D_COLLISION_API const ndCurve& GetRotationCurve() const;

	D_COLLISION_API void ApplyTransform(const ndMatrix& transform);
	D_COLLISION_API ndMatrix CalculateGlobalMatrix(ndMesh* const parent = nullptr) const;

	D_COLLISION_API ndSharedPtr<ndMeshBody> GetRigidBody() const;
	D_COLLISION_API void SetRigidBody(ndSharedPtr<ndMeshBody>& rigidBody);

	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollision();
	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionFromChildren();

	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionBox();
	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionTire();
	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionSphere();
	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionConvex();
	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionCapsule();
	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionChamferCylinder();
	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionTree(bool optimize = true);
	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionConvexApproximation(bool lowDetail = false);

	protected:
	ndMatrix CalculateLocalMatrix(ndVector& size) const;

	ndMatrix m_matrix;
	ndMatrix m_geometryMatrix;

	ndString m_name;
	ndCurve m_scale;
	ndCurve m_posit;
	ndCurve m_rotation;
	ndWeakPtr<ndMesh> m_parent;
	ndSharedPtr<ndMeshEffect> m_mesh;
	ndSharedPtr<ndMeshBody> m_rigidBody;
	ndSharedPtr<ndMeshShapeInstance> m_meshPrimitive;
	ndList<ndSharedPtr<ndMesh>> m_children;
	ndList<ndSharedPtr<ndMesh>>::ndNode* m_selfChildNode;
	ndVector m_boneTarget;
	ndNodeType m_type;

	friend class ndMeshFile;
	friend class ndMeshLoader;
};

inline ndMatrix ndMesh::GetMatrix() const
{
	return m_matrix;
}

inline void ndMesh::SetMatrix(const ndMatrix& matrix)
{
	m_matrix = matrix;
}

inline ndMatrix ndMesh::GetGeometryMatrix() const
{
	return m_geometryMatrix;
}

inline void ndMesh::SetGeometryMatrix(const ndMatrix& matrix)
{
	m_geometryMatrix = matrix;
}

#endif

