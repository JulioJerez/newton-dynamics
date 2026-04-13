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

#include "ndNewtonStdafx.h"

class ndMeshBody;
class ndMeshJoint;
class ndMeshEffect;
class ndShapeInstance;
class ndMeshShapeInstance;

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

	D_NEWTON_API ndMesh();
	D_NEWTON_API ndMesh(const ndMesh& src);
	D_NEWTON_API ndMesh(const ndShapeInstance& src, ndUvMapingMode mapping = m_box);

	D_NEWTON_API virtual ~ndMesh();
	D_NEWTON_API ndMesh* CreateClone() const;

	ndMatrix GetMatrix() const;
	void SetMatrix(const ndMatrix& matrix);

	ndMatrix GetGeometryMatrix() const;
	void SetGeometryMatrix(const ndMatrix& matrix);

	D_NEWTON_API void AddChild(const ndSharedPtr<ndMesh>& child);
	D_NEWTON_API void RemoveChild(const ndSharedPtr<ndMesh>& child);

	D_NEWTON_API ndMesh* GetParent();
	D_NEWTON_API const ndMesh* GetParent() const;

	D_NEWTON_API ndList<ndSharedPtr<ndMesh>>& GetChildren();
	D_NEWTON_API const ndList<ndSharedPtr<ndMesh>>& GetChildren() const;

	D_NEWTON_API ndMesh* IteratorFirst();
	D_NEWTON_API ndMesh* IteratorNext(const ndMesh* const root);

	D_NEWTON_API ndMesh* FindByName(const ndString& name) const;
	D_NEWTON_API ndMesh* FindByClosestMatch(const ndString& name) const;

	D_NEWTON_API ndSharedPtr<ndMesh> GetSharedPtr() const;

	D_NEWTON_API ndSharedPtr<ndMeshEffect>& GetMesh();
	D_NEWTON_API const ndSharedPtr<ndMeshEffect>& GetMesh() const;
	D_NEWTON_API void SetMesh(const ndSharedPtr<ndMeshEffect>& mesh);

	D_NEWTON_API ndSharedPtr<ndMeshBody>& GetRigidBody();
	D_NEWTON_API const ndSharedPtr<ndMeshBody>& GetRigidBody() const;
	D_NEWTON_API void SetRigidBody(ndSharedPtr<ndMeshBody>& rigidBody);

	D_NEWTON_API ndSharedPtr<ndMeshJoint>& GetJoint();
	D_NEWTON_API const ndSharedPtr<ndMeshJoint>& GetJoint() const;
	D_NEWTON_API void SetJoint(const ndSharedPtr<ndMeshJoint>& primitive);

	D_NEWTON_API const ndString& GetName() const;
	D_NEWTON_API void SetName(const ndString& name);

	D_NEWTON_API ndNodeType GetNodeType() const;
	D_NEWTON_API void SetNodeType(ndNodeType type);

	D_NEWTON_API ndVector GetBoneTarget() const;
	D_NEWTON_API void SetBoneTarget(const ndVector& target);

	D_NEWTON_API ndCurve& GetScaleCurve();
	D_NEWTON_API ndCurve& GetPositCurve();
	D_NEWTON_API ndCurve& GetRotationCurve();

	D_NEWTON_API const ndCurve& GetScaleCurve() const;
	D_NEWTON_API const ndCurve& GetPositCurve() const;
	D_NEWTON_API const ndCurve& GetRotationCurve() const;

	D_NEWTON_API void ApplyTransform(const ndMatrix& transform);
	D_NEWTON_API ndMatrix CalculateGlobalMatrix(ndMesh* const parent = nullptr) const;
	D_NEWTON_API void CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const;

	D_NEWTON_API ndSharedPtr<ndShapeInstance> CreateCollision();
	D_NEWTON_API ndSharedPtr<ndJointBilateralConstraint> CreateJoint();
	D_NEWTON_API ndSharedPtr<ndShapeInstance> CreateCollisionFromChildren();

	D_NEWTON_API ndSharedPtr<ndShapeInstance> CreateCollisionBox();
	D_NEWTON_API ndSharedPtr<ndShapeInstance> CreateCollisionTire();
	D_NEWTON_API ndSharedPtr<ndShapeInstance> CreateCollisionNull();
	D_NEWTON_API ndSharedPtr<ndShapeInstance> CreateCollisionSphere();
	D_NEWTON_API ndSharedPtr<ndShapeInstance> CreateCollisionConvex();
	D_NEWTON_API ndSharedPtr<ndShapeInstance> CreateCollisionCapsule();
	D_NEWTON_API ndSharedPtr<ndShapeInstance> CreateCollisionCylinder();
	D_NEWTON_API ndSharedPtr<ndShapeInstance> CreateCollisionChamferCylinder();
	D_NEWTON_API ndSharedPtr<ndShapeInstance> CreateCollisionTree(bool optimize = true);
	D_NEWTON_API ndSharedPtr<ndShapeInstance> CreateCollisionConvexApproximation(bool lowDetail = false);

	template <typename Function>
	void NodeIterator(Function func);

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
	ndSharedPtr<ndMeshJoint> m_joint;
	ndSharedPtr<ndMeshBody> m_rigidBody;
	ndList<ndSharedPtr<ndMesh>> m_children;
	ndSharedPtr<ndMeshLoopJoint> m_loopJoint;
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

template <typename Function>
void ndMesh::NodeIterator(Function func)
{
	ndFixSizeArray<ndMesh*, 1024> stack;
	stack.PushBack(this);
	while (stack.GetCount())
	{
		ndMesh* const node = stack.Pop();
		func(node);
		for (ndList<ndSharedPtr<ndMesh>>::ndNode* childNode = node->m_children.GetFirst(); childNode; childNode = childNode->GetNext())
		{
			ndMesh* const child = *childNode->GetInfo();
			stack.PushBack(child);
		}
	}
}

#endif

