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
class ndMeshLoopJoint;
class ndCollidingPairs;
class ndMeshShapeInstance;
class ndCloseLoopConstraints;

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

		ndCurve(const ndCurve& src)
			:ndList<ndCurveValue>()
			,m_lenght(src.m_lenght)
		{
			for (ndNode* ptr = src.GetFirst(); ptr; ptr = ptr->GetNext())
			{
				Append(ptr->GetInfo());
			}
		}

		ndReal m_lenght;
	};

	D_NEWTON_API ndMesh();
	D_NEWTON_API ndMesh(const ndMesh& src);
	D_NEWTON_API ndMesh(const ndShapeInstance& src, ndUvMapingMode mapping = m_box);

	D_NEWTON_API virtual ~ndMesh();
	D_NEWTON_API virtual ndMesh* CreateClone() const;
	D_NEWTON_API virtual void CreateCloneFixDependencies();

	D_NEWTON_API virtual bool operator==(const ndMesh& other) const;

	D_NEWTON_API ndMatrix GetMatrix() const;
	D_NEWTON_API void SetMatrix(const ndMatrix& matrix);

	D_NEWTON_API ndMatrix GetGeometryMatrix() const;
	D_NEWTON_API void SetGeometryMatrix(const ndMatrix& matrix);

	D_NEWTON_API ndMatrix GetBasePoseMatrix() const;
	D_NEWTON_API void SetBasePoseMatrix(const ndMatrix& matrix);

	D_NEWTON_API void AddChild(const ndSharedPtr<ndMesh>& child);
	D_NEWTON_API void RemoveChild(const ndSharedPtr<ndMesh>& child);

	D_NEWTON_API ndMesh* GetParent();
	D_NEWTON_API const ndMesh* GetParent() const;

	D_NEWTON_API ndList<ndSharedPtr<ndMesh>>& GetChildren();
	D_NEWTON_API const ndList<ndSharedPtr<ndMesh>>& GetChildren() const;

	D_NEWTON_API ndList<ndSharedPtr<ndMeshCustomProperty>>& GetCustomProperties();
	D_NEWTON_API const ndList<ndSharedPtr<ndMeshCustomProperty>>& GetCustomProperties() const;
	D_NEWTON_API ndMeshCustomProperty* GetCustomPropertyByName(const char* const name) const;

	D_NEWTON_API ndMesh* IteratorFirst();
	D_NEWTON_API ndMesh* IteratorNext(const ndMesh* const root);

	D_NEWTON_API ndMesh* FindByName(const ndString& name) const;
	D_NEWTON_API ndMesh* FindByClosestMatch(const ndString& name) const;

	D_NEWTON_API ndMesh* GetRoot();
	D_NEWTON_API const ndMesh* GetRoot() const;
	D_NEWTON_API ndSharedPtr<ndMesh> GetSharedPtr() const;

	D_NEWTON_API ndSharedPtr<ndMeshEffect>& GetGeometry();
	D_NEWTON_API const ndSharedPtr<ndMeshEffect>& GetGeometry() const;
	D_NEWTON_API void SetGeometry(const ndSharedPtr<ndMeshEffect>& mesh);

	D_NEWTON_API ndSharedPtr<ndMeshBody>& GetRigidBody();
	D_NEWTON_API const ndSharedPtr<ndMeshBody>& GetRigidBody() const;
	D_NEWTON_API void SetRigidBody(const ndSharedPtr<ndMeshBody>& rigidBody);

	D_NEWTON_API ndSharedPtr<ndMeshJoint>& GetJoint();
	D_NEWTON_API const ndSharedPtr<ndMeshJoint>& GetJoint() const;
	D_NEWTON_API void SetJoint(const ndSharedPtr<ndMeshJoint>& joint);

	D_NEWTON_API ndSharedPtr<ndMeshTransformModifier>& GetModifier();
	D_NEWTON_API const ndSharedPtr<ndMeshTransformModifier>& GetModifier() const;
	D_NEWTON_API void SetModifier(const ndSharedPtr<ndMeshTransformModifier>& modifier);

	D_NEWTON_API virtual ndMesh* GetAsMesh();
	D_NEWTON_API virtual const ndMesh* GetAsMesh() const;
	D_NEWTON_API virtual ndCollidingPairs* GetAsCollidingPairs();
	D_NEWTON_API virtual const ndCollidingPairs* GetAsCollidingPairs() const;
	D_NEWTON_API virtual ndCloseLoopConstraints* GetAsCloseLoopConstraints();
	D_NEWTON_API virtual const ndCloseLoopConstraints* GetAsCloseLoopConstraints() const;

	D_NEWTON_API ndCloseLoopConstraints* GetLoopJoints();
	D_NEWTON_API const ndCloseLoopConstraints* GetLoopJoints() const;
	D_NEWTON_API void AddLoopJoint(const ndSharedPtr<ndMeshLoopJoint>& joint);

	D_NEWTON_API ndCollidingPairs* GetCollingPairs();
	D_NEWTON_API const ndCollidingPairs* GetCollingPairs() const;
	D_NEWTON_API void SetCollidingSubSelection(const ndMesh* const node0, const ndMesh* const node1);

	D_NEWTON_API const ndString& GetName() const;
	D_NEWTON_API void SetName(const ndString& name);

	D_NEWTON_API ndNodeType GetNodeType() const;
	D_NEWTON_API void SetNodeType(ndNodeType type);

	D_NEWTON_API bool GetVisibility() const;
	D_NEWTON_API void SetVisibility(bool flag);

	D_NEWTON_API ndInt32 GetToolFlags() const;
	D_NEWTON_API void SetToolFlags(ndInt32 flags);

	D_NEWTON_API ndVector GetBoneTarget() const;
	D_NEWTON_API void SetBoneTarget(const ndVector& target);

	D_NEWTON_API ndCurve& GetScaleCurve();
	D_NEWTON_API ndCurve& GetPositCurve();
	D_NEWTON_API ndCurve& GetRotationCurve();

	D_NEWTON_API const ndCurve& GetScaleCurve() const;
	D_NEWTON_API const ndCurve& GetPositCurve() const;
	D_NEWTON_API const ndCurve& GetRotationCurve() const;

	D_NEWTON_API void ApplyTransform(const ndMatrix& transform);
	D_NEWTON_API void ApplyBonesRotation(const ndMatrix& rotation);
	D_NEWTON_API void ApplyPivotsRotation(const ndMatrix& rotation);
	D_NEWTON_API void ApplyCoordinateRotation(const ndMatrix& rotation);
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

	D_NEWTON_API ndSharedPtr<ndMeshJoint> LoadJoint(const nd::TiXmlElement* const xmlJoint) const;
	D_NEWTON_API ndSharedPtr<ndMeshTransformModifier> LoadModifier(const nd::TiXmlElement* const xmlModifier) const;

	template <typename Function>
	void NodeIterator(Function func);

	protected:
	ndMatrix CalculateLocalMatrix(ndVector& size) const;

	ndMatrix m_matrix;
	ndMatrix m_basePoseMatrix;
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
	ndList<ndSharedPtr<ndMesh>>::ndNode* m_selfChildNode;
	ndSharedPtr<ndMeshTransformModifier> m_transformModifier;
	ndList<ndSharedPtr<ndMeshCustomProperty>> m_customProperties;
	ndVector m_boneTarget;
	ndNodeType m_type;
	ndInt32 m_assetToolFlags;
	bool m_isVisible;

	friend class ndMeshFile;
	friend class ndMeshLoader;
};

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

class ndCloseLoopConstraints: public ndMesh
{
	public:
	D_NEWTON_API ndCloseLoopConstraints();
	D_NEWTON_API ndCloseLoopConstraints(const ndCloseLoopConstraints& src);

	D_NEWTON_API virtual ndMesh* GetAsMesh();
	D_NEWTON_API virtual const ndMesh* GetAsMesh() const;

	D_NEWTON_API virtual ndCloseLoopConstraints* GetAsCloseLoopConstraints() override;
	D_NEWTON_API virtual const ndCloseLoopConstraints* GetAsCloseLoopConstraints() const override;

	D_NEWTON_API void UpdateNames();

	D_NEWTON_API virtual void CreateCloneFixDependencies() override;
	D_NEWTON_API virtual ndMesh* CreateClone() const override;
	D_NEWTON_API virtual bool operator==(const ndMesh& other) const override;

	ndList<ndSharedPtr<ndMeshLoopJoint>> m_loopJoints;
};

class ndCollidingPairs : public ndMesh
{
	public:
	D_NEWTON_API ndCollidingPairs();
	D_NEWTON_API ndCollidingPairs(const ndCollidingPairs& src);

	D_NEWTON_API virtual ndMesh* GetAsMesh();
	D_NEWTON_API virtual const ndMesh* GetAsMesh() const;

	D_NEWTON_API void UpdateNames();
	D_NEWTON_API virtual ndCollidingPairs* GetAsCollidingPairs() override;
	D_NEWTON_API virtual const ndCollidingPairs* GetAsCollidingPairs() const override;

	D_NEWTON_API virtual void CreateCloneFixDependencies() override;
	D_NEWTON_API virtual ndMesh* CreateClone() const override;
	D_NEWTON_API virtual bool operator==(const ndMesh& other) const override;

	ndList<ndSharedPtr<ndMeshCollidingPair>> m_collidingPairs;
};

#endif

