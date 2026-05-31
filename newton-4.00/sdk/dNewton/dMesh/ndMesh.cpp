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
#include "ndCollision.h"
#include "ndJointGear.h"
#include "ndJointPlane.h"
#include "ndMeshEffect.h"
#include "ndMeshLoader.h"
#include "ndJointHinge.h"
#include "ndJointWheel.h"
#include "ndJointSlider.h"
#include "ndJointRoller.h"
#include "ndBodyDynamic.h"
#include "ndJointFix6dof.h"
#include "ndJointSpherical.h"
#include "ndMeshComponents.h"
#include "ndJointDoubleHinge.h"
#include "ndIkSwivelPositionEffector.h"
#include "ndMultiBodyVehicleDifferentialAxle.h"

ndMesh::ndMesh()
	:ndClassAlloc()
	,m_matrix(ndGetIdentityMatrix())
	,m_basePoseMatrix(ndGetIdentityMatrix())
	,m_geometryMatrix(ndGetIdentityMatrix())
	,m_name()
	,m_scale()
	,m_posit()
	,m_rotation()
	,m_parent(nullptr)
	,m_mesh(nullptr)
	,m_joint(nullptr)
	,m_rigidBody(nullptr)
	,m_children()
	,m_selfChildNode(nullptr)
	,m_transformModifier(nullptr)
	,m_customProperties()
	,m_boneTarget(ndVector::m_wOne)
	,m_type(m_node)
{
}

ndMesh::ndMesh(const ndShapeInstance& shape, ndUvMapingMode mapping)
	:ndClassAlloc()
	,m_matrix(ndGetIdentityMatrix())
	,m_basePoseMatrix(ndGetIdentityMatrix())
	,m_geometryMatrix(ndGetIdentityMatrix())
	,m_name("node")
	,m_scale()
	,m_posit()
	,m_rotation()
	,m_parent(nullptr)
	,m_mesh(new ndMeshEffect(shape))
	,m_selfChildNode(nullptr)
	,m_transformModifier(nullptr)
	,m_customProperties()
	,m_boneTarget(ndVector::m_wOne)
	,m_type(m_node)
{
	switch (mapping)
	{
		//case m_capsule:
		//case m_spherical:
		//{
		//	ndMatrix flipMatrix(ndGetIdentityMatrix());
		//	flipMatrix[0][0] = ndFloat32(-1.0f);
		//	ndMatrix aligmentUV(flipMatrix * descriptor.m_uvMatrix);
		//	mesh.SphericalMapping(textureId, aligmentUV);
		//	break;
		//}
		//
		//case m_cylindrical:
		//{
		//	ndMatrix flipMatrix(ndGetIdentityMatrix());
		//	flipMatrix[0][0] = ndFloat32(-1.0f);
		//	ndMatrix aligmentUV(flipMatrix * descriptor.m_uvMatrix);
		//	mesh.CylindricalMapping(textureId, aligmentUV);
		//	break;
		//}
		//
		//case m_box:
		//{
		//	if (descriptor.m_stretchMaping)
		//	{
		//		mesh.BoxMapping(textureId, textureId, textureId, descriptor.m_uvMatrix);
		//	}
		//	else
		//	{
		//		mesh.UniformBoxMapping(textureId, descriptor.m_uvMatrix);
		//	}
		//	break;
		//}
		case m_box:
		default:
		{
			m_mesh->UniformBoxMapping(0, ndGetIdentityMatrix());
		}
	}
}

ndMesh::ndMesh(const ndMesh& src)
	:ndClassAlloc()
	,m_matrix(src.m_matrix)
	,m_basePoseMatrix(src.m_basePoseMatrix)
	,m_geometryMatrix(src.m_geometryMatrix)
	,m_name(src.m_name)
	,m_scale(src.m_scale)
	,m_posit(src.m_posit)
	,m_rotation(src.m_rotation)
	,m_parent(nullptr)
	,m_mesh(src.m_mesh ? ndSharedPtr<ndMeshEffect>(new ndMeshEffect (**src.m_mesh)) : ndSharedPtr<ndMeshEffect>(nullptr))
	,m_joint(src.m_joint ? ndSharedPtr<ndMeshJoint>(src.m_joint->Duplicate()) : ndSharedPtr<ndMeshJoint>(nullptr))
	,m_rigidBody(src.m_rigidBody ? ndSharedPtr<ndMeshBody>(src.m_rigidBody->Duplicate()) : ndSharedPtr<ndMeshBody>(nullptr))
	,m_selfChildNode(nullptr)
	,m_transformModifier(src.m_transformModifier ? ndSharedPtr<ndMeshTransformModifier>(src.m_transformModifier->Duplicate()) : ndSharedPtr<ndMeshTransformModifier>(nullptr))
	,m_customProperties()
	,m_boneTarget(src.m_boneTarget)
	,m_type(src.m_type)
{
	if (src.m_customProperties)
	{
		for (ndList<ndSharedPtr<ndMeshCustomProperty>>::ndNode* node = src.m_customProperties.GetFirst(); node; node = node->GetNext())
		{
			ndMeshCustomProperty* const property = *node->GetInfo();
			m_customProperties.Append(ndSharedPtr<ndMeshCustomProperty>(property->Duplicate()));
		}
	}

	for (ndList<ndSharedPtr<ndMesh>>::ndNode* ptr = src.GetChildren().GetFirst(); ptr; ptr = ptr->GetNext())
	{
		const ndSharedPtr<ndMesh>& child = ptr->GetInfo();
		ndSharedPtr<ndMesh> childMesh (child->CreateClone());
		AddChild(childMesh);
	}
}

ndMesh::~ndMesh()
{
}

ndMatrix ndMesh::GetMatrix() const
{
	return m_matrix;
}

void ndMesh::SetMatrix(const ndMatrix& matrix)
{
	m_matrix = matrix;
}

ndMatrix ndMesh::GetGeometryMatrix() const
{
	return m_geometryMatrix;
}

void ndMesh::SetGeometryMatrix(const ndMatrix& matrix)
{
	m_geometryMatrix = matrix;
}

ndMatrix ndMesh::GetBasePoseMatrix() const
{
	return m_basePoseMatrix;
}

void ndMesh::SetBasePoseMatrix(const ndMatrix& matrix)
{
	m_basePoseMatrix = matrix;
}

ndMesh* ndMesh::GetAsMesh()
{
	return this;
}
const ndMesh* ndMesh::GetAsMesh() const
{
	return this;
}

ndCloseLoopConstraints* ndMesh::GetAsCloseLoopConstraints()
{
	return nullptr;
}

const ndCloseLoopConstraints* ndMesh::GetAsCloseLoopConstraints() const
{
	return nullptr;
}

ndCollidingPairs* ndMesh::GetAsCollidingPairs()
{
	return nullptr;
}

const ndCollidingPairs* ndMesh::GetAsCollidingPairs() const
{
	return nullptr;
}

void ndMesh::AddChild(const ndSharedPtr<ndMesh>& child)
{
	ndAssert(!child->m_parent);
	child->m_parent = this;
	child->m_selfChildNode = m_children.Append(child);
}

void ndMesh::RemoveChild(const ndSharedPtr<ndMesh>& child)
{
	ndAssert(child->m_selfChildNode);
	ndAssert(child->m_parent && (child->m_parent == this));

	ndList<ndSharedPtr<ndMesh>>::ndNode* const node = child->m_selfChildNode;
	child->m_parent = nullptr;
	child->m_selfChildNode = nullptr;

	m_children.Remove(node);
}

ndMesh* ndMesh::GetParent()
{
	return *m_parent;
}

const ndMesh* ndMesh::GetParent() const
{
	return *m_parent;
}

ndList<ndSharedPtr<ndMesh>>& ndMesh::GetChildren()
{
	return m_children;
}

const ndList<ndSharedPtr<ndMesh>>& ndMesh::GetChildren() const
{
	return m_children;
}

ndList<ndSharedPtr<ndMeshCustomProperty>>& ndMesh::GetCustomProperties()
{
	return m_customProperties;
}

const ndList<ndSharedPtr<ndMeshCustomProperty>>& ndMesh::GetCustomProperties() const
{
	return m_customProperties;
}


ndMesh::ndNodeType ndMesh::GetNodeType() const
{
	return m_type;
}

void ndMesh::SetNodeType(ndNodeType type)
{
	m_type = type;
}

ndVector ndMesh::GetBoneTarget() const
{
	return m_boneTarget;
}

void ndMesh::SetBoneTarget(const ndVector& target)
{
	m_boneTarget = target;
	m_boneTarget.m_w = ndFloat32(1.0f);
}

const ndString& ndMesh::GetName() const
{
	return m_name;
}

ndMesh::ndCurve& ndMesh::GetScaleCurve()
{
	return m_scale;
}

const ndMesh::ndCurve& ndMesh::GetScaleCurve() const
{
	return m_scale;
}

ndMesh::ndCurve& ndMesh::GetPositCurve()
{
	return m_posit;
}

const ndMesh::ndCurve& ndMesh::GetPositCurve() const
{
	return m_posit;
}

ndMesh::ndCurve& ndMesh::GetRotationCurve()
{
	return m_rotation;
}

const ndMesh::ndCurve& ndMesh::GetRotationCurve() const
{
	return m_rotation;
}

void ndMesh::SetName(const ndString& name)
{
	m_name = name;
}

void ndMesh::CreateCloneFixDependencies()
{
	const ndMesh* const myRoot = GetRoot();

	if (m_rigidBody)
	{
		m_rigidBody->DuplicateFixDependencies(myRoot);
	}

	if (m_joint)
	{
		m_joint->DuplicateFixDependencies(myRoot);
	}

	if (m_transformModifier)
	{
		m_transformModifier->DuplicateFixDependencies(myRoot);
	}

	for (ndList<ndSharedPtr<ndMesh>>::ndNode* ptr = GetChildren().GetFirst(); ptr; ptr = ptr->GetNext())
	{
		ndMesh* const child = *ptr->GetInfo();
		child->CreateCloneFixDependencies();
	}
}

ndMesh* ndMesh::CreateClone() const
{
	ndMesh* const copy = new ndMesh(*this);
	if (!m_parent)
	{
		copy->CreateCloneFixDependencies();
	}

	return copy;
}

bool ndMesh::operator==(const ndMesh& other) const
{
	bool test = m_name == other.m_name;
	test = test && (m_matrix * other.m_matrix.OrthoInverse()).TestIdentity();
	test = test && (m_geometryMatrix * other.m_geometryMatrix.OrthoInverse()).TestIdentity();
	test = test && (m_type * other.m_type);
	test = test && (m_parent * other.m_parent);

	const ndVector target(m_boneTarget - other.m_boneTarget);
	test = test && (target.DotProduct(target).GetScalar() < ndFloat32(1.0e-6f));

	return test;
}

ndSharedPtr<ndMeshEffect>& ndMesh::GetGeometry()
{
	return m_mesh;
}

const ndSharedPtr<ndMeshEffect>& ndMesh::GetGeometry() const
{
	return m_mesh;
}

void ndMesh::SetGeometry(const ndSharedPtr<ndMeshEffect>& mesh)
{
	m_mesh = mesh;
}

ndSharedPtr<ndMeshJoint>& ndMesh::GetJoint()
{
	return m_joint;
}

const ndSharedPtr<ndMeshJoint>& ndMesh::GetJoint() const
{
	return m_joint;
}

void ndMesh::SetJoint(const ndSharedPtr<ndMeshJoint>& joint)
{
	m_joint = joint;
}

ndSharedPtr<ndMeshTransformModifier>& ndMesh::GetModifier()
{
	return m_transformModifier;
}

const ndSharedPtr<ndMeshTransformModifier>& ndMesh::GetModifier() const
{
	return m_transformModifier;
}

void ndMesh::SetModifier(const ndSharedPtr<ndMeshTransformModifier>& modifier)
{
	m_transformModifier = modifier;
}

ndCloseLoopConstraints* ndMesh::GetLoopJoints()
{
	ndMesh* mesh = GetRoot()->FindByName(ND_MESH_CONSTRAINT_LOOPS);
	if (!mesh)
	{
		ndMesh* const rootNode = GetRoot();
		ndAssert(rootNode);
		ndSharedPtr<ndMesh> loops(new ndCloseLoopConstraints());
		loops->m_parent = rootNode;
		loops->m_selfChildNode = rootNode->m_children.Addtop(loops);
		mesh = loops->GetAsCloseLoopConstraints();
	}
	return mesh->GetAsCloseLoopConstraints();
}

const ndCloseLoopConstraints* ndMesh::GetLoopJoints() const 
{
	const ndMesh* mesh = GetRoot()->FindByName(ND_MESH_CONSTRAINT_LOOPS);
	if (!mesh)
	{
		ndMesh* const rootNode = ((ndMesh*)this)->GetRoot();
		ndAssert(rootNode);
		ndSharedPtr<ndMesh> loops(new ndCloseLoopConstraints());
		loops->m_parent = rootNode;
		loops->m_selfChildNode = rootNode->m_children.Addtop(loops);
		mesh = loops->GetAsCloseLoopConstraints();
	}
	return mesh->GetAsCloseLoopConstraints();
}

void ndMesh::AddLoopJoint(const ndSharedPtr<ndMeshLoopJoint>& joint)
{
	ndCloseLoopConstraints* const mesh = GetLoopJoints();
	mesh->m_loopJoints.Append(joint);
}

ndCollidingPairs* ndMesh::GetCollingPairs()
{
	ndMesh* mesh = GetRoot()->FindByName(ND_MESH_COLLIDING_PAIRS);
	if (!mesh)
	{
		ndMesh* const rootNode = GetRoot();
		ndAssert(rootNode);
		ndSharedPtr<ndMesh> pairs(new ndCollidingPairs());
		pairs->m_parent = rootNode;
		pairs->m_selfChildNode = rootNode->m_children.Addtop(pairs);
		mesh = pairs->GetCollingPairs();
	}
	return mesh->GetAsCollidingPairs();
}

const ndCollidingPairs* ndMesh::GetCollingPairs() const
{
	ndMesh* mesh = GetRoot()->FindByName(ND_MESH_COLLIDING_PAIRS);
	if (!mesh)
	{
		ndMesh* const rootNode = ((ndMesh*)this)->GetRoot();
		ndAssert(rootNode);
		ndSharedPtr<ndMesh> pairs(new ndCollidingPairs());
		pairs->m_parent = rootNode;
		pairs->m_selfChildNode = rootNode->m_children.Addtop(pairs);
		mesh = pairs->GetCollingPairs();
	}
	return mesh->GetAsCollidingPairs();
}

void ndMesh::SetCollidingSubSelection(const ndMesh* const node0, const ndMesh* const node1)
{
	ndAssert(node0 != node1);
	ndCollidingPairs* const mesh = GetCollingPairs();
	const ndMesh* const childNode = (node0->m_name < node1->m_name) ? node0 : node1;
	const ndMesh* const parentNode = (node0->m_name < node1->m_name) ? node1 : node0;
	for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* node = mesh->m_collidingPairs.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndMeshCollidingPair> pair = node->GetInfo();
		if ((pair->m_childNode->GetName() == childNode->GetName()) &&
			(pair->m_parentNode->GetName() == parentNode->GetName()))
		{
			return;
		}
	}
	ndSharedPtr<ndMeshCollidingPair> newPair(new ndMeshCollidingPair(node0, node1));
	mesh->m_collidingPairs.Append(newPair);
}

ndSharedPtr<ndMesh> ndMesh::GetSharedPtr() const
{
	if (m_selfChildNode)
	{
		return m_selfChildNode->GetInfo();
	}
	return ndSharedPtr<ndMesh>(nullptr);
}

ndMesh* ndMesh::GetRoot()
{
	ndMesh* self = this;
	while (self->m_parent)
	{
		self = *self->m_parent;
	}
	return self;
}

const ndMesh* ndMesh::GetRoot() const
{
	const ndMesh* self = this;
	while (self->m_parent)
	{
		self = *self->m_parent;
	}
	return self;
}

ndMesh* ndMesh::FindByName(const ndString& name) const
{
	ndMesh* const self = (ndMesh*)this;
	for (ndMesh* node = self->IteratorFirst(); node; node = node->IteratorNext(self))
	{
		if (name.CompareIgnoreCase(node->m_name))
		{
			return node;
		}
	}
	return nullptr;
}

ndMesh* ndMesh::FindByClosestMatch(const ndString& name) const
{
	ndMesh* closestMatch = FindByName(name);
	if (!closestMatch)
	{
		ndString lowerCaseName(name);
		lowerCaseName.ToLower();

		ndMesh* const self = (ndMesh*)this;
		for (ndMesh* node = self->IteratorFirst(); node; node = node->IteratorNext(self))
		{
			ndString nodeName(node->m_name);
			nodeName.ToLower();
			ndInt32 findIndex = nodeName.Find(lowerCaseName);
			if (findIndex != -1)
			{
				return node;
			}
		}

		ndInt32 bestScore = 10000;
		for (ndMesh* node = self->IteratorFirst(); node && bestScore; node = node->IteratorNext(self))
		{
			ndInt32 distance = node->m_name.Distance(name);
			ndAssert(distance >= 0);
			if (distance < bestScore)
			{
				bestScore = distance;
				closestMatch = node;
			}
		}
	}
	return closestMatch;
}

ndMesh* ndMesh::IteratorFirst()
{
	ndMesh* ptr = this;
	while (ptr->m_children.GetCount())
	{
		ptr = *ptr->m_children.GetFirst()->GetInfo();
	}
	return ptr;
}

ndMesh* ndMesh::IteratorNext(const ndMesh* const root)
{
	if (this == root)
	{
		return nullptr;
	}
	if (m_selfChildNode)
	{
		ndList<ndSharedPtr<ndMesh>>::ndNode* next = m_selfChildNode->GetNext();
		if (next)
		{
			if (next->GetInfo()->m_children.GetCount())
			{
				return next->GetInfo()->IteratorFirst();
			}
			return *next->GetInfo();
		}
		return *m_parent;
	}

	return nullptr;
}

ndMatrix ndMesh::CalculateGlobalMatrix(ndMesh* const parent) const
{
	ndMatrix matrix(ndGetIdentityMatrix());
	for (const ndMesh* ptr = this; ptr != parent; ptr = *ptr->m_parent)
	{
		matrix = matrix * ptr->m_matrix;
	}
	return matrix;
}

void ndMesh::ApplyTransform(const ndMatrix& transform)
{
	auto GetKeyframe = [](const ndCurveValue& scale, const ndCurveValue& position, const ndCurveValue& rotation)
	{
		ndMatrix scaleMatrix(ndGetIdentityMatrix());
		scaleMatrix[0][0] = scale.m_x;
		scaleMatrix[1][1] = scale.m_y;
		scaleMatrix[2][2] = scale.m_z;
		ndMatrix matrix(scaleMatrix * ndPitchMatrix(rotation.m_x) * ndYawMatrix(rotation.m_y) * ndRollMatrix(rotation.m_z));
		matrix.m_posit = ndVector(position.m_x, position.m_y, position.m_z, 1.0f);
		return matrix;
	};

	ndFixSizeArray<ndMesh*, 1024> stack;
	ndFixSizeArray<ndMatrix, 1024> matrix;
	ndFixSizeArray<ndMatrix, 1024> basePoseMatrix;

	stack.PushBack(this);
	matrix.PushBack(transform);
	basePoseMatrix.PushBack(transform);
	while (stack.GetCount())
	{
		ndMesh* const node = stack.Pop();
		ndMatrix parentMatrix(matrix.Pop());
		ndMatrix parentBasePoseMatrix(basePoseMatrix.Pop());
		{
			ndVector scale;
			ndMatrix axis;
			ndMatrix orthoMatrix;
			ndMatrix newBasePoseTransform(node->GetBasePoseMatrix() * parentBasePoseMatrix);
			newBasePoseTransform.PolarDecomposition(orthoMatrix, scale, axis);
			node->SetBasePoseMatrix(orthoMatrix);

			ndMatrix scaleMatrix(ndGetIdentityMatrix());
			scaleMatrix[0][0] = scale[0];
			scaleMatrix[1][1] = scale[1];
			scaleMatrix[2][2] = scale[2];
			parentBasePoseMatrix = axis * scaleMatrix;
		}

		ndVector scale;
		ndMatrix axis;
		ndMatrix orthoMatrix;
		ndMatrix newTransform(node->GetMatrix() * parentMatrix);
		newTransform.PolarDecomposition(orthoMatrix, scale, axis);
		node->SetMatrix(orthoMatrix);
		
		ndMatrix scaleMatrix(ndGetIdentityMatrix());
		scaleMatrix[0][0] = scale[0];
		scaleMatrix[1][1] = scale[1];
		scaleMatrix[2][2] = scale[2];
		parentMatrix = axis * scaleMatrix;

		// calculate geomtry transform
		ndMatrix newTransformGeo(node->GetGeometryMatrix() * parentMatrix);

		ndVector scaleGeo;
		ndMatrix axisGeo;
		ndMatrix orthoMatrixGeo;
		newTransformGeo.PolarDecomposition(orthoMatrixGeo, scaleGeo, axisGeo);
		node->SetGeometryMatrix(orthoMatrixGeo);

		ndMatrix scaleMatrixGeo(ndGetIdentityMatrix());
		scaleMatrixGeo[0][0] = scaleGeo[0];
		scaleMatrixGeo[1][1] = scaleGeo[1];
		scaleMatrixGeo[2][2] = scaleGeo[2];
		newTransformGeo = axisGeo * scaleMatrixGeo;

		node->SetBoneTarget(newTransformGeo.TransformVector(node->GetBoneTarget()));

		ndSharedPtr<ndMeshEffect> mesh (node->GetGeometry());
		if (mesh)
		{
			mesh->ApplyTransform(newTransformGeo);
		}

		ndMesh::ndCurve& positCurve = node->GetPositCurve();
		ndMesh::ndCurve& rotationCurve = node->GetRotationCurve();
		if (positCurve.GetCount() || rotationCurve.GetCount())
		{
			ndMesh::ndCurve::ndNode* positNode = node->GetPositCurve().GetFirst();
			ndMesh::ndCurve::ndNode* rotationNode = node->GetRotationCurve().GetFirst();
		
			ndMesh::ndCurveValue scaleValue;
			scaleValue.m_x = ndFloat32(1.0f);
			scaleValue.m_y = ndFloat32(1.0f);
			scaleValue.m_z = ndFloat32(1.0f);
			for (ndInt32 i = 0; i < positCurve.GetCount(); ++i)
			{
				ndMesh::ndCurveValue& positValue = positNode->GetInfo();
				ndMesh::ndCurveValue& rotationValue = rotationNode->GetInfo();
		
				ndVector animScale;
				ndMatrix stretchAxis;
				ndMatrix animTransformMatrix;
				ndMatrix keyframe(GetKeyframe(scaleValue, positValue, rotationValue) * parentMatrix);
				keyframe.PolarDecomposition(animTransformMatrix, animScale, stretchAxis);
		
				ndVector euler0;
				ndVector euler(animTransformMatrix.CalcPitchYawRoll(euler0));
		
				rotationValue.m_x = ndReal (euler.m_x);
				rotationValue.m_y = ndReal (euler.m_y);
				rotationValue.m_z = ndReal (euler.m_z);
		
				positValue.m_x = ndReal (animTransformMatrix.m_posit.m_x);
				positValue.m_y = ndReal (animTransformMatrix.m_posit.m_y);
				positValue.m_z = ndReal (animTransformMatrix.m_posit.m_z);
		
				positNode = positNode->GetNext();
				rotationNode = rotationNode->GetNext();
			}
		}

		for (ndList<ndSharedPtr<ndMesh>>::ndNode* childNode = node->GetChildren().GetFirst(); childNode; childNode = childNode->GetNext())
		{
			ndMesh* const child = *childNode->GetInfo();
			stack.PushBack(child);
			matrix.PushBack(parentMatrix);
			basePoseMatrix.PushBack(parentBasePoseMatrix);
		}
	}
}

void ndMesh::ApplyBonesRotation(const ndMatrix& rotation)
{
	ndAssert(rotation.TestOrthogonal());
	const ndMatrix invRotation(rotation.OrthoInverse());

	ndAssert(0);
	auto BoneRotation = [&rotation, &invRotation](ndMesh* const node)
	{
		if ((node->GetNodeType() == ndMesh::m_bone) || ((node->GetNodeType() == ndMesh::m_boneEnd)))
		{
			const ndMatrix matrix(node->m_matrix);
			const ndMatrix geoMatrix(node->GetGeometryMatrix());
			const ndMatrix rotatedMatrix(rotation * matrix);
			node->SetMatrix(rotatedMatrix);
			node->SetBasePoseMatrix(rotation * node->GetBasePoseMatrix());

			const ndMatrix rotatedGeoMatrix(geoMatrix * invRotation);
			node->SetGeometryMatrix(rotatedGeoMatrix);
			node->SetBoneTarget(invRotation.TransformVector(node->GetBoneTarget()));

			ndMesh::ndCurve& positCurve = node->GetPositCurve();
			ndMesh::ndCurve& rotationCurve = node->GetRotationCurve();
			if (positCurve.GetCount() || rotationCurve.GetCount())
			{
				ndMesh::ndCurve::ndNode* positNode = node->GetPositCurve().GetFirst();
				ndMesh::ndCurve::ndNode* rotationNode = node->GetRotationCurve().GetFirst();

				ndMesh::ndCurveValue scaleValue;
				scaleValue.m_x = 1.0f;
				scaleValue.m_y = 1.0f;
				scaleValue.m_z = 1.0f;

				auto GetKeyframe = [](const ndCurveValue& scale, const ndCurveValue& position, const ndCurveValue& rotation)
				{
					ndMatrix scaleMatrix(ndGetIdentityMatrix());
					scaleMatrix[0][0] = scale.m_x;
					scaleMatrix[1][1] = scale.m_y;
					scaleMatrix[2][2] = scale.m_z;
					ndMatrix matrix(scaleMatrix * ndPitchMatrix(rotation.m_x) * ndYawMatrix(rotation.m_y) * ndRollMatrix(rotation.m_z));
					matrix.m_posit = ndVector(position.m_x, position.m_y, position.m_z, 1.0f);
					return matrix;
				};

				for (ndInt32 i = 0; i < positCurve.GetCount(); ++i)
				{
					ndMesh::ndCurveValue& positValue = positNode->GetInfo();
					ndMesh::ndCurveValue& rotationValue = rotationNode->GetInfo();

					ndMatrix keyframe(GetKeyframe(scaleValue, positValue, rotationValue) * invRotation);
					ndVector euler0;
					ndVector euler(keyframe.CalcPitchYawRoll(euler0));

					rotationValue.m_x = ndReal(euler.m_x);
					rotationValue.m_y = ndReal(euler.m_y);
					rotationValue.m_z = ndReal(euler.m_z);

					positValue.m_x = ndReal(keyframe.m_posit.m_x);
					positValue.m_y = ndReal(keyframe.m_posit.m_y);
					positValue.m_z = ndReal(keyframe.m_posit.m_z);

					positNode = positNode->GetNext();
					rotationNode = rotationNode->GetNext();
				}
			}

			for (ndList<ndSharedPtr<ndMesh>>::ndNode* childNode = node->GetChildren().GetFirst(); childNode; childNode = childNode->GetNext())
			{
				ndMesh* const child = *childNode->GetInfo();
				child->SetMatrix(child->GetMatrix() * invRotation);
				child->SetBasePoseMatrix(child->GetBasePoseMatrix() * invRotation);
			}
		}
	};
	NodeIterator(BoneRotation);
}

void ndMesh::ApplyPivotsRotation(const ndMatrix& rotation)
{
	ndAssert(rotation.TestOrthogonal());
	const ndMatrix invRotation(rotation.OrthoInverse());

	auto PivotRotation = [&rotation, &invRotation](ndMesh* const node)
	{
		auto ApplyChildTransforms = [&rotation, &invRotation](ndMesh* const node)
		{
			node->SetBasePoseMatrix(rotation * node->GetBasePoseMatrix());

			const ndMatrix geoMatrix(node->GetGeometryMatrix());
			const ndMatrix rotatedGeoMatrix(geoMatrix * invRotation);
			node->SetGeometryMatrix(rotatedGeoMatrix);
			node->SetBoneTarget(invRotation.TransformVector(node->GetBoneTarget()));

			ndMesh::ndCurve& positCurve = node->GetPositCurve();
			ndMesh::ndCurve& rotationCurve = node->GetRotationCurve();
			if (positCurve.GetCount() || rotationCurve.GetCount())
			{
				ndMesh::ndCurve::ndNode* positNode = node->GetPositCurve().GetFirst();
				ndMesh::ndCurve::ndNode* rotationNode = node->GetRotationCurve().GetFirst();

				ndMesh::ndCurveValue scaleValue;
				scaleValue.m_x = 1.0f;
				scaleValue.m_y = 1.0f;
				scaleValue.m_z = 1.0f;

				auto GetKeyframe = [](const ndCurveValue& scale, const ndCurveValue& position, const ndCurveValue& rotation)
				{
					ndMatrix scaleMatrix(ndGetIdentityMatrix());
					scaleMatrix[0][0] = scale.m_x;
					scaleMatrix[1][1] = scale.m_y;
					scaleMatrix[2][2] = scale.m_z;
					ndMatrix matrix(scaleMatrix * ndPitchMatrix(rotation.m_x) * ndYawMatrix(rotation.m_y) * ndRollMatrix(rotation.m_z));
					matrix.m_posit = ndVector(position.m_x, position.m_y, position.m_z, 1.0f);
					return matrix;
				};

				for (ndInt32 i = 0; i < positCurve.GetCount(); ++i)
				{
					ndMesh::ndCurveValue& positValue = positNode->GetInfo();
					ndMesh::ndCurveValue& rotationValue = rotationNode->GetInfo();

					ndMatrix keyframe(GetKeyframe(scaleValue, positValue, rotationValue) * invRotation);
					ndVector euler0;
					ndVector euler(keyframe.CalcPitchYawRoll(euler0));

					rotationValue.m_x = ndReal(euler.m_x);
					rotationValue.m_y = ndReal(euler.m_y);
					rotationValue.m_z = ndReal(euler.m_z);

					positValue.m_x = ndReal(keyframe.m_posit.m_x);
					positValue.m_y = ndReal(keyframe.m_posit.m_y);
					positValue.m_z = ndReal(keyframe.m_posit.m_z);

					positNode = positNode->GetNext();
					rotationNode = rotationNode->GetNext();
				}
			}

			ndSharedPtr<ndMeshBody>& body(node->GetRigidBody());
			if (body)
			{
				// scale center of mass
				ndMeshBodyDynamic* const dynBody = (ndMeshBodyDynamic*)*body;
				dynBody->m_localCentreOfMass = invRotation.RotateVector(dynBody->m_localCentreOfMass);

				// scale the diagonal inertia matrix (assume of box pinciapl axis)
				ndVector invInertia(dynBody->m_invMass);
				ndVector inertia(invInertia.Reciproc());
				ndMatrix diagonalInertia(ndGetIdentityMatrix());
				diagonalInertia[0][0] = inertia[0];
				diagonalInertia[1][1] = inertia[1];
				diagonalInertia[2][2] = inertia[2];

				ndVector inertiaAxis(dynBody->m_inertiaPrincipalAxis.Scale(ndDegreeToRad));
				ndMatrix axisAngles(ndPitchMatrix(inertiaAxis[0]) * ndYawMatrix(inertiaAxis[1]) * ndRollMatrix(inertiaAxis[2]));
				ndMatrix newRotation(axisAngles * invRotation);
				ndMatrix newIntertia(newRotation.OrthoInverse() * diagonalInertia * newRotation);

				ndVector eigenValues(newIntertia.EigenVectors());
				eigenValues.m_w = inertia.m_w;
				ndVector newEigenValues(eigenValues.Reciproc());
				dynBody->m_invMass = newEigenValues;

				ndVector eulers1;
				ndVector eulers0(newIntertia.CalcPitchYawRoll(eulers1));
				dynBody->m_inertiaPrincipalAxis = eulers0.Scale(ndRadToDegree);

				ndMeshShapeInstance& shapeInstance = dynBody->m_shapeInstance;
				shapeInstance.m_localMatrix = shapeInstance.m_localMatrix * invRotation;

				ndSharedPtr<ndMeshJoint>& joint(node->GetJoint());
				if (joint)
				{
					joint->m_localFrame0 = rotation * joint->m_localFrame0 * invRotation;
					joint->m_localFrame1 = rotation * joint->m_localFrame1 * invRotation;
				}
			}
		};

		if (!node->GetParent())
		{
			const ndMatrix matrix(node->m_matrix);
			const ndMatrix rotatedMatrix(rotation * matrix);
			node->SetMatrix(rotatedMatrix);
			ApplyChildTransforms(node);
		}
		else
		{
			const ndMatrix matrix(node->m_matrix);
			const ndMatrix rotatedMatrix(rotation * matrix * invRotation);
			node->SetMatrix(rotatedMatrix);
			ApplyChildTransforms(node);
		}
	};
	NodeIterator(PivotRotation);
}

void ndMesh::ApplyCoordinateRotation(const ndMatrix& rotation)
{
	ndAssert(rotation.TestOrthogonal());
	const ndMatrix invRotation(rotation.OrthoInverse());

	auto CoordinateRotation = [&rotation, &invRotation](ndMesh* const node)
	{
		const ndMatrix matrix(node->m_matrix);
		const ndMatrix basePoseMatrix(node->m_basePoseMatrix);
		const ndMatrix geoMatrix(node->GetGeometryMatrix());

		const ndMatrix rotatedMatrix(invRotation * matrix * rotation);
		const ndMatrix rotatedGeoMatrix(invRotation * geoMatrix * rotation);
		const ndMatrix rotatedBasePoseMatrix(invRotation * basePoseMatrix * rotation);

		node->SetMatrix(rotatedMatrix);
		node->SetGeometryMatrix(rotatedGeoMatrix);
		node->SetBasePoseMatrix(rotatedBasePoseMatrix);
		node->SetBoneTarget(rotation.TransformVector(node->GetBoneTarget()));

		ndSharedPtr<ndMeshEffect> mesh(node->GetGeometry());
		if (mesh)
		{
			mesh->ApplyTransform(rotation);
		}
		
		ndMesh::ndCurve& positCurve = node->GetPositCurve();
		ndMesh::ndCurve& rotationCurve = node->GetRotationCurve();
		if (positCurve.GetCount() || rotationCurve.GetCount())
		{
			ndMesh::ndCurve::ndNode* positNode = node->GetPositCurve().GetFirst();
			ndMesh::ndCurve::ndNode* rotationNode = node->GetRotationCurve().GetFirst();
		
			ndMesh::ndCurveValue scaleValue;
			scaleValue.m_x = 1.0f;
			scaleValue.m_y = 1.0f;
			scaleValue.m_z = 1.0f;

			auto GetKeyframe = [](const ndCurveValue& scale, const ndCurveValue& position, const ndCurveValue& rotation)
			{
				ndMatrix scaleMatrix(ndGetIdentityMatrix());
				scaleMatrix[0][0] = scale.m_x;
				scaleMatrix[1][1] = scale.m_y;
				scaleMatrix[2][2] = scale.m_z;
				ndMatrix matrix(scaleMatrix * ndPitchMatrix(rotation.m_x) * ndYawMatrix(rotation.m_y) * ndRollMatrix(rotation.m_z));
				matrix.m_posit = ndVector(position.m_x, position.m_y, position.m_z, 1.0f);
				return matrix;
			};

			for (ndInt32 i = 0; i < positCurve.GetCount(); ++i)
			{
				ndMesh::ndCurveValue& positValue = positNode->GetInfo();
				ndMesh::ndCurveValue& rotationValue = rotationNode->GetInfo();
		
				ndVector animScale;
				ndMatrix stretchAxis;
				ndMatrix animTransformMatrix;
				ndMatrix keyframe(invRotation * GetKeyframe(scaleValue, positValue, rotationValue) * rotation);
				keyframe.PolarDecomposition(animTransformMatrix, animScale, stretchAxis);
		
				ndVector euler0;
				ndVector euler(animTransformMatrix.CalcPitchYawRoll(euler0));
		
				rotationValue.m_x = ndReal(euler.m_x);
				rotationValue.m_y = ndReal(euler.m_y);
				rotationValue.m_z = ndReal(euler.m_z);
		
				positValue.m_x = ndReal(animTransformMatrix.m_posit.m_x);
				positValue.m_y = ndReal(animTransformMatrix.m_posit.m_y);
				positValue.m_z = ndReal(animTransformMatrix.m_posit.m_z);
		
				positNode = positNode->GetNext();
				rotationNode = rotationNode->GetNext();
			}
		}
		
		ndSharedPtr<ndMeshBody>& body(node->GetRigidBody());
		if (body)
		{
			// scale center of mass
			ndMeshBodyDynamic* const dynBody = (ndMeshBodyDynamic*)*body;
			dynBody->m_localCentreOfMass = rotation.RotateVector(dynBody->m_localCentreOfMass);

			// scale the diagonal inertia matrix (assume of box pinciapl axis)
			ndVector invInertia(dynBody->m_invMass);
			ndVector inertia(invInertia.Reciproc());
			ndMatrix diagonalInertia(ndGetIdentityMatrix());
			diagonalInertia[0][0] = inertia[0];
			diagonalInertia[1][1] = inertia[1];
			diagonalInertia[2][2] = inertia[2];

			ndVector inertiaAxis(dynBody->m_inertiaPrincipalAxis.Scale(ndDegreeToRad));
			ndMatrix axisAngles(ndPitchMatrix(inertiaAxis[0]) * ndYawMatrix(inertiaAxis[1]) * ndRollMatrix(inertiaAxis[2]));
			ndMatrix newRotation(axisAngles * invRotation);
			ndMatrix newIntertia(newRotation.OrthoInverse() * diagonalInertia * newRotation);

			ndVector eigenValues(newIntertia.EigenVectors());
			eigenValues.m_w = inertia.m_w;
			ndVector newEigenValues(eigenValues.Reciproc());
			dynBody->m_invMass = newEigenValues;

			ndVector eulers1;
			ndVector eulers0(newIntertia.CalcPitchYawRoll(eulers1));
			dynBody->m_inertiaPrincipalAxis = eulers0.Scale(ndRadToDegree);

			ndMeshShapeInstance& shapeInstance = dynBody->m_shapeInstance;
			shapeInstance.m_localMatrix = shapeInstance.m_localMatrix * invRotation;

			ndSharedPtr<ndMeshJoint>& joint(node->GetJoint());
			if (joint)
			{
				joint->m_localFrame0 = invRotation * joint->m_localFrame0 * rotation;
				joint->m_localFrame1 = invRotation * joint->m_localFrame1 * rotation;
			}
		}
	};
	NodeIterator(CoordinateRotation);
}

ndMatrix ndMesh::CalculateLocalMatrix(ndVector& sizeOut) const
{
	const ndSharedPtr<ndMeshEffect>& meshEffect = GetGeometry();

	sizeOut.m_x = ndFloat32(0.5f);
	sizeOut.m_y = ndFloat32(0.5f);
	sizeOut.m_z = ndFloat32(0.5f);
	sizeOut.m_w = ndFloat32(0.0f);

	ndMatrix localAxis(ndGetIdentityMatrix());
	if (GetRigidBody())
	{
		const ndMeshBodyKinematic* const body = (ndMeshBodyKinematic*)*GetRigidBody();
		localAxis = body->m_shapeInstance.m_localMatrix;
		localAxis.m_posit = ndVector::m_wOne;
	}
	//ndMatrix matrix(GetGeometryMatrix());
	//ndMatrix matrix(GetGeometryMatrix() * localAxis.OrthoInverse());
	ndMatrix matrix(localAxis);
	if (meshEffect)
	{
		const ndMatrix transform(GetGeometryMatrix() * localAxis.OrthoInverse());
		const ndInt32 pointsCount = meshEffect->GetVertexCount();
		const ndInt32 pointsStride = ndInt32(meshEffect->GetVertexStrideInByte() / sizeof(ndFloat64));
		const ndFloat64* const pointsBuffer = meshEffect->GetVertexPool();

		ndVector minP(ndFloat32(1.0e10f));
		ndVector maxP(ndFloat32(-1.0e10f));
		for (ndInt32 i = 0; i < pointsCount; ++i)
		{
			ndFloat32 x = ndFloat32(pointsBuffer[i * pointsStride + 0]);
			ndFloat32 y = ndFloat32(pointsBuffer[i * pointsStride + 1]);
			ndFloat32 z = ndFloat32(pointsBuffer[i * pointsStride + 2]);
			const ndVector locaPoint(x, y, z, ndFloat32(1.0f));
			const ndVector point(transform.TransformVector(locaPoint));
			minP = minP.GetMin(point);
			maxP = maxP.GetMax(point);
		}
		const ndVector size(ndVector::m_half * (maxP - minP));
		const ndVector origin(ndVector::m_half * (maxP + minP));

		sizeOut.m_x = size.m_x;
		sizeOut.m_y = size.m_y;
		sizeOut.m_z = size.m_z;
		
		matrix.m_posit = origin;
		matrix.m_posit.m_w = ndFloat32(1.0f);
	}
	return matrix;
}

void ndMesh::CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const
{
	p0 = ndVector(1.0e10f);
	p1 = ndVector(-1.0e10f);
	auto GetAabb = [&matrix, &p0, &p1](ndMesh* const node)
	{
		if (node->GetGeometry())
		{
			const ndMatrix nodeMatrix(node->GetGeometryMatrix() * node->CalculateGlobalMatrix() * matrix);

			ndInt32 count = node->GetGeometry()->GetVertexCount();
			ndInt32 stride = ndInt32(node->GetGeometry()->GetVertexStrideInByte() / sizeof (ndFloat64));
			const ndFloat64* const array = node->GetGeometry()->GetVertexPool();
			for (ndInt32 i = 0; i < count; ++i)
			{
				ndVector p(nodeMatrix.TransformVector(ndVector(array[i * stride + 0], array[i * stride + 1], array[i * stride + 2], ndFloat64(0.0f))));
				p0 = p0.GetMin(p);
				p1 = p1.GetMax(p);
			}
		}
	};
	((ndMesh*)this)->NodeIterator(GetAabb);
	if ((p1.m_x - p0.m_x) < ndFloat32(0.0f))
	{
		p0 = ndVector::m_zero;
		p1 = ndVector::m_zero;
	}
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionBox()
{
	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));
	size = size.Scale(ndFloat32(2.0f));
	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeBox(size.m_x, size.m_y, size.m_z)));
	shape->SetLocalMatrix(localMatrix);
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionNull()
{
	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));
	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeNull()));

	shape->SetLocalMatrix(localMatrix * m_geometryMatrix);
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionSphere()
{
	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));
	ndFloat32 radios = ndMax(size.m_x, (ndMax(size.m_y, size.m_z)));
	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeSphere(radios)));
	shape->SetLocalMatrix(localMatrix);
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionCapsule()
{
	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));

	ndFloat32 radios = ndMax(size.m_y, size.m_z);
	ndFloat32 high = ndFloat32(2.0f) * (ndMax (size.m_x - radios, ndFloat32 (0.0f)));

	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeCapsule(radios, radios, high)));
	shape->SetLocalMatrix(localMatrix);
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionCylinder()
{
	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));

	ndFloat32 high = ndFloat32(2.0f) * size.m_x;
	ndFloat32 radios = ndMax(size.m_y, size.m_z);

	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeCylinder(radios, radios, high)));
	shape->SetLocalMatrix(localMatrix);
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionChamferCylinder()
{
	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));

	ndFloat32 width = size.m_x * ndFloat32(2.0f);
	ndFloat32 radius = ndMax (size.m_z - size.m_x, ndFloat32 (0.0f));
	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeChamferCylinder(radius, width)));
	shape->SetLocalMatrix(localMatrix);
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionTire()
{
	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));

	ndFloat32 width = size.m_x;
	ndFloat32 radius = size.m_y;
	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeWheel()));
	ndVector scale(ndFloat32(4.0f) * width, radius, radius, 0.0f);
	shape->SetScale(scale);
	shape->SetLocalMatrix(localMatrix);
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionConvex()
{
	ndAssert(*m_mesh);
	ndSharedPtr<ndShapeInstance>shape(m_mesh->CreateConvexCollision(1.0e-3f));
	shape->SetLocalMatrix(shape->GetLocalMatrix() * m_geometryMatrix);
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionConvexApproximation(bool lowDetail)
{
	const ndInt32 pointsCount = m_mesh->GetVertexCount();
	const ndInt32 pointsStride = ndInt32(m_mesh->GetVertexStrideInByte() / sizeof(ndFloat64));
	const ndFloat64* const pointsBuffer = m_mesh->GetVertexPool();
	
	ndArray<ndReal> meshPoints;
	for (ndInt32 i = 0; i < pointsCount; ++i)
	{
		meshPoints.PushBack(ndReal(pointsBuffer[i * pointsStride + 0]));
		meshPoints.PushBack(ndReal(pointsBuffer[i * pointsStride + 1]));
		meshPoints.PushBack(ndReal(pointsBuffer[i * pointsStride + 2]));
	}
	
	ndArray<ndInt32> indices;
	ndInt32 mark = m_mesh->IncLRU();
	ndPolyhedra::Iterator iter(**m_mesh);
	for (iter.Begin(); iter; iter++)
	{
		ndEdge* const face = &iter.GetNode()->GetInfo();
		if ((face->m_mark != mark) && (face->m_incidentFace > 0))
		{
			ndEdge* ptr = face;
			ptr->m_mark = mark;
			indices.PushBack(ptr->m_incidentVertex);
	
			ptr = ptr->m_next;
			ptr->m_mark = mark;
			indices.PushBack(ptr->m_incidentVertex);
	
			ptr = ptr->m_next;
			do
			{
				indices.PushBack(ptr->m_incidentVertex);
				ptr->m_mark = mark;
	
				ptr = ptr->m_next;
			} while (ptr != face);
		}
	}
	
	ndSharedPtr<nd::VHACD::IVHACD> interfaceVHACD (nd::VHACD::CreateVHACD());
	nd::VHACD::IVHACD::Parameters paramsVHACD;
	paramsVHACD.m_concavityToVolumeWeigh = lowDetail ? 1.0f : 0.5f;
	interfaceVHACD->Compute(&meshPoints[0], uint32_t(meshPoints.GetCount() / 3), (uint32_t*)&indices[0], uint32_t(indices.GetCount() / 3), paramsVHACD);
	
	ndSharedPtr<ndShapeInstance> compoundShapeInstance(new ndShapeInstance(new ndShapeCompound()));
	ndShapeCompound* const compoundShape = compoundShapeInstance->GetShape()->GetAsShapeCompound();
	compoundShape->BeginAddRemove();
	ndInt32 hullCount = ndInt32(interfaceVHACD->GetNConvexHulls());
	ndArray<ndVector> convexMeshPoints;
	for (ndInt32 i = 0; i < hullCount; ++i)
	{
		nd::VHACD::IVHACD::ConvexHull ch;
		interfaceVHACD->GetConvexHull(uint32_t(i), ch);
		convexMeshPoints.SetCount(ndInt32(ch.m_nPoints));
		for (ndInt32 j = 0; j < ndInt32(ch.m_nPoints); ++j)
		{
			ndVector p(ndFloat32(ch.m_points[j * 3 + 0]), ndFloat32(ch.m_points[j * 3 + 1]), ndFloat32(ch.m_points[j * 3 + 2]), ndFloat32(0.0f));
			convexMeshPoints[j] = p;
		}
		ndShapeInstance hullShape(new ndShapeConvexHull(ndInt32(convexMeshPoints.GetCount()), sizeof(ndVector), 0.01f, &convexMeshPoints[0].m_x));
		compoundShape->AddCollision(&hullShape);
	}
	compoundShape->EndAddRemove();
	
	compoundShapeInstance->SetLocalMatrix(ndGetIdentityMatrix());
	
	return compoundShapeInstance;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionTree(bool optimize)
{
	ndPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();

	ndFixSizeArray<ndMesh*, 1024> entBuffer;
	ndFixSizeArray<ndMatrix, 1024> matrixBuffer;

	entBuffer.PushBack(this);
	matrixBuffer.PushBack(m_matrix.OrthoInverse());

	while (entBuffer.GetCount())
	{
		ndMesh* node = entBuffer.Pop();
		ndMatrix matrix(node->m_matrix * matrixBuffer.Pop());

		ndSharedPtr<ndMeshEffect> meshEffect = node->GetGeometry();
		if (*meshEffect)
		{
			ndInt32 vertexStride = meshEffect->GetVertexStrideInByte() / ndInt32(sizeof(ndFloat64));
			const ndFloat64* const vertexData = meshEffect->GetVertexPool();
		
			ndInt32 mark = meshEffect->IncLRU();
			ndPolyhedra::Iterator iter(*(*meshEffect));
		
			const ndMatrix worldMatrix(matrix);
			for (iter.Begin(); iter; iter++)
			{
				ndEdge* const edge = &(*iter);
				if ((edge->m_incidentFace >= 0) && (edge->m_mark != mark))
				{
					ndFixSizeArray<ndVector, 256> face;
					ndEdge* ptr = edge;
					do
					{
						ndInt32 i = ptr->m_incidentVertex * vertexStride;
						ndVector point(ndFloat32(vertexData[i + 0]), ndFloat32(vertexData[i + 1]), ndFloat32(vertexData[i + 2]), ndFloat32(1.0f));
						face.PushBack(worldMatrix.TransformVector(point));
						ptr->m_mark = mark;
						ptr = ptr->m_next;
					} while (ptr != edge);
		
					ndInt32 materialIndex = meshEffect->GetFaceMaterial(edge);
					meshBuilder.AddFace(&face[0], face.GetCount(), materialIndex);
				}
			}
		}

		for (ndList<ndSharedPtr<ndMesh>>::ndNode* childNode = node->GetChildren().GetFirst(); childNode; childNode = childNode->GetNext())
		{
			ndMesh* const child = *childNode->GetInfo();
			entBuffer.PushBack(child);
			matrixBuffer.PushBack(matrix);
		}
	}
	meshBuilder.End(optimize);
	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeStatic_bvh(meshBuilder)));
	return shape;
}

void ndMesh::SetRigidBody(const ndSharedPtr<ndMeshBody>& rigidBody)
{
	m_rigidBody = rigidBody;
}

ndSharedPtr<ndMeshBody>& ndMesh::GetRigidBody()
{
	return m_rigidBody;
}

const ndSharedPtr<ndMeshBody>& ndMesh::GetRigidBody() const
{
	return m_rigidBody;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionFromChildren()
{
	ndArray<ndVector> points;
	ndFixSizeArray<ndSharedPtr<ndShapeInstance>, 256> shapeArray;

	for (ndList<ndSharedPtr<ndMesh>>::ndNode* ptr = GetChildren().GetFirst(); ptr; ptr = ptr->GetNext())
	{
		ndMesh* const meshNode = *ptr->GetInfo();
		ndString tmpName(meshNode->GetName());
		tmpName.ToLower();
		const char* const name = tmpName.GetStr();

		if (strstr(name, "-sphere"))
		{
			ndSharedPtr<ndShapeInstance> subShape(meshNode->CreateCollision());
			const ndMatrix matrix(subShape->GetLocalMatrix() * meshNode->m_matrix);
			subShape->SetLocalMatrix(matrix);
			shapeArray.PushBack(subShape);
		}
		else if (strstr(name, "-box"))
		{
			ndSharedPtr<ndShapeInstance> subShape(meshNode->CreateCollision());
			const ndMatrix matrix(subShape->GetLocalMatrix() * meshNode->m_matrix);
			subShape->SetLocalMatrix(matrix);
			shapeArray.PushBack(subShape);
		}
		else if (strstr(name, "-capsule"))
		{
			ndSharedPtr<ndShapeInstance> subShape(meshNode->CreateCollision());
			const ndMatrix matrix(subShape->GetLocalMatrix() * meshNode->m_matrix);
			subShape->SetLocalMatrix(matrix);
			shapeArray.PushBack(subShape);
		}
		else if (strstr(name, "-cylinder"))
		{
			ndSharedPtr<ndShapeInstance> subShape(meshNode->CreateCollision());
			const ndMatrix matrix(subShape->GetLocalMatrix() * meshNode->m_matrix);
			subShape->SetLocalMatrix(matrix);
			shapeArray.PushBack(subShape);
		}
		else if (strstr(name, "-convexhull"))
		{
			ndSharedPtr<ndShapeInstance> subShape(meshNode->CreateCollision());
			const ndMatrix matrix(subShape->GetLocalMatrix() * meshNode->m_matrix);
			subShape->SetLocalMatrix(matrix);
			shapeArray.PushBack(subShape);
		}
		else if (strstr(name, "-vhacd"))
		{
			ndAssert(0);
			//ndArray<ndInt32> indices;
			//ndDemoMesh* const mesh = (ndDemoMesh*)*node->GetInfo()->GetMesh();
			//ndAssert(mesh);
			//mesh->GetVertexArray(points);
			//mesh->GetIndexArray(indices);
			//
			//ndArray<ndTriplex> meshPoints;
			//for (ndInt32 i = 0; i < points.GetCount(); ++i)
			//{
			//	ndTriplex p;
			//	p.m_x = points[i].m_x;
			//	p.m_y = points[i].m_y;
			//	p.m_z = points[i].m_z;
			//	meshPoints.PushBack(p);
			//}
			//nd_::VHACD::IVHACD* const interfaceVHACD = nd_::VHACD::CreateVHACD();
			//
			//nd_::VHACD::IVHACD::Parameters paramsVHACD;
			////paramsVHACD.m_concavityToVolumeWeigh = 1.0;
			//paramsVHACD.m_concavityToVolumeWeigh = 0.5f;
			//interfaceVHACD->Compute(&meshPoints[0].m_x, uint32_t(points.GetCount()),
			//	(uint32_t*)&indices[0], uint32_t(indices.GetCount()) / 3, paramsVHACD);
			//
			//ndInt32 hullCount = ndInt32(interfaceVHACD->GetNConvexHulls());
			//ndArray<ndVector> convexMeshPoints;
			//for (ndInt32 i = 0; i < hullCount; ++i)
			//{
			//	nd_::VHACD::IVHACD::ConvexHull ch;
			//	interfaceVHACD->GetConvexHull(uint32_t(i), ch);
			//	convexMeshPoints.SetCount(ndInt32(ch.m_nPoints));
			//	for (ndInt32 j = 0; j < ndInt32(ch.m_nPoints); ++j)
			//	{
			//		ndVector p(ndFloat32(ch.m_points[j * 3 + 0]), ndFloat32(ch.m_points[j * 3 + 1]), ndFloat32(ch.m_points[j * 3 + 2]), ndFloat32(0.0f));
			//		convexMeshPoints[j] = p;
			//	}
			//	shapeArray.PushBack(new ndShapeInstance(new ndShapeConvexHull(ndInt32(convexMeshPoints.GetCount()), sizeof(ndVector), 0.01f, &convexMeshPoints[0].m_x)));
			//	const ndMatrix matrix(node->GetInfo()->GetMeshMatrix() * node->GetInfo()->GetCurrentMatrix());
			//	shapeArray[shapeArray.GetCount() - 1]->SetLocalMatrix(matrix);
			//}
			//
			//interfaceVHACD->Clean();
			//interfaceVHACD->Release();
		}
	}

	if (shapeArray.GetCount() > 1)
	{
		ndSharedPtr<ndShapeInstance> compoundInstance(new ndShapeInstance(new ndShapeCompound()));
		ndShapeCompound* const compound = compoundInstance->GetShape()->GetAsShapeCompound();

		compound->BeginAddRemove();
		for (ndInt32 i = 0; i < shapeArray.GetCount(); ++i)
		{
			compound->AddCollision(*shapeArray[i]);
		}
		compound->EndAddRemove();
		shapeArray[0] = compoundInstance;
	}
	else
	{
		shapeArray.PushBack(ndSharedPtr<ndShapeInstance>(new ndShapeInstance(new ndShapeCompound())));
	}
	return shapeArray[0];
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollision()
{
	ndString tmpName(GetName());
	tmpName.ToLower();
	const char* const name = tmpName.GetStr();

	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeNull()));
	if (strstr(name, "-box"))
	{
		shape = CreateCollisionBox();
	}
	else if (strstr(name, "-sphere"))
	{
		shape = CreateCollisionSphere();
	}
	else if (strstr(name, "-capsule"))
	{
		shape = CreateCollisionCapsule();
	}
	else if (strstr(name, "-cylinder"))
	{
		shape = CreateCollisionCylinder();
	}
	else if (strstr(name, "-chamferedCylinder"))
	{
		shape = CreateCollisionChamferCylinder();
	}
	
	else if (strstr(name, "-tire"))
	{
		shape = CreateCollisionTire();
	}
	else if (strstr(name, "-convexhull"))
	{
		shape = CreateCollisionConvex();
	}
	else if (strstr(name, "-mesh"))
	{
		shape = CreateCollisionTree();
	}
	else if (strstr(name, "-compound"))
	{
		shape = CreateCollisionFromChildren();
	}
	else if (strstr(name, "-vhacd"))
	{
		shape = CreateCollisionConvexApproximation();
	}
	else
	{
		ndExpandTraceMessage("ndMesh Node: %s has knowns collision shape\n", name);
	}
	return shape;
}

ndSharedPtr<ndJointBilateralConstraint> ndMesh::CreateJoint()
{
	ndString tmpName(GetName());
	tmpName.ToLower();
	const char* const name = tmpName.GetStr();
	ndSharedPtr<ndJointBilateralConstraint> joint(new ndJointFix6dof());
	if (strstr(name, "-hinge"))
	{
		joint = ndSharedPtr<ndJointBilateralConstraint>(new ndJointHinge());
	}
	else if (strstr(name, "-slider"))
	{
		joint = ndSharedPtr<ndJointBilateralConstraint>(new ndJointSlider());
	}
	else if (strstr(name, "-roller"))
	{
		joint = ndSharedPtr<ndJointBilateralConstraint>(new ndJointSlider());
	}
	else if (strstr(name, "-cylindrical"))
	{
		joint = ndSharedPtr<ndJointBilateralConstraint>(new ndJointSlider());
	}
	else if (strstr(name, "-doublehinge"))
	{
		joint = ndSharedPtr<ndJointBilateralConstraint>(new ndJointDoubleHinge());
	}
	else if (strstr(name, "-plane"))
	{
		joint = ndSharedPtr<ndJointBilateralConstraint>(new ndJointDoubleHinge());
	}
	else if (strstr(name, "-wheel"))
	{
		joint = ndSharedPtr<ndJointBilateralConstraint>(new ndJointWheel());
	}
	else if (strstr(name, "-spherical"))
	{
		joint = ndSharedPtr<ndJointBilateralConstraint>(new ndJointSpherical());
	}
	else
	{
		ndExpandTraceMessage("ndMesh Node: %s unknown joint, using ndJointFix6dof as place holder\n", name);
	}

	ndMesh* parent = GetParent();
	for (; parent && !(*parent->GetRigidBody()); parent = parent->GetParent());
	ndAssert(parent);
	const ndMatrix childMatrix(CalculateGlobalMatrix());
	const ndMatrix parentMatrix(parent->CalculateGlobalMatrix());
	joint->SetLocalMatrix0(ndGetIdentityMatrix());
	joint->SetLocalMatrix1(childMatrix * parentMatrix.OrthoInverse());

	return joint;
}

ndCloseLoopConstraints::ndCloseLoopConstraints()
	:ndMesh()
	,m_loopJoints()
{
	SetName(ND_MESH_CONSTRAINT_LOOPS);
}

ndCloseLoopConstraints::ndCloseLoopConstraints(const ndCloseLoopConstraints& src)
	:ndMesh(src)
	,m_loopJoints()
{
	const ndMesh* const root = src.GetRoot();
	for (ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* ptr = src.m_loopJoints.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		ndSharedPtr<ndMeshLoopJoint>& srcLoop = ptr->GetInfo();

		const ndSharedPtr<ndMeshJoint> joint(srcLoop->m_joint->Duplicate());
		ndMesh* const childReference = root->FindByName(srcLoop->m_childNode->GetName());
		ndMesh* const parentdReference = root->FindByName(srcLoop->m_parentNode->GetName());
		ndSharedPtr<ndMeshLoopJoint> loopJoint(new ndMeshLoopJoint(this, joint, childReference, parentdReference));
		m_loopJoints.Append(loopJoint);
	}
}

ndMesh* ndCloseLoopConstraints::GetAsMesh()
{
	return nullptr;
}

const ndMesh* ndCloseLoopConstraints::GetAsMesh() const
{
	return nullptr;
}

ndCloseLoopConstraints* ndCloseLoopConstraints::GetAsCloseLoopConstraints()
{
	return this;
}

const ndCloseLoopConstraints* ndCloseLoopConstraints::GetAsCloseLoopConstraints() const
{
	return this;
}

void ndCloseLoopConstraints::UpdateNames()
{
	for (ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* ptrLoops = m_loopJoints.GetFirst(); ptrLoops; ptrLoops = ptrLoops->GetNext())
	{
		ndSharedPtr<ndMeshLoopJoint>& loop = ptrLoops->GetInfo();
		loop->UpdateName();
	}
}

ndMesh* ndCloseLoopConstraints::CreateClone() const
{
	const ndCloseLoopConstraints* const self = GetAsCloseLoopConstraints();
	ndAssert(self);
	return new ndCloseLoopConstraints(*self);
}

void ndCloseLoopConstraints::CreateCloneFixDependencies()
{
	ndMesh::CreateCloneFixDependencies();

	const ndMesh* const root = GetRoot();
	//ndCloseLoopConstraints* const loopList = GetAsCloseLoopConstraints();
	//for (ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* ptrLoops = loopList->m_loopJoints.GetFirst(); ptrLoops; ptrLoops = ptrLoops->GetNext())
	for (ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* ptrLoops = m_loopJoints.GetFirst(); ptrLoops; ptrLoops = ptrLoops->GetNext())
	{
		ndSharedPtr<ndMeshLoopJoint>& loop = ptrLoops->GetInfo();
		ndMesh* const childReference = root->FindByName(loop->m_childNode->GetName());
		ndMesh* const parentdReference = root->FindByName(loop->m_parentNode->GetName());
		ndAssert(childReference);
		ndAssert(parentdReference);
		loop->m_childNode = childReference;
		loop->m_parentNode = parentdReference;
		loop->m_joint->DuplicateFixDependencies(root);
	}
}

bool ndCloseLoopConstraints::operator==(const ndMesh& other) const
{
	bool test = ndMesh::operator==(other);
	ndAssert(0);
	return test;
}

ndCollidingPairs::ndCollidingPairs()
	:ndMesh()
	,m_collidingPairs()
{
	SetName(ND_MESH_COLLIDING_PAIRS);
}

ndCollidingPairs::ndCollidingPairs(const ndCollidingPairs& src)
	:ndMesh(src)
	,m_collidingPairs()
{
	const ndMesh* const root = src.GetRoot();
	for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* ptr = src.m_collidingPairs.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		ndSharedPtr<ndMeshCollidingPair>& srcPair = ptr->GetInfo();

		ndMesh* const childReference = root->FindByName(srcPair->m_childNode->GetName());
		ndMesh* const parentdReference = root->FindByName(srcPair->m_parentNode->GetName());
		ndSharedPtr<ndMeshCollidingPair> pair(new ndMeshCollidingPair(childReference, parentdReference));
		m_collidingPairs.Append(pair);
	}
}

ndMesh* ndCollidingPairs::CreateClone() const
{
	const ndCollidingPairs* const self = GetAsCollidingPairs();
	ndAssert(self);
	return new ndCollidingPairs(*self);
}

void ndCollidingPairs::CreateCloneFixDependencies()
{
	ndMesh::CreateCloneFixDependencies();

	const ndMesh* const root = GetRoot();
	//ndCollidingPairs* const pairList = GetAsCollidingPairs();
	for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* ptrLoops = m_collidingPairs.GetFirst(); ptrLoops; ptrLoops = ptrLoops->GetNext())
	{
		ndSharedPtr<ndMeshCollidingPair>& pair = ptrLoops->GetInfo();
		ndMesh* const childReference = root->FindByName(pair->m_childNode->GetName());
		ndMesh* const parentdReference = root->FindByName(pair->m_parentNode->GetName());
		ndAssert(childReference);
		ndAssert(parentdReference);
		pair->m_childNode = childReference;
		pair->m_parentNode = parentdReference;
	}
}

bool ndCollidingPairs::operator==(const ndMesh& other) const
{
	bool test = ndMesh::operator==(other);
	ndAssert(0);
	return test;
}

ndMesh* ndCollidingPairs::GetAsMesh()
{
	return nullptr;
}

const ndMesh* ndCollidingPairs::GetAsMesh() const
{
	return nullptr;
}

ndCollidingPairs* ndCollidingPairs::GetAsCollidingPairs()
{
	return this;
}

void ndCollidingPairs::UpdateNames()
{
	//for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* ptrLoops = m_collidingPairs.GetFirst(); ptrLoops; ptrLoops = ptrLoops->GetNext())
	//{
	//	//ndSharedPtr<ndMeshCollidingPair>& pair = ptrLoops->GetInfo();
	//}
}

const ndCollidingPairs* ndCollidingPairs::GetAsCollidingPairs() const
{
	return this;
}

ndSharedPtr<ndMeshJoint> ndMesh::LoadJoint(const nd::TiXmlElement* const xmlJoint) const
{
	const ndMesh* mesh = this;
	const char* const constructor = xmlGetString(xmlJoint, "constructor");
	ndSharedPtr<ndMeshJoint> joint(new ndMeshJointFix6dof(mesh));
	if (strcmp(constructor, ndJointHinge::StaticClassName()) == 0)
	{
		joint = ndSharedPtr<ndMeshJoint>(new ndMeshJointHinge(mesh));
	}
	else if (strcmp(constructor, ndJointRoller::StaticClassName()) == 0)
	{
		joint = ndSharedPtr<ndMeshJoint>(new ndMeshJointRoller(mesh));
	}
	else if (strcmp(constructor, ndJointDoubleHinge::StaticClassName()) == 0)
	{
		joint = ndSharedPtr<ndMeshJoint>(new ndMeshJointDoubleHinge(mesh));
	}
	else if (strcmp(constructor, ndJointSpherical::StaticClassName()) == 0)
	{
		joint = ndSharedPtr<ndMeshJoint>(new ndMeshJointSpherical(mesh));
	}
	else if (strcmp(constructor, ndJointWheel::StaticClassName()) == 0)
	{
		joint = ndSharedPtr<ndMeshJoint>(new ndMeshJointWheel(mesh));
	}
	else if (strcmp(constructor, ndJointSlider::StaticClassName()) == 0)
	{
		joint = ndSharedPtr<ndMeshJoint>(new ndMeshJointSlider(mesh));
	}
	else if (strcmp(constructor, ndJointFix6dof::StaticClassName()) == 0)
	{
		joint = ndSharedPtr<ndMeshJoint>(new ndMeshJointFix6dof(mesh));
	}
	else if (strcmp(constructor, ndIkSwivelPositionEffector::StaticClassName()) == 0)
	{
		joint = ndSharedPtr<ndMeshJoint>(new ndMeshJointIkSwivelPositionEffector(mesh));
	}
	else if (strcmp(constructor, ndJointPlane::StaticClassName()) == 0)
	{
		joint = ndSharedPtr<ndMeshJoint>(new ndMeshJointPlane(mesh));
	}
	else if (strcmp(constructor, ndJointGear::StaticClassName()) == 0)
	{
		joint = ndSharedPtr<ndMeshJoint>(new ndMeshJointGear(mesh));
	}
	else if (strcmp(constructor, ndMultiBodyVehicleDifferentialAxle::StaticClassName()) == 0)
	{
		joint = ndSharedPtr<ndMeshJoint>(new ndMeshJointDifferentialAxle(mesh));
	}
	else
	{
		ndAssert(0);
	}

	joint->DeserializeFromXml(xmlJoint);
	return joint;
}

ndSharedPtr<ndMeshTransformModifier> ndMesh::LoadModifier(const nd::TiXmlElement* const xmlModifier) const
{
	ndSharedPtr<ndMeshTransformModifier> modifier(nullptr);

	const char* const constructor = xmlGetString(xmlModifier, "constructor");
	if (strcmp(constructor, ndMeshTransformModifierLookAt::StaticClassName()) == 0)
	{
		modifier = ndSharedPtr<ndMeshTransformModifier>(new ndMeshTransformModifierLookAt(this, nullptr));
		modifier->DeserializeFromXml(xmlModifier);
	}
	else if (strcmp(constructor, ndMeshTransformModifierTwoLinksIK::StaticClassName()) == 0)
	{
		modifier = ndSharedPtr<ndMeshTransformModifier>(new ndMeshTransformModifierTwoLinksIK(this, nullptr));
		modifier->DeserializeFromXml(xmlModifier);
	}
	else
	{
		ndAssert(0);
	}

	return modifier;
}