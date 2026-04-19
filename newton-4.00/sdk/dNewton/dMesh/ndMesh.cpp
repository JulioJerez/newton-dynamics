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
#include "ndJointHinge.h"
#include "ndJointWheel.h"
#include "ndJointSlider.h"
#include "ndJointFix6dof.h"
#include "ndJointSpherical.h"
#include "ndMeshComponents.h"
#include "ndJointDoubleHinge.h"

ndMesh::ndMesh()
	:ndClassAlloc()
	,m_matrix(ndGetIdentityMatrix())
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
	,m_boneTarget(ndVector::m_wOne)
	,m_type(m_node)
{
}

ndMesh::ndMesh(const ndMesh&)
	:ndClassAlloc()
{
	ndAssert(0);
}

ndMesh::ndMesh(const ndShapeInstance& shape, ndUvMapingMode mapping)
	:ndClassAlloc()
	,m_matrix(ndGetIdentityMatrix())
	,m_geometryMatrix(ndGetIdentityMatrix())
	,m_name("node")
	,m_scale()
	,m_posit()
	,m_rotation()
	,m_parent(nullptr)
	,m_mesh(new ndMeshEffect(shape))
	,m_selfChildNode(nullptr)
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

ndMesh::~ndMesh()
{
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

ndMesh* ndMesh::CreateClone() const
{
	return new ndMesh(*this);
}

ndSharedPtr<ndMeshEffect>& ndMesh::GetMesh()
{
	return m_mesh;
}

const ndSharedPtr<ndMeshEffect>& ndMesh::GetMesh() const
{
	return m_mesh;
}

void ndMesh::SetMesh(const ndSharedPtr<ndMeshEffect>& mesh)
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

ndCloseLoopConstraints* ndMesh::GetLoopJoints()
{
	ndMesh* const contrains = GetRoot()->FindByName(ND_MESH_LOOP_JOINTS);
	return (ndCloseLoopConstraints*)contrains;
}

const ndCloseLoopConstraints* ndMesh::GetLoopJoints() const 
{
	ndMesh* const contrains = GetRoot()->FindByName(ND_MESH_LOOP_JOINTS);
	return (ndCloseLoopConstraints*)contrains;
}

void ndMesh::AddLoopJoint(const ndSharedPtr<ndMeshLoopJoint>& joint)
{
	ndCloseLoopConstraints* constrains = GetLoopJoints();
	if (!constrains)
	{
		ndMesh* const rootNode = GetRoot();
		ndAssert(rootNode);
		ndSharedPtr<ndMesh> loops(new ndCloseLoopConstraints());
		rootNode->AddChild(loops);
		constrains = GetLoopJoints();
	}
	constrains->m_loopJoints.Append(joint);
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
		if (name == node->m_name)
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

	const ndMatrix invTransform(transform.Inverse4x4());
	for (ndMesh* node = IteratorFirst(); node; node = node->IteratorNext(this))
	{
		const ndMatrix entMatrix(invTransform * node->m_matrix * transform);
		node->m_matrix = entMatrix;

		ndSharedPtr<ndMeshEffect> mesh (node->GetMesh());
		if (mesh)
		{
			const ndMatrix meshMatrix(invTransform * node->GetGeometryMatrix() * transform);
			node->SetGeometryMatrix(meshMatrix);
			mesh->ApplyTransform(transform);
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
			for (ndInt32 i = 0; i < positCurve.GetCount(); ++i)
			{
				ndMesh::ndCurveValue& positValue = positNode->GetInfo();
				ndMesh::ndCurveValue& rotationValue = rotationNode->GetInfo();

				ndVector animScale;
				ndMatrix stretchAxis;
				ndMatrix animTransformMatrix;
				ndMatrix keyframe(invTransform * GetKeyframe(scaleValue, positValue, rotationValue) * transform);
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
	}
}

ndMatrix ndMesh::CalculateLocalMatrix(ndVector& sizeOut) const
{
	ndSharedPtr<ndMeshEffect> meshEffect = GetMesh();

	const ndInt32 pointsCount = meshEffect->GetVertexCount();
	const ndInt32 pointsStride = ndInt32(meshEffect->GetVertexStrideInByte() / sizeof(ndFloat64));
	const ndFloat64* const pointsBuffer = meshEffect->GetVertexPool();

	// geometry points are for rendering, therefore they may be have duplicate points 
	// that can skew the covariance matrix
	ndArray<ndBigVector> uniquePoints;
	for (ndInt32 i = 0; i < pointsCount; ++i)
	{
		ndFloat64 x = ndFloat32(pointsBuffer[i * pointsStride + 0]);
		ndFloat64 y = ndFloat32(pointsBuffer[i * pointsStride + 1]);
		ndFloat64 z = ndFloat32(pointsBuffer[i * pointsStride + 2]);
		const ndBigVector p(x, y, z, ndFloat64(0.0f));
		uniquePoints.PushBack(p);
	}
	ndArray<ndInt32> indexList;
	indexList.SetCount(pointsCount);
	const ndInt32 vertexCount = ndVertexListToIndexList(&uniquePoints[0].m_x, ndInt32(sizeof(ndBigVector)), 3, pointsCount, &indexList[0], ndFloat32(1.0e-6f));
	uniquePoints.SetCount(vertexCount);

	ndVector minP(ndFloat32(1.0e10f));
	ndVector maxP(ndFloat32(-1.0e10f));
	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		const ndVector p(uniquePoints[i]);
		minP = minP.GetMin(p);
		maxP = maxP.GetMax(p);
	}

	ndVector size(ndVector::m_half * (maxP - minP));
	ndVector origin(ndVector::m_half * (maxP + minP));
	ndMatrix covariance(ndGetZeroMatrix());
	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		const ndVector p((ndVector(uniquePoints[i]) - origin) & ndVector::m_triplexMask);
		ndAssert(p.m_w == ndFloat32(0.0f));
		const ndMatrix matrix(ndCovarianceMatrix(p, p));
		covariance.m_front += matrix.m_front;
		covariance.m_up += matrix.m_up;
		covariance.m_right += matrix.m_right;
	}
	// since it is an stimate, we can zero out small misallgmnets.
	for (ndInt32 i = 0; i < 2; ++i)
	{
		for (ndInt32 j = i + 1; j < 3; ++j)
		{
			if (ndAbs(covariance[i][j]) < ndFloat32 (1.0e-5f))
			{
				covariance[i][j] = ndFloat32 (0.0f);
				covariance[j][i] = ndFloat32(0.0f);
			}
		}
	}

	//const ndMatrix xxxn(covariance);
	const ndVector eigen(covariance.EigenVectors() & ndVector::m_triplexMask);

	//ndMatrix xxxxx(ndGetIdentityMatrix());
	//xxxxx[0][0] = eigen[0];
	//xxxxx[1][1] = eigen[1];
	//xxxxx[2][2] = eigen[2];
	//ndMatrix xxxxxxx(covariance.OrthoInverse() * xxxxx * covariance);

	covariance.m_posit = origin;
	covariance.m_posit.m_w = ndFloat32(1.0f);

	sizeOut = size;
	sizeOut.m_w = ndFloat32(0.0f);
	return covariance;
}

void ndMesh::CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const
{
	p0 = ndVector(1.0e10f);
	p1 = ndVector(-1.0e10f);
	auto GetAabb = [&matrix, &p0, &p1](ndMesh* const node)
	{
		if (node->GetMesh())
		{
			const ndMatrix nodeMatrix(node->GetGeometryMatrix() * node->CalculateGlobalMatrix() * matrix);

			ndInt32 count = node->GetMesh()->GetVertexCount();
			ndInt32 stride = ndInt32(node->GetMesh()->GetVertexStrideInByte() / sizeof (ndFloat64));
			const ndFloat64* const array = node->GetMesh()->GetVertexPool();
			for (ndInt32 i = 0; i < count; ++i)
			{
				ndVector p(nodeMatrix.TransformVector(ndVector(array[i * stride + 0], array[i * stride + 1], array[i * stride + 2], ndFloat64(0.0f))));
				p0 = p0.GetMin(p);
				p1 = p1.GetMax(p);
			}
		}
	};
	((ndMesh*)this)->NodeIterator(GetAabb);
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionBox()
{
	ndSharedPtr<ndMeshEffect> meshEffect = GetMesh();
	ndAssert(*meshEffect);
	
	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));
	size = size.Scale(ndFloat32(2.0f));
	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeBox(size.m_x, size.m_y, size.m_z)));
	shape->SetLocalMatrix(localMatrix * m_geometryMatrix);
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionNull()
{
	ndSharedPtr<ndMeshEffect> meshEffect = GetMesh();
	ndAssert(*meshEffect);

	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));
	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeNull()));

	shape->SetLocalMatrix(localMatrix * m_geometryMatrix);
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionSphere()
{
	ndSharedPtr<ndMeshEffect> meshEffect = GetMesh();
	ndAssert(*meshEffect);

	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));
	ndFloat32 radios = ndMax(size.m_x, (ndMax(size.m_y, size.m_z)));
	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeSphere(radios)));

	shape->SetLocalMatrix(localMatrix * m_geometryMatrix);
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionCapsule()
{
	ndSharedPtr<ndMeshEffect> meshEffect = GetMesh();
	ndAssert(*meshEffect);

	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));

	ndFloat32 radios = ndMax(size.m_y, size.m_z);
	ndFloat32 high = ndFloat32(2.0f) * (ndMax (size.m_x - radios, ndFloat32 (0.0f)));

	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeCapsule(radios, radios, high)));
	shape->SetLocalMatrix(localMatrix * m_geometryMatrix);
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionCylinder()
{
	ndSharedPtr<ndMeshEffect> meshEffect = GetMesh();
	ndAssert(*meshEffect);

	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));

	ndFloat32 high = ndFloat32(2.0f) * size.m_x;
	ndFloat32 radios = ndMax(size.m_y, size.m_z);

	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeCylinder(radios, radios, high)));
	shape->SetLocalMatrix(localMatrix * m_geometryMatrix);
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionChamferCylinder()
{
	ndSharedPtr<ndMeshEffect> meshEffect = GetMesh();
	ndAssert(*meshEffect);

	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));

	ndFloat32 width = size.m_x * ndFloat32(2.0f);
	ndFloat32 radius = ndMax (size.m_z - size.m_x, ndFloat32 (0.0f));
	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeChamferCylinder(radius, width)));
	shape->SetLocalMatrix(localMatrix * m_geometryMatrix);
	return shape;
}

ndSharedPtr<ndShapeInstance> ndMesh::CreateCollisionTire()
{
	ndSharedPtr<ndMeshEffect> meshEffect = GetMesh();
	ndAssert(*meshEffect);

	ndVector size;
	ndMatrix localMatrix(CalculateLocalMatrix(size));

	ndFloat32 width = size.m_x;
	ndFloat32 radius = size.m_y;
	ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeChamferCylinder(ndFloat32(0.75f), ndFloat32(0.5f))));
	ndVector scale(ndFloat32(4.0f) * width, radius, radius, 0.0f);
	shape->SetScale(scale);

	shape->SetLocalMatrix(localMatrix * m_geometryMatrix);
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

		ndSharedPtr<ndMeshEffect> meshEffect = node->GetMesh();
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

void ndMesh::SetRigidBody(ndSharedPtr<ndMeshBody>& rigidBody)
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

	ndAssert(shapeArray.GetCount());
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
	if (strstr(name, "-spherical"))
	{
		joint = ndSharedPtr<ndJointBilateralConstraint>(new ndJointSpherical());
	}
	if (strstr(name, "-slider"))
	{
		joint = ndSharedPtr<ndJointBilateralConstraint>(new ndJointSlider());
	}
	if (strstr(name, "-doublehinge"))
	{
		joint = ndSharedPtr<ndJointBilateralConstraint>(new ndJointDoubleHinge());
	}
	if (strstr(name, "-wheel"))
	{
		joint = ndSharedPtr<ndJointBilateralConstraint>(new ndJointWheel());
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
	SetName(ND_MESH_LOOP_JOINTS);
}

ndCloseLoopConstraints::ndCloseLoopConstraints(const ndMesh& src)
	:ndMesh(src)
	,m_loopJoints()
{
	ndAssert(0);
}

ndCloseLoopConstraints* ndCloseLoopConstraints::GetAsCloseLoopConstraints()
{
	return this;
}

const ndCloseLoopConstraints* ndCloseLoopConstraints::GetAsCloseLoopConstraints() const
{
	return this;
}

ndMesh* ndCloseLoopConstraints::CreateClone() const
{
	ndAssert(0);
	return nullptr;
}

