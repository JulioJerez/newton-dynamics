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
#include "ndContact.h"
#include "ndShapeInstance.h"
#include "ndContactSolver.h"
#include "ndBodyKinematic.h"
#include "ndShapeCompound.h"
#include "ndMeshBaseComponents.h"

#define D_TOP_DOWN_COMPOUND_BUILD

#define D_MAX_MIN_VOLUME	ndFloat32 (1.0e-3f)

ndShapeCompound::ndNodeBase::~ndNodeBase()
{
	if (m_shapeInstance) 
	{
		delete m_shapeInstance;
	}
	if (m_left) 
	{
		delete m_left;
	}
	if (m_right) 
	{
		delete m_right;
	}
}

ndShapeCompound::ndNodeBase::ndNodeBase()
	:ndClassAlloc()
	,m_type(m_node)
	,m_left(nullptr)
	,m_right(nullptr)
	,m_parent(nullptr)
	,m_myNode(nullptr)
	,m_shapeInstance(nullptr)
{
}

ndShapeCompound::ndNodeBase::ndNodeBase(const ndNodeBase& copyFrom)
	:ndClassAlloc()
	,m_p0(copyFrom.m_p0)
	,m_p1(copyFrom.m_p1)
	,m_size(copyFrom.m_size)
	,m_origin(copyFrom.m_origin)
	,m_area(copyFrom.m_area)
	,m_type(copyFrom.m_type)
	,m_left(nullptr)
	,m_right(nullptr)
	,m_parent(nullptr)
	,m_myNode(nullptr)
	,m_shapeInstance(nullptr)
{
	ndAssert(!copyFrom.m_shapeInstance);
}

ndShapeCompound::ndNodeBase::ndNodeBase(ndShapeInstance* const instance)
	:ndClassAlloc()
	,m_type(m_leaf)
	,m_left(nullptr)
	,m_right(nullptr)
	,m_parent(nullptr)
	,m_myNode(nullptr)
	,m_shapeInstance(new ndShapeInstance(*instance))
{
	CalculateAABB();
}

ndShapeCompound::ndNodeBase::ndNodeBase(ndNodeBase* const left, ndNodeBase* const right)
	:ndClassAlloc()
	,m_type(m_node)
	,m_left(left)
	,m_right(right)
	,m_parent(nullptr)
	,m_myNode(nullptr)
	,m_shapeInstance(nullptr)
{
	m_left->m_parent = this;
	m_right->m_parent = this;

	ndVector p0(left->m_p0.GetMin(right->m_p0));
	ndVector p1(left->m_p1.GetMax(right->m_p1));
	SetBox(p0, p1);
}

void ndShapeCompound::ndNodeBase::Init(ndNodeBase* const left, ndNodeBase* const right)
{
	m_bhvLinked = 1;
	m_parent = nullptr;
	left->m_bhvLinked = 1;
	right->m_bhvLinked = 1;
	ndVector p0(left->m_p0.GetMin(right->m_p0));
	ndVector p1(left->m_p1.GetMax(right->m_p1));
	SetBox(p0, p1);
}

ndShapeInstance* ndShapeCompound::ndNodeBase::GetShape() const
{
	return m_shapeInstance;
}

inline void ndShapeCompound::ndNodeBase::CalculateAABB()
{
	ndVector p0;
	ndVector p1;
	m_shapeInstance->CalculateAabb(m_shapeInstance->GetLocalMatrix(), p0, p1);
	SetBox(p0, p1);
}

inline void ndShapeCompound::ndNodeBase::SetBox(const ndVector& p0, const ndVector& p1)
{
	m_p0 = p0;
	m_p1 = p1;
	ndAssert(m_p0.m_w == ndFloat32(0.0f));
	ndAssert(m_p1.m_w == ndFloat32(0.0f));
	m_size = ndVector::m_half * (m_p1 - m_p0);
	m_origin = ndVector::m_half * (m_p1 + m_p0);
	m_area = m_size.DotProduct(m_size.ShiftTripleRight()).m_x;
}

const ndShapeCompound::ndTreeArray& ndShapeCompound::GetTree() const
{
	return m_array;
}

class ndShapeCompound::ndSpliteInfo
{
	public:
	ndSpliteInfo(ndNodeBase** const boxArray, ndInt32 boxCount)
	{
		ndVector minP(ndFloat32(1.0e15f));
		ndVector maxP(-ndFloat32(1.0e15f));

		if (boxCount == 2)
		{
			m_axis = 1;
			for (ndInt32 i = 0; i < boxCount; ++i)
			{
				ndNodeBase* const node = boxArray[i];
				ndAssert(node->m_type == m_leaf);
				minP = minP.GetMin(node->m_p0);
				maxP = maxP.GetMax(node->m_p1);
			}
		}
		else
		{
			ndVector median(ndVector::m_zero);
			ndVector varian(ndVector::m_zero);

			for (ndInt32 i = 0; i < boxCount; ++i)
			{
				ndNodeBase* const node = boxArray[i];
				ndAssert(node->m_type == m_leaf);
				minP = minP.GetMin(node->m_p0);
				maxP = maxP.GetMax(node->m_p1);
				ndVector p(ndVector::m_half * (node->m_p0 + node->m_p1));
				median += p;
				varian += p * p;
			}

			varian = varian.Scale(ndFloat32(boxCount)) - median * median;

			ndInt32 index = 0;
			ndFloat32 maxVarian = ndFloat32(-1.0e10f);
			for (ndInt32 i = 0; i < 3; ++i)
			{
				if (varian[i] > maxVarian)
				{
					index = i;
					maxVarian = varian[i];
				}
			}

			ndVector center = median.Scale(ndFloat32(1.0f) / ndFloat32(boxCount));

			ndFloat32 test = center[index];

			ndInt32 i0 = 0;
			ndInt32 i1 = boxCount - 1;
			do
			{
				for (; i0 <= i1; i0++)
				{
					ndNodeBase* const node = boxArray[i0];
					ndFloat32 val = (node->m_p0[index] + node->m_p1[index]) * ndFloat32(0.5f);
					if (val > test)
					{
						break;
					}
				}

				for (; i1 >= i0; i1--)
				{
					ndNodeBase* const node = boxArray[i1];
					ndFloat32 val = (node->m_p0[index] + node->m_p1[index]) * ndFloat32(0.5f);
					if (val < test)
					{
						break;
					}
				}

				if (i0 < i1)
				{
					ndSwap(boxArray[i0], boxArray[i1]);
					i0++;
					i1--;
				}

			} while (i0 <= i1);

			if (i0 > 0)
			{
				i0--;
			}
			if ((i0 + 1) >= boxCount)
			{
				i0 = boxCount - 2;
			}

			m_axis = i0 + 1;
		}

		ndAssert(maxP.m_x - minP.m_x >= ndFloat32(0.0f));
		ndAssert(maxP.m_y - minP.m_y >= ndFloat32(0.0f));
		ndAssert(maxP.m_z - minP.m_z >= ndFloat32(0.0f));
		m_p0 = minP;
		m_p1 = maxP;
	}

	ndInt32 m_axis;
	ndVector m_p0;
	ndVector m_p1;
};

ndShapeCompound::ndTreeArray::ndTreeArray()
	:ndTree<ndNodeBase*, ndInt32, ndContainersFreeListAlloc<ndNodeBase*>>()
{
}

void ndShapeCompound::ndTreeArray::AddNode(ndNodeBase* const node, ndInt32 index, const ndShapeInstance* const parent)
{
	ndTreeArray::ndNode* const myNode = Insert(node, index);
	node->m_myNode = myNode;
	node->m_shapeInstance->m_subCollisionHandle = myNode;
	node->m_shapeInstance->m_parent = const_cast<ndShapeInstance*>(parent);
}

ndShapeCompound::ndShapeCompound()
	:ndShape(m_compound)
	,m_array()
	,m_treeEntropy(ndFloat64(0.0f))
	,m_boxMinRadius(ndFloat32(0.0f))
	,m_boxMaxRadius(ndFloat32(0.0f))
	,m_root(nullptr)
	,m_idIndex(0)
	,m_owner(nullptr)
	,m_isStatic(false)
{
}

ndShapeCompound::ndShapeCompound(const ndShapeCompound& source)
	:ndShape(source)
	,m_array()
	,m_treeEntropy(ndFloat64(0.0f))
	,m_boxMinRadius(ndFloat32(0.0f))
	,m_boxMaxRadius(ndFloat32(0.0f))
	,m_root(nullptr)
	,m_idIndex(0)
	,m_owner(nullptr)
	,m_isStatic(source.m_isStatic)
{
	ndAssert(0);
}

ndShapeCompound::ndShapeCompound(const ndShapeCompound& source, const ndShapeInstance* const parent)
	:ndShape(source)
	,m_array()
	,m_treeEntropy(ndFloat64(0.0f))
	,m_boxMinRadius(ndFloat32(0.0f))
	,m_boxMaxRadius(ndFloat32(0.0f))
	,m_root(nullptr)
	,m_idIndex(0)
	,m_owner(const_cast<ndShapeInstance*>(parent))
	,m_isStatic(source.m_isStatic)
{
	ndTreeArray::Iterator iter(source.m_array);
	for (iter.Begin(); iter; iter++) 
	{
		ndNodeBase* const node = iter.GetNode()->GetInfo();
		ndShapeInstance* const shape = node->GetShape();
		ndNodeBase* const newNode = new ndNodeBase(shape);
		m_array.AddNode(newNode, iter.GetNode()->GetKey(), parent);
	}

	if (source.m_root) 
	{
		ndFixSizeArray<ndNodeBase*, D_COMPOUND_STACK_DEPTH> pool;
		ndFixSizeArray<ndNodeBase*, D_COMPOUND_STACK_DEPTH> parents;

		pool.PushBack(source.m_root);
		parents.PushBack(nullptr);

		while (pool.GetCount())
		{
			ndNodeBase* const sourceNode = pool.Pop();
			ndNodeBase* const stackParent = parents.Pop();
	
			ndNodeBase* parentNode = nullptr;
			if (sourceNode->m_type == m_node) 
			{
				parentNode = new ndNodeBase(*sourceNode);
				if (!sourceNode->m_parent) 
				{
					m_root = parentNode;
				}
				else 
				{
					parentNode->m_parent = stackParent;
					if (parentNode->m_parent)
					{
						if (sourceNode->m_parent->m_left == sourceNode) 
						{
							parentNode->m_parent->m_left = parentNode;
						}
						else 
						{
							ndAssert(sourceNode->m_parent->m_right == sourceNode);
							parentNode->m_parent->m_right = parentNode;
						}
					}
				}
			}
			else 
			{
				ndNodeBase* const node = m_array.Find(sourceNode->m_myNode->GetKey())->GetInfo();
				ndAssert(node);
				node->m_parent = stackParent;
				if (node->m_parent) 
				{
					if (sourceNode->m_parent->m_left == sourceNode) 
					{
						node->m_parent->m_left = node;
					}
					else 
					{
						ndAssert(sourceNode->m_parent->m_right == sourceNode);
						node->m_parent->m_right = node;
					}
				}
				else 
				{
					m_root = node;
				}
			}
	
			if (sourceNode->m_left) 
			{
				parents.PushBack(parentNode);
				pool.PushBack(sourceNode->m_left);
			}
	
			if (sourceNode->m_right) 
			{
				parents.PushBack(parentNode);
				pool.PushBack(sourceNode->m_right);
			}
		}
	}
}

ndShapeCompound::~ndShapeCompound()
{
	if (m_root) 
	{
		delete m_root;
	}
}

ndShape* ndShapeCompound::Clone(ndShapeInstance* const parent) const
{
	return new ndShapeCompound(*this, parent);
}

ndShapeCompound* ndShapeCompound::GetAsShapeCompound()
{
	return this;
}

ndShapeInfo ndShapeCompound::GetShapeInfo() const
{
	ndShapeInfo info(ndShape::GetShapeInfo());
	info.m_compound.m_noUsed = 0;
	return info;
}

void ndShapeCompound::DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const
{
	ndTreeArray::Iterator iter(m_array);
	for (iter.Begin(); iter; iter++) 
	{
		ndShapeInstance* const collision = iter.GetNode()->GetInfo()->GetShape();
		collision->DebugShape(matrix, debugCallback);
	}
}

bool ndShapeCompound::IsStatic() const
{
	return m_isStatic;
}

ndFloat32 ndShapeCompound::GetVolume() const
{
	ndFloat32 volume = ndFloat32(0.0f);
	ndTreeArray::Iterator iter(m_array);
	for (iter.Begin(); iter; iter++)
	{
		const ndShapeInstance* const collision = iter.GetNode()->GetInfo()->GetShape();
		volume += collision->GetVolume();
	}
	return volume;
}

ndFloat32 ndShapeCompound::GetBoxMinRadius() const
{
	return m_boxMinRadius;
}

ndFloat32 ndShapeCompound::GetBoxMaxRadius() const
{
	return m_boxMaxRadius;
}

void ndShapeCompound::CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const
{
	if (m_root) 
	{
		const ndVector origin(matrix.TransformVector(m_root->m_origin));
		const ndVector size(matrix.m_front.Abs().Scale(m_root->m_size.m_x) + matrix.m_up.Abs().Scale(m_root->m_size.m_y) + matrix.m_right.Abs().Scale(m_root->m_size.m_z));
		p0 = (origin - size) & ndVector::m_triplexMask;
		p1 = (origin + size) & ndVector::m_triplexMask;
	}
	else 
	{
		p0 = ndVector::m_zero;
		p1 = ndVector::m_zero;
	}
}

ndVector ndShapeCompound::SupportVertex(const ndVector& dir) const
{
	ndVector supportVector(ndVector::m_zero);
	ndFloat32 maxDist2 = ndFloat32(-1.0e20f);

	ndAssert(dir.m_w == ndFloat32(0.0f));
	if (m_array.GetCount() < 8)
	{
		// brute force calculation
		ndTreeArray::Iterator iter(m_array);
		for (iter.Begin(); iter; iter++)
		{
			ndNodeBase* const node = iter.GetNode()->GetInfo();
			ndShapeInstance* const collision = node->GetShape();

			const ndMatrix& localMatrix = collision->GetLocalMatrix();
			const ndVector localDir(localMatrix.UnrotateVector(dir));
			const ndVector supportPoint(localMatrix.TransformVector(collision->SupportVertex(localDir)));
			ndFloat32 dist2 = supportPoint.DotProduct(dir).GetScalar();
			if (dist2 > maxDist2)
			{
				maxDist2 = dist2;
				supportVector = supportPoint;
			}
		}
	}
	else
	{
		// the tree is deep enought that th eoverhead of traversin 
		// the hiearchy is smaller that teh brrute force
		ndAssert(m_root);

		const ndVector mask(dir < ndVector::m_zero);
		auto BoxSupport = [&dir, &mask](const ndNodeBase* const node)
		{
			const ndVector support (node->m_p1.Select(node->m_p0, mask));
			return support.DotProduct(dir).GetScalar();
		};

		ndFixSizeArray<ndFloat32, D_COMPOUND_STACK_DEPTH> distance;
		ndFixSizeArray<const ndNodeBase*, D_COMPOUND_STACK_DEPTH> stackPool;

		stackPool.PushBack(m_root);
		distance.PushBack(BoxSupport(m_root));

		ndFloat32 limitingDist2 = ndFloat32(-1.0e20f);
		while (stackPool.GetCount())
		{
			const ndNodeBase* const self = stackPool.Pop();
			ndAssert(self);
			ndFloat32 dist = distance.Pop();

			if (dist < limitingDist2)
			{
				break;
			}
			if (self->m_type == m_leaf)
			{
				ndShapeInstance* const collision = self->GetShape();
				const ndMatrix& localMatrix = collision->GetLocalMatrix();
				const ndVector localDir(localMatrix.UnrotateVector(dir));
				const ndVector supportPoint(localMatrix.TransformVector(collision->SupportVertex(localDir)));
				ndFloat32 dist2 = supportPoint.DotProduct(dir).GetScalar();
				if (dist2 > maxDist2)
				{
					maxDist2 = dist2;
					supportVector = supportPoint;
				}
				limitingDist2 = ndMax (limitingDist2, dist2);
			}
			else
			{
				ndAssert(self->m_type == m_node);
				const ndNodeBase* const left = self->m_left;
				ndAssert(left);
				ndFloat32 dist1 = BoxSupport(left);
				if (dist1 > limitingDist2)
				{
					stackPool.PushBack(left);
					distance.PushBack(dist1);
					ndInt32 j = stackPool.GetCount() - 1;
					for (; j && (dist1 < distance[j - 1]); --j)
					{
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					stackPool[j] = left;
					distance[j] = dist1;
				}
			
				const ndNodeBase* const right = self->m_right;
				ndAssert(right);
				dist1 = BoxSupport(right);
				if (dist1 > limitingDist2)
				{
					stackPool.PushBack(right);
					distance.PushBack(dist1);
					ndInt32 j = stackPool.GetCount() - 1;
					for (; j && (dist1 < distance[j - 1]); --j)
					{
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					stackPool[j] = right;
					distance[j] = dist1;
				}
			}
		}
	}
	return supportVector;
}

ndVector ndShapeCompound::SupportVertexSpecialProjectPoint(const ndVector&, const ndVector& dir) const
{ 
	return SupportVertex(dir);
}

ndVector ndShapeCompound::SupportVertexSpecial(const ndVector& dir, ndFloat32) const
{
	return SupportVertex(dir);
}

ndInt32 ndShapeCompound::CalculatePlaneIntersection(const ndVector&, const ndVector&, ndVector* const) const
{
	ndAssert(0);
	return 0;
}

ndVector ndShapeCompound::CalculateVolumeIntegral(const ndMatrix& globalMatrix, const ndVector& globalPlane, const ndShapeInstance& parentScale) const
{
	ndFloat32 volume = ndFloat32(0.0f);
	ndVector volumePosit(ndVector::m_zero);
	ndTreeArray::Iterator iter(m_array);
	for (iter.Begin(); iter; iter++)
	{
		ndNodeBase* const node = iter.GetNode()->GetInfo();
		const ndShapeInstance* const collision = node->GetShape();

		const ndMatrix matrix(collision->GetLocalMatrix() * globalMatrix);
		const ndVector partialVolume(collision->GetShape()->CalculateVolumeIntegral(matrix, globalPlane, parentScale));

		volume += partialVolume.m_w;
		volumePosit += partialVolume.Scale(partialVolume.m_w);
	}

	if (volume > ndFloat32(1.0e-5f))
	{
		volumePosit = volumePosit.Scale(ndFloat32(1.0f) / volume);
		volumePosit.m_w = volume;
	}
	else
	{
		volumePosit = ndVector::m_zero;
	}

	return volumePosit;
}

ndFloat32 ndShapeCompound::RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut, ndShapeInstance* const) const
{
	if (!m_root) 
	{
		return ndFloat32 (1.2f);
	}

	ndFixSizeArray<ndFloat32, D_COMPOUND_STACK_DEPTH> distance;
	ndFixSizeArray<const ndNodeBase*, D_COMPOUND_STACK_DEPTH> stackPool;

	ndFastRay ray (localP0, localP1);

	stackPool.PushBack(m_root);
	distance.PushBack (ray.BoxIntersect(m_root->m_p0, m_root->m_p1));

	while (stackPool.GetCount())
	{
		ndFloat32 dist = distance.Pop();
		const ndNodeBase* const self = stackPool.Pop();

		if (dist > maxT) 
		{
			break;
		} 
		else 
		{
			ndAssert (self);
			if (self->m_type == m_leaf)
			{
				ndContactPoint tmpContactOut;
				tmpContactOut.Init();
				ndShapeInstance* const shapeInstance = self->GetShape();
				const ndVector p0 (shapeInstance->GetLocalMatrix().UntransformVector (localP0) & ndVector::m_triplexMask);
				const ndVector p1 (shapeInstance->GetLocalMatrix().UntransformVector (localP1) & ndVector::m_triplexMask);
				ndFloat32 param = shapeInstance->RayCast(callback, p0, p1, body, tmpContactOut, maxT);
				if (param < maxT) 
				{
					maxT = param;
					contactOut.m_normal = shapeInstance->GetLocalMatrix().RotateVector (tmpContactOut.m_normal);
					contactOut.m_shapeId0 = tmpContactOut.m_shapeId0;
					contactOut.m_shapeId1 = tmpContactOut.m_shapeId0;
					contactOut.m_shapeInstance0 = shapeInstance;
					contactOut.m_shapeInstance1 = shapeInstance;
				}
			} 
			else 
			{
				ndAssert (self->m_type == m_node);
				const ndNodeBase* const left = self->m_left;
				ndAssert (left);
				ndFloat32 dist1 = ray.BoxIntersect(left->m_p0, left->m_p1);
				if (dist1 < maxT) 
				{
					stackPool.PushBack(left);
					distance.PushBack(dist1);
					ndInt32 j = stackPool.GetCount() - 1;
					for ( ; j && (dist1 > distance[j - 1]); --j) 
					{
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					stackPool[j] = left;
					distance[j] = dist1;
				}
				
				const ndNodeBase* const right = self->m_right;
				ndAssert (right);
				dist1 = ray.BoxIntersect(right->m_p0, right->m_p1);
				if (dist1 < maxT) 
				{
					stackPool.PushBack(right);
					distance.PushBack(dist1);
					ndInt32 j = stackPool.GetCount() - 1;
					for ( ; j && (dist1 > distance[j - 1]); --j) 
					{
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					stackPool[j] = right;
					distance[j] = dist1;
				}
			}
		}
	}
	return maxT;
}

ndFloat32 ndShapeCompound::CalculateSurfaceArea(ndNodeBase* const node0, ndNodeBase* const node1, ndVector& minBox, ndVector& maxBox) const
{
	minBox = node0->m_p0.GetMin(node1->m_p0);
	maxBox = node0->m_p1.GetMax(node1->m_p1);
	ndVector side0(ndVector::m_half * (maxBox - minBox));
	return side0.DotProduct(side0.ShiftTripleRight()).GetScalar();
}

void ndShapeCompound::ImproveNodeFitness(ndNodeBase* const node) const
{
	ndAssert(node->m_left);
	ndAssert(node->m_right);

	if (node->m_parent) 
	{
		if (node->m_parent->m_left == node) 
		{
			ndFloat32 cost0 = node->m_area;

			ndVector cost1P0;
			ndVector cost1P1;
			ndFloat32 cost1 = CalculateSurfaceArea(node->m_right, node->m_parent->m_right, cost1P0, cost1P1);

			ndVector cost2P0;
			ndVector cost2P1;
			ndFloat32 cost2 = CalculateSurfaceArea(node->m_left, node->m_parent->m_right, cost2P0, cost2P1);

			ndAssert(node->m_parent->m_p0.m_w == ndFloat32(0.0f));
			ndAssert(node->m_parent->m_p1.m_w == ndFloat32(0.0f));

			if ((cost1 <= cost0) && (cost1 <= cost2)) 
			{
				ndNodeBase* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area;
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) 
				{
					if (parent->m_parent->m_left == parent) 
					{
						parent->m_parent->m_left = node;
					}
					else 
					{
						ndAssert(parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_right->m_parent = parent;
				parent->m_left = node->m_right;
				node->m_right = parent;
				parent->m_p0 = cost1P0;
				parent->m_p1 = cost1P1;
				parent->m_area = cost1;
				parent->m_size = (parent->m_p1 - parent->m_p0) * ndVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * ndVector::m_half;

			}
			else if ((cost2 <= cost0) && (cost2 <= cost1)) 
			{
				ndNodeBase* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area;
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) 
				{
					if (parent->m_parent->m_left == parent) 
					{
						parent->m_parent->m_left = node;
					}
					else 
					{
						ndAssert(parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_left->m_parent = parent;
				parent->m_left = node->m_left;
				node->m_left = parent;

				parent->m_p0 = cost2P0;
				parent->m_p1 = cost2P1;
				parent->m_area = cost2;
				parent->m_size = (parent->m_p1 - parent->m_p0) * ndVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * ndVector::m_half;
			}
		}
		else 
		{
			ndFloat32 cost0 = node->m_area;

			ndVector cost1P0;
			ndVector cost1P1;
			ndFloat32 cost1 = CalculateSurfaceArea(node->m_left, node->m_parent->m_left, cost1P0, cost1P1);

			ndVector cost2P0;
			ndVector cost2P1;
			ndFloat32 cost2 = CalculateSurfaceArea(node->m_right, node->m_parent->m_left, cost2P0, cost2P1);

			if ((cost1 <= cost0) && (cost1 <= cost2)) 
			{
				ndNodeBase* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area;
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) 
				{
					if (parent->m_parent->m_left == parent) 
					{
						parent->m_parent->m_left = node;
					}
					else 
					{
						ndAssert(parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_left->m_parent = parent;
				parent->m_right = node->m_left;
				node->m_left = parent;

				parent->m_p0 = cost1P0;
				parent->m_p1 = cost1P1;
				parent->m_area = cost1;
				parent->m_size = (parent->m_p1 - parent->m_p0) * ndVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * ndVector::m_half;

			}
			else if ((cost2 <= cost0) && (cost2 <= cost1)) 
			{
				ndNodeBase* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area;
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) 
				{
					if (parent->m_parent->m_left == parent) 
					{
						parent->m_parent->m_left = node;
					}
					else 
					{
						ndAssert(parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_right->m_parent = parent;
				parent->m_right = node->m_right;
				node->m_right = parent;

				parent->m_p0 = cost2P0;
				parent->m_p1 = cost2P1;
				parent->m_area = cost2;
				parent->m_size = (parent->m_p1 - parent->m_p0) * ndVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * ndVector::m_half;
			}
		}
	}
}

ndFloat64 ndShapeCompound::CalculateEntropy(ndInt32 count, ndNodeBase** array)
{
	ndFloat64 cost0 = ndFloat64(1.0e20f);
	ndFloat64 cost1 = cost0;
	do {
		cost1 = cost0;
		for (ndInt32 i = 0; i < count; ++i) 
		{
			ndNodeBase* const node = array[i];
			ImproveNodeFitness(node);
		}

		cost0 = ndFloat64(0.0f);
		for (ndInt32 i = 0; i < count; ++i)
		{
			ndNodeBase* const node = array[i];
			cost0 += ndFloat64(node->m_area);
		}
	} while (cost0 < (cost1 * ndFloat64(0.9999f)));
	return cost0;
}


ndShapeCompound::ndNodeBase* ndShapeCompound::BuildTopDown(ndNodeBase** const leafArray, ndInt32 firstBox, ndInt32 lastBox, ndNodeBase** rootNodesMemory, ndInt32& rootIndex)
{
	ndAssert(lastBox >= 0);
	ndAssert(firstBox >= 0);
	
	if (lastBox == firstBox) 
	{
		ndNodeBase* const node = leafArray[firstBox];
		return node;
	}
	
	ndSpliteInfo info(&leafArray[firstBox], lastBox - firstBox + 1);
		
	ndNodeBase* const parent = rootNodesMemory[rootIndex];
	rootIndex++;
	parent->m_parent = nullptr;
		
	parent->SetBox(info.m_p0, info.m_p1);
	parent->m_right = BuildTopDown(leafArray, firstBox + info.m_axis, lastBox, rootNodesMemory, rootIndex);
	parent->m_right->m_parent = parent;
		
	parent->m_left = BuildTopDown(leafArray, firstBox, firstBox + info.m_axis - 1, rootNodesMemory, rootIndex);
	parent->m_left->m_parent = parent;
	return parent;
}

ndShapeCompound::ndNodeBase* ndShapeCompound::BuildTopDownBig(ndNodeBase** const leafArray, ndInt32 firstBox, ndInt32 lastBox, ndNodeBase** rootNodesMemory, ndInt32& rootIndex)
{
	if (lastBox == firstBox) 
	{
		return BuildTopDown(leafArray, firstBox, lastBox, rootNodesMemory, rootIndex);
	}

	ndInt32 midPoint = -1;
	const ndFloat32 scale = ndFloat32(10.0f);
	const ndFloat32 scale2 = ndFloat32(3.0f) * scale * scale;
	const ndInt32 count = lastBox - firstBox;
	for (ndInt32 i = 0; i < count; ++i) 
	{
		const ndNodeBase* const node0 = leafArray[firstBox + i];
		const ndNodeBase* const node1 = leafArray[firstBox + i + 1];
		if (node1->m_area > (scale2 * node0->m_area)) 
		{
			midPoint = i;
			break;
		}
	}

	if (midPoint == -1) 
	{
		return BuildTopDown(leafArray, firstBox, lastBox, rootNodesMemory, rootIndex);
	}

	ndNodeBase* const parent = rootNodesMemory[rootIndex];
	rootIndex++;
	parent->m_parent = nullptr;
	
	ndVector minP(ndFloat32(1.0e15f));
	ndVector maxP(-ndFloat32(1.0e15f));
	for (ndInt32 i = 0; i <= count; ++i) 
	{
		const ndNodeBase* const node = leafArray[firstBox + i];
		ndAssert(node->m_shapeInstance);
		minP = minP.GetMin(node->m_p0);
		maxP = maxP.GetMax(node->m_p1);
	}
	
	parent->SetBox(minP, maxP);
	parent->m_left = BuildTopDown(leafArray, firstBox, firstBox + midPoint, rootNodesMemory, rootIndex);
	parent->m_left->m_parent = parent;
	
	parent->m_right = BuildTopDownBig(leafArray, firstBox + midPoint + 1, lastBox, rootNodesMemory, rootIndex);
	parent->m_right->m_parent = parent;
	return parent;
}

ndShapeInstance* ndShapeCompound::GetShapeInstance(ndTreeArray::ndNode* const node)
{
	return node->GetInfo()->GetShape();
}

const ndShapeInstance* ndShapeCompound::GetShapeInstance(ndTreeArray::ndNode* const node) const
{
	return node->GetInfo()->GetShape();
}

ndShapeCompound::ndTreeArray::ndNode* ndShapeCompound::AddCollision(ndShapeInstance* const subInstance)
{
	ndNodeBase* const newNode = new ndNodeBase(subInstance);
	m_array.AddNode(newNode, m_idIndex, *m_owner);

	m_idIndex++;
	
	if (!m_root) 
	{
		m_root = newNode;
	}
	else 
	{
		ndVector p0;
		ndVector p1;
		ndNodeBase* sibling = m_root;
		ndFloat32 surfaceArea = CalculateSurfaceArea(newNode, sibling, p0, p1);
		while (sibling->m_left && sibling->m_right) 
		{
			if (surfaceArea > sibling->m_area) 
			{
				break;
			}
	
			sibling->SetBox(p0, p1);
	
			ndVector leftP0;
			ndVector leftP1;
			ndFloat32 leftSurfaceArea = CalculateSurfaceArea(newNode, sibling->m_left, leftP0, leftP1);
	
			ndVector rightP0;
			ndVector rightP1;
			ndFloat32 rightSurfaceArea = CalculateSurfaceArea(newNode, sibling->m_right, rightP0, rightP1);
	
			if (leftSurfaceArea < rightSurfaceArea) 
			{
				sibling = sibling->m_left;
				p0 = leftP0;
				p1 = leftP1;
				surfaceArea = leftSurfaceArea;
			}
			else 
			{
				sibling = sibling->m_right;
				p0 = rightP0;
				p1 = rightP1;
				surfaceArea = rightSurfaceArea;
			}
		}
	
		if (!sibling->m_parent) 
		{
			m_root = new ndNodeBase(sibling, newNode);
		}
		else 
		{
			ndNodeBase* const parent = sibling->m_parent;
			if (parent->m_left == sibling) 
			{
				ndNodeBase* const node = new ndNodeBase(sibling, newNode);
				parent->m_left = node;
				node->m_parent = parent;
			}
			else 
			{
				ndAssert(parent->m_right == sibling);
				ndNodeBase* const node = new ndNodeBase(sibling, newNode);
				parent->m_right = node;
				node->m_parent = parent;
			}
		}
	}
	
	return newNode->m_myNode;
}

void ndShapeCompound::RemoveNode(ndTreeArray::ndNode* const node)
{
	m_array.Remove(node);
	ndNodeBase* const treeNode = node->GetInfo();

	if (!treeNode->m_parent)
	{
		delete (m_root);
		m_root = nullptr;
	}
	else if (!treeNode->m_parent->m_parent)
	{
		ndNodeBase* const root = m_root;
		if (treeNode->m_parent->m_left == treeNode)
		{
			m_root = treeNode->m_parent->m_right;
			treeNode->m_parent->m_right = nullptr;
		}
		else
		{
			ndAssert(treeNode->m_parent->m_right == treeNode);
			m_root = treeNode->m_parent->m_left;
			treeNode->m_parent->m_left = nullptr;
		}
		m_root->m_parent = nullptr;
		delete (root);
	}
	else
	{
		ndNodeBase* const root = treeNode->m_parent->m_parent;
		if (treeNode->m_parent == root->m_left)
		{
			if (treeNode->m_parent->m_right == treeNode)
			{
				root->m_left = treeNode->m_parent->m_left;
				treeNode->m_parent->m_left = nullptr;
			}
			else
			{
				ndAssert(treeNode->m_parent->m_left == treeNode);
				root->m_left = treeNode->m_parent->m_right;
				treeNode->m_parent->m_right = nullptr;
			}
			root->m_left->m_parent = root;
		}
		else
		{
			if (treeNode->m_parent->m_right == treeNode)
			{
				root->m_right = treeNode->m_parent->m_left;
				treeNode->m_parent->m_left = nullptr;
			}
			else
			{
				ndAssert(treeNode->m_parent->m_left == treeNode);
				root->m_right = treeNode->m_parent->m_right;
				treeNode->m_parent->m_right = nullptr;
			}
			root->m_right->m_parent = root;
		}
		delete (treeNode->m_parent);
	}
}

void ndShapeCompound::MassProperties()
{
#ifdef _DEBUG
	//	ndVector origin_ (ndVector::m_zero);
	//	ndVector inertia_ (ndVector::m_zero);
	//	ndVector crossInertia_ (ndVector::m_zero);
	//	dgPolyhedraMassProperties localData;
	//	DebugCollision (dgGetIdentityMatrix(), CalculateInertia, &localData);
	//	ndFloat32 volume_ = localData.MassProperties (origin_, inertia_, crossInertia_);
	//	ndAssert (volume_ > ndFloat32 (0.0f));
	//	ndFloat32 invVolume_ = ndFloat32 (1.0f)/volume_;
	//	m_centerOfMass = origin_.Scale (invVolume_);
	//	m_centerOfMass.m_w = volume_;
	//	m_inertia = inertia_.Scale (invVolume_);
	//	m_crossInertia = crossInertia_.Scale(invVolume_);
#endif

	ndFloat32 volume = ndFloat32(0.0f);
	ndVector origin(ndVector::m_zero);
	ndVector inertiaII(ndVector::m_zero);
	ndVector inertiaIJ(ndVector::m_zero);
	ndTreeArray::Iterator iter(m_array);
	bool hasVolume = true;
	for (iter.Begin(); iter; iter++) 
	{
		ndShapeInstance* const collision = iter.GetNode()->GetInfo()->GetShape();
		ndMatrix shapeInertia(collision->CalculateInertia());
		ndFloat32 shapeVolume = collision->GetVolume();

		hasVolume = hasVolume && (collision->GetShape()->GetAsShapeStaticMesh() != nullptr);
		volume += shapeVolume;
		origin += shapeInertia.m_posit.Scale(shapeVolume);
		inertiaII += ndVector(shapeInertia[0][0], shapeInertia[1][1], shapeInertia[2][2], ndFloat32(0.0f)).Scale(shapeVolume);
		inertiaIJ += ndVector(shapeInertia[1][2], shapeInertia[0][2], shapeInertia[0][1], ndFloat32(0.0f)).Scale(shapeVolume);
	}

	m_inertia = ndVector::m_zero;
	m_crossInertia = ndVector::m_zero;
	m_centerOfMass = ndVector::m_zero;
	if (hasVolume && (volume > ndFloat32(0.0f)))
	{
		ndFloat32 invVolume = ndFloat32(1.0f) / volume;
		m_inertia = inertiaII.Scale(invVolume);
		m_crossInertia = inertiaIJ.Scale(invVolume);
		m_centerOfMass = origin.Scale(invVolume);
		m_centerOfMass.m_w = volume;
		ndShape::MassProperties();
	}
}

void ndShapeCompound::SetSubShapeOwner(ndBodyKinematic* const body)
{
	ndTreeArray::Iterator iter(m_array);
	for (iter.Begin(); iter; iter++)
	{
		ndNodeBase* const node = iter.GetNode()->GetInfo();
		ndShapeInstance* const collision = node->GetShape();
		collision->m_ownerBody = body;
	}
}

void ndShapeCompound::ApplyScale(const ndVector& scale)
{
	ndTreeArray::Iterator iter(m_array);

	ndMatrix scaleMatrix(ndGetIdentityMatrix());
	scaleMatrix[0][0] = scale.m_x;
	scaleMatrix[1][1] = scale.m_y;
	scaleMatrix[2][2] = scale.m_z;
	for (iter.Begin(); iter; iter++) 
	{
		ndNodeBase* const node = iter.GetNode()->GetInfo();
		ndShapeInstance* const collision = node->GetShape();
		const ndMatrix matrix(collision->GetLocalMatrix() * scaleMatrix);
		collision->SetLocalMatrix(ndGetIdentityMatrix());
		collision->SetGlobalScale(matrix);
	}
	m_treeEntropy = ndFloat64(0.0f);
	EndAddRemove();
}

ndFloat32 ndShapeCompound::CalculateMassProperties(const ndMatrix& offset, ndVector& inertia, ndVector& crossInertia, ndVector& centerOfMass) const
{
	class ndCalculateMassProperties: public ndShapeDebugNotify
	{
		public:

		virtual void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceArray, const ndEdgeType* const) override
		{
			m_localData.AddInertiaAndCrossFace(vertexCount, faceArray);
		}

		ndPolyhedraMassProperties m_localData;
	};

	ndCalculateMassProperties massPropretiesCalculator;
	DebugShape(offset, massPropretiesCalculator);
	return massPropretiesCalculator.m_localData.MassProperties(centerOfMass, inertia, crossInertia);
}

ndMatrix ndShapeCompound::CalculateInertiaAndCenterOfMass(const ndMatrix& alignMatrix, const ndVector& localScale, const ndMatrix& matrix) const
{
	ndVector inertiaII;
	ndVector crossInertia;
	ndVector centerOfMass;
	ndMatrix scaledMatrix(matrix);
	scaledMatrix[0] = scaledMatrix[0].Scale(localScale.m_x);
	scaledMatrix[1] = scaledMatrix[1].Scale(localScale.m_y);
	scaledMatrix[2] = scaledMatrix[2].Scale(localScale.m_z);
	scaledMatrix = alignMatrix * scaledMatrix;

	ndFloat32 volume = CalculateMassProperties(scaledMatrix, inertiaII, crossInertia, centerOfMass);
	if (volume < D_MAX_MIN_VOLUME)
	{
		volume = D_MAX_MIN_VOLUME;
	}

	ndFloat32 invVolume = ndFloat32(1.0f) / volume;
	centerOfMass = centerOfMass.Scale(invVolume);
	centerOfMass.m_w = ndFloat32(1.0f);
	inertiaII = inertiaII.Scale(invVolume);
	crossInertia = crossInertia.Scale(invVolume);
	ndMatrix inertia(ndGetIdentityMatrix());
	inertia[0][0] = inertiaII[0];
	inertia[1][1] = inertiaII[1];
	inertia[2][2] = inertiaII[2];
	inertia[0][1] = crossInertia[2];
	inertia[1][0] = crossInertia[2];
	inertia[0][2] = crossInertia[1];
	inertia[2][0] = crossInertia[1];
	inertia[1][2] = crossInertia[0];
	inertia[2][1] = crossInertia[0];
	inertia[3] = centerOfMass;
	return inertia;
}

ndUnsigned64 ndShapeCompound::GetHash(ndUnsigned64 hash) const
{
	ndUnsigned64 crc = hash;
	ndShapeCompound::ndTreeArray::Iterator it(GetTree());

	//ndShapeCompound* const compoundShape = (ndShapeCompound*)this;
	ndShapeCompound* const compoundShape = const_cast<ndShapeCompound*>(this);
	for (it.Begin(); it; it++)
	{
		const ndShapeInstance* const childInstance = compoundShape->GetShapeInstance(it.GetNode());
		const ndShape* const childShape = childInstance->GetShape();
		crc = childShape->GetHash(crc);
	}
	return crc;
}

ndShapeInstance* ndShapeCompound::GetOwner() const
{
	return const_cast<ndShapeInstance*>(*m_owner);
}

ndSharedPtr<ndMeshCollisionShape> ndShapeCompound::GetMeshShape() const
{
	ndMeshCollisionShapeCompound* const shape = new ndMeshCollisionShapeCompound;
	//ndShapeCompound* const compoundShape = (ndShapeCompound*)this;
	ndShapeCompound* const compoundShape = const_cast<ndShapeCompound*>(this);

	ndShapeCompound::ndTreeArray::Iterator it(GetTree());
	for (it.Begin(); it; it++)
	{
		const ndShapeInstance* const childInstance = compoundShape->GetShapeInstance(it.GetNode());
		ndSharedPtr<ndMeshShapeInstance> subMeshShape(new ndMeshShapeInstance);
		childInstance->Serialize(*subMeshShape);
		shape->m_subShapes.Append(subMeshShape);
	}
	return ndSharedPtr<ndMeshCollisionShape>(shape);
}

void ndShapeCompound::BeginAddRemove()
{
	//ndAssert(m_ownerInstance);
}

#if defined (D_TOP_DOWN_COMPOUND_BUILD)
void ndShapeCompound::EndAddRemove()
{
	m_isStatic = false;
	if (m_root)
	{
		ndTreeArray::Iterator iter(m_array);
		for (iter.Begin(); iter; iter++)
		{
			ndNodeBase* const node = iter.GetNode()->GetInfo();
			node->CalculateAABB();
			m_isStatic = m_isStatic || node->m_shapeInstance->IsStatic();
		}

		ndInt32 nodeCount = 0;
		ndNodeBase** nodeArray = ndAlloca(ndNodeBase*, m_array.GetCount() + 10);
		ndFixSizeArray<ndNodeBase*, D_COMPOUND_STACK_DEPTH> stackBuffer;

		stackBuffer.PushBack(m_root);
		while (stackBuffer.GetCount())
		{
			ndNodeBase* const node = stackBuffer.Pop();

			if (node->m_type == m_node)
			{
				nodeArray[nodeCount] = node;
				nodeCount++;
				ndAssert(nodeCount <= m_array.GetCount());

				stackBuffer.PushBack(node->m_right);
				stackBuffer.PushBack(node->m_left);
			}
		}

		if (nodeCount)
		{
			ndFloat64 cost = CalculateEntropy(nodeCount, nodeArray);
			if ((cost > m_treeEntropy * ndFloat64(2.0f)) || (cost < m_treeEntropy * ndFloat64(0.5f)))
			{
				ndInt32 leafNodesCount = 0;
				ndNodeBase** leafArray = ndAlloca(ndNodeBase*, nodeCount + 12);
				for (ndInt32 i = 0; i < nodeCount; ++i)
				{
					ndNodeBase* const node = nodeArray[i];
					if (node->m_left->m_type == m_leaf)
					{
						leafArray[leafNodesCount] = node->m_left;
						leafNodesCount++;
						ndAssert(leafNodesCount <= (nodeCount + 1));
					}
					if (node->m_right->m_type == m_leaf)
					{
						leafArray[leafNodesCount] = node->m_right;
						leafNodesCount++;
						ndAssert(leafNodesCount <= (nodeCount + 1));
					}
				}

				ndSort(leafArray, leafNodesCount, [](const ndNodeBase* const elementA, const ndNodeBase* const elementB)
				{
					ndFloat32 areaA = elementA->m_area;
					ndFloat32 areaB = elementB->m_area;
					if (areaA < areaB)
					{
						return 1;
					}
					if (areaA > areaB)
					{
						return -1;
					}
					return 0;
				});

				ndInt32 rootIndex = 0;
				m_root = BuildTopDownBig(&leafArray[0], 0, leafNodesCount - 1, nodeArray, rootIndex);
				m_treeEntropy = CalculateEntropy(nodeCount, nodeArray);
			}
			while (m_root->m_parent)
			{
				m_root = m_root->m_parent;
			}
		}
		else
		{
			m_treeEntropy = ndFloat64(2.0f);
		}

		ndAssert(m_root->m_size.m_w == ndFloat32(0.0f));
		m_boxMinRadius = ndMin(ndMin(m_root->m_size.m_x, m_root->m_size.m_y), m_root->m_size.m_z);
		m_boxMaxRadius = ndSqrt(m_root->m_size.DotProduct(m_root->m_size).GetScalar());

		m_boxSize = m_root->m_size;
		m_boxOrigin = m_root->m_origin;
		MassProperties();
	}
}

#else

void ndShapeCompound::EndAddRemove()
{
	if (!m_root)
	{
		return;
	}

	enum
	{
		m_linkedCell,
		m_insideCell,
		m_outsideCell,
	};
	
	class ndBottomUpCell
	{
		public:
		//ndBottomUpCell(){}
	
		ndBottomUpCell(ndInt32 x, ndInt32 y, ndInt32 z, ndNodeBase* const node)
			:m_x(x)
			,m_y(y)
			,m_z(z)
			,m_node(node)
		{
		}
	
		union
		{
			ndInt32 m_index[3];
			struct
			{
				ndInt32 m_x;
				ndInt32 m_y;
				ndInt32 m_z;
			};
		};
		ndNodeBase* m_node;
	};
	
	class ndCellScanPrefix
	{
		public:
		//ndCellScanPrefix() {}
		ndCellScanPrefix(ndInt32 location, ndUnsigned32 cellTest)
			:m_location(location)
			,m_cellTest(cellTest)
		{
		}
		ndInt32 m_location : 30;
		ndUnsigned32 m_cellTest : 1;
	};
	
	auto CalculateSize = [](ndInt32 nodeCount, ndNodeBase** const buffer)
	{
		ndFloat32 minSize = ndFloat32(1.0e15f);
		for (ndInt32 i = 0; i < nodeCount; ++i)
		{
			const ndNodeBase* const node = buffer[i];
			const ndVector size(node->m_p1 - node->m_p0);
			ndFloat32 maxDim = ndMax(ndMax(size.m_x, size.m_y), size.m_z);
			minSize = ndMin(maxDim, minSize);
		}
		return minSize;
	};
	
	if (m_array.GetCount() == 1)
	{
		m_root = m_array.GetRoot()->GetInfo();
		return;
	}

	ndInt32 bufferSizeInElements = 2 * m_array.GetCount() + 16;
	ndInt32 bufferSizeInBytes = 2 * (sizeof (ndNodeBase*) + sizeof (ndBottomUpCell) + sizeof(ndCellScanPrefix));

	char* memory = ndAlloca(char, bufferSizeInBytes * bufferSizeInElements);

	ndNodeBase** const buffer = ndAlignPtr(ndNodeBase*, memory);
	ndNodeBase** const scratchBuffer = ndAlignPtr(ndNodeBase*, &buffer[bufferSizeInElements]);
	ndBottomUpCell* const cellBuffer0 = ndAlignPtr(ndBottomUpCell, &scratchBuffer[bufferSizeInElements]);
	ndBottomUpCell* const cellBuffer1 = ndAlignPtr(ndBottomUpCell, &cellBuffer0[bufferSizeInElements]);
	ndCellScanPrefix* const cellsCount = ndAlignPtr(ndCellScanPrefix, &cellBuffer1[bufferSizeInElements]);
	ndCellScanPrefix* const cellsCountScratch = ndAlignPtr(ndCellScanPrefix, &cellsCount[bufferSizeInElements]);
	
	ndNodeBase** leafNodes = buffer;
	ndNodeBase** freeNodes = buffer + m_array.GetCount();
	
	auto BuildSmallBvhTree = [&](ndInt32 spansCount)
	{
		auto MakeTwoNodesTree = [](ndNodeBase* const root, ndNodeBase* const left, ndNodeBase* const right)
		{
			ndAssert(!left->m_bhvLinked);
			ndAssert(!right->m_bhvLinked);
	
			root->Init(left, right);
			left->m_parent = root;
			right->m_parent = root;
	
			root->m_left = left;
			root->m_right = right;
		};
	
		auto MakeThreeNodesTree = [](ndNodeBase* const root, ndNodeBase* const subRoot, ndNodeBase* const node0, ndNodeBase* const node1, ndNodeBase* const node2)
		{
			class ndNodeOrder
			{
				public:
				ndVector m_p0;
				ndVector m_p1;
				ndNodeBase* m_node0;
				ndNodeBase* m_node1;
				ndNodeBase* m_node2;
				ndFloat32 m_area;
			};
	
			ndNodeOrder order[3];
	
			ndAssert(!node0->m_bhvLinked);
			ndAssert(!node1->m_bhvLinked);
			ndAssert(!node2->m_bhvLinked);
	
			order[0].m_node0 = node0;
			order[0].m_node1 = node1;
			order[0].m_node2 = node2;
	
			order[1].m_node0 = node1;
			order[1].m_node1 = node2;
			order[1].m_node2 = node0;
	
			order[2].m_node0 = node2;
			order[2].m_node1 = node0;
			order[2].m_node2 = node1;
	
			for (ndInt32 i = 0; i < 3; ++i)
			{
				order[i].m_p0 = order[i].m_node0->m_p0.GetMin(order[i].m_node1->m_p0);
				order[i].m_p1 = order[i].m_node0->m_p1.GetMax(order[i].m_node1->m_p1);
				const ndVector size(order[i].m_p1 - order[i].m_p0);
				order[i].m_area = size.DotProduct(size.ShiftTripleRight()).GetScalar();
			}
	
			if (order[2].m_area < order[1].m_area)
			{
				ndSwap(order[1], order[2]);
			}
			if (order[1].m_area < order[0].m_area)
			{
				ndSwap(order[1], order[0]);
			}
	
			subRoot->Init(order[0].m_node0, order[0].m_node1);
			subRoot->m_parent = root;
			subRoot->m_left = order[0].m_node0;
			subRoot->m_right = order[0].m_node1;
			subRoot->m_p0 = order[0].m_p0;
			subRoot->m_p1 = order[0].m_p1;
			subRoot->m_left->m_parent = subRoot;
			subRoot->m_right->m_parent = subRoot;
	
			root->Init(subRoot, order[0].m_node2);
			root->m_right = subRoot;
			root->m_left = order[0].m_node2;
			root->m_left->m_parent = root;
		};
	
		ndInt32 nextFreeNodes = 0;
		for (ndInt32 i = 0; i < spansCount; ++i)
		{
			const ndInt32 nodesCount = cellsCountScratch[i + 1].m_location - cellsCountScratch[i].m_location;
			if (nodesCount == 1)
			{
				const ndInt32 childIndex = cellsCount[i].m_location;
				ndNodeBase* const node0 = cellBuffer0[childIndex + 0].m_node;
				ndNodeBase* const node1 = cellBuffer0[childIndex + 1].m_node;
				
				ndNodeBase* const root = freeNodes[nextFreeNodes];
				nextFreeNodes++;
				ndAssert(root);
				MakeTwoNodesTree(root, node0, node1);
				root->m_bhvLinked = 0;
			}
			else if (nodesCount == 2)
			{
				const ndInt32 childIndex = cellsCount[i].m_location;
				
				ndNodeBase* const node0 = cellBuffer0[childIndex + 0].m_node;
				ndNodeBase* const node1 = cellBuffer0[childIndex + 1].m_node;
				ndNodeBase* const node2 = cellBuffer0[childIndex + 2].m_node;
				
				ndNodeBase* const root = freeNodes[nextFreeNodes];
				ndNodeBase* const subParent = freeNodes[nextFreeNodes + 1];
				nextFreeNodes += 2;
				
				MakeThreeNodesTree(root, subParent, node0, node1, node2);
				root->m_bhvLinked = 0;
			}
			else if (nodesCount > 2)
			{
				class ndBlockSegment
				{
					public:
					ndBlockSegment() {}
					ndBlockSegment(ndNodeBase* const parent, ndInt32 start, ndInt32 count)
						:m_parent(parent)
						,m_start(start)
						,m_count(count)
					{
					}
				
					ndNodeBase* m_parent;
					ndInt32 m_start;
					ndInt32 m_count;
				};
				
				ndFixSizeArray<ndBlockSegment, 128> stackPool;
				
				ndNodeBase* const rootNode = freeNodes[nextFreeNodes];
				nextFreeNodes++;
				rootNode->SetBox(ndVector::m_zero, ndVector::m_zero);
				rootNode->Init(rootNode, rootNode);
				
				rootNode->m_bhvLinked = 0;
				stackPool.PushBack(ndBlockSegment(rootNode, cellsCount[i].m_location, cellsCount[i + 1].m_location - cellsCount[i].m_location));
				
				while (stackPool.GetCount())
				{
					const ndBlockSegment block(stackPool.Pop());
					ndAssert(block.m_count > 2);
				
					ndVector minP(ndFloat32(1.0e15f));
					ndVector maxP(ndFloat32(-1.0e15f));
					ndVector median(ndVector::m_zero);
					ndVector varian(ndVector::m_zero);
				
					for (ndInt32 m = 0; m < block.m_count; ++m)
					{
						ndNodeBase* const node = cellBuffer0[block.m_start + m].m_node;
						minP = minP.GetMin(node->m_p0);
						maxP = maxP.GetMax(node->m_p1);
						ndVector p(ndVector::m_half * (node->m_p0 + node->m_p1));
						median += p;
						varian += p * p;
					}
				
					ndNodeBase* const root = block.m_parent;
					root->SetBox(minP, maxP);
				
					ndInt32 index = 0;
					ndFloat32 maxVarian = ndFloat32(-1.0e15f);
					varian = varian.Scale(ndFloat32(block.m_count)) - median * median;
					for (ndInt32 m = 0; m < 3; ++m)
					{
						if (varian[m] > maxVarian)
						{
							index = m;
							maxVarian = varian[m];
						}
					}
				
					ndInt32 index0 = block.m_start + block.m_count / 2;
					if (maxVarian > ndFloat32(1.0e-3f))
					{
						ndUnsigned32 scan[5];
						const ndFloat32 midPoint = median[index] / ndFloat32(block.m_count);
						ndCountingSortInPlace(&cellBuffer0[block.m_start], &cellBuffer1[0], block.m_count, scan, 2, [midPoint, index](const ndBottomUpCell& cell)
						{
							const ndNodeBase* const node = cell.m_node;
							const ndVector p(ndVector::m_half * (node->m_p0 + node->m_p1));
							ndInt32 key = p[index] >= midPoint;
							return key;
						});
				
						index0 = block.m_start + ndInt32(scan[1]);
						if (index0 == block.m_start)
						{
							index0 = block.m_start + block.m_count / 2;
						}
						if (index0 == (block.m_start + block.m_count))
						{
							index0 = block.m_start + block.m_count / 2;
						}
					}
				
					ndAssert(index0 > block.m_start);
					ndAssert(index0 < (block.m_start + block.m_count));
				
					ndInt32 count0 = index0 - block.m_start;
				
					ndAssert(count0);
					if (count0 == 1)
					{
						ndNodeBase* const node = cellBuffer0[block.m_start].m_node;
						node->m_bhvLinked = 1;
						node->m_parent = root;
						root->m_left = node;
					}
					else if (count0 == 2)
					{
						ndNodeBase* const node0 = cellBuffer0[block.m_start + 0].m_node;
						ndNodeBase* const node1 = cellBuffer0[block.m_start + 1].m_node;
						ndNodeBase* const leftChild = freeNodes[nextFreeNodes];
						nextFreeNodes++;
						MakeTwoNodesTree(leftChild, node0, node1);
						leftChild->m_parent = root;
						root->m_left = leftChild;
					}
					else if (count0 == 3)
					{
						ndNodeBase* const node0 = cellBuffer0[block.m_start + 0].m_node;
						ndNodeBase* const node1 = cellBuffer0[block.m_start + 1].m_node;
						ndNodeBase* const node2 = cellBuffer0[block.m_start + 2].m_node;
				
						ndNodeBase* const leftChild = freeNodes[nextFreeNodes];
						nextFreeNodes++;
				
						ndNodeBase* const children = freeNodes[nextFreeNodes];
						nextFreeNodes++;
				
						MakeThreeNodesTree(leftChild, children, node0, node1, node2);
						leftChild->m_parent = root;
						root->m_left = leftChild;
					}
					else
					{
						ndNodeBase* const leftChild = freeNodes[nextFreeNodes];
						nextFreeNodes++;
						leftChild->Init(root, root);
						leftChild->SetBox(ndVector::m_zero, ndVector::m_zero);
				
						leftChild->m_parent = root;
						leftChild->m_left = nullptr;
						leftChild->m_right = nullptr;
						root->m_left = leftChild;
						stackPool.PushBack(ndBlockSegment(leftChild, block.m_start, count0));
					}
				
					ndInt32 count1 = block.m_start + block.m_count - index0;
					ndAssert(count1);
					if (count1 == 1)
					{
						ndNodeBase* const node = cellBuffer0[index0].m_node;
						node->m_bhvLinked = 1;
						node->m_parent = root;
						root->m_right = node;
					}
					else if (count1 == 2)
					{
						ndNodeBase* const node0 = cellBuffer0[index0 + 0].m_node;
						ndNodeBase* const node1 = cellBuffer0[index0 + 1].m_node;
						ndNodeBase* const rightChild = freeNodes[nextFreeNodes];
						nextFreeNodes++;
						MakeTwoNodesTree(rightChild, node0, node1);
						rightChild->m_parent = root;
						root->m_right = rightChild;
					}
					else if (count1 == 3)
					{
						ndNodeBase* const node0 = cellBuffer0[index0 + 0].m_node;
						ndNodeBase* const node1 = cellBuffer0[index0 + 1].m_node;
						ndNodeBase* const node2 = cellBuffer0[index0 + 2].m_node;
				
						ndNodeBase* const rightChild = freeNodes[nextFreeNodes];
						nextFreeNodes++;
				
						ndNodeBase* const children = freeNodes[nextFreeNodes];
						nextFreeNodes++;
				
						MakeThreeNodesTree(rightChild, children, node0, node1, node2);
						rightChild->m_parent = root;
						root->m_right = rightChild;
					}
					else
					{
						ndNodeBase* const rightChild = freeNodes[nextFreeNodes];
						nextFreeNodes++;
						rightChild->Init(root, root);
						rightChild->SetBox(ndVector::m_zero, ndVector::m_zero);
				
						rightChild->m_parent = root;
						rightChild->m_left = nullptr;
						rightChild->m_right = nullptr;
						root->m_right = rightChild;
				
						stackPool.PushBack(ndBlockSegment(rightChild, index0, count1));
					}
				}
				rootNode->m_bhvLinked = 0;
			}
		}
		return nextFreeNodes;
	};
	
	ndInt32 index = 0;
	m_isStatic = false;
	ndTreeArray::Iterator iter(m_array);
	for (iter.Begin(); iter; iter++)
	{
		ndNodeBase* const node = iter.GetNode()->GetInfo();
		node->CalculateAABB();
		node->m_bhvLinked = 0;
		m_isStatic = m_isStatic || node->m_shapeInstance->IsStatic();
		buffer[index] = node;
		index++;
	}

	ndFixSizeArray<ndNodeBase*, D_COMPOUND_STACK_DEPTH> stackBuffer;
	stackBuffer.PushBack(m_root);
	while (stackBuffer.GetCount())
	{
		ndNodeBase* const node = stackBuffer.Pop();
	
		if (node->m_type == m_node)
		{
			buffer[index] = node;
			node->m_bhvLinked = 0;
			index++;
			ndAssert(index <= (m_array.GetCount() * 2));
	
			stackBuffer.PushBack(node->m_right);
			stackBuffer.PushBack(node->m_left);
		}
	}

	const ndInt32 count = m_array.GetCount();

	ndVector origin(ndFloat32(1.0e15f));
	for (ndInt32 i = 0; i < count; ++i)
	{
		const ndNodeBase* const node = buffer[i];
		origin = origin.GetMin(node->m_p0);
	}

	ndInt32 leafNodesCount = count;
	ndFloat32 minSize = CalculateSize(leafNodesCount, &leafNodes[0]);

	while (leafNodesCount > 1)
	{
		minSize = minSize * ndFloat32(2.0f);

		// classify nodes categorically
		const ndVector invSize(ndVector::m_triplexMask & ndVector(ndFloat32(1.0f) / minSize));

		ndInt32 m_code[4];
		ndUnsigned32 prefixScan[5];
		m_code[0] = m_outsideCell;
		m_code[1] = m_insideCell;
		m_code[2] = m_linkedCell;
		m_code[3] = m_linkedCell;
		ndCountingSortInPlace(leafNodes, &scratchBuffer[0], leafNodesCount, prefixScan, 2, [&](const ndNodeBase* const node)
		{
			const ndVector minPosit((invSize * (node->m_p0 - origin)).GetInt());
			const ndVector maxPosit((invSize * (node->m_p1 - origin)).GetInt());
			const ndInt32 x0 = ndInt32(minPosit.m_ix);
			const ndInt32 y0 = ndInt32(minPosit.m_iy);
			const ndInt32 z0 = ndInt32(minPosit.m_iz);
			const ndInt32 x1 = ndInt32(maxPosit.m_ix);
			const ndInt32 y1 = ndInt32(maxPosit.m_iy);
			const ndInt32 z1 = ndInt32(maxPosit.m_iz);

			ndAssert(x0 >= 0);
			ndAssert(y0 >= 0);
			ndAssert(z0 >= 0);
			ndAssert(x1 >= 0);
			ndAssert(y1 >= 0);
			ndAssert(z1 >= 0);

			const ndUnsigned32 test_x = ndUnsigned32(((x1 - x0) >> 1) == 0);
			const ndUnsigned32 test_y = ndUnsigned32(((y1 - y0) >> 1) == 0);
			const ndUnsigned32 test_z = ndUnsigned32(((z1 - z0) >> 1) == 0);
			const ndUnsigned32 test = test_x & test_y & test_z;
			const ndUnsigned32 codeIndex = node->m_bhvLinked * 2 + test;
			return m_code[codeIndex];
		});

		const ndInt32 insideCellsCount = ndInt32(prefixScan[m_insideCell + 1] - prefixScan[m_insideCell]);
		if (insideCellsCount)
		{
			// make integer grids
			ndInt32 maxGrid[3];
			maxGrid[0] = 0;
			maxGrid[1] = 0;
			maxGrid[2] = 0;

			const ndUnsigned32 linkedNodes = prefixScan[m_linkedCell + 1] - prefixScan[m_linkedCell];
			leafNodes += linkedNodes;
			leafNodesCount -= linkedNodes;
			ndAssert(leafNodesCount > 0);
			ndAssert(leafNodesCount <= count);
			if (leafNodesCount == 1)
			{
				break;
			}

			for (ndInt32 i = 0; i < insideCellsCount; ++i)
			{
				ndNodeBase* const node = leafNodes[i];

				const ndVector dist(node->m_p0 - origin);
				const ndVector posit(invSize * dist);
				const ndVector intPosit(posit.GetInt());
				cellBuffer0[i] = ndBottomUpCell(ndInt32(intPosit.m_ix), ndInt32(intPosit.m_iy), ndInt32(intPosit.m_iz), node);
				maxGrid[0] = ndMax(ndInt32(intPosit.m_ix), maxGrid[0]);
				maxGrid[1] = ndMax(ndInt32(intPosit.m_iy), maxGrid[1]);
				maxGrid[2] = ndMax(ndInt32(intPosit.m_iz), maxGrid[2]);
			}
			ndAssert(maxGrid[0] < 0x7fffffff);
			ndAssert(maxGrid[1] < 0x7fffffff);
			ndAssert(maxGrid[2] < 0x7fffffff);

			for (ndInt32 i = 0; i < 3; ++i)
			{
				ndInt64 shift = 0;
				do
				{
					ndCountingSortInPlace(&cellBuffer0[0], &cellBuffer1[0], insideCellsCount, nullptr, 8, [&](const ndBottomUpCell& cell)
					{
						ndInt32 val = cell.m_index[i] >> shift;
						return val & 0xff;
					});

					shift += 8;
				} while (maxGrid[i] >= (1 << shift));
			}

			cellBuffer0[insideCellsCount] = ndBottomUpCell(0x7fffffff, 0x7fffffff, 0x7fffffff, nullptr);

			cellsCount[0] = ndCellScanPrefix(0, 0);
			for (ndInt32 i = 0; i < insideCellsCount; ++i)
			{
				const ndBottomUpCell& cell0 = cellBuffer0[i];
				const ndBottomUpCell& cell1 = cellBuffer0[i + 1];
				const ndUnsigned8 test = ndUnsigned8((cell0.m_x == cell1.m_x) & (cell0.m_y == cell1.m_y) & (cell0.m_z == cell1.m_z) & (cell1.m_node != nullptr));
				cellsCount[i + 1] = ndCellScanPrefix(i + 1, test);
			}

			ndUnsigned32 cellPrefixScan[3];
			ndCountingSortInPlace(&cellsCount[0], &cellsCountScratch[0], insideCellsCount + 1, cellPrefixScan, 1, [](const ndCellScanPrefix& cell)
			{
				return ndInt32(cell.m_cellTest);
			});

			ndInt32 sum = 0;
			const ndInt32 batchCount = ndInt32(cellPrefixScan[1] - 1);
			for (ndInt32 i = 0; i < batchCount; ++i)
			{
				const ndInt32 nodesCount = cellsCount[i + 1].m_location - cellsCount[i].m_location - 1;
				cellsCountScratch[i] = ndCellScanPrefix(sum, 0);
				sum += nodesCount;
			}
			cellsCountScratch[batchCount] = ndCellScanPrefix(sum, 0);

			if (sum)
			{
				ndInt32 newNodes = BuildSmallBvhTree(batchCount);

				ndAssert(newNodes <= sum);
				freeNodes += newNodes;
				leafNodesCount += newNodes;

				// restart the size
				ndFloat32 restartSize = ndFloat32(1.0e15f);
				for (ndInt32 i = 0; i < leafNodesCount; ++i)
				{
					const ndNodeBase* const node = leafNodes[i];
					if (!node->m_bhvLinked)
					{
						const ndVector size(node->m_p1 - node->m_p0);
						ndFloat32 maxDim = ndMax(ndMax(size.m_x, size.m_y), size.m_z);
						restartSize = ndMin(maxDim, restartSize);
					}
				}
				minSize = ndMin(minSize, restartSize);
			}
		}
	}
	m_root = *leafNodes;
}

#endif
