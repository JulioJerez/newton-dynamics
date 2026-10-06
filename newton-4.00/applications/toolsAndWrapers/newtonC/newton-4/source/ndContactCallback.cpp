/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndModelStdafx.h"
#include "ndContactCallback.h"

ndApplicationMaterial::ndApplicationMaterial()
	:ndMaterial()
{
}

ndApplicationMaterial::ndApplicationMaterial(const ndApplicationMaterial& copy)
	:ndMaterial(copy)
{
}

ndApplicationMaterial::~ndApplicationMaterial()
{
}

bool ndApplicationMaterial::OnAabbOverlap(const ndBodyKinematic* const, const ndBodyKinematic* const) const
{
	return true;
}

bool ndApplicationMaterial::OnAabbOverlap(const ndContact* const, ndFloat32, const ndShapeInstance&, const ndShapeInstance&) const
{
	return true;
}

void ndApplicationMaterial::OnContactCallback(const ndContact* const, ndFloat32) const
{
}


ndMaterialGraph::ndMaterialGraph()
	:ndTree<ndApplicationMaterial*, ndMaterialHash, ndContainersFreeListAlloc<ndMaterialGraph*>>()
{
}

ndMaterialGraph::~ndMaterialGraph()
{
	Iterator it(*this);
	for (it.Begin(); it; it++)
	{
		ndApplicationMaterial* const material = it.GetNode()->GetInfo();
		delete material;
	}
}

ndMaterialGraph::ndNode* ndMaterialGraph::GetNode(ndUnsigned32 id0, ndUnsigned32 id1) const
{
	ndMaterialHash key(id0, id1);
	return Find(key);
}

ndApplicationMaterial& ndContactCallback::RegisterMaterial(const ndApplicationMaterial& material, ndUnsigned32 id0, ndUnsigned32 id1)
{
	//ndMaterialHash key(id0, id1);
	//ndMaterialGraph::ndNode* node = m_materialGraph.Find(key);
	ndMaterialGraph::ndNode* node = m_materialGraph.GetNode(id0, id1);
	if (!node)
	{
		ndApplicationMaterial* const materialCopy = material.Clone();
		node = m_materialGraph.Insert(materialCopy, ndMaterialHash(id0, id1));
	}
	return *node->GetInfo();
}

//**********************************************************************
// 
//**********************************************************************
ndContactCallback::ndContactCallback()
	:ndContactNotify(nullptr)
	,m_materialGraph()
	,m_defaultMaterial()
{
}

ndContactCallback::~ndContactCallback()
{
}

bool ndContactCallback::HasMaterial(ndUnsigned32 id0, ndUnsigned32 id1) const
{
	return m_materialGraph.GetNode(id0, id1) ? true : false;
}

ndMaterial* ndContactCallback::GetMaterial(ndUnsigned32 id0, ndUnsigned32 id1) const
{
	ndMaterialGraph::ndNode* const node = m_materialGraph.GetNode(id0, id1);
	return node ? node->GetInfo() : (ndMaterial*)&m_defaultMaterial;
}

ndMaterial* ndContactCallback::GetMaterial(const ndContact* const, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const
{
	ndMaterialGraph::ndNode* const node = m_materialGraph.GetNode(ndUnsigned32(instance0.GetMaterial().m_userId), ndUnsigned32(instance1.GetMaterial().m_userId));
	return node ? node->GetInfo() : (ndMaterial*)&m_defaultMaterial;
}

bool ndContactCallback::OnAabbOverlap(const ndBodyKinematic* const body0, const ndBodyKinematic* const body1) const
{
	const ndShapeInstance& instanceShape0 = body0->GetCollisionShape();
	const ndShapeInstance& instanceShape1 = body1->GetCollisionShape();
	ndMaterialGraph::ndNode* node = m_materialGraph.GetNode(ndUnsigned32(instanceShape0.m_shapeMaterial.m_userId), ndUnsigned32(instanceShape1.m_shapeMaterial.m_userId));
	if (node)
	{
		return node->GetInfo()->OnAabbOverlap(body0, body1);
	}

	return true;
}

bool ndContactCallback::OnAabbOverlap(const ndContact* const contactJoint, ndFloat32 timestep) const
{
	const ndApplicationMaterial* const material = (ndApplicationMaterial*)contactJoint->GetMaterial();
	ndAssert(material);

	const ndBodyKinematic* const body0 = contactJoint->GetBody0();
	const ndBodyKinematic* const body1 = contactJoint->GetBody1();
	const ndShapeInstance& instanceShape0 = body0->GetCollisionShape();
	const ndShapeInstance& instanceShape1 = body1->GetCollisionShape();
	return material->OnAabbOverlap(contactJoint, timestep, instanceShape0, instanceShape1);
}

bool ndContactCallback::OnCompoundSubShapeOverlap(const ndContact* const contactJoint, ndFloat32 timestep, const ndShapeInstance* const instance0, const ndShapeInstance* const instance1) const
{
	const ndApplicationMaterial* const material = (ndApplicationMaterial*)contactJoint->GetMaterial();
	ndAssert(material);
	return material->OnAabbOverlap(contactJoint, timestep, *instance0, *instance1);
}

void ndContactCallback::OnContactCallback(const ndContact* const contactJoint, ndFloat32 timestep) const
{
	const ndApplicationMaterial* const material = (ndApplicationMaterial*)contactJoint->GetMaterial();
	ndAssert(material);
	material->OnContactCallback(contactJoint, timestep);
}
