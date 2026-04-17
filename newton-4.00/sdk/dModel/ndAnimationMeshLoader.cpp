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
#include "ndAnimationSequence.h"
#include "ndAnimationMeshLoader.h"
#include "ndAnimationKeyframesTrack.h"

ndAnimationMeshLoader::ndAnimationMeshLoader()
	:ndMeshLoader()
{
}

ndAnimationMeshLoader::~ndAnimationMeshLoader()
{
}

const ndSharedPtr<ndAnimationSequence> ndAnimationMeshLoader::FindSequence(const ndString& fbxPathAnimName) const
{
	ndTree<ndSharedPtr<ndAnimationSequence>, ndString>::ndNode* const node = m_animationCache.Find(fbxPathAnimName);
	if (node)
	{
		return node->GetInfo();
	}
	return ndSharedPtr<ndAnimationSequence>(nullptr);
}

ndSharedPtr<ndAnimationSequence> ndAnimationMeshLoader::ImportFbxAnimation(const ndString& fbxPathAnimName)
{
	ndTree<ndSharedPtr<ndAnimationSequence>, ndString>::ndNode* node = m_animationCache.Find(fbxPathAnimName);
	if (!node)
	{
		ndFbxMeshLoader animLoader;
		ndSharedPtr<ndAnimationSequence> sequence (animLoader.LoadAnimation(fbxPathAnimName.GetStr()));
		if (sequence)
		{
			node = m_animationCache.Insert(sequence, fbxPathAnimName);
		}
	}
	return node ? node->GetInfo() : ndSharedPtr<ndAnimationSequence>(nullptr);
}

ndSharedPtr<ndAnimationSequence> ndAnimationMeshLoader::GetAnimationSequence(const ndString& pathAnimName)
{
	//ndAssert(0);
	//return ndSharedPtr<ndAnimationSequence>(nullptr);
	return ImportFbxAnimation(pathAnimName);
}

void ndAnimationMeshLoader::SetTranslationTracks(const ndString& boneName)
{
	ndTree<ndSharedPtr<ndAnimationSequence>, ndString>::Iterator it(m_animationCache);
	for (it.Begin(); it; it++)
	{
		const ndSharedPtr<ndAnimationSequence>& cycle = it.GetNode()->GetInfo();
		for (ndList<ndAnimationKeyFramesTrack>::ndNode* node = cycle->GetTracks().GetFirst(); node; node = node->GetNext())
		{
			ndAnimationKeyFramesTrack& track = node->GetInfo();
			ndString name(track.GetName());
			name.ToLower();
			if (name.Find(boneName) != -1)
			{
				ndAnimationKeyFramesTrack& translationTrack = cycle->GetTranslationTrack();
				ndVector translation(ndVector::m_zero);
				ndReal offset = ndReal(track.m_position[0].m_x);
				for (ndInt32 i = 0; i < track.m_position.GetCount(); ++i)
				{
					translation.m_x = track.m_position[i].m_x - offset;
					translationTrack.m_position.PushBack(translation);
					translationTrack.m_position.m_time.PushBack(track.m_position.m_time[i]);
					track.m_position[i].m_x = offset;
				}
				break;
			}
		}
	}
}

bool ndAnimationMeshLoader::ImportFbx(const ndString& fbxPathMeshName)
{
	ndFbxMeshLoader loader;
	m_mesh = ndSharedPtr<ndMesh>(loader.LoadMesh(fbxPathMeshName.GetStr(), false));

	ndInt32 index = 0;
	ndArray<ndVector> hullPoints;
	for (ndMesh* childMesh = m_mesh->IteratorFirst(); childMesh; childMesh = childMesh->IteratorNext(*m_mesh))
	{
		ndString name (childMesh->GetName());
		ndInt32 vhacd = name.Find("-vhacd");
		if (vhacd != -1)
		{
			// remove the -vhacd from name
			name.Replace (vhacd, 6, "-compound", 9);
			childMesh->SetName(name);
			ndSharedPtr<ndShapeInstance>compoundShapeInstance(childMesh->CreateCollisionConvexApproximation());
			ndShapeCompound* const compoundShape = compoundShapeInstance->GetShape()->GetAsShapeCompound();
			ndShapeCompound::ndTreeArray::Iterator it(compoundShape->GetTree());
			for (it.Begin(); it; it++)
			{
				ndShapeInstance* const subShape = compoundShape->GetShapeInstance(it.GetNode());
				ndAssert(subShape);

				ndSharedPtr<ndMesh> convexMesh(new ndMesh);
				char collisionshapeName[256];
				snprintf(collisionshapeName, 255, "collision%d-convexhull", index++);
				ndSharedPtr<ndMeshEffect> hullMesh(new ndMeshEffect(*subShape));

				convexMesh->SetMesh(hullMesh);
				convexMesh->SetMatrix(subShape->GetLocalMatrix());
				convexMesh->SetNodeType(ndMesh::m_collisionShape);
				convexMesh->SetName(collisionshapeName);
				childMesh->AddChild(convexMesh);
			}
		}
	}

	auto BindApplicationData = [](ndMesh* const node)
	{
		ndString name(node->GetName());
		name.ToLower();
		const char* const namePtr = name.GetStr();
		if (strstr(namePtr, "-rb"))
		{
			ndSharedPtr<ndShapeInstance> instance (node->CreateCollision());
			if (instance->GetShape()->GetAsShapeNull())
			{
				instance = node->CreateCollisionFromChildren();
			}

			ndBodyDynamic body;
			body.SetCollisionShape(**instance);
			body.SetMassMatrix(ndFloat32 (1.0f), **instance);
			body.Serialize(node);
		}

		ndMesh* parent = node->GetParent();
		for (; parent && !(*parent->GetRigidBody()); parent = parent->GetParent());
		if (parent)
		{
			ndSharedPtr<ndJointBilateralConstraint> joint(node->CreateJoint());
			ndSharedPtr<ndMeshJoint> meshJoint(joint->GetMeshJoint(node));
			node->SetJoint(meshJoint);
		}
	};
	m_mesh->NodeIterator(BindApplicationData);
	return m_mesh;
}

