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

#include "ndRenderStdafx.h"
#include "ndRender.h"
#include "ndRenderTexture.h"
#include "ndRenderPrimitive.h"
#include "ndRenderSceneNode.h"
#include "ndRenderMeshLoader.h"
#include "ndRenderTextureCache.h"

ndRenderMeshLoader::ndRenderMeshLoader(ndRender* const renderer)
	:ndAnimationMeshLoader()
	,m_owner(renderer) 
{
}

ndRenderMeshLoader::~ndRenderMeshLoader()
{
}

bool ndRenderMeshLoader::MeshToRenderSceneNode(const ndString& materialBasePath)
{
	m_renderMesh = ndSharedPtr<ndRenderSceneNode>(CreateRenderSceneMesh(m_owner, *m_mesh, materialBasePath));
	return m_renderMesh ? true : false;
}

ndSharedPtr<ndRenderSceneNode> ndRenderMeshLoader::CreateRenderSceneMesh(ndRender* const renderer, const ndMesh* const meshRoot, const ndString materialBasePath)
{
	class EntityMeshPair
	{
		public:
		EntityMeshPair()
			:m_mesh(nullptr)
			,m_entity(nullptr)
		{
		}

		EntityMeshPair(const EntityMeshPair& src)
			:m_mesh(src.m_mesh)
			,m_entity(src.m_entity)
		{
		}

		EntityMeshPair(ndSharedPtr<ndRenderSceneNode> entity, const ndMesh* const mesh)
			:m_mesh(mesh)
			,m_entity(entity)
		{
		}

		const ndMesh* m_mesh;
		ndSharedPtr<ndRenderSceneNode> m_entity;
	};

	ndList<EntityMeshPair> meshList;
	ndFixSizeArray<const ndMesh*, 1024> effectNodeList;
	ndList<ndSharedPtr<ndRenderSceneNode>> parentEntityList;

	effectNodeList.PushBack(meshRoot);
	parentEntityList.Append(ndSharedPtr<ndRenderSceneNode>(nullptr));

	ndSharedPtr<ndRenderSceneNode> renderMesh(nullptr);
	while (effectNodeList.GetCount())
	{
		const ndMesh* const mesh = effectNodeList.Pop();
		ndSharedPtr<ndRenderSceneNode> parentNode = parentEntityList.GetLast()->GetInfo();
		parentEntityList.Remove(parentEntityList.GetLast());
		ndSharedPtr<ndRenderSceneNode> entity(nullptr);
		if (!parentNode)
		{
			renderMesh = ndSharedPtr<ndRenderSceneNode>(new ndRenderSceneNode(mesh->GetMatrix()));
			renderMesh->SetPrimitiveMatrix(mesh->GetGeometryMatrix());
			entity = renderMesh;
		}
		else
		{
			ndSharedPtr<ndRenderSceneNode> childNode(new ndRenderSceneNode(mesh->GetMatrix()));
			childNode->SetPrimitiveMatrix(mesh->GetGeometryMatrix());
			parentNode->AddChild(childNode);
			entity = childNode;
		}
		entity->m_name = mesh->GetName();
	
		if (entity->m_name.Find("-hidden") == -1)
		{
			ndSharedPtr<ndMeshEffect> meshEffect(mesh->GetMesh());
			if (*meshEffect)
			{
				meshList.Append(EntityMeshPair(entity, mesh));
			}
		}
	
		for (ndList<ndSharedPtr<ndMesh>>::ndNode* childNode = mesh->GetChildren().GetFirst(); childNode; childNode = childNode->GetNext())
		{
			ndMesh::ndNodeType type = childNode->GetInfo()->GetNodeType();
			if (type != ndMesh::m_collisionShape)
			{
				parentEntityList.Append(entity);
				effectNodeList.PushBack(*childNode->GetInfo());
			}
		}
	}
	
	for (ndList<EntityMeshPair>::ndNode* node = meshList.GetFirst(); node; node = node->GetNext())
	{
		EntityMeshPair& pair = node->GetInfo();
	
		ndAssert(pair.m_mesh);
		ndAssert(pair.m_entity);
	
		ndSharedPtr<ndMeshEffect> meshEffect(pair.m_mesh->GetMesh());
		ndArray<ndMeshEffect::ndMaterial>& materials = meshEffect->GetMaterials();
	
		ndRenderPrimitive::ndDescriptor descriptor(renderer);
		descriptor.m_meshNode = meshEffect;
		descriptor.m_skeleton = pair.m_entity;
	
		for (ndInt32 j = 0; j < materials.GetCount(); ++j)
		{
			const ndString texturePathName(materialBasePath + materials[j].m_textureName);
			ndRenderPrimitiveMaterial& material = descriptor.AddMaterial(renderer->GetTextureCache()->GetTexture(texturePathName));
			material.m_diffuse = materials[j].m_diffuse;
			material.m_specular = materials[j].m_specular;
			material.m_reflection = materials[j].m_reflection;
			material.m_specularPower = ndReal(materials[j].m_shiness);
			material.m_opacity = ndReal(materials[j].m_opacity);
			material.m_castShadows = true;
		}
	
		ndSharedPtr<ndRenderPrimitive> geometry(new ndRenderPrimitive(descriptor));
		pair.m_entity->SetPrimitive(geometry);
	}
	return renderMesh;
}

bool ndRenderMeshLoader::LoadMesh(const ndString& fullPathMeshName)
{
	bool ret = ndAnimationMeshLoader::LoadMesh(fullPathMeshName);
	if (ret)
	{
		return MeshToRenderSceneNode(GetPath(fullPathMeshName));
	}
	return ret;
}

bool ndRenderMeshLoader::ImportFbx(const ndString& fbxPathMeshName)
{
	if (ndAnimationMeshLoader::ImportFbx(fbxPathMeshName))
	{
		return MeshToRenderSceneNode(GetPath(fbxPathMeshName));
	}
	return false;
}

