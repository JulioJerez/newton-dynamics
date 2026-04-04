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

#include "ndNewAssetStdafx.h"
//#include "ndCoreStdafx.h"
//#include "ndCollisionStdafx.h"
//#include "ndMesh.h"
//#include "ndMeshLoader.h"
//#include "ndJointHinge.h"
//#include "ndMeshEffect.h"
//#include "ndMeshLoader.h"
//#include "ndJointWheel.h"
//#include "ndBodyDynamic.h"
//#include "ndJointSpherical.h"
//#include "ndMeshComponents.h"


#if 0
class ndUrdfMaterial
{
	public:
	ndUrdfMaterial()
		:m_color(1.0f, 1.0f, 1.0f, 1.0f)
	{
		m_texture[0] = 0;
	}
	ndVector m_color;
	char m_texture[256];
};

class ndUrdfHierarchy
{
	public:
	ndUrdfHierarchy(const nd::TiXmlNode* const link)
		:m_parent(nullptr)
		,m_link(link)
		,m_joint(link)
		,m_parentLink(nullptr)
		,m_articulation(nullptr)
		,m_parentArticulation(nullptr)
	{
	}

	ndUrdfHierarchy* m_parent;
	const nd::TiXmlNode* m_link;
	const nd::TiXmlNode* m_joint;
	const nd::TiXmlNode* m_parentLink;
	ndMesh* m_articulation;
	ndMesh* m_parentArticulation;
};


bool ndMeshLoader::ImportUrdf(const ndString& fullPathMeshName)
{
	ndString oldloc(setlocale(LC_ALL, 0));
	nd::TiXmlDocument doc(fullPathMeshName.GetStr());
	doc.LoadFile();
	if (doc.Error())
	{
		m_mesh = ndSharedPtr<ndMesh>(nullptr);
		setlocale(LC_ALL, oldloc.GetStr());
		ndTrace(("file: %s not found", fullPathMeshName.GetStr()));
		return false;
	}

	ndTree<ndUrdfHierarchy, ndString> bodyLinks;

	const nd::TiXmlElement* const rootNode = doc.RootElement();

	for (const nd::TiXmlNode* node = rootNode->FirstChild("link"); node; node = node->NextSibling("link"))
	{
		const nd::TiXmlElement* const linkNode = (nd::TiXmlElement*)node;
		const char* const name = linkNode->Attribute("name");
		bodyLinks.Insert(ndUrdfHierarchy(node), name);
	}

	for (const nd::TiXmlNode* node = rootNode->FirstChild("joint"); node; node = node->NextSibling("joint"))
	{
		const nd::TiXmlElement* const jointNode = (nd::TiXmlElement*)node;
		const nd::TiXmlElement* const childNode = (nd::TiXmlElement*)jointNode->FirstChild("child");
		const nd::TiXmlElement* const parentNode = (nd::TiXmlElement*)jointNode->FirstChild("parent");

		const char* const childName = childNode->Attribute("link");
		const char* const parentName = parentNode->Attribute("link");

		ndTree<ndUrdfHierarchy, ndString>::ndNode* const hierarchyChildNode = bodyLinks.Find(childName);
		ndTree<ndUrdfHierarchy, ndString>::ndNode* const hierarchyParentNode = bodyLinks.Find(parentName);

		ndUrdfHierarchy& hierachyChild = hierarchyChildNode->GetInfo();
		ndUrdfHierarchy& hierachyParent = hierarchyParentNode->GetInfo();

		hierachyChild.m_joint = node;
		hierachyChild.m_parent = &hierachyParent;
		hierachyChild.m_parentLink = hierachyParent.m_link;
	}

	ndArray<ndUrdfMaterial> materials;
	ndTree<ndInt32, ndString> materialMap;
	auto ImportMaterials = [&materials, &materialMap](const nd::TiXmlNode* const rootNode)
	{
		materials.SetCount(0);
		materialMap.RemoveAll();
		materials.PushBack(ndUrdfMaterial());
		snprintf(materials[0].m_texture, sizeof(materials[0].m_texture), "wood_0.png");

		size_t readValues = 0;
		for (const nd::TiXmlNode* node = rootNode->FirstChild("material"); node; node = node->NextSibling("material"))
		{
			const nd::TiXmlElement* const materialNode = (nd::TiXmlElement*)node;

			const char* const name = materialNode->Attribute("name");
			materialMap.Insert(ndInt32(materials.GetCount()), name);

			ndUrdfMaterial material;
			const nd::TiXmlElement* const color = (nd::TiXmlElement*)node->FirstChild("color");
			if (color)
			{
				ndReal r;
				ndReal g;
				ndReal b;
				ndReal a;
				const char* const rgba = color->Attribute("rgba");
				readValues += sscanf(rgba, "%f %f %f %f", &r, &g, &b, &a);
				material.m_color.m_x = r;
				material.m_color.m_y = g;
				material.m_color.m_z = b;
				material.m_color.m_w = a;
			}
			const nd::TiXmlElement* const texture = (nd::TiXmlElement*)node->FirstChild("texture");
			if (texture)
			{
				const char* const texName = texture->Attribute("filename");
				snprintf(material.m_texture, sizeof(material.m_texture), "%s", texName);
			}
			else
			{
				snprintf(material.m_texture, sizeof(material.m_texture), "default.png");
			}
			materials.PushBack(material);
		}
	};
	ImportMaterials(rootNode);


	return true;
}

#endif