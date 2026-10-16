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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndMesh.h"
#include "ndMeshLoader.h"
#include "ndBodyDynamic.h"
#include "ndMeshComponents.h"

ndMeshLoader::ndMeshLoader()
	:ndClassAlloc()
	,m_mesh(nullptr)
{
}

ndMeshLoader::ndMeshLoader(const ndSharedPtr<ndMesh>& mesh)
	:ndClassAlloc()
	,m_mesh(mesh)
{
}

ndMeshLoader::~ndMeshLoader()
{
}

bool ndMeshLoader::LoadMesh(const ndString& fullPathMeshName)
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

	struct MeshXmlNodePair
	{
		MeshXmlNodePair() {}
		MeshXmlNodePair(ndMesh* const mesh, const nd::TiXmlElement* const xmlNode)
			:m_mesh(mesh)
			,m_xmlNode(xmlNode)
		{
		}
		ndMesh* m_mesh;
		const nd::TiXmlElement* m_xmlNode;
	};

	ndFixSizeArray<MeshXmlNodePair, 1024> stack;

	const nd::TiXmlElement* const rootNode = doc.RootElement();
	ndAssert(strcmp(rootNode->Value(), "ndMesh") == 0);

	m_mesh = ndSharedPtr<ndMesh>(new ndMesh());
	stack.PushBack(MeshXmlNodePair(*m_mesh, rootNode));

	// create the hiearchy
	while (stack.GetCount())
	{
		MeshXmlNodePair entry(stack.Pop());

		ndMesh* const mesh = entry.m_mesh;
		mesh->m_name = ndString(xmlGetString(entry.m_xmlNode, "name"));

		for (const nd::TiXmlNode* node = entry.m_xmlNode->FirstChild("ndMesh"); node; node = node->NextSibling("ndMesh"))
		{
			const nd::TiXmlElement* const linkNode = (nd::TiXmlElement*)node;
			ndAssert(strcmp(linkNode->Value(), "ndMesh") == 0);

			ndSharedPtr<ndMesh> child(new ndMesh());
			const char* const linkName = xmlGetString(linkNode, "name");
			if (strcmp(linkName, ND_MESH_CONSTRAINT_LOOPS) == 0)
			{
				child = ndSharedPtr<ndMesh>(new ndCloseLoopConstraints);
			}
			else if (strcmp(linkName, ND_MESH_COLLIDING_PAIRS) == 0)
			{
				child = ndSharedPtr<ndMesh>(new ndCollidingPairs);
			}
			
			entry.m_mesh->AddChild(child);
			stack.PushBack(MeshXmlNodePair(*child, linkNode));
		}
	}

	// populate mesh hierachy
	ndList<MeshXmlNodePair> modifiers;
	ndList<MeshXmlNodePair> customProps;
	const nd::TiXmlElement* m_xmlPair = nullptr;
	const nd::TiXmlElement* m_xmlLoops = nullptr;

	stack.PushBack(MeshXmlNodePair(*m_mesh, rootNode));
	while (stack.GetCount())
	{
		MeshXmlNodePair entry(stack.Pop());
	
		ndMesh* const mesh = entry.m_mesh;
		ndAssert (mesh->m_name == ndString(xmlGetString(entry.m_xmlNode, "name")));
		mesh->m_matrix = xmlGetMatrix(entry.m_xmlNode, "matrix");
		mesh->m_geometryMatrix = xmlGetMatrix(entry.m_xmlNode, "geometryMatrix");
		mesh->m_basePoseMatrix = mesh->m_matrix;
		if (xmlHasParam(entry.m_xmlNode, "basePoseMatrix"))
		{
			mesh->m_basePoseMatrix = xmlGetMatrix(entry.m_xmlNode, "basePoseMatrix");
		}
	
		nd::TiXmlElement* const xmlNodeType = (nd::TiXmlElement*)entry.m_xmlNode->FirstChild("type");
		ndAssert(xmlNodeType);
		const char* const nodeType = xmlGetNameAttribute(xmlNodeType, "nodeType");
		if (!strcmp(nodeType, "node"))
		{
			mesh->SetNodeType(ndMesh::m_node);
		}
		else if (!strcmp(nodeType, "bone"))
		{
			mesh->SetNodeType(ndMesh::m_bone);
		}
		else if (!strcmp(nodeType, "endBone"))
		{
			mesh->SetNodeType(ndMesh::m_boneEnd);
		}
		else if (!strcmp(nodeType, "collisionShape"))
		{
			mesh->SetNodeType(ndMesh::m_collisionShape);
		}
		else
		{
			ndAssert(0);
		}

		if (xmlHasAttribute(entry.m_xmlNode, "visible"))
		{
			mesh->m_isVisible = xmlGetIntAttribute(entry.m_xmlNode, "visible") ? true : false;
		}
		else if (mesh->GetNodeType() == ndMesh::m_collisionShape)
		{
			mesh->m_isVisible = false;
		}
		else if (mesh->GetName().Find("-hidden") != -1)
		{
			mesh->m_isVisible = false;
		}

		ndTriplexReal target(xmlGetTriplexRealAttribute(xmlNodeType, "target"));
		mesh->SetBoneTarget(ndVector(target.m_x, target.m_y, target.m_z, ndReal(1.0f)));
	
		const nd::TiXmlElement* const xmlGeometry = (nd::TiXmlElement*)entry.m_xmlNode->FirstChild("geometry");
		if (xmlGeometry)
		{
			ndSharedPtr<ndMeshEffect> geometry(new ndMeshEffect());
			geometry->DeserializeFromXml(xmlGeometry);
			mesh->SetGeometry(geometry);
		}
	
		const nd::TiXmlElement* const xmlRigidBody = (nd::TiXmlElement*)entry.m_xmlNode->FirstChild("rigidbody");
		if (xmlRigidBody)
		{
			const char* const constructor = xmlGetString(xmlRigidBody, "constructor");
			if (strcmp(constructor, ndBodyDynamic::StaticClassName()) == 0)
			{
				ndSharedPtr<ndMeshBody> rigidBody(new ndMeshBodyDynamic(mesh));
				rigidBody->DeserializeFromXml(xmlRigidBody);
				mesh->SetRigidBody(rigidBody);
			}
			else
			{
				ndAssert(0);
				ndAssert(strcmp(constructor, ndBodyKinematic::StaticClassName()) == 0);
				ndSharedPtr<ndMeshBody> rigidBody(new ndMeshBodyKinematic(mesh));
				rigidBody->DeserializeFromXml(xmlRigidBody);
				mesh->SetRigidBody(rigidBody);
			}
		}
	
		const nd::TiXmlElement* const xmlJoint = (nd::TiXmlElement*)entry.m_xmlNode->FirstChild("joint");
		if (xmlJoint)
		{
			mesh->SetJoint(mesh->LoadJoint(xmlJoint));
		}

		const nd::TiXmlElement* const xmlModifier = (nd::TiXmlElement*)entry.m_xmlNode->FirstChild("modifier");
		if (xmlModifier)
		{
			modifiers.Append(entry);
		}

		const nd::TiXmlElement* const xmlCustomProp = (nd::TiXmlElement*)entry.m_xmlNode->FirstChild("customProperties");
		if (xmlCustomProp)
		{
			for (const nd::TiXmlNode* xmpProperty = xmlCustomProp->FirstChild("property"); xmpProperty; xmpProperty = xmpProperty->NextSibling("property"))
			{
				const char* const constructor = xmlGetString(xmpProperty, "className");
				if (strcmp(constructor, ndMeshCustomPropertyFloat::StaticClassName()) == 0)
				{
					ndMeshCustomPropertyFloat* const property = new ndMeshCustomPropertyFloat(mesh);
					property->DeserializeFromXml((nd::TiXmlElement*)xmpProperty);
					mesh->m_customProperties.Append(ndSharedPtr<ndMeshCustomProperty>(property));
				}
				else if (strcmp(constructor, ndMeshCustomPropertyString::StaticClassName()) == 0)
				{
					ndMeshCustomPropertyString* const property = new ndMeshCustomPropertyString(mesh);
					property->DeserializeFromXml((nd::TiXmlElement*)xmpProperty);
					mesh->m_customProperties.Append(ndSharedPtr<ndMeshCustomProperty>(property));
				}
				else if (strcmp(constructor, ndMeshCustomPropertyNode::StaticClassName()) == 0)
				{
					MeshXmlNodePair propEntry(entry.m_mesh, (nd::TiXmlElement*)xmpProperty);
					customProps.Append(propEntry);
				}
				else
				{
					ndAssert(0);
				}
			}
		}

		if (mesh->m_name == ND_MESH_CONSTRAINT_LOOPS)
		{
			m_xmlLoops = entry.m_xmlNode;
		}

		if (mesh->m_name == ND_MESH_COLLIDING_PAIRS)
		{
			m_xmlPair = entry.m_xmlNode;
		}
	
		ndList<ndSharedPtr<ndMesh>>::ndNode* childNode = mesh->GetChildren().GetLast();
		for (const nd::TiXmlNode* node = entry.m_xmlNode->LastChild("ndMesh"); node; node = node->PreviousSibling("ndMesh"))
		{
			const nd::TiXmlElement* const linkNode = (nd::TiXmlElement*)node;
			ndAssert(strcmp(linkNode->Value(), "ndMesh") == 0);
			ndSharedPtr<ndMesh> child(childNode->GetInfo());
			stack.PushBack(MeshXmlNodePair(*child, linkNode));
			childNode = childNode->GetPrev();
		}
	}

	for (ndList<MeshXmlNodePair>::ndNode* ptr = modifiers.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		MeshXmlNodePair entry(ptr->GetInfo());
		const nd::TiXmlElement* const xmlModifier = (nd::TiXmlElement*)entry.m_xmlNode->FirstChild("modifier");
		entry.m_mesh->SetModifier(entry.m_mesh->LoadModifier(xmlModifier));
	}

	for (ndList<MeshXmlNodePair>::ndNode* ptr = customProps.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		MeshXmlNodePair entry(ptr->GetInfo());
		const nd::TiXmlNode* const xmpProperty = entry.m_xmlNode;
		ndMeshCustomPropertyNode* const property = new ndMeshCustomPropertyNode(entry.m_mesh);
		property->DeserializeFromXml((nd::TiXmlElement*)xmpProperty);
		entry.m_mesh->m_customProperties.Append(ndSharedPtr<ndMeshCustomProperty>(property));
	}

	if (m_xmlPair)
	{
		for (const nd::TiXmlNode* node = m_xmlPair->FirstChild("collindPair"); node; node = node->NextSibling("collindPair"))
		{
			const nd::TiXmlElement* const pairNode = (nd::TiXmlElement*)node;
			const char* const referenceName0 = xmlGetString(pairNode, "reference0");
			const char* const referenceName1 = xmlGetString(pairNode, "reference1");
			ndAssert(referenceName0);
			ndAssert(referenceName1);
			ndMesh* const referenceNode0 = m_mesh->FindByName(referenceName0);
			ndMesh* const referenceNode1 = m_mesh->FindByName(referenceName1);
			ndAssert(referenceNode0);
			ndAssert(referenceNode1);
			m_mesh->SetCollidingSubSelection(referenceNode0, referenceNode1);
		}
	}

	if (m_xmlLoops)
	{
		const ndCloseLoopConstraints* const loopJointList = m_mesh->GetLoopJoints();
		for (const nd::TiXmlNode* node = m_xmlLoops->FirstChild("loopJoint"); node; node = node->NextSibling("loopJoint"))
		{
			const nd::TiXmlElement* const xmlLoopJointNode = (nd::TiXmlElement*)node;
			ndSharedPtr<ndMeshLoopJoint> loopJoint(new ndMeshLoopJoint(loopJointList));
			loopJoint->DeserializeFromXml(xmlLoopJointNode);
			m_mesh->AddLoopJoint(loopJoint);
		}
	}

	setlocale(LC_ALL, oldloc.GetStr());
	return true;
}

void ndMeshLoader::SaveMesh(const ndString& fullPathName) const
{
	ndString oldloc(setlocale(LC_ALL, 0));
	ndSharedPtr<nd::TiXmlDocument> doc(new nd::TiXmlDocument(""));
	nd::TiXmlDeclaration* const decl = new nd::TiXmlDeclaration("1.0", "", "");
	doc->LinkEndChild(decl);

	nd::TiXmlElement* const rootNode = new nd::TiXmlElement("ndMesh");
	doc->LinkEndChild(rootNode);

	// make the bone list for skin and other dependencies
	ndTree<ndString, ndUnsigned32> bonesMap;
	auto MakeBonesMap = [&bonesMap](ndMesh* const node)
	{
		if (node->m_name.GetStr())
		{
			ndUnsigned32 hash = ndUnsigned32(ndCRC64(node->m_name.GetStr()) & 0xffffffff);
			bonesMap.Insert(node->m_name, hash);
		}
	};
	m_mesh->NodeIterator(MakeBonesMap);

	struct MeshXmlNodePair
	{
		const ndMesh* m_meshNode;
		nd::TiXmlElement* m_parentXml;
	};

	ndFixSizeArray<MeshXmlNodePair, 1024> stack;
	MeshXmlNodePair pair;
	pair.m_meshNode = *m_mesh;
	pair.m_parentXml = rootNode;
	stack.PushBack(pair);

	while (stack.GetCount())
	{
		MeshXmlNodePair entry(stack.Pop());
		xmlSaveParam(entry.m_parentXml, "name", entry.m_meshNode->m_name.GetStr());
		xmlSaveParam(entry.m_parentXml, "matrix", entry.m_meshNode->m_matrix);
		xmlSaveParam(entry.m_parentXml, "basePoseMatrix", entry.m_meshNode->m_basePoseMatrix);
		xmlSaveParam(entry.m_parentXml, "geometryMatrix", entry.m_meshNode->m_geometryMatrix);
		xmlSaveAttribute(entry.m_parentXml, "visible", entry.m_meshNode->m_isVisible ? 1 : 0);

		nd::TiXmlElement* const xmlNodeType = new nd::TiXmlElement("type");
		entry.m_parentXml->LinkEndChild(xmlNodeType);
		switch (entry.m_meshNode->m_type)
		{
			case ndMesh::m_node:
				xmlSaveAttribute(xmlNodeType, "nodeType", "node");
				break;
			case ndMesh::m_bone:
				xmlSaveAttribute(xmlNodeType, "nodeType", "bone");
				break;
			case ndMesh::m_boneEnd:
				xmlSaveAttribute(xmlNodeType, "nodeType", "endBone");
				break;

			case ndMesh::m_collisionShape:
				xmlSaveAttribute(xmlNodeType, "nodeType", "collisionShape");
				break;
		}
		ndVector boneTarget(entry.m_meshNode->GetBoneTarget());
		xmlSaveAttribute(xmlNodeType, "target", ndTriplexReal(ndReal(boneTarget.m_x), ndReal(boneTarget.m_y), ndReal(boneTarget.m_z)));

		if (entry.m_meshNode->GetGeometry())
		{
			nd::TiXmlElement* const geometry = new nd::TiXmlElement("geometry");
			entry.m_parentXml->LinkEndChild(geometry);
			entry.m_meshNode->GetGeometry()->SerializeToXml(geometry, bonesMap);
		}

		if (entry.m_meshNode->m_rigidBody)
		{
			nd::TiXmlElement* const rigidBodyNode = new nd::TiXmlElement("rigidbody");
			entry.m_parentXml->LinkEndChild(rigidBodyNode);

			const ndMeshBody* const rigidBody = *entry.m_meshNode->m_rigidBody;
			xmlSaveParam(rigidBodyNode, "constructor", rigidBody->m_classConstructor.GetStr());
			rigidBody->SerializeToXml(rigidBodyNode);
		}

		if (entry.m_meshNode->m_joint)
		{
			nd::TiXmlElement* const jointNode = new nd::TiXmlElement("joint");
			entry.m_parentXml->LinkEndChild(jointNode);
			const ndMeshJoint* const joint = *entry.m_meshNode->m_joint;
			joint->SerializeToXml(jointNode);
		}

		if (entry.m_meshNode->m_transformModifier)
		{
			nd::TiXmlElement* const modifierNode = new nd::TiXmlElement("modifier");
			entry.m_parentXml->LinkEndChild(modifierNode);
			const ndMeshTransformModifier* const modifier = *entry.m_meshNode->m_transformModifier;
			modifier->SerializeToXml(modifierNode);
		}

		if (entry.m_meshNode->m_customProperties.GetCount())
		{
			nd::TiXmlElement* const customProperties = new nd::TiXmlElement("customProperties");
			entry.m_parentXml->LinkEndChild(customProperties);

			const ndList<ndSharedPtr<ndMeshCustomProperty>>& propsList = entry.m_meshNode->m_customProperties;
			for (ndList<ndSharedPtr<ndMeshCustomProperty>>::ndNode* ptr = propsList.GetFirst(); ptr; ptr = ptr->GetNext())
			{
				nd::TiXmlElement* const properties = new nd::TiXmlElement("property");
				customProperties->LinkEndChild(properties);

				const ndMeshCustomProperty* const property = *ptr->GetInfo();
				property->SerializeToXml(properties);
			}
		}

		if (entry.m_meshNode->GetAsCloseLoopConstraints())
		{
			const ndCloseLoopConstraints* const closeLoops = entry.m_meshNode->GetAsCloseLoopConstraints();
			for (ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* loopPtr = closeLoops->m_loopJoints.GetFirst(); loopPtr; loopPtr = loopPtr->GetNext())
			{
				nd::TiXmlElement* const jointNode = new nd::TiXmlElement("loopJoint");
				entry.m_parentXml->LinkEndChild(jointNode);

				const ndMeshLoopJoint* const joint = *loopPtr->GetInfo();
				joint->SerializeToXml(jointNode);
			}
		}

		if (entry.m_meshNode->GetAsCollidingPairs())
		{
			const ndCollidingPairs* const collidningPairs = entry.m_meshNode->GetAsCollidingPairs();
			for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* pairPtr = collidningPairs->m_collidingPairs.GetFirst(); pairPtr; pairPtr = pairPtr->GetNext())
			{
				nd::TiXmlElement* const pairNode = new nd::TiXmlElement("collindPair");
				entry.m_parentXml->LinkEndChild(pairNode);

				const ndMeshCollidingPair* const pairEntry = *pairPtr->GetInfo();
				pairEntry->SerializeToXml(pairNode);
			}
		}

		for (ndList<ndSharedPtr<ndMesh>>::ndNode* node = entry.m_meshNode->m_children.GetFirst(); node; node = node->GetNext())
		{
			MeshXmlNodePair childPair;
			childPair.m_meshNode = *node->GetInfo();
			nd::TiXmlElement* const child = new nd::TiXmlElement("ndMesh");
			entry.m_parentXml->LinkEndChild(child);
			childPair.m_parentXml = child;
			stack.PushBack(childPair);
		}
	};

	doc->SaveFile(fullPathName.GetStr());
	setlocale(LC_ALL, oldloc.GetStr());
}
