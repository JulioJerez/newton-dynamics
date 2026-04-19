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
#include "ndJointGear.h"
#include "ndJointHinge.h"
#include "ndJointPlane.h"
#include "ndMeshEffect.h"
#include "ndMeshLoader.h"
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

void ndMeshLoader::SaveMesh(const ndString& fullPathName) const
{
	ndString oldloc(setlocale(LC_ALL, 0));
	ndSharedPtr<nd::TiXmlDocument> doc(new nd::TiXmlDocument(""));
	nd::TiXmlDeclaration* const decl = new nd::TiXmlDeclaration("1.0", "", "");
	doc->LinkEndChild(decl);

	nd::TiXmlElement* const rootNode = new nd::TiXmlElement("ndMesh");
	doc->LinkEndChild(rootNode);

	// make the bone list for sikn and othe dependencies
	ndTree<ndString, ndUnsigned32> bonesMap;
	for (ndMesh* node = m_mesh->IteratorFirst(); node; node = node->IteratorNext(*m_mesh))
	{
		if (node->m_name.GetStr())
		{
			ndUnsigned32 hash = ndUnsigned32(ndCRC64(node->m_name.GetStr()) & 0xffffffff);
			bonesMap.Insert(node->m_name, hash);
		}
	}

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
		xmlSaveParam(entry.m_parentXml, "geometryMatrix", entry.m_meshNode->m_geometryMatrix);

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

		if (entry.m_meshNode->GetMesh())
		{
			nd::TiXmlElement* const geometry = new nd::TiXmlElement("geometry");
			entry.m_parentXml->LinkEndChild(geometry);
			entry.m_meshNode->GetMesh()->SerializeToXml(geometry, bonesMap);
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
			if (strcmp(linkName, ND_MESH_LOOP_JOINTS) == 0)
			{
				child = ndSharedPtr<ndMesh>(new (ndCloseLoopConstraints));
			}
			
			entry.m_mesh->AddChild(child);
			stack.PushBack(MeshXmlNodePair(*child, linkNode));
		}
	}

	// populate mesh hierachy
	stack.PushBack(MeshXmlNodePair(*m_mesh, rootNode));
	while (stack.GetCount())
	{
		MeshXmlNodePair entry(stack.Pop());
	
		ndMesh* const mesh = entry.m_mesh;
		ndAssert (mesh->m_name == ndString(xmlGetString(entry.m_xmlNode, "name")));
		mesh->m_matrix = xmlGetMatrix(entry.m_xmlNode, "matrix");
		mesh->m_geometryMatrix = xmlGetMatrix(entry.m_xmlNode, "geometryMatrix");
	
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
		ndTriplexReal target(xmlGetTriplexRealAttribute(xmlNodeType, "target"));
	
		const nd::TiXmlElement* const xmlGeometry = (nd::TiXmlElement*)entry.m_xmlNode->FirstChild("geometry");
		if (xmlGeometry)
		{
			ndSharedPtr<ndMeshEffect> geometry(new ndMeshEffect());
			geometry->DeserializeFromXml(xmlGeometry);
			mesh->SetMesh(geometry);
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

		auto LoadJoint = [&mesh](const nd::TiXmlElement* const xmlJoint)
		{
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
		};
	
		const nd::TiXmlElement* const xmlJoint = (nd::TiXmlElement*)entry.m_xmlNode->FirstChild("joint");
		if (xmlJoint)
		{
			mesh->SetJoint(LoadJoint(xmlJoint));
		}

		if (mesh->m_name == ND_MESH_LOOP_JOINTS)
		{
			for (const nd::TiXmlNode* node = entry.m_xmlNode->FirstChild("loopJoint"); node; node = node->NextSibling("loopJoint"))
			{
				const nd::TiXmlElement* const loopJointNode = (nd::TiXmlElement*)node;
				const char* const childNodeMame = xmlGetString(loopJointNode, "childReference");
				const char* const parentNodeMame = xmlGetString(loopJointNode, "parentReference");
				ndAssert(childNodeMame);
				ndAssert(parentNodeMame);
				ndMesh* const childNode = m_mesh->FindByName(childNodeMame);
				ndMesh* const parentNode = m_mesh->FindByName(parentNodeMame);
				ndAssert(childNode);
				ndAssert(parentNode);
				
				const nd::TiXmlElement* const xmlCloseJoint = (nd::TiXmlElement*)loopJointNode->FirstChild("joint");
				ndAssert(xmlCloseJoint);
				ndSharedPtr<ndMeshJoint> jointLoadJoint(LoadJoint(xmlCloseJoint));
				ndSharedPtr<ndMeshLoopJoint> loopJoint(new ndMeshLoopJoint(jointLoadJoint, childNode, parentNode));
				m_mesh->AddLoopJoint(loopJoint);
			}
		}
	
		ndList<ndSharedPtr<ndMesh>>::ndNode* childNode = mesh->GetChildren().GetFirst();
		for (const nd::TiXmlNode* node = entry.m_xmlNode->FirstChild("ndMesh"); node; node = node->NextSibling("ndMesh"))
		{
			const nd::TiXmlElement* const linkNode = (nd::TiXmlElement*)node;
			ndAssert(strcmp(linkNode->Value(), "ndMesh") == 0);
			ndSharedPtr<ndMesh> child(childNode->GetInfo());
			childNode = childNode->GetNext();
			stack.PushBack(MeshXmlNodePair(*child, linkNode));
		}
	}

	setlocale(LC_ALL, oldloc.GetStr());
	return true;
}
