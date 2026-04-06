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

#ifndef _ND_URDF_FILE_H_
#define _ND_URDF_FILE_H_

#include "ndNewAssetStdafx.h"

#if 0
class ndUrdfFile : public ndClassAlloc
{
	class Material
	{
		public:
		Material()
			:m_color(1.0f, 1.0f, 1.0f, 1.0f)
		{
			m_texture[0] = 0;
		}
		ndVector m_color;
		char m_texture[256];
	};

	public:
	D_NEWTON_API virtual void Export(const char* const fileName, ndModelArticulation* const model);

	private:
	class Hierarchy
	{
		public:
		Hierarchy(const nd::TiXmlNode* const link)
			:m_parent(nullptr)
			,m_link(link)
			,m_joint(link)
			,m_parentLink(nullptr)
			,m_articulation(nullptr)
			,m_parentArticulation(nullptr)
		{
		}

		Hierarchy* m_parent;
		const nd::TiXmlNode* m_link;
		const nd::TiXmlNode* m_joint;
		const nd::TiXmlNode* m_parentLink;
		ndModelArticulation::ndNode* m_articulation;
		ndModelArticulation::ndNode* m_parentArticulation;
	};

	class Surrogate: public ndNodeHierarchy<Surrogate>
	{
		public:
		ndVector m_com;
		ndMatrix m_bodyMatrix;
		ndMatrix m_shapeLocalMatrix;
		ndMatrix m_bodyInertia;
		ndMatrix m_jointLocalMatrix0;
		ndMatrix m_jointLocalMatrix1;
		ndModelArticulation::ndNode* m_articulation;
	};

	void ExportMakeNamesUnique(ndModelArticulation* const model);
	Surrogate* ExportMakeSurrogate(ndModelArticulation* const model);
	//void ExportCollectTransforms(Surrogate* const surrogateRoot);
	void ExportOrigin(nd::TiXmlElement* const linkNode, const ndMatrix& pose);
	void ExportLink(nd::TiXmlElement* const rootNode, const Surrogate* const link);
	void ExportJoint(nd::TiXmlElement* const rootNode, const Surrogate* const link);
	void ExportVisual(nd::TiXmlElement* const linkNode, const Surrogate* const link);
	void ExportInertia(nd::TiXmlElement* const linkNode, const Surrogate* const link);
	void ExportMaterials(nd::TiXmlElement* const rootNode, const Surrogate* const link);
	void ExportCollision(nd::TiXmlElement* const linkNode, const Surrogate* const link);

	ndString m_path;
	ndArray<Material> m_materials;
	ndTree<Hierarchy, ndString> m_bodyLinks;
	ndTree<ndInt32, ndString> m_materialMap;
};
#endif

class ndUrdfMeshLoader : public ndRenderMeshLoader
{
	class Material
	{
		public:
		Material()
			:m_color(1.0f, 1.0f, 1.0f, 1.0f)
		{
			m_name[0] = 0;
			m_texture[0] = 0;
		}
		ndVector m_color;
		char m_name[256];
		char m_texture[256];
	};

	class Hierarchy
	{
		public:
		Hierarchy(const nd::TiXmlNode* const link)
			:m_parent(nullptr)
			,m_link(link)
			,m_joint(nullptr)
			,m_parentLink(nullptr)
			,m_articulation(nullptr)
		{
		}

		Hierarchy* m_parent;
		const nd::TiXmlNode* m_link;
		const nd::TiXmlNode* m_joint;
		const nd::TiXmlNode* m_parentLink;
		ndModelArticulation::ndNode* m_articulation;
	};

	public:
	ndUrdfMeshLoader(ndRender* const renderer);
	
	virtual bool Import(const ndString& urdfPathMeshName);

	private:
	ndMatrix ImportOrigin(const nd::TiXmlNode* const parentNode) const;
	void ImportCollision(const nd::TiXmlNode* const linkNode, ndBodyDynamic* const body);
	void ImportVisual(const nd::TiXmlNode* const linkNode, ndMesh* const meshNode) const;
	ndJointBilateralConstraint* ImportJoint(const nd::TiXmlNode* const jointNode, ndBodyDynamic* const childBody, ndBodyDynamic* const parentBody);

	bool ImportStlMesh(const ndMatrix& matrix, const char* const pathName, ndMeshEffect* const meshEffect, ndInt32 materialIndex, const ndVector& scale) const;
	bool ImportObjMesh(const ndMatrix& matrix, const char* const pathName, ndMeshEffect* const meshEffect, ndInt32 materialIndex, const ndVector& scale) const;

	ndString m_path;
	ndArray<Material> m_materials;
	ndTree<Hierarchy, ndString> m_bodyLinks;
};

#endif