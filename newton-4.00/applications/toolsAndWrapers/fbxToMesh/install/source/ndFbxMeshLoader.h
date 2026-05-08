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

#ifndef _ND_FBX_MESH_LOADER_H_
#define _ND_FBX_MESH_LOADER_H_

#include "ndMesh.h"
class ndAnimationSequence;

using namespace ndOfbx;

#define ND_FBX_MAX_CHILDREN	1024

class ndFbxMeshLoader : public ndClassAlloc
{
	class ndFbxAnimationTrack;
	class ndFbx2ndMeshNodeMap;
	class ndFbx2MeshNodeStackData;

	public:
	ndFbxMeshLoader();
	ndFbxMeshLoader(const ndFbxMeshLoader& src);
	virtual ~ndFbxMeshLoader();

	virtual ndSharedPtr<ndAnimationSequence> LoadAnimation(const char* const fullPathName);
	virtual ndSharedPtr<ndMesh> LoadMesh(const char* const fullPathName, bool loadAnimation);

	private:
	void FreezeScale(ndMesh* const entity);
	void AlignToWorld(ndMesh* const entity);
	void OptimizeCurve(ndMesh::ndCurve& curve);
	void OptimizeAnimation(ndMesh* const model);
	ndMesh* FbxToMesh(ndOfbx::IScene* const fbxScene);
	void CalculateBoneProperties(ndMesh* const entity);
	void OptimizeRotationCurve(ndMesh::ndCurve& curve);
	ndMatrix ofbxMatrix2dMatrix(const ndOfbx::Matrix& fbxMatrix);
	ndMatrix GetCoordinateSystemMatrix(ndOfbx::IScene* const fbxScene);
	void ApplyTransform(ndMesh* const entity, const ndMatrix& transform);
	void ApplyAllTransforms(ndMesh* const mesh, const ndMatrix& unitMatrix);
	void LoadAnimation(const ndOfbx::IScene* const fbxScene, ndMesh* const model);
	void ImportMeshNode(ndOfbx::Object* const fbxNode, ndFbx2ndMeshNodeMap& nodeMap);
	void ImportMaterials(const ndOfbx::Mesh* const fbxMesh, ndMeshEffect* const mesh);
	ndFixSizeArray<ndOfbx::Object*, ND_FBX_MAX_CHILDREN> GetChildrenNodes(const ndOfbx::Object* const node);
	ndAnimationSequence* CreateSequence(ndMesh* const model, const char* const name);
	ndMesh* CreateMeshHierarchy(ndOfbx::IScene* const fbxScene, ndFbx2ndMeshNodeMap& nodeMap);
	ndMatrix GetKeyframe(ndMesh::ndCurveValue& scale, ndMesh::ndCurveValue& position, ndMesh::ndCurveValue& rotation);
	void LoadAnimationLayer(ndTree <ndFbxAnimationTrack, ndString>& tracks, const ndOfbx::IScene* const fbxScene, const ndOfbx::AnimationLayer* const animLayer);
	void LoadAnimationCurve(ndTree <ndFbxAnimationTrack, ndString>& tracks, const ndOfbx::IScene* const, const ndOfbx::Object* const bone, const ndOfbx::AnimationLayer* const animLayer, ndFloat32 duration, ndInt32 framesCount);
};

#endif