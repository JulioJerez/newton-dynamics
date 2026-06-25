/* Copyright (c) <2003-2018> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "stdafx.h"
#include <ndNewtonInc.h>

class exportVector
{
public:
	inline exportVector()
	{
	}

	inline exportVector(float val)
		:m_x(val), m_y(val), m_z(val), m_w(val)
	{
	}

	inline exportVector(const exportVector& v)
		:m_x(v.m_x), m_y(v.m_y), m_z(v.m_z), m_w(v.m_w)
	{
	}

	inline exportVector(const float* const ptr)
		: m_x(ptr[0])
		, m_y(ptr[1])
		, m_z(ptr[2])
		, m_w(ptr[3])
	{
	}

	inline exportVector(float x, float y, float z, float w)
		:m_x(x), m_y(y), m_z(z), m_w(w)
	{
	}

	inline float GetScalar() const
	{
		return m_x;
	}

	inline float& operator[] (int i)
	{
		return (&m_x)[i];
	}

	inline const float& operator[] (int i) const
	{
		return (&m_x)[i];
	}

	inline exportVector operator+ (const exportVector& A) const
	{
		return exportVector(m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w);
	}

	inline exportVector operator- (const exportVector& A) const
	{
		return exportVector(m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
	}

	inline exportVector operator* (const exportVector& A) const
	{
		return exportVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w);
	}

	inline exportVector& operator+= (const exportVector& A)
	{
		return (*this = exportVector(m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w));
	}

	inline exportVector& operator-= (const exportVector& A)
	{
		return (*this = exportVector(m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w));
	}

	inline exportVector& operator*= (const exportVector& A)
	{
		return (*this = exportVector(m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, m_w * A.m_w));
	}

	inline exportVector MulAdd(const exportVector& A, const exportVector& B) const
	{
		return *this + A * B;
	}

	inline exportVector MulSub(const exportVector& A, const exportVector& B) const
	{
		return *this - A * B;
	}

	inline exportVector AddHorizontal() const
	{
		return exportVector(m_x + m_y + m_z + m_w);
	}

	inline exportVector Scale(float scale) const
	{
		return exportVector(m_x * scale, m_y * scale, m_z * scale, m_w * scale);
	}

	// return cross product
	inline exportVector CrossProduct(const exportVector& B) const
	{
		return exportVector(m_y * B.m_z - m_z * B.m_y,
			m_z * B.m_x - m_x * B.m_z,
			m_x * B.m_y - m_y * B.m_x, m_w);
	}

	inline float DotProduct(const exportVector& A) const
	{
		return m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w;
	}


	inline exportVector Normalize() const
	{
		const exportVector& me = *this;
		return me.Scale(1.0f / float(sqrt(me.DotProduct(me))));
	}

	inline exportVector Abs() const
	{
		return exportVector(
			(m_x > float(0.0f)) ? m_x : -m_x,
			(m_y > float(0.0f)) ? m_y : -m_y,
			(m_z > float(0.0f)) ? m_z : -m_z,
			(m_w > float(0.0f)) ? m_w : -m_w);
	}

	inline static void Transpose4x4(exportVector& dst0, exportVector& dst1, exportVector& dst2, exportVector& dst3, const exportVector& src0, const exportVector& src1, const exportVector& src2, const exportVector& src3)
	{
		exportVector tmp0(src0);
		exportVector tmp1(src1);
		exportVector tmp2(src2);
		exportVector tmp3(src3);

		dst0 = exportVector(tmp0.m_x, tmp1.m_x, tmp2.m_x, tmp3.m_x);
		dst1 = exportVector(tmp0.m_y, tmp1.m_y, tmp2.m_y, tmp3.m_y);
		dst2 = exportVector(tmp0.m_z, tmp1.m_z, tmp2.m_z, tmp3.m_z);
		dst3 = exportVector(tmp0.m_w, tmp1.m_w, tmp2.m_w, tmp3.m_w);
	}

	float m_x;
	float m_y;
	float m_z;
	float m_w;
};

int InitializeSdkObjects(FbxManager*& fbsManager, FbxScene*& fbsScene)
{
	//The first thing to do is to create the FBX Manager which is the object allocator for almost all the classes in the SDK
	fbsManager = FbxManager::Create();
	if (!fbsManager)
	{
		FBXSDK_printf("Error: Unable to create FBX Manager!\n");
		exit(1);
	}
	else FBXSDK_printf("Autodesk FBX SDK version %s\n", fbsManager->GetVersion());

	//Create an IOSettings object. This object holds all import/export settings.
	FbxIOSettings* ios = FbxIOSettings::Create(fbsManager, IOSROOT);
	fbsManager->SetIOSettings(ios);

	//Load plugins from the executable directory (optional)
	FbxString lPath = FbxGetApplicationDirectory();
	fbsManager->LoadPluginsDirectory(lPath.Buffer());

	//Create an FBX fbsScene. This object holds most objects imported/exported from/to files.
	fbsScene = FbxScene::Create(fbsManager, "My Scene");
	if (!fbsScene)
	{
		FBXSDK_printf("Error: Unable to create FBX fbsScene!\n");
		return 0;
	}
	return 1;
}

FbxNode* CreateSkeleton(ndMesh* const model, FbxScene* const fbxScene)
{
	ndFixSizeArray<ndMesh*, 256> ndNodePool;
	ndFixSizeArray<FbxNode*, 256> fbxNodesParent;
	
	ndNodePool.PushBack(model);
	fbxNodesParent.PushBack(nullptr);
	
	FbxNode* skeleton = nullptr;
	while (ndNodePool.GetCount())
	{
		ndMesh* const node = ndNodePool.Pop();
		FbxNode* const fbxParent = fbxNodesParent.Pop();
		if (node->GetAsMesh())
		{	
			ndVector euler1;
			const char* const name = node->GetName().GetStr();
			FbxNode* const fbxNode = FbxNode::Create(fbxScene, name);

			const ndMatrix matrix(node->GetMatrix());
			const ndVector euler0(matrix.CalcPitchYawRoll(euler1));
			const ndVector euler(euler0.Scale(ndRadToDegree));
			const ndVector posit(matrix.m_posit);
		
			fbxNode->LclRotation.Set(FbxVector4(euler.m_x, euler.m_y, euler.m_z));
			fbxNode->LclTranslation.Set(FbxVector4(posit.m_x, posit.m_y, posit.m_z));

			if (node->GetParent())
			{
				fbxParent->AddChild(fbxNode);
			}

			//FbxSkeleton* const attribute = FbxSkeleton::Create(fbxScene, name);
			//if (fbxParent)
			//{
			//	attribute->Size.Set(ndFloat32(0.1f));
			//	attribute->SetSkeletonType(FbxSkeleton::eLimbNode);
			//	fbxParent->AddChild(fbxNode);
			//}
			//else
			//{
			//	attribute->SetSkeletonType(FbxSkeleton::eRoot);
			//}
			//fbxNode->SetNodeAttribute(attribute);

			if (!skeleton)
			{
				skeleton = fbxNode;
			}

			for (ndList<ndSharedPtr<ndMesh>>::ndNode* childNode = node->GetChildren().GetFirst(); childNode; childNode = childNode->GetNext())
			{
				ndMesh* const child = *childNode->GetInfo();
				ndNodePool.PushBack(child);
				fbxNodesParent.PushBack(fbxNode);
			}
		}
	}
	return skeleton;
}

FbxMesh* CreateGeometry(const ndMesh* const node, FbxScene* const fbxScene, FbxNode* const fbxNode, const ndString& path)
{
	FbxMesh* const fbxMesh = FbxMesh::Create(fbxScene, node->GetName().GetStr());
	ndSharedPtr<ndMeshEffect> geometry(node->GetGeometry());
	ndAssert(*geometry);

	// specify uv per polygon vertex.
	FbxLayer* fbxLayer = fbxMesh->GetLayer(0);
	if (fbxLayer == nullptr)
	{
		fbxMesh->CreateLayer();
		fbxLayer = fbxMesh->GetLayer(0);
	}

	ndFixSizeArray<ndInt32, 32> fbxMaterialIndex;
	const ndArray<ndMeshEffect::ndMaterial>& materials = geometry->GetMaterials();
	for (ndInt32 i = 0; i < ndInt32(materials.GetCount()); ++i)
	{
		const ndMeshEffect::ndMaterial& material = materials[i];

		ndInt32 fbxIndex = -1;
		FbxSurfacePhong* fbxMaterial = fbxMesh->GetSrcObject<FbxSurfacePhong>(0);
		if (fbxMaterial == nullptr)
		{
			FbxString lShadingName = "Phong";
			FbxString lMaterialName = material.m_name;
			fbxMaterial = FbxSurfacePhong::Create(fbxScene, lMaterialName.Buffer());

			// Generate primary and secondary colors.
			fbxMaterial->Emissive.Set(FbxDouble3(0.0, 0.0, 0.0));
			fbxMaterial->Ambient.Set(FbxDouble3(material.m_ambient.m_x, material.m_ambient.m_y, material.m_ambient.m_z));
			fbxMaterial->AmbientFactor.Set(1.);

			// Add texture for diffuse channel
			fbxMaterial->Diffuse.Set(FbxDouble3(material.m_diffuse.m_x, material.m_diffuse.m_y, material.m_diffuse.m_z));
			fbxMaterial->DiffuseFactor.Set(1.);
			fbxMaterial->TransparencyFactor.Set(material.m_opacity);
			fbxMaterial->ShadingModel.Set(lShadingName);
			fbxMaterial->Shininess.Set(material.m_shiness);
			fbxMaterial->Specular.Set(FbxDouble3(material.m_specular.m_x, material.m_specular.m_y, material.m_specular.m_z));
			fbxMaterial->SpecularFactor.Set(1.0);
			fbxIndex = fbxNode->AddMaterial(fbxMaterial);
		}

		fbxMaterialIndex.PushBack(fbxIndex);
		FbxFileTexture* const fbxTexture = FbxFileTexture::Create(fbxScene, "Diffuse Texture");

		// Set texture properties.
		ndString texturePath(path + material.m_textureName);
		fbxTexture->SetFileName(texturePath.GetStr());
		fbxTexture->SetTextureUse(FbxTexture::eStandard);
		fbxTexture->SetMappingType(FbxTexture::eUV);
		fbxTexture->SetMaterialUse(FbxFileTexture::eModelMaterial);
		fbxTexture->SetSwapUV(false);
		fbxTexture->SetTranslation(0.0, 0.0);
		fbxTexture->SetScale(1.0, 1.0);
		fbxTexture->SetRotation(0.0, 0.0);

		// don't forget to connect the texture to the corresponding property of the material
		if (fbxMaterial)
		{
			fbxMaterial->Diffuse.ConnectSrcObject(fbxTexture);
		}

		////fbxTexture = FbxFileTexture::Create(pScene, "Ambient Texture");
		//// Set texture properties.
		//fbxTexture->SetFileName(material.m_textureName);
		//fbxTexture->SetTextureUse(FbxTexture::eStandard);
		//fbxTexture->SetMappingType(FbxTexture::eUV);
		//fbxTexture->SetMaterialUse(FbxFileTexture::eModelMaterial);
		//fbxTexture->SetSwapUV(false);
		//fbxTexture->SetTranslation(0.0, 0.0);
		//fbxTexture->SetScale(1.0, 1.0);
		//fbxTexture->SetRotation(0.0, 0.0);
		//
		//// don't forget to connect the texture to the corresponding property of the material
		//if (fbxMaterial)
		//{
		//	fbxMaterial->Ambient.ConnectSrcObject(fbxTexture);
		//}
		//
		//fbxTexture = FbxFileTexture::Create(fbxScene, "Emissive Texture");
		//
		//// Set texture properties.
		//fbxTexture->SetFileName(material.m_textureName);
		//fbxTexture->SetTextureUse(FbxTexture::eStandard);
		//fbxTexture->SetMappingType(FbxTexture::eUV);
		//fbxTexture->SetMaterialUse(FbxFileTexture::eModelMaterial);
		//fbxTexture->SetSwapUV(false);
		//fbxTexture->SetTranslation(0.0, 0.0);
		//fbxTexture->SetScale(1.0, 1.0);
		//fbxTexture->SetRotation(0.0, 0.0);
		//
		//// don't forget to connect the texture to the corresponding property of the material
		//if (fbxMaterial)
		//{
		//	fbxMaterial->Emissive.ConnectSrcObject(fbxTexture);
		//}
	}

	const ndInt32 controlPointCount = geometry->GetVertexCount();
	fbxMesh->InitControlPoints(controlPointCount);
	ndInt32 stride = geometry->GetVertexStrideInByte() / sizeof (ndFloat64);
	const ndFloat64* const points = geometry->GetVertexPool();
	FbxVector4* const vertex = fbxMesh->GetControlPoints();
	const ndMeshEffect::ndAttibuteFormat& vertexAtributes = geometry->GetVertexAtributes();
	const ndInt32* const materialId = &vertexAtributes.m_materialChannel[0];

	const ndMatrix geometryMatrix(node->GetGeometryMatrix());
	for (ndInt32 i = 0; i < controlPointCount; ++i)
	{
		ndInt32 index = i * stride;
		const ndVector point(points[index + 0], points[index + 1], points[index + 2], ndFloat64 (1.0f));
		const ndVector p(geometryMatrix.TransformVector(point));
		vertex[i].Set(p.m_x, p.m_y, p.m_z);
	}

	{
		// add all the faces
		FbxGeometryElementMaterial* const fbxMaterialElement = fbxMesh->CreateElementMaterial();
		//fbxMaterialElement->SetMappingMode(FbxGeometryElement::eAllSame);
		fbxMaterialElement->SetMappingMode(FbxGeometryElement::eByPolygon);
		fbxMaterialElement->SetReferenceMode(FbxGeometryElement::eIndexToDirect);
		//fbxMaterialElement->GetIndexArray().Add(i);
		fbxLayer->SetMaterials(fbxMaterialElement);

		ndInt32 mark = geometry->IncLRU();
		ndMeshEffect::Iterator iter(**geometry);
		for (iter.Begin(); iter; iter++)
		{
			ndEdge* const face = &iter.GetNode()->GetInfo();

			ndInt32 materialIndex = materialId[face->m_userData];
			if ((face->m_mark != mark) && (face->m_incidentFace > 0))
			{
				fbxMesh->BeginPolygon(materialIndex);
				ndEdge* edgePtr = face;
				do
				{
					fbxMesh->AddPolygon(edgePtr->m_incidentVertex);
					edgePtr->m_mark = mark;
					edgePtr = edgePtr->m_next;
				} while (edgePtr != face);

				fbxMesh->EndPolygon();
			}
			face->m_mark = mark;
		}
	}

	FbxGeometryElementUV* const uvElement = fbxMesh->CreateElementUV("UVChannel_1");
	uvElement->SetMappingMode(FbxGeometryElement::eByPolygonVertex);
	uvElement->SetReferenceMode(FbxGeometryElement::eIndexToDirect);
	fbxLayer->SetUVs(uvElement);

	const ndMeshEffect::ndChannel<ndMeshEffect::ndUV, ndMeshEffect::m_uv0>& uvArray = vertexAtributes.m_uv0Channel;
	for (ndInt32 i = 0; i < uvArray.GetCount(); ++i)
	{
		ndMeshEffect::ndUV uv(uvArray[i]);
		uvElement->GetDirectArray().Add(FbxVector2(uv.m_u, uv.m_v));
	}
	{
		//add uv maping
		ndInt32 mark = geometry->IncLRU();
		ndMeshEffect::Iterator iter(**geometry);
		for (iter.Begin(); iter; iter++)
		{
			ndEdge* const face = &iter.GetNode()->GetInfo();
			if ((face->m_mark != mark) && (face->m_incidentFace > 0))
			{
				ndEdge* edgePtr = face;
				do
				{
					edgePtr->m_mark = mark;
					ndInt32 uvPointIndex = ndInt32 (edgePtr->m_userData);
					uvElement->GetIndexArray().Add(uvPointIndex);
					edgePtr = edgePtr->m_next;
				} while (edgePtr != face);
			}
			face->m_mark = mark;
		}
	}

	// skip normals, let the app calculate it

	return fbxMesh;
}

void CreateGeometries(ndMesh* const model, FbxScene* const fbxScene, const ndString& path)
{
	auto GenerateGeometry = [fbxScene, &path](ndMesh* const node)
	{
		if (node->GetGeometry())
		{
			FbxNode* const fbxNode = fbxScene->FindNodeByName(FbxString (node->GetName().GetStr()));
			if (node->GetGeometry())
			{
				FbxMesh* const mesh = CreateGeometry(node, fbxScene, fbxNode, path);
				fbxNode->SetNodeAttribute(mesh);
				fbxNode->SetShadingMode(FbxNode::eTextureShading);
			}
		}
	};
	model->NodeIterator(GenerateGeometry);
}

bool CreateScene(ndMesh* const model, FbxManager* const sdkManager, FbxScene* const fbxScene, const ndString& path)
{
	// create sbxScene info
	FbxDocumentInfo* const sceneInfo = FbxDocumentInfo::Create(sdkManager, "SceneInfo");
	sceneInfo->mTitle = "";
	sceneInfo->mSubject = "";
	sceneInfo->mAuthor = "Newton Dynamics";
	sceneInfo->mRevision = "rev. 1.0";
	sceneInfo->mKeywords = "";
	sceneInfo->mComment = "";

	// we need to add the sceneInfo before calling AddThumbNailToScene because
	// that function is asking the sbxScene for the sceneInfo.
	fbxScene->SetSceneInfo(sceneInfo);

	FbxGlobalSettings& globalSettings = fbxScene->GetGlobalSettings();
	globalSettings.SetSystemUnit(100.0f);
	globalSettings.SetOriginalSystemUnit(100.0f);

	FbxNode* const fbxSceneRootNode = fbxScene->GetRootNode();
	FbxNode* const fbxModelRootNode = CreateSkeleton(model, fbxScene);
	CreateGeometries(model, fbxScene, path);
	fbxSceneRootNode->AddChild(fbxModelRootNode);
#if 0
	//// Store poses
	//LinkPatchToSkeleton(sbxScene, lPatch, fbxModelRootNode);
	//StoreBindPose(sbxScene, lPatch);
	//StoreRestPose(sbxScene, fbxModelRootNode);

	// Animation
	AnimateSkeleton(model, fbxScene, fbxModelRootNode);
#endif
	return true;
}

bool SaveScene(FbxManager* const fbsManager, FbxDocument* const fbsScene, const char* const filename, int fileFormat = -1, bool embedMedia = false)
{
	int major, minor, revision;
	bool status = true;

	// Create an exporter.
	FbxExporter* const fbxExporter = FbxExporter::Create(fbsManager, "");

	if (fileFormat < 0 || fileFormat >= fbsManager->GetIOPluginRegistry()->GetWriterFormatCount())
	{
		// Write in fall back format in less no ASCII format found
		fileFormat = fbsManager->GetIOPluginRegistry()->GetNativeWriterFormat();

		//Try to export in ASCII if possible
		int	lFormatCount = fbsManager->GetIOPluginRegistry()->GetWriterFormatCount();

		for (int lFormatIndex = 0; lFormatIndex < lFormatCount; lFormatIndex++)
		{
			if (fbsManager->GetIOPluginRegistry()->WriterIsFBX(lFormatIndex))
			{
				FbxString lDesc = fbsManager->GetIOPluginRegistry()->GetWriterFormatDescription(lFormatIndex);
				const char* lASCII = "ascii";
				//const char *lASCII = "binary";
				if (lDesc.Find(lASCII) >= 0)
				{
					fileFormat = lFormatIndex;
					break;
				}
			}
		}
	}

	// Set the export states. By default, the export states are always set to 
	// true except for the option eEXPORT_TEXTURE_AS_EMBEDDED. The code below 
	// shows how to change these states.
	//IOS_REF.SetBoolProp(EXP_FBX_MATERIAL, true);
	//IOS_REF.SetBoolProp(EXP_FBX_TEXTURE, true);
	//IOS_REF.SetBoolProp(EXP_FBX_EMBEDDED, embedMedia);
	//IOS_REF.SetBoolProp(EXP_FBX_SHAPE, true);
	//IOS_REF.SetBoolProp(EXP_FBX_GOBO, true);
	//IOS_REF.SetBoolProp(EXP_FBX_ANIMATION, true);
	//IOS_REF.SetBoolProp(EXP_FBX_GLOBAL_SETTINGS, true);

	// Initialize the exporter by providing a filename.
	if (fbxExporter->Initialize(filename, fileFormat, fbsManager->GetIOSettings()) == false)
	{
		FBXSDK_printf("Call to FbxExporter::Initialize() failed.\n");
		FBXSDK_printf("Error returned: %s\n\n", fbxExporter->GetStatus().GetErrorString());
		return false;
	}

	FbxManager::GetFileFormatVersion(major, minor, revision);
	FBXSDK_printf("FBX file format version %d.%d.%d\n\n", major, minor, revision);

	// Export the fbsScene.
	status = fbxExporter->Export(fbsScene);

	// Destroy the exporter.
	fbxExporter->Destroy();
	return status;
}

#if 0
void AnimateSkeleton(const exportMeshNode* const model, FbxScene* const fbxScene, FbxNode* const fbxModelRoot)
{
	FbxAnimStack* const animStack = FbxAnimStack::Create(fbxScene, "animStack");
	FbxAnimLayer* const animLayer = FbxAnimLayer::Create(fbxScene, "baseLayer");	// the AnimLayer object name is "Base Layer"
	animStack->AddMember(animLayer);

	double fps = 1.0f / 30.0f;
	const exportMeshNode* nodePool[256];
	//int stack = 1;
	//nodePool[0] = model;

	int stack = 0;
	for (std::list<exportMeshNode*>::const_iterator iter = model->m_children.begin();
		iter != model->m_children.end(); iter++)
	{
		nodePool[stack] = *iter;
		stack++;
	}

	while (stack)
	{
		stack--;
		const exportMeshNode* const node = nodePool[stack];

		//if (node->m_keyFrame.size())
		//{
		//	FbxNode* const fbxNode = node->m_fbxNode;
		//	fbxNode->LclTranslation.GetCurveNode(animLayer, true);
		//	FbxAnimCurve* const curvePositX = fbxNode->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_X, true);
		//	FbxAnimCurve* const curvePositY = fbxNode->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Y, true);
		//	FbxAnimCurve* const curvePositZ = fbxNode->LclTranslation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Z, true);
		//	FbxAnimCurve* const curveRotationX = fbxNode->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_X, true);
		//	FbxAnimCurve* const curveRotationY = fbxNode->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Y, true);
		//	FbxAnimCurve* const curveRotationZ = fbxNode->LclRotation.GetCurve(animLayer, FBXSDK_CURVENODE_COMPONENT_Z, true);
		//
		//	FbxTime lTime;
		//	curvePositX->KeyModifyBegin();
		//	curvePositY->KeyModifyBegin();
		//	curvePositZ->KeyModifyBegin();
		//	curveRotationX->KeyModifyBegin();
		//	curveRotationY->KeyModifyBegin();
		//	curveRotationZ->KeyModifyBegin();
		//
		//	exportVector euler1;
		//	exportVector eulerRef(node->m_keyFrame[0].CalcPitchYawRoll(euler1));
		//	for (int i = 0; i < node->m_keyFrame.size(); ++i)
		//	{
		//		double time = double(i) * fps;
		//		lTime.SetSecondDouble(time);
		//	
		//		exportVector posit(node->m_keyFrame[i].m_posit);
		//	
		//		//if (node->m_name == "Hips")
		//		//{
		//		//	//posit.m_x = 0;
		//		//	posit.m_z = 0;
		//		//}
		//	
		//		int keyIndexPositX = curvePositX->KeyAdd(lTime);
		//		curvePositX->KeySetValue(keyIndexPositX, posit.m_x);
		//		curvePositX->KeySetInterpolation(keyIndexPositX, FbxAnimCurveDef::eInterpolationCubic);
		//	
		//		int keyIndexPositY = curvePositY->KeyAdd(lTime);
		//		curvePositY->KeySetValue(keyIndexPositY, posit.m_y);
		//		curvePositY->KeySetInterpolation(keyIndexPositY, FbxAnimCurveDef::eInterpolationCubic);
		//	
		//		int keyIndexPositZ = curvePositZ->KeyAdd(lTime);
		//		curvePositZ->KeySetValue(keyIndexPositZ, posit.m_z);
		//		curvePositZ->KeySetInterpolation(keyIndexPositZ, FbxAnimCurveDef::eInterpolationCubic);
		//
		//		exportVector euler0 (node->m_keyFrame[i].CalcPitchYawRoll(euler1));
		//		float angleError = node->CalculateDeltaAngle(euler0.m_z, eulerRef.m_z);
		//		if (fabsf(angleError) > 90.0 * M_PI / 180.0f)
		//		{
		//			euler0 = euler1;
		//		}
		//		float deltax = node->CalculateDeltaAngle(euler0.m_x, eulerRef.m_x);
		//		float deltay = node->CalculateDeltaAngle(euler0.m_y, eulerRef.m_y);
		//		float deltaz = node->CalculateDeltaAngle(euler0.m_z, eulerRef.m_z);
		//
		//		eulerRef.m_x += deltax;
		//		eulerRef.m_y += deltay;
		//		eulerRef.m_z += deltaz;
		//		exportVector eulers(eulerRef.Scale(180.0f / M_PI));
		//		
		//		int keyIndexRotationX = curveRotationX->KeyAdd(lTime);
		//		curveRotationX->KeySetValue(keyIndexRotationX, eulers.m_x);
		//		curveRotationX->KeySetInterpolation(keyIndexRotationX, FbxAnimCurveDef::eInterpolationCubic);
		//		
		//		int keyIndexRotationY = curveRotationY->KeyAdd(lTime);
		//		curveRotationY->KeySetValue(keyIndexRotationY, eulers.m_y);
		//		curveRotationY->KeySetInterpolation(keyIndexRotationY, FbxAnimCurveDef::eInterpolationCubic);
		//		
		//		int keyIndexRotationZ = curveRotationZ->KeyAdd(lTime);
		//		curveRotationZ->KeySetValue(keyIndexRotationZ, eulers.m_z);
		//		curveRotationZ->KeySetInterpolation(keyIndexRotationZ, FbxAnimCurveDef::eInterpolationCubic);
		//	}
		//
		//	curvePositX->KeyModifyEnd();
		//	curvePositY->KeyModifyEnd();
		//	curvePositZ->KeyModifyEnd();
		//	curveRotationX->KeyModifyEnd();
		//	curveRotationY->KeyModifyEnd();
		//	curveRotationZ->KeyModifyEnd();
		//}

		for (std::list<exportMeshNode*>::const_iterator iter = node->m_children.begin();
			iter != node->m_children.end(); iter++)
		{
			nodePool[stack] = *iter;
			stack++;
		}
	}
}
#endif

int main(int argc, char** argv)
{
	const char* name = nullptr;
	if ((argc > 1) && strstr(argv[1], ".nd"))
	{
		name = argv[1];
	}
	if (!name)
	{
		printf("usage meshToFbx [nd_file_name]\n");
		return 0;
	}

	const ndString ndPath(name);
	ndString fbxPath (ndPath.GetPath() + ndPath.GetName() + ".fbx");

	ndMeshLoader loader;
	loader.LoadMesh(ndPath);
	if (!loader.m_mesh)
	{
		printf("file not found %s\n", ndPath.GetStr());
		return 0;
	}
	loader.m_mesh->ApplyTransform(ndYawMatrix(ndFloat32(-90.0f) * ndDegreeToRad));

	FbxScene* fbxScene = nullptr;
	FbxManager* fbxManager = nullptr;
	if (!InitializeSdkObjects(fbxManager, fbxScene))
	{
		FBXSDK_printf("failed to initialize fbx sdk: %s\n", name);
		return 0;
	}

	// Create the scene.
	bool lResult = CreateScene(*loader.m_mesh, fbxManager, fbxScene, fbxPath.GetPath());
	
	if (lResult == false)
	{
		FBXSDK_printf("\n\nAn error occurred while creating the fbsScene...\n");
		fbxManager->Destroy();
		return 0;
	}
	
	// Save the fbsScene.
	lResult = SaveScene(fbxManager, fbxScene, fbxPath.GetStr());
	if (lResult == false)
	{
		FBXSDK_printf("\n\nAn error occurred while saving the fbsScene...\n");
		fbxManager->Destroy();
		return 0;
	}
	
	fbxManager->Destroy();

	return 0;
}


