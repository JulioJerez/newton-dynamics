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
#include "ndMenuRenderPass.h"
#include "ndAssetEditor.h"
#include "ndDebugDisplayRenderPass.h"

ndDebugDisplayRenderPass::ndDebugDisplayRenderPass(ndAssetEditor* const owner)
	:ndRenderPassDebug(*owner->GetRenderer(), nullptr)
	,m_meshColor(ndFloat32(1.0f))
	,m_shapeColor(ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(1.0f))
	,m_selectedColor(ndFloat32(0.42f), ndFloat32(0.73f), ndFloat32(0.98f), ndFloat32(1.0f))
	,m_loopJointColor(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f))
	,m_collidingPairColor0(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f))
	,m_collidingPairColor1(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(1.0f))
	,m_collidingPairPreviewColor(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f))
	,m_manager(owner)
{
}

ndDebugDisplayRenderPass::~ndDebugDisplayRenderPass()
{
}

void ndDebugDisplayRenderPass::RenderBoneSelection()
{
	m_owner->ClearZBuffer();
	RenderSkeleton();

	RenderDynamicPrimitives();
	ClearDynamicPrimitives();
	m_owner->ClearZBuffer();

	//if (m_manager->m_renderMode == ndAssetEditor::m_wireframe)
	//{
	//	RenderWireFrame();
	//}
	//else if (m_manager->m_renderMode == ndAssetEditor::m_hiddenSurface)
	//{
	//	RenderHiddenSurface();
	//	RenderWireFrame();
	//}

	if (m_manager->m_currentSelection)
	{
		if (m_manager->m_currentSelection->GetNodeType() == ndMesh::m_bone)
		{
			DrawSelectedBone(*m_manager->m_currentSelection);
		}
		//if (m_manager->m_currentSelection->GetAsMesh())
		//{
		//	RenderSelectedNode();
		//}
		//
		//if (m_debugLines.GetCount())
		//{
		//	const ndMatrix matrix(ndGetIdentityMatrix());
		//	m_renderLinesPrimitive->Render(m_owner, matrix, m_debugLineArray);
		//}
		//
		//// do not call base class
		//if (m_manager->m_renderMode == ndAssetEditor::m_wireframe)
		//{
		//	RenderWireFrame();
		//}
		//else if (m_manager->m_renderMode == ndAssetEditor::m_hiddenSurface)
		//{
		//	RenderHiddenSurface();
		//	RenderWireFrame();
		//}
		//
		if (m_manager->m_currentSelection->GetAsMesh())
		{
		//	if (m_manager->m_showCollisionShape)
		//	{
		//		RenderCollisionShape();
		//	}
			m_owner->ClearZBuffer();
			RenderOptions();
		}
		else if (m_manager->m_currentSelection->GetAsCloseLoopConstraints())
		{
			RenderCloseLoopJoints();
		}
		else if (m_manager->m_currentSelection->GetAsCollidingPairs())
		{
			RenderCollisionPair();
		}
	}
}

void ndDebugDisplayRenderPass::RenderScene()
{
	ClearDynamicPrimitives();
	if (m_manager->m_raycastBones)
	{
		RenderBoneSelection();
	}
	else
	{
		RenderMeshSelection();
	}
	RenderDynamicPrimitives();
}

void ndDebugDisplayRenderPass::RebuildVisualDebugMesh()
{
	const ndString& selected = m_manager->m_currentSelection->GetName();
	ndAssert(m_manager->m_currentSelection->GetRigidBody());
	ndSharedPtr<ndMeshBody> body(m_manager->m_currentSelection->GetRigidBody());
	ndMeshBodyKinematic* const kinematicBody = (ndMeshBodyKinematic*)*body;
	for (ndList<ndDebugMesh>::ndNode* ptr = m_debugMesh.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		ndDebugMesh& debugMesh = ptr->GetInfo();
		if (debugMesh.m_parent->m_name == selected)
		{
			const ndMeshShapeInstance shapeInstance = kinematicBody->m_shapeInstance;
			if (strcmp(shapeInstance.m_shape->m_constructor.GetStr(), ndShapeNull::StaticClassName()) == 0)
			{
				debugMesh.m_zBufferShape = ndSharedPtr<ndRenderPrimitive>(nullptr);
				debugMesh.m_wireFrameShape = ndSharedPtr<ndRenderPrimitive>(nullptr);
			}
			else
			{
				ndRenderPrimitive::ndDescriptor descriptor(m_owner);
				descriptor.m_collision = ndSharedPtr<ndShapeInstance>(kinematicBody->m_shapeInstance.CreateObject());
				//descriptor.m_collision->SetScale(ndVector::m_one);
				//descriptor.m_collision->SetLocalMatrix(ndGetIdentityMatrix());
				descriptor.m_collision->SetScale(kinematicBody->m_shapeInstance.m_scale);
				descriptor.m_collision->SetLocalMatrix(kinematicBody->m_shapeInstance.m_localMatrix);
				descriptor.m_collision->m_alignmentMatrix = kinematicBody->m_shapeInstance.m_alignmentMatrix;

				descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugWireFrame;
				debugMesh.m_wireFrameMesh = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));
			}
			break;
		}
	}
}

void ndDebugDisplayRenderPass::RebuildDebugCollision()
{
	const ndString& selected = m_manager->m_currentSelection->GetName();
	ndAssert(m_manager->m_currentSelection->GetRigidBody());
	ndSharedPtr<ndMeshBody> body(m_manager->m_currentSelection->GetRigidBody());
	ndMeshBodyKinematic* const kinematicBody = (ndMeshBodyKinematic*)*body;
	for (ndList<ndDebugMesh>::ndNode* ptr = m_debugMesh.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		ndDebugMesh& debugMesh = ptr->GetInfo();
		if (debugMesh.m_parent->m_name == selected)
		{
			const ndMeshShapeInstance shapeInstance = kinematicBody->m_shapeInstance;
			if (strcmp(shapeInstance.m_shape->m_constructor.GetStr(), ndShapeNull::StaticClassName()) == 0)
			{
				debugMesh.m_zBufferShape = ndSharedPtr<ndRenderPrimitive>(nullptr);
				debugMesh.m_wireFrameShape = ndSharedPtr<ndRenderPrimitive>(nullptr);
			}
			else
			{
				ndRenderPrimitive::ndDescriptor descriptor(m_owner);
				descriptor.m_collision = ndSharedPtr<ndShapeInstance>(kinematicBody->m_shapeInstance.CreateObject());
				descriptor.m_collision->SetScale(ndVector::m_one);
				descriptor.m_collision->SetLocalMatrix(ndGetIdentityMatrix());

				descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugHiddenLines;
				debugMesh.m_zBufferShape = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));

				descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugWireFrame;
				debugMesh.m_wireFrameShape = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));
			}
			break;
		}
	}
}

void ndDebugDisplayRenderPass::ResetScene()
{
	m_debugMesh.RemoveAll();

	if (!*m_manager->m_mesh)
	{
		return;
	}

	auto BuildDebugMesh = [this](ndMesh* const node)
	{
		ndSharedPtr<ndMeshBody> body(node->GetRigidBody());
		ndSharedPtr<ndMeshEffect>& geometry = node->GetGeometry();

		if (*geometry || *body)
		{
			ndDebugMesh& entry = m_debugMesh.Append()->GetInfo();
			entry.m_parent = ndWeakPtr<ndRenderSceneNode>(m_manager->m_entity->FindByName(node->GetName()));

			if (*geometry)
			{
				ndRenderPrimitive::ndDescriptor descriptor(m_owner);
				descriptor.m_meshNode = geometry;

				descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugHiddenLines;
				entry.m_zBufferMesh = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));

				descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugWireFrame;
				entry.m_wireFrameMesh = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));

				descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugFlatShaded;
				entry.m_flatShadedMesh = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));
			}

			if (*body)
			{
				ndMeshBodyKinematic* const kinBody = (ndMeshBodyKinematic*)*body;

				ndRenderPrimitive::ndDescriptor descriptor(m_owner);
				descriptor.m_collision = ndSharedPtr<ndShapeInstance>(kinBody->m_shapeInstance.CreateObject());
				descriptor.m_collision->SetScale(ndVector::m_one);
				descriptor.m_collision->SetLocalMatrix(ndGetIdentityMatrix());

				descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugHiddenLines;
				entry.m_zBufferShape = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));

				descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugWireFrame;
				entry.m_wireFrameShape = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));
			}
		}
	};
	m_manager->m_mesh->NodeIterator(BuildDebugMesh);
}

void ndDebugDisplayRenderPass::DrawLine(const ndVector& p0, const ndVector& p1, const ndVector& color)
{
	ndPointColor line;

	line.m_point = p0;
	line.m_color = color;
	m_debugLines.PushBack(line);

	line.m_point = p1;
	line.m_color = color;
	m_debugLines.PushBack(line);
}

ndFixSizeArray<ndDebugDisplayRenderPass::ndPointNormalColor, 256> ndDebugDisplayRenderPass::GenerateBone(const ndMesh* boneNode)
{
	const ndVector localPoint(boneNode->GetBoneTarget());

	ndFloat32 dist = localPoint.m_x * ndFloat32(0.1f);
	ndVector points[32];
	points[0] = ndVector(dist, dist, dist, ndFloat32(1.0f));
	points[1] = ndVector(dist, dist, -dist, ndFloat32(1.0f));
	points[2] = ndVector(dist, -dist, -dist, ndFloat32(1.0f));
	points[3] = ndVector(dist, -dist, dist, ndFloat32(1.0f));

	ndInt32 i0 = 3;
	ndVector testPoint(ndVector::m_wOne);
	testPoint.m_x = dist;

	ndFixSizeArray<ndPointNormalColor, 256> temp;
	for (ndInt32 i = 0; i < 4; ++i)
	{
		{
			ndVector p0(0.0f, 0.0f, 0.0f, 1.0f);
			ndVector p1(points[i]);
			ndVector p2(points[i0]);
			ndVector e0(p1 - p0);
			ndVector e1(p2 - p0);
			ndVector normal(e0.CrossProduct(e1) & ndVector::m_triplexMask);
			if (normal.DotProduct(normal).GetScalar() < ndFloat32 (1.0e-8f))
			{
				normal.m_x = ndFloat32 (1.0f);
			}
			normal = normal.Normalize();

			if (normal.DotProduct(testPoint - p0).GetScalar() > ndFloat32 (0.0f))
			{
				ndSwap(p1, p2);
				normal = normal * ndVector::m_negOne;
			}

			ndPointNormalColor trianglePoint;
			trianglePoint.m_normal = normal;
			trianglePoint.m_color = ndVector(0.6f, 0.6f, 0.6f, 1.0f);

			trianglePoint.m_point = p0;
			temp.PushBack(trianglePoint);

			trianglePoint.m_point = p1;
			temp.PushBack(trianglePoint);

			trianglePoint.m_point = p2;
			temp.PushBack(trianglePoint);
		}

		{
			ndVector p0(localPoint);
			ndVector p1(points[i0]);
			ndVector p2(points[i]);
			ndVector e0(p1 - p0);
			ndVector e1(p2 - p0);
			ndVector normal(e0.CrossProduct(e1) & ndVector::m_triplexMask);
			if (normal.DotProduct(normal).GetScalar() < ndFloat32(1.0e-8f))
			{
				normal.m_x = ndFloat32(1.0f);
			}
			normal = normal.Normalize();

			if (normal.DotProduct(testPoint - p0).GetScalar() > ndFloat32(0.0f))
			{
				ndSwap(p1, p2);
				normal = normal * ndVector::m_negOne;
			}

			ndPointNormalColor trianglePoint;
			trianglePoint.m_normal = normal;
			trianglePoint.m_color = ndVector(0.6f, 0.6f, 0.6f, 1.0f);

			trianglePoint.m_point = p0;
			temp.PushBack(trianglePoint);

			trianglePoint.m_point = p1;
			temp.PushBack(trianglePoint);

			trianglePoint.m_point = p2;
			temp.PushBack(trianglePoint);
		}

		i0 = i;
	}

	const ndMatrix boneMatrix(boneNode->CalculateGlobalMatrix());
	for (ndInt32 i = 0; i < temp.GetCount(); ++i)
	{
		temp[i].m_point = boneMatrix.TransformVector(temp[i].m_point);
		temp[i].m_normal = boneMatrix.RotateVector(temp[i].m_normal);
	}
	return temp;
}

void ndDebugDisplayRenderPass::DrawBone(const ndMesh* const boneNode)
{
	ndFixSizeArray<ndPointNormalColor, 256> bone(GenerateBone(boneNode));

	for (ndInt32 i = 0; i < bone.GetCount(); ++i)
	{
		m_debugTriangles.PushBack(bone[i]);
	}
}

void ndDebugDisplayRenderPass::DrawSelectedBone(const ndMesh* const boneNode)
{
	ndFixSizeArray<ndPointNormalColor, 256> bone(GenerateBone(boneNode));

	for (ndInt32 i = 0; i < bone.GetCount(); i += 3)
	{
		ndInt32 j0 = 2;
		for (ndInt32 j1 = 0; j1 < 3; ++j1)
		{
			ndPointColor line;

			line.m_point = bone[i + j0].m_point;
			line.m_color = m_selectedColor;
			m_debugLines.PushBack(line);

			line.m_point = bone[i + j1].m_point;
			line.m_color = m_selectedColor;
			m_debugLines.PushBack(line);

			j0 = j1;
		}
	}
}

void ndDebugDisplayRenderPass::DrawFrame(const ndMatrix& matrix)
{
	ndReal debugScale = m_manager->m_gizmoScale;
	ndVector x(matrix.m_posit + matrix.RotateVector(ndVector(debugScale, ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f))));
	DrawLine(matrix.m_posit, x, ndVector(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f)));
	
	ndVector y(matrix.m_posit + matrix.RotateVector(ndVector(ndFloat32(0.0f), debugScale, ndFloat32(0.0f), ndFloat32(0.0f))));
	DrawLine(matrix.m_posit, y, ndVector(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(1.0f)));
	
	ndVector z(matrix.m_posit + matrix.RotateVector(ndVector(ndFloat32(0.0f), ndFloat32(0.0f), debugScale, ndFloat32(0.0f))));
	DrawLine(matrix.m_posit, z, ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(1.0f)));
}

void ndDebugDisplayRenderPass::RenderSelectedNode()
{
	auto RenderNode = [this](const ndMesh* const node, const ndVector& color)
	{
		const ndString& selected = node->GetName();
		for (ndList<ndDebugMesh>::ndNode* ptr = m_debugMesh.GetFirst(); ptr; ptr = ptr->GetNext())
		{
			const ndDebugMesh& debugMesh = ptr->GetInfo();
			if (m_manager->m_showSelectedNode)
			{
				const ndRenderPrimitive* const primitive = *debugMesh.m_wireFrameMesh;
				if (primitive && primitive->m_segments.GetCount())
				{
					const ndMatrix pivotMatrix(debugMesh.m_parent->m_globalMatrix);
					const ndMatrix gemetryMatrix(debugMesh.m_parent->m_primitiveMatrix * pivotMatrix);

					ndRenderPrimitiveSegment& segment = primitive->m_segments.GetFirst()->GetInfo();
					ndRenderPrimitiveMaterial* const material = &segment.m_material;

					if (debugMesh.m_parent->m_name == selected)
					{
						material->m_diffuse = color;
						primitive->Render(m_owner, gemetryMatrix, m_debugDisplayWireFrameMesh);
					}
				}
			}
		}
	};
	RenderNode(*m_manager->m_currentSelection, m_selectedColor);

	ndSharedPtr<ndMeshTransformModifier> modifier(m_manager->m_currentSelection->GetModifier());
	if (modifier)
	{
		if (modifier->m_target)
		{
			RenderNode(*modifier->m_target, m_collidingPairColor1);
		}

		ndFixSizeArray<const ndMesh*, 256> subNodes(modifier->GetAffectedNodes());
		for (ndInt32 i = 0; i < subNodes.GetCount(); ++i)
		{
			RenderNode(subNodes[i], m_collidingPairColor1);
		}
	}
	if (m_manager->m_subSelection == ndAssetEditor::m_alignToTarget)
	{
		if (m_manager->m_currentSubSelection)
		{
			RenderNode(*m_manager->m_currentSubSelection, m_collidingPairColor1);
		}
	}
}

void ndDebugDisplayRenderPass::RenderCollisionShape()
{
	const ndMesh* const selection = *m_manager->m_currentSelection;
	ndAssert(selection);
	if (selection->GetRigidBody())
	{
		auto DisplayShape = [this](const ndMesh* const mesh, const ndVector& color)
		{
			const ndMesh* bodyMesh = mesh;
			while (!bodyMesh->GetRigidBody())
			{
				bodyMesh = bodyMesh->GetParent();
			}

			const ndString& selected = bodyMesh->GetName();
			for (ndList<ndDebugMesh>::ndNode* ptr = m_debugMesh.GetFirst(); ptr; ptr = ptr->GetNext())
			{
				const ndDebugMesh& debugMesh = ptr->GetInfo();
				if ((debugMesh.m_parent->m_name == selected))
				{
					ndSharedPtr<ndMeshBody> body(bodyMesh->GetRigidBody());
					const ndMeshBodyKinematic* const kinBody = (ndMeshBodyKinematic*)*body;
					const ndVector scale(kinBody->m_shapeInstance.m_scale);
					ndMatrix scaleMatrix(ndGetIdentityMatrix());
					scaleMatrix[0][0] = scale[0];
					scaleMatrix[1][1] = scale[1];
					scaleMatrix[2][2] = scale[2];
					const ndMatrix pivotMatrix(scaleMatrix * kinBody->m_shapeInstance.m_localMatrix * debugMesh.m_parent->m_globalMatrix);
	
					if (m_manager->m_showSelectedNode)
					{
						const ndRenderPrimitive* const primitive = *debugMesh.m_zBufferShape;
						if (primitive && debugMesh.m_wireFrameShape->m_segments.GetCount())
						{
							primitive->Render(m_owner, pivotMatrix, m_debugDisplaySetZbuffer);
							ndRenderPrimitiveSegment& segment = debugMesh.m_wireFrameShape->m_segments.GetFirst()->GetInfo();
							ndRenderPrimitiveMaterial* const material = &segment.m_material;
							material->m_diffuse = color;
							debugMesh.m_wireFrameShape->Render(m_owner, pivotMatrix, m_debugDisplayWireFrameMesh);
						}
					}
				}
			}
		};
		//const ndVector color (selection->GetAsMesh() ? m_collidingPairColor0 : m_shapeColor);
		const ndVector color(selection->GetAsMesh() ? m_shapeColor : m_collidingPairColor0);
		DisplayShape(selection, color);
	
		switch (m_manager->m_subSelection)
		{
			case ndAssetEditor::m_collidingPair:
			{
				const ndCollidingPairs* const collidingPairs = m_manager->m_mesh->GetCollingPairs();
				ndAssert(collidingPairs);
				for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* node = collidingPairs->m_collidingPairs.GetFirst(); node; node = node->GetNext())
				{
					const ndSharedPtr<ndMeshCollidingPair>& self = node->GetInfo();
					if (*self->m_childNode == *m_manager->m_currentSelection)
					{
						DisplayShape(*self->m_parentNode, m_collidingPairColor1);
					}
					else if (*self->m_parentNode == *m_manager->m_currentSelection)
					{
						DisplayShape(*self->m_childNode, m_collidingPairColor1);
					}
				}

				if (m_manager->m_currentSubSelection)
				{
					DisplayShape(*m_manager->m_currentSubSelection, m_collidingPairPreviewColor);
				}
				break;
			}
			case ndAssetEditor::m_loopJoint:
			{
				if (m_manager->m_currentSubSelection)
				{
					DisplayShape(*m_manager->m_currentSubSelection, m_collidingPairPreviewColor);
				}
				break;
			}

			case ndAssetEditor::m_transformModifier:
			{
				//ndAssert(0);
				if (m_manager->m_currentSubSelection)
				{
					DisplayShape(*m_manager->m_currentSubSelection, m_collidingPairPreviewColor);
				}
				break;
			}

			default:;
			{

			}
		}
	}
}

void ndDebugDisplayRenderPass::RenderCloseLoopJoints()
{
	ndCloseLoopConstraints* const loops = m_manager->m_currentSelection->GetAsCloseLoopConstraints();;
	ndAssert(loops);

	ndInt32 i = 0;
	ndSharedPtr<ndMeshLoopJoint> loop(nullptr);
	for (ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* ptr = loops->m_loopJoints.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		if (i == m_manager->m_closeLoopIndex)
		{
			loop = ptr->GetInfo();
			break;
		}
		i++;
	}

	if (loop)
	{
		const ndMeshLoopJoint* const currentLoopJointSelection = *loop;

		// draw Geometry
		for (ndList<ndDebugMesh>::ndNode* ptr = m_debugMesh.GetFirst(); ptr; ptr = ptr->GetNext())
		{
			const ndDebugMesh& debugMesh = ptr->GetInfo();
			const ndRenderPrimitive* const primitive = *debugMesh.m_zBufferShape;
			if (primitive && primitive->m_segments.GetCount())
			{
				ndSharedPtr<ndMeshBody> body(nullptr);
				if (currentLoopJointSelection->m_childNode->GetName() == debugMesh.m_parent->m_name)
				{
					body = currentLoopJointSelection->m_childNode->GetRigidBody();
				}
				else if (currentLoopJointSelection->m_parentNode->GetName() == debugMesh.m_parent->m_name)
				{
					body = currentLoopJointSelection->m_parentNode->GetRigidBody();
				}
				if (body)
				{
					const ndMeshBodyKinematic* const kinBody = (ndMeshBodyKinematic*)*body;
					const ndVector scale(kinBody->m_shapeInstance.m_scale);
					ndMatrix scaleMatrix(ndGetIdentityMatrix());
					scaleMatrix[0][0] = scale[0];
					scaleMatrix[1][1] = scale[1];
					scaleMatrix[2][2] = scale[2];
					const ndMatrix pivotMatrix(scaleMatrix * kinBody->m_shapeInstance.m_localMatrix * debugMesh.m_parent->m_globalMatrix);
					primitive->Render(m_owner, pivotMatrix, m_debugDisplaySetZbuffer);

					ndRenderPrimitiveSegment& segment = debugMesh.m_wireFrameShape->m_segments.GetFirst()->GetInfo();
					ndRenderPrimitiveMaterial* const material = &segment.m_material;
					material->m_diffuse = m_loopJointColor;
					debugMesh.m_wireFrameShape->Render(m_owner, pivotMatrix, m_debugDisplayWireFrameMesh);
				}
			}
		}

		// draw Gizmo
		m_owner->ClearZBuffer();
		for (ndList<ndDebugMesh>::ndNode* ptr = m_debugMesh.GetFirst(); ptr; ptr = ptr->GetNext())
		{
			const ndDebugMesh& debugMesh = ptr->GetInfo();
			const ndRenderPrimitive* const primitive = *debugMesh.m_zBufferShape;
			if (primitive && primitive->m_segments.GetCount())
			{
				if (currentLoopJointSelection->m_childNode->GetName() == debugMesh.m_parent->m_name)
				{
					const ndSharedPtr<ndMeshBody> body (currentLoopJointSelection->m_childNode->GetRigidBody());
					const ndMatrix frame(currentLoopJointSelection->m_joint->m_localFrame0 * debugMesh.m_parent->m_globalMatrix);
					DrawFrame(frame);
				}
				else if (currentLoopJointSelection->m_parentNode->GetName() == debugMesh.m_parent->m_name)
				{
					const ndSharedPtr<ndMeshBody> body(currentLoopJointSelection->m_parentNode->GetRigidBody());
					const ndMatrix frame(currentLoopJointSelection->m_joint->m_localFrame1 * debugMesh.m_parent->m_globalMatrix);
					DrawFrame(frame);
				}
			}
		}
	}
}

void ndDebugDisplayRenderPass::RenderCollisionPair()
{
	ndCollidingPairs* const collidingPairs = m_manager->m_currentSelection->GetAsCollidingPairs();
	if (!collidingPairs->m_collidingPairs.GetCount())
	{
		return;
	}
	
	ndInt32 i = 0;
	for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* ptr = collidingPairs->m_collidingPairs.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		if (i == m_manager->m_collidingPairIndex)
		{
			auto DisplayShape = [this](const ndMesh* const mesh, const ndVector& color)
			{
				const ndString& selected = mesh->GetName();
				for (ndList<ndDebugMesh>::ndNode* ptr = m_debugMesh.GetFirst(); ptr; ptr = ptr->GetNext())
				{
					const ndDebugMesh& debugMesh = ptr->GetInfo();
					if ((debugMesh.m_parent->m_name == selected))
					{
						ndSharedPtr<ndMeshBody> body(mesh->GetRigidBody());
						const ndMeshBodyKinematic* const kinBody = (ndMeshBodyKinematic*)*body;
						const ndVector scale(kinBody->m_shapeInstance.m_scale);
						ndMatrix scaleMatrix(ndGetIdentityMatrix());
						scaleMatrix[0][0] = scale[0];
						scaleMatrix[1][1] = scale[1];
						scaleMatrix[2][2] = scale[2];
						const ndMatrix pivotMatrix(scaleMatrix * kinBody->m_shapeInstance.m_localMatrix * debugMesh.m_parent->m_globalMatrix);

						if (m_manager->m_showSelectedNode)
						{
							const ndRenderPrimitive* const primitive = *debugMesh.m_zBufferShape;
							if (primitive && debugMesh.m_wireFrameShape->m_segments.GetCount())
							{
								primitive->Render(m_owner, pivotMatrix, m_debugDisplaySetZbuffer);
								ndRenderPrimitiveSegment& segment = debugMesh.m_wireFrameShape->m_segments.GetFirst()->GetInfo();
								ndRenderPrimitiveMaterial* const material = &segment.m_material;
								material->m_diffuse = color;
								debugMesh.m_wireFrameShape->Render(m_owner, pivotMatrix, m_debugDisplayWireFrameMesh);
							}
						}
					}
				}
			};

			const ndSharedPtr<ndMeshCollidingPair>& pair = ptr->GetInfo();

			DisplayShape(pair->m_childNode->GetAsMesh(), m_collidingPairColor0);
			DisplayShape(pair->m_parentNode->GetAsMesh(), m_collidingPairColor0);
			break;
		}
		i++;
	}
}

void ndDebugDisplayRenderPass::RenderOptions()
{
	auto RenderOptions = [this](const ndMesh* const node)
	{
		const ndString& selected = node->GetName();
		ndRenderSceneNode* const sceneNode = m_manager->m_entity->FindByName(selected);
		if (sceneNode)
		{
			const ndMatrix pivotMatrix(sceneNode->m_globalMatrix);
			if (m_manager->m_showPivot)
			{
				if (m_manager->m_parentSpaceTransform && sceneNode->GetParent())
				{
					//ndMatrix matrix (sceneNode->GetParent()->m_matrix);
					//matrix.m_posit = matrix.TransformVector(sceneNode->m_matrix.m_posit);
					ndMatrix matrix(sceneNode->GetParent()->m_globalMatrix);
					matrix.m_posit = pivotMatrix.m_posit;
					DrawFrame(matrix);
				}
				else
				{
					DrawFrame(pivotMatrix);
				}
			}

			if (m_manager->m_geometryPivot)
			{
				const ndMatrix geometryPivot(sceneNode->GetPrimitiveMatrix() * pivotMatrix);
				DrawFrame(geometryPivot);
			}

			if (m_manager->m_showShapePivot)
			{
				const ndSharedPtr<ndMeshBody>& meshRigidBody = m_manager->m_currentSelection->GetRigidBody();
				if (meshRigidBody)
				{
					ndMeshBodyKinematic* const kinematic = (ndMeshBodyKinematic*)(*meshRigidBody);
					const ndMatrix shapeMatrix(kinematic->m_shapeInstance.m_localMatrix * pivotMatrix);
					DrawFrame(shapeMatrix);
				}
			}

			if (m_manager->m_showCenterOfMass)
			{
				const ndSharedPtr<ndMeshBody>& meshRigidBody = m_manager->m_currentSelection->GetRigidBody();
				if (meshRigidBody)
				{
					ndMatrix comMatrix(pivotMatrix);
					ndVector localcom(meshRigidBody->m_localCentreOfMass);
					localcom.m_w = 1.0f;
					comMatrix.m_posit = pivotMatrix.TransformVector(localcom);
					DrawFrame(comMatrix);
				}
			}
			if (m_manager->m_showJoints)
			{
				const ndSharedPtr<ndMeshJoint>& meshJoint = m_manager->m_currentSelection->GetJoint();
				if (meshJoint)
				{
					ndBodyDynamic body0;
					ndBodyDynamic body1;
					body0.SetMatrix(pivotMatrix);
					body1.SetMatrix(sceneNode->GetParent()->m_globalMatrix);
					body0.SetMassMatrix(ndVector(1.0f));
					body1.SetMassMatrix(ndVector(1.0f));
					ndSharedPtr<ndJointBilateralConstraint> joint(meshJoint->CreateObject(&body0, &body1));

					class DebugJoint : public ndConstraintDebugCallback
					{
						public:
						DebugJoint(ndDebugDisplayRenderPass* const self)
							:ndConstraintDebugCallback()
							,m_self(self)
						{
							SetScale(m_self->m_manager->m_gizmoScale);
						}

						void DrawPoint(const ndVector&, const ndVector&, ndFloat32)
						{
							ndAssert(0);
						}

						virtual void DrawLine(const ndVector& p0, const ndVector& p1, const ndVector& color, ndFloat32)
						{
							m_self->DrawLine(p0, p1, color);
						}

						ndDebugDisplayRenderPass* m_self;
					};

					DebugJoint debugCallback(this);
					joint->DebugJoint(debugCallback);
				}
			}
		}
	};
	RenderOptions(*m_manager->m_currentSelection);

	ndSharedPtr<ndMeshTransformModifier> modifier(m_manager->m_currentSelection->GetModifier());
	if (modifier)
	{
		if (modifier->m_target)
		{
			RenderOptions(*modifier->m_target);
		}

		ndFixSizeArray<const ndMesh*, 256> subNodes(modifier->GetAffectedNodes());
		for (ndInt32 i = 0; i < subNodes.GetCount(); ++i)
		{
			RenderOptions(subNodes[i]);
		}
	}

	if (m_manager->m_subSelection == ndAssetEditor::m_alignToTarget)
	{
		if (m_manager->m_currentSubSelection)
		{
			RenderOptions(*m_manager->m_currentSubSelection);
		}
	}
}

void ndDebugDisplayRenderPass::RenderHiddenSurface()
{
	for (ndList<ndDebugMesh>::ndNode* ptr = m_debugMesh.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		const ndDebugMesh& debugMesh = ptr->GetInfo();
		if (*debugMesh.m_zBufferMesh)
		{
			const ndMatrix matrix(debugMesh.m_parent->m_primitiveMatrix * debugMesh.m_parent->m_globalMatrix);
			debugMesh.m_zBufferMesh->Render(m_owner, matrix, m_debugDisplaySetZbuffer);
		}
	}
}

void ndDebugDisplayRenderPass::RenderWireFrame()
{
	for (ndList<ndDebugMesh>::ndNode* ptr = m_debugMesh.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		const ndDebugMesh& debugMesh = ptr->GetInfo();
		const ndRenderPrimitive* const primitive = *debugMesh.m_wireFrameMesh;
		if (primitive && primitive->m_segments.GetCount())
		{
			const ndMatrix matrix(debugMesh.m_parent->m_primitiveMatrix * debugMesh.m_parent->m_globalMatrix);
			ndRenderPrimitiveSegment& segment = primitive->m_segments.GetFirst()->GetInfo();
			ndRenderPrimitiveMaterial* const material = &segment.m_material;

			material->m_diffuse = m_meshColor;
			primitive->Render(m_owner, matrix, m_debugDisplayWireFrameMesh);
		}
	}
}

void ndDebugDisplayRenderPass::RenderSkeleton()
{
	auto ForEachMesh = [this](ndMesh* const node)
	{
		ndMesh::ndNodeType type = node->GetNodeType();
		if (type == ndMesh::m_bone)
		{
			DrawBone(node);
		}
	};
	m_manager->m_mesh->NodeIterator(ForEachMesh);
}

void ndDebugDisplayRenderPass::RenderMeshSelection()
{
	if (m_manager->m_currentSelection)
	{
		if (m_manager->m_currentSelection->GetAsMesh())
		{
			RenderSelectedNode();
		}
		m_owner->ClearZBuffer();

		if (m_debugLines.GetCount())
		{
			const ndMatrix matrix(ndGetIdentityMatrix());
			m_renderLinesPrimitive->Render(m_owner, matrix, m_debugLineArray);
		}

		// do not call base class
		if (m_manager->m_renderMode == ndAssetEditor::m_wireframe)
		{
			RenderWireFrame();
		}
		else if (m_manager->m_renderMode == ndAssetEditor::m_hiddenSurface)
		{
			RenderHiddenSurface();
			RenderWireFrame();
		}

		if (m_manager->m_currentSelection->GetAsMesh())
		{
			if (m_manager->m_showCollisionShape)
			{
				RenderCollisionShape();
			}

			m_owner->ClearZBuffer();
			RenderOptions();
		}
		else if (m_manager->m_currentSelection->GetAsCloseLoopConstraints())
		{
			//m_owner->ClearZBuffer();
			RenderCloseLoopJoints();
		}
		else if (m_manager->m_subSelection == ndAssetEditor::m_transformModifier)
		{
			ndAssert(0);
			//m_owner->ClearZBuffer();
			RenderCollisionPair();
		}
	}
}
