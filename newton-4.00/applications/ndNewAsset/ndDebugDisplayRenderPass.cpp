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
	,m_selectedColor(ndFloat32(0.42f), ndFloat32(0.73f), ndFloat32(0.98f), ndFloat32(1.0f))
	,m_manager(owner)
{
}

ndDebugDisplayRenderPass::~ndDebugDisplayRenderPass()
{
}

ndDebugDisplayRenderPass::ndDebugMesh* ndDebugDisplayRenderPass::CreateRenderPrimitive(const ndShapeInstance& shapeInstance) const
{
	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(shapeInstance));
	shape->SetLocalMatrix(ndGetIdentityMatrix());

	ndRender* const render = m_owner;
	ndDebugMesh* const debugMesh = new ndDebugMesh;

	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;

	descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugFlatShaded;
	debugMesh->m_flatShaded = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));

	descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugWireFrame;
	debugMesh->m_wireFrameShareEdge = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));

	descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugHiddenLines;
	debugMesh->m_zBuffer = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));

	return debugMesh;
}

void ndDebugDisplayRenderPass::RenderCollisionShape()
{
	ndAssert(0);
	//ndPhysicsWorld* const world = m_manager->GetWorld();
	//const ndBodyListView& bodyList = world->GetBodyList();
	//
	//for (ndBodyListView::ndNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	//{
	//	ndBodyKinematic* const body = bodyNode->GetInfo()->GetAsBodyKinematic();
	//	const ndShapeInstance& shapeInstance = body->GetCollisionShape();
	//
	//	ndShape* const key = (ndShape*)shapeInstance.GetShape();
	//	if (key->GetAsShapeNull())
	//	{
	//		continue;
	//	}
	//
	//	ndTree<ndSharedPtr<ndDebugMesh>, ndShape*>::ndNode* node = m_meshCache.Find(key);
	//	if (!node)
	//	{
	//		ndSharedPtr<ndDebugMesh> debugMesh(CreateRenderPrimitive(shapeInstance));
	//		node = m_meshCache.Insert(debugMesh, key);
	//	}
	//
	//	const ndMatrix matrix(shapeInstance.GetLocalMatrix() * body->GetMatrix());
	//	ndSharedPtr<ndDebugMesh>& debugMesh = node->GetInfo();
	//	const ndVector color((body->GetSleepState() == 1) ? m_sleepColor : m_awakeColor);
	//
	//	switch (m_showCollisionMeshMode)
	//	{
	//		case 1:
	//		{
	//			// render solid color collision mesh
	//			ndRenderPrimitive* const mesh = *debugMesh->m_flatShaded;
	//			ndRenderPrimitiveSegment& segment = mesh->m_segments.GetFirst()->GetInfo();
	//			ndRenderPrimitiveMaterial* const material = &segment.m_material;
	//			material->m_diffuse = color;
	//			debugMesh->m_flatShaded->Render(m_owner, matrix, m_debugDisplaySolidMesh);
	//			break;
	//		}
	//
	//		case 2:
	//		{
	//			// render solid color collsion mesh
	//			ndRenderPrimitive* const mesh = *debugMesh->m_wireFrameShareEdge;
	//			ndRenderPrimitiveSegment& segment = mesh->m_segments.GetFirst()->GetInfo();
	//			ndRenderPrimitiveMaterial* const material = &segment.m_material;
	//			material->m_diffuse = color;
	//			debugMesh->m_wireFrameShareEdge->Render(m_owner, matrix, m_debugDisplayWireFrameMesh);
	//			break;
	//		}
	//
	//		case 3:
	//		default:
	//		{
	//			debugMesh->m_zBuffer->Render(m_owner, matrix, m_debugDisplaySetZbuffer);
	//			break;
	//		}
	//	}
	//}
	//
	//if (m_showCollisionMeshMode == 3)
	//{
	//	for (ndBodyListView::ndNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	//	{
	//		ndBodyKinematic* const body = bodyNode->GetInfo()->GetAsBodyKinematic();
	//		const ndShapeInstance& shapeInstance = body->GetCollisionShape();
	//		ndShape* const key = (ndShape*)shapeInstance.GetShape();
	//		if (key->GetAsShapeNull() || key->GetAsShapeStaticProceduralMesh())
	//		{
	//			continue;
	//		}
	//		ndTree<ndSharedPtr<ndDebugMesh>, ndShape*>::ndNode* const node = m_meshCache.Find(key);
	//		ndAssert(node);
	//		const ndMatrix matrix(shapeInstance.GetLocalMatrix() * body->GetMatrix());
	//		ndSharedPtr<ndDebugMesh>& debugMesh = node->GetInfo();
	//		const ndVector color((body->GetSleepState() == 1) ? m_sleepColor : m_awakeColor);
	//
	//		ndRenderPrimitive* const mesh = *debugMesh->m_wireFrameShareEdge;
	//		ndRenderPrimitiveSegment& segment = mesh->m_segments.GetFirst()->GetInfo();
	//		ndRenderPrimitiveMaterial* const material = &segment.m_material;
	//		material->m_diffuse = color;
	//		debugMesh->m_wireFrameShareEdge->Render(m_owner, matrix, m_debugDisplayWireFrameMesh);
	//	}
	//}
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
		ndSharedPtr<ndMeshEffect>& geometry = node->GetMesh();
		if (*geometry)
		{
			ndDebugMesh& entry = m_debugMesh.Append()->GetInfo();
			entry.m_parent = ndWeakPtr<ndRenderSceneNode>(m_manager->m_entity->FindByName(node->GetName()));

			ndRenderPrimitive::ndDescriptor descriptor(m_owner);
			descriptor.m_meshNode = geometry;

			descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugHiddenLines;
			entry.m_zBuffer = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));

			descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugWireFrame;
			entry.m_wireFrameShareEdge = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));

			descriptor.m_meshBuildMode = ndRenderPrimitive::m_debugFlatShaded;
			entry.m_flatShaded = ndSharedPtr<ndRenderPrimitive>(new ndRenderPrimitive(descriptor));
		}
	};
	m_manager->m_mesh->NodeIterator(BuildDebugMesh);
}

void ndDebugDisplayRenderPass::RenderWireFrame()
{
	for (ndList<ndDebugMesh>::ndNode* ptr = m_debugMesh.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		const ndDebugMesh& debugMesh = ptr->GetInfo();
		const ndMatrix matrix(debugMesh.m_parent->m_primitiveMatrix * debugMesh.m_parent->m_globalMatrix);
		const ndRenderPrimitive* const primitive = *debugMesh.m_wireFrameShareEdge;

		ndRenderPrimitiveSegment& segment = primitive->m_segments.GetFirst()->GetInfo();
		ndRenderPrimitiveMaterial* const material = &segment.m_material;

		material->m_diffuse = m_meshColor;
		primitive->Render(m_owner, matrix, m_debugDisplayWireFrameMesh);
	}
}

void ndDebugDisplayRenderPass::DrawLine(const ndVector& p0, const ndVector& p1, const ndVector& color)
{
	ndPoint line;

	line.m_point = p0;
	line.m_color = color;
	m_debugLines.PushBack(line);

	line.m_point = p1;
	line.m_color = color;
	m_debugLines.PushBack(line);
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
	const ndString& seletecName = m_manager->m_currentSelection->GetName();
	for (ndList<ndDebugMesh>::ndNode* ptr = m_debugMesh.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		const ndDebugMesh& debugMesh = ptr->GetInfo();
		if (debugMesh.m_parent->m_name == seletecName)
		{
			const ndMatrix pivotMatrix(debugMesh.m_parent->m_globalMatrix);
			if (m_manager->m_showSelectedNode)
			{
				const ndMatrix gemetryMatrix(debugMesh.m_parent->m_primitiveMatrix * pivotMatrix);
				const ndRenderPrimitive* const primitive = *debugMesh.m_wireFrameShareEdge;

				ndRenderPrimitiveSegment& segment = primitive->m_segments.GetFirst()->GetInfo();
				ndRenderPrimitiveMaterial* const material = &segment.m_material;

				material->m_diffuse = m_selectedColor;
				primitive->Render(m_owner, gemetryMatrix, m_debugDisplayWireFrameMesh);
			}

			if (m_manager->m_showPivot)
			{
				DrawFrame(pivotMatrix);
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
					body1.SetMatrix(debugMesh.m_parent->GetParent()->m_globalMatrix);
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

						//void DrawPoint(const ndVector& point, const ndVector& color, ndFloat32 thickness = ndFloat32(8.0f))
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
	}
}

void ndDebugDisplayRenderPass::RenderHiddenSurface()
{
	for (ndList<ndDebugMesh>::ndNode* ptr = m_debugMesh.GetFirst(); ptr; ptr = ptr->GetNext())
	{
		const ndDebugMesh& debugMesh = ptr->GetInfo();
		const ndMatrix matrix(debugMesh.m_parent->m_primitiveMatrix * debugMesh.m_parent->m_globalMatrix);
		debugMesh.m_zBuffer->Render(m_owner, matrix, m_debugDisplaySetZbuffer);
	}
}

void ndDebugDisplayRenderPass::RenderScene()
{
	if (m_debugLines.GetCount())
	{
		const ndMatrix matrix(ndGetIdentityMatrix());
		m_renderLinesPrimitive->Render(m_owner, matrix, m_debugLineArray);
	}

	m_debugLines.SetCount(0);
	m_debugPoints.SetCount(0);

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

	if (m_manager->m_currentSelection)
	{
		RenderSelectedNode();
	}
}
