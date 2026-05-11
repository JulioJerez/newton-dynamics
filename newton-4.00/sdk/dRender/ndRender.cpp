/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndRenderStdafx.h"
#include "ndRender.h"
#include "ndRenderPass.h"
#include "ndRenderTexture.h"
#include "ndRenderContext.h"
#include "ndRenderSceneNode.h"
#include "ndRenderSceneCamera.h"
#include "ndRenderTextureCache.h"
#include "ndRenderTransformModifier.h"

ndRender::ndRender(ndSharedPtr<ndUserCallback>& owner, ndInt32 width, ndInt32 height, const char* const title)
	:ndClassAlloc()
	,m_owner(owner)
	,m_context(nullptr)
	,m_camera(nullptr)
	,m_textureCache(nullptr)
	,m_scene()
	,m_deadNodes()
	,m_renderPasses()
	,m_sunLightDir(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(1.0f))
	,m_sunLightAmbient(ndFloat32(0.2f), ndFloat32(0.2f), ndFloat32(0.2f), ndFloat32(0.0f))
	,m_sunLightIntesity(ndFloat32(0.7f), ndFloat32(0.7f), ndFloat32(0.7f), ndFloat32(0.0f))
	,m_backgroundColor(ndFloat32(0.45f), ndFloat32(0.55f), ndFloat32(0.60f), ndFloat32(1.0f))
	,m_addRemoveLock()
	,m_cachedDebugPass(nullptr)
	,m_cachedShadowPass(nullptr)
	,m_cachedEnvironmentPass(nullptr)
{
	m_context = ndSharedPtr<ndRenderContext>(new ndRenderContext(this, width, height, title));
	m_camera = ndSharedPtr<ndRenderSceneNode>(new ndRenderSceneCamera(this));
	m_textureCache = ndSharedPtr<ndRenderTextureCache>(new ndRenderTextureCache(this));
}

ndRender::~ndRender()
{
}

void ndRender::SetTitle(const char* const title)
{
	m_context->SetTitle(title);
}

void ndRender::InitImGui(const char* const fontPathName)
{
	m_context->InitImGui(fontPathName);
}

void ndRender::SetSunLight(const ndVector& direction, const ndVector& intensity)
{
	m_sunLightIntesity = intensity & ndVector::m_triplexMask;
	m_sunLightDir = (direction & ndVector::m_triplexMask).Normalize();
}

ndSharedPtr<ndRender::ndUserCallback>& ndRender::GetOwner()
{
	return m_owner;
}

void ndRender::SetCamera(const ndSharedPtr<ndRenderSceneNode>& camera)
{
	m_camera = camera;
	ndAssert(camera->FindCameraNode());
	ndInt32 display_w = m_context->GetWidth();
	ndInt32 display_h = m_context->GetHeight();
	m_camera->FindCameraNode()->SetViewMatrix(display_w, display_h);
}

ndSharedPtr<ndRenderSceneNode>& ndRender::GetCamera()
{
	return m_camera;
}

const ndSharedPtr<ndRenderSceneNode>& ndRender::GetCamera() const
{
	return m_camera;
}

ndSharedPtr<ndRenderContext>& ndRender::GetContext()
{
	return m_context;
}

const ndSharedPtr<ndRenderContext>& ndRender::GetContext() const
{
	return m_context;
}

ndSharedPtr<ndRenderTextureCache>& ndRender::GetTextureCache()
{
	return m_textureCache;
}

ndList<ndSharedPtr<ndRenderSceneNode>>& ndRender::GetScene()
{
	return  m_scene;
}

ndInt32 ndRender::GetWidth() const
{
	return m_context->GetWidth();
}

void ndRender::SetViewport(ndInt32 x0, ndInt32 y0, ndInt32 x1, ndInt32 y1) const
{
	m_context->SetViewport(x0, y0, x1, y1);
}

ndInt32 ndRender::GetHeight() const
{
	return m_context->GetHeight();
}

void ndRender::MaximizeWindow() const
{
	m_context->MaximizeWindow();
}

bool ndRender::ShouldFinish() const
{
	return m_context->ShouldFinish();
}

void ndRender::Terminate()
{
	m_context->Terminate();
}

bool ndRender::PollEvents() const
{
	return m_context->PollEvents();
}

void ndRender::Present()
{
	m_context->Present();
}

void ndRender::AddRenderPass(const ndSharedPtr<ndRenderPass>& renderPass)
{
	ndAssert(renderPass->m_owner == this);
	m_renderPasses.Append(renderPass);
}

void ndRender::ClearZBuffer()
{
	m_context->ClearZBuffer();
}

void ndRender::ClearFrameBuffer(const ndVector& color)
{
	m_context->ClearFrameBuffer(color);
}

void ndRender::ResetScene()
{
	m_scene.RemoveAll();
	for (ndList<ndSharedPtr<ndRenderPass>>::ndNode* node = m_renderPasses.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndRenderPass>& pass = node->GetInfo();
		pass->ResetScene();
	}
}

void ndRender::AddSceneNode(const ndSharedPtr<ndRenderSceneNode>& node)
{
	ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* const handle = m_scene.Append(node);
	ndAssert(!node->m_owner);
	ndAssert(!node->m_sceneHandle);
	node->m_owner = this;
	node->m_sceneHandle = handle;
}

void ndRender::RemoveSceneNode(const ndSharedPtr<ndRenderSceneNode>& node)
{
	ndScopeSpinLock lock(m_addRemoveLock);
	m_deadNodes.Append(node);
}

void ndRender::InterpolateTransforms(ndFloat32 param)
{
	m_camera->InterpolateTransforms(param);
	for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = m_scene.GetFirst(); node; node = node->GetNext())
	{
		ndRenderSceneNode* const sceneNode = *node->GetInfo();
		sceneNode->InterpolateTransforms(param);
	}
}

void ndRender::UpdateGlobalMatrices() const
{
	ndList<ndRenderSceneNode*, ndContainersFreeListAlloc<ndRenderSceneNode*>> stackList;
	ndList<ndRenderSceneNode*, ndContainersFreeListAlloc<ndRenderSceneNode*>> transformNodes;

	auto AddNode = [&transformNodes](ndRenderSceneNode* const sceneNode)
	{
		if (*sceneNode->m_primitive)
		{
			if (sceneNode->GetAsInstance() || sceneNode->m_primitive->IsSkinnedMesh())
			{
				transformNodes.Append(sceneNode);
			}
		}
	};

	ndFixSizeArray<ndRenderTransformModifier*, 256> modifiers;
	for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* rootSceneNode = m_scene.GetFirst(); rootSceneNode; rootSceneNode = rootSceneNode->GetNext())
	{
		stackList.RemoveAll();
		ndRenderSceneNode* const rootNode = *rootSceneNode->GetInfo();

		rootNode->m_globalMatrix = rootNode->m_matrix;
		for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* childSceneNode = rootNode->m_children.GetFirst(); childSceneNode; childSceneNode = childSceneNode->GetNext())
		{
			ndRenderSceneNode* const child = *childSceneNode->GetInfo();
			stackList.Append(child);
		}

		// add root primitive to the transfrom list
		AddNode(rootNode);

		// add all children primitive the skinning list
		while (stackList.GetCount())
		{
			ndRenderSceneNode* const sceneNode = stackList.GetLast()->GetInfo();

			if (sceneNode->m_transformModifier)
			{
				modifiers.PushBack(*sceneNode->m_transformModifier);
			}

			AddNode(sceneNode);
			stackList.Remove(stackList.GetLast());
			sceneNode->m_globalMatrix = sceneNode->m_matrix * sceneNode->m_parent->m_globalMatrix;
			for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* childSceneNode = sceneNode->m_children.GetFirst(); childSceneNode; childSceneNode = childSceneNode->GetNext())
			{
				ndRenderSceneNode* const child = *childSceneNode->GetInfo();
				stackList.Append(child);
			}
		}

		for (ndList<ndRenderSceneNode*, ndContainersFreeListAlloc<ndRenderSceneNode*>>::ndNode* node = transformNodes.GetFirst();	node; node = node->GetNext())
		{
			ndRenderSceneNode* const meshNode = node->GetInfo();
			meshNode->ApplyPrimitiveTransforms();
		}
	}

	for (ndInt32 i = 0; i < modifiers.GetCount(); ++i)
	{
		modifiers[i]->Update();
	}
}

void ndRender::RemoveDefferedEntities()
{
	for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* deadNode = m_deadNodes.GetFirst(); deadNode; deadNode = deadNode->GetNext())
	{
		ndSharedPtr<ndRenderSceneNode> node(deadNode->GetInfo());
		ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* const handle = node->m_sceneHandle;
		if (handle)
		{
			node->m_owner = nullptr;
			node->m_sceneHandle = nullptr;
			m_scene.Remove(handle);
		}
		else
		{
			// see if this node is part of the scene
			const ndRenderSceneNode* parent = node->GetParent();
			for (; parent && !parent->m_sceneHandle; parent = parent->GetParent());
			if (parent)
			{
				node->GetParent()->RemoveChild(node);
			}
		}
	}
	m_deadNodes.RemoveAll();
}

void ndRender::BegingRender()
{
	m_context->BeginFrame();
}

void ndRender::EndRender()
{
	m_context->EndFrame();
}

void ndRender::Render()
{
	m_context->ClearFrameBuffer(m_backgroundColor);
	ndScopeSpinLock lock(m_addRemoveLock);

	if (m_deadNodes.GetCount())
	{
		RemoveDefferedEntities();
	}

	ImGuiIO& io = ImGui::GetIO();
	ndInt32 fb_width = (ndInt32)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
	ndInt32 fb_height = (ndInt32)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
	if (!(fb_width == 0 || fb_height == 0))
	{
		// calculate all grobal matrices
		UpdateGlobalMatrices();

		ndInt32 display_w = m_context->GetWidth();
		ndInt32 display_h = m_context->GetHeight();
		ndRenderSceneCamera* const camera = m_camera->FindCameraNode();
		camera->SetViewMatrix(display_w, display_h);

		for (ndList<ndSharedPtr<ndRenderPass>>::ndNode* node = m_renderPasses.GetFirst(); node; node = node->GetNext())
		{
			const ndSharedPtr<ndRenderPass>& pass = node->GetInfo();
			if (pass->m_active)
			{
				pass->RenderScene();
			}
		}

		// render the camera mesh, usually an icon
		if (!m_camera->m_parent)
		{
			m_camera->Render(*m_camera->m_owner, ndGetIdentityMatrix(), m_directionalDiffusseNoShadow);
		}
	}
}
