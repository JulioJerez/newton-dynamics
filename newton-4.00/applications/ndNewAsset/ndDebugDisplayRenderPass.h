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
#ifndef __ND_DEBUG_DISPLAY_RENDER_PASS_H__
#define __ND_DEBUG_DISPLAY_RENDER_PASS_H__

class ndAssetEditor;

class ndDebugDisplayRenderPass : public ndRenderPassDebug
{
	public:
	class ndDebugMesh
	{
		public:
		ndDebugMesh()
			:m_zBuffer()
			,m_flatShaded()
			,m_wireFrameShareEdge()
		{
		}

		ndWeakPtr<ndRenderSceneNode> m_parent;
		ndSharedPtr<ndRenderPrimitive> m_zBuffer;
		ndSharedPtr<ndRenderPrimitive> m_flatShaded;
		ndSharedPtr<ndRenderPrimitive> m_wireFrameShareEdge;
	};

	ndDebugDisplayRenderPass(ndAssetEditor* const owner);
	~ndDebugDisplayRenderPass();

	private:
	void RenderWireFrame();
	void RenderHiddenSurface();
	void RenderCollisionShape();

	virtual void ResetScene() override;
	virtual void RenderScene() override;
	ndDebugMesh* CreateRenderPrimitive(const ndShapeInstance& shapeInstance) const;

	//ndVector m_awakeColor;
	//ndVector m_sleepColor;
	ndWeakPtr<ndAssetEditor> m_manager;
	ndList<ndDebugMesh> m_debugMesh;
};

#endif