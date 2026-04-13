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
			:m_zBufferMesh()
			,m_zBufferShape()
			,m_wireFrameMesh()
			,m_flatShadedMesh()
			,m_wireFrameShape()
		{
		}

		ndWeakPtr<ndRenderSceneNode> m_parent;
		ndSharedPtr<ndRenderPrimitive> m_zBufferMesh;
		ndSharedPtr<ndRenderPrimitive> m_zBufferShape;
		ndSharedPtr<ndRenderPrimitive> m_wireFrameMesh;
		ndSharedPtr<ndRenderPrimitive> m_flatShadedMesh;
		ndSharedPtr<ndRenderPrimitive> m_wireFrameShape;
	};

	ndDebugDisplayRenderPass(ndAssetEditor* const owner);
	~ndDebugDisplayRenderPass();

	void RebuildDebugCollision();
	virtual void ResetScene() override;

	private:
	void RenderOptions();
	void RenderWireFrame();
	void RenderSelectedNode();
	void RenderHiddenSurface();
	void RenderCollisionShape();
	virtual void RenderScene() override;
	
	//ndDebugMesh* CreateRenderPrimitive(const ndShapeInstance& shapeInstance) const;

	void DrawFrame(const ndMatrix& matrix);
	void DrawLine(const ndVector& p0, const ndVector& p1, const ndVector& color);

	ndVector m_meshColor;
	ndVector m_shapeColor;
	ndVector m_selectedColor;
	ndWeakPtr<ndAssetEditor> m_manager;
	ndList<ndDebugMesh> m_debugMesh;
};

#endif