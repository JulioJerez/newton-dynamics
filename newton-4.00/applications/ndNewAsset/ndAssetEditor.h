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
#ifndef __ASSET_EDITOR_H__
#define __ASSET_EDITOR_H__

#include "ndUndoRedo.h"

class ndPhysicsWorld;
class ndDebugDisplayRenderPass;

class ndAssetEditor : public ndClassAlloc
{
	public:
	enum ndRenderModes
	{
		m_shaded,
		m_wireframe,
		m_hiddenSurface,
		m_size = 0xffffffff
	};

	class ndRenderCallback : public ndRender::ndUserCallback
	{
		public:
		ndRenderCallback(ndAssetEditor* const owner)
			:ndRender::ndUserCallback()
			,m_owner(owner)
		{
		}
	
		virtual void KeyCallback(ndInt32 key, ndInt32 action) override
		{
			m_owner->KeyCallback(key, action);
		}
	
		virtual void CharCallback(ndUnsigned32 ch)
		{
			m_owner->CharCallback(ch);
		}
	
		virtual void CursorposCallback(ndReal x, ndReal y)
		{
			m_owner->CursorposCallback(x, y);
		}
	
		virtual void MouseScrollCallback(ndReal x, ndReal y)
		{
			m_owner->MouseScrollCallback(x, y);
		}
	
		virtual void MouseButtonCallback(ndInt32 button, ndInt32 action)
		{
			m_owner->MouseButtonCallback(button, action);
		}
	
		ndWeakPtr<ndAssetEditor> m_owner;
	};

	class ndAssetTool: public ndClassAlloc
	{
		public:
		ndAssetTool(ndAssetEditor* const owner)
			:ndClassAlloc()
			,m_owner(owner)
		{
		}

		virtual ~ndAssetTool()
		{
		}

		virtual void Execute() = 0;

		ndWeakPtr<ndAssetEditor> m_owner;
	};

	ndAssetEditor ();
	~ndAssetEditor ();

	void Run();

	ndInt32 GetWidth() const;
	ndInt32 GetHeight() const;
	ndSharedPtr<ndRender>& GetRenderer();

	ndFloat32 GetMouseWheel() const;
	bool GetMousePosition (ndFloat32& posX, ndFloat32& posY) const;
	void SetCameraMatrix (const ndQuaternion& rotation, const ndVector& position);

	void Terminate();
	bool GetCaptured() const;
	void CharCallback(ndUnsigned32 ch);
	void CursorposCallback(ndReal x, ndReal y);
	bool GetMouseKeyState(ndInt32 button) const;
	void MouseScrollCallback(ndReal x, ndReal y);
	void KeyCallback(ndInt32 key, ndInt32 action);
	void MouseButtonCallback(ndInt32 button, ndInt32 action);

	void TestImGui();
	void RenderLayout();
	ndDebugDisplayRenderPass* GetDebugDisplay() const;

	bool GetActiveTool() const;
	void SetActiveTool(bool toolState);

	ndSharedPtr<ndMesh>& GetMesh();
	const ndSharedPtr<ndMesh>& GetMesh() const;
	
	const ndString& GetPath() const;
	void SetVisualScene(const ndRenderMeshLoader& loader);

	//private:
	void RenderScene();
	void UpdatePhysics(ndFloat32 timestep);
	void OnSubStepPostUpdate(ndFloat32 timestep);

	void ConfigureDockSpace();
	void ShowMainMenuBar();
	void ShowMainToolbar();
	void ShowOutlierPanel();
	void ShowOutlierExplorer(const ndSharedPtr<ndMesh>& root);
	void ShowOutlierExplorerCloseLoop(const ndSharedPtr<ndMesh>& closeLoop);
	void ShowOutlierExplorerCollidindPairs(const ndSharedPtr<ndMesh>& closeLoop);

	void ShowPropertiesPanel();
	void ShowPropertiesMeshInfo();
	void ShowPropertiesJointInfo();
	void ShowPropertiesCollisionInfo();
	void ShowPropertiesRigidBodyInfo();

	void ShowJointGlobalMatrix();
	void JointsEditFix6dof();
	void JointsEditHingeJoint();
	void JointsEditSliderJoint();
	void JointsEditPlaneJoint();
	void JointsEditRollerJoint();
	void JointsEditCylinderJoint();
	void JointsEditDoubleHingeJoint();
	void JointsEditWheelJoint();
	void JointsEditSpherical();

	void ShowPropertiesJointsLoopInfo();
	void ShowLoopJointLocalMatrix();
	void ShowLoopJointGlobalMatrix();
	void JointsLoopEditFix6dof();
	void JointsLoopEditHingeJoint();
	void JointsLoopEditSliderJoint();
	void JointsLoopEditPlaneJoint();
	void JointsLoopEditRollerJoint();
	void JointsLoopEditCylinderJoint();
	void JointsLoopEditDoubleHingeJoint();
	void JointsLoopEditWheelJoint();
	void JointsLoopEditSpherical();
	void JointsLoopEditGear();
	void JointsLoopEditDifferentialAxle();
	void JointsLoopEditSwivelPositionEffector();

	void ShowShapeTransform();
	void ShowCollisionBox();
	void ShowCollisionSphere();
	void ShowCollisionCapsule();
	void ShowCollisionCylinder();
	void ShowCollisionCompound();
	void ShowCollisionConvexHull();
	void ShowCollisionChamferCylinder();
		
	ndSharedPtr<ndMesh> m_mesh;
	ndSharedPtr<ndRender> m_renderer;
	ndSharedPtr<ndRenderSceneNode> m_entity;
	ndSharedPtr<ndRenderPass> m_menuRenderPass;
	ndSharedPtr<ndRenderPass> m_colorRenderPass;
	ndSharedPtr<ndRenderPass> m_shadowRenderPass;
	ndSharedPtr<ndRenderPass> m_environmentRenderPass;
	ndSharedPtr<ndRenderTexture> m_environmentTexture;
	ndSharedPtr<ndRenderPass> m_debugDisplayRenderPass;

	ndSharedPtr<ndRenderSceneNode> m_defaultCamera;

	ndSharedPtr<ndMesh> m_newMesh;
	ndSharedPtr<ndRenderSceneNode> m_newSceneMesh;

	ndSharedPtr<ndAssetTool> m_currentTool;
	ndSharedPtr<ndMesh> m_currentSelection;
	ndSharedPtr<ndMeshLoopJoint> m_currentLoopJointSelection;
	ndSharedPtr<ndMeshCollidingPair> m_currentCollingPairSelection;

	ndString m_currentPath;
	ndUndoRedo m_undoRedo;

	bool m_runScene;
	bool m_showPivot;
	bool m_showJoints;
	bool m_showCenterOfMass;
	bool m_showSelectedNode;
	bool m_showCollisionShape;
	bool m_showPreTransform;

	bool m_toolActive;
	//bool m_addCollidingBody;
	//bool m_removeCollidingBody;
	//ndInt32 m_addCollingPairSelection;
	//ndInt32 m_addCollingPairCandidateSelection;
	
	ndInt32 m_renderMode;
	ndReal m_gizmoScale;
	
	friend class ndUndoRedo;
	friend class ndUndoRedoCommand;
	friend class ndEditorCameraFlyby;
	friend class ndDebugDisplayRenderPass;
};

#endif