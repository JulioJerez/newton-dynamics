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
	
		ndAssetEditor* m_owner;
	};

	class ndKeyTrigger
	{
		public: 
		ndKeyTrigger()
			:m_memory(false)
		{
		}

		bool Update(bool value)
		{
			bool ret = bool (!m_memory & value);
			m_memory = value;
			return ret;
		}

		bool m_memory;
	};

	class ndDemoUIpanel : public ndClassAlloc
	{
		public:
		ndDemoUIpanel()
			:ndClassAlloc()
		{
		}

		virtual ~ndDemoUIpanel() 
		{
		}

		virtual void Update(ndAssetEditor* const scene) = 0;
	};

	class ndDemoHelper: public ndClassAlloc
	{
		public:
		ndDemoHelper()
			:ndClassAlloc()
			,m_currentTime(ndGetTimeInMicroseconds())
		{
			ResetTime();
		}

		void ResetTime()
		{
			m_currentTime = ndGetTimeInMicroseconds();
		}

		bool ExpirationTime() const
		{
			// stops diplay the legend afte 5 secunds
			ndUnsigned64 timestep = ndGetTimeInMicroseconds() - m_currentTime;
			return timestep > 5 * 1024 * 1024;
		}

		virtual ~ndDemoHelper() {}
		virtual void PresentHelp(ndAssetEditor* const scene) = 0;

		ndUnsigned64 m_currentTime;
	};

	class OnPostUpdate : public ndClassAlloc
	{
		public:
		OnPostUpdate()
			:ndClassAlloc()
		{
		}

		virtual ~OnPostUpdate()
		{
		}

		virtual void OnDebug(ndAssetEditor* const, bool) {}
		virtual void Update(ndAssetEditor* const scene, ndFloat32 timestep) = 0;
	};

	class ButtonKey
	{
		public:
		ButtonKey (bool initialState);
		ndInt32 UpdateTrigger (bool triggerValue);
		ndInt32 UpdatePushButton (bool triggerValue);
		ndInt32 GetPushButtonState() const { return m_state ? 1 : 0;}

		private:
		bool m_state;
		bool m_memory0;
		bool m_memory1;
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
	
	bool AnyKeyDown() const;
	bool IsShiftKeyDown () const;
	bool IsControlKeyDown () const;
	bool GetKeyState(ndInt32 key) const;

	void Terminate();

	void CharCallback(ndUnsigned32 ch);
	void CursorposCallback(ndReal x, ndReal y);
	void MouseScrollCallback(ndReal x, ndReal y);
	void KeyCallback(ndInt32 key, ndInt32 action);
	void MouseButtonCallback(ndInt32 button, ndInt32 action);

	bool GetCaptured () const;
	bool GetMouseKeyState (ndInt32 button ) const;
	ndInt32 Print (const ndVector& color, const char *fmt, ... ) const;

	void TestImGui();
	void RenderLayout();
	void SetDemoHelp(ndSharedPtr<ndDemoHelper>& helper);
	void SetDemoUIpanel(ndSharedPtr<ndDemoUIpanel>& panel);

	void RegisterPostUpdate(const ndSharedPtr<OnPostUpdate>& postUpdate);

	private:
	void RenderScene();
	void UpdatePhysics(ndFloat32 timestep);
	void SetVisualScene(const ndRenderMeshLoader& loader);
	
	void OnSubStepPostUpdate(ndFloat32 timestep);

	void ConfigureDockSpace();
	void ShowMainMenuBar();
	void ShowMainToolbar();
	void ShowOutlierPanel();
	void ShowOutlierExplorer(const ndSharedPtr<ndMesh>& root);

	void ShowPropertiesPanel();
	void ShowPropertiesMeshInfo();
	void ShowPropertiesJointInfo();
	void ShowPropertiesCollisionInfo();
	void ShowPropertiesRigidBodyInfo();
		
	ndSharedPtr<ndMesh> m_mesh;
	ndSharedPtr<ndModel> m_model;
	ndSharedPtr<ndRender> m_renderer;
	ndSharedPtr<ndRenderSceneNode> m_entity;
	ndSharedPtr<ndRenderPass> m_menuRenderPass;
	ndSharedPtr<ndRenderPass> m_colorRenderPass;
	ndSharedPtr<ndRenderPass> m_shadowRenderPass;
	ndSharedPtr<ndRenderPass> m_environmentRenderPass;
	ndSharedPtr<ndRenderTexture> m_environmentTexture;
	ndSharedPtr<ndRenderPass> m_debugDisplayRenderPass;

	ndSharedPtr<ndMesh> m_newMesh;
	ndSharedPtr<ndRenderSceneNode> m_newSceneMesh;

	ndSharedPtr<ndMesh> m_currentSelection;
	ndSharedPtr<ndRenderSceneNode> m_defaultCamera;

	ndSharedPtr<OnPostUpdate> m_onPostUpdate;
	ndString m_currentPath;
	ndUndoRedo m_undoRedo;

	bool m_runScene;
	bool m_showPivot;
	bool m_showJoints;
	bool m_showCenterOfMass;
	bool m_showSelectedNode;
	bool m_showCollisionShape;
	bool m_showParentRelativeTransform;
	ndInt32 m_renderMode;
	ndReal m_gizmoScale;
	
	friend class ndUndoRedo;
	friend class ndUndoRedoCommand;
	friend class ndEditorCameraFlyby;
	friend class ndDebugDisplayRenderPass;
};

#endif