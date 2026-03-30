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

class ndPhysicsWorld;
class ndDebugDisplayRenderPass;

class ndAssetEditor : public ndClassAlloc
{
	public:
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

	enum ndMenuSelection
	{
		m_new,
		//m_load,
		//m_save,
		//m_saveModel,
		m_none,
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
	
	//ndPhysicsWorld* GetWorld() const;
	ndSharedPtr<ndRender>& GetRenderer();
	//ndDebugDisplayRenderPass* GetDebugRenderPass();
	//void AddEntity(const ndSharedPtr<ndRenderSceneNode>& entity);
	//void RemoveEntity(const ndSharedPtr<ndRenderSceneNode>& entity);
	//void ImportPLYfile (const char* const name);

	bool GetMouseSpeed(ndFloat32& posX, ndFloat32& posY) const;
	bool GetMousePosition (ndFloat32& posX, ndFloat32& posY) const;
	void SetCameraMatrix (const ndQuaternion& rotation, const ndVector& position);
	
	bool AnyKeyDown() const;
	bool IsShiftKeyDown () const;
	bool JoystickDetected() const;
	bool IsControlKeyDown () const;
	bool GetKeyState(ndInt32 key) const;
	void GetJoystickAxis (ndFixSizeArray<ndFloat32, 8>& axisValues);
	void GetJoystickButtons (ndFixSizeArray<char, 32>& axisbuttons);

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
	void Cleanup();
	void RenderScene();
	ndInt32 ParticleCount() const;
	void SetParticleUpdateMode() const;
	void UpdatePhysics(ndFloat32 timestep);
	
	void ShowMainMenuBar();
	void ToggleProfiler();
	void ApplyOptions();
	void ApplyMenuOptions();
	void OnSubStepPostUpdate(ndFloat32 timestep);

	void EndDockSpace();
	void BeginDockSpace();

	void ShowOutlierPanel();
	void ShowOutlierToolBar();
	void ShowOutlierExplorer(const ndSharedPtr<ndMesh>& root);

	void ShowPropertiesPanel();
	void ShowPropertiesMeshInfo();
	void ShowPropertiesJointInfo();
	void ShowPropertiesCollisionInfo();
	void ShowPropertiesRigidBodyInfo();
		
	ndSharedPtr<ndMesh> m_model;
	ndSharedPtr<ndRender> m_renderer;
	ndSharedPtr<ndRenderPass> m_menuRenderPass;
	ndSharedPtr<ndRenderPass> m_colorRenderPass;
	ndSharedPtr<ndMesh> m_currentSelection;

	//ndSharedPtr<ndRenderPass> m_debugDisplayRenderPass;
	//ndSharedPtr<ndRenderTexture> m_environmentTexture;
	ndSharedPtr<ndRenderSceneNode> m_defaultCamera;

	ndSharedPtr<OnPostUpdate> m_onPostUpdate;
	ndString m_currentPath;


	ndInt32 m_currentPlugin;
	ndInt32 m_solverPasses;
	ndInt32 m_solverSubSteps;
	ndInt32 m_workerThreads;
	ndInt32 m_debugDisplayMode;
	ndInt32 m_showCollisionMeshMode;
	
	bool m_runScene;
	ndWorld::ndSolverModes m_solverMode;

	class WindowFrame
	{
		public:
		ImVec2 m_posit;
		ImVec2 m_size;
	};
	ndFixSizeArray<WindowFrame, 16> m_windowSizes;
	
	friend class ndPhysicsWorld;
	friend class ndDebugDisplayRenderPass;
};

#endif