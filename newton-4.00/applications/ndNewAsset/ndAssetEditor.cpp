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
#include "ndAssetEditor.h"
#include "ndFileBrowser.h"
//#include "ndPhysicsWorld.h"
//#include "ndPhysicsUtils.h"
//#include "ndTestDeepBrain.h"
//#include "ndDemoCameraNode.h"
#include "ndMenuRenderPass.h"
//#include "ndHighResolutionTimer.h"
//#include "ndDemoCameraNodeFlyby.h"
//#include "ndDebugDisplayRenderPass.h"

ndDemoEntityManager::ButtonKey::ButtonKey (bool state)
	:m_state(state)
	,m_memory0(false)
	,m_memory1(false)
{
}

ndInt32 ndDemoEntityManager::ButtonKey::UpdateTrigger (bool triggerValue)
{
	m_memory0 = m_memory1;
	m_memory1 = triggerValue;
	return (!m_memory0 && m_memory1) ? 1 : 0;
}

ndInt32 ndDemoEntityManager::ButtonKey::UpdatePushButton (bool triggerValue)
{
	if (UpdateTrigger (triggerValue)) 
	{
		m_state = ! m_state;
	}
	return m_state ? 1 : 0;
}

// ImGui - standalone example application for Glfw + OpenGL 2, using fixed pipeline
// If you are new to ImGui, see examples/README.txt and documentation at the top of imgui.cpp.
ndDemoEntityManager::ndDemoEntityManager()
	:ndClassAlloc()
	//,m_world(nullptr)
	//,m_renderer(nullptr)
	//,m_menuRenderPass(nullptr)
	//,m_colorRenderPass(nullptr)
	//,m_shadowRenderPass(nullptr)
	//,m_environmentRenderPass(nullptr)
	//,m_transparentRenderPass(nullptr)
	//,m_debugDisplayRenderPass(nullptr)
	//,m_environmentTexture(nullptr)
	//,m_demoHelper(nullptr)
	//,m_demoUIpanel(nullptr)
	//,m_currentScene(DEFAULT_SCENE)
	//,m_lastCurrentScene(DEFAULT_SCENE)
	,m_framesCount(0)
	,m_physicsFramesCount(0)
	,m_currentPlugin(0)
	,m_solverPasses(6)
	,m_solverSubSteps(2)
	,m_workerThreads(4)
	,m_debugDisplayMode(0)
	,m_showCollisionMeshMode(0)
	,m_fps(0.0f)
	,m_timestepAcc(0.0f)
	,m_currentListenerTimestep(0.0f)
	,m_showUI(true)
	,m_showAABB(false)
	,m_showStats(true)
	,m_helperLegend(false)
	,m_autoSleepMode(true)
	,m_showScene(false)
	//,m_showConcaveEdge(false)
	,m_hideVisualMeshes(false)
	,m_showNormalForces(false)
	,m_showCenterOfMass(false)
	,m_showBodyFrame(false)
	,m_showMeshSkeleton(false)
	,m_updateMenuOptions(true)
	,m_showContactPoints(false)
	,m_showJointDebugInfo(false)
	,m_showModelsDebugInfo(false)
	,m_suspendPhysicsUpdate(false)
	,m_synchronousPhysicsUpdate(false)
	,m_synchronousParticlesUpdate(false)
	,m_showStaticMeshCollidingFaces(false)
	,m_showRaycastHit(false)
	,m_profilerMode(false)
	,m_nextActiveCamera()
	,m_solverMode(ndWorld::ndSimdSoaSolver)
{
	// Setup window
	char title[256];

	ndSharedPtr<ndRender::ndUserCallback> callbacks(new ndRenderCallback(this));
	snprintf(title, sizeof(title), "Newton Asset Editor %d.%.2i", D_NEWTON_ENGINE_MAJOR_VERSION, D_NEWTON_ENGINE_MINOR_VERSION);
	m_renderer = ndSharedPtr<ndRender>(new ndRender(callbacks, 1280, 768, title));

	const ndString fontPathName(ndGetWorkingFileName("Cousine-Regular.ttf"));
	m_renderer->InitImGui(fontPathName.GetStr());

	//// load the environment texture
	//ndFixSizeArray<ndString, 6> environmentTexturePath;
	//environmentTexturePath.PushBack(ndGetWorkingFileName("Sorsele3/negx.png"));
	//environmentTexturePath.PushBack(ndGetWorkingFileName("Sorsele3/posx.png"));
	//environmentTexturePath.PushBack(ndGetWorkingFileName("Sorsele3/posy.png"));
	//environmentTexturePath.PushBack(ndGetWorkingFileName("Sorsele3/negy.png"));
	//environmentTexturePath.PushBack(ndGetWorkingFileName("Sorsele3/negz.png"));
	//environmentTexturePath.PushBack(ndGetWorkingFileName("Sorsele3/posz.png"));
	//m_environmentTexture = m_renderer->GetTextureCache()->GetCubeMap(environmentTexturePath);

	//// create render passes
	m_menuRenderPass = ndSharedPtr<ndRenderPass>(new ndMenuRenderPass(this));
	//m_debugDisplayRenderPass = ndSharedPtr<ndRenderPass>(new ndDebugDisplayRenderPass(this));
	//m_colorRenderPass = ndSharedPtr<ndRenderPass>(new ndRenderPassColor(*m_renderer));
	//m_shadowRenderPass = ndSharedPtr<ndRenderPass>(new ndRenderPassShadows(*m_renderer));
	//m_transparentRenderPass = ndSharedPtr<ndRenderPass>(new ndRenderPassTransparency(*m_renderer));
	//m_environmentRenderPass = ndSharedPtr<ndRenderPass>(new ndRenderPassEnvironment(*m_renderer, m_environmentTexture));

	//// add render passes in order of execution
	//m_renderer->AddRenderPass(m_shadowRenderPass);
	//m_renderer->AddRenderPass(m_colorRenderPass);
	//m_renderer->AddRenderPass(m_environmentRenderPass);
	//m_renderer->AddRenderPass(m_transparentRenderPass);
	//m_renderer->AddRenderPass(m_debugDisplayRenderPass);
	m_renderer->AddRenderPass(m_menuRenderPass);
	
	////add main directional light
	//m_renderer->SetSunLight(ndVector(-0.5f, 1.0f, -0.5f, 0.0f), ndVector(0.7f, 0.7f, 0.7f, 0.0f));

	// initialized the physics world for the new scene
	//m_showUI = false;
	//m_showAABB = true;
	//m_showScene = true;
	//m_showConcaveEdge = true;
	//m_showMeshSkeleton = true;
	//m_autoSleepMode = false;
	///m_hidePostUpdate = true;
	//m_hideVisualMeshes = true;
	//m_solverMode = ndWorld::ndStandardSolver;
	//m_solverMode = ndWorld::ndSimdSoaSolver;
	//m_solverMode = ndWorld::ndSimdAvx2Solver;
	//m_solverPasses = 4;
	//m_workerThreads = 1;
	//m_solverSubSteps = 2;
	//m_showRaycastHit = true;
	//m_showCenterOfMass = true;
	//m_showNormalForces = true;
	//m_showContactPoints = true;
	//m_showJointDebugInfo = true;
	//m_showModelsDebugInfo = true;
	//m_showCollisionMeshMode = 1;
	//m_showCollisionMeshMode = 2;
	//m_showCollisionMeshMode = 3;		// solid wire frame
	//m_synchronousPhysicsUpdate = true;
	//m_synchronousParticlesUpdate = true;
	//m_showStaticMeshCollidingFaces = true;

	Cleanup();
	//ndResetTimer();
	ApplyOptions();
}

ndDemoEntityManager::~ndDemoEntityManager ()
{
	//Cleanup ();
	//
	//// destroy the empty world
	//if (m_world) 
	//{
	//	delete m_world;
	//}
}

//ndPhysicsWorld* ndDemoEntityManager::GetWorld() const
//{
//	return m_world;
//}

ndSharedPtr<ndRender>& ndDemoEntityManager::GetRenderer()
{
	return m_renderer;
}

//ndDebugDisplayRenderPass* ndDemoEntityManager::GetDebugRenderPass()
//{
//	return (ndDebugDisplayRenderPass*)*m_debugDisplayRenderPass;
//}
//
//void ndDemoEntityManager::Terminate()
//{
//	m_renderer->Terminate();
//}

//ndInt32 ndDemoEntityManager::GetWidth() const
//{
//	return m_renderer->GetWidth();
//}
//
//ndInt32 ndDemoEntityManager::GetHeight() const
//{
//	return m_renderer->GetHeight();
//}

bool ndDemoEntityManager::GetKeyState(ndInt32 key) const
{
	const ImGuiIO& io = ImGui::GetIO();
	bool state = io.KeysDown[key];
	return state;
}

bool ndDemoEntityManager::AnyKeyDown() const
{
	const ImGuiIO& io = ImGui::GetIO();
	for (ndInt32 i = 0; i < ImGuiKey_COUNT; ++i)
	{
		if (io.KeysDown[i])
		{
			return true;
		}
	}
	return false;
}

void ndDemoEntityManager::CharCallback(ndUnsigned32 ch)
{
	ImGuiIO& io = ImGui::GetIO();
	io.AddInputCharacter((unsigned short)ch);
}

void ndDemoEntityManager::CursorposCallback(ndReal x, ndReal y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MousePos = ImVec2(x, y);
}

void ndDemoEntityManager::MouseScrollCallback(ndReal, ndReal y)
{
	//ndTrace(("%f %f\n", x, y));
	ImGuiIO& io = ImGui::GetIO();
	io.MouseWheel += y;
}

void ndDemoEntityManager::MouseButtonCallback(ndInt32 button, ndInt32 action)
{
	const ndInt32 KEY_PRESS = 1;
	const ndInt32 KEY_RELEASE = 0;

	if (button >= 0 && button < 3) 
	{
		ImGuiIO& io = ImGui::GetIO();
		if (action == KEY_PRESS)
		{
			io.MouseDown[button] = true;    
		} 
		else if (action == KEY_RELEASE)
		{
			io.MouseDown[button] = false;    
		}
	}
}

void ndDemoEntityManager::KeyCallback(ndInt32 key, ndInt32)
{
	if (key == ImGuiKey_F1)
	{
		ndAssert(0);
		//// reload the demo. 
		//const ndTransform transform(m_renderer->GetCamera()->GetTransform());
		//LoadDemo(m_lastCurrentScene);
		//m_renderer->GetCamera()->SetTransform(transform);
		//m_renderer->GetCamera()->SetTransform(transform);
	}
	else if (key == ImGuiKey_F10)
	{
		// set debug tracer here;
		//ndAssert(0);
	}
}

bool ndDemoEntityManager::IsShiftKeyDown () const
{
	const ImGuiIO& io = ImGui::GetIO();
	const ndInt32 KEY_LEFT_SHIFT = 340;
	const ndInt32 KEY_RIGHT_SHIFT = 344;
	bool state = io.KeysDown[KEY_LEFT_SHIFT] || io.KeysDown[KEY_RIGHT_SHIFT];
	return state;
}

bool ndDemoEntityManager::IsControlKeyDown () const
{
	ndAssert(0);
	return 0;
	//const ImGuiIO& io = ImGui::GetIO();
	//bool state = io.KeysDown[GLFW_KEY_LEFT_CONTROL] || io.KeysDown[GLFW_KEY_RIGHT_CONTROL];
	//return state;
}

bool ndDemoEntityManager::GetCaptured() const
{
	ImGuiIO& io = ImGui::GetIO();
	return io.WantCaptureMouse;
}

bool ndDemoEntityManager::GetMouseKeyState (ndInt32 button) const
{
	ImGuiIO& io = ImGui::GetIO();
	return io.MouseDown[button];
}

bool ndDemoEntityManager::JoystickDetected() const
{
	ndAssert(0);
	return 0;
	//return glfwJoystickPresent(0) ? true : false;
}

void ndDemoEntityManager::GetJoystickAxis(ndFixSizeArray<ndFloat32, 8>&)
{
	ndAssert(0);
	//if (JoystickDetected())
	//{
	//	bool isInitialized = false;
	//	static ndFixSizeArray<ndFloat32, 8> initialValues;
	//	if (!initialValues.GetCount())
	//	{
	//		ndInt32 axisCount = 0;
	//		const float* const axis = glfwGetJoystickAxes(0, &axisCount);
	//		axisCount = ndMin(axisCount, axisValues.GetCapacity());
	//		for (ndInt32 i = 0; i < axisCount; ++i)
	//		{
	//			initialValues.PushBack(axis[i]);
	//		}
	//	}
	//	
	//	if (!isInitialized)
	//	{
	//		ndInt32 axisCount = 0;
	//		const float* const axis = glfwGetJoystickAxes(0, &axisCount);
	//		for (ndInt32 i = 0; i < axisCount; ++i)
	//		{
	//			ndFloat32 diff = ndAbs(axis[i] - initialValues[i]);
	//			isInitialized = isInitialized || (diff != ndFloat32(0.0f));
	//		}
	//	}
	//
	//	axisValues.SetCount(0);
	//	for (ndInt32 i = 0; i < axisValues.GetCapacity(); ++i)
	//	{
	//		axisValues.PushBack(ndFloat32 (1.0f));
	//	}
	//	axisValues[0] = 0.0f;
	//
	//	if (isInitialized)
	//	{
	//		ndInt32 axisCount = 0;
	//		const float* const axis = glfwGetJoystickAxes(0, &axisCount);
	//		axisCount = ndMin(axisCount, axisValues.GetCapacity());
	//
	//		axisValues.SetCount(0);
	//		for (ndInt32 i = 0; i < axisCount; ++i)
	//		{
	//			axisValues.PushBack(axis[i]);
	//		}
	//	}
	//}
}

void ndDemoEntityManager::GetJoystickButtons(ndFixSizeArray<char, 32>&)
{
	ndAssert(0);
	//if (JoystickDetected())
	//{
	//	ndInt32 buttonsCount = 0;
	//	axisbuttons.SetCount(0);
	//	const unsigned char* const buttons = glfwGetJoystickButtons(0, &buttonsCount);
	//	buttonsCount = ndMin(buttonsCount, axisbuttons.GetCapacity());
	//
	//	for (ndInt32 i = 0; i < buttonsCount; ++i)
	//	{
	//		axisbuttons.PushBack(char(buttons[i]));
	//	}
	//}
}

void ndDemoEntityManager::RegisterPostUpdate(const ndSharedPtr<OnPostUpdate>& postUpdate)
{
	m_onPostUpdate = postUpdate;
}

//void ndDemoEntityManager::AddEntity(const ndSharedPtr<ndRenderSceneNode>& entity)
//{
//	//ndScopeSpinLock lock(m_addDeleteLock);
//	m_renderer->AddSceneNode(entity);
//}
//
//void ndDemoEntityManager::RemoveEntity (const ndSharedPtr<ndRenderSceneNode>& entity)
//{
//	//ndScopeSpinLock lock(m_addDeleteLock);
//	m_renderer->RemoveSceneNode(entity);
//}

void ndDemoEntityManager::Cleanup ()
{
	// is we are run asynchronous we need make sure no update in on flight.
	//if (m_world) 
	//{
	//	m_world->Sync();
	//}
	//
	//m_renderer->ResetScene();
	//RegisterPostUpdate(ndSharedPtr<OnPostUpdate>(nullptr));
	//
	//// destroy the Newton world
	//if (m_world) 
	//{
	//	// get serialization call back before destroying the world
	//	m_world->CleanUp();
	//	delete m_world;
	//}
	//
	//// create the newton world
	//m_world = new ndPhysicsWorld(this);
	//ApplyMenuOptions();
}

void ndDemoEntityManager::ApplyMenuOptions()
{
	//m_world->Sync();
	//m_world->SetSubSteps(m_solverSubSteps);
	//m_world->SetSolverIterations(m_solverPasses);
	//m_world->SetThreadCount(m_workerThreads);
	//
	//bool state = m_autoSleepMode ? true : false;
	//const ndBodyListView& bodyList = m_world->GetBodyList();
	//for (ndBodyListView::ndNode* node = bodyList.GetFirst(); node; node = node->GetNext())
	//{
	//	ndBodyKinematic* const body = node->GetInfo()->GetAsBodyKinematic();
	//	body->SetAutoSleep(state);
	//}
	//
	//SetParticleUpdateMode();
	//m_world->SelectSolver(m_solverMode);
	//m_solverMode = m_world->GetSelectedSolver();
}

void ndDemoEntityManager::ApplyOptions()
{
	//m_colorRenderPass->MakeActive(!m_hideVisualMeshes);
	//m_shadowRenderPass->MakeActive(!m_hideVisualMeshes);
	//m_transparentRenderPass->MakeActive(!m_hideVisualMeshes);
	//
	//ndDebugDisplayRenderPass* const debugDisplay = (ndDebugDisplayRenderPass*)*m_debugDisplayRenderPass;
	//debugDisplay->SetDebugDisplayOptions();
}

void ndDemoEntityManager::ShowMainMenuBar()
{
	ndMenuSelection menuSelection = m_none;
	if (ImGui::BeginMainMenuBar())
	{
		//if (ImGui::BeginMenu("File")) 
		//{
		//	m_suspendPhysicsUpdate = true;
		//
		//	if (ImGui::MenuItem("Preferences", "")) 
		//	{
		//		ndAssert (0);
		//	}
		//	ImGui::Separator();
		//
		//	if (ImGui::MenuItem("New", "")) 
		//	{
		//		menuSelection = m_new;
		//	}
		//
		//	ImGui::Separator();
		//	if (ImGui::MenuItem("import ply file", "")) 
		//	{
		//		//mainMenu = 4;
		//	}
		//
		//	ImGui::Separator();
		//	if (ImGui::MenuItem("Exit", "")) 
		//	{
		//		m_renderer->Terminate();
		//	}
		//
		//	ImGui::EndMenu();
		//}
	
		//if (ImGui::BeginMenu("Demos")) 
		//{
		//	m_suspendPhysicsUpdate = true;
		//	ndInt32 demosCount = ndInt32 (sizeof (m_demosSelection) / sizeof m_demosSelection[0]);
		//	for (ndInt32 i = 0; i < demosCount; ++i) 
		//	{
		//		if (ImGui::MenuItem(m_demosSelection[i].m_name, "")) 
		//		{
		//			m_currentScene = i;
		//		}
		//	}
		//
		//	ImGui::EndMenu();
		//}

		bool optionsOn = ImGui::BeginMenu("Options");
		if (optionsOn) 
		{
			m_updateMenuOptions = true;
			m_suspendPhysicsUpdate = true;
	
			ImGui::Checkbox("auto sleep mode", &m_autoSleepMode);
			ImGui::Checkbox("show UI", &m_showUI);
			ImGui::Checkbox("show stats", &m_showStats);
			ImGui::Checkbox("show helper legend", &m_helperLegend);
			ImGui::Checkbox("synchronous physics update", &m_synchronousPhysicsUpdate);
			ImGui::Checkbox("synchronous particle update", &m_synchronousParticlesUpdate);
			ImGui::Separator();
	
			ImGui::Text("solvers");
			ndInt32 solverMode(m_solverMode);
			ImGui::RadioButton("default", &solverMode, ndWorld::ndStandardSolver);
			ImGui::RadioButton("sse", &solverMode, ndWorld::ndSimdSoaSolver);
			ImGui::RadioButton("avx2", &solverMode, ndWorld::ndSimdAvx2Solver);
	
			m_solverMode = ndWorld::ndSolverModes(solverMode);
			ImGui::Separator();
	
			ImGui::Text("solver sub steps");
			ImGui::SliderInt("##solv", &m_solverSubSteps, 2, 8);
			ImGui::Text("iterative solver passes");
			ImGui::SliderInt("##intera", &m_solverPasses, 4, 32);
			ImGui::Text("worker threads");
			ImGui::SliderInt("##worker", &m_workerThreads, 1, ndThreadPool::GetMaxThreads());

			ImGui::Separator();
			ImGui::Checkbox("hide visual meshes", &m_hideVisualMeshes);
			//ImGui::Checkbox("show mesh skeleton", &m_showMeshSkeleton);

			ImGui::Separator();
			//ImGui::RadioButton("show UI", &m_showUI);
			ImGui::RadioButton("hide collision Mesh", &m_showCollisionMeshMode, 0);
			ImGui::RadioButton("show solid collision", &m_showCollisionMeshMode, 1);
			ImGui::RadioButton("show wire frame collision", &m_showCollisionMeshMode, 2);
			ImGui::RadioButton("show hidden wire frame collision", &m_showCollisionMeshMode, 3);
	
			ImGui::Separator();
			ImGui::Checkbox("show aabb", &m_showAABB);
			ImGui::Checkbox("show body frame", &m_showBodyFrame);
			ImGui::Checkbox("show broad phase", &m_showScene);
			ImGui::Checkbox("show contact points", &m_showContactPoints);
			ImGui::Checkbox("show contact forces", &m_showNormalForces);
			ImGui::Checkbox("show center of mass", &m_showCenterOfMass);
			ImGui::Checkbox("show joints debug info", &m_showJointDebugInfo);
			ImGui::Checkbox("show models debug info", &m_showModelsDebugInfo);
			ImGui::Checkbox("show colliding faces", &m_showStaticMeshCollidingFaces);

			//ImGui::Checkbox("show ray cast hit point", &m_showRaycastHit);
			//ImGui::Checkbox("show concave edges", &m_showConcaveEdge);
			

			ApplyOptions();
	
			ImGui::EndMenu();
	
			SetParticleUpdateMode();
		}

		if (ImGui::BeginMenu("Help")) 
		{
			m_suspendPhysicsUpdate = true;
			ImGui::EndMenu();
		}
	
		ImGui::EndMainMenuBar();
	
		if (!optionsOn && m_updateMenuOptions) 
		{
			m_updateMenuOptions = false;
			ApplyMenuOptions();
		}
	}
	
	switch (menuSelection)
	{
		case m_new:
		{
			// menu new 
			ndAssert(0);
			//ndMatrix matrix (GetCamera()->GetCurrentMatrix());
			//Cleanup();
			//ApplyMenuOptions();
			//ResetTimer();
			//m_currentScene = -1;
			//SetCameraMatrix(ndQuaternion(matrix), matrix.m_posit);
			break;
		}
	
		case m_none:
		default:
		{
			// load a demo 
			//if (m_currentScene != -1) 
			//{
			//	//m_selectedModel = nullptr;
			//	//RegisterPostUpdate(nullptr);
			//	LoadDemo(m_currentScene);
			//	m_lastCurrentScene = m_currentScene;
			//	m_currentScene = -1;
			//}
		}
	}
}

//void ndDemoEntityManager::LoadDemo(ndInt32 menuIndex)
//{
//	Cleanup();
//	
//	char newTitle[256];
//
//	// add a demo camera per demo
//	m_demoHelper = ndSharedPtr<ndDemoHelper>(nullptr);
//	m_demoUIpanel = ndSharedPtr<ndDemoUIpanel>(nullptr);
//	m_defaultCamera = ndSharedPtr<ndRenderSceneNode>(new ndDemoCameraNodeFlyby(*m_renderer));
//	m_renderer->SetCamera(m_defaultCamera);
//
//	if (menuIndex < MACHINE_LEARNING_BASE)
//	{
//		m_demosSelection[menuIndex].m_demoLauncher(this);
//		snprintf(newTitle, sizeof(newTitle), "Newton Dynamics %d.%.2i demo: %s", D_NEWTON_ENGINE_MAJOR_VERSION, D_NEWTON_ENGINE_MINOR_VERSION, m_demosSelection[menuIndex].m_name);
//	}
//	else
//	{
//		menuIndex -= MACHINE_LEARNING_BASE;
//		m_machineLearning[menuIndex].m_demoLauncher(this);
//		snprintf(newTitle, sizeof(newTitle), "Newton Dynamics %d.%.2i demo: %s", D_NEWTON_ENGINE_MAJOR_VERSION, D_NEWTON_ENGINE_MINOR_VERSION, m_machineLearning[menuIndex].m_name);
//	}
//
//	m_renderer->SetTitle(newTitle);
//	ApplyMenuOptions();
//	ndResetTimer();
//	
//	ndAssert (m_world->ValidateScene());
//}

bool ndDemoEntityManager::GetMouseSpeed(ndFloat32& speedX, ndFloat32& speedY) const
{
	ImVec2 speed(ImGui::GetMouseDragDelta(0, 0.0f));
	speedX = speed.x;
	speedY = speed.y;
	return true;
}

bool ndDemoEntityManager::GetMousePosition (ndFloat32& posX, ndFloat32& posY) const
{
	ImVec2 posit(ImGui::GetMousePos());
	posX = ndClamp(posit.x, ndReal(-1.0e10f), ndReal(1.0e10f));
	posY = ndClamp(posit.y, ndReal(-1.0e10f), ndReal(1.0e10f));
	return true;
}

void ndDemoEntityManager::ToggleProfiler()
{
	#ifdef D_PROFILER
		ndAssert(m_world);
		ndTrace(("profiler Enable\n"));
		m_world->Sync();
		dProfilerEnableProling();
	#endif
}

ndInt32 ndDemoEntityManager::ParticleCount() const
{
	ndInt32 count = 0;
	//const ndBodyList& particles = m_world->GetParticleList();
	//for (ndBodyList::ndNode* node = particles.GetFirst(); node; node = node->GetNext())
	//{
	//	ndBodyParticleSet* const set = node->GetInfo()->GetAsBodyParticleSet();
	//	count += ndInt32(set->GetPositions().GetCount());
	//}
	return count;
}

void ndDemoEntityManager::SetParticleUpdateMode() const
{
	//const ndBodyList& particles = m_world->GetParticleList();
	//for (ndBodyList::ndNode* node = particles.GetFirst(); node; node = node->GetNext())
	//{
	//	ndBodyParticleSet* const set = node->GetInfo()->GetAsBodyParticleSet();
	//	set->SetAsynUpdate(!m_synchronousParticlesUpdate);
	//}
}

void ndDemoEntityManager::RenderStats()
{
	if (m_showStats) 
	{
		char text[1024];
		
		ndAssert(0);
		//if (ImGui::Begin("statistics", &m_showStats)) 
		//{
		//	snprintf(text, sizeof (text), "fps:            %6.3f", m_fps);
		//	ImGui::Text(text, "");
		//
		//	snprintf(text, sizeof (text), "physics time:  %6.3f ms", m_world->GetAverageUpdateTime() * 1.0e3f);
		//	ImGui::Text(text, "");
		//
		//	snprintf(text, sizeof (text), "update mode:    %s", m_synchronousPhysicsUpdate ? "synchronous" : "asynchronous");
		//	ImGui::Text(text, "");
		//
		//	snprintf(text, sizeof (text), "particle mode:  %s", m_synchronousParticlesUpdate ? "synchronous" : "asynchronous");
		//	ImGui::Text(text, "");
		//
		//	snprintf(text, sizeof (text), "bodies:         %d", m_world->GetBodyList().GetCount());
		//	ImGui::Text(text, "");
		//
		//	snprintf(text, sizeof (text), "joints:         %d", m_world->GetJointList().GetCount());
		//	ImGui::Text(text, "");
		//
		//	snprintf(text, sizeof (text), "contact joints: %d", m_world->GetContactList().GetActiveContacts());
		//	ImGui::Text(text, "");
		//
		//	snprintf(text, sizeof (text), "particles:      %d", ParticleCount());
		//	ImGui::Text(text, "");
		//
		//	snprintf(text, sizeof (text), "memory used:   %6.3f mbytes", ndFloat32(ndFloat64(ndMemory::GetMemoryUsed()) / (1024 * 1024)));
		//	ImGui::Text(text, "");
		//
		//	snprintf(text, sizeof (text), "threads:        %d", m_world->GetThreadCount());
		//	ImGui::Text(text, "");
		//
		//	snprintf(text, sizeof (text), "iterations:     %d", m_world->GetSolverIterations());
		//	ImGui::Text(text, "");
		//
		//	snprintf(text, sizeof (text), "Substeps:       %d", m_world->GetSubSteps());
		//	ImGui::Text(text, "");
		//
		//	snprintf(text, sizeof (text), "solver:         %s", m_world->GetSolverString());
		//	ImGui::Text(text, "");
		//
		//	m_suspendPhysicsUpdate = m_suspendPhysicsUpdate || (ImGui::IsWindowHovered() && ImGui::IsMouseDown(0));
		//	ImGui::End();
		//}
	}
	
	//if (*m_demoHelper)
	//{
	//	if (m_helperLegend)
	//	{	
	//		m_helperLegend = false;
	//		m_demoHelper->ResetTime();
	//	}
	//
	//	if (!m_demoHelper->ExpirationTime())
	//	{
	//		bool dummy = true;
	//		if (ImGui::Begin("User Interface", &dummy))
	//		{
	//			m_demoHelper->PresentHelp(this);
	//			ImGui::End();
	//		}
	//	}
	//}

	//if (*m_demoUIpanel)
	//{
	//	bool dummy = true;
	//	if (ImGui::Begin("Control panel", &dummy))
	//	{
	//		m_demoUIpanel->Update(this);
	//		ImGui::End();
	//	}
	//}

	ShowMainMenuBar();
}

void ndDemoEntityManager::SetDemoHelp(ndSharedPtr<ndDemoHelper>& helper)
{
	//m_demoHelper = helper;
}

void ndDemoEntityManager::SetDemoUIpanel(ndSharedPtr<ndDemoUIpanel>& panel)
{
	//m_demoUIpanel = panel;
}

void ndDemoEntityManager::SetNextActiveCamera()
{
	ndAssert(0);
	//if (!m_nextActiveCamera.Update(GetKeyState(ImGuiKey_C) ? true : false))
	//{
	//	return;
	//}
	//
	//ndFixSizeArray<const ndRenderSceneCamera*, 256> cameraPallete;
	//cameraPallete.PushBack(m_defaultCamera->FindCameraNode());
	//
	//ndList<ndSharedPtr<ndRenderSceneNode>>& scene = m_renderer->GetScene();
	//for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* sceneNode = scene.GetFirst(); sceneNode; sceneNode = sceneNode->GetNext())
	//{
	//	ndSharedPtr<ndRenderSceneNode>& node = sceneNode->GetInfo();
	//	const ndRenderSceneCamera* cameraNode = node->FindCameraNode();
	//	if (cameraNode)
	//	{
	//		cameraPallete.PushBack(cameraNode);
	//	}
	//}
	//
	//const ndRenderSceneCamera* const currentCamera = m_renderer->GetCamera()->FindCameraNode();
	//for (ndInt32 i = 0; i < cameraPallete.GetCount(); ++i)
	//{
	//	if (cameraPallete[i] == currentCamera)
	//	{
	//		ndInt32 j = (i + 1) % cameraPallete.GetCount();
	//		if (j == 0)
	//		{
	//			const ndTransform tranform (currentCamera->CalculateGlobalTransform());
	//			m_defaultCamera->SetTransform(tranform);
	//			m_defaultCamera->SetTransform(tranform);
	//			m_renderer->SetCamera(m_defaultCamera);
	//		}
	//		else
	//		{
	//			ndRenderSceneNode* const camera = cameraPallete[j]->FindByName("__PlayerCamera__");
	//			ndSharedPtr<ndRenderSceneNode> cameraNode(camera->GetSharedPtr());
	//			m_renderer->SetCamera(cameraNode);
	//		}
	//		break;
	//	}
	//}
}

void ndDemoEntityManager::CalculateFPS(ndFloat32 timestep)
{
	m_framesCount ++;
	m_timestepAcc += timestep;

	// this probably happing on loading of and a pause, just rest counters
	if ((m_timestepAcc <= 0.0f) || (m_timestepAcc > 4.0f))
	{
		m_timestepAcc = 0;
		m_framesCount = 0;
	}

	//update fps every quarter of a second
	const ndFloat32 movingAverage = 0.5f;
	if (m_timestepAcc >= movingAverage)
	{
		m_fps = ndFloat32 (m_framesCount) / m_timestepAcc;
		m_timestepAcc -= movingAverage;
		m_framesCount = 0;
	}
}

//void ndDemoEntityManager::ImportPLYfile(const char* const)
//{
//	ndAssert(0);
//	//m_showCollisionMeshMode = 2;
//	//CreatePLYMesh (this, fileName, true);
//}

ndInt32 ndDemoEntityManager::Print (const ndVector&, const char *fmt, ... ) const
{
	va_list argptr;
	char string[1024];

	va_start (argptr, fmt);
	vsnprintf (string, sizeof (string), fmt, argptr);
	va_end( argptr );
	ImGui::Text(string, "");
	return 0;
}

void ndDemoEntityManager::SetCameraMatrix (const ndQuaternion& rotation, const ndVector& position)
{
	ndAssert(0);
	//ndRenderSceneNode* const cameraNode = *m_renderer->GetCamera();
	//cameraNode->SetTransform(rotation, position);
	//cameraNode->SetTransform(rotation, position);
}

void ndDemoEntityManager::UpdatePhysics(ndFloat32 timestep)
{
	ndAssert(0);
	//// update the physics
	//if (m_world && !m_suspendPhysicsUpdate) 
	//{
	//	m_world->AdvanceTime(timestep);
	//}
}

void ndDemoEntityManager::SetAcceleratedUpdate()
{
	ndAssert(0);
	//m_world->AccelerateUpdates();
}

//void ndDemoEntityManager::OnSubStepPostUpdate(ndFloat32 timestep)
void ndDemoEntityManager::OnSubStepPostUpdate(ndFloat32)
{
	//if (m_colorRenderPass)
	//{
	////	((ndRenderPassColor*)m_colorRenderPass)->UpdateDebugDisplay(timestep);
	//}
}

void ndDemoEntityManager::RenderScene()
{
	//D_TRACKTIME();
	//ndFloat32 timestep = ndGetElapsedSeconds();	
	//CalculateFPS(timestep);
	//UpdatePhysics(timestep);
	m_renderer->Render();
}

void ndDemoEntityManager::TestImGui()
{
	// Main loop
	bool show_demo_window = true;
	bool show_another_window = false;

	// 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
	if (show_demo_window)
	{
		ImGui::ShowDemoWindow(&show_demo_window);
	}

	// 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
	if (1)
	{
		static float f = 0.0f;
		static int counter = 0;

		ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

		ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
		ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
		ImGui::Checkbox("Another Window", &show_another_window);

		ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
		ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
		ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

		if (ImGui::Button("Button"))
		{
			// Buttons return true when clicked (most widgets return true when edited/activated)
			counter++;
		}
		ImGui::SameLine();
		ImGui::Text("counter = %d", counter);

		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::End();
	}

	// 3. Show another simple window.
	if (show_another_window)
	{
		ImGui::Begin("Another Window", &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
		ImGui::Text("Hello from another window!");
		if (ImGui::Button("Close Me"))
		{
			show_another_window = false;
		}
		ImGui::End();
	}
	ImGui::Render();
}

void ndDemoEntityManager::Run()
{
	// Main loop
	ndFloatExceptions exception;
	while (!m_renderer->ShouldFinish())
	{
		if (m_profilerMode)
		{
			ToggleProfiler();
			m_profilerMode = false;
		}
	
		m_suspendPhysicsUpdate = false;
		D_TRACKTIME();
		
		if (m_renderer->PollEvents())
		{
			RenderScene();
		}
	}
}

