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

ndAssetEditor::ButtonKey::ButtonKey (bool state)
	:m_state(state)
	,m_memory0(false)
	,m_memory1(false)
{
}

ndInt32 ndAssetEditor::ButtonKey::UpdateTrigger (bool triggerValue)
{
	m_memory0 = m_memory1;
	m_memory1 = triggerValue;
	return (!m_memory0 && m_memory1) ? 1 : 0;
}

ndInt32 ndAssetEditor::ButtonKey::UpdatePushButton (bool triggerValue)
{
	if (UpdateTrigger (triggerValue)) 
	{
		m_state = ! m_state;
	}
	return m_state ? 1 : 0;
}

// ImGui - standalone example application for Glfw + OpenGL 2, using fixed pipeline
// If you are new to ImGui, see examples/README.txt and documentation at the top of imgui.cpp.
ndAssetEditor::ndAssetEditor()
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
	,m_currentPath("")
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
	,m_runScene(false)
	//,m_showUI(true)
	//,m_showAABB(false)
	//,m_showStats(true)
	//,m_helperLegend(false)
	//,m_autoSleepMode(true)
	//,m_showScene(false)
	//,m_showConcaveEdge(false)
	//,m_hideVisualMeshes(false)
	//,m_showNormalForces(false)
	//,m_showCenterOfMass(false)
	//,m_showBodyFrame(false)
	//,m_showMeshSkeleton(false)
	//,m_updateMenuOptions(true)
	//,m_showContactPoints(false)
	//,m_showJointDebugInfo(false)
	//,m_showModelsDebugInfo(false)
	//,m_suspendPhysicsUpdate(false)
	//,m_synchronousPhysicsUpdate(false)
	//,m_synchronousParticlesUpdate(false)
	//,m_showStaticMeshCollidingFaces(false)
	//,m_showRaycastHit(false)
	//,m_profilerMode(false)
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

	ImGuiIO& io = ImGui::GetIO();
	//io.ConfigFlags |= ImGuiWindowFlags_MenuBar;
	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

	Cleanup();
	ApplyOptions();
}

ndAssetEditor::~ndAssetEditor ()
{
	//Cleanup ();
	//
	//// destroy the empty world
	//if (m_world) 
	//{
	//	delete m_world;
	//}
}

//ndPhysicsWorld* ndAssetEditor::GetWorld() const
//{
//	return m_world;
//}

ndSharedPtr<ndRender>& ndAssetEditor::GetRenderer()
{
	return m_renderer;
}

//ndDebugDisplayRenderPass* ndAssetEditor::GetDebugRenderPass()
//{
//	return (ndDebugDisplayRenderPass*)*m_debugDisplayRenderPass;
//}
//
//void ndAssetEditor::Terminate()
//{
//	m_renderer->Terminate();
//}

//ndInt32 ndAssetEditor::GetWidth() const
//{
//	return m_renderer->GetWidth();
//}
//
//ndInt32 ndAssetEditor::GetHeight() const
//{
//	return m_renderer->GetHeight();
//}

bool ndAssetEditor::GetKeyState(ndInt32 key) const
{
	const ImGuiIO& io = ImGui::GetIO();
	bool state = io.KeysDown[key];
	return state;
}

bool ndAssetEditor::AnyKeyDown() const
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

void ndAssetEditor::CharCallback(ndUnsigned32 ch)
{
	ImGuiIO& io = ImGui::GetIO();
	io.AddInputCharacter((unsigned short)ch);
}

void ndAssetEditor::CursorposCallback(ndReal x, ndReal y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MousePos = ImVec2(x, y);
}

void ndAssetEditor::MouseScrollCallback(ndReal, ndReal y)
{
	//ndTrace(("%f %f\n", x, y));
	ImGuiIO& io = ImGui::GetIO();
	io.MouseWheel += y;
}

void ndAssetEditor::MouseButtonCallback(ndInt32 button, ndInt32 action)
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

void ndAssetEditor::KeyCallback(ndInt32 key, ndInt32)
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

bool ndAssetEditor::IsShiftKeyDown () const
{
	const ImGuiIO& io = ImGui::GetIO();
	const ndInt32 KEY_LEFT_SHIFT = 340;
	const ndInt32 KEY_RIGHT_SHIFT = 344;
	bool state = io.KeysDown[KEY_LEFT_SHIFT] || io.KeysDown[KEY_RIGHT_SHIFT];
	return state;
}

bool ndAssetEditor::IsControlKeyDown () const
{
	ndAssert(0);
	return 0;
	//const ImGuiIO& io = ImGui::GetIO();
	//bool state = io.KeysDown[GLFW_KEY_LEFT_CONTROL] || io.KeysDown[GLFW_KEY_RIGHT_CONTROL];
	//return state;
}

bool ndAssetEditor::GetCaptured() const
{
	ImGuiIO& io = ImGui::GetIO();
	return io.WantCaptureMouse;
}

bool ndAssetEditor::GetMouseKeyState (ndInt32 button) const
{
	ImGuiIO& io = ImGui::GetIO();
	return io.MouseDown[button];
}

bool ndAssetEditor::JoystickDetected() const
{
	ndAssert(0);
	return 0;
	//return glfwJoystickPresent(0) ? true : false;
}

void ndAssetEditor::GetJoystickAxis(ndFixSizeArray<ndFloat32, 8>&)
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

void ndAssetEditor::GetJoystickButtons(ndFixSizeArray<char, 32>&)
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

void ndAssetEditor::RegisterPostUpdate(const ndSharedPtr<OnPostUpdate>& postUpdate)
{
	m_onPostUpdate = postUpdate;
}

//void ndAssetEditor::AddEntity(const ndSharedPtr<ndRenderSceneNode>& entity)
//{
//	//ndScopeSpinLock lock(m_addDeleteLock);
//	m_renderer->AddSceneNode(entity);
//}
//
//void ndAssetEditor::RemoveEntity (const ndSharedPtr<ndRenderSceneNode>& entity)
//{
//	//ndScopeSpinLock lock(m_addDeleteLock);
//	m_renderer->RemoveSceneNode(entity);
//}

void ndAssetEditor::Cleanup ()
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

void ndAssetEditor::ApplyMenuOptions()
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

void ndAssetEditor::ApplyOptions()
{
	//m_colorRenderPass->MakeActive(!m_hideVisualMeshes);
	//m_shadowRenderPass->MakeActive(!m_hideVisualMeshes);
	//m_transparentRenderPass->MakeActive(!m_hideVisualMeshes);
	//
	//ndDebugDisplayRenderPass* const debugDisplay = (ndDebugDisplayRenderPass*)*m_debugDisplayRenderPass;
	//debugDisplay->SetDebugDisplayOptions();
}


bool ndAssetEditor::GetMouseSpeed(ndFloat32& speedX, ndFloat32& speedY) const
{
	ImVec2 speed(ImGui::GetMouseDragDelta(0, 0.0f));
	speedX = speed.x;
	speedY = speed.y;
	return true;
}

bool ndAssetEditor::GetMousePosition (ndFloat32& posX, ndFloat32& posY) const
{
	ImVec2 posit(ImGui::GetMousePos());
	posX = ndClamp(posit.x, ndReal(-1.0e10f), ndReal(1.0e10f));
	posY = ndClamp(posit.y, ndReal(-1.0e10f), ndReal(1.0e10f));
	return true;
}

void ndAssetEditor::ToggleProfiler()
{
	#ifdef D_PROFILER
		ndAssert(m_world);
		ndTrace(("profiler Enable\n"));
		m_world->Sync();
		dProfilerEnableProling();
	#endif
}

ndInt32 ndAssetEditor::ParticleCount() const
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

void ndAssetEditor::SetParticleUpdateMode() const
{
	//const ndBodyList& particles = m_world->GetParticleList();
	//for (ndBodyList::ndNode* node = particles.GetFirst(); node; node = node->GetNext())
	//{
	//	ndBodyParticleSet* const set = node->GetInfo()->GetAsBodyParticleSet();
	//	set->SetAsynUpdate(!m_synchronousParticlesUpdate);
	//}
}

//void ndAssetEditor::SetDemoHelp(ndSharedPtr<ndDemoHelper>& helper)
void ndAssetEditor::SetDemoHelp(ndSharedPtr<ndDemoHelper>&)
{
	//m_demoHelper = helper;
}

//void ndAssetEditor::SetDemoUIpanel(ndSharedPtr<ndDemoUIpanel>& panel)
void ndAssetEditor::SetDemoUIpanel(ndSharedPtr<ndDemoUIpanel>&)
{
	//m_demoUIpanel = panel;
}

void ndAssetEditor::SetNextActiveCamera()
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

void ndAssetEditor::CalculateFPS(ndFloat32 timestep)
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

ndInt32 ndAssetEditor::Print (const ndVector&, const char *fmt, ... ) const
{
	va_list argptr;
	char string[1024];

	va_start (argptr, fmt);
	vsnprintf (string, sizeof (string), fmt, argptr);
	va_end( argptr );
	ImGui::Text(string, "");
	return 0;
}

//void ndAssetEditor::SetCameraMatrix (const ndQuaternion& rotation, const ndVector& position)
void ndAssetEditor::SetCameraMatrix(const ndQuaternion&, const ndVector&)
{
	ndAssert(0);
	//ndRenderSceneNode* const cameraNode = *m_renderer->GetCamera();
	//cameraNode->SetTransform(rotation, position);
	//cameraNode->SetTransform(rotation, position);
}

//void ndAssetEditor::UpdatePhysics(ndFloat32 timestep)
void ndAssetEditor::UpdatePhysics(ndFloat32)
{
	ndAssert(0);
	//// update the physics
	//if (m_world && !m_suspendPhysicsUpdate) 
	//{
	//	m_world->AdvanceTime(timestep);
	//}
}

void ndAssetEditor::SetAcceleratedUpdate()
{
	ndAssert(0);
	//m_world->AccelerateUpdates();
}

//void ndAssetEditor::OnSubStepPostUpdate(ndFloat32 timestep)
void ndAssetEditor::OnSubStepPostUpdate(ndFloat32)
{
	//if (m_colorRenderPass)
	//{
	////	((ndRenderPassColor*)m_colorRenderPass)->UpdateDebugDisplay(timestep);
	//}
}

void ndAssetEditor::RenderScene()
{
	//D_TRACKTIME();
	//ndFloat32 timestep = ndGetElapsedSeconds();	
	//CalculateFPS(timestep);
	//UpdatePhysics(timestep);
	m_renderer->Render();
}

void ndAssetEditor::TestImGui()
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

void ndAssetEditor::Run()
{
	// Main loop
	ndFloatExceptions exception;
	while (!m_renderer->ShouldFinish())
	{
		if (m_renderer->PollEvents())
		{
			RenderScene();
		}
	}
}

void ndAssetEditor::RenderLayout()
{
	ShowMainMenuBar();
	ShowOutlier();
}

void ndAssetEditor::ShowMainMenuBar()
{
	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu("File"))
		{
			if (ImGui::MenuItem("New", ""))
			{
				ndAssert(0);
				//menuSelection = m_new;
			}

			if (ImGui::MenuItem("Load", ""))
			{
				char fileName[2048];
				if (dGetLoadNdFileName(fileName, sizeof(fileName) - 1))
				{
					m_currentPath = ndString(fileName);
					ndRenderMeshLoader loader(*m_renderer);
					loader.LoadMesh(m_currentPath);
					m_model = loader.m_mesh;
				}
			}

			if (ImGui::MenuItem("Save", ""))
			{
				ndAssert(0);
				//char fileName[2048];
				//if (dGetLoadNdFileName(fileName, sizeof(fileName) - 1))
				//{
				//	m_currentPath = ndString(fileName);
				//	ndRenderMeshLoader loader(*m_renderer);
				//	loader.LoadMesh(m_currentPath);
				//}
			}

			if (ImGui::MenuItem("Save As ...", ""))
			{
				ndAssert(0);
				//char fileName[2048];
				//if (dGetLoadNdFileName(fileName, sizeof(fileName) - 1))
				//{
				//	m_currentPath = ndString(fileName);
				//	ndRenderMeshLoader loader(*m_renderer);
				//	loader.LoadMesh(m_currentPath);
				//}
			}

			if (ImGui::MenuItem("Exit", ""))
			{
				m_renderer->Terminate();
			}

			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Run", !m_runScene))
		{
			m_runScene = true;
			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("Stop", m_runScene))
		{
			m_runScene = false;
			ImGui::EndMenu();
		}

		ImGui::EndMainMenuBar();
	}
}


void ndAssetEditor::ShowOutlierToolBar()
{
	// draw some bottom to control the explorer
	ImGui::Button("undo");
	ImGui::SameLine();
	ImGui::Button("redo");
}

void ndAssetEditor::ShowOutlierExplorer(ndMesh* const root)
{

	// draw explorer tree.
//if (ImGui::TreeNode("Mesh Model"))
//{
//	//IMGUI_DEMO_MARKER("Widgets/Trees/Basic trees");
//	if (ImGui::TreeNode("Basic trees"))
//	{
//		for (int i = 0; i < 5; i++)
//		{
//			// Use SetNextItemOpen() so set the default state of a node to be open. We could
//			// also use TreeNodeEx() with the ImGuiTreeNodeFlags_DefaultOpen flag to achieve the same thing!
//			if (i == 0)
//				ImGui::SetNextItemOpen(true, ImGuiCond_Once);
//
//			if (ImGui::TreeNode((void*)(intptr_t)i, "Child %d", i))
//			{
//				ImGui::Text("blah blah");
//				ImGui::SameLine();
//				if (ImGui::SmallButton("button")) {}
//				ImGui::TreePop();
//			}
//		}
//		ImGui::TreePop();
//	}
//
//	//IMGUI_DEMO_MARKER("Widgets/Trees/Advanced, with Selectable nodes");
//	if (ImGui::TreeNode("Advanced, with Selectable nodes"))
//	{
//		//HelpMarker("This is a more typical looking tree with selectable nodes.\n"
//		//	"Click to select, CTRL+Click to toggle, click on arrows or double-click to open.");
//		static ImGuiTreeNodeFlags base_flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick | ImGuiTreeNodeFlags_SpanAvailWidth;
//		static bool align_label_with_current_x_position = false;
//		static bool test_drag_and_drop = false;
//		ImGui::CheckboxFlags("ImGuiTreeNodeFlags_OpenOnArrow", &base_flags, ImGuiTreeNodeFlags_OpenOnArrow);
//		ImGui::CheckboxFlags("ImGuiTreeNodeFlags_OpenOnDoubleClick", &base_flags, ImGuiTreeNodeFlags_OpenOnDoubleClick);
//		//ImGui::CheckboxFlags("ImGuiTreeNodeFlags_SpanAvailWidth", &base_flags, ImGuiTreeNodeFlags_SpanAvailWidth); ImGui::SameLine(); HelpMarker("Extend hit area to all available width instead of allowing more items to be laid out after the node.");
//		ImGui::CheckboxFlags("ImGuiTreeNodeFlags_SpanFullWidth", &base_flags, ImGuiTreeNodeFlags_SpanFullWidth);
//		ImGui::Checkbox("Align label with current X position", &align_label_with_current_x_position);
//		ImGui::Checkbox("Test tree node as drag source", &test_drag_and_drop);
//		ImGui::Text("Hello!");
//		if (align_label_with_current_x_position)
//			ImGui::Unindent(ImGui::GetTreeNodeToLabelSpacing());
//
//		// 'selection_mask' is dumb representation of what may be user-side selection state.
//		//  You may retain selection state inside or outside your objects in whatever format you see fit.
//		// 'node_clicked' is temporary storage of what node we have clicked to process selection at the end
//		/// of the loop. May be a pointer to your own node type, etc.
//		static int selection_mask = (1 << 2);
//		int node_clicked = -1;
//		for (int i = 0; i < 6; i++)
//		{
//			// Disable the default "open on single-click behavior" + set Selected flag according to our selection.
//			// To alter selection we use IsItemClicked() && !IsItemToggledOpen(), so clicking on an arrow doesn't alter selection.
//			ImGuiTreeNodeFlags node_flags = base_flags;
//			const bool is_selected = (selection_mask & (1 << i)) != 0;
//			if (is_selected)
//				node_flags |= ImGuiTreeNodeFlags_Selected;
//			if (i < 3)
//			{
//				// Items 0..2 are Tree Node
//				bool node_open = ImGui::TreeNodeEx((void*)(intptr_t)i, node_flags, "Selectable Node %d", i);
//				if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
//					node_clicked = i;
//				if (test_drag_and_drop && ImGui::BeginDragDropSource())
//				{
//					ImGui::SetDragDropPayload("_TREENODE", NULL, 0);
//					ImGui::Text("This is a drag and drop source");
//					ImGui::EndDragDropSource();
//				}
//				if (node_open)
//				{
//					ImGui::BulletText("Blah blah\nBlah Blah");
//					ImGui::TreePop();
//				}
//			}
//			else
//			{
//				// Items 3..5 are Tree Leaves
//				// The only reason we use TreeNode at all is to allow selection of the leaf. Otherwise we can
//				// use BulletText() or advance the cursor by GetTreeNodeToLabelSpacing() and call Text().
//				node_flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen; // ImGuiTreeNodeFlags_Bullet
//				ImGui::TreeNodeEx((void*)(intptr_t)i, node_flags, "Selectable Leaf %d", i);
//				if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
//					node_clicked = i;
//				if (test_drag_and_drop && ImGui::BeginDragDropSource())
//				{
//					ImGui::SetDragDropPayload("_TREENODE", NULL, 0);
//					ImGui::Text("This is a drag and drop source");
//					ImGui::EndDragDropSource();
//				}
//			}
//		}
//		if (node_clicked != -1)
//		{
//			// Update selection state
//			// (process outside of tree loop to avoid visual inconsistencies during the clicking frame)
//			if (ImGui::GetIO().KeyCtrl)
//				selection_mask ^= (1 << node_clicked);          // CTRL+click to toggle
//			else //if (!(selection_mask & (1 << node_clicked))) // Depending on selection behavior you want, may want to preserve selection when clicking on item that is part of the selection
//				selection_mask = (1 << node_clicked);           // Click to single-select
//		}
//		if (align_label_with_current_x_position)
//			ImGui::Indent(ImGui::GetTreeNodeToLabelSpacing());
//		ImGui::TreePop();
//	}
//	ImGui::TreePop();
//}

	ndInt32 options = 0;
	//options |= ImGuiTreeNodeFlags_SpanAvailWidth;
	options |= ImGuiTreeNodeFlags_DefaultOpen;
	//options |= ImGuiTreeNodeFlags_Bullet;
	//options |= ImGuiTreeNodeFlags_Leaf;

	if (ImGui::TreeNodeEx(root->GetName().GetStr(), options))
	{
		const ndList<ndSharedPtr<ndMesh>>& children = root->GetChildren();
		for (ndList<ndSharedPtr<ndMesh>>::ndNode* child = children.GetFirst(); child; child = child->GetNext())
		{
			ShowOutlierExplorer(*child->GetInfo());
		}
		ImGui::TreePop();
	}
}

void ndAssetEditor::ShowOutlier()
{
	bool* open = false;

	ndInt32 flags = 0;
	ImGui::Begin("Oulier", open, flags);
	ImGui::PushItemWidth(ImGui::GetFontSize() * -12);

	ShowOutlierToolBar();
	if (*m_model)
	{
		ShowOutlierExplorer(* m_model);
	}

	ImGui::End();
}