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
#include "ndMenuRenderPass.h"
#include "ndEditorCameraFlyby.h"
#include "ndHighResolutionTimer.h"
#include "ndDebugDisplayRenderPass.h"

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
	,m_currentPath("")
	,m_runScene(false)
	,m_showPivot(true)
	,m_showSelectedNode(true)
	,m_showCollisionShape(false)
	,m_renderMode(m_shaded)
	,m_gizmosScale(0.5f)
{
	// Setup window
	char title[256];

	ndSharedPtr<ndRender::ndUserCallback> callbacks(new ndRenderCallback(this));
	snprintf(title, sizeof(title), "Newton Asset Editor %d.%.2i", D_NEWTON_ENGINE_MAJOR_VERSION, D_NEWTON_ENGINE_MINOR_VERSION);
	m_renderer = ndSharedPtr<ndRender>(new ndRender(callbacks, 1280, 768, title));

	const ndString fontPathName(ndGetWorkingFileName("Cousine-Regular.ttf"));
	m_renderer->InitImGui(fontPathName.GetStr());

	// load the environment texture
	ndFixSizeArray<ndString, 6> environmentTexturePath;
	environmentTexturePath.PushBack(ndGetWorkingFileName("gray.png"));
	environmentTexturePath.PushBack(ndGetWorkingFileName("gray.png"));
	environmentTexturePath.PushBack(ndGetWorkingFileName("gray.png"));
	environmentTexturePath.PushBack(ndGetWorkingFileName("gray.png"));
	environmentTexturePath.PushBack(ndGetWorkingFileName("gray.png"));
	environmentTexturePath.PushBack(ndGetWorkingFileName("gray.png"));
	m_environmentTexture = m_renderer->GetTextureCache()->GetCubeMap(environmentTexturePath);

	// create render passes
	m_menuRenderPass = ndSharedPtr<ndRenderPass>(new ndMenuRenderPass(this));
	m_colorRenderPass = ndSharedPtr<ndRenderPass>(new ndRenderPassColor(*m_renderer));
	m_shadowRenderPass = ndSharedPtr<ndRenderPass>(new ndRenderPassShadows(*m_renderer));
	m_debugDisplayRenderPass = ndSharedPtr<ndRenderPass>(new ndDebugDisplayRenderPass(this));
	m_environmentRenderPass = ndSharedPtr<ndRenderPass>(new ndRenderPassEnvironment(*m_renderer, m_environmentTexture));

	// add render passes in order of execution
	m_renderer->AddRenderPass(m_shadowRenderPass);
	m_renderer->AddRenderPass(m_colorRenderPass);
	m_renderer->AddRenderPass(m_environmentRenderPass);
	m_renderer->AddRenderPass(m_debugDisplayRenderPass);
	m_renderer->AddRenderPass(m_menuRenderPass);
	
	//add main directional light
	m_renderer->SetSunLight(ndVector(-1.0f, 1.0f, 0.f, 0.0f), ndVector(0.7f, 0.7f, 0.7f, 0.0f));

	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

	m_defaultCamera = ndSharedPtr<ndRenderSceneNode>(new ndEditorCameraFlyby(*m_renderer));
	m_renderer->SetCamera(m_defaultCamera);
}

ndAssetEditor::~ndAssetEditor ()
{
}


ndSharedPtr<ndRender>& ndAssetEditor::GetRenderer()
{
	return m_renderer;
}

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

void ndAssetEditor::RegisterPostUpdate(const ndSharedPtr<OnPostUpdate>& postUpdate)
{
	m_onPostUpdate = postUpdate;
}

ndFloat32 ndAssetEditor::GetMouseWheel() const
{
	ndFloat32 wheel = ImGui::GetIO().MouseWheel;
	return wheel;
}

bool ndAssetEditor::GetMousePosition (ndFloat32& posX, ndFloat32& posY) const
{
	ImVec2 posit(ImGui::GetMousePos());
	posX = ndClamp(posit.x, ndReal(-1.0e10f), ndReal(1.0e10f));
	posY = ndClamp(posit.y, ndReal(-1.0e10f), ndReal(1.0e10f));
	return true;
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

void ndAssetEditor::SetCameraMatrix (const ndQuaternion& rotation, const ndVector& position)
{
	ndRenderSceneNode* const cameraNode = *m_renderer->GetCamera();
	cameraNode->SetTransform(rotation, position);
	cameraNode->SetTransform(rotation, position);
}

void ndAssetEditor::OnSubStepPostUpdate(ndFloat32)
{
}

void ndAssetEditor::UpdatePhysics(ndFloat32)
{
	if (m_runScene)
	{
		ndAssert(0);
		//// update the physics
		//if (m_world && !m_suspendPhysicsUpdate) 
		//{
		//	m_world->AdvanceTime(timestep);
		//}
	}
}

void ndAssetEditor::RenderScene()
{
	ndFloat32 timestep = ndGetElapsedSeconds();
	if (timestep > 1.0f / 60.0f)
	{
		timestep = 1.0f / 60.0f;
		ndResetTimer();
	}

	//UpdatePhysics(timestep);

	m_colorRenderPass->MakeActive(m_renderMode == m_shaded);

	ndEditorCameraNode* const camera = (ndEditorCameraNode*)*m_renderer->GetCamera();
	camera->TickUpdate(timestep);

	m_renderer->BegingRender();
	ConfigureDockSpace();

	m_renderer->Render();

	m_renderer->EndRender();
	m_renderer->Present();
}

void ndAssetEditor::RenderLayout()
{
	ShowMainMenuBar();
	ShowMainToolbar();
	ShowOutlierPanel();
	ShowPropertiesPanel();
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

void ndAssetEditor::ConfigureDockSpace()
{
	ImGuiWindowFlags dockspace_flags = ImGuiWindowFlags_None;
	dockspace_flags |= ImGuiWindowFlags_NoBackground;
	//dockspace_flags |= ImGuiWindowFlags_NoMove;
	//dockspace_flags |= ImGuiWindowFlags_NoResize;
	//dockspace_flags |= ImGuiWindowFlags_NoDocking;
	//dockspace_flags |= ImGuiWindowFlags_NoTitleBar;
	//dockspace_flags |= ImGuiWindowFlags_NoCollapse;
	//dockspace_flags |= ImGuiWindowFlags_NoNavFocus;
	dockspace_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus;

	ImGuiViewport* const viewport = ImGui::GetMainViewport();
	ndAssert(viewport);
	ImGui::SetNextWindowPos(viewport->Pos);
	ImGui::SetNextWindowSize(viewport->Size);
	ImGui::SetNextWindowViewport(viewport->ID);

	ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

	ImGui::Begin("dockFrame", nullptr, dockspace_flags);
	ImGui::PopStyleVar(3);

	ImGuiID dickSpaceId = ImGui::GetID("dockFrameDockSpace");
	ndAssert(dickSpaceId);

	ImGui::DockSpace(dickSpaceId, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode);
	ImGui::End();
}

void ndAssetEditor::ShowMainMenuBar()
{
	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu("File"))
		{
			if (ImGui::MenuItem("New", ""))
			{
				m_mesh = ndSharedPtr<ndMesh>(nullptr);
			}

			if (ImGui::MenuItem("Load", ""))
			{
				char fileName[2048];
				if (dGetLoadNdFileName(fileName, sizeof(fileName) - 1))
				{
					m_currentSelection = ndSharedPtr<ndMesh>(nullptr);
					m_currentPath = ndString(fileName);
					ndRenderMeshLoader loader(*m_renderer);
					loader.LoadMesh(m_currentPath);
					SetVisualScene(loader);
				}
			}

			if (ImGui::MenuItem("Save", ""))
			{
				if (*m_mesh)
				{
					m_currentSelection = ndSharedPtr<ndMesh>(nullptr);
					ndRenderMeshLoader loader(*m_renderer);
					loader.m_mesh = m_mesh;
					loader.SaveMesh(ndString(m_currentPath));
				}
			}

			if (ImGui::MenuItem("Save As ...", ""))
			{
				char fileName[2048];
				if (*m_mesh && dGetSaveNdFileName(fileName, sizeof(fileName) - 1))
				{
					m_currentSelection = ndSharedPtr<ndMesh>(nullptr);
					m_currentPath = ndString(fileName);
					ndRenderMeshLoader loader(*m_renderer);
					loader.m_mesh = m_mesh;
					loader.SaveMesh(ndString(m_currentPath));
				}
			}

			ImGui::Separator();
			if (ImGui::MenuItem("Import fbx", ""))
			{
				char fileName[2048];
				if (dGetImportFbxFileName(fileName, sizeof(fileName) - 1))
				{
					m_currentSelection = ndSharedPtr<ndMesh>(nullptr);
					m_currentPath = ndString(fileName);
					ndRenderMeshLoader loader(*m_renderer);
					loader.ImportFbx(m_currentPath);
					SetVisualScene(loader);
				}
			}

			ImGui::Separator();
			if (ImGui::MenuItem("Exit", ""))
			{
				m_currentSelection = ndSharedPtr<ndMesh>(nullptr);
				m_renderer->Terminate();
			}

			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Tools"))
		{
			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Options"))
		{
			ImGui::Text("render mode");
			ImGui::RadioButton("shaded", &m_renderMode, m_shaded);
			ImGui::RadioButton("wireframe", &m_renderMode, m_wireframe);
			ImGui::RadioButton("hidden Surface", &m_renderMode, m_hiddenSurface);

			ImGui::Separator();
			if (ImGui::DragFloat("gizmo scale", &m_gizmosScale, 0.1f))
			{
				m_gizmosScale = ndClamp(m_gizmosScale, ndReal(0.1f), ndReal(2.0f));
			}

			ImGui::Separator();
			ImGui::Checkbox("show node", &m_showSelectedNode);
			ImGui::Checkbox("show pivot", &m_showPivot);
			ImGui::Checkbox("show collision", &m_showCollisionShape);

			ImGui::EndMenu();
		}

		ImGui::EndMainMenuBar();
	}
}

void ndAssetEditor::SetVisualScene(const ndRenderMeshLoader& loader)
{
	m_undoRedo.Clear();
	m_newMesh = loader.m_mesh;
	m_newSceneMesh = loader.m_renderMesh;
}

void ndAssetEditor::Run()
{
	// Main loop
	ndFloatExceptions exception;
	while (!m_renderer->ShouldFinish())
	{
		if (m_renderer->PollEvents())
		{
			if (!*m_mesh && m_entity)
			{
				m_undoRedo.Clear();
				m_currentSelection = ndSharedPtr<ndMesh>(nullptr);
				m_renderer->RemoveSceneNode(m_entity);
				m_entity = ndSharedPtr<ndRenderSceneNode>(nullptr);
				m_debugDisplayRenderPass->ResetScene();
				m_model = ndSharedPtr<ndModel>(nullptr);
			}

			//if (*m_newMesh || *m_newSceneMesh)
			if (*m_newMesh)
			{
				ndAssert(*m_newSceneMesh);
				if (*m_entity)
				{
					m_renderer->RemoveSceneNode(m_entity);
				}

				m_mesh = m_newMesh;
				m_entity = m_newSceneMesh;
				m_model = ndSharedPtr<ndModel>(new ndModelArticulation());

				m_newMesh = ndSharedPtr<ndMesh>(nullptr);
				m_newSceneMesh = ndSharedPtr<ndRenderSceneNode>(nullptr);

				m_renderer->AddSceneNode(m_entity);
				m_debugDisplayRenderPass->ResetScene();

				ndVector p0;
				ndVector p1;
				const ndMatrix matrix(ndGetIdentityMatrix());
				m_mesh->CalculateAabb(matrix, p0, p1);
				ndVector size(ndVector::m_half * (p1 - p0));
				ndVector origin(ndVector::m_half * (p1 + p0));
				ndFloat32 maxSize = ndMax(ndMax(size.m_x, size.m_y), size.m_z);
				origin.m_x -= maxSize * 4.0f;
				ndQuaternion rot;

				SetCameraMatrix(rot, origin);
			}

			RenderScene();
		}
	}
}
