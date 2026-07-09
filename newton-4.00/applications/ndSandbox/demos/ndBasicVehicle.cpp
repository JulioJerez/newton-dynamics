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

#include "ndSandboxStdafx.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndVehicleCommon.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndGameControllerInputs.h"
#include "ndDemoCameraNodeFollow.h"
#include "ndHeightFieldPrimitive.h"
#include "ndDemoCameraNodeLookAtTarget.h"

namespace ndMotorVehicle
{
	class ndPlacementMatrix : public ndMatrix
	{
		public:
		ndPlacementMatrix(const ndMatrix base, const ndVector& offset)
			:ndMatrix(base)
		{
			m_posit += offset;
		}
	};

	class ndVehicleEngineSound : public ndSoundSourceNotify
	{
		public:
		ndVehicleEngineSound(ndMultiBodyVehicle* const vehicle)
			:ndSoundSourceNotify()
			,m_vehicle(vehicle)
		{
		}

		virtual void Update(ndSoundSource* const source)
		{
			ndMultiBodyVehicleMotor* const motor = m_vehicle->GetMotor();

			ndFloat32 rpm = ndAbs(motor->GetRpm());
			ndVehicleCommonNotify* const controller = (ndVehicleCommonNotify*)*m_vehicle->GetNotifyCallback();
			if (controller->EngineOn())
			{
				rpm = ndMax (ndFloat32(20.0f), rpm);
			}
			if (rpm < ndFloat32(10.0f))
			{
				source->Stop();
			}
			else if (!source->IsPlaying())
			{
				source->Play();
			}
			else
			{
				const ndBodyKinematic* const chassis = m_vehicle->GetRoot()->m_body->GetAsBodyKinematic();
				source->SetVelocity(chassis->GetVelocity());
				source->SetPosition(chassis->GetMatrix().m_posit);

				const ndMultiBodyVehicleMotor::ndEngineTorqueCurve& torqueCurve = motor->GetCurve();
				ndFloat32 ideRpm = torqueCurve.GetIdleRpm();
				ndFloat32 maxRpm = torqueCurve.GetPickPowerRpm();
				ndFloat32 pitch = ndFloat32(1.0f) + (rpm - ideRpm) / (maxRpm - ideRpm);
				source->SetPitch(ndClamp(pitch, ndFloat32(1.0f), ndFloat32(2.0f)));
			}
		}

		ndWeakPtr<ndMultiBodyVehicle> m_vehicle;
	};

	class ndVehicleController : public ndVehicleCommonNotify
	{
		public:
		ndVehicleController(ndMultiBodyVehicle* const vehicle, ndSharedPtr<ndSoundSource> engineSound)
			:ndVehicleCommonNotify(vehicle)
			,m_engineSound(engineSound)
		{
		}

		ndSharedPtr<ndSoundSource> m_engineSound;
	};

	ndSharedPtr<ndModel> CreateBasicVehicle(ndDemoEntityManager* const scene, const char* const modelName, const ndMatrix& matrix)
	{
		ndMeshLoader loader;
		loader.LoadMesh(ndGetWorkingFileName(modelName));
		const ndMesh* const mesh = *loader.m_mesh;

		ndPhysicsWorld* const world = scene->GetWorld();

		// we first load the model as like any other arcilation
		ndSharedPtr<ndModel> vehicleModel(new ndMultiBodyVehicle());
		ndMultiBodyVehicle* const vehicle = vehicleModel->GetAsMultiBodyVehicle();
		vehicle->Deserialize(mesh);

		// then, we convet the mode to a multibody vehicle.
		vehicle->ConvertToMotorVehicle();

		ndRender* const renderer = *scene->GetRenderer();
		ndSharedPtr<ndRenderSceneNode> sceneMesh(ndRenderMeshLoader::CreateRenderSceneMesh(renderer, *loader.m_mesh, ndGetWorkingFileName("")));

		ndSharedPtr<ndSoundSource> engineSound;
		auto BindApplicationData = [scene, mesh, vehicle, &sceneMesh, &engineSound](ndModelArticulation::ndNode* const node)
		{
			if (vehicle->IsCloseLoop(node))
			{
				ndTrace(("do something\n"));
			}
			else
			{
				const ndMesh* const meshNode = mesh->FindByClosestMatch(node->m_name);
				ndAssert(meshNode);

				if (meshNode->GetName() == "engine")
				{
					ndMeshCustomPropertyString* const property = (ndMeshCustomPropertyString*)meshNode->GetCustomPropertyByName("engineSound");
					if (property)
					{
						engineSound = scene->GetSoundManager()->AddSound(property->m_value.GetStr());
						engineSound->SetLooping(true);

						ndSharedPtr<ndSoundSourceNotify> notify(new ndVehicleEngineSound(vehicle));
						engineSound->SetNotify(notify);
					}
				}

				// find the visual node this body control by name. 
				const ndMatrix matrix(node->m_body->GetMatrix());
				ndRenderSceneNode* const visualEntityPtr = sceneMesh->FindByClosestMatch(meshNode->GetName());
				ndAssert(visualEntityPtr);
				ndSharedPtr<ndRenderSceneNode> visualEntity((visualEntityPtr == *sceneMesh) ? sceneMesh : visualEntityPtr->GetSharedPtr());

				// add a rigid body with notification callback
				ndBodyKinematic* const parentBody = node->GetParent() ? node->GetParent()->m_body->GetAsBodyKinematic() : nullptr;
				ndSharedPtr<ndBodyNotify> notify(new ndDemoEntityNotify(scene, visualEntity, parentBody));
				if (node->m_joint)
				{
					if ((strcmp(node->m_joint->ClassName(), ndMultiBodyVehicleMotor::StaticClassName()) == 0) ||
						(strcmp(node->m_joint->ClassName(), ndMultiBodyVehicleDifferential::StaticClassName()) == 0) ||
						(strcmp(node->m_joint->ClassName(), ndMultiBodyVehicleTireJoint::StaticClassName()) == 0))
					{
						// fast moving wheel
						ndDemoEntityNotify* const fastNotify = (ndDemoEntityNotify*)*notify;
						fastNotify->m_capOmega = ndFloat32(10000.0f);
					}
					if ((strcmp(node->m_joint->ClassName(), ndMultiBodyVehicleTireJoint::StaticClassName()) == 0))
					{
						////costumize friction model if desired
						////plot the curve to check it is a value form
						//ndMultiBodyVehicleTireJoint* const joint = (ndMultiBodyVehicleTireJoint*)*node->m_joint;
						//ndTireFrictionModel frictionMode(joint->GetFrictionModel());
						//frictionMode.PlotPacejkaCurves(node->m_name.GetStr());
					}
					if ((strcmp(node->m_joint->ClassName(), ndMultiBodyVehicleMotor::StaticClassName()) == 0))
					{
						//override the default trque rpm curve, if desired
						//ndMultiBodyVehicleMotor* const motor = (ndMultiBodyVehicleMotor*)*node->m_joint;
						//motor->SetCurve(superCar.m_curve);
					}
				}
				node->m_body->SetNotifyCallback(notify);
			}
		};
		vehicle->NodeIterator(BindApplicationData);

		// add a third person camera
		ndSharedPtr<ndRenderSceneNode> camera(nullptr);
		ndRenderSceneNode* const cameraPivotNode = sceneMesh->FindByName("cameraPivot");
		if (cameraPivotNode)
		{
			ndVector cameraPivot(ndVector::m_zero);
			const ndMesh* const cameraPivotMesh = loader.m_mesh->FindByName("cameraPivot");
			ndAssert(cameraPivotMesh);
			ndMeshCustomPropertyFloat* const property = (ndMeshCustomPropertyFloat*)cameraPivotMesh->GetCustomPropertyByName("cameraDistance");
			ndFloat32 dist = property ? -ndAbs(property->m_value) : ndFloat32(-5.0f);
			camera = ndSharedPtr<ndRenderSceneNode>(new ndDemoCameraNodeFollow(renderer, cameraPivot, dist));
			cameraPivotNode->AddChild(camera);
		}

		// a more explicit way to add the visual modifier
		auto AddGraphicsModiers = [sceneMesh](ndMesh* const node)
		{
			ndSharedPtr<ndMeshTransformModifier> modifier(node->GetModifier());
			if (modifier)
			{
				ndAssert(modifier->m_owner);
				ndAssert(modifier->m_target);
				ndRenderSceneNode* const owner = sceneMesh->FindByName(modifier->m_owner->GetName());
				ndRenderSceneNode* const target = sceneMesh->FindByName(modifier->m_target->GetName());
				ndAssert(owner);
				ndAssert(target);

				if (strcmp(modifier->ClassName(), ndMeshTransformModifierLookAt::StaticClassName()) == 0)
				{
					ndSharedPtr<ndRenderTransformModifier> renderModifier(new ndRenderTransformModifierLookAtNode(owner, target));
					owner->SetTransformModifier(renderModifier);
				}
				else if (strcmp(modifier->ClassName(), ndMeshTransformModifierTwoLinksIK::StaticClassName()) == 0)
				{
					const ndMeshTransformModifierTwoLinksIK* const modifierIk = (ndMeshTransformModifierTwoLinksIK*)*modifier;
					ndRenderSceneNode* const link = sceneMesh->FindByName(modifierIk->m_childLink->GetName());
					ndAssert(link);
					ndSharedPtr<ndRenderTransformModifier> renderModifier(new ndRenderTransformModifierTwoLinksIK(owner, link, target, modifierIk->m_solutionSign));
					owner->SetTransformModifier(renderModifier);
				}
				else
				{
					ndAssert(0);
				}
			}
		};
		loader.m_mesh->NodeIterator(AddGraphicsModiers);


		//add the notification for binding to the application.
		ndSharedPtr<ndModelNotify> controller(new ndVehicleController(vehicle, engineSound));
		vehicle->SetNotifyCallback(controller);

		scene->AddEntity(sceneMesh);
		world->AddModel(vehicleModel);

		vehicle->SetTransform(matrix);
		sceneMesh->SetTransform(matrix);
		sceneMesh->SetTransform(matrix);
		return vehicleModel;
	}

	class ndDashboard : public ndDemoEntityManager::ndDemoUIpanel
	{
		public:
		ndDashboard()
			:ndDemoUIpanel()
			,m_vehicle(nullptr)
		{
		}

		virtual void Update(ndDemoEntityManager* const) override
		{
			if (m_vehicle)
			{
				auto DrawDial = [](ndReal originX, ndReal originY, ndReal radius, ndReal value, ndReal range, ndUnsigned32 color)
				{
					ImVec2 canvas_pos = ImGui::GetCursorScreenPos();

					// Calculate a dynamic center point offset from your canvas space
					ImVec2 dynamic_center = ImVec2(canvas_pos.x + originX, canvas_pos.y + originY);

					// Safely draw your shape relative to the layout window
					ndInt32 plateColor = 64;
					ImDrawList* const drawList = ImGui::GetWindowDrawList();
					drawList->AddCircleFilled(dynamic_center, radius, IM_COL32(plateColor, plateColor, plateColor, 255), 0);

					ImVec2 needle[] =
					{
						{0.0f, 0.1f},
						{-0.1f, 0.0f},
						{0.0f, -0.1f},
						{1.0f, 0.0f},
						{0.0f, 0.1f},
					};

					ndFloat32 scale = radius * 0.9f;
					const ndInt32 size = sizeof(needle) / sizeof(needle[0]);

					ndFloat32 angleRange = 300.0f;
					ndFloat32 initialAngle = -240.0f;
					ndFloat32 angle = ndDegreeToRad * (initialAngle + value * angleRange / range);
					ndReal sinAngle = ndReal(ndSin(angle));
					ndReal cosAngle = ndReal(ndCos(angle));

					for (ndInt32 i = 0; i < size; ++i)
					{
						ndReal x = needle[i][0] * cosAngle - needle[i][1] * sinAngle;
						ndReal y = needle[i][0] * sinAngle + needle[i][1] * cosAngle;
						needle[i][0] = ndReal(x * scale + dynamic_center.x);
						needle[i][1] = ndReal(y * scale + dynamic_center.y);
					}
					drawList->AddConvexPolyFilled(needle, size, color);
				};

				ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
				const ndMultiBodyVehicle* const vehicle = m_vehicle->GetAsMultiBodyVehicle();
				const ndMultiBodyVehicleMotor* const motor = vehicle->GetMotor();

				// draw engine rpm
				ndReal rpm = ndReal(motor->GetRpm());
				ImGui::Text("  rmp %04d", ndInt32 (rpm));
				DrawDial(60.0f, 50.0f, 50.0f, rpm, ndReal(motor->GetMaxRpm()), IM_COL32(180, 0, 0, 255));

				ImGui::SameLine();
				ndReal speed = ndReal(vehicle->GetSpeed() * 3.6f);
				ImGui::Text("  kmh %03d", ndInt32(speed));
				DrawDial(160.0f, 50.0f, 50.0f, speed, ndReal(motor->GetTopSpeed() * 3.6f), IM_COL32(180, 180, 0, 255));

				const ndSharedPtr<ndModelNotify>& notify = vehicle->GetNotifyCallback();
				const ndVehicleCommonNotify* const controller = (ndVehicleCommonNotify*)*notify;

				ImGui::NewLine();
				ImGui::NewLine();
				ImGui::NewLine();
				ImGui::NewLine();
				ImGui::NewLine();

				if (controller->m_transmission == ndVehicleCommonNotify::m_manual)
				{
					ImGui::Text("transmission: manual");
				}
				else
				{
					ImGui::Text("transmission: automatic");
				}

				if (controller->m_driverState == ndVehicleCommonNotify::m_parked)
				{
					ImGui::Text("current gear: parked");
				}
				else
				{
					switch (controller->m_currentGear)
					{
						case ndMultiBodyVehicleGearBox::ndGearBox::m_revertGear:
						{
							ImGui::Text("current gear: reverse");
							break;
						}

						case ndMultiBodyVehicleGearBox::ndGearBox::m_neutralGear:
						{
							ImGui::Text("current gear: neutral");
							break;
						}

						case ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear:
						{
							ImGui::Text("current gear: first");
							break;
						}

						case ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear + 1:
						{
							ImGui::Text("current gear: secund");
							break;
						}

						case ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear + 2:
						{
							ImGui::Text("current gear: third");
							break;
						}

						case ndMultiBodyVehicleGearBox::ndGearBox::m_firstGear + 3:
						{
							ImGui::Text("current gear: fourth");
							break;
						}

						default:;
							ndAssert(0);
					}
				}
			}
		}

		ndSharedPtr<ndModel> m_vehicle;
	};

	class SceneNavigation : public ndDemoEntityManager::OnPostUpdate
	{
		public:
		SceneNavigation(ndSharedPtr<ndDemoEntityManager::ndDemoUIpanel>& dashboard)
			:OnPostUpdate()
			,m_dashboard(*dashboard)
			,m_changePlayer()
		{
		}

		void OnDebug(ndDemoEntityManager* const, bool) override
		{
		}

		virtual void Update(ndDemoEntityManager* const manager, ndFloat32) override
		{
			// driver moves to a diffrent vehicle
			const ndFixSizeArray<bool, 32>& buttons = manager->GetGameController()->GetButtons();
			if (m_changePlayer.Update(buttons[ndGameControllerInputs::m_changePlayer] ? true : false))
			{
				ndFixSizeArray<ndModelList::ndNode*, 256> vehicleArray;
				const ndModelList& models = manager->GetWorld()->GetModelList();
			
				// get the vehicle array
				ndInt32 currentPlayerIndex = 0;
				for (ndModelList::ndNode* node = models.GetFirst(); node; node = node->GetNext())
				{
					ndMultiBodyVehicle* const vehicle = node->GetInfo()->GetAsMultiBodyVehicle();
					if (vehicle)
					{
						ndVehicleCommonNotify* const modelNotify = (ndVehicleCommonNotify*)*vehicle->GetNotifyCallback();
						if (modelNotify->GetPlayerState())
						{
							currentPlayerIndex = vehicleArray.GetCount();
						}
						vehicleArray.PushBack(node);

						ndSharedPtr<ndBodyNotify>& notify = vehicle->GetRoot()->m_body->GetNotifyCallback();
						ndDemoEntityNotify* const vehicleNotify = (ndDemoEntityNotify*)*notify;
						ndRenderSceneNode* const visualNode = *vehicleNotify->m_entity;
						ndRenderSceneCamera* const cameraNode = visualNode->FindCameraNode();
						cameraNode->SetActiveState(false);
					}
				}
			
				// make this vehicle inactive
				ndMultiBodyVehicle* const currentVehicle = vehicleArray[currentPlayerIndex]->GetInfo()->GetAsMultiBodyVehicle();
				ndVehicleCommonNotify* const currentModelNotify = (ndVehicleCommonNotify*)*currentVehicle->GetNotifyCallback();
				currentModelNotify->SetAsPlayer(false);

				// activate next vehicle
				ndInt32 nextPlayerIndex = (currentPlayerIndex + 1) % vehicleArray.GetCount();
				ndMultiBodyVehicle* const vehicle = vehicleArray[nextPlayerIndex]->GetInfo()->GetAsMultiBodyVehicle();
				ndVehicleCommonNotify* const modelNotify = (ndVehicleCommonNotify*)*vehicle->GetNotifyCallback();
				modelNotify->SetAsPlayer(true);

				// asign camera to the vehicle
				ndSharedPtr<ndBodyNotify>& notify = vehicle->GetRoot()->m_body->GetNotifyCallback();
				ndAssert(strcmp(notify->ClassName(), ndDemoEntityNotify::StaticClassName()) == 0);
				ndDemoEntityNotify* const vehicleNotify = (ndDemoEntityNotify*)*notify;
				ndRenderSceneNode* const visualNode = *vehicleNotify->m_entity;
				ndRenderSceneNode* const cameraNode = visualNode->FindByName("__PlayerCamera__");
				ndSharedPtr<ndRenderSceneNode> cameraPtr(cameraNode->GetSharedPtr());
				manager->GetWorld()->SetCamera(cameraPtr);

				// activae this camera
				ndRenderSceneCamera* const renderCameraNode = cameraNode->FindCameraNode();
				renderCameraNode->SetActiveState(true);

				// set this vehicle as the target
				ndDemoCameraNodeLookAtTarget* const lookAtCamera = (ndDemoCameraNodeLookAtTarget*)*manager->GetLookAtCamera();
				lookAtCamera->SetTarget(vehicleNotify->m_entity);

				// set the dashboard 
				ndDashboard* const dashboard = (ndDashboard*)*m_dashboard;
				dashboard->m_vehicle = vehicleArray[nextPlayerIndex]->GetInfo();
			}
		}

		ndWeakPtr<ndDemoEntityManager::ndDemoUIpanel> m_dashboard;
		ndDemoEntityManager::ndKeyTrigger m_changePlayer;
	};

	//void AddMaterial(ndDemoEntityManager* const scene)
	void AddMaterial(ndDemoEntityManager* const)
	{
		//ndVehicleMaterial material;
		//material.m_restitution = 0.1f;
		//material.m_staticFriction0 = 0.8f;
		//material.m_staticFriction1 = 0.8f;
		//material.m_dynamicFriction0 = 0.8f;
		//material.m_dynamicFriction1 = 0.8f;
		//
		//ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
		//callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_default);
		//callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_modelPart);
		//callback->RegisterMaterial(material, ndDemoContactCallback::m_vehicleTirePart, ndDemoContactCallback::m_modelPart);
		//callback->RegisterMaterial(material, ndDemoContactCallback::m_vehicleTirePart, ndDemoContactCallback::m_vehicleTirePart);
		//callback->RegisterMaterial(material, ndDemoContactCallback::m_vehicleTirePart, ndDemoContactCallback::m_default);
	}

	void LoadMap(ndDemoEntityManager* const scene)
	{
		// get the file full path
		ndPhysicsWorld* const world = scene->GetWorld();
		const ndString fileName(ndGetWorkingFileName("racetrack/racetrack.nd"));

		// load the mesh
		ndMeshLoader loader;
		loader.LoadMesh(fileName);

		// set all teh alpha test materials
		auto SetAlphaTest = [](ndMesh* const node)
		{
			ndMeshEffect* const	geometry = *node->GetGeometry();
			if (geometry)
			{
				if (node->GetName().Find("Bush") != -1)
				{
					ndArray<ndMeshEffect::ndMaterial>& materialArray = geometry->GetMaterials();
					for (ndInt32 i = 0; i < materialArray.GetCount(); ++i)
					{
						ndMeshEffect::ndMaterial& material = materialArray[i];
						material.m_useAlphaTest = true;
					}
				}
				else if (node->GetParent()->GetName().Find("Pine") != -1)
				{
					ndArray<ndMeshEffect::ndMaterial>& materialArray = geometry->GetMaterials();
					for (ndInt32 i = 0; i < materialArray.GetCount(); ++i)
					{
						ndMeshEffect::ndMaterial& material = materialArray[i];
						material.m_useAlphaTest = true;
					}
				}
			}

			// make sure props are invisible whne rendering the map
			ndMeshBodyDynamic* const propMesh = (ndMeshBodyDynamic*)*node->GetRigidBody();
			if (propMesh)
			{
				if (propMesh->m_invMass.m_w > 0.0f)
				{
					// make sure we hide this node
					node->SetVisibility(false);
					ndList<ndSharedPtr<ndMesh>>& children = node->GetChildren();
					for (ndList<ndSharedPtr<ndMesh>>::ndNode* child = children.GetFirst(); child; child = child->GetNext())
					{
						child->GetInfo()->SetVisibility(false);
					}
				}
			}
		};
		loader.m_mesh->NodeIterator(SetAlphaTest);

		// generate the scene rigid body
		ndSharedPtr<ndBody> bodyFloor(new ndBodyDynamic());
		bodyFloor->Deserialize(*loader.m_mesh->GetRigidBody());

		// genereta the visual mesh
		ndRender* const renderer = *scene->GetRenderer();
		const ndString materialPath(fileName.GetPath());
		ndSharedPtr<ndRenderSceneNode> sceneMesh(ndRenderMeshLoader::CreateRenderSceneMesh(renderer, *loader.m_mesh, materialPath));

		// bind scene and physics with a rb notification 
		ndSharedPtr<ndBodyNotify> notify(new ndDemoEntityNotify(scene, sceneMesh, nullptr));
		bodyFloor->SetNotifyCallback(notify);

		// add rb and visual mesh to the world and visual scene
		scene->AddEntity(sceneMesh);
		world->AddBody(bodyFloor);

		// now add all of dynamics props
		auto AddProps = [world, scene, &fileName](ndMesh* const node)
		{
			ndMeshBodyDynamic* const propMesh = (ndMeshBodyDynamic*)*node->GetRigidBody();
			if (propMesh)
			{
				if (propMesh->m_invMass.m_w > 0.0f)
				{
					// make sure it is visible before convert to visual mesh
					node->SetVisibility(true);
					ndList<ndSharedPtr<ndMesh>>& children = node->GetChildren();
					for (ndList<ndSharedPtr<ndMesh>>::ndNode* child = children.GetFirst(); child; child = child->GetNext())
					{
						child->GetInfo()->SetVisibility(true);
					}

					// make a prop rigid body
					ndSharedPtr<ndBody> propBody(new ndBodyDynamic());
					propBody->Deserialize(propMesh);
					const ndMatrix matrix(node->CalculateGlobalMatrix());
					propBody->SetMatrix(matrix);

					// make a prop visual mesh
					ndRender* const renderer = *scene->GetRenderer();
					const ndString materialPath(fileName.GetPath());
					ndSharedPtr<ndRenderSceneNode> visualPropMesh(ndRenderMeshLoader::CreateRenderSceneMesh(renderer, node, materialPath));

					// bind notification and add to world and scene
					ndSharedPtr<ndBodyNotify> notify(new ndDemoEntityNotify(scene, visualPropMesh, nullptr));
					propBody->SetNotifyCallback(notify);

					world->AddBody(propBody);
					scene->AddEntity(visualPropMesh);
				}
			}
		};
		loader.m_mesh->NodeIterator(AddProps);
	}
};

using namespace ndMotorVehicle;

void ndBasicVehicle (ndDemoEntityManager* const scene)
{
	LoadMap(scene);
	//BuildPlayground(scene);
	//BuildFloorBox(scene, ndGetIdentityMatrix(), "marblecheckboard.png", 0.1f, true);

	ndPhysicsWorld* const world = scene->GetWorld();
	ndVector location(0.0f, 2.0f, 0.0f, 1.0f);

	// add a vehicle material
	AddMaterial(scene);
	
	ndMatrix matrix(ndYawMatrix (ndFloat32 (90.0f) * ndDegreeToRad));
	ndVector floor(FindFloor(*world, location, 100.0f));
	matrix.m_posit = floor;
	matrix.m_posit.m_y += 0.5f;
	
	//ndSharedPtr<ndModel> vehicle0(CreateBasicVehicle(scene, "testarossaMultiBody.nd", ndPlacementMatrix(matrix, ndVector(0.0f, 0.0f, -10.0f, 0.0f))));
	//ndSharedPtr<ndModel> vehicle1(CreateBasicVehicle(scene, "pickupTruck.nd", ndPlacementMatrix(matrix, ndVector(0.0f, 0.0f, -5.0f, 0.0f))));
	ndSharedPtr<ndModel> vehicle2(CreateBasicVehicle(scene, "truck.nd", ndPlacementMatrix(matrix, ndVector(0.0f, 1.0f, 0.0f, 0.0f))));
	ndSharedPtr<ndModel> vehicle3(CreateBasicVehicle(scene, "lav-25.nd", ndPlacementMatrix(matrix, ndVector(-4.0f, 1.0f, 4.0f, 0.0f))));
	ndSharedPtr<ndModel> vehicle4(CreateBasicVehicle(scene, "tractor.nd", ndPlacementMatrix(matrix, ndVector(12.0f, 1.0f, 6.0f, 0.0f))));
	
	//matrix.m_posit.m_x += 40.0f;
	//matrix.m_posit.m_z += 5.0f;
	//AddPlanks(scene, matrix, 60.0f, 5);

	// set a ui paner to see vehicle state
	ndSharedPtr<ndDemoEntityManager::ndDemoUIpanel> dashboard(new ndDashboard());
	scene->SetDemoUIpanel(dashboard);

	// add a scene navigation post update
	ndSharedPtr<ndDemoEntityManager::OnPostUpdate> driver(new SceneNavigation(dashboard));
	scene->RegisterPostUpdate(driver);

	ndQuaternion rot;
	ndVector origin(-10.0f, 2.0f, -0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
