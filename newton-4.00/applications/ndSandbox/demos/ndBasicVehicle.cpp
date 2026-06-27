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

	class ndBasicVehicleDectriptor : public ndVehicleDectriptor
	{
		public:
		ndBasicVehicleDectriptor()
			:ndVehicleDectriptor()
			,m_curve()
		{
		}
		ndMultiBodyVehicleMotor::ndEngineTorqueCurve m_curve;
	};

	class ndVehicleDectriptorSuperCar : public ndBasicVehicleDectriptor
	{
		public:
		ndVehicleDectriptorSuperCar()
			:ndBasicVehicleDectriptor()
		{
			m_name = "supercar";
			ndFloat32 idleTorquePoundFoot = ndFloat32(300.0f);
			ndFloat32 idleRmp = ndFloat32(700.0f);
			ndFloat32 horsePower = ndFloat32(400.0f);
			ndFloat32 rpm0 = ndFloat32(5000.0f);
			ndFloat32 rpm1 = ndFloat32(6200.0f);
			ndFloat32 horsePowerAtRedLine = ndFloat32(100.0f);
			ndFloat32 redLineRpm = ndFloat32(8000.0f);
			m_curve.Init(idleTorquePoundFoot, idleRmp,
				horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);
		}

		ndMultiBodyVehicleMotor::ndEngineTorqueCurve m_curve;
	};

	ndSharedPtr<ndModel> CreateBasicVehicle(ndDemoEntityManager* const scene, const char* const modelName, const ndMatrix& matrix)
	{
		ndMeshLoader loader;
		loader.LoadMesh(ndGetWorkingFileName(modelName));
		const ndMesh* const mesh = *loader.m_mesh;

		ndPhysicsWorld* const world = scene->GetWorld();

		// we first load the model as like any other arcilated model
		ndSharedPtr<ndModel> vehicleModel(new ndMultiBodyVehicle());
		ndMultiBodyVehicle* const vehicle = vehicleModel->GetAsMultiBodyVehicle();
		vehicle->Deserialize(mesh);

		// the vehicle descriptor specify the kind of vehicle 
		// we configure it then we convert the model to multibody vehicle.
		ndVehicleDectriptorSuperCar superCar;
		vehicle->ConvertToMotorVehicle(superCar);

		ndRender* const renderer = *scene->GetRenderer();
		ndSharedPtr<ndRenderSceneNode> sceneMesh(ndRenderMeshLoader::CreateRenderSceneMesh(renderer, *loader.m_mesh, ndGetWorkingFileName("")));

		auto BindApplicationData = [scene, mesh, vehicle, &sceneMesh, &superCar](ndModelArticulation::ndNode* const node)
		{
			if (vehicle->IsCloseLoop(node))
			{
				ndTrace(("do something\n"));
			}
			else
			{
				const ndMesh* const meshNode = mesh->FindByClosestMatch(node->m_name);
				ndAssert(meshNode);

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

		//add the notification to bind to the application.
		ndSharedPtr<ndModelNotify> controller(new ndVehicleCommonNotify(vehicle));
		vehicle->SetNotifyCallback(controller);

		scene->AddEntity(sceneMesh);
		world->AddModel(vehicleModel);

		vehicle->SetTransform(matrix);
		sceneMesh->SetTransform(matrix);
		sceneMesh->SetTransform(matrix);
		return vehicleModel;
	}

	class SceneNavigation : public ndDemoEntityManager::OnPostUpdate
	{
		public:
		SceneNavigation()
			:OnPostUpdate()
		{
		}

		void OnDebug(ndDemoEntityManager* const, bool) override
		{
		}

		virtual void Update(ndDemoEntityManager* const manager, ndFloat32) override
		{
			if (manager->CameraChanged())
			{
				ndFixSizeArray<ndMultiBodyVehicle*, 256> vehicleArray;
				const ndModelList& models = manager->GetWorld()->GetModelList();

				// get the vehicle array
				for (ndModelList::ndNode* node = models.GetFirst(); node; node = node->GetNext())
				{
					ndMultiBodyVehicle* const vehicle = node->GetInfo()->GetAsMultiBodyVehicle();
					if (vehicle)
					{
						vehicleArray.PushBack(vehicle);
					}
				}
				// check if the camera is attach to a vehicle

				const ndRenderSceneCamera* const currentCamera = manager->GetRenderer()->GetCamera()->FindCameraNode();
				for (ndInt32 i = 0; i < vehicleArray.GetCount(); ++i)
				{
					ndMultiBodyVehicle* const vehicle = vehicleArray[i];
					ndVehicleCommonNotify* const modelNotifyCallback = (ndVehicleCommonNotify*)*vehicle->GetNotifyCallback();

					ndSharedPtr<ndBodyNotify>& notify = vehicle->GetRoot()->m_body->GetNotifyCallback();
					ndAssert(strcmp(notify->ClassName(), ndDemoEntityNotify::StaticClassName()) == 0);
					const ndDemoEntityNotify* const vehicleNotify = (ndDemoEntityNotify*)*notify;
					const ndRenderSceneNode* const visual = *vehicleNotify->m_entity;
					const ndRenderSceneCamera* const cameraNode = visual->FindCameraNode();
					if (cameraNode)
					{
						if (cameraNode == currentCamera)
						{
							modelNotifyCallback->SetPlaterState(true);
						}
						else
						{
							modelNotifyCallback->SetPlaterState(false);
						}
					}
					else
					{
						modelNotifyCallback->SetPlaterState(false);
					}
				}

				// iterate of the scene and 
			}
		}
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
};

using namespace ndMotorVehicle;

void ndBasicVehicle (ndDemoEntityManager* const scene)
{
	//ndSharedPtr<ndBody> bodyFloor(BuildPlayground(scene));
	//ndSharedPtr<ndBody> bodyFloor(BuildCompoundScene(scene, ndGetIdentityMatrix()));
	ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marblecheckboard.png", 0.1f, true));

	ndPhysicsWorld* const world = scene->GetWorld();
	ndVector location(0.0f, 2.0f, 0.0f, 1.0f);

	// add a vehicle material
	AddMaterial(scene);
	
	ndMatrix matrix(ndGetIdentityMatrix());
	ndVector floor(FindFloor(*world, location, 100.0f));
	matrix.m_posit = floor;
	matrix.m_posit.m_y += 0.5f;
	
	ndSharedPtr<ndModel> vehicle0(CreateBasicVehicle(scene, "testarossaMultiBody.nd", ndPlacementMatrix(matrix, ndVector(0.0f, 0.0f, -10.0f, 0.0f))));
	ndSharedPtr<ndModel> vehicle1(CreateBasicVehicle(scene, "pickupTruck.nd", ndPlacementMatrix(matrix, ndVector(0.0f, 0.0f, -5.0f, 0.0f))));
	ndSharedPtr<ndModel> vehicle2(CreateBasicVehicle(scene, "truck.nd", ndPlacementMatrix(matrix, ndVector(0.0f, 1.0f, 0.0f, 0.0f))));
	ndSharedPtr<ndModel> vehicle3(CreateBasicVehicle(scene, "tractor.nd", ndPlacementMatrix(matrix, ndVector(0.0f, 1.0f, 5.0f, 0.0f))));
	ndSharedPtr<ndModel> vehicle4(CreateBasicVehicle(scene, "lav-25.nd", ndPlacementMatrix(matrix, ndVector(0.0f, 1.0f, 10.0f, 0.0f))));

	ndVehicleCommonNotify* const notifyCallback = (ndVehicleCommonNotify*)*vehicle4->GetNotifyCallback();
	notifyCallback->SetPlaterState(true);
	
	matrix.m_posit.m_x += 40.0f;
	matrix.m_posit.m_z += 5.0f;
	AddPlanks(scene, matrix, 60.0f, 5);

	// add a scene navigation post update
	ndSharedPtr<ndDemoEntityManager::OnPostUpdate> driver(new SceneNavigation());
	scene->RegisterPostUpdate(driver);

	ndQuaternion rot;
	ndVector origin(-10.0f, 2.0f, -0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
