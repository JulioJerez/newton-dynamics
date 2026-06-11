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
#include "ndDemoCameraNodeFollow.h"
#include "ndHeightFieldPrimitive.h"

#if 0
class ndVehicleDectriptorViper : public ndVehicleDectriptor
{
	public:
	ndVehicleDectriptorViper()
		:ndVehicleDectriptor("viper.fbx")
	{
		ndFloat32 idleTorquePoundFoot = 300.0f;
		ndFloat32 idleRmp = 700.0f;
		ndFloat32 horsePower = 400.0f;
		ndFloat32 rpm0 = 5000.0f;
		ndFloat32 rpm1 = 6200.0f;
		ndFloat32 horsePowerAtRedLine = 100.0f;
		ndFloat32 redLineRpm = 8000.0f;
		m_engine.Init(idleTorquePoundFoot, idleRmp, 
					  horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

		m_comDisplacement = ndVector(0.0f, -0.5f, 0.0f, 0.0f);

		m_frontTire.m_mass = 25.0f;
		m_frontTire.m_handBrakeTorque = 0.0f;

		m_rearTire.m_mass = 25.0f;
		m_rearTire.m_handBrakeTorque = 100000.0f;

		// Get a stock pacejka curve set and modified a litle for dramatic driving
		ndTireFrictionModel::ndPacejkaTireModel lateral;
		ndTireFrictionModel::ndPacejkaTireModel longitudinal;
		m_frontTire.GetPacejkaCurves(ndTireFrictionModel::m_pacejkaSport, longitudinal, lateral);
		lateral.m_d = 0.4f;

		// override the tire cuves.
		m_rearTire.SetPacejkaCurves(longitudinal, lateral);
		m_frontTire.SetPacejkaCurves(longitudinal, lateral);

		// plot the curve to check it is a value form
		m_frontTire.PlotPacejkaCurves("sportTireModel");
	}
};

class ndVehicleDectriptorJeep : public ndVehicleDectriptor
{
	public:
	ndVehicleDectriptorJeep()
		:ndVehicleDectriptor("jeep.fbx")
	{
		ndFloat32 idleTorquePoundFoot = 200.0f;
		ndFloat32 idleRmp = 800.0f;
		ndFloat32 horsePower = 400.0f;
		ndFloat32 rpm0 = 5000.0f;
		ndFloat32 rpm1 = 6200.0f;
		ndFloat32 horsePowerAtRedLine = 400.0f;
		ndFloat32 redLineRpm = 8000.0f;
		m_engine.Init(idleTorquePoundFoot, idleRmp, 
					  horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

		m_comDisplacement = ndVector(0.0f, -0.65f, 0.0f, 0.0f);

		m_chassisMass = 1000.0f;
		m_frontTire.m_mass = 25.0f;
		m_frontTire.m_verticalOffset = 0.0f;
		m_frontTire.m_steeringAngle = 35.0f * ndDegreeToRad;
		m_frontTire.m_springK = 800.0f;
		m_frontTire.m_damperC = 50.0f;
		m_frontTire.m_regularizer = 0.1f;
		m_frontTire.m_lowerStop = -0.05f;
		m_frontTire.m_upperStop = 0.4f;
		m_frontTire.m_brakeTorque = 1500.0f;
		m_frontTire.m_handBrakeTorque = 0.0f;

		m_rearTire.m_mass = 25.0f;
		m_rearTire.m_verticalOffset = 0.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 800.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.1f;
		m_rearTire.m_lowerStop = -0.05f;
		m_rearTire.m_upperStop = 0.4f;
		m_rearTire.m_brakeTorque = 3000.0f;
		m_rearTire.m_handBrakeTorque = 100000.0f;
		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_fourWheeldrive;

		// Get a stock pacejka curve set and modified a litle for dramatic driving
		ndTireFrictionModel::ndPacejkaTireModel lateral;
		ndTireFrictionModel::ndPacejkaTireModel longitudinal;
		m_frontTire.GetPacejkaCurves(ndTireFrictionModel::m_pacejkaSport, longitudinal, lateral);
		lateral.m_d = 0.5f;

		// override the tire cuves.
		m_rearTire.SetPacejkaCurves(longitudinal, lateral);
		m_frontTire.SetPacejkaCurves(longitudinal, lateral);

		// plot the curve to check it is a value form
		m_frontTire.PlotPacejkaCurves("sportTireModel");
	}
};

class ndVehicleDectriptorMonsterTruck0: public ndVehicleDectriptor
{
	public:
	ndVehicleDectriptorMonsterTruck0()
		:ndVehicleDectriptor("monsterTruck0.fbx")
	{
		ndFloat32 idleTorquePoundFoot = 250.0f;
		ndFloat32 idleRmp = 800.0f;
		ndFloat32 horsePower = 400.0f;
		ndFloat32 rpm0 = 5000.0f;
		ndFloat32 rpm1 = 6200.0f;
		ndFloat32 horsePowerAtRedLine = 150.0f;
		ndFloat32 redLineRpm = 8000.0f;
		m_engine.Init(idleTorquePoundFoot, idleRmp, 
					  horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

		m_chassisMass = 1000.0f;
		m_comDisplacement = ndVector(0.0f, -0.6f, 0.0f, 0.0f);

		//m_frontTire.m_mass = 150.0f;
		m_frontTire.m_mass = 100.0f;
		m_frontTire.m_verticalOffset = 0.0f;
		m_frontTire.m_steeringAngle = 35.0f * ndDegreeToRad;
		m_frontTire.m_springK = 500.0f;
		m_frontTire.m_damperC = 20.0f;
		m_frontTire.m_regularizer = 0.05f;
		m_frontTire.m_lowerStop = -0.05f;
		m_frontTire.m_upperStop = 0.4f;
		m_frontTire.m_brakeTorque = 1000.0f;
		m_frontTire.m_handBrakeTorque = 0.0f;

		m_rearTire.m_mass = 150.0f;
		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_verticalOffset = 0.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 20.0f;
		m_rearTire.m_regularizer = 0.05f;
		m_rearTire.m_lowerStop = -0.05f;
		m_rearTire.m_upperStop = 0.4f;
		m_rearTire.m_brakeTorque = 5000.0f;
		m_rearTire.m_handBrakeTorque = 100000.0f;
		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_fourWheeldrive;

		// Get a stock pacejka curve set and modified a litle for dramatic driving
		ndTireFrictionModel::ndPacejkaTireModel lateral;
		ndTireFrictionModel::ndPacejkaTireModel longitudinal;
		m_frontTire.GetPacejkaCurves(ndTireFrictionModel::m_pacejkaUtility, longitudinal, lateral);
		lateral.m_d = 0.25f;

		// override the tire cuves.
		m_rearTire.SetPacejkaCurves(longitudinal, lateral);
		m_frontTire.SetPacejkaCurves(longitudinal, lateral);

		// plot the curve to check it is a value form
		m_frontTire.PlotPacejkaCurves("utilityVehicleTireModel");
	}
};

class ndVehicleDectriptorMonsterTruck1 : public ndVehicleDectriptor
{
	public:
	ndVehicleDectriptorMonsterTruck1()
		:ndVehicleDectriptor("monsterTruck1.fbx")
	{
		// reset gear box ratios
		m_transmission.m_gearsCount = 4;
		m_transmission.m_forwardRatios[0] = 2.5f;
		m_transmission.m_forwardRatios[1] = 1.5f;
		m_transmission.m_forwardRatios[2] = 1.1f;
		m_transmission.m_forwardRatios[3] = 0.8f;
		m_transmission.m_crownGearRatio = 20.0f;

		m_comDisplacement = ndVector(0.0f, -1.3f, 0.0f, 0.0f);
		
		ndFloat32 idleTorquePoundFoot = 300.0f;
		ndFloat32 idleRmp = 800.0f;
		ndFloat32 horsePower = 600.0f;
		ndFloat32 rpm0 = 5000.0f;
		ndFloat32 rpm1 = 6200.0f;
		ndFloat32 horsePowerAtRedLine = 150.0f;
		ndFloat32 redLineRpm = 8000.0f;
		m_engine.Init(idleTorquePoundFoot, idleRmp,
			horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

		m_frontTire.m_mass = 100.0f;
		m_frontTire.m_verticalOffset = 0.0f;
		m_frontTire.m_steeringAngle = 35.0f * ndDegreeToRad;
		m_frontTire.m_springK = 500.0f;
		m_frontTire.m_damperC = 50.0f;
		m_frontTire.m_regularizer = 0.2f;
		m_frontTire.m_lowerStop = -0.05f;
		m_frontTire.m_upperStop = 0.4f;
		m_frontTire.m_brakeTorque = 10000.0f;
		m_frontTire.m_handBrakeTorque = 0.0f;

		m_rearTire.m_mass = 100.0f;
		m_rearTire.m_verticalOffset = 0.0f;
		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 500.0f;
		m_rearTire.m_damperC = 50.0f;
		m_rearTire.m_regularizer = 0.2f;
		m_rearTire.m_lowerStop = -0.05f;
		m_rearTire.m_upperStop = 0.4f;
		m_rearTire.m_brakeTorque = 10000.0f;
		m_rearTire.m_handBrakeTorque = 50000.0f;
		m_torsionBarType = m_fourWheelAxle;
		m_differentialType = m_fourWheeldrive;

		ndTireFrictionModel::ndPacejkaTireModel lateral;
		ndTireFrictionModel::ndPacejkaTireModel longitudinal;
		m_frontTire.GetPacejkaCurves(ndTireFrictionModel::m_pacejkaUtility, longitudinal, lateral);
		lateral.m_d = 0.3f;

		// override the tire cuves.
		m_rearTire.SetPacejkaCurves(longitudinal, lateral);
		m_frontTire.SetPacejkaCurves(longitudinal, lateral);

		// plot the curve to check it is a value form
		m_frontTire.PlotPacejkaCurves("utilityVehicleTireModel");
	}
};

static ndVehicleDectriptorJeep jeepDesc;
static ndVehicleDectriptorViper viperDesc;
static ndVehicleDectriptorMonsterTruck0 monterTruckDesc0;
static ndVehicleDectriptorMonsterTruck1 monterTruckDesc1;

static ndDemoEntity* LoadVehicleMeshModel(ndDemoEntityManager* const scene, const char* const filename)
{
	ndRenderMeshLoader loader;
	ndDemoEntity* const vehicleEntity = loader.ImportFbx(filename, scene);
	return vehicleEntity;
}

//static void TestPlayerCapsuleInteraction(ndDemoEntityManager* const scene, const ndMatrix& location)
static void TestPlayerCapsuleInteraction(ndDemoEntityManager* const, const ndMatrix&)
{
	//ndMatrix localAxis(ndGetIdentityMatrix());
	//localAxis[0] = ndVector(0.0, 1.0f, 0.0f, 0.0f);
	//localAxis[1] = ndVector(1.0, 0.0f, 0.0f, 0.0f);
	//localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);
	//
	//ndFloat32 height = 1.9f;
	//ndFloat32 radio = 0.5f;
	//ndFloat32 mass = 100.0f;
	//ndDemoEntity* const entity = ndDemoEntity::LoadFbx("walker.fbx", scene);
	//new ndBasicPlayerCapsule(scene, entity, localAxis, location, mass, radio, height, height / 4.0f);
	//delete entity;
}
#endif

namespace ndMotorVehicle
{
	class ndVehicleDectriptorSuperCar : public ndVehicleDectriptor
	{
		public:
		ndVehicleDectriptorSuperCar()
			:ndVehicleDectriptor()
		{
			m_name = "supercar";
			ndFloat32 idleTorquePoundFoot = ndFloat32(300.0f);
			ndFloat32 idleRmp = ndFloat32(700.0f);
			ndFloat32 horsePower = ndFloat32(400.0f);
			ndFloat32 rpm0 = ndFloat32(5000.0f);
			ndFloat32 rpm1 = ndFloat32(6200.0f);
			ndFloat32 horsePowerAtRedLine = ndFloat32(100.0f);
			ndFloat32 redLineRpm = ndFloat32(8000.0f);
			m_engine.Init(idleTorquePoundFoot, idleRmp,
				horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

			m_tireFrictionModel.SetPacejkaCurves(ndTireFrictionModel::m_pacejkaSport);
			m_tireFrictionModel.m_lateralPacejka.m_d = ndFloat32 (0.4f);

			// plot the curve to check it is a value form
			//m_tireFrictionModel.PlotPacejkaCurves("supercar");
		}
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
		//ndVehicleDectriptor defaultDesc;
		//defaultDesc.m_name = "testarossa";
		//defaultDesc.m_tireFrictionModel.SetPacejkaCurves(ndTireFrictionModel::m_pacejkaSport);
		ndVehicleDectriptorSuperCar superCar;
		vehicle->ConvertToMotorVehicle(superCar);

		ndRender* const renderer = *scene->GetRenderer();
		ndSharedPtr<ndRenderSceneNode> sceneMesh(ndRenderMeshLoader::CreateRenderSceneMesh(renderer, *loader.m_mesh, ndGetWorkingFileName("")));

		auto BindApplicationData = [scene, mesh, vehicle, &sceneMesh](ndModelArticulation::ndNode* const node)
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
		//ndSharedPtr<ndModelNotify> controller(new ndVehicleCommonNotify(scene, *loader.m_mesh, camera, articulation));
		ndSharedPtr<ndModelNotify> controller(new ndVehicleCommonNotify(vehicle));
		vehicle->SetNotifyCallback(controller);

		scene->AddEntity(sceneMesh);
		world->AddModel(vehicleModel);

		vehicle->SetTransform(matrix);
		sceneMesh->SetTransform(matrix);
		sceneMesh->SetTransform(matrix);
		return vehicleModel;
	}
};

using namespace ndMotorVehicle;

void ndBasicVehicle (ndDemoEntityManager* const scene)
{
	//ndSharedPtr<ndBody> bodyFloor(BuildPlayground(scene));
	//ndSharedPtr<ndBody> bodyFloor(BuildCompoundScene(scene, ndGetIdentityMatrix()));
	ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marblecheckboard.png", 0.1f, true));

	class ndPlacementMatrix : public ndMatrix
	{
		public:
		ndPlacementMatrix(const ndMatrix base, const ndVector& offset)
			:ndMatrix(base)
		{
			m_posit += offset;
		}
	};

	ndPhysicsWorld* const world = scene->GetWorld();
	//ndMatrix sceneLocation(ndGetIdentityMatrix());
	//sceneLocation.m_posit.m_x = -200.0f;
	//sceneLocation.m_posit.m_z = -200.0f;

	ndVector location(0.0f, 2.0f, 0.0f, 1.0f);
	
	ndMatrix matrix(ndGetIdentityMatrix());
	ndVector floor(FindFloor(*world, location, 100.0f));
	matrix.m_posit = floor;
	matrix.m_posit.m_y += 0.5f;
	
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
	//
	//// add a model for general controls
	//ndSharedPtr<ndModel> controls(new ndVehicleSelector());
	//world->AddModel(controls);
	//
	//ndSharedPtr<ndUIEntity> vehicleUI(new ndVehicleUI(scene));
	//scene->Set2DDisplayRenderFunction(vehicleUI);
	
	ndSharedPtr<ndModel> vehicle0(CreateBasicVehicle(scene, "testarossaMultiBody.nd", ndPlacementMatrix(matrix, ndVector(0.0f, 0.0f, 0.0f, 0.0f))));
	//ndSharedPtr<ndModel> vehicle1 (CreateBasicVehicle(scene, jeepDesc, ndPlacementMatrix(matrix, ndVector(0.0f, 0.0f,  -6.0f, 0.0f)), (ndVehicleUI*)*vehicleUI));
	//ndSharedPtr<ndModel> vehicle2 (CreateBasicVehicle(scene, monterTruckDesc0, ndPlacementMatrix(matrix, ndVector(0.0f, 0.0f, 0.0f, 0.0f)), (ndVehicleUI*)*vehicleUI));
	//ndSharedPtr<ndModel> vehicle3 (CreateBasicVehicle(scene, monterTruckDesc1, ndPlacementMatrix(matrix, ndVector(0.0f, 0.0f, 6.0f, 0.0f)), (ndVehicleUI*)*vehicleUI));

	////test removing model from world
	////vehicle1->RemoveBodiesAndJointsFromWorld();
	////world->RemoveModel(*vehicle1);
	//
	//ndSharedPtr<ndModel> vehicle(vehicle0);
	//ndVehicleCommonNotify* const notifyCallback = (ndVehicleCommonNotify*)*vehicle->GetNotifyCallback();
	//notifyCallback->SetAsPlayer(scene);
	//matrix.m_posit.m_x += 5.0f;
	////TestPlayerCapsuleInteraction(scene, matrix);
	//
	//matrix.m_posit.m_x += 40.0f;
	//matrix.m_posit.m_z += 5.0f;
	//AddPlanks(scene, matrix, 60.0f, 5);

	ndQuaternion rot;
	ndVector origin(-10.0f, 2.0f, -0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
