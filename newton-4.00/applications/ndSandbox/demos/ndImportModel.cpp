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
#include "ndFileBrowser.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoCameraNodeFollow.h"
#include "ndHeightFieldPrimitive.h"
#include "ndGameControllerInputs.h"

class ndVanillaController : public ndModelNotify
{
    public:
    ndVanillaController(ndDemoEntityManager* const scene,
        const ndMesh* const mesh,
        ndSharedPtr<ndRenderSceneNode>& camera,
        ndModelArticulation* const model)
        :ndModelNotify()
        ,m_scene(scene)
        ,m_cameraNode(*camera)
        ,m_motor(model->FindByName("motor"))
        ,m_steerAngle(ndFloat32(0.0f))
        ,m_engineDrag(ndFloat32(0.1f))
        ,m_engineTorque(ndFloat32 (0.0f))
        ,m_engineMaxTorque(ndFloat32(1000.0f))
    {
        SetModel(model);

        //bind Motor speed.
        const ndMesh* const motorMesh = mesh->FindByName("motor");
        if (motorMesh)
        {
            const ndMeshCustomPropertyFloat* const maxTorque = (ndMeshCustomPropertyFloat*)motorMesh->GetCustomPropertyByName("torque");
            if (maxTorque)
            {
                m_engineMaxTorque = maxTorque->m_value;
            }

            const ndMeshCustomPropertyFloat* const maxOmega = (ndMeshCustomPropertyFloat*)motorMesh->GetCustomPropertyByName("omega");
            if (maxOmega)
            {
                m_engineDrag = m_engineMaxTorque / (maxOmega->m_value * maxOmega->m_value);
            }
        }

        // find the wheels
        auto FindWheels = [this](ndModelArticulation::ndNode* const node)
        {
            if (node->m_joint && (strcmp(node->m_joint->ClassName(), ndJointWheel::StaticClassName()) == 0))
            {
                m_wheels.Append((ndJointWheel*)*node->m_joint);
            }
        };
        model->NodeIterator(FindWheels);
    }

    void UpdateEngine(ndFloat32)
    {
        // reset the motor matrix to align with the chassis matrix
        if (m_motor)
        {
            // very simple electric motor.
            ndMatrix engineMatrix;
            ndMatrix chassisMatrix;

            ndJointDoubleHinge* const engineJoint = (ndJointDoubleHinge*)*m_motor->m_joint;
            ndBodyDynamic* const engineBody = m_motor->m_body->GetAsBodyDynamic();
            ndBodyDynamic* const chassisBody = m_motor->GetParent()->m_body->GetAsBodyDynamic();

            engineJoint->CalculateGlobalMatrix(engineMatrix, chassisMatrix);
            engineBody->SetMatrixNoSleep(engineJoint->GetLocalMatrix0().OrthoInverse() * chassisMatrix);

            const ndVector veloc(chassisBody->GetVelocityAtPoint(chassisMatrix.m_posit));
            engineBody->SetVelocityNoSleep(veloc);

            const ndVector engineOmega(engineBody->GetOmega());
            const ndVector chassisOmega(chassisBody->GetOmega());
            const ndVector wy(chassisMatrix.m_up.Scale(chassisMatrix.m_up.DotProduct(engineOmega).GetScalar()));
            const ndVector wx(chassisMatrix.m_front.Scale(chassisMatrix.m_front.DotProduct(engineOmega).GetScalar()));
            const ndVector wz(chassisMatrix.m_right.Scale(chassisMatrix.m_right.DotProduct(chassisOmega).GetScalar()));
            engineBody->SetOmegaNoSleep(wx + wy + wz);

            ndFloat32 omega = chassisMatrix.m_up.DotProduct(engineOmega).GetScalar();
            ndFloat32 torqueMag = m_engineTorque - m_engineDrag * omega * omega * ndSign(omega);

            const ndVector torque(engineMatrix.m_front.Scale(torqueMag));
            engineBody->SetTorque(torque);

            for (ndList<ndWeakPtr<ndJointWheel>>::ndNode* node = m_wheels.GetFirst(); node; node = node->GetNext())
            {
                ndJointWheel* const wheel = *node->GetInfo();
                wheel->UpdateTireSteeringAngleMatrix();
            }
        }
    }

    void Update(ndFloat32 timestep) override
    {
        ndModelNotify::Update(timestep);
        UpdateEngine(timestep);
    }

    void PostTransformUpdate(ndFloat32 timestep) override
    {
        ndRender* const renderer = *m_scene->GetRenderer();
        ndSharedPtr<ndRenderSceneNode> camera(renderer->GetCamera());
        if (*camera == *m_cameraNode)
        {
            if (m_motor)
            {
                // very simplistic moter power system
                ndBodyDynamic* const engine = m_motor->m_body->GetAsBodyDynamic();

                const ndSharedPtr<ndGameControllerInputs>& gameController = m_scene->GetGameController();
                const ndFixSizeArray<ndFloat32, 8>& axis = gameController->GetAxis();

                ndFloat32 throttle = axis[ndGameControllerInputs::m_gasPedal];

                //ndTrace(("%f %f\n", throttle, brake));
                m_engineTorque = throttle * m_engineMaxTorque;
                if (ndAbs(throttle) > 0.1f)
                {
                    engine->SetSleepState(false);
                }
            }

            if (m_wheels.GetCount())
            {
                const ndSharedPtr<ndGameControllerInputs>& gameController = m_scene->GetGameController();
                const ndFixSizeArray<ndFloat32, 8>& axis = gameController->GetAxis();

                // very simplistic steering system
                ndFloat32 brake = axis[ndGameControllerInputs::m_brakePedal];
                ndFloat32 steeringParam = axis[ndGameControllerInputs::m_steeringWheel];
                m_steerAngle = steeringParam * ndFloat32 (1.0f);

                for (ndList<ndWeakPtr<ndJointWheel>>::ndNode* node = m_wheels.GetFirst(); node; node = node->GetNext())
                {
                    ndJointWheel* const wheel = *node->GetInfo();
                    ndFloat32 angle0 = wheel->GetSteering();
                    ndFloat32 filter = ndFloat32(15.0f * timestep);
                    ndFloat32 angle = angle0 + (m_steerAngle - angle0) * filter;

                    if (ndAbs(angle0 - angle) > ndFloat32(1.0e-3f))
                    {
                        ndBodyDynamic* const wheelBody = wheel->GetBody0()->GetAsBodyDynamic();
                        wheelBody->SetSleepState(false);
                    }
                    if (ndAbs(brake) > ndFloat32 (0.1f))
                    {
                        ndBodyDynamic* const wheelBody = wheel->GetBody0()->GetAsBodyDynamic();
                        wheelBody->SetSleepState(false);
                    }
                    wheel->SetBrake(brake);
                    wheel->SetSteering(angle);
                }
            }
        }
    }

    ndWeakPtr<ndDemoEntityManager> m_scene;
    ndWeakPtr<ndRenderSceneNode> m_cameraNode;
    ndWeakPtr<ndModelArticulation::ndNode> m_motor;
    ndList<ndWeakPtr<ndJointWheel>> m_wheels;
    ndFloat32 m_steerAngle;
    ndFloat32 m_engineDrag;
    ndFloat32 m_engineTorque;
    ndFloat32 m_engineMaxTorque;
};

static ndSharedPtr<ndModel> LoadAndBindModel(ndDemoEntityManager* const scene, const ndMatrix& location, const char* const pathFileName)
{
    ndMeshLoader loader;
    loader.LoadMesh(pathFileName);

    // make an articulated from the loaded mesh
    ndSharedPtr<ndModel> model(new ndModelArticulation());
    model->GetAsModelArticulation()->Deserialize(*loader.m_mesh);

    // make a hierarchical render mesh and add to the render scene
    ndRender* const renderer = *scene->GetRenderer();
    ndSharedPtr<ndRenderSceneNode> sceneMesh(ndRenderMeshLoader::CreateRenderSceneMesh(renderer, *loader.m_mesh, ndGetWorkingFileName("")));
    scene->AddEntity(sceneMesh);

    // set the matrix location to both visual and physic
    ndModelArticulation* const articulation = model->GetAsModelArticulation();
    const ndModelArticulation::ndNode* const rootNode = articulation->GetRoot();
    const ndMatrix matrix(rootNode ? rootNode->m_body->GetMatrix() * location : location);
    sceneMesh->SetTransform(matrix);
    sceneMesh->SetTransform(matrix);
    model->GetAsModelArticulation()->SetTransform(matrix);

    // bind a camera to the the cemara pivot if it has one
    ndSharedPtr<ndRenderSceneNode> camera(nullptr);
    ndRenderSceneNode* const cameraPivotNode = sceneMesh->FindByName("cameraPivot");
    if (cameraPivotNode)
    {
        ndVector cameraPivot(ndVector::m_zero);
        const ndMesh* const cameraPivotMesh = loader.m_mesh->FindByName("cameraPivot");
        ndAssert(cameraPivotMesh);
        ndMeshCustomPropertyFloat* const property = (ndMeshCustomPropertyFloat*)cameraPivotMesh->GetCustomPropertyByName("cameraDistance");
        ndFloat32 dist = property ? -ndAbs(property->m_value) : ndFloat32 (-5.0f);
        camera = ndSharedPtr<ndRenderSceneNode>(new ndDemoCameraNodeFollow(renderer, cameraPivot, dist));
        cameraPivotNode->AddChild(camera);
    }

    // Bind application data to the model, 
    // this could be a render mesh or something else. 
    // For this demo we use ndRenderSceneNode mesh
    const ndMesh* const rootMesh = *loader.m_mesh;
    auto BindApplicationData = [scene, articulation, rootMesh, &sceneMesh](ndModelArticulation::ndNode* const node)
    {
        if (articulation->IsCloseLoop(node))
        {
            ndTrace(("do somthing\n"));
        }
        else
        { 
            const ndMesh* const meshNode = rootMesh->FindByClosestMatch(node->m_name);
            ndAssert(meshNode);

            // find the visual node this body control by name. 
            const ndMatrix matrix(node->m_body->GetMatrix());
            ndRenderSceneNode* const visualEntityPtr = sceneMesh->FindByClosestMatch(meshNode->GetName());
            ndAssert(visualEntityPtr);
            ndSharedPtr<ndRenderSceneNode> visualEntity((visualEntityPtr == *sceneMesh) ? sceneMesh : visualEntityPtr->GetSharedPtr());

            // add a rigid body with notification callback
            ndBodyKinematic* const parentBody = node->GetParent() ? node->GetParent()->m_body->GetAsBodyKinematic() : nullptr;
            ndSharedPtr<ndBodyNotify> notify(new ndDemoEntityNotify(scene, visualEntity, parentBody));

            if (node->m_body)
            {
                ndDemoEntityNotify* const fastNotify = (ndDemoEntityNotify*)*notify;
                fastNotify->m_capSpeed = ndFloat32(30.0f);
                node->m_body->GetAsBodyDynamic()->SetMaxLinearAndAngularIntegrationStep(node->m_body->GetAsBodyDynamic()->GetMaxAngularStep(), ndFloat32(10.0f));
            }

            if (node->m_joint)
            {
                if ((strcmp(node->m_joint->ClassName(), ndJointWheel::StaticClassName()) == 0) ||
                    (strcmp(node->m_joint->ClassName(), ndJointDoubleHinge::StaticClassName()) == 0))
                {
                    // fast moving wheel
                    ndDemoEntityNotify* const fastNotify = (ndDemoEntityNotify*)*notify;
                    fastNotify->m_capOmega = ndFloat32(100.0f);
                    node->m_body->GetAsBodyDynamic()->SetMaxLinearAndAngularIntegrationStep(ndFloat32(360.0f) * ndDegreeToRad, node->m_body->GetAsBodyDynamic()->GetMaxLinearStep());
                }
            }
            node->m_body->SetNotifyCallback(notify);
        }
    };
    articulation->NodeIterator(BindApplicationData);

    ndSharedPtr<ndModelNotify> controller(new ndVanillaController(scene, *loader.m_mesh, camera, articulation));
    model->SetNotifyCallback(controller);

    return model;
}

void ndImportModel(ndDemoEntityManager* const scene)
{
    ndSharedPtr<ndBody> floor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

    ndMatrix origin(ndGetIdentityMatrix());
    origin.m_posit.m_y = 3.0f;

    char pathFileName[1024];
    const ndString& lastFile = scene->GetLastLoadMesh();
    snprintf(pathFileName, sizeof(pathFileName), "%s", lastFile.GetStr());

    ndTransform cameraTransform(scene->GetCameraMatrix());
    if (dGetLoadNdFileName(pathFileName, sizeof(pathFileName)))
    {
        ndPhysicsWorld* const world = scene->GetWorld();
        ndSharedPtr<ndModel> testModel(LoadAndBindModel(scene, origin, pathFileName));
        world->AddModel(testModel);

        if (strcmp(pathFileName, lastFile.GetStr()) != 0)
        {
            ndQuaternion rot;
            origin.m_posit.m_x -= 8.0f;
            origin.m_posit.m_y = 2.0f;
            cameraTransform = ndTransform(rot, origin.m_posit);
            scene->SetLastLoadMesh(ndString(pathFileName));
        }
    }

    scene->SetCameraMatrix(cameraTransform.m_rotation, cameraTransform.m_position);
}
