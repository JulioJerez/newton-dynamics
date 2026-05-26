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

class ndVanillaController : public ndModelNotify
{
    public:
    ndVanillaController(ndDemoEntityManager* const scene,
        ndSharedPtr<ndRenderSceneNode>& camera,
        ndModelArticulation* const model)
        :ndModelNotify()
        ,m_scene(scene)
        ,m_cameraNode(*camera)
        ,m_motor(model->FindByName("motor"))
        ,m_frontWheel(model->FindByName("frontWheel"))
        ,m_steerAngle(ndFloat32(0.0f))
        ,m_engineOmega(ndFloat32 (0.0f))
    {
        ndAssert(m_motor);
        SetModel(model);
    }

    void UpdateEngine(ndFloat32 timestep)
    {
        // reset the motor matrix to align with the chassis matrix
        if (m_motor)
        {
            ndJointDoubleHinge* const engine = (ndJointDoubleHinge*)*m_motor->m_joint;
            const ndMatrix matrix(engine->GetLocalMatrix0().OrthoInverse() * engine->GetLocalMatrix1() * engine->GetBody1()->GetMatrix());
            engine->GetBody0()->SetMatrixNoSleep(matrix);

            // integrate the joints angle;
            ndFloat32 fowardAngle = engine->GetAngle1();
            engine->SetTargetAngle1(fowardAngle + m_engineOmega * timestep);
        }

        if (m_frontWheel)
        {
            ndJointWheel* const frontWheel = (ndJointWheel*)*m_frontWheel->m_joint;
            frontWheel->UpdateTireSteeringAngleMatrix();
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
                m_engineOmega = ndFloat32(0.0f);
                const ndFloat32 engineOmega = ndFloat32(30.0f);
                if (m_scene->GetKeyState(ImGuiKey_W))
                {
                    m_engineOmega = -engineOmega;
                    engine->SetSleepState(false);
                }
                else if (m_scene->GetKeyState(ImGuiKey_S))
                {
                    m_engineOmega = engineOmega;
                    engine->SetSleepState(false);
                }
            }

            if (m_frontWheel)
            {
                // very simplistic steering system
                m_steerAngle = ndFloat32(0.0f);
                if (m_scene->GetKeyState(ImGuiKey_A))
                {
                    m_steerAngle = ndFloat32(1.0f);
                }
                else if (m_scene->GetKeyState(ImGuiKey_D))
                {
                    m_steerAngle = ndFloat32(-1.0f);
                }

                ndJointWheel* const frontWheel = (ndJointWheel*)*m_frontWheel->m_joint;
                ndFloat32 angle0 = frontWheel->GetSteering();
                ndFloat32 filter = ndFloat32(15.0f * timestep);
                ndFloat32 angle = angle0 + (m_steerAngle - angle0) * filter;
                
                if (ndAbs(angle0 - angle) > ndFloat32(1.0e-3f))
                {
                    ndBodyDynamic* const wheel = m_frontWheel->m_body->GetAsBodyDynamic();
                    wheel->SetSleepState(false);
                    frontWheel->SetSteering(angle);
                }
            }
        }
    }

    ndWeakPtr<ndDemoEntityManager> m_scene;
    ndWeakPtr<ndRenderSceneNode> m_cameraNode;
    ndWeakPtr<ndModelArticulation::ndNode> m_motor;
    ndWeakPtr<ndModelArticulation::ndNode> m_frontWheel;
    ndFloat32 m_steerAngle;
    ndFloat32 m_engineOmega;
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
    ndSharedPtr<ndRenderSceneNode> cameraPivotNode(sceneMesh->FindByName("cameraPivot")->GetSharedPtr());
    if (cameraPivotNode)
    {
        ndVector cameraPivot(ndVector::m_zero);
        camera = ndSharedPtr<ndRenderSceneNode>(new ndDemoCameraNodeFollow(renderer, cameraPivot, -4.0f));
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
            node->m_body->SetNotifyCallback(notify);
        }
    };
    articulation->NodeIterator(BindApplicationData);

    ndSharedPtr<ndModelNotify> controller(new ndVanillaController(scene, camera, articulation));
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
