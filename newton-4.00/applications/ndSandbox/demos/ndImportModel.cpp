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
    const ndModelArticulation::ndNode* const rootNode = model->GetAsModelArticulation()->GetRoot();
    const ndMatrix matrix(rootNode ? rootNode->m_body->GetMatrix() * location : location);
    sceneMesh->SetTransform(matrix);
    sceneMesh->SetTransform(matrix);
    model->GetAsModelArticulation()->SetTransform(matrix);

    // Bind application data to the model, 
    // this could be a render mesh or something else. 
    // For this demo it is ndRenderSceneNode mesh
    const ndMesh* const rootMesh = *loader.m_mesh;
    auto BindApplicationData = [scene, rootMesh, &sceneMesh](ndModelArticulation::ndNode* const node)
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
    };

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
