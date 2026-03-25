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

static ndSharedPtr<ndModel> LoadAndBindModel(ndDemoEntityManager* const scene, const ndVector& location, const char* const pathFileName)
{
    ndMeshLoader loader;
    loader.LoadMesh(pathFileName);

    // make an articulated from the loaded mesh
    ndSharedPtr<ndModel> model(new ndModelArticulation());
    model->GetAsModelArticulation()->Deserialize(*loader.m_mesh);

    // set the matrix location
    ndMatrix matrix(ndGetIdentityMatrix());
    matrix.m_posit = location;
    model->GetAsModelArticulation()->SetTransform(matrix);

    // Bind application data to the model
    auto BindApplicationData = [scene, &loader, model]()
    {
        ndRender* const render = *scene->GetRenderer();

        const ndMesh* const rootMesh = *loader.m_mesh;
        for (ndModelArticulation::ndNode* node = model->GetAsModelArticulation()->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
        {
            // find the mesh node
            const ndMesh* const meshNode = rootMesh->FindByClosestMatch(node->m_name);
            ndAssert(meshNode);

            const ndMatrix matrix(node->m_body->GetMatrix());
            ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(matrix));
            scene->AddEntity(entity);

            // create a graphic primitive for visualization
            ndSharedPtr<ndMeshShapeInstance> primitive(meshNode->GetPrimitive());
            ndSharedPtr<ndShapeInstance> primitiveInstance(primitive->CreateObject());
            if (!primitiveInstance->GetShape()->GetAsShapeNull())
            {
                ndRenderPrimitive::ndDescriptor descriptor(render);
                descriptor.m_collision = ndSharedPtr<ndShapeInstance>(primitive->CreateObject());
                descriptor.m_mapping = ndRenderPrimitive::m_box;
                descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_0.png")));
                ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));
                entity->SetPrimitiveMatrix(meshNode->GetGeometryMatrix());
                entity->SetPrimitive(mesh);
            }

            // add a rigid body notification callback
            ndSharedPtr<ndBodyNotify> notify(new ndDemoEntityNotify(scene, entity));
            ((ndDemoEntityNotify*)*notify)->ResetEntityTransform(matrix);
            node->m_body->SetNotifyCallback(notify);
        }
    };
    BindApplicationData();
    return model;
}

void ndImportModel(ndDemoEntityManager* const scene)
{
    ndSharedPtr<ndBody> floor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

    ndVector origin(ndVector::m_wOne);
    origin.m_y = 3.0f;

    char pathFileName[1024];
    const ndString& lastFile = scene->GetLastLoadMesh();
    snprintf(pathFileName, sizeof(pathFileName), "%s", lastFile.GetStr());

    ndTransform cameraTransform(scene->GetCameraMatrix());
    if (dGetLoadNdFileName(pathFileName, sizeof(pathFileName)))
    {
        ndPhysicsWorld* const world = scene->GetWorld();
        ndSharedPtr<ndModel> testModel(LoadAndBindModel(scene, origin, pathFileName));
        world->AddModel(testModel);

        if (strcmp(pathFileName, lastFile.GetStr()))
        {
            ndQuaternion rot;
            origin.m_x -= 8.0f;
            origin.m_y = 2.0f;
            cameraTransform = ndTransform(rot, origin);
            scene->SetLastLoadMesh(ndString(pathFileName));
        }
    }

    scene->SetCameraMatrix(cameraTransform.m_rotation, cameraTransform.m_position);
}
