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

void ndImportMesh(ndDemoEntityManager* const scene)
{
    ndSharedPtr<ndBody> floor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

    ndMatrix origin(ndGetIdentityMatrix());
    origin.m_posit.m_y = 3.0f;

    char pathFileName[1024];
    const ndString& lastFile = scene->GetLastLoadMesh();
    snprintf(pathFileName, sizeof(pathFileName), "%s", lastFile.GetStr());

    ndTransform cameraTransform(scene->GetCameraMatrix());
    if (ndGetLoadFileName(pathFileName, sizeof(pathFileName)))
    {
		// load the mesh
		ndMeshLoader loader;
		loader.LoadMesh(pathFileName);

		// set all the alpha test materials
		auto ProcessMeshNodes = [scene, &pathFileName](ndMesh* const node)
		{
			// make sure props are invisible whne rendering the map
			ndMeshBodyDynamic* const rigidBody = (ndMeshBodyDynamic*)*node->GetRigidBody();
			if (rigidBody)
			{
				// make a new rigi bode and add to teh scene
				ndSharedPtr<ndBody> dynBody(new ndBodyDynamic());
				dynBody->Deserialize(rigidBody);

				const ndMatrix matrix(node->CalculateGlobalMatrix());
				dynBody->SetMatrix(matrix);

				// generate the visual mesh
				ndRender* const renderer = *scene->GetRenderer();
				const ndString materialPath(ndString(pathFileName).GetPath());
				ndSharedPtr<ndRenderSceneNode> sceneMesh(ndRenderMeshLoader::CreateRenderSceneMesh(renderer, node, materialPath));
				
				// bind scene and physics with a rb notification 
				ndSharedPtr<ndBodyNotify> notify(new ndDemoEntityNotify(scene, sceneMesh, nullptr));
				dynBody->SetNotifyCallback(notify);
				
				// add rb and visual mesh to the world and visual scene
				ndPhysicsWorld* const world = scene->GetWorld();
				scene->AddEntity(sceneMesh);
				world->AddBody(dynBody);
			}
		};
		loader.m_mesh->NodeIterator(ProcessMeshNodes);
    }

    scene->SetCameraMatrix(cameraTransform.m_rotation, cameraTransform.m_position);
}
