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
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"

//#define D_COLLISION_UPDATE_ONLY

class ndApplyImpulse : public ndDemoEntityNotify
{
	public:
	ndApplyImpulse(ndDemoEntityManager* const manager, const ndSharedPtr<ndRenderSceneNode>& entity)
		:ndDemoEntityNotify(manager, entity)
		,m_addImpulse(false)
	{
	}

	void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep) override
	{
		ndDemoEntityNotify::OnApplyExternalForce(threadIndex, timestep);
		if (m_addImpulse)
		{
			ndBodyKinematic* const body = GetBody()->GetAsBodyKinematic();
			if (body->GetInvMass() > 0.0f)
			{
				ndVector upPush(ndVector::m_zero);
				upPush.m_y = ndFloat32(5.0f) / body->GetInvMass();
				upPush.m_z = ndFloat32(2.0f) / body->GetInvMass();
				body->ApplyImpulsePair(upPush, ndVector::m_zero, timestep);
			}
		}

		m_addImpulse = false;
	}

	bool m_addImpulse;
};

class ndBuildWallPanel : public ndDemoEntityManager::ndDemoUIpanel
{
	public:
	ndBuildWallPanel(ndApplyImpulse* const impulse)
		:ndDemoUIpanel()
		,m_applyImpulse(impulse)
	{
	}

	void BuildPyramid(ndDemoEntityManager* const scene)
	{
		ndMatrix matrix(ndGetIdentityMatrix());
		matrix.m_posit.m_x = 4.0f;
		matrix.m_posit.m_z = 4.0f;
		ndVector boxSize(0.5f, 0.25f, 0.8f, 0.0f);

		// create the shape and visual mesh as a common data to be re used
		ndFloat32 stepz = boxSize.m_z + 1.0e-2f;
		ndFloat32 stepy = boxSize.m_y + 1.0e-2f;
		stepy = boxSize.m_y;

		ndInt32 count = 10;
		ndFloat32 y0 = matrix.m_posit.m_y;
		ndFloat32 z0 = matrix.m_posit.m_z - stepz * (ndFloat32)count / 2;

		matrix.m_posit.m_y = y0;
		matrix.m_posit.m_y -= 0.01f;

		ndShapeInstance shape(new ndShapeBox(boxSize.m_x, boxSize.m_y, boxSize.m_z));

		ndWorld* const world = scene->GetWorld();
		for (ndInt32 j = 0; j < count; ++j)
		{
			matrix.m_posit.m_z = z0;
			matrix = FindFloor(*world, matrix, shape, 100.0f);
			for (ndInt32 i = 0; i < (count - j); ++i)
			{
				m_wall.Append(AddBox(scene, matrix, 1.0f, boxSize.m_x, boxSize.m_y, boxSize.m_z, "wood_2.png"));
				matrix.m_posit.m_z += stepz;
			}
			z0 += stepz * 0.5f;
		}
	}

	virtual void Update(ndDemoEntityManager* const scene) override
	{
		ndVector color(1.0f, 1.0f, 0.0f, 0.0f);

		if (ImGui::Button("ApplyPush"))
		{
			m_applyImpulse->m_addImpulse = true;
		}

		if (ImGui::Button("BuildWall"))
		{
			if (!m_wall.GetCount())
			{
				BuildPyramid(scene);
			}
		}
		if (ImGui::Button("DeleteWall"))
		{
			ndPhysicsWorld* const world = scene->GetWorld();
			for (ndList<ndSharedPtr<ndBody>>::ndNode* node = m_wall.GetFirst(); node; node = node->GetNext())
			{
				// get the rigid body
				ndSharedPtr<ndBody> body (node->GetInfo());

				// get the visual mesh for the notification
				ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)*body->GetNotifyCallback();
				ndSharedPtr<ndRenderSceneNode> sceneNode (notify->GetUserData());

				// remove body from world
				world->DefferedRemoveBody(body);

				// remove visual mesh from world
				world->DefferedRemoveSceneNode(sceneNode);
			}
			m_wall.RemoveAll();
		}
	}

	ndList<ndSharedPtr<ndBody>> m_wall;
	ndWeakPtr<ndApplyImpulse> m_applyImpulse;
};

void ndBasicCollisionOnly(ndDemoEntityManager* const scene)
{
	// build a floor
	ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	ndMatrix matrix(ndGetIdentityMatrix());
	ndSharedPtr<ndBody> testBody(AddBox(scene, matrix, 1.0f, 1.0f, 1.0f, 1.0f, "wood_0.png"));
	ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)*testBody->GetNotifyCallback();
	ndSharedPtr<ndBodyNotify> impulseNotify(new ndApplyImpulse(scene, notify->GetUserData()));
	testBody->SetNotifyCallback(impulseNotify);

#ifdef D_COLLISION_UPDATE_ONLY
	// set the physics as collision only mode
	scene->GetWorld()->SetUpdateMode(true);
#endif

	// add an UI panel for recreating Steve Hurley's crash
	ndSharedPtr<ndDemoEntityManager::ndDemoUIpanel> controlPanel(new ndBuildWallPanel((ndApplyImpulse*)*testBody->GetNotifyCallback()));
	scene->SetDemoUIpanel(controlPanel);

	matrix.m_posit.m_x -= 8.0f;
	matrix.m_posit.m_y += 2.0f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
