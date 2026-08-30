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
#include "ndUnicyclePlayer.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"

namespace ndUnicyclePlayer
{
	class ndHelpLegend_Sac : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "Pre-trained inverted pendulum environment.");
			scene->Print(color, "This is a classic reinforcement learning benchmark,");
			scene->Print(color, "commonly used to validate algorithm implementations.");
			scene->Print(color, "The model was trained using the Soft Actor-Critic (SAC) algorithm.");
			scene->Print(color, "The system consists of a heavy box attached to a pole,");
			scene->Print(color, "than can swing freely around a pivot over the box center of mass");
			scene->Print(color, "the pole has a wheel at the end that rolls by applying torque.");
			scene->Print(color, "The objective is to train a neural network to keep");
			scene->Print(color, "the pole balanced in an upright position.");
			scene->Print(color, "You can interact with the simulation");
			scene->Print(color, "and try to knock the pole over using the mouse.");
		}
	};
	
	class ndHelpLegend_Ppo : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "unicycle is a typical reinforcement learning");
			scene->Print(color, "It is used to test the correctness of an algorithm implementation.");
			scene->Print(color, "The model is trained using Proximal Policy Gradient (PPO).");
			scene->Print(color, "It consists of a pole attached by a hinge to a sliding cart.");
			scene->Print(color, "The objective goal was to train a neural network to keep");
			scene->Print(color, "the pole balanced in an upright position.");
			scene->Print(color, "You can interact with the simulation and try.");
			scene->Print(color, "to knock the pole over using the mouse.");
		}
	};

	class ndPlaybackController : public ndController
	{
		public:
		ndPlaybackController()
			:ndController()
		{
		}
	};

	ndController::ndController()
		:ndModelNotify()
		,m_solver()
		,m_agent(nullptr)
		,m_timestep(0.0f)
		,m_isTrainning(false)
	{
	}

	void ndController::Update(ndFloat32 timestep, ndInt32)
	{
		m_timestep = timestep;
		m_agent->Step();
	}

	//#pragma optimize( "", off )
	void ndController::PostUpdate(ndFloat32, ndInt32)
	{
		ndMatrix matrix (m_topBox->GetMatrix());
		//if (ndAbs(matrix.m_posit.m_x) > 300.0f)
		if (ndAbs(matrix.m_posit.m_x) > 30.0f)
		{
			matrix.m_posit.m_x = ndFloat32(0.0f);
			GetModel()->GetAsModelArticulation()->SetTransform(matrix);
		}
	}

	void ndController::ResetModel()
	{
		// reset the model to it base pose
		for (ndInt32 i = 0; i < m_basePose.GetCount(); i++)
		{
			m_basePose[i].SetPose();
		}

		// clear joint internal states
		GetModel()->GetAsModelArticulation()->ClearMemory();

		// calculate a new random placement transform, 
		// and teleport the model to that location
		ndFloat32 angle = (ndRand() - ndFloat32(0.5f)) * ndFloat32(10.0f);
		ndMatrix spawnMatrix(ndRollMatrix(angle * ndDegreeToRad));
		spawnMatrix.m_posit = m_topBox->GetMatrix().m_posit;

		ndFloat32 randX = ndFloat32(20.0f) * (ndRand() - ndFloat32(0.5f));
		spawnMatrix.m_posit.m_x = randX;
	
		GetModel()->GetAsModelArticulation()->SetTransform(spawnMatrix);
	}
		
	// calculate pole angle relative to the world.
	ndFloat32 ndController::GetPoleAngle() const
	{
		const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		const ndMatrix matrix(hinge->CalculateGlobalMatrix0());
		ndFloat32 angle = ndAtan2(matrix.m_up.m_x, matrix.m_up.m_y);
		return angle;
	}

	ndFloat32 ndController::GetPoleOmega() const
	{
		const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		const ndMatrix matrix(hinge->CalculateGlobalMatrix0());
		ndFloat32 omega = matrix.m_front.DotProduct(m_poleHinge->GetBody0()->GetOmega()).GetScalar();
		return omega;
	}

	//ndFloat32 ndController::GetWheelOmega() const
	//{
	//	const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
	//	const ndMatrix matrix(hinge->CalculateGlobalMatrix1());
	//	ndFloat32 omega = matrix.m_front.DotProduct(m_poleHinge->GetBody1()->GetOmega()).GetScalar();
	//	return omega;
	//}

	ndFloat32 ndController::GetWheelAlpha() const
	{
		const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		const ndMatrix matrix(hinge->CalculateGlobalMatrix1());
		ndFloat32 omega = matrix.m_front.DotProduct(m_poleHinge->GetBody1()->GetAlpha()).GetScalar();
		return omega;
	}

	ndFloat32 ndController::GetBoxAngle() const
	{
		const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		const ndMatrix matrix(hinge->CalculateGlobalMatrix1());
		ndFloat32 angle = ndAtan2(matrix.m_up.m_x, matrix.m_up.m_y);
		return angle;
	}

	ndFloat32 ndController::GetBoxOmega() const
	{
		const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		const ndMatrix matrix(hinge->CalculateGlobalMatrix1());
		const ndVector omega(m_topBox->GetOmega());
		return omega.DotProduct(matrix.m_front).GetScalar();
	}

	//#pragma optimize( "", off )
	bool ndController::IsTerminal() const
	{
		bool fail = ndAbs(GetPoleAngle()) > ND_TERMINATION_ANGLE;
		fail = fail || (ndAbs(GetBoxAngle()) > ndFloat32 (60.0f) * ndDegreeToRad);
		return fail;
	}

	#pragma optimize( "", off )
	ndBrainFloat ndController::CalculateReward() const
	{
		if (IsTerminal())
		{
			return ndBrainFloat(-1.0f);
		}
		const ndMatrix comFrame(ndYawMatrix(ndFloat32(90.0f) * ndDegreeToRad));

		auto PolynomialOmegaReward = [](ndFloat32 omega)
		{
			ndFloat32 maxOmega = 2.0f;
			omega = ndClamp(omega, -maxOmega, maxOmega);
			ndFloat32 r = ndFloat32(1.0f) - ndAbs(omega) / maxOmega;
			ndFloat32 reward = r * r * r * r;
			return reward;
		};

		auto PolynomialAccelerationReward = [](ndFloat32 alpha)
		{
			ndFloat32 maxAlpha = 5.0f;
			alpha = ndClamp(alpha, -maxAlpha, maxAlpha);
			ndFloat32 r = ndFloat32(1.0f) - ndAbs(alpha) / maxAlpha;
			ndFloat32 reward = r * r;
			return reward;
		};
		ndFixSizeArray<ndJointBilateralConstraint*, D_INV_IK_MAX_LINKS> extraJoint;
		const ndModelArticulation::ndCenterOfMassDynamics comDynamics(GetModel()->GetAsModelArticulation()->CalculateCentreOfMassDynamics(m_solver, comFrame, extraJoint, m_timestep));
		const ndVector comOmega(comDynamics.m_omega);
		const ndVector comAlpha(comDynamics.m_alpha);

		ndVector veloc(m_topBox->GetVelocity());
		ndFloat32 speedReward = ndExp(-100.0f * veloc.m_x * veloc.m_x);

		ndFloat32 omegaReward = PolynomialOmegaReward(comOmega.m_x);
		ndFloat32 alphaReward = PolynomialAccelerationReward(comAlpha.m_x);
		ndFloat32 reward = ndFloat32(0.2f) * speedReward + ndFloat32(0.4f) * omegaReward + ndFloat32(0.4f) * alphaReward;

		if (IsOnAir())
		{
			// penalize air borne high angular velocity
			const ndMatrix wheelMatrix(m_wheelRoller->CalculateGlobalMatrix0());
			const ndVector wheelOmega(m_wheel->GetOmega());
			ndFloat32 speed = (wheelOmega.DotProduct(wheelMatrix.m_front)).GetScalar();

			ndFloat32 arg = ndFloat32 (-0.5f) * speed * speed;
			reward = ndExp(arg);
		}

		return ndBrainFloat(reward);
	}

	#pragma optimize( "", off )
	void ndController::ApplyActions(ndBrainFloat* const actions)
	{
		const ndVector wheelMass(m_wheel->GetAsBodyDynamic()->GetMassMatrix());
		const ndMatrix wheelMatrix(m_wheelRoller->CalculateGlobalMatrix0());

		const ndVector wheelOmega(m_wheel->GetOmega());
		ndFloat32 speed = (wheelOmega.DotProduct(wheelMatrix.m_front)).GetScalar();

		ndFloat32 drag = ndFloat32(0.25f) * speed * speed * ndSign(speed);
		ndFloat32 wheelTorque = wheelMass.m_z * actions[m_wheelTorque] * ND_MAX_WHEEL_ALPHA;

		//ndExpandTraceMessage("%g %g %g\n", speed, drag, wheelTorque);

		ndVector torque(wheelMatrix.m_front.Scale(wheelTorque - drag));
		m_wheel->GetAsBodyDynamic()->SetTorque(torque);
	}

	ndBrainFloat ndController::IsOnAir() const
	{
		ndBodyKinematic::ndContactMap& contacts = m_wheel->GetAsBodyKinematic()->GetContactMap();
		ndBodyKinematic::ndContactMap::Iterator it(contacts);
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = *it;
			if (contact->IsActive())
			{
				const ndContactPointList& contactPoints = contact->GetContactPoints();
				return contactPoints.GetCount() ? ndBrainFloat(0.0f) : ndBrainFloat(1.0f);
			}
		}
		return ndBrainFloat(1.0f);
	};

	#pragma optimize( "", off)
	void ndController::GetObservation(ndBrainFloat* const observation)
	{
		const ndMatrix poleMatrix(m_wheelRoller->CalculateGlobalMatrix1());
		const ndVector planeDir(poleMatrix.m_front);

		ndFloat32 poleAngle = ndAtan2(poleMatrix.m_up.m_x, poleMatrix.m_up.m_y);
		ndFloat32 poleOmega = planeDir.DotProduct(m_pole->GetOmega()).GetScalar();
		ndFloat32 wheelOmega = planeDir.DotProduct(m_wheel->GetOmega()).GetScalar();
		ndFloat32 speed = m_topBox->GetVelocity().m_x;

		observation[m_velocity] = ndBrainFloat(speed);
		observation[m_poleAngle] = ndBrainFloat(poleAngle);
		observation[m_poleOmega] = ndBrainFloat(poleOmega);
		observation[m_wheelOmega] = ndBrainFloat(wheelOmega);
		observation[m_hasSupportContact] = IsOnAir();
	}

	void ndController::CreateArticulatedModel(
		ndDemoEntityManager* const scene,
		ndModelArticulation* const model,
		ndSharedPtr<ndMesh> mesh,
		ndSharedPtr<ndRenderSceneNode> visualMesh)
	{
		model->GetAsModelArticulation()->Deserialize(*mesh);

		auto BindApplicationData = [this, scene, &visualMesh](ndModelArticulation::ndNode* const node)
		{
			ndRenderSceneNode* const visualEntityPtr = visualMesh->FindByClosestMatch(node->m_name);
			ndAssert(visualEntityPtr);
			ndSharedPtr<ndRenderSceneNode> visualEntity((visualEntityPtr == *visualMesh) ? visualMesh : visualEntityPtr->GetSharedPtr());

			// add a rigid body with notification callback
			ndBodyKinematic* const parentBody = node->GetParent() ? node->GetParent()->m_body->GetAsBodyKinematic() : nullptr;
			ndSharedPtr<ndBodyNotify> notify(new ndDemoEntityNotify(scene, visualEntity, parentBody));
			node->m_body->SetNotifyCallback(notify);

			ndBasePose pose(node->m_body->GetAsBodyDynamic());
			m_basePose.PushBack(pose);

			ndShapeMaterial& material = node->m_body->GetAsBodyDynamic()->GetCollisionShape().m_shapeMaterial;
			material.m_userId = ndDemoContactCallback::m_modelPart;

			if (node->m_name.Find("Box001-rb-box") > -1)
			{
				m_topBox = node->m_body;
			}
			else if (node->m_name.Find("pole-rb-convexhull-hinge") > -1)
			{
				m_pole = node->m_body;
				m_poleHinge = node->m_joint;
			}
			else if (node->m_name.Find("ball-rb-sphere-roller") > -1)
			{
				m_wheel = node->m_body;
				m_wheelRoller = node->m_joint;
			}
		};
		model->GetAsModelArticulation()->NodeIterator(BindApplicationData);

		// fix to the word with a plane joint
		ndWorld* const world = scene->GetWorld();
		const ndMatrix planeMatrix(m_poleHinge->CalculateGlobalMatrix1());
		m_plane = ndSharedPtr<ndJointBilateralConstraint>(new ndJointPlane(planeMatrix.m_posit, planeMatrix.m_front, m_topBox->GetAsBodyKinematic(), world->GetSentinelBody()));
		model->AddCloseLoop(m_plane);
	}

	ndModelArticulation* ndController::CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndRenderMeshLoader& loader, const char* const name)
	{
		ndMatrix matrix(location);
		matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);
		matrix.m_posit.m_y += ndFloat32(0.1f);
		loader.m_mesh->SetMatrix(loader.m_mesh->GetMatrix() * matrix);
		
		ndSharedPtr<ndRenderSceneNode> visualMesh(loader.m_renderMesh->Clone());
		visualMesh->SetTransform(loader.m_mesh->GetMatrix());
		visualMesh->SetTransform(loader.m_mesh->GetMatrix());
		
		ndModelArticulation* const model = new ndModelArticulation();
		ndSharedPtr<ndModelNotify> controller(new ndPlaybackController());
		model->SetNotifyCallback(controller);
		ndPlaybackController* const playerController = (ndPlaybackController*)(*controller);
		playerController->CreateArticulatedModel(scene, model, loader.m_mesh, visualMesh);

		char nameExt[256];
		snprintf(nameExt, sizeof(nameExt) - 1, "%s.dnn", name);
		ndString fileName(ndGetWorkingFileName(nameExt));
		ndSharedPtr<ndBrain> policy(ndBrainLoad::Load(fileName.GetStr()));
		playerController->m_agent = ndSharedPtr<ndBrainAgent>(new ndController::ndAgent(policy, playerController));

		// add model a visual mesh to the scene and world
		ndWorld* const world = scene->GetWorld();
		world->AddModel(model);
		scene->AddEntity(visualMesh);
		return model;
	}
}
using namespace ndUnicyclePlayer;

void ndUnicyclePlayer_SAC(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend_Sac());
	scene->SetDemoHelp(demoHelper);

	// oveload the ground friction
	// make sure the ground has enough friction
	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	ndMaterial* const defaultMaterial = callback->GetMaterial(ndDemoContactCallback::m_default, ndDemoContactCallback::m_default);
	ndAssert(defaultMaterial);
	defaultMaterial->m_dynamicFriction0 = defaultMaterial->m_staticFriction0;
	defaultMaterial->m_dynamicFriction1 = defaultMaterial->m_staticFriction1;

	//ndModelMaterial material;
	//callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_modelPart);

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("unicycle.nd"));
	ndController::CreateModel(scene, matrix, loader, CONTROLLER_NAME_SAC);

	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += -9.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), -90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}

void ndUnicyclePlayer_PPO(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend_Ppo());
	scene->SetDemoHelp(demoHelper);

	// oveload the ground friction
	// make sure the ground has enough friction
	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	ndMaterial* const defaultMaterial = callback->GetMaterial(ndDemoContactCallback::m_default, ndDemoContactCallback::m_default);
	ndAssert(defaultMaterial);
	defaultMaterial->m_dynamicFriction0 = defaultMaterial->m_staticFriction0;
	defaultMaterial->m_dynamicFriction1 = defaultMaterial->m_staticFriction1;

	//ndModelMaterial material;
	//callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_modelPart);

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("unicycle.nd"));
	ndController::CreateModel(scene, matrix, loader, CONTROLLER_NAME_PPO);

	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += -9.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), -90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
