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
#include "ndCartpolePlayer.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"

namespace ndCarpolePlayer
{
	class ndHelpLegend_Sac : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "Cart Pole is the classic hello world of reinforcement learning");
			scene->Print(color, "it is use to test the correctness of an algorithm implementation.");
			scene->Print(color, "The model is trained using Soft Actor Critic(SAC).");
			scene->Print(color, "It consists of a pole attached by a hinge to a sliding cart.");
			scene->Print(color, "The objective goal was to train a neural network to keep");
			scene->Print(color, "the pole balanced in an upright position.");
			scene->Print(color, "You can interact with the simulation and try.");
			scene->Print(color, "to knock the pole over using the mouse.");
		}
	};
	
	class ndHelpLegend_Ppo : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "Cart Pole is the classic hello world of reinforcement learning");
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
		,m_agent(nullptr)
		,m_timestep(0.0f)
		,m_randomImpulseCounter(ndInt32(1 + (ndRandInt() % 200)))
		,m_isTrainning(false)
	{
	}

	void ndController::Update(ndFloat32 timestep, ndInt32)
	{
		m_timestep = timestep;
		m_agent->Step();
	}

	void ndController::PostUpdate(ndFloat32, ndInt32)
	{
		m_randomImpulseCounter = (m_randomImpulseCounter + 1) % ND_RANDOM_IMPULSE_MOD;
	}

	void ndController::ResetModel()
	{
		ndMatrix cartMatrix(ndGetIdentityMatrix());
		cartMatrix.m_posit = m_cart->GetMatrix().m_posit;
		cartMatrix.m_posit.m_x = ndFloat32(0.0f);
		cartMatrix.m_posit.m_x = ndFloat32(10.0f) * (ndRand() - ndFloat32(0.5f));
		cartMatrix.m_posit.m_y = ndFloat32(0.1f);
		m_cart->SetMatrix(cartMatrix);

		const ndMatrix poleMatrix(m_poleHinge->CalculateGlobalMatrix1());
		m_pole->SetMatrix(poleMatrix);

		GetModel()->GetAsModelArticulation()->ClearMemory();
	}

	bool ndController::IsTerminal() const
	{
		const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		const ndJointSlider* const slider = (ndJointSlider*)*m_slider;
		ndFloat32 angle = hinge->GetAngle();
		ndFloat32 speed = slider->GetSpeed();
		bool isdead = ndAbs(angle) > (REWARD_MIN_ANGLE * ndFloat32(2.0f));
		isdead = isdead || (ndAbs(speed) > REWARD_MAX_SPEED);
		return isdead;
	}

	ndBrainFloat ndController::CalculateReward() const
	{
		if (IsTerminal())
		{
			// a terminal reward of zero should make for smoother MDPs. 
			// training small networks could be much harder with negative terminal rewards..
			return ndBrainFloat(-1.0f);
		}

		ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		ndJointSlider* const slider = (ndJointSlider*)*m_slider;

		ndFloat32 omega = hinge->GetOmega();
		ndFloat32 angle = hinge->GetAngle() / REWARD_MIN_ANGLE;
		ndFloat32 speed = slider->GetSpeed() / REWARD_MAX_SPEED;

		ndFloat32 invSigma2 = ndFloat32(20.0f);
		ndFloat32 speedReward = ndExp(-100.0f * speed * speed);
		ndFloat32 omegaReward = ndExp(-invSigma2 * omega * omega);
		ndFloat32 angleReward = ndExp(-invSigma2 * angle * angle);

		// make sure the reward is never negative, 
		// to avoid the possibility of MDP states with negative values.
		ndFloat32 reward = ndFloat32(0.3f) * angleReward + ndFloat32(0.3f) * omegaReward + ndFloat32(0.4f) * speedReward;
		return ndBrainFloat(reward);
	}

	void ndController::ApplyActions(ndBrainFloat* const actions)
	{
		ndBrainFloat action = actions[0];
		ndBrainFloat accel = PUSH_ACCEL * action;
		ndFloat32 pushForce = accel * (m_cart->GetAsBodyDynamic()->GetMassMatrix().m_w);

		ndJointSlider* const slider = (ndJointSlider*)*m_slider;
		const ndMatrix matrix(slider->CalculateGlobalMatrix0());

		ndVector force(m_cart->GetAsBodyDynamic()->GetForce() + matrix.m_front.Scale(pushForce));
		m_cart->GetAsBodyDynamic()->SetForce(force);

		if (m_isTrainning && (m_randomImpulseCounter == 0))
		{
			// when in training mode,
			// apply a random impulse to the top box every m_randomImpulseCounter steps
			ndFloat32 randSpeed = ND_RANDOM_IMPULSE_MAGNITUD * (ndFloat32(0.5f) - ndRand());
			const ndVector mass(m_cart->GetAsBodyDynamic()->GetMassMatrix());
			const ndVector randomImpulse(matrix.m_front.Scale(mass.m_w * randSpeed));
			m_cart->GetAsBodyDynamic()->ApplyImpulsePair(randomImpulse, ndVector::m_zero, m_timestep);
		}
	}

	void ndController::GetObservation(ndBrainFloat* const observation)
	{
		const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		const ndJointSlider* const slider = (ndJointSlider*)*m_slider;

		ndFloat32 omega = hinge->GetOmega();
		ndFloat32 angle = hinge->GetAngle();
		ndFloat32 speed = slider->GetSpeed();

		observation[m_poleAngle] = ndBrainFloat(angle);
		observation[m_poleOmega] = ndBrainFloat(omega);
		observation[m_cartSpeed] = ndBrainFloat(speed);
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

			if (node->m_name.Find("cart") > -1)
			{ 
				m_cart = *node->m_body;
			}
			else if (node->m_name.Find("pole") > -1)
			{
				m_pole = *node->m_body;
				m_poleHinge = *node->m_joint;
			}
		};
		model->GetAsModelArticulation()->NodeIterator(BindApplicationData);

		ndWorld* const world = scene->GetWorld();
		const ndMatrix sliderMatrix(model->GetRoot()->m_body->GetMatrix());
		m_slider = new ndJointSlider(sliderMatrix, model->GetRoot()->m_body->GetAsBodyKinematic(), world->GetSentinelBody());
		model->AddCloseLoop(ndSharedPtr<ndJointBilateralConstraint>(*m_slider));
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
using namespace ndCarpolePlayer;

void ndCartpolePlayer_SAC(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend_Sac());
	scene->SetDemoHelp(demoHelper);

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("cartpole.nd"));
	ndController::CreateModel(scene, matrix, loader, CONTROLLER_NAME_SAC);

	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}

void ndCartpolePlayer_PPO(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend_Ppo());
	scene->SetDemoHelp(demoHelper);

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("cartpole.nd"));
	ndController::CreateModel(scene, matrix, loader, CONTROLLER_NAME_PPO);

	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}