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
#include "ndMeshLoader.h"
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
			scene->Print(color, "unicycle is a typical reinforcement learning");
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
		,m_agent(nullptr)
		,m_timestep(0.0f)
	{
	}

	void ndController::Update(ndFloat32 timestep)
	{
		m_timestep = timestep;
		m_agent->Step();
	}

	void ndController::ResetModel()
	{
		ndMatrix boxMatrix(ndGetIdentityMatrix());
		boxMatrix.m_posit = m_topBox->GetMatrix().m_posit;
		boxMatrix.m_posit.m_x = ndFloat32(0.0f);
		//boxMatrix.m_posit.m_y = ndFloat32(2.0f) + ndFloat32(2.0f) * ndRand();
		boxMatrix.m_posit.m_y = ndFloat32(2.5f);
		m_topBox->SetMatrix(boxMatrix);
		
		const ndMatrix poleMatrix(m_poleHinge->GetLocalMatrix0().OrthoInverse() * m_poleHinge->CalculateGlobalMatrix1());
		m_pole->SetMatrix(poleMatrix);

		const ndMatrix ballMatrix(m_wheelRoller->GetLocalMatrix0().OrthoInverse() * m_wheelRoller->CalculateGlobalMatrix1());
		m_wheel->SetMatrix(ballMatrix);
		
		m_pole->SetOmega(ndVector::m_zero);
		m_pole->SetVelocity(ndVector::m_zero);
		
		m_topBox->SetOmega(ndVector::m_zero);
		m_topBox->SetVelocity(ndVector::m_zero);

		m_wheel->SetOmega(ndVector::m_zero);
		m_wheel->SetVelocity(ndVector::m_zero);

		GetModel()->GetAsModelArticulation()->ClearMemory();

		// randomize start position 
		ndFloat32 angle = ndDegreeToRad * (ndRand() - 0.5f) * 10.0f;
		ndMatrix rollMatrix(ndRollMatrix(angle));
		ndMatrix matrix (rollMatrix * boxMatrix);
		GetModel()->GetAsModelArticulation()->SetTransform(matrix);
	}

	// calculate pole omega relative to the world.
	ndFloat32 ndController::GetPoleOmega() const
	{
		const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		const ndMatrix matrix(hinge->CalculateGlobalMatrix0());
		const ndVector omega(m_pole->GetOmega());
		return omega.DotProduct(matrix.m_front).GetScalar();
	}

	// calculate pole angle relative to the world.
	ndFloat32 ndController::GetPoleAngle() const
	{
		const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		const ndMatrix matrix(hinge->CalculateGlobalMatrix0());
		ndFloat32 angle = ndAcos(-ndClamp(matrix.m_up.m_y, ndFloat32(-1.0f), ndFloat32(1.0f)));
		return angle;
	}

	//// calculate box angle relative to the world.
	//ndFloat32 ndController::GetBoxAngle() const
	//{
	//	const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
	//	const ndMatrix matrix(hinge->CalculateGlobalMatrix1());
	//	ndFloat32 angle = ndAcos(-ndClamp(matrix.m_up.m_y, ndFloat32(-1.0f), ndFloat32(1.0f)));
	//	return angle;
	//}
	//
	//// calculate box omega relative to the world.
	//ndFloat32 ndController::GetBoxOmega() const
	//{
	//	const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
	//	const ndMatrix matrix(hinge->CalculateGlobalMatrix0());
	//	const ndVector omega(m_topBox->GetOmega());
	//	return omega.DotProduct(matrix.m_front).GetScalar();
	//}

	bool ndController::IsTerminal() const
	{
		ndFloat32 angle = GetPoleAngle();
		bool fail = ndAbs(angle) > ND_TERMINATION_ANGLE;
		return fail;
	}

	#pragma optimize( "", off )
	ndBrainFloat ndController::CalculateReward() const
	{
		if (IsTerminal())
		{
			// a terminal reward of zero should make for smoother MDPs. 
			// training small networks could be much harder with negative terminal rewards..
			return ndBrainFloat(-1.0f);
		}

#if 0
		// objective function that use heuristic intution.
		// it converge by tent to generate a weak controller
		ndFloat32 boxAngle = GetBoxAngle();
		ndFloat32 boxOmega = GetBoxOmega();
		ndFloat32 poleAngle = GetPoleAngle();
		ndFloat32 poleOmega = GetPoleOmega();
		ndVector veloc(m_topBox->GetVelocity());

		ndFloat32 speedReward = ndExp(-100.0f * veloc.m_x * veloc.m_x);
		ndFloat32 boxAngleReward = ndExp(-500.0f * boxAngle * boxAngle);
		ndFloat32 boxOmegaReward = ndExp(-100.0f * boxOmega * boxOmega);
		ndFloat32 poleAngleReward = ndExp(-500.0f * poleAngle * poleAngle);
		ndFloat32 poleOmegaReward = ndExp(-100.0f * poleOmega * poleOmega);

		if (IsOnAir())
		{
			boxAngleReward = 0.0f;
			boxOmegaReward = 0.0f;
			poleAngleReward = 0.0f;
			poleOmegaReward = 0.0f;
		}
		//ndTrace(("%f %f %f\n", poleAngleReward, poleOmegaReward, speedReward));
		ndFloat32 reward = 
			ndFloat32(1.0 / 2.0f) * boxAngleReward +
			ndFloat32(1.0 / 2.0f) * boxOmegaReward +
			ndFloat32(1.0 / 2.0f) * poleAngleReward +
			ndFloat32(1.0 / 2.0f) * poleOmegaReward +
			ndFloat32(1.0 / 2.0f) * speedReward;

#else

		// trying with center of mass dynammics
		// but the result so far the results are very dissapointing
		// this however word much better is an order version 
		// maybe I have bugs that I have to track

		if (IsTerminal())
		{
			return ndBrainFloat(-1.0f);
		}

		ndMatrix comFrame(m_wheelRoller->CalculateGlobalMatrix1());
		comFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
		comFrame.m_right = comFrame.m_front.CrossProduct(comFrame.m_up).Normalize();
		comFrame.m_up = comFrame.m_right.CrossProduct(comFrame.m_front).Normalize();
		ndModelArticulation::ndCenterOfMassDynamics comDynamics(GetModel()->GetAsModelArticulation()->CalculateCentreOfMassDynamics(comFrame));
		//ndModelArticulation::ndCenterOfMassDynamics comKinematic(GetModel()->GetAsModelArticulation()->CalculateCentreOfMassDynamics(comFrame));

		ndFloat32 omegaReward = ndExp(-0.5f * comDynamics.m_omega.m_x * comDynamics.m_omega.m_x);
		ndFloat32 speedReward = ndExp(-4.0f * comDynamics.m_veloc.m_z * comDynamics.m_veloc.m_z);
		if (IsOnAir())
		{
			speedReward = ndFloat32(0.0f);
		}
		ndFloat32 reward = ndFloat32(0.8f) * omegaReward + ndFloat32(0.2f) * speedReward;

#endif
		return ndBrainFloat(reward);
	}

	void ndController::ApplyActions(ndBrainFloat* const actions)
	{
		const ndVector wheelMass(m_wheel->GetAsBodyDynamic()->GetMassMatrix());
		const ndMatrix wheelMatrix(m_wheelRoller->CalculateGlobalMatrix0());

		ndFloat32 wheelTorque = wheelMass.m_z * actions[m_wheelTorque] * ND_MAX_WHEEL_ALPHA;
		if (IsOnAir())
		{
			ndFloat32 omega = m_wheel->GetOmega().m_z;
			ndFloat32 drag = ndFloat32(0.1f) * omega * omega * ndSign(omega);
			wheelTorque = -drag;
		}

		//ndExpandTraceMessage("%g %g %g\n", speed, drag, wheelTorque);
		ndVector torque(wheelMatrix.m_front.Scale(wheelTorque));
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

	void ndController::GetObservation(ndBrainFloat* const observation)
	{
		//ndFloat32 poleJointOmega = ((ndJointHinge*)*m_poleHinge)->GetOmega();
		//ndFloat32 poleJointAngle = ((ndJointHinge*)*m_poleHinge)->GetAngle() / ND_MAX_LEG_JOINT_ANGLE;
		//ndFloat32 wheelOmega = ((ndJointRoller*)*m_wheelRoller)->GetOmega() / ndFloat32(20.0f);
		//ndFloat32 wheelSpeed = ndClamp(m_wheel->GetVelocity().m_x, ndFloat32(-6.0f), ndFloat32(6.0f)) / ndFloat32 (6.0f);

		//ndFloat32 comSpeed = m_wheel->GetVelocity().m_x;
		ndMatrix comFrame(m_wheelRoller->CalculateGlobalMatrix1());
		comFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
		comFrame.m_right = comFrame.m_front.CrossProduct(comFrame.m_up).Normalize();
		comFrame.m_up = comFrame.m_right.CrossProduct(comFrame.m_front).Normalize();
		ndModelArticulation::ndCenterOfMassDynamics comKinematics(GetModel()->GetAsModelArticulation()->CalculateCentreOfMassKinematics(comFrame));

		ndFloat32 comSpeed = comKinematics.m_veloc.m_z;
		ndFloat32 wheelOmega = ((ndJointRoller*)*m_wheelRoller)->GetOmega();

		ndFloat32 poleOmega = GetPoleOmega();
		ndFloat32 poleAngle = GetPoleAngle() / ND_MAX_LEG_JOINT_ANGLE;
		
		observation[m_comSpeed] = ndBrainFloat(comSpeed);
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
		auto CreateRigidBody = [scene](ndSharedPtr<ndMesh>& mesh, ndSharedPtr<ndRenderSceneNode>& visualMesh, ndFloat32 mass, ndBodyDynamic* const parentBody)
		{
			ndSharedPtr<ndShapeInstance> shape(mesh->CreateCollision());

			ndBodyKinematic* const body = new ndBodyDynamic();
			body->SetNotifyCallback(new ndDemoEntityNotify(scene, visualMesh, parentBody));
			body->SetMatrix(mesh->CalculateGlobalMatrix());
			body->SetCollisionShape(*(*shape));
			body->GetAsBodyDynamic()->SetMassMatrix(mass, *(*shape));
			return body;
		};

		// add the root body
		m_topBox = ndSharedPtr<ndBody>(CreateRigidBody(mesh, visualMesh, BOX_MASS, nullptr));
		ndModelArticulation::ndNode* const modelRootNode = model->AddRootBody(m_topBox);

		// add the pole mesh and body
		ndSharedPtr<ndMesh> poleMesh(mesh->GetChildren().GetFirst()->GetInfo());
		ndSharedPtr<ndRenderSceneNode> poleEntity(visualMesh->GetChildren().GetFirst()->GetInfo());
		m_pole = ndSharedPtr<ndBody>(CreateRigidBody(poleMesh, poleEntity, POLE_MASS, m_topBox->GetAsBodyDynamic()));

		// add ball mesh and body
		ndSharedPtr<ndMesh> ballMesh(poleMesh->GetChildren().GetFirst()->GetInfo());
		ndSharedPtr<ndRenderSceneNode> ballEntity(poleEntity->GetChildren().GetFirst()->GetInfo());
		m_wheel = ndSharedPtr<ndBody>(CreateRigidBody(ballMesh, ballEntity, BALL_MASS, m_pole->GetAsBodyDynamic()));

		// add links
		const ndMatrix poleMatrix(m_pole->GetMatrix());
		m_poleHinge = ndSharedPtr<ndJointBilateralConstraint>(new ndJointHinge(poleMatrix, m_pole->GetAsBodyKinematic(), m_topBox->GetAsBodyKinematic()));
		ndModelArticulation::ndNode* const poleNode = model->AddLimb(modelRootNode, m_pole, m_poleHinge);

		const ndMatrix ballMatrix(m_wheel->GetMatrix());
		m_wheelRoller = ndSharedPtr<ndJointBilateralConstraint>(new ndJointRoller(ballMatrix, m_wheel->GetAsBodyKinematic(), m_pole->GetAsBodyKinematic()));
		((ndJointRoller*)*m_wheelRoller)->SetAsSpringDamperPosit(0.01f, 1000.0f, 15.0f);
		model->AddLimb(poleNode, m_wheel, m_wheelRoller);

		// fix to the word with a plane joint
		ndWorld* const world = scene->GetWorld();
		const ndMatrix planeMatrix(m_topBox->GetMatrix());
		m_plane = ndSharedPtr<ndJointBilateralConstraint>(new ndJointPlane(planeMatrix.m_posit, planeMatrix.m_right, m_topBox->GetAsBodyKinematic(), world->GetSentinelBody()));
		model->AddCloseLoop(m_plane);
	}

	ndModelArticulation* ndController::CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndRenderMeshLoader& loader, const char* const name)
	{
		ndMatrix matrix(location);
		matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);
		matrix.m_posit.m_y += ndFloat32(0.1f);
		loader.m_mesh->m_matrix = loader.m_mesh->m_matrix * matrix;
		
		ndSharedPtr<ndRenderSceneNode> visualMesh(loader.m_renderMesh->Clone());
		visualMesh->SetTransform(loader.m_mesh->m_matrix);
		visualMesh->SetTransform(loader.m_mesh->m_matrix);
		
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
		//model->AddBodiesAndJointsToWorld();

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

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("unicycle.nd"));
	ndController::CreateModel(scene, matrix, loader, CONTROLLER_NAME_SAC);

	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}

void ndUnicyclePlayer_PPO(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend_Ppo());
	scene->SetDemoHelp(demoHelper);

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
