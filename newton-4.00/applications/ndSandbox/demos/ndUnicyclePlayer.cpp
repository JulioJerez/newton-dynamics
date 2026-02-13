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
		,m_bestReward(0.0f)
	{
	}

	void ndController::Update(ndFloat32 timestep)
	{
		m_timestep = timestep;
		m_agent->Step();
	}

	#pragma optimize( "", off )
	void ndController::PostUpdate(ndFloat32)
	{
		ndMatrix matrix (m_topBox->GetMatrix());
		if (ndAbs(matrix.m_posit.m_x) > 399.0f)
		{
			matrix.m_posit.m_x = ndFloat32(0.0f);
			GetModel()->GetAsModelArticulation()->SetTransform(matrix);
		}
	}

	void ndController::ResetModel()
	{
#if 1
		ndMatrix boxMatrix(ndGetIdentityMatrix());
		boxMatrix.m_posit = m_topBox->GetMatrix().m_posit;
		boxMatrix.m_posit.m_x = ndFloat32(0.0f);
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
#else	
		for (ndInt32 i = 0; i < m_modelPose.GetCount(); ++i)
		{
			ndBodyKinematic* const body = m_modelPose[i].m_body;
			body->SetMatrix(m_modelPose[i].m_location);
			body->SetOmega(m_modelPose[i].m_omega);
			body->SetVelocity(m_modelPose[i].m_veloc);
		}
#endif
		m_bestReward = ndFloat32(0.0f);
	}
		
	void ndController::SaveInitialPose(ndFloat32 expectedReward)
	{
		if (expectedReward > m_bestReward)
		{
			// save this state as initial pose
			m_modelPose.SetCount(0);
			m_bestReward = expectedReward;
			ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
				ndPose pose;
				pose.m_body = body;
				pose.m_omega = body->GetOmega();
				pose.m_veloc = body->GetVelocity();
				pose.m_location = body->GetMatrix();
				m_modelPose.PushBack(pose);
			}
		}
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

	#pragma optimize( "", off )
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
			return ndBrainFloat(-1.0f);
		}

		// trying with center of mass dynammics
		// b*ut the result so far the results are very dissapointing
		// this however word much better is an order version 
		// maybe I have bugs that I have to track
		ndMatrix comFrame(m_wheelRoller->CalculateGlobalMatrix1());
		comFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
		comFrame.m_right = comFrame.m_front.CrossProduct(comFrame.m_up).Normalize();
		comFrame.m_up = comFrame.m_right.CrossProduct(comFrame.m_front).Normalize();

		// exclude the wheel angular momentum from the com kinematics
		const ndVector savedWheelOmega(m_wheel->GetOmega());
		m_wheel->SetOmegaNoSleep(ndVector::m_zero);

		ndFixSizeArray<ndJointBilateralConstraint*, D_INV_IK_MAX_LINKS> extraJoints;
		ndModelArticulation::ndCenterOfMassDynamics comDynamics(GetModel()->GetAsModelArticulation()->CalculateCentreOfMassKinematics(comFrame));
		//ndModelArticulation::ndCenterOfMassDynamics comDynamics(GetModel()->GetAsModelArticulation()->CalculateCentreOfMassDynamics(*((ndIkSolver*)*m_solver), comFrame, extraJoints, m_timestep));
		m_wheel->SetOmegaNoSleep(savedWheelOmega);

		ndFloat32 angle = GetPoleAngle() / (0.25f * ND_TERMINATION_ANGLE);
		ndFloat32 omega = comDynamics.m_omega.m_x / ndFloat32 (0.5f);
		ndFloat32 speed = ndMax((ndAbs(comDynamics.m_veloc.m_z) - ndFloat32(10.0f)), ndFloat32(0.0f));
		//ndTrace(("a=%f w=%f s=%f\n", angle, omega, speed));

		const ndFloat32 invSigma2 = ndFloat32(4.0f);
		ndFloat32 angleReward = ndExp(-invSigma2 * angle * angle);
		ndFloat32 omegaReward = ndExp(-invSigma2 * omega * omega);
		ndFloat32 speedPenalty = ndExp(-invSigma2 * speed * speed) - ndFloat32(1.0f);
		if (IsOnAir())
		{
			//omegaReward = ndFloat32(0.0f);
			//angleReward = ndFloat32(0.0f);
			//speedPenalty = ndFloat32(0.0f);
		}
		
		ndFloat32 reward = ndFloat32(0.0f);
		reward += angleReward * ndFloat32(0.6f);
		reward += omegaReward * ndFloat32(0.4f);
		reward += speedPenalty * ndFloat32(0.5f);

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

	#pragma optimize( "", off)
	void ndController::GetObservation(ndBrainFloat* const observation)
	{
		ndMatrix comFrame(m_wheelRoller->CalculateGlobalMatrix1());
		comFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
		comFrame.m_right = comFrame.m_front.CrossProduct(comFrame.m_up).Normalize();
		comFrame.m_up = comFrame.m_right.CrossProduct(comFrame.m_front).Normalize();
		const ndVector savedWheelOmega(m_wheel->GetOmega());
		m_wheel->SetOmegaNoSleep(ndVector::m_zero);
		ndModelArticulation::ndCenterOfMassDynamics comKinematics(GetModel()->GetAsModelArticulation()->CalculateCentreOfMassKinematics(comFrame));
		m_wheel->SetOmegaNoSleep(savedWheelOmega);

		ndFloat32 comSpeed = comKinematics.m_veloc.m_z * ndFloat32(0.25f);

		ndFloat32 hingeOmega = ((ndJointHinge*)*m_poleHinge)->GetOmega();
		ndFloat32 hingeAngle = ((ndJointHinge*)*m_poleHinge)->GetAngle() / ND_MAX_LEG_JOINT_ANGLE;

		ndFloat32 poleOmega = GetPoleOmega();
		ndFloat32 poleAngle = GetPoleAngle() / ND_MAX_LEG_JOINT_ANGLE;
		
		observation[m_hasSupportContact] = IsOnAir();
		observation[m_comSpeed] = ndBrainFloat(comSpeed);
		observation[m_poleAngle] = ndBrainFloat(poleAngle);
		observation[m_poleOmega] = ndBrainFloat(poleOmega); 
		observation[m_hingeAngle] = ndBrainFloat(hingeAngle);
		observation[m_hingeOmega] = ndBrainFloat(hingeOmega);
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

		m_bestReward = -1.0f;
		SaveInitialPose(ndFloat32(0.0f));
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
