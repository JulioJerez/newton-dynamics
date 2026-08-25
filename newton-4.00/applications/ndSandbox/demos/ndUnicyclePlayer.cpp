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

	//#pragma optimize( "", off )
	void ndController::PostUpdate(ndFloat32, ndInt32)
	{
		ndMatrix matrix (m_topBox->GetMatrix());
		if (ndAbs(matrix.m_posit.m_x) > 300.0f)
		{
			matrix.m_posit.m_x = ndFloat32(0.0f);
			GetModel()->GetAsModelArticulation()->SetTransform(matrix);
		}
		m_randomImpulseCounter = (m_randomImpulseCounter + 1) % ND_RANDOM_IMPULSE_MOD;
	}

	void ndController::ResetModel()
	{
		ndMatrix boxMatrix(ndGetIdentityMatrix());
		boxMatrix.m_posit = m_topBox->GetMatrix().m_posit;

		ndFloat32 randX = ndFloat32(20.0f) * (ndRand() - ndFloat32(0.5f));
		boxMatrix.m_posit.m_x = randX;
		boxMatrix.m_posit.m_y = ndFloat32(3.5f);
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
	}
		
	// calculate pole angle relative to the world.
	ndFloat32 ndController::GetPoleAngle() const
	{
		const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		const ndMatrix matrix(hinge->CalculateGlobalMatrix0());
		ndFloat32 angle = ndAcos(ndClamp(matrix.m_up.m_y, ndFloat32(-1.0f), ndFloat32(1.0f)));
		return angle;
	}

	ndFloat32 ndController::GetPoleOmega() const
	{
		const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		const ndMatrix matrix(hinge->CalculateGlobalMatrix0());
		ndFloat32 omega = matrix.m_front.DotProduct(m_poleHinge->GetBody0()->GetOmega()).GetScalar();
		return omega;
	}

	ndFloat32 ndController::GetWheelOmega() const
	{
		const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		const ndMatrix matrix(hinge->CalculateGlobalMatrix1());
		ndFloat32 omega = matrix.m_front.DotProduct(m_poleHinge->GetBody1()->GetOmega()).GetScalar();
		return omega;
	}

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
		ndFloat32 angle = ndAcos(ndClamp(matrix.m_up.m_y, ndFloat32(-1.0f), ndFloat32(1.0f)));
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
		fail = fail || ndAbs(GetBoxAngle()) > ndFloat32 (90.0f) * ndDegreeToRad;
		return fail;
	}

	#pragma optimize( "", off )
	ndBrainFloat ndController::CalculateReward() const
	{
		if (IsTerminal())
		{
			return ndBrainFloat(-1.0f);
		}

		const ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
		ndModelArticulation::ndCenterOfMassDynamics kinematic(model->CalculateCentreOfMassKinematics(ndGetIdentityMatrix()));

		ndFloat32 comSpeed = kinematic.m_veloc.m_x;
		ndFloat32 comSpeedReward = ndExp(-ndFloat32(0.1f) * comSpeed * comSpeed);

		const ndFloat32 invSigma2 = ndFloat32(900.0f);
		const ndFloat32 poleAngle = ndMax((ndAbs(GetPoleAngle()) - ndFloat32(10.0f * ndDegreeToRad)), ndFloat32(0.0f));
		const ndFloat32 poleAngleReward = ndExp(-invSigma2 * poleAngle * poleAngle);

		ndFloat32 reward = ndFloat32(0.0f);
		reward += comSpeedReward * ndFloat32(0.5f);
		reward += poleAngleReward * ndFloat32(0.5f);
		
		return ndBrainFloat(reward);
	}

	#pragma optimize( "", off )
	void ndController::ApplyActions(ndBrainFloat* const actions)
	{
		const ndVector wheelMass(m_wheel->GetAsBodyDynamic()->GetMassMatrix());
		const ndMatrix wheelMatrix(m_wheelRoller->CalculateGlobalMatrix0());

		ndFloat32 wheelTorque = ND_MAX_WHEEL_ALPHA * wheelMass.m_z * actions[m_wheelTorque];
		if (IsOnAir())
		{
			ndFloat32 omega = m_wheel->GetOmega().m_z;
			ndFloat32 drag = ndFloat32(0.1f) * omega * omega * ndSign(omega);
			wheelTorque = -drag;
		}

		//ndExpandTraceMessage("%g %g %g\n", speed, drag, wheelTorque);
		ndVector torque(wheelMatrix.m_front.Scale(wheelTorque));
		m_wheel->GetAsBodyDynamic()->SetTorque(torque);

		if (m_isTrainning && (m_randomImpulseCounter == 0))
		{
			// when in training mode,
			// apply a random impulse to the top box every m_randomImpulseCounter steps
			ndFloat32 randOmega = ND_RANDOM_IMPULSE_MAGNITUD * (ndFloat32(0.5f) - ndRand());
			const ndVector mass(m_topBox->GetAsBodyDynamic()->GetMassMatrix());
			const ndVector pin(m_poleHinge->CalculateGlobalMatrix1().m_front.Scale (randOmega));
			const ndVector randomImpulseTorque(pin * mass);
			m_topBox->GetAsBodyDynamic()->ApplyImpulsePair(ndVector::m_zero, randomImpulseTorque, m_timestep);
		}
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
		ndFloat32 boxAngle = GetBoxAngle();
		ndFloat32 boxOmega = GetBoxOmega();
		ndFloat32 poleAngle = GetPoleAngle();
		ndFloat32 poleOmega = GetPoleOmega();
		ndFloat32 wheelOmega = GetWheelOmega();
		//ndFloat32 wheelAlpha = GetWheelAlpha();

		observation[m_hasContactSupport] = IsOnAir();
		observation[m_boxAngle] = ndBrainFloat(boxAngle);
		observation[m_boxOmega] = ndBrainFloat(boxOmega);
		observation[m_poleAngle] = ndBrainFloat(poleAngle);
		observation[m_poleOmega] = ndBrainFloat(poleOmega);
		observation[m_wheelOmega] = ndBrainFloat(wheelOmega);
		//observation[m_wheelAlpha] = ndBrainFloat(wheelAlpha);
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

			if (node->m_name.Find("body") > -1)
			{
				m_topBox = node->m_body;
			}
			else if (node->m_name.Find("pole") > -1)
			{
				m_pole = node->m_body;
				m_poleHinge = node->m_joint;
				//((ndJointHinge*)*m_poleHinge)->SetAsSpringDamper(0.5f, 0.0f, 1.0f);
			}
			else if (node->m_name.Find("roller") > -1)
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
