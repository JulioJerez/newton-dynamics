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
#include "ndBipedPlayer.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"

namespace ndBipedPlayer
{
	class ndHelpLegend_Sac : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "Pre-trained biped.");
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
		,m_randomImpulseCounter(1)
		,m_isTrainning(false)
	{
	}

	void ndController::Update(ndFloat32 timestep)
	{
		m_timestep = timestep;
		//m_agent->Step();
	}

	void ndController::PostUpdate(ndFloat32)
	{
		//ndMatrix matrix (m_topBox->GetMatrix());
		//if (ndAbs(matrix.m_posit.m_x) > 399.0f)
		//{
		//	matrix.m_posit.m_x = ndFloat32(0.0f);
		//	GetModel()->GetAsModelArticulation()->SetTransform(matrix);
		//}
		//m_randomImpulseCounter = (m_randomImpulseCounter + 1) % ND_RANDOM_IMPULSE_MOD;
	}

	void ndController::ResetModel()
	{
		ndAssert(0);
		//ndMatrix boxMatrix(ndGetIdentityMatrix());
		//boxMatrix.m_posit = m_topBox->GetMatrix().m_posit;
		//boxMatrix.m_posit.m_x = ndFloat32(0.0f);
		//boxMatrix.m_posit.m_y = ndFloat32(2.5f);
		//m_topBox->SetMatrix(boxMatrix);
		//
		//const ndMatrix poleMatrix(m_poleHinge->GetLocalMatrix0().OrthoInverse() * m_poleHinge->CalculateGlobalMatrix1());
		//m_pole->SetMatrix(poleMatrix);
		//
		//const ndMatrix ballMatrix(m_wheelRoller->GetLocalMatrix0().OrthoInverse() * m_wheelRoller->CalculateGlobalMatrix1());
		//m_wheel->SetMatrix(ballMatrix);
		//
		//m_pole->SetOmega(ndVector::m_zero);
		//m_pole->SetVelocity(ndVector::m_zero);
		//
		//m_topBox->SetOmega(ndVector::m_zero);
		//m_topBox->SetVelocity(ndVector::m_zero);
		//
		//m_wheel->SetOmega(ndVector::m_zero);
		//m_wheel->SetVelocity(ndVector::m_zero);
		//
		//GetModel()->GetAsModelArticulation()->ClearMemory();
	}
		
	//// calculate pole angle relative to the world.
	//ndFloat32 ndController::GetPoleAngle() const
	//{
	//	const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
	//	const ndMatrix matrix(hinge->CalculateGlobalMatrix0());
	//	ndFloat32 angle = ndAcos(ndClamp(matrix.m_up.m_y, ndFloat32(-1.0f), ndFloat32(1.0f)));
	//	return angle;
	//}
	//
	//ndFloat32 ndController::GetBoxAngle() const
	//{
	//	const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
	//	const ndMatrix matrix(hinge->CalculateGlobalMatrix1());
	//	ndFloat32 angle = ndAcos(ndClamp(matrix.m_up.m_y, ndFloat32(-1.0f), ndFloat32(1.0f)));
	//	return angle;
	//}
	//
	//ndFloat32 ndController::GetBoxOmega() const
	//{
	//	const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
	//	const ndMatrix matrix(hinge->CalculateGlobalMatrix1());
	//	const ndVector omega(m_topBox->GetOmega());
	//	return omega.DotProduct(matrix.m_front).GetScalar();
	//}

	#pragma optimize( "", off )
	bool ndController::IsTerminal() const
	{
		//bool fail = ndAbs(GetPoleAngle()) > ND_TERMINATION_ANGLE;
		//fail = fail || ndAbs(GetBoxAngle()) > ndFloat32 (90.0f) * ndDegreeToRad;
		//return fail;
		return true;
	}

	#pragma optimize( "", off )
	ndBrainFloat ndController::CalculateReward() const
	{
		//if (IsTerminal())
		//{
		//	return ndBrainFloat(-1.0f);
		//}
		//
		//// trying with center of mass dynammics
		//// b*ut the result so far the results are very dissapointing
		//// this however word much better is an order version 
		//// maybe I have bugs that I have to track
		//ndMatrix comFrame(m_wheelRoller->CalculateGlobalMatrix1());
		//comFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
		//comFrame.m_right = comFrame.m_front.CrossProduct(comFrame.m_up).Normalize();
		//comFrame.m_up = comFrame.m_right.CrossProduct(comFrame.m_front).Normalize();
		//
		//ndAssert(*m_solver);
		//// exclude the wheel angular momentum from the com kinematics
		//const ndVector savedWheelOmega(m_wheel->GetOmega());
		//m_wheel->SetOmegaNoSleep(ndVector::m_zero);
		//ndFixSizeArray<ndJointBilateralConstraint*, D_INV_IK_MAX_LINKS> extraJoints;
		//ndModelArticulation::ndCenterOfMassDynamics comDynamics(GetModel()->GetAsModelArticulation()->CalculateCentreOfMassDynamics(*((ndIkSolver*)*m_solver), comFrame, extraJoints, m_timestep));
		//m_wheel->SetOmegaNoSleep(savedWheelOmega);
		//
		//const ndFloat32 poleAngle = ndFloat32(8.0f) * GetPoleAngle() / ND_TERMINATION_ANGLE;
		//const ndFloat32 comOmega = ndFloat32(2.0f) * comDynamics.m_omega.m_x;
		//const ndFloat32 comAlpha = ndFloat32(0.5f) * comDynamics.m_alpha.m_x;
		//const ndFloat32 comSpeed = ndMax(ndAbs(comDynamics.m_veloc.m_z) - ndFloat32(8.0f), ndFloat32(0.0f));
		//const ndFloat32 boxAngle = ndMax(ndAbs(GetBoxAngle()) - ndFloat32(45.f) * ndDegreeToRad, ndFloat32(0.0f));
		//
		//const ndFloat32 invSigma2 = ndFloat32(4.0f);
		//const ndFloat32 poleAngleReward = ndExp(-invSigma2 * poleAngle * poleAngle);
		//const ndFloat32 comOmegaReward = ndExp(-invSigma2 * comOmega * comOmega);
		//const ndFloat32 comAlphaReward = ndExp(-invSigma2 * comAlpha * comAlpha);
		//const ndFloat32 comSpeedPenalty = ndExp(-invSigma2 * comSpeed * comSpeed) - ndFloat32(1.0f);
		//const ndFloat32 boxAnglePenalty = ndExp(-invSigma2 * boxAngle * boxAngle) - ndFloat32(1.0f);
		//
		//ndFloat32 reward = ndFloat32(0.0f);
		//reward += poleAngleReward * ndFloat32(0.6f);
		//reward += comOmegaReward * ndFloat32(0.2f);
		//reward += comAlphaReward * ndFloat32(0.2f);
		//reward += comSpeedPenalty * ndFloat32(0.5f);
		//reward += boxAnglePenalty * ndFloat32(0.5f);
		//
		//return ndBrainFloat(reward);
		return 0;
	}

	void ndController::ApplyActions(ndBrainFloat* const actions)
	{
		//const ndVector wheelMass(m_wheel->GetAsBodyDynamic()->GetMassMatrix());
		//const ndMatrix wheelMatrix(m_wheelRoller->CalculateGlobalMatrix0());
		//
		//ndFloat32 wheelTorque = wheelMass.m_z * actions[m_wheelTorque] * ND_MAX_WHEEL_ALPHA;
		//if (IsOnAir())
		//{
		//	ndFloat32 omega = m_wheel->GetOmega().m_z;
		//	ndFloat32 drag = ndFloat32(0.1f) * omega * omega * ndSign(omega);
		//	wheelTorque = -drag;
		//}
		//
		////ndExpandTraceMessage("%g %g %g\n", speed, drag, wheelTorque);
		//ndVector torque(wheelMatrix.m_front.Scale(wheelTorque));
		//m_wheel->GetAsBodyDynamic()->SetTorque(torque);
		//
		//if (m_isTrainning && (m_randomImpulseCounter == 0))
		//{
		//	// when in tranning mode,
		//	// apply a random impulse to the top box every m_randomImpulseCounter steps
		//	ndFloat32 randOmega = ND_RANDOM_IMPULSE_MAGNITUD * (ndFloat32(0.5f) - ndRand());
		//	const ndVector mass(m_topBox->GetAsBodyDynamic()->GetMassMatrix());
		//	const ndVector pin(m_poleHinge->CalculateGlobalMatrix1().m_front.Scale (randOmega));
		//	const ndVector randomImpulseTorque(pin * mass);
		//	m_topBox->GetAsBodyDynamic()->ApplyImpulsePair(ndVector::m_zero, randomImpulseTorque, m_timestep);
		//}
	}

	ndBrainFloat ndController::IsOnAir() const
	{
		//ndBodyKinematic::ndContactMap& contacts = m_wheel->GetAsBodyKinematic()->GetContactMap();
		//ndBodyKinematic::ndContactMap::Iterator it(contacts);
		//for (it.Begin(); it; it++)
		//{
		//	ndContact* const contact = *it;
		//	if (contact->IsActive())
		//	{
		//		const ndContactPointList& contactPoints = contact->GetContactPoints();
		//		return contactPoints.GetCount() ? ndBrainFloat(0.0f) : ndBrainFloat(1.0f);
		//	}
		//}
		return ndBrainFloat(1.0f);
	};

	#pragma optimize( "", off)
	void ndController::GetObservation(ndBrainFloat* const observation)
	{
		//ndMatrix comFrame(m_wheelRoller->CalculateGlobalMatrix1());
		//comFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
		//comFrame.m_right = comFrame.m_front.CrossProduct(comFrame.m_up).Normalize();
		//comFrame.m_up = comFrame.m_right.CrossProduct(comFrame.m_front).Normalize();
		//const ndVector savedWheelOmega(m_wheel->GetOmega());
		//m_wheel->SetOmegaNoSleep(ndVector::m_zero);
		//ndModelArticulation::ndCenterOfMassDynamics comKinematics(GetModel()->GetAsModelArticulation()->CalculateCentreOfMassKinematics(comFrame));
		//m_wheel->SetOmegaNoSleep(savedWheelOmega);
		//
		//ndFloat32 boxAngle = GetBoxAngle();
		//ndFloat32 boxOmega = GetBoxOmega();
		//ndFloat32 comSpeed = comKinematics.m_veloc.m_z;
		//ndFloat32 hingeAngle = ((ndJointHinge*)*m_poleHinge)->GetAngle();
		//ndFloat32 hingeOmega = ((ndJointHinge*)*m_poleHinge)->GetOmega();
		//
		//observation[m_hasContactSupport] = IsOnAir();
		//observation[m_comSpeed] = ndBrainFloat(comSpeed);
		//observation[m_boxAngle] = ndBrainFloat(boxAngle);
		//observation[m_boxOmega] = ndBrainFloat(boxOmega);
		//observation[m_hingeAngle] = ndBrainFloat(hingeAngle);
		//observation[m_hingeOmega] = ndBrainFloat(hingeOmega);
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

			//char name[256];
			//static int xxxx = 0;
			//sprintf_s(name, 255, "xxx%d.nd", xxxx);
			//xxxx++;
			//ndMesh::SaveRigidBody(body, ndGetWorkingFileName(name).GetStr());
			return body;
		};

		// add the root body
		ndSharedPtr<ndBody> rootBody(CreateRigidBody(mesh, visualMesh, 1.0f, nullptr));
		ndModelArticulation::ndNode* const modelRootNode = model->AddRootBody(rootBody);

		// add right leg
		{
			// hip
			ndSharedPtr<ndMesh> hipMesh(mesh->FindByClosestMatch("rightHip")->GetSharedPtr());
			ndSharedPtr<ndRenderSceneNode> hipEntity(visualMesh->FindByClosestMatch("rightHip")->GetSharedPtr());
			ndSharedPtr<ndBody> hipBody(CreateRigidBody(hipMesh, hipEntity, 1.0f, rootBody->GetAsBodyDynamic()));
			const ndMatrix hipMatrix(hipMesh->CalculateGlobalMatrix());
			ndSharedPtr<ndJointBilateralConstraint> hipJoint(new ndJointHinge(hipMatrix, hipBody->GetAsBodyKinematic(), rootBody->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const hipLink = model->AddLimb(modelRootNode, hipBody, hipJoint);

			// upper thigh
			ndSharedPtr<ndMesh> upperThighMesh(hipMesh->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndRenderSceneNode> upperThighEntity(hipEntity->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndBody> upperThighBody(CreateRigidBody(upperThighMesh, upperThighEntity, 1.0f, hipBody->GetAsBodyDynamic()));
			const ndMatrix upperThighMatrix(upperThighMesh->CalculateGlobalMatrix());
			ndSharedPtr<ndJointBilateralConstraint> upperThighJoint(new ndJointHinge(upperThighMatrix, upperThighBody->GetAsBodyKinematic(), hipBody->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const upperThighBodyLink = model->AddLimb(hipLink, upperThighBody, upperThighJoint);

			// lower thigh
			ndSharedPtr<ndMesh> lowerThighMesh(upperThighMesh->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndRenderSceneNode> lowerThighEntity(upperThighEntity->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndBody> lowerThighBody(CreateRigidBody(lowerThighMesh, lowerThighEntity, 1.0f, upperThighBody->GetAsBodyDynamic()));
			const ndMatrix lowerThighMatrix(lowerThighMesh->CalculateGlobalMatrix());
			ndSharedPtr<ndJointBilateralConstraint> lowerThighJoint(new ndJointHinge(lowerThighMatrix, lowerThighBody->GetAsBodyKinematic(), upperThighBody->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const lowerThighBodyLink = model->AddLimb(upperThighBodyLink, lowerThighBody, lowerThighJoint);

			// calf
			ndSharedPtr<ndMesh> calfMesh(lowerThighMesh->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndRenderSceneNode> calfEntity(lowerThighEntity->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndBody> calfBody(CreateRigidBody(calfMesh, calfEntity, 1.0f, lowerThighBody->GetAsBodyDynamic()));
			const ndMatrix calfMatrix(calfMesh->CalculateGlobalMatrix());
			ndSharedPtr<ndJointBilateralConstraint> calfJoint(new ndJointHinge(calfMatrix, calfBody->GetAsBodyKinematic(), lowerThighBody->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const calfBodyLink = model->AddLimb(lowerThighBodyLink, calfBody, calfJoint);

			// soft Contact
			ndSharedPtr<ndMesh> softMesh(calfMesh->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndRenderSceneNode> softEntity(calfEntity->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndBody> softBody(CreateRigidBody(softMesh, softEntity, 1.0f, calfBody->GetAsBodyDynamic()));
			const ndMatrix softMatrix(softMesh->CalculateGlobalMatrix());
			ndSharedPtr<ndJointBilateralConstraint> softJoint(new ndJointSlider(softMatrix, softBody->GetAsBodyKinematic(), calfBody->GetAsBodyKinematic()));
			((ndJointSlider*)*softJoint)->SetAsSpringDamper(0.001f, 1000.0f, 20.0f);
			ndModelArticulation::ndNode* const softBodyLink = model->AddLimb(calfBodyLink, softBody, softJoint);
		}

		// add left leg
		{
			// hip
			ndSharedPtr<ndMesh> hipMesh(mesh->FindByClosestMatch("leftHip")->GetSharedPtr());
			ndSharedPtr<ndRenderSceneNode> hipEntity(visualMesh->FindByClosestMatch("leftHip")->GetSharedPtr());
			ndSharedPtr<ndBody> hipBody(CreateRigidBody(hipMesh, hipEntity, 1.0f, rootBody->GetAsBodyDynamic()));
			const ndMatrix hipMatrix(hipMesh->CalculateGlobalMatrix());
			ndSharedPtr<ndJointBilateralConstraint> hipJoint(new ndJointHinge(hipMatrix, hipBody->GetAsBodyKinematic(), rootBody->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const hipLink = model->AddLimb(modelRootNode, hipBody, hipJoint);

			// thigh
			ndSharedPtr<ndMesh> thighMesh(hipMesh->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndRenderSceneNode> thighEntity(hipEntity->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndBody> thighBody(CreateRigidBody(thighMesh, thighEntity, 1.0f, hipBody->GetAsBodyDynamic()));
			const ndMatrix thighMatrix(thighMesh->CalculateGlobalMatrix());
			ndSharedPtr<ndJointBilateralConstraint> thighJoint(new ndJointHinge(thighMatrix, thighBody->GetAsBodyKinematic(), hipBody->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const thighBodyLink = model->AddLimb(hipLink, thighBody, thighJoint);

			//ndMesh::SaveRigidBody(body, ndGetWorkingFileName(name).GetStr());
		}
		
		// fix to the world with a fix 6 dof joint
		ndWorld* const world = scene->GetWorld();
		const ndMatrix fixMatrix(rootBody->GetMatrix());
		ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(fixMatrix, rootBody->GetAsBodyKinematic(), world->GetSentinelBody()));
		model->AddCloseLoop(fixJoint);
	}

	ndModelArticulation* ndController::CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndRenderMeshLoader& loader, const char* const name)
	{
		ndMatrix matrix(location);
		matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);
		matrix.m_posit.m_y += ndFloat32(2.5f);
		loader.m_mesh->SetMatrix(loader.m_mesh->GetMatrix() * matrix);
		
		ndSharedPtr<ndRenderSceneNode> visualMesh(loader.m_renderMesh->Clone());
		visualMesh->SetTransform(loader.m_mesh->GetMatrix());
		visualMesh->SetTransform(loader.m_mesh->GetMatrix());
		
		ndModelArticulation* const model = new ndModelArticulation();
		ndSharedPtr<ndModelNotify> controller(new ndPlaybackController());
		model->SetNotifyCallback(controller);
		ndPlaybackController* const playerController = (ndPlaybackController*)(*controller);
		playerController->CreateArticulatedModel(scene, model, loader.m_mesh, visualMesh);

		//char nameExt[256];
		//snprintf(nameExt, sizeof(nameExt) - 1, "%s.dnn", name);
		//ndString fileName(ndGetWorkingFileName(nameExt));
		//ndSharedPtr<ndBrain> policy(ndBrainLoad::Load(fileName.GetStr()));
		//playerController->m_agent = ndSharedPtr<ndBrainAgent>(new ndController::ndAgent(policy, playerController));

		// add model a visual mesh to the scene and world
		ndWorld* const world = scene->GetWorld();
		world->AddModel(model);
		scene->AddEntity(visualMesh);
		return model;
	}

	class DaveGravelModel: public ndRenderMeshLoader
	{
		public:
		DaveGravelModel(ndDemoEntityManager* const scene)
			:ndRenderMeshLoader(*scene->GetRenderer())
		{
			auto CreateBox = [scene](ndMesh* const parent, const ndMatrix& matrix, ndFloat32 x, ndFloat32 y, ndFloat32 z, const char* const name, const char* const texture = "smilli.png")
			{
				ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeBox(x, y, z)));
				ndSharedPtr<ndMesh> mesh(new ndMesh(**shape));
				mesh->SetMatrix(matrix);
				//naming convention for ndMesh rigid body funtionality
				ndString meshName(name);
				meshName += "-box";
				mesh->SetName(meshName);

				if (parent)
				{
					parent->AddChild(mesh);
				}
				return mesh;
			};

			auto CreateSphere = [scene](ndMesh* const parent, const ndMatrix& matrix, ndFloat32 radio, const char* const name, const char* const texture = "smilli.png")
			{
				ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeSphere(radio * 0.5f)));
				ndSharedPtr<ndMesh> mesh(new ndMesh(**shape));
				mesh->SetMatrix(matrix);
				//naming convention for ndMesh rigid body funtionality
				ndString meshName(name);
				meshName += "-sphere";
				mesh->SetName(meshName);

				if (parent)
				{
					parent->AddChild(mesh);
				}
				return mesh;
			};

			auto CreateCapsule = [scene](ndMesh* const parent, const ndMatrix& matrix, ndFloat32 radio0, ndFloat32 radio1, ndFloat32 height, const char* const name, const char* texture = "smilli.png")
			{
				radio0 *= 0.5f;
				radio1 *= 0.5f;
				ndSharedPtr<ndShapeInstance> capsule(new ndShapeInstance(new ndShapeCapsule(radio0, radio1, height)));
				ndSharedPtr<ndMesh> mesh(new ndMesh(**capsule));
				mesh->SetMatrix(matrix);
				//naming convention for ndMesh rigid body funtionality
				ndString meshName(name);
				meshName += "-capsule";
				mesh->SetName(meshName);

				if (parent)
				{
					parent->AddChild(mesh);
				}
				return mesh;
			};

			// torso
			ndSharedPtr<ndMesh> root(CreateCapsule(nullptr, ndGetIdentityMatrix(), 0.325f, 0.325f, 0.2f, "torso"));
			root->SetGeometryMatrix(ndRollMatrix(90.0f * ndDegreeToRad));

			// spine1
			ndMatrix spine1Matrix(ndRollMatrix(90.0f * ndDegreeToRad));
			spine1Matrix.m_posit.m_y = 0.525f;
			ndSharedPtr<ndMesh> spine1(CreateCapsule(*root, spine1Matrix, 0.3f, 0.3f, 0.4f, "spine1"));
			
			// spine2
			ndMatrix spine2Matrix(ndGetIdentityMatrix());
			spine2Matrix.m_posit.m_x = 0.625f;
			ndSharedPtr<ndMesh> spine2(CreateCapsule(*spine1, spine2Matrix, 0.35f, 0.35f, 0.4f, "spine2"));
			
			// neck
			ndMatrix neckMatrix(ndGetIdentityMatrix());
			neckMatrix.m_posit.m_x = 0.35f;
			ndSharedPtr<ndMesh> neck(CreateCapsule(*spine2, neckMatrix, 0.2f, 0.2f, 0.125f, "neck"));
			
			// head
			ndMatrix headMatrix(ndGetIdentityMatrix());
			headMatrix.m_posit.m_x = 0.35f;
			CreateSphere(*neck, headMatrix, 0.5f, "head");

			// right arm
			{
				ndMatrix shouldMatrix(ndYawMatrix(-100.0f * ndDegreeToRad));
				shouldMatrix.m_posit.m_x = 0.2f;
				shouldMatrix.m_posit.m_z = 0.25f;
				ndSharedPtr<ndMesh> shoulder(CreateCapsule(*spine2, shouldMatrix, 0.25f, 0.25f, 0.1f, "rightShoulder"));
			
				ndMatrix armMatrix (ndYawMatrix(-65.0f * ndDegreeToRad));
				armMatrix.m_posit.m_x = 0.2f;
				ndSharedPtr<ndMesh> arm(CreateCapsule(*shoulder, armMatrix, 0.25f, 0.2f, 0.4f, "rightArm"));
				ndMatrix offset(ndGetIdentityMatrix());
				offset.m_posit.m_x = 0.2f;
				arm->SetGeometryMatrix(offset);
			
				ndMatrix forwardArmMatrix (ndRollMatrix(-10.0f * ndDegreeToRad));
				forwardArmMatrix.m_posit.m_x = 0.4f;
				ndSharedPtr<ndMesh> forwardArm(CreateCapsule(*arm, forwardArmMatrix, 0.2f, 0.15f, 0.4f, "rightForwardArm"));
				offset.m_posit.m_x = 0.3f;
				forwardArm->SetGeometryMatrix(offset);
			
				// hand
				ndMatrix handMatrix = ndGetIdentityMatrix();
				handMatrix.m_posit.m_x = 0.55f;
				ndSharedPtr<ndMesh> foot (CreateBox(*forwardArm, handMatrix, 0.2f, 0.2f, 0.1f, "rightHand"));
				offset.m_posit.m_x = 0.1f;
				foot->SetGeometryMatrix(offset);
			}

			// left arm
			{
				ndMatrix shouldMatrix(ndYawMatrix(100.0f * ndDegreeToRad));
				shouldMatrix.m_posit.m_x = 0.2f;
				shouldMatrix.m_posit.m_z -= 0.25f;
				ndSharedPtr<ndMesh> shoulder(CreateCapsule(*spine2, shouldMatrix, 0.25f, 0.25f, 0.1f, "leftShoulder"));
			
				ndMatrix armMatrix(ndYawMatrix(65.0f * ndDegreeToRad));
				armMatrix.m_posit.m_x = 0.2f;
				ndSharedPtr<ndMesh> arm(CreateCapsule(*shoulder, armMatrix, 0.25f, 0.2f, 0.4f, "leftArm"));
				ndMatrix offset(ndGetIdentityMatrix());
				offset.m_posit.m_x = 0.2f;
				arm->SetGeometryMatrix(offset);

				ndMatrix forwardArmMatrix(ndRollMatrix(-10.0f * ndDegreeToRad));
				forwardArmMatrix.m_posit.m_x = 0.4f;
				ndSharedPtr<ndMesh> forwardArm(CreateCapsule(*arm, forwardArmMatrix, 0.2f, 0.15f, 0.4f, "leftForwardArm"));
				offset.m_posit.m_x = 0.3f;
				forwardArm->SetGeometryMatrix(offset);

				// hand
				ndMatrix handMatrix = ndGetIdentityMatrix();
				handMatrix.m_posit.m_x = 0.55f;
				ndSharedPtr<ndMesh> foot(CreateBox(*forwardArm, handMatrix, 0.2f, 0.2f, 0.1f, "leftHand"));
				offset.m_posit.m_x = 0.1f;
				foot->SetGeometryMatrix(offset);
			}

			// right leg
			{
				// hip
				ndMatrix hipMatrix(ndYawMatrix(90.0f * ndDegreeToRad));
				hipMatrix.m_posit.m_y = -0.25f;
				hipMatrix.m_posit.m_z = 0.22f;
				ndSharedPtr<ndMesh> hip(CreateCapsule(*root, hipMatrix, 0.25f, 0.25f, 0.1f, "rightHip"));

				// upper thigh
				//ndMatrix upperThighMatrix = ndRollMatrix(90.0f * ndDegreeToRad);
				ndMatrix upperThighMatrix (ndYawMatrix(90.0f * ndDegreeToRad));
				ndSharedPtr<ndMesh> upperThigh(CreateCapsule(*hip, upperThighMatrix, 0.3f, 0.3f, 0.5f, "upperRightThigh"));
				ndMatrix offset(ndRollMatrix(85.0f * ndDegreeToRad));
				offset.m_posit.m_y = -0.385f;
				upperThigh->SetGeometryMatrix(offset);

				// lower thigh
				ndMatrix lowerThighMatrix = ndRollMatrix(85.0f * ndDegreeToRad);
				ndSharedPtr<ndMesh> lowerThigh(CreateCapsule(*upperThigh, lowerThighMatrix, 0.3f, 0.3f, 0.5f, "lowerRightThigh"));
				offset = ndGetIdentityMatrix();
				offset.m_posit.m_x = -0.385f;
				lowerThigh->SetGeometryMatrix(offset);
				
				// calf
				//ndMatrix calfMatrix = ndGetIdentityMatrix();
				ndMatrix calfMatrix (ndYawMatrix(90.0f * ndDegreeToRad));
				calfMatrix.m_posit.m_x = -0.7f;
				ndSharedPtr<ndMesh> calf(CreateCapsule(*lowerThigh, calfMatrix, 0.25f, 0.25f, 0.65f, "rightCaft"));
				offset = ndYawMatrix(-90.0f * ndDegreeToRad);
				offset.m_posit.m_z = -0.385f;
				calf->SetGeometryMatrix(offset);
				
				// soft contact
				ndMatrix softMatrix (ndYawMatrix(90.0f * ndDegreeToRad));
				softMatrix.m_posit.m_z = -0.8f;
				ndSharedPtr<ndMesh> softContact(CreateCapsule(*calf, softMatrix, 0.185f, 0.185f, 0.15f, "rightContact"));
				
				// foot
				ndMatrix footMatrix(ndGetIdentityMatrix());
				footMatrix.m_posit.m_x = 0.2f;
				footMatrix.m_posit.m_y = 0.085f;
				CreateBox(*softContact, footMatrix, 0.175f, 0.525f, 0.3f, "rightFoot");
			}

			// left leg
			{
				// hip
				ndMatrix hipMatrix(ndYawMatrix(-90.0f * ndDegreeToRad));
				hipMatrix.m_posit.m_y = -0.25f;
				hipMatrix.m_posit.m_z = -0.22f;
				ndSharedPtr<ndMesh> hip(CreateCapsule(*root, hipMatrix, 0.25f, 0.25f, 0.1f, "leftHip"));

				// upper thigh
				ndMatrix thighMatrix = ndRollMatrix(85.0f * ndDegreeToRad);
				ndSharedPtr<ndMesh> thigh(CreateCapsule(*hip, thighMatrix, 0.3f, 0.3f, 0.5f, "leftThigh"));
				ndMatrix offset(ndGetIdentityMatrix());
				offset.m_posit.m_x = -0.385f;
				thigh->SetGeometryMatrix(offset);

				// calf
				ndMatrix calfMatrix = ndGetIdentityMatrix();
				calfMatrix.m_posit.m_x = -1.0f;
				ndSharedPtr<ndMesh> calf(CreateCapsule(*thigh, calfMatrix, 0.25f, 0.25f, 0.65f, "leftCaft"));

				// soft contact
				ndMatrix softMatrix = ndGetIdentityMatrix();
				softMatrix.m_posit.m_x = -0.425f;
				ndSharedPtr<ndMesh> softContact(CreateCapsule(*calf, softMatrix, 0.185f, 0.185f, 0.15f, "leftContact"));

				// foot
				ndMatrix footMatrix = ndGetIdentityMatrix();
				footMatrix.m_posit.m_x = -0.25f;
				footMatrix.m_posit.m_z = -0.085f;
				CreateBox(*softContact, footMatrix, 0.175f, 0.3f, 0.525f, "leftFoot");
			}

			m_mesh = root;
			MeshToRenderSceneNode(ndGetWorkingFileName(""));
		}
	};
}
using namespace ndBipedPlayer;

void ndBipedPlayer_SAC(ndDemoEntityManager* const scene)
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

void ndBipedPlayer_PPO(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend_Ppo());
	scene->SetDemoHelp(demoHelper);
	
	//// oveload the ground friction
	//// make sure the ground has enough friction
	//ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	//ndMaterial* const defaultMaterial = callback->GetMaterial(ndDemoContactCallback::m_default, ndDemoContactCallback::m_default);
	//ndAssert(defaultMaterial);
	//defaultMaterial->m_dynamicFriction0 = defaultMaterial->m_staticFriction0;
	//defaultMaterial->m_dynamicFriction1 = defaultMaterial->m_staticFriction1;
	
	ndMatrix matrix(ndGetIdentityMatrix());
	//ndRenderMeshLoader loader(*scene->GetRenderer());
	//loader.LoadMesh(ndGetWorkingFileName("ragdoll.nd"));
	DaveGravelModel loader(scene);
	ndController::CreateModel(scene, matrix, loader, CONTROLLER_NAME_PPO);

	matrix.m_posit.m_x -= 10.0f;
	matrix.m_posit.m_y += 2.0f;
	matrix.m_posit.m_z += 0.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}