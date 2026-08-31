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

using namespace ndUnicyclePlayer;

namespace ndUnicycleTrainer_ppo
{
	class ndHelpLegend : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			const ndVector color(ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
			scene->Print(color, "training a double pendulum using Proximal Policy Optimization method");
			scene->Print(color, "training goes for 100 millions steps. Therefore the training");
			scene->Print(color, "section may take several hours with GPU back end");
		}
	};

	class TrainMaterial : public ndApplicationMaterial
	{
		public:
		TrainMaterial()
			:ndApplicationMaterial()
		{
		}

		TrainMaterial(const TrainMaterial& src)
			:ndApplicationMaterial(src)
		{
		}

		ndApplicationMaterial* Clone() const override
		{
			return new TrainMaterial(*this);
		}

		virtual bool OnAabbOverlap(const ndBodyKinematic* const, const ndBodyKinematic* const) const override
		{
			return false;
		}
	};

	class ndAgent : public ndBrainAgentOnPolicyGradient_Agent
	{
		public:
		ndAgent(ndSharedPtr<ndBrainAgentOnPolicyGradient_Trainer>& master, ndController* const owner, ndInt32 maxTrajectories)
			:ndBrainAgentOnPolicyGradient_Agent(*master, maxTrajectories)
			,m_owner(owner)
		{
		}

		ndBrainFloat CalculateReward() override
		{
			return m_owner->CalculateReward();
		}

		bool IsTerminal() const override
		{
			return m_owner->IsTerminal();
		}

		void GetObservation(ndBrainFloat* const observation) override
		{
			m_owner->GetObservation(observation);
		}

		virtual void ApplyActions(ndBrainFloat* const actions) override
		{
			m_owner->ApplyActions(actions);
		}

		void ResetModel() override
		{
			m_owner->ResetModel();
		}

		ndController* m_owner;
	};

	class TrainingUpdata : public ndDemoEntityManager::OnPostUpdate
	{
		public:
		#pragma optimize( "", off)
		TrainingUpdata(ndDemoEntityManager* const scene, const ndMatrix& location, const ndRenderMeshLoader& loader)
			:OnPostUpdate()
			,m_master()
			,m_outFile(nullptr)
			,m_timer(ndGetTimeInMicroseconds())
			,m_savedScore(ndFloat32(-1.0e10f))
			,m_discountRewardFactor(0.99f)
			,m_horizon(ndFloat32(1.0f) / (ndFloat32(1.0f) - m_discountRewardFactor))
			,m_lastEpisode(0xfffffff)
			,m_stopTraining(200 * 1000000)
			,m_modelIsTrained(false)
		{
			char name[256];
			snprintf(name, sizeof(name), "%s.csv", CONTROLLER_NAME_PPO);
			m_outFile = fopen(name, "wb");
			fprintf(m_outFile, "ppo\n");

			// set random see for replication
			ndSetRandSeed(47);

			// create a proximal policy training agent
			ndBrainAgentOnPolicyGradient_Trainer::HyperParameters hyperParameters;
			
			hyperParameters.m_useGpuBackend = false;
			hyperParameters.m_numberOfHiddenLayers = 3;
			hyperParameters.m_maxTrajectorySteps = 2048;
			hyperParameters.m_batchTrajectoryCount = 1000;
			hyperParameters.m_discountRewardFactor = 0.995f;
			hyperParameters.m_entropyTemperature = 0.005f;
			hyperParameters.m_hiddenLayersNumberOfNeurons = 128;
			hyperParameters.m_numberOfActions = m_actionsSize;
			hyperParameters.m_numberOfObservations = m_observationsSize;
			hyperParameters.m_maxNumberOfTrainingSteps = ndInt32(m_stopTraining);
			hyperParameters.m_discountRewardFactor = ndReal(m_discountRewardFactor);
			
			m_master = ndSharedPtr<ndBrainAgentOnPolicyGradient_Trainer>(new ndBrainAgentOnPolicyGradient_Trainer(hyperParameters));
			
			snprintf(name, sizeof(name), "%s.dnn", CONTROLLER_NAME_PPO);
			m_master->SetName(name);
			
			// create a visual mesh and add to the scene.
			ndWorld* const world = scene->GetWorld();
			ndMatrix matrix(location);
			matrix.m_posit.m_y = ndFloat32(0.25f);
			loader.m_mesh->SetMatrix(loader.m_mesh->GetMatrix() * matrix);
			
			// create an articulated model
			const ndInt32 numberOfAgents = 350;
			const ndInt32 maxTrajectories = (hyperParameters.m_batchTrajectoryCount + numberOfAgents - 1) / numberOfAgents;
			for (ndInt32 i = 0; i < numberOfAgents; ++i)
			{
				ndFloat32 x = ndFloat32(10.0f) * (ndRand() - ndFloat32(0.5f));
				//loader.m_mesh->m_matrix.m_posit.m_x = x;
				ndMatrix meshMatrix(loader.m_mesh->GetMatrix());
				meshMatrix.m_posit.m_x = x;
				loader.m_mesh->SetMatrix(meshMatrix);

				ndSharedPtr<ndRenderSceneNode> visualMesh(loader.m_renderMesh->Clone());
				visualMesh->SetTransform(loader.m_mesh->GetMatrix());
				visualMesh->SetTransform(loader.m_mesh->GetMatrix());
			
				ndSharedPtr<ndModel>model(CreateModel(scene, loader.m_mesh, visualMesh, maxTrajectories));

				//add a control for the reward function
				ndController* const controller = (ndController*)(*model->GetNotifyCallback());
				controller->m_isTrainning = true;
			
				// add model a visual mesh to the scene and world
				world->AddModel(model);
				scene->AddEntity(visualMesh);
			}
		}

		~TrainingUpdata()
		{
			if (m_outFile)
			{
				fclose(m_outFile);
			}
		}

		ndModelArticulation* CreateModel(
			ndDemoEntityManager* const scene, 
			ndSharedPtr<ndMesh> mesh,
			ndSharedPtr<ndRenderSceneNode> visualMesh,
			ndInt32 maxTrajectories)
		{
			ndModelArticulation* const model = new ndModelArticulation();
			ndSharedPtr<ndModelNotify> controller(new ndController());
			model->SetNotifyCallback(controller);

			ndController* const playerController = (ndController*)(*controller);
			playerController->CreateArticulatedModel(scene, model, mesh, visualMesh);

			for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				ndShapeMaterial& material = node->m_body->GetAsBodyDynamic()->GetCollisionShape().m_shapeMaterial;
				material.m_userId = ndDemoContactCallback::m_modelPart;
			}

			ndSharedPtr<ndBrainAgentOnPolicyGradient_Agent> agent(new ndAgent(m_master, playerController, maxTrajectories));
			playerController->m_agent = (ndSharedPtr<ndBrainAgent>&) agent;
			m_master->AddAgent(agent);

			return model;
		}

		void OnDebug(ndDemoEntityManager* const, bool)
		{
		}

		virtual void Update(ndDemoEntityManager* const manager, ndFloat32)
		{
			ndUnsigned32 stopTraining = m_master->GetFramesCount();
			if (stopTraining <= m_stopTraining)
			{
				ndUnsigned32 episodeCount = m_master->GetEposideCount();
				m_master->OptimizeStep();
			
				episodeCount -= m_master->GetEposideCount();
				if (episodeCount)
				{
					const ndFloat32 score = m_master->GetAverageScore();

					if ((stopTraining > m_stopTraining / 3) && (score >= m_savedScore))
					{
						m_savedScore = score;

						// save partial controller in case of crash 
						ndBrain* const actor = *m_master->GetPolicyNetwork();
						ndString fileName(ndGetWorkingFileName(m_master->GetName().GetStr()));
						actor->SaveToFile(fileName.GetStr());
						ndExpandTraceMessage("best actor episode: %d\treward %f\ttrajectoryFrames: %f\n", m_master->GetEposideCount(), score, m_master->GetAverageFrames());
					}

					if (!m_master->IsSampling())
					{
						ndExpandTraceMessage("steps: %d\treward: %g\t  trajectoryFrames: %g\n", m_master->GetFramesCount(), score, m_master->GetAverageFrames());
						if (m_outFile)
						{
							fprintf(m_outFile, "%g\n", score);
							fflush(m_outFile);
						}
					}
				}
			}
			
			if ((stopTraining >= m_stopTraining) || (m_master->GetAverageScore() > ndBrainFloat(0.96f)))
			{
				m_modelIsTrained = true;
				ndUnsigned64 timer = ndGetTimeInMicroseconds() - m_timer;
				ndExpandTraceMessage("training complete\n");
				ndExpandTraceMessage("training time: %g seconds\n", ndFloat32(ndFloat64(timer) * ndFloat32(1.0e-6f)));
		
				manager->Terminate();
			}
		}

		ndSharedPtr<ndBrainAgentOnPolicyGradient_Trainer> m_master;
		ndList<ndModelArticulation*> m_models;
		FILE* m_outFile;
		ndUnsigned64 m_timer;
		ndFloat32 m_savedScore;
		ndFloat32 m_discountRewardFactor;
		ndFloat32 m_horizon;
		ndUnsigned32 m_lastEpisode;
		ndUnsigned32 m_stopTraining;
		bool m_modelIsTrained;
	};
}
using namespace ndUnicycleTrainer_ppo;

void ndUnicycleTrainingPPO(ndDemoEntityManager* const scene)
{
	//ndSharedPtr<ndBody> ground(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));
	ndSharedPtr<ndBody> ground(BuildFlatPlane(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", true));

	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend());
	scene->SetDemoHelp(demoHelper);

	// get the material graph
	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();

	// overload the ground friction
	// make sure the ground has enough friction
	ndMaterial* const defaultMaterial = callback->GetMaterial(ndDemoContactCallback::m_default, ndDemoContactCallback::m_default);
	ndAssert(defaultMaterial);
	defaultMaterial->m_dynamicFriction0 = defaultMaterial->m_staticFriction0;
	defaultMaterial->m_dynamicFriction1 = defaultMaterial->m_staticFriction1;

	// create a material that make models in training non collidable
	ndModelMaterial material;
	callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_modelPart);

	//load the mesh so that is can be re used
	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("unicycle.nd"));

	// create the trainer agent
	ndSharedPtr<ndDemoEntityManager::OnPostUpdate>trainer(new TrainingUpdata(scene, matrix, loader));
	scene->RegisterPostUpdate(trainer);

	// suppress v sync refresh rate for fast training
	scene->SetAcceleratedUpdate();
	
	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += 20.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}