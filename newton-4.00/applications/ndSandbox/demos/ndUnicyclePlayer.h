/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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

namespace ndUnicyclePlayer
{
	#define CONTROLLER_NAME_SAC		"unicycleSac"
	#define CONTROLLER_NAME_PPO		"unicyclePpo"

	#define BOX_MASS				ndFloat32(10.0f)
	#define POLE_MASS				ndFloat32(1.0f)
	#define BALL_MASS				ndFloat32(5.0f)

	#define ND_MAX_LEG_JOINT_ANGLE	(ndFloat32 (45.0f) * ndDegreeToRad)

	#define ND_MAX_WHEEL_ALPHA		(ndFloat32 (100.0f))

	#define ND_TERMINATION_ANGLE	(ndFloat32 (45.0f) * ndDegreeToRad)
	#define ND_TRAJECTORY_STEPS		(1024 * 4)

	#define ND_RANDOM_IMPULSE_MOD		256
	#define ND_RANDOM_IMPULSE_MAGNITUD	ndFloat32 (7.0f)


	enum ndActionSpace
	{
		m_wheelTorque,
		m_actionsSize
	};

	enum ndStateSpace
	{
		m_boxAngle,
		m_boxOmega,
		m_hingeOmega,
		m_hingeAngle,
		m_comSpeed,
		m_hasContactSupport,
		m_observationsSize
	};

	class ndModelMaterial : public ndApplicationMaterial
	{
		public:
		ndModelMaterial()
			:ndApplicationMaterial()
		{
		}

		ndModelMaterial(const ndModelMaterial& src)
			:ndApplicationMaterial(src)
		{
		}

		ndApplicationMaterial* Clone() const
		{
			return new ndModelMaterial(*this);
		}

		virtual bool OnAabbOverlap(const ndBodyKinematic* const, const ndBodyKinematic* const) const override
		{
			return false;
		}
	};

	class ndController : public ndModelNotify
	{
		public:
		class ndAgent : public ndBrainAgentContinuePolicyGradient
		{
			public:
			ndAgent(ndSharedPtr<ndBrain>& brain, ndController* const owner)
				:ndBrainAgentContinuePolicyGradient(brain)
				,m_owner(owner)
			{
			}

			void GetObservation(ndBrainFloat* const observation) override
			{
				m_owner->GetObservation(observation);
			}

			virtual void ApplyActions(ndBrainFloat* const actions) override
			{
				m_owner->ApplyActions(actions);
			}

			bool IsTerminal() const override
			{
				return m_owner->IsTerminal();
			}

			virtual ndFloat32 GetExpectedReward() const override
			{
				return 0.0f;
			}
			ndController* m_owner;
		};

		ndController();
		void Update(ndFloat32 timestep) override;
		void PostUpdate(ndFloat32 timestep) override;

		void ResetModel();
		ndBrainFloat IsOnAir() const;

		bool IsTerminal() const;
		ndFloat32 GetBoxAngle() const;
		ndFloat32 GetBoxOmega() const;
		ndFloat32 GetPoleAngle() const;
		ndBrainFloat CalculateReward() const;
		void ApplyActions(ndBrainFloat* const actions);
		void GetObservation(ndBrainFloat* const observation);

		void CreateArticulatedModel(
			ndDemoEntityManager* const scene,
			ndModelArticulation* const model,
			ndSharedPtr<ndMesh> mesh,
			ndSharedPtr<ndRenderSceneNode> visualMesh);

		static ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndRenderMeshLoader& loader, const char* const name);

		ndSharedPtr<ndBody> m_pole;
		ndSharedPtr<ndBody> m_wheel;
		ndSharedPtr<ndBody> m_topBox;
		ndSharedPtr<ndJointBilateralConstraint> m_plane;
		ndSharedPtr<ndJointBilateralConstraint> m_poleHinge;
		ndSharedPtr<ndJointBilateralConstraint> m_wheelRoller;
		ndSharedPtr<ndIkSolver> m_solver;
		ndSharedPtr<ndBrainAgent> m_agent;
		ndFloat32 m_timestep;
		ndInt32 m_randomImpulseCounter;
		bool m_isTrainning;
	};
};
