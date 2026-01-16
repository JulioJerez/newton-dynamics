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
#include "ndMeshLoader.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"

namespace ndUnicyclePlayer
{
	#define CONTROLLER_NAME_SAC		"unicycleSac"
	#define CONTROLLER_NAME_PPO		"unicyclePpo"

	#define BOX_MASS				ndFloat32(20.0f)
	#define POLE_MASS				ndFloat32(1.0f)
	#define BALL_MASS				ndFloat32(5.0f)

	//#define TRAJECTORY_STEPS		(1024 * 4)
	//
	//#define PUSH_ACCEL				ndBrainFloat (-10.0f * DEMO_GRAVITY)
	//#define REWARD_MIN_ANGLE		ndBrainFloat (20.0f * ndDegreeToRad)

	#define ND_MAX_LEG_JOINT_ANGLE	(ndFloat32 (45.0f) * ndDegreeToRad)

	#define ND_MAX_WHEEL_ALPHA		(ndFloat32 (500.0f))

	#define ND_TERMINATION_ANGLE	(ndFloat32 (25.0f) * ndDegreeToRad)
	#define ND_TRAJECTORY_STEPS		(1024 * 4)

	enum ndActionSpace
	{
		m_wheelTorque,
		m_actionsSize
	};

	enum ndStateSpace
	{
		m_poleAngle,
		m_poleOmega,
		m_wheelOmega,
		m_wheelVelocity,
		m_poleJointAngle,
		m_poleJointOmega,
		m_hasSupportContact,
		m_observationsSize
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

			void GetObservation(ndBrainFloat* const observation)
			{
				m_owner->GetObservation(observation);
			}

			virtual void ApplyActions(ndBrainFloat* const actions)
			{
				m_owner->ApplyActions(actions);
			}

			bool IsTerminal() const
			{
				return m_owner->IsTerminal();
			}
			ndController* m_owner;
		};

		ndController();

		void Update(ndFloat32 timestep);

		void ResetModel();
		ndBrainFloat IsOnAir() const;

		bool IsTerminal() const;
		ndFloat32 GetPoleAngle() const;
		ndFloat32 GetPoleOmega() const;
		ndFloat32 GetBoxAngle() const;
		ndFloat32 GetBoxOmega() const;
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
		ndSharedPtr<ndBrainAgent> m_agent;
		ndFloat32 m_timestep;
	};
};
