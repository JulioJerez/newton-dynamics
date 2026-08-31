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

	#define ND_MAX_WHEEL_ALPHA		(ndFloat32 (500.0f))
	#define ND_TERMINATION_ANGLE	(ndFloat32 (45.0f) * ndDegreeToRad)
	#define ND_MAX_LEG_JOINT_ANGLE	(ndFloat32 (45.0f) * ndDegreeToRad)

	enum ndActionSpace
	{
		m_wheelTorque,
		m_actionsSize
	};

	enum ndStateSpace
	{
		m_velocity,
		m_poleAngle,
		m_poleOmega,
		m_wheelOmega,
		m_hasSupportContact,
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

		ndApplicationMaterial* Clone() const override
		{
			return new ndModelMaterial(*this);
		}

		virtual bool OnAabbOverlap(const ndBodyKinematic* const, const ndBodyKinematic* const) const override
		{
			return false;
		}
	};

	class ndBasePose
	{
		public:
		ndBasePose()
			:m_body(nullptr)
		{
		}

		ndBasePose(ndBodyDynamic* const body)
			:m_veloc(body->GetVelocity())
			,m_omega(body->GetOmega())
			,m_posit(body->GetPosition())
			,m_rotation(body->GetRotation())
			,m_body(body)
		{
		}

		void SetPose() const
		{
			m_body->SetMatrix(ndCalculateMatrix(m_rotation, m_posit));
			m_body->SetOmega(m_omega);
			m_body->SetVelocity(m_veloc);
		}

		ndVector m_veloc;
		ndVector m_omega;
		ndVector m_posit;
		ndQuaternion m_rotation;
		ndBodyDynamic* m_body;
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
		void Update(ndFloat32 timestep, ndInt32 threadId) override;
		void PostUpdate(ndFloat32 timestep, ndInt32 threadId) override;

		void ResetModel();
		void CalculateWholeInvInertia();
		ndBrainFloat IsOnAir() const;

		bool IsTerminal() const;
		ndFloat32 GetBoxAngle() const;
		ndFloat32 GetBoxOmega() const;
		ndFloat32 GetPoleAngle() const;
		ndFloat32 GetPoleOmega() const;
		//ndFloat32 GetWheelOmega() const;
		ndFloat32 GetWheelAlpha() const;
		ndBrainFloat CalculateReward() const;
		void ApplyActions(ndBrainFloat* const actions);
		void GetObservation(ndBrainFloat* const observation);

		void CreateArticulatedModel(
			ndDemoEntityManager* const scene,
			ndModelArticulation* const model,
			ndSharedPtr<ndMesh> mesh,
			ndSharedPtr<ndRenderSceneNode> visualMesh);

		static ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndRenderMeshLoader& loader, const char* const name);

		mutable ndIkSolver m_solver;
		ndSharedPtr<ndBody> m_pole;
		ndSharedPtr<ndBody> m_wheel;
		ndSharedPtr<ndBody> m_topBox;
		ndSharedPtr<ndJointBilateralConstraint> m_plane;
		ndSharedPtr<ndJointBilateralConstraint> m_poleHinge;
		ndSharedPtr<ndJointBilateralConstraint> m_wheelRoller;
		ndSharedPtr<ndBrainAgent> m_agent;
		ndFixSizeArray<ndBasePose, 8> m_basePose;
		ndFloat32 m_timestep;
		ndFloat32 m_invInertiaScale;
		bool m_isTrainning;
	};
};
