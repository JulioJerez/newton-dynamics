/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndBrainStdafx.h"
#include "ndBrainVector.h"
#include "ndBrainAgentContinuePolicyGradient.h"

#define ND_POLICY_MAX_SIGMA_SQUARE	ndBrainFloat(1.0f)
#define ND_POLICY_MIN_SIGMA_SQUARE	ndBrainFloat(0.01f)

ndContinuePolicyGradientHyperParameters::ndContinuePolicyGradientHyperParameters()
{
	m_randomSeed = 47;
	m_learnRate = ndBrainFloat(1.0e-4f);
	m_minSigmaSquared = ND_POLICY_MIN_SIGMA_SQUARE;
	m_maxSigmaSquared = ND_POLICY_MAX_SIGMA_SQUARE;

	m_policyRegularizer = ndBrainFloat(1.0e-4f);
	m_criticRegularizer = ndBrainFloat(1.0e-3f);

	m_discountRewardFactor = ndBrainFloat(0.99f);

	m_miniBatchSize = 256;
	m_numberOfActions = 0;
	m_numberOfObservations = 0;

	m_numberOfHiddenLayers = 3;
	m_hiddenLayersNumberOfNeurons = 128;
	
	m_maxTrajectorySteps = 4096;
	m_maxNumberOfTrainingSteps = 0;
	m_policyRegularizerType = m_ridge;
	m_criticRegularizerType = m_ridge;

	m_useGpuBackend = true;
}

ndBrainAgentContinuePolicyGradient::ndBrainAgentContinuePolicyGradient(const ndSharedPtr<ndBrain>& policy)
	:ndBrainAgent(policy)
{
	m_actions.SetCount(m_brain->GetOutputSize());
	m_observations.SetCount(m_brain->GetInputSize());
}

ndBrainAgentContinuePolicyGradient::ndBrainAgentContinuePolicyGradient(const ndBrainAgentContinuePolicyGradient& src)
	:ndBrainAgent(src)
{
	SetBrain(src.m_brain);
}

ndBrainAgentContinuePolicyGradient::~ndBrainAgentContinuePolicyGradient()
{
}

bool ndBrainAgentContinuePolicyGradient::IsTrainer() const
{
	return false;
}

void ndBrainAgentContinuePolicyGradient::InitWeights()
{
	ndAssert(0);
}

bool ndBrainAgentContinuePolicyGradient::IsTerminal() const
{
	ndAssert(0);
	return false;
}

ndBrainFloat ndBrainAgentContinuePolicyGradient::CalculateReward()
{
	ndAssert(0);
	return ndBrainFloat(0.0f);
}

void ndBrainAgentContinuePolicyGradient::ResetModel()
{
	ndAssert(0);
}

ndInt32 ndBrainAgentContinuePolicyGradient::GetEpisodeFrames() const
{
	ndAssert(0);
	return 0;
}

ndFloat32 ndBrainAgentContinuePolicyGradient::GetExpectedReward() const
{
	return ndFloat32(0.0f);
}

void ndBrainAgentContinuePolicyGradient::Save(ndBrainSave* const)
{
	ndAssert(0);
}

void ndBrainAgentContinuePolicyGradient::OptimizeStep()
{
}

void ndBrainAgentContinuePolicyGradient::Step()
{
	GetObservation(&m_observations[0]);
	m_brain->MakePrediction(m_observations, m_actions);
	ApplyActions(&m_actions[0]);
}
