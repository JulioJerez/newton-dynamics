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

#include "ndBrainLayer.h"
#include "ndBrainTrainer.h"
#include "ndBrainSaveLoad.h"
#include "ndBrainCpuContext.h"
#include "ndBrainDualNumber.h"
#include "ndBrainGpuContext.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainIntegerBuffer.h"
#include "ndBrainOptimizerAdam.h"
#include "ndBrainLayerActivationRelu.h"
#include "ndBrainLayerActivationTanh.h"
#include "ndBrainLossLeastSquaredError.h"
#include "ndBrainLayerActivationLeakyRelu.h"
#include "ndBrainAgentPolicyGradientActivation.h"
#include "ndBrainAgentOnPolicyGradient_Trainer.h"
#include "ndBrainLayerActivationLinearNormalize.h"

//#define ND_DEBUG_CONTINUE_PROXIMA_POLICY
//#define ND_CONTINUE_PROXIMA_POLICY_BOOTHSTRAP_METHOD

#define ND_POLICY_MAX_KL_DIVERGENCE_PASSES			8
#define ND_MAX_MINIBATCHES_ITERATIONS				64
#define ND_POLICY_DOWN_SAMPLE_LEARN_RATE			ndBrainFloat(1.0f)
#define ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON		ndBrainFloat(0.2f)
#define ND_POLICY_KL_DIVERGENCE_STOP_THRESHHOLD		ndBrainFloat(1.0e-3f)

ndBrainAgentOnPolicyGradient_Trainer::HyperParameters::HyperParameters()
{
	m_batchTrajectoryCount = 1000;
	m_divergenceMaxPasses = ND_POLICY_MAX_KL_DIVERGENCE_PASSES;
	m_divergenceStopThreshold = ND_POLICY_KL_DIVERGENCE_STOP_THRESHHOLD;
}

ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::ndTrajectory()
	:m_reward()
	,m_terminal()
	,m_actions()
	,m_observations()
	,m_nextObservations()
	,m_actionsSize(0)
	,m_obsevationsSize(0)
{
}

ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::ndTrajectory(ndInt32 actionsSize, ndInt32 obsevationsSize)
	:m_reward()
	,m_terminal()
	,m_actions()
	,m_observations()
	,m_nextObservations()
	,m_actionsSize(0)
	,m_obsevationsSize(0)
{
	Init(actionsSize, obsevationsSize);
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::Init(ndInt32 actionsSize, ndInt32 obsevationsSize)
{
	m_actionsSize = actionsSize * 2;
	m_obsevationsSize = obsevationsSize;
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::Clear(ndInt32 entry)
{
	m_reward[entry] = ndBrainFloat(0.0f);
	m_terminal[entry] = ndBrainFloat(0.0f);
	m_expectedReward[entry] = ndBrainFloat(0.0f);
	ndMemSet(&m_actions[entry * m_actionsSize], ndBrainFloat(0.0f), m_actionsSize);
	ndMemSet(&m_observations[entry * m_obsevationsSize], ndBrainFloat(0.0f), m_obsevationsSize);
	ndMemSet(&m_nextObservations[entry * m_obsevationsSize], ndBrainFloat(0.0f), m_obsevationsSize);
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::CopyFrom(ndInt32 entry, ndTrajectory& src, ndInt32 srcEntry)
{
	m_reward[entry] = src.m_reward[srcEntry];
	m_terminal[entry] = src.m_terminal[srcEntry];
	m_expectedReward[entry] = src.m_expectedReward[srcEntry];
	ndMemCpy(&m_actions[entry * m_actionsSize], &src.m_actions[srcEntry * m_actionsSize], m_actionsSize);
	ndMemCpy(&m_observations[entry * m_obsevationsSize], &src.m_observations[srcEntry * m_obsevationsSize], m_obsevationsSize);
	ndMemCpy(&m_nextObservations[entry * m_obsevationsSize], &src.m_nextObservations[srcEntry * m_obsevationsSize], m_obsevationsSize);
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetCount() const
{
	return ndInt32(m_reward.GetCount());
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::SetCount(ndInt32 count)
{
	m_reward.SetCount(count);
	m_terminal.SetCount(count);
	m_expectedReward.SetCount(count);
	m_actions.SetCount(count * m_actionsSize);
	m_observations.SetCount(count * m_obsevationsSize);
	m_nextObservations.SetCount(count * m_obsevationsSize);
}

ndBrainFloat ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetReward(ndInt32 entry) const
{
	return m_reward[entry];
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::SetReward(ndInt32 entry, ndBrainFloat reward)
{
	m_reward[entry] = reward;
}

ndBrainFloat ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetExpectedReward(ndInt32 entry) const
{
	return m_expectedReward[entry];
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::SetExpectedReward(ndInt32 entry, ndBrainFloat expectedReward)
{
	m_expectedReward[entry] = expectedReward;
}

bool ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetTerminalState(ndInt32 entry) const
{
	return (m_terminal[entry] == 0.0f) ? true : false;
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::SetTerminalState(ndInt32 entry, bool isTernimal)
{
	m_terminal[entry] = isTernimal ? ndBrainFloat(0.0f) : ndBrainFloat(1.0f);
}

ndBrainFloat* ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetActions(ndInt32 entry)
{
	return &m_actions[entry * m_actionsSize];
}

const ndBrainFloat* ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetActions(ndInt32 entry) const
{
	return &m_actions[entry * m_actionsSize];
}

ndBrainFloat* ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetObservations(ndInt32 entry)
{
	return &m_observations[entry * m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetObservations(ndInt32 entry) const
{
	return &m_observations[entry * m_obsevationsSize];
}

ndBrainFloat* ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetNextObservations(ndInt32 entry)
{
	return &m_nextObservations[entry * m_obsevationsSize];
}

const ndBrainFloat* ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetNextObservations(ndInt32 entry) const
{
	return &m_nextObservations[entry * m_obsevationsSize];
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetRewardOffset() const
{
	return 0;
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetExpectedRewardOffset() const
{
	return GetRewardOffset() + 1;
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetTerminalOffset() const
{
	return GetExpectedRewardOffset() + 1;
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetActionOffset() const
{
	return GetTerminalOffset() + 1;
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetObsevationOffset() const
{
	return m_actionsSize + GetActionOffset();
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetNextObsevationOffset() const
{
	return m_obsevationsSize + GetObsevationOffset();
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetStride() const
{
	return m_obsevationsSize + GetNextObsevationOffset();
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetFlatArray(ndInt32 index, ndBrainVector& output) const
{
	output.SetCount(GetStride());
	output[GetRewardOffset()] = m_reward[index];
	output[GetExpectedRewardOffset()] = m_expectedReward[index];
	output[GetTerminalOffset()] = m_terminal[index];
	ndMemCpy(&output[GetActionOffset()], &m_actions[index * m_actionsSize], m_actionsSize);
	ndMemCpy(&output[GetObsevationOffset()], &m_observations[index * m_obsevationsSize], m_obsevationsSize);
	ndMemCpy(&output[GetNextObsevationOffset()], &m_nextObservations[index * m_obsevationsSize], m_obsevationsSize);
}

ndBrainAgentOnPolicyGradient_Agent::ndBrainAgentOnPolicyGradient_Agent(ndBrainAgentOnPolicyGradient_Trainer* const master, ndInt32 maxTrajecxtoires)
	:ndBrainAgent(master->GetPolicyNetwork())
	,m_trajectory()
	,m_normalDistribution()
	,m_owner(master)
	,m_trajectoryIndex(0)
	,m_maxTrajectores(maxTrajecxtoires)
	,m_isDead(false)
{
	m_trajectory.Init(master->m_parameters.m_numberOfActions, master->m_parameters.m_numberOfObservations);

	ndUnsigned32 seed = master->m_uniformDistribution.Generate();
	m_normalDistribution.Init(seed);
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::GetEpisodeFrames() const
{
	ndAssert(0);
	return 0;
}

ndFloat32 ndBrainAgentOnPolicyGradient_Agent::GetExpectedReward() const
{
	ndAssert(m_trajectory.GetCount());
	ndFloat32 gamma = m_owner->m_parameters.m_discountRewardFactor;
	ndFloat32 sum = m_trajectory.GetReward(m_trajectory.GetCount() - 1);
	for (ndInt32 i = m_trajectory.GetCount() - 2; i >= 0; --i)
	{
		ndFloat32 r = m_trajectory.GetReward(i);
		sum = r + gamma * sum;
	}
	return sum;
}

void ndBrainAgentOnPolicyGradient_Agent::SampleActions(ndBrainVector& actions)
{
	const ndInt32 size = ndInt32(actions.GetCount()) / 2;
	for (ndInt32 i = size - 1; i >= 0; --i)
	{
		ndBrainFloat sigma = actions[size + i];
		ndBrainFloat normalSample = ndBrainFloat(m_normalDistribution());
		ndBrainFloat sample = ndBrainFloat(actions[i]) + normalSample * sigma;
		ndBrainFloat clippedAction = ndClamp(sample, ndBrainFloat(-1.0f), ndBrainFloat(1.0f));
		actions[i] = clippedAction;
	}
}

void ndBrainAgentOnPolicyGradient_Agent::Step()
{
	ndInt32 entryIndex = m_trajectory.GetCount();
	m_trajectory.SetCount(entryIndex + 1);
	m_trajectory.Clear(entryIndex);

	ndBrainAgentOnPolicyGradient_Trainer* const owner = *m_owner;

	const ndBrain* const policy = *GetBrain();
	ndBrainMemVector actions(m_trajectory.GetActions(entryIndex), policy->GetOutputSize());
	ndBrainMemVector observation(m_trajectory.GetObservations(entryIndex), owner->m_parameters.m_numberOfObservations);
	
	GetObservation(&observation[0]);
	policy->MakePrediction(observation, actions);
	SampleActions(actions);
	ApplyActions(&actions[0]);

	bool isdead = IsTerminal();
	ndBrainFloat reward = CalculateReward();
	m_trajectory.SetReward(entryIndex, reward);
	m_trajectory.SetTerminalState(entryIndex, isdead);
	m_isDead = m_isDead || isdead;
}

ndBrainAgentOnPolicyGradient_Trainer::ndBrainAgentOnPolicyGradient_Trainer(const HyperParameters& parameters)
	:ndClassAlloc()
	,m_name()
	,m_parameters(parameters)
	,m_context()
	,m_policyTrainer(nullptr)
	,m_criticTrainer(nullptr)
	,m_uniformDistribution()
	,m_agents()
	,m_trainingBuffer(nullptr)
	,m_advantageBuffer(nullptr)
	,m_policyActionBuffer(nullptr)
	,m_invLikelihoodBuffer(nullptr)
	,m_policyGradientAccumulator(nullptr)
	,m_minibatchAdvantageBuffer(nullptr)
	,m_minibatchInvLikelihoodBuffer(nullptr)
	,m_minibatchLikelihoodRatioBuffer(nullptr)
	,m_minibatchCriticStateValueBuffer(nullptr)
	,m_minibatchBrocastAdvantageBuffer(nullptr)
	,m_minibatchClippedLikelihoodRatioBuffer(nullptr)
	,m_randomShuffleBuffer(nullptr)
	,m_randomCriticShuffleBuffer(nullptr)
	,m_minibatchRandomShuffleBuffer(nullptr)
	,m_minibatchCriticRandomShuffleBuffer(nullptr)
	,m_lastPolicy()
	,m_scratchBuffer()
	,m_shuffleBuffer()
	,m_trajectoryAccumulator()
	,m_averageExpectedRewards()
	,m_averageFramesPerEpisodes()
	,m_learnRate(m_parameters.m_learnRate)
	,m_frameCount(0)
	,m_horizonSteps(0)
	,m_eposideCount(0)
	,m_trajectiesCount(0)
	,m_numberOfIterations(0)
{
	ndAssert(m_parameters.m_numberOfActions);
	ndAssert(m_parameters.m_numberOfObservations);

	ndSetRandSeed(ndUnsigned32(m_parameters.m_randomSeed));
	m_uniformDistribution.Init(ndRandInt());

	m_trajectoryAccumulator.Init(m_parameters.m_numberOfActions, m_parameters.m_numberOfObservations);
	m_parameters.m_discountRewardFactor = ndClamp(m_parameters.m_discountRewardFactor, ndBrainFloat(0.9f), ndBrainFloat(0.999f));

	if (m_parameters.m_useGpuBackend)
	{
		m_context = ndSharedPtr<ndBrainContext>(new ndBrainGpuContext);
	}
	else
	{
		m_context = ndSharedPtr<ndBrainContext>(new ndBrainCpuContext);
	}
	
	ndFloat32 gain = ndFloat32(1.0f);
	ndFloat32 maxGain = ndFloat32(0.99f) / (ndFloat32(1.0f) - m_parameters.m_discountRewardFactor);
	for (ndInt32 i = 0; (i < m_parameters.m_maxTrajectorySteps) && (gain < maxGain); ++i)
	{
		gain = ndFloat32(1.0f) + m_parameters.m_discountRewardFactor * gain;
		m_horizonSteps++;
	}

	m_parameters.m_divergenceMaxPasses = ndClamp(m_parameters.m_divergenceMaxPasses, 1, ND_POLICY_MAX_KL_DIVERGENCE_PASSES);
	m_parameters.m_divergenceStopThreshold = ndClamp(m_parameters.m_divergenceStopThreshold, ND_POLICY_KL_DIVERGENCE_STOP_THRESHHOLD, ndBrainFloat(0.01f));

	// create actor class
	BuildPolicyClass();
	BuildCriticClass();

	m_policyGradientAccumulator = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_policyTrainer->GetWeightAndBiasGradientBuffer()));

	m_minibatchCriticStateValueBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minibatchAdvantageBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minibatchBrocastAdvantageBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, 2 * m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_minibatchInvLikelihoodBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minibatchLikelihoodRatioBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minibatchRandomShuffleBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minibatchCriticRandomShuffleBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minibatchClippedLikelihoodRatioBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));

	m_meanBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_sigmaBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	//m_reparametrizedMeanBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	//m_reparametrizedSgmaBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_minibatchGaussianDistribution = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_zMeanBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_invSigmaBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_invSigma2Buffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_meanGradiendBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_sigmaGradiendBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));

	m_advantageBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * ND_MAX_MINIBATCHES_ITERATIONS));
	m_invLikelihoodBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * ND_MAX_MINIBATCHES_ITERATIONS));
	m_policyActionBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, 2 * m_parameters.m_numberOfActions * m_parameters.m_miniBatchSize * ND_MAX_MINIBATCHES_ITERATIONS));

	//m_uniformDistributionBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_divergenceMaxPasses * m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps * m_parameters.m_numberOfActions));
	m_uniformDistributionBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps * m_parameters.m_numberOfActions));
	m_trainingBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_trajectoryAccumulator.GetStride() * (m_parameters.m_batchTrajectoryCount + 1) * m_parameters.m_maxTrajectorySteps));

	ndInt32 shuffleSize0 = m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps;
	ndInt32 shuffleSize1 = m_parameters.m_miniBatchSize * m_parameters.m_divergenceMaxPasses * ND_MAX_MINIBATCHES_ITERATIONS;
	m_randomShuffleBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, ndMax(shuffleSize0, shuffleSize1)));
	m_randomCriticShuffleBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, ndMax(shuffleSize0, shuffleSize1)));
}

ndSharedPtr<ndBrain> ndBrainAgentOnPolicyGradient_Trainer::GetPolicyNetwork()
{
	return m_policyTrainer->GetBrain();
}

void ndBrainAgentOnPolicyGradient_Trainer::AddAgent(ndSharedPtr<ndBrainAgentOnPolicyGradient_Agent>& agent)
{
	m_agents.Append(agent);
	agent->m_owner = ndWeakPtr<ndBrainAgentOnPolicyGradient_Trainer>(this);
}

const ndString& ndBrainAgentOnPolicyGradient_Trainer::GetName() const
{
	return m_name;
}

void ndBrainAgentOnPolicyGradient_Trainer::SetName(const ndString& name)
{
	m_name = name;
}

void ndBrainAgentOnPolicyGradient_Trainer::BuildPolicyClass()
{
	ndFixSizeArray<ndBrainLayer*, 32> layers(0);

	//layers.PushBack(new ndBrainLayerActivationLinearNormalize(m_parameters.m_numberOfObservations));
	layers.PushBack(new ndBrainLayerLinear(m_parameters.m_numberOfObservations, m_parameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));

	for (ndInt32 i = 0; i < m_parameters.m_numberOfHiddenLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		//layers.PushBack(new ndBrainLayerActivationRelu(layers[layers.GetCount() - 1]->GetOutputSize()));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_numberOfActions * 2));
	layers.PushBack(new ndBrainAgentPolicyGradientActivation(layers[layers.GetCount() - 1]->GetOutputSize(), ndBrainFloat(ndSqrt(m_parameters.m_minSigmaSquared)), ndBrainFloat(ndSqrt(m_parameters.m_maxSigmaSquared))));

	ndSharedPtr<ndBrain> policy (new ndBrain);
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		policy->AddLayer(layers[i]);
	}
	policy->InitWeights();
	m_policyInputNormalization = ndWeakPtr<ndBrainLayerActivationLinearNormalize> ((ndBrainLayerActivationLinearNormalize*)policy->FindLayer(ND_BRAIN_LAYER_ACTIVATION_LINEAR_NORMALIZE_NAME));

	ndSharedPtr<ndBrainOptimizer> optimizer(new ndBrainOptimizerAdam(m_context));
	optimizer->SetRegularizer(m_parameters.m_policyRegularizer);
	optimizer->SetRegularizerType(m_parameters.m_policyRegularizerType);

	ndTrainerDescriptor descriptor(policy, m_context, m_parameters.m_miniBatchSize);
	m_policyTrainer = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptor, optimizer));
}

void ndBrainAgentOnPolicyGradient_Trainer::BuildCriticClass()
{
	ndFixSizeArray<ndBrainLayer*, 32> layers(0);
	const ndBrain& policy = **m_policyTrainer->GetBrain();

	layers.PushBack(new ndBrainLayerLinear(policy.GetInputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));

	for (ndInt32 i = 0; i < m_parameters.m_numberOfHiddenLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		//layers.PushBack(new ndBrainLayerActivationRelu(layers[layers.GetCount() - 1]->GetOutputSize()));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), 1));
	layers.PushBack(new ndBrainLayerActivationLeakyRelu(layers[layers.GetCount() - 1]->GetOutputSize()));
	
	ndSharedPtr<ndBrain> critic(new ndBrain);
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		critic->AddLayer(layers[i]);
	}
	critic->InitWeights();
	
	ndSharedPtr<ndBrainOptimizer> optimizer(new ndBrainOptimizerAdam(m_context));
	optimizer->SetRegularizer(m_parameters.m_criticRegularizer);
	optimizer->SetRegularizerType(m_parameters.m_criticRegularizerType);

	ndTrainerDescriptor descriptor(critic, m_context, m_parameters.m_miniBatchSize);
	m_criticTrainer = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptor, optimizer));
}

bool ndBrainAgentOnPolicyGradient_Trainer::IsValid() const
{
	return m_context->IsValid();
}

bool ndBrainAgentOnPolicyGradient_Trainer::IsSampling() const
{
	return false;
}

ndUnsigned32 ndBrainAgentOnPolicyGradient_Trainer::GetFramesCount() const
{
	return m_frameCount;
}

ndUnsigned32 ndBrainAgentOnPolicyGradient_Trainer::GetEposideCount() const
{
	return m_eposideCount;
}

ndFloat32 ndBrainAgentOnPolicyGradient_Trainer::GetAverageScore() const
{
	ndBrainFloat maxScore = ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) - m_parameters.m_discountRewardFactor);
	ndBrainFloat score = ndBrainFloat(1.0f) * m_averageExpectedRewards.GetAverage() / maxScore;
	return score;
}

ndFloat32 ndBrainAgentOnPolicyGradient_Trainer::GetAverageFrames() const
{
	return m_averageFramesPerEpisodes.GetAverage();
}

void ndBrainAgentOnPolicyGradient_Trainer::SaveTrajectory(ndBrainAgentOnPolicyGradient_Agent* const agent)
{
	if (agent->m_trajectoryIndex < agent->m_maxTrajectores)
	{
		ndBrainAgentOnPolicyGradient_Agent::ndTrajectory& trajectory = agent->m_trajectory;
		// if the agent is dead, then remove all transitions past the last dead step.
		if (agent->m_isDead)
		{
			ndInt32 start = ndMax(0, ndInt32(trajectory.GetCount() - 64));
			for (ndInt32 i = start; i < trajectory.GetCount(); ++i)
			{
				if (trajectory.GetTerminalState(i))
				{
					trajectory.SetCount(i + 1);
					break;
				}
			}
		}

		ndMemCpy(trajectory.GetNextObservations(trajectory.GetCount() - 1), trajectory.GetObservations(trajectory.GetCount() - 1), m_parameters.m_numberOfObservations);
		for (ndInt32 i = trajectory.GetCount() - 2; i >= 0; --i)
		{
			ndMemCpy(trajectory.GetNextObservations(i), trajectory.GetObservations(i + 1), m_parameters.m_numberOfObservations);
		}

		ndBrainFloat gamma = m_parameters.m_discountRewardFactor;
		ndBrainFloat stateExpectedReward = trajectory.GetReward(trajectory.GetCount() - 1);
		if ((trajectory.GetCount() - 1) > m_parameters.m_maxTrajectorySteps)
		{
			// using the Bellman equation to calculate trajectory expected rewards score.
			// (Monte Carlo method)

			ndInt32 numberOfSteps = ndInt32(trajectory.GetCount());
			trajectory.m_expectedReward.SetCount(numberOfSteps);
			ndBrainFloat trajectoryReward = trajectory.GetReward(numberOfSteps - 1);
			trajectory.SetExpectedReward(numberOfSteps - 1, trajectoryReward);

			for (ndInt32 i = trajectory.GetCount() - 2; i >= m_parameters.m_maxTrajectorySteps; --i)
			{
				ndBrainFloat r = trajectory.GetReward(i);
				stateExpectedReward = r + gamma * stateExpectedReward;
				trajectory.SetExpectedReward(i, stateExpectedReward);
			}
			trajectory.SetCount(m_parameters.m_maxTrajectorySteps);
		}

		// (Monte Carlo method)
		// using the Bellman equation to calculate trajectory expected rewards score.
		const ndInt32 numberOfSteps = ndInt32 (trajectory.GetCount());
		trajectory.m_expectedReward.SetCount(numberOfSteps);
		ndBrainFloat trajectoryReward = trajectory.GetReward(numberOfSteps - 1);
		trajectory.SetExpectedReward(numberOfSteps - 1, trajectoryReward);
		for (ndInt32 i = trajectory.GetCount() - 2; i >= 0; --i)
		{
			ndBrainFloat r = trajectory.GetReward(i);
			stateExpectedReward = r + gamma * stateExpectedReward;
			trajectoryReward += stateExpectedReward;
			trajectory.SetExpectedReward(i, stateExpectedReward);
		}
		// remove the last terminal transition
		//trajectory.SetCount(trajectory.GetCount() - 1);

		// append the transitions to the end of the data buffer
		const ndInt32 base = m_trajectoryAccumulator.GetCount();
		m_trajectoryAccumulator.SetCount(m_trajectoryAccumulator.GetCount() + trajectory.GetCount());
		ndAssert(m_trajectoryAccumulator.GetCount() <= (m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps));
		for (ndInt32 i = 0; i < trajectory.GetCount(); ++i)
		{
			m_trajectoryAccumulator.CopyFrom(base + i, trajectory, i);
		}
		m_trajectiesCount++;
	}
	agent->m_trajectoryIndex++;
}

void ndBrainAgentOnPolicyGradient_Trainer::UpdateScore()
{
	ndBrainFloat averageSum = ndBrainFloat(0.0f);
	for (ndInt32 i = ndInt32(m_trajectoryAccumulator.GetCount()) - 1; i >= 0; --i)
	{
		averageSum += ndBrainFloat(m_trajectoryAccumulator.GetExpectedReward(i));
	}
	m_averageExpectedRewards.Update(averageSum / ndBrainFloat(m_trajectoryAccumulator.GetCount()));
	m_averageFramesPerEpisodes.Update(ndBrainFloat(m_trajectoryAccumulator.GetCount()) / ndBrainFloat(m_parameters.m_batchTrajectoryCount));
}

//#pragma optimize( "", off)
void ndBrainAgentOnPolicyGradient_Trainer::TrajectoryToGpuBuffers()
{
	const ndInt32 stride = m_trajectoryAccumulator.GetStride();
	if (m_trajectoryAccumulator.GetCount() < m_parameters.m_miniBatchSize)
	{
		// make sure we have at least one minibatch worth of data
		const ndInt32 transitionsCount = m_trajectoryAccumulator.GetCount();
		m_trajectoryAccumulator.SetCount(m_parameters.m_miniBatchSize);

		ndInt32 modIndex = 0;
		for (ndInt32 i = transitionsCount; i < m_parameters.m_miniBatchSize; ++i)
		{
			m_trajectoryAccumulator.CopyFrom(i, m_trajectoryAccumulator, modIndex);
			modIndex = (modIndex + 1) % transitionsCount;
		}
	}
	ndAssert(m_trajectoryAccumulator.GetCount() >= m_parameters.m_miniBatchSize);

	const ndInt32 numberOfSteps = m_trajectoryAccumulator.GetCount();
	m_trajectoryAccumulator.SetCount(numberOfSteps - numberOfSteps % m_parameters.m_miniBatchSize);
	
	const ndInt32 count = m_trajectoryAccumulator.GetCount();
	m_shuffleBuffer.SetCount(count);
	m_scratchBuffer.SetCount(count * stride);
	for (ndInt32 i = 0; i < count; ++i)
	{
		m_shuffleBuffer[i] = i;
		ndBrainMemVector dst(&m_scratchBuffer[i * stride], stride);
		m_trajectoryAccumulator.GetFlatArray(i, dst);
	}
	m_numberOfIterations = ndUnsigned32(ndMin(ndInt32(m_trajectoryAccumulator.GetCount() / m_parameters.m_miniBatchSize), ND_MAX_MINIBATCHES_ITERATIONS));

	ndAssert(m_shuffleBuffer.GetCount() >= m_parameters.m_miniBatchSize);
	m_shuffleBuffer.RandomShuffle(m_shuffleBuffer.GetCount());

	m_shuffleBuffer.SetCount(m_numberOfIterations * m_parameters.m_miniBatchSize);
	m_criticShuffleBuffer.SetCount(m_numberOfIterations * m_parameters.m_miniBatchSize);
	m_shuffleBufferBuilder.SetCount(m_numberOfIterations * m_parameters.m_miniBatchSize);
	m_criticShuffleBufferBuilder.SetCount(m_numberOfIterations * m_parameters.m_miniBatchSize);

	for (ndInt32 i = 0; i < ndInt32(m_numberOfIterations * m_parameters.m_miniBatchSize); ++i)
	{
		m_criticShuffleBufferBuilder[i] = i;
		m_shuffleBufferBuilder[i] = m_shuffleBuffer[i];
	}

	m_shuffleBuffer.SetCount(m_numberOfIterations * m_parameters.m_miniBatchSize * m_parameters.m_divergenceMaxPasses);
	m_criticShuffleBuffer.SetCount(m_numberOfIterations * m_parameters.m_miniBatchSize * m_parameters.m_divergenceMaxPasses);
	for (ndInt32 i = 0; i < ndInt32 (m_parameters.m_divergenceMaxPasses); ++i)
	{
		ndInt32 dst = ndInt32(i * m_numberOfIterations * m_parameters.m_miniBatchSize);
		ndMemCpy(&m_shuffleBuffer[dst], &m_shuffleBufferBuilder[0], m_numberOfIterations * m_parameters.m_miniBatchSize);
		ndMemCpy(&m_criticShuffleBuffer[dst], &m_criticShuffleBufferBuilder[0], m_numberOfIterations * m_parameters.m_miniBatchSize);

		for (ndInt32 j = ndInt32(m_numberOfIterations * m_parameters.m_miniBatchSize) - 1; j > 0; --j)
		{
			ndInt64 randomIndex = ndRandInt();
			ndInt64 k = randomIndex % j;
			ndSwap(m_shuffleBufferBuilder[k], m_shuffleBufferBuilder[j]);
			ndSwap(m_criticShuffleBufferBuilder[k], m_criticShuffleBufferBuilder[j]);
		}
	}

	m_trainingBuffer->VectorToDevice(m_scratchBuffer);
	m_randomShuffleBuffer->MemoryToDevice(0, m_shuffleBuffer.GetCount() * sizeof(ndInt32), &m_shuffleBuffer[0]);
	m_randomCriticShuffleBuffer->MemoryToDevice(0, m_shuffleBuffer.GetCount() * sizeof(ndInt32), &m_criticShuffleBuffer[0]);
	
	m_scratchBuffer.SetCount(m_numberOfIterations * m_parameters.m_miniBatchSize);
	for (ndInt32 i = ndInt32(m_numberOfIterations * m_parameters.m_miniBatchSize) - 1; i >= 0; --i)
	{
		m_scratchBuffer[i] = ndBrainFloat(m_uniformDistribution());
	}
	m_uniformDistributionBuffer->VectorToDevice(m_scratchBuffer);
}

#ifdef ND_CONTINUE_PROXIMA_POLICY_BOOTHSTRAP_METHOD
// using pure bootstrapping method, but the critic always blows up
void ndBrainAgentOnPolicyGradient_Trainer::CalculateAdvantage()
{
	ndBrainFloatBuffer* const inputBuffer = m_criticTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const outputBuffer = m_criticTrainer->GetOuputBuffer();

	ndCopyBufferCommandInfo advantageInfo;
	advantageInfo.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	advantageInfo.m_srcOffsetInByte = 0;
	advantageInfo.m_dstOffsetInByte = 0;
	advantageInfo.m_dstStrideInByte = advantageInfo.m_srcStrideInByte;
	advantageInfo.m_bytesToCopy = advantageInfo.m_srcStrideInByte;

	ndCopyBufferCommandInfo observationInfo;
	observationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	observationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	observationInfo.m_dstOffsetInByte = 0;
	observationInfo.m_dstStrideInByte = ndInt32(m_criticTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	observationInfo.m_bytesToCopy = observationInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo nextObservationInfo(observationInfo);
	nextObservationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetNextObsevationOffset() * sizeof(ndReal));

	ndCopyBufferCommandInfo stateRewardInfo(observationInfo);
	stateRewardInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetRewardOffset() * sizeof(ndReal));
	stateRewardInfo.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	stateRewardInfo.m_bytesToCopy = stateRewardInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo stateTerminalInfo(stateRewardInfo);
	stateTerminalInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetTerminalOffset() * sizeof(ndReal));

	const ndInt32 advantageStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	const ndInt32 numberOfBatches = ndInt32(m_trajectoryAccumulator.GetCount() / m_parameters.m_miniBatchSize);
	const ndInt32 transitionStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	ndAssert(numberOfBatches >= 1);

	// calculate GAE(l, 1) // very noisy, the policy colapse most of the time.
	// calculate GAE(l, 0) // too smooth, and doesn't seem to work either
	// just using bellman equation to calculate state expected reward.
	// advantage(i) = reward(i) + alive(i) * (gamma * Value(i + 1) - value(i))
	for (ndInt32 i = 0; i < numberOfBatches; ++i)
	{
		// calculate 
		// At = expectedStateValue - StateValue
		// Qt = r(t) + g * V(t+1) - V(t)
		// Qt = r(t) + (g * V(t+1) - V(t)) * terminal

		// get next state value
		inputBuffer->CopyBuffer(nextObservationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_criticTrainer->MakePrediction();
		outputBuffer->Scale(m_parameters.m_discountRewardFactor);
		m_minibatchAdvantageBuffer->Set(*outputBuffer);

		// get state value
		inputBuffer->CopyBuffer(observationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_criticTrainer->MakePrediction();
		m_minibatchAdvantageBuffer->Sub(*outputBuffer);

		// if state is terminal the value is zero
		outputBuffer->CopyBuffer(stateTerminalInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_minibatchAdvantageBuffer->Mul(*outputBuffer);

		// add the state rewrad
		outputBuffer->CopyBuffer(stateRewardInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_minibatchAdvantageBuffer->Add(*outputBuffer);
		m_minibatchAdvantageBuffer->Min(ndBrainFloat(10.0f));
		m_minibatchAdvantageBuffer->Max(ndBrainFloat(-10.0f));

		// save advantage
		m_advantageBuffer->CopyBuffer(advantageInfo, 1, **m_minibatchAdvantageBuffer);

		advantageInfo.m_dstOffsetInByte += advantageStrideInBytes;
		stateRewardInfo.m_srcOffsetInByte += transitionStrideInBytes;
		observationInfo.m_srcOffsetInByte += transitionStrideInBytes;
		stateTerminalInfo.m_srcOffsetInByte += transitionStrideInBytes;
		nextObservationInfo.m_srcOffsetInByte += transitionStrideInBytes;
	}
}

void ndBrainAgentOnPolicyGradient_Trainer::OptimizeCritic()
{
	// calculate value function by bootstrapping the trajectory transitions.
	ndCopyBufferCommandInfo shuffleBufferInfo;
	shuffleBufferInfo.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	shuffleBufferInfo.m_srcOffsetInByte = 0;
	shuffleBufferInfo.m_dstOffsetInByte = 0;
	shuffleBufferInfo.m_dstStrideInByte = shuffleBufferInfo.m_srcStrideInByte;
	shuffleBufferInfo.m_bytesToCopy = shuffleBufferInfo.m_srcStrideInByte;

	ndCopyBufferCommandInfo rewardInfo;
	rewardInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	rewardInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetRewardOffset() * sizeof(ndReal));
	rewardInfo.m_dstOffsetInByte = 0;
	rewardInfo.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	rewardInfo.m_bytesToCopy = rewardInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo terminalInfo(rewardInfo);
	terminalInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetTerminalOffset() * sizeof(ndReal));

	ndCopyBufferCommandInfo observationInfo(rewardInfo);
	observationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	observationInfo.m_dstStrideInByte = ndInt32(m_criticTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	observationInfo.m_bytesToCopy = observationInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo nextObservationInfo(observationInfo);
	nextObservationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetNextObsevationOffset() * sizeof(ndReal));

	ndBrainFloatBuffer* const inputBuffer = m_criticTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const outputBuffer = m_criticTrainer->GetOuputBuffer();
	ndBrainFloatBuffer* const outputGradientBuffer = m_criticTrainer->GetOuputGradientBuffer();

	//const ndInt32 numberOfIterations = ndMin(ndInt32(m_shuffleBuffer.GetCount()) / m_parameters.m_miniBatchSize, 1024);
	//only take 20% of the collected samples to train the critics.
	const ndInt32 samplesFraction = 5;
	const ndInt32 numberOfIterations = ((ndInt32(m_shuffleBuffer.GetCount()) / m_parameters.m_miniBatchSize) + samplesFraction - 1) / samplesFraction;
	ndAssert(numberOfIterations >= 1);

	// calculate GAE(? = 0): too smooth, and also doesn’t seem to work.
	// calculate GAE(? = 1): very noisy, the policy collapses most of the time.
	// gradValue(i) = 0.5 * (value(i) - reward(i) - alive(i) * gamma * value(i + 1))^2 
	// For some reason, I have never gotten this method to works reliably.
	// The Monte Carlo estimates consistently produce extremely high variance.

	// In my experience, the bootstrapping method using 
	// the Bellman equation to estimate the state value has been 
	// the most reliable approach so far.
	// 
	// So I’m going with bootstrapping.
	// Q = 1/2 * (V(t) - R(t)) ^ 2
	// grad(Q) = L = V(t) - R(t);
	// L = Clamp(V(t) - r(t) - g * V(t + 1), -1.0, 1.0);
	for (ndInt32 i = 0; i < numberOfIterations; ++i)
	{
		m_minibatchRandomShuffleBuffer->CopyBuffer(shuffleBufferInfo, 1, **m_randomShuffleBuffer);

		// calculate the g * V(t+1) 
		inputBuffer->CopyBufferIndirect(nextObservationInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
		m_criticTrainer->MakePrediction();
		outputBuffer->Scale(m_parameters.m_discountRewardFactor);

		// calculate the r(t) + g * V(t+1) 
		m_minibatchCriticStateValueBuffer->CopyBufferIndirect(rewardInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
		m_minibatchCriticStateValueBuffer->Add(*outputBuffer);

		//// for testing
		//m_minibatchCriticStateValueBuffer->Set(15.0f);

		// calculate the V(t) 
		inputBuffer->CopyBufferIndirect(observationInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
		m_criticTrainer->MakePrediction();

		//static ndBrainVector value;
		//outputBuffer->VectorFromDevice(value);

		// calculate the -[V(t) - (r(t) + g * V(t+1))]
		// do not override the output since 
		// the previous layer may use it to calculate the chain rule grad
		m_minibatchCriticStateValueBuffer->Sub(*outputBuffer);

		// calculate the V(t) - (r(t) + g * V(t+1))
		m_minibatchCriticStateValueBuffer->Scale(ndBrainFloat(-1.0f));

		// make sure terminal value gradient is zero
		outputGradientBuffer->CopyBufferIndirect(terminalInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
		outputGradientBuffer->Mul(**m_minibatchCriticStateValueBuffer);

		// clip gradient around +- 1
		ndBrainFloat huberSlope = ndBrainFloat(0.1f);
		outputGradientBuffer->Min(huberSlope);
		outputGradientBuffer->Max(-huberSlope);

		//static ndBrainVector valueGrad;
		//outputGradientBuffer->VectorFromDevice(valueGrad);

		// back propagate the critic loss
		m_criticTrainer->BackPropagate();
		m_criticTrainer->ApplyLearnRate(m_learnRate);
		
		// only the shuffle buffer is increment to the next batch
		shuffleBufferInfo.m_srcOffsetInByte += ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	}
}

#else
// using pure Monte Carlos method
// The Monte Carlo estimates consistently produce extremely high variance.
//#pragma optimize( "", off)
void ndBrainAgentOnPolicyGradient_Trainer::CalculateAdvantage()
{
	ndBrainFloatBuffer* const inputBuffer = m_criticTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const outputBuffer = m_criticTrainer->GetOuputBuffer();

	ndCopyBufferCommandInfo shuffleBufferInfo;
	shuffleBufferInfo.m_srcOffsetInByte = 0;
	shuffleBufferInfo.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	shuffleBufferInfo.m_dstOffsetInByte = 0;
	shuffleBufferInfo.m_dstStrideInByte = shuffleBufferInfo.m_srcStrideInByte;
	shuffleBufferInfo.m_bytesToCopy = shuffleBufferInfo.m_srcStrideInByte;

	ndCopyBufferCommandInfo observationInfo;
	observationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	observationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	observationInfo.m_dstOffsetInByte = 0;
	observationInfo.m_dstStrideInByte = ndInt32(m_criticTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	observationInfo.m_bytesToCopy = observationInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo expectedRewardInfo;
	expectedRewardInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	expectedRewardInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetExpectedRewardOffset() * sizeof(ndReal));
	expectedRewardInfo.m_dstOffsetInByte = 0;
	expectedRewardInfo.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	expectedRewardInfo.m_bytesToCopy = expectedRewardInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo advantageInfo;
	advantageInfo.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	advantageInfo.m_srcOffsetInByte = 0;
	advantageInfo.m_dstOffsetInByte = 0;
	advantageInfo.m_dstStrideInByte = advantageInfo.m_srcStrideInByte;
	advantageInfo.m_bytesToCopy = advantageInfo.m_srcStrideInByte;

	const ndInt32 numberOfIterations = ndInt32(m_numberOfIterations);
	ndAssert(numberOfIterations >= 1);

	// advantage(i) = ExpectedReward(t) - StateValue(t)
	for (ndInt32 i = 0; i < numberOfIterations; ++i)
	{
		advantageInfo.m_dstOffsetInByte = i * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
		shuffleBufferInfo.m_srcOffsetInByte = i * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));

		// Get the state value for thei minibatch
		m_minibatchRandomShuffleBuffer->CopyBuffer(shuffleBufferInfo, 1, **m_randomShuffleBuffer);
		inputBuffer->CopyBufferIndirect(observationInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
		m_criticTrainer->MakePrediction();

		// calculate the advantage A(i) = ExpectedReward(t) - StateValue(t)
		m_minibatchAdvantageBuffer->CopyBufferIndirect(expectedRewardInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
		m_minibatchAdvantageBuffer->Sub(*outputBuffer);

		// clip huge advantages ?
		ndBrainFloat maxAdvantageClipping = ndBrainFloat(5.0f);
		m_minibatchAdvantageBuffer->Min(maxAdvantageClipping);
		m_minibatchAdvantageBuffer->Max(-maxAdvantageClipping);

		// save advantage
		m_advantageBuffer->CopyBuffer(advantageInfo, 1, **m_minibatchAdvantageBuffer);
	}
}

//#pragma optimize( "", off)
void ndBrainAgentOnPolicyGradient_Trainer::OptimizeCritic()
{
	ndCopyBufferCommandInfo shuffleBufferInfo;
	shuffleBufferInfo.m_srcOffsetInByte = 0;
	shuffleBufferInfo.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	shuffleBufferInfo.m_dstOffsetInByte = 0;
	shuffleBufferInfo.m_dstStrideInByte = shuffleBufferInfo.m_srcStrideInByte;
	shuffleBufferInfo.m_bytesToCopy = shuffleBufferInfo.m_srcStrideInByte;

	ndCopyBufferCommandInfo observationInfo;
	observationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	observationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	observationInfo.m_dstOffsetInByte = 0;
	observationInfo.m_dstStrideInByte = ndInt32(m_criticTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	observationInfo.m_bytesToCopy = observationInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo expectedRewardInfo;
	expectedRewardInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	expectedRewardInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetExpectedRewardOffset() * sizeof(ndReal));
	expectedRewardInfo.m_dstOffsetInByte = 0;
	expectedRewardInfo.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	expectedRewardInfo.m_bytesToCopy = expectedRewardInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo previousValueInfo;
	previousValueInfo.m_srcOffsetInByte = 0;
	previousValueInfo.m_srcStrideInByte = ndInt32(sizeof(ndInt32));
	previousValueInfo.m_dstOffsetInByte = 0;
	previousValueInfo.m_dstStrideInByte = ndInt32(sizeof(ndInt32));
	previousValueInfo.m_bytesToCopy = previousValueInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo stateValueInfo;
	stateValueInfo.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	stateValueInfo.m_srcOffsetInByte = 0;
	stateValueInfo.m_dstOffsetInByte = 0;
	stateValueInfo.m_dstStrideInByte = stateValueInfo.m_srcStrideInByte;
	stateValueInfo.m_bytesToCopy = stateValueInfo.m_srcStrideInByte;

	ndBrainFloatBuffer* const inputBuffer = m_criticTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const outputBuffer = m_criticTrainer->GetOuputBuffer();
	ndBrainFloatBuffer* const outputGradientBuffer = m_criticTrainer->GetOuputGradientBuffer();

	const ndInt32 numberOfIterations = ndInt32(m_numberOfIterations);
	ndAssert(numberOfIterations >= 1);

	// caculate all of the base values
	for (ndInt32 i = 0; i < numberOfIterations; ++i)
	{
		// only the shuffle buffer is increment to the next batch
		stateValueInfo.m_dstOffsetInByte = i * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
		shuffleBufferInfo.m_srcOffsetInByte = i * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));

		m_minibatchRandomShuffleBuffer->CopyBuffer(shuffleBufferInfo, 1, **m_randomShuffleBuffer);
		inputBuffer->CopyBufferIndirect(observationInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
		m_criticTrainer->MakePrediction();

		// reuse advantage buffer to store the previus state value
		m_advantageBuffer->CopyBuffer(stateValueInfo, 1, *outputBuffer);
	}
	// reset the use infor
	stateValueInfo.m_dstOffsetInByte = 0;
	shuffleBufferInfo.m_srcOffsetInByte = 0;

	ndBrainFloatBuffer* const epsilon = *m_minibatchLikelihoodRatioBuffer;
	ndBrainFloatBuffer* const blendBuffer = *m_minibatchClippedLikelihoodRatioBuffer;
	epsilon->Set(ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);

	for (ndInt32 j = 0; j < m_parameters.m_divergenceMaxPasses; ++j)
	{
		// Q = 1/2 * (V(t) - ExpectedReward(t)) ^ 2
		// Qclipped = 1/2 * [clip(V(t), V(t-1) - epsilon, V(t-1) + epsilon) - ExpectedReward(t)]^2 
		// Loss = Gradient (max(Q, Qclipped));
		const ndInt32 base = ndInt32(j * m_parameters.m_miniBatchSize * numberOfIterations * sizeof(ndInt32));

		for (ndInt32 i = 0; i < numberOfIterations; ++i)
		{
			shuffleBufferInfo.m_srcOffsetInByte = base + i * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndInt32));

			m_minibatchRandomShuffleBuffer->CopyBuffer(shuffleBufferInfo, 1, **m_randomShuffleBuffer);
			inputBuffer->CopyBufferIndirect(observationInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
			m_criticTrainer->MakePrediction();
			#ifdef ND_DEBUG_CONTINUE_PROXIMA_POLICY
				static ndBrainVector value;
				outputBuffer->VectorFromDevice(value);
			#endif

			// calculate clip(V(t), V(t-1) - epsilon, V(t-1) + epsilon)
			// 
			// -calculate Max (V(t), V(t-1) - epsilon)
			m_minibatchCriticRandomShuffleBuffer->CopyBuffer(shuffleBufferInfo, 1, **m_randomCriticShuffleBuffer);
			outputGradientBuffer->CopyBufferIndirect(previousValueInfo, **m_minibatchCriticRandomShuffleBuffer, **m_advantageBuffer);
			outputGradientBuffer->Sub(*epsilon);
			outputGradientBuffer->Max(*outputBuffer);

			// -calculate Min (((V(t), V(t-1) - epsilon)), V(t-1) + epsilon))
			m_minibatchCriticStateValueBuffer->CopyBufferIndirect(previousValueInfo, **m_minibatchCriticRandomShuffleBuffer, **m_advantageBuffer);
			m_minibatchCriticStateValueBuffer->Add(*epsilon);
			outputGradientBuffer->Min(**m_minibatchCriticStateValueBuffer);

			// calculate Q = V(t) - ExpectedReward(t)
			m_minibatchAdvantageBuffer->CopyBufferIndirect(expectedRewardInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
			outputBuffer->Sub(**m_minibatchAdvantageBuffer);

			// calculate Qclipped = clip(V(t), V(t-1) - epsilon, V(t-1) + epsilon) - ExpectedReward(t)
			outputGradientBuffer->Sub(**m_minibatchAdvantageBuffer);

			// calculate Q^2 > Qclip^2 ? 1.0 : 0.0
			m_minibatchAdvantageBuffer->Set(*outputGradientBuffer);
			m_minibatchAdvantageBuffer->Mul(*outputGradientBuffer);
			blendBuffer->Set(*outputBuffer);
			blendBuffer->Mul(*outputBuffer);
			blendBuffer->GreaterEqual(**m_minibatchAdvantageBuffer);

			// calculate Gradient (max(Q, Qclipped));
			//outputGradientBuffer->Max(*outputBuffer);
			outputGradientBuffer->Blend (*outputBuffer, *blendBuffer);

			#ifdef ND_DEBUG_CONTINUE_PROXIMA_POLICY
				// validate gradient using the automatic differentation
				class AutoGradient : public ndFuntionEvaluator
				{
					public:
					AutoGradient(ndBrainFloat vt0, ndBrainFloat reward)
						:ndFuntionEvaluator()
						,m_vt0(vt0)
						,m_reward(reward)
						,m_epsilon(ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON)
					{
						m_vt0 = m_vt0 + ndBrainDualNumber(ndBrainFloat(0.0f), ndBrainFloat(1.0f));
					}

					// Q = 1/2 * (V(t) - ExpectedReward(t)) ^ 2
					// Qclipped = 1/2 * [clip(V(t), V(t-1) - epsilon, V(t-1) + epsilon) - ExpectedReward(t)]^2 
					// Loss = Gradient (max(Q, Qclipped));
					ndBrainDualNumber Evaluate(ndBrainFloat v)
					{
						const ndBrainDualNumber half(0.5f);
						const ndBrainDualNumber vt(ndBrainDualNumber(v) + ndBrainDualNumber (0.0f, 1.0f));

						const ndBrainDualNumber h(vt - m_reward);
						const ndBrainDualNumber Q(half * h * h);

						const ndBrainDualNumber vClip(vt.Max(m_vt0 - m_epsilon).Min(m_vt0 + m_epsilon));
						const ndBrainDualNumber hClip(vClip - m_reward);
						const ndBrainDualNumber Qclip(half * hClip * hClip);

						const ndBrainDualNumber loss(Q.Max(Qclip));

						return loss;
					}

					ndBrainDualNumber m_vt0;
					ndBrainDualNumber m_reward;
					ndBrainDualNumber m_epsilon;
				};

				ndAssert(0);
				m_minibatchCriticStateValueBuffer->CopyBuffer(previousValueInfo____, 1, **m_advantageBuffer);
				m_minibatchAdvantageBuffer->CopyBufferIndirect(expectedRewardInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);

				static ndBrainVector gradientBatch;
				static ndBrainVector expectedReward;
				static ndBrainVector previousStateValue;
				outputGradientBuffer->VectorFromDevice(gradientBatch);
				m_minibatchAdvantageBuffer->VectorFromDevice(expectedReward);
				m_minibatchCriticStateValueBuffer->VectorFromDevice(previousStateValue);

				for (ndInt32 k = 0; k < gradientBatch.GetCount(); ++k)
				{
					AutoGradient autoGrad(previousStateValue[k], expectedReward[k]);
					ndBrainFloat gradiant0(autoGrad.Evaluate(value[k]).m_gradient);
					ndBrainFloat gradiant1(gradientBatch[k]);
					ndAssert(ndAreEqual(gradiant0, gradiant1, ndBrainFloat(1.0e-4f)));
					gradiant0 *= 1;
					gradiant1 *= 1;
				}
			#endif

			// maybe apply a huber loss here
			ndBrainFloat huberSlope = ndBrainFloat(1.0f);
			outputGradientBuffer->Min(huberSlope);
			outputGradientBuffer->Max(-huberSlope);

			// back propagate the critic loss
			m_criticTrainer->BackPropagate();
			m_criticTrainer->ApplyLearnRate(m_learnRate);
		}
	}
}
#endif

ndBrainFloat ndBrainAgentOnPolicyGradient_Trainer::CalculateKLdivergence()
{
	// https://en.wikipedia.org/wiki/Kullback%E2%80%93Leibler_divergence
	// since I am using a diagonal sigma, I do not have to use Cholesky 

	ndBrainFloatBuffer* const policyMinibatchDivergence = m_policyTrainer->GetOuputBuffer();
	ndBrainFloatBuffer* const inputBuffer = m_policyTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const policyMinibatchBaseDivergence = m_policyTrainer->GetOuputGradientBuffer();
	
	ndSharedPtr<ndBrainFloatBuffer> minbatchDivergence(m_minibatchCriticStateValueBuffer);
	ndSharedPtr<ndBrainFloatBuffer> minbatchDivergenceAcc(m_minibatchAdvantageBuffer);

	ndCopyBufferCommandInfo shuffleBufferInfo;
	shuffleBufferInfo.m_srcOffsetInByte = 0;
	shuffleBufferInfo.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	shuffleBufferInfo.m_dstOffsetInByte = 0;
	shuffleBufferInfo.m_dstStrideInByte = shuffleBufferInfo.m_srcStrideInByte;
	shuffleBufferInfo.m_bytesToCopy = shuffleBufferInfo.m_srcStrideInByte;

	ndCopyBufferCommandInfo policyObservationInfo;
	policyObservationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	policyObservationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	policyObservationInfo.m_dstOffsetInByte = 0;
	policyObservationInfo.m_dstStrideInByte = ndInt32(m_parameters.m_numberOfObservations * sizeof(ndReal));
	policyObservationInfo.m_bytesToCopy = ndInt32(m_parameters.m_numberOfObservations * sizeof(ndReal));
	
	//ndCopyBufferCommandInfo policyActionInfo;
	//policyActionInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	//policyActionInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetActionOffset() * sizeof(ndReal));
	//policyActionInfo.m_dstOffsetInByte = 0;
	//policyActionInfo.m_dstStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	//policyActionInfo.m_bytesToCopy = policyActionInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo policyActionInfo;
	policyActionInfo.m_srcOffsetInByte = 0;
	policyActionInfo.m_srcStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	policyActionInfo.m_dstOffsetInByte = 0;
	policyActionInfo.m_dstStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	policyActionInfo.m_bytesToCopy = policyActionInfo.m_dstStrideInByte;
	
	minbatchDivergenceAcc->Set(ndBrainFloat(0.0f));

	const ndInt32 numberOfIterations = ndInt32 (m_numberOfIterations);
	ndAssert(numberOfIterations >= 1);
	for (ndInt32 i = 0; i < numberOfIterations; ++i)
	{
		shuffleBufferInfo.m_srcOffsetInByte = i * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
		policyActionInfo.m_srcOffsetInByte = 2 * i * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
		m_minibatchRandomShuffleBuffer->CopyBuffer(shuffleBufferInfo, 1, **m_randomShuffleBuffer);

		inputBuffer->CopyBufferIndirect(policyObservationInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
		m_policyTrainer->MakePrediction();

		policyMinibatchBaseDivergence->CopyBuffer(policyActionInfo, m_parameters.m_miniBatchSize, **m_policyActionBuffer);
		minbatchDivergence->CalculatePartialKlDivergence(*policyMinibatchBaseDivergence, *policyMinibatchDivergence);
		minbatchDivergenceAcc->Add(**minbatchDivergence);
	}
	
	minbatchDivergenceAcc->ReductionSum();
	ndBrainFloat sumDivergence = minbatchDivergenceAcc->GetElement(0);
	ndBrainFloat den = ndBrainFloat(numberOfIterations * m_parameters.m_miniBatchSize);
	return sumDivergence / den;
}

void ndBrainAgentOnPolicyGradient_Trainer::OptimizePolicy()
{
	ndBrainFloatBuffer* const inputBuffer = m_policyTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const outputBuffer = m_policyTrainer->GetOuputBuffer();
	ndBrainFloatBuffer* const outputGradientBuffer = m_policyTrainer->GetOuputGradientBuffer();
	ndBrainFloatBuffer* const weightAndBiasGradientBuffer = m_policyTrainer->GetWeightAndBiasGradientBuffer();

	ndCopyBufferCommandInfo shuffleBufferInfo;
	shuffleBufferInfo.m_srcOffsetInByte = 0;
	shuffleBufferInfo.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	shuffleBufferInfo.m_dstOffsetInByte = 0;
	shuffleBufferInfo.m_dstStrideInByte = shuffleBufferInfo.m_srcStrideInByte;
	shuffleBufferInfo.m_bytesToCopy = shuffleBufferInfo.m_srcStrideInByte;

	ndCopyBufferCommandInfo policyObservationInfo;
	policyObservationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	policyObservationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	policyObservationInfo.m_dstOffsetInByte = 0;
	policyObservationInfo.m_dstStrideInByte = ndInt32(m_parameters.m_numberOfObservations * sizeof(ndReal));
	policyObservationInfo.m_bytesToCopy = ndInt32(m_parameters.m_numberOfObservations * sizeof(ndReal));

	ndCopyBufferCommandInfo policyActionInfo;
	policyActionInfo.m_srcOffsetInByte = 0;
	policyActionInfo.m_srcStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	policyActionInfo.m_dstOffsetInByte = 0;
	policyActionInfo.m_dstStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	policyActionInfo.m_bytesToCopy = policyActionInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo meanSampledActions;
	meanSampledActions.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	meanSampledActions.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetActionOffset() * sizeof(ndReal));
	meanSampledActions.m_dstOffsetInByte = 0;
	meanSampledActions.m_dstStrideInByte = ndInt32(m_parameters.m_numberOfActions * sizeof(ndReal));
	meanSampledActions.m_bytesToCopy = meanSampledActions.m_dstStrideInByte;

	ndCopyBufferCommandInfo copyMeanActions;
	copyMeanActions.m_srcOffsetInByte = 0;
	copyMeanActions.m_srcStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	copyMeanActions.m_dstOffsetInByte = 0;
	copyMeanActions.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	copyMeanActions.m_bytesToCopy = ndInt32(sizeof(ndReal));

	ndCopyBufferCommandInfo copySigmaActions;
	copySigmaActions.m_srcOffsetInByte = ndInt32(sizeof(ndReal));
	copySigmaActions.m_srcStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	copySigmaActions.m_dstOffsetInByte = 0;
	copySigmaActions.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	copySigmaActions.m_bytesToCopy = ndInt32(sizeof(ndReal));

	ndCopyBufferCommandInfo advantageReadWriteInfo;
	advantageReadWriteInfo.m_srcOffsetInByte = 0;
	advantageReadWriteInfo.m_srcStrideInByte = ndInt32(sizeof(ndReal));
	advantageReadWriteInfo.m_dstOffsetInByte = 0;
	advantageReadWriteInfo.m_dstStrideInByte = ndInt32(sizeof(ndReal));
	advantageReadWriteInfo.m_bytesToCopy = advantageReadWriteInfo.m_dstStrideInByte;

	const ndInt32 numberOfIterations = ndInt32(m_numberOfIterations);
	ndAssert(numberOfIterations >= 1);

	// calculate all of the base values
	for (ndInt32 i = 0; i < numberOfIterations; ++i)
	{
		shuffleBufferInfo.m_srcOffsetInByte = i * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
		m_minibatchRandomShuffleBuffer->CopyBuffer(shuffleBufferInfo, 1, **m_randomShuffleBuffer);

		policyActionInfo.m_dstOffsetInByte = 2 * i * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
		advantageReadWriteInfo.m_dstOffsetInByte = i * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));

		// get the policy output
		inputBuffer->CopyBufferIndirect(policyObservationInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
		m_policyTrainer->MakePrediction();

		// save base mean and sigma for later use calculation KL divergence.
		m_policyActionBuffer->CopyBuffer(policyActionInfo, m_parameters.m_miniBatchSize, *outputBuffer);
	
		// calculate the z mean and sigma for the entropy
		m_meanBuffer->CopyBuffer(copyMeanActions, m_parameters.m_miniBatchSize, *outputBuffer);
		m_sigmaBuffer->CopyBuffer(copySigmaActions, m_parameters.m_miniBatchSize, *outputBuffer);
	
		m_zMeanBuffer->CopyBufferIndirect(meanSampledActions, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
		m_zMeanBuffer->Sub(**m_meanBuffer);

		// calculate the likelihood
		m_minibatchInvLikelihoodBuffer->CalculateLikelihood(**m_zMeanBuffer, **m_sigmaBuffer);
		m_minibatchInvLikelihoodBuffer->Max(ndBrainFloat(1.0e-5f));
		m_minibatchInvLikelihoodBuffer->Reciprocal(**m_minibatchInvLikelihoodBuffer);

		// save the inverse likelihood
		m_invLikelihoodBuffer->CopyBuffer(advantageReadWriteInfo, m_parameters.m_miniBatchSize, **m_minibatchInvLikelihoodBuffer);
	}
	// reset infos
	policyActionInfo.m_dstOffsetInByte = 0;
	shuffleBufferInfo.m_srcOffsetInByte = 0;
	advantageReadWriteInfo.m_dstOffsetInByte = 0;

	ndBrainFloat divergence = ndBrainFloat(0.0f);
	const ndBrainFloat stopDivergence = m_parameters.m_divergenceStopThreshold;
	ndInt32 passes = 0;
	for (ndInt32 j = 0; (j < m_parameters.m_divergenceMaxPasses) && (divergence < stopDivergence); ++j)
	{
		passes++;
		m_policyGradientAccumulator->Set(ndBrainFloat(0.0f));
		for (ndInt32 i = 0; i < numberOfIterations; ++i)
		{
			shuffleBufferInfo.m_srcOffsetInByte = i * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
			m_minibatchRandomShuffleBuffer->CopyBuffer(shuffleBufferInfo, 1, **m_randomShuffleBuffer);
			advantageReadWriteInfo.m_srcOffsetInByte = i * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));

			// get the policy output
			inputBuffer->CopyBufferIndirect(policyObservationInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
			m_policyTrainer->MakePrediction();
			
			m_meanBuffer->CopyBuffer(copyMeanActions, m_parameters.m_miniBatchSize, *outputBuffer);
			m_sigmaBuffer->CopyBuffer(copySigmaActions, m_parameters.m_miniBatchSize, *outputBuffer);

			m_zMeanBuffer->CopyBufferIndirect(meanSampledActions, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
			m_zMeanBuffer->Sub(**m_meanBuffer);
			
			// get this mini batch advantage
			m_minibatchAdvantageBuffer->CopyBuffer(advantageReadWriteInfo, m_parameters.m_miniBatchSize, **m_advantageBuffer);
			
			//calculate the clip surrogate factor
			m_minibatchLikelihoodRatioBuffer->CalculateLikelihood(**m_zMeanBuffer, **m_sigmaBuffer);
			m_minibatchInvLikelihoodBuffer->CopyBuffer(advantageReadWriteInfo, m_parameters.m_miniBatchSize, **m_invLikelihoodBuffer);
			m_minibatchLikelihoodRatioBuffer->Mul(**m_minibatchInvLikelihoodBuffer);
			
			m_minibatchClippedLikelihoodRatioBuffer->Set(**m_minibatchLikelihoodRatioBuffer);
			m_minibatchClippedLikelihoodRatioBuffer->Min(ndBrainFloat(1.0f) + ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);
			m_minibatchClippedLikelihoodRatioBuffer->Max(ndBrainFloat(1.0f) - ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);
			
			m_minibatchLikelihoodRatioBuffer->Mul(**m_minibatchAdvantageBuffer);
			m_minibatchAdvantageBuffer->Mul(**m_minibatchClippedLikelihoodRatioBuffer);
			
			m_minibatchAdvantageBuffer->Min(**m_minibatchLikelihoodRatioBuffer);
			
			// calculate gradient
			outputGradientBuffer->CalculateEntropyRegularizationGradient(**m_zMeanBuffer, **m_sigmaBuffer, ndBrainFloat(1.0f), m_parameters.m_numberOfActions);
			
			// get a advantage mini batch and make gradient ascend 
			m_minibatchBrocastAdvantageBuffer->BroadcastScaler(**m_minibatchAdvantageBuffer);
			outputGradientBuffer->Mul(**m_minibatchBrocastAdvantageBuffer);
			
			if (m_parameters.m_entropyTemperature > ndFloat32(0.0f))
			{
				ndAssert(0);
			//	// calculate entropy regularization
			//	// 1-get a random sample using the re parametrization trick
			//	//m_minibatchGaussianDistribution->CopyBuffer(minibatchReparametization, m_parameters.m_miniBatchSize, **m_uniformDistributionBuffer);
			//	//m_minibatchGaussianDistribution->StandardNormalDistribution();
			//	//m_minibatchGaussianDistribution->Mul(**m_sigmaBuffer);
			//	//m_minibatchGaussianDistribution->Add(**m_meanBuffer);
			//	//m_minibatchGaussianDistribution->Min(ndBrainFloat(1.0f));
			//	//m_minibatchGaussianDistribution->Max(ndBrainFloat(-1.0f));
			//	//outputBuffer->CalculateEntropyRegularizationGradient(**m_minibatchGaussianDistribution, **m_sigmaBuffer, m_parameters.m_entropyTemperature, m_parameters.m_numberOfActions);
			//	//
			//	//outputGradientBuffer->Sub(*outputBuffer);
			}
			
			outputGradientBuffer->Scale(ndReal(-1.0f));
			
			m_policyTrainer->BackPropagate();
			m_policyGradientAccumulator->Add(*weightAndBiasGradientBuffer);
		}

		// average all the mini batches
		m_policyGradientAccumulator->Scale(ndBrainFloat(1.0f) / ndBrainFloat(numberOfIterations));
		weightAndBiasGradientBuffer->Set(**m_policyGradientAccumulator);

		// update network weight and bias.
		m_policyTrainer->ApplyLearnRate(m_learnRate * ND_POLICY_DOWN_SAMPLE_LEARN_RATE);

		// calculate the KL divergence
		divergence = CalculateKLdivergence();
	}
	ndExpandTraceMessage("surrogate loss passes %d\n", passes);
}

void ndBrainAgentOnPolicyGradient_Trainer::OptimizeStep()
{
	for (ndList<ndSharedPtr<ndBrainAgentOnPolicyGradient_Agent>>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
	{
		ndBrainAgentOnPolicyGradient_Agent* const agent = *node->GetInfo();
		ndAssert(agent->m_trajectory.GetCount());

		bool isTeminal = agent->m_isDead || (agent->m_trajectory.GetCount() >= ndInt32(m_parameters.m_maxTrajectorySteps + m_horizonSteps));
		if (isTeminal && (ndInt32(m_trajectiesCount) < m_parameters.m_batchTrajectoryCount))
		{
			SaveTrajectory(agent);
			agent->m_trajectory.SetCount(0);
			agent->ResetModel();
			agent->m_isDead = false;
		}
		m_frameCount++;
	}

	if (ndInt32 (m_trajectiesCount) >= m_parameters.m_batchTrajectoryCount)
	{
		Optimize();

		for (ndList<ndSharedPtr<ndBrainAgentOnPolicyGradient_Agent>>::ndNode* node = m_agents.GetFirst(); node; node = node->GetNext())
		{
			ndBrainAgentOnPolicyGradient_Agent* const agent = *node->GetInfo();
			agent->m_trajectory.SetCount(0);
			agent->ResetModel();
			agent->m_isDead = false;
			agent->m_trajectoryIndex = 0;
		}
		m_eposideCount++;
		m_trajectiesCount = 0;
		m_trajectoryAccumulator.SetCount(0);

		m_context->SyncBufferCommandQueue();
		m_policyTrainer->GetWeightAndBiasBuffer()->VectorFromDevice(m_lastPolicy);
		m_policyTrainer->UpdateParameters(m_lastPolicy);

		ndAssert(ndMemory::CheckMemoryHeap());
	}
}

void ndBrainAgentOnPolicyGradient_Trainer::Optimize()
{
m_parameters.m_entropyTemperature = 0.0f;

	UpdateScore();
	TrajectoryToGpuBuffers();

	CalculateAdvantage();
	OptimizePolicy();
	OptimizeCritic();
}