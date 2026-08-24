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
#include "ndBrainLayerActivationTanh.h"
#include "ndBrainLayerActivationLinear.h"
#include "ndBrainLossLeastSquaredError.h"
#include "ndBrainLayerActivationLeakyRelu.h"
#include "ndBrainAgentOnPolicyGradient_Trainer.h"

#define ND_POLICY_MAX_KL_DIVERGENCE_PASSES			8
#define ND_POLICY_DOWN_SAMPLE_LEARN_RATE			ndBrainFloat(1.0f)
#define ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON		ndBrainFloat(0.2f)
#define ND_POLICY_KL_DIVERGENCE_STOP_THRESHHOLD		ndBrainFloat(1.0e-3f)

//entropy regularization improvement from 
//https://arxiv.org/pdf/1912.01557

ndBrainAgentOnPolicyGradient_Trainer::HyperParameters::HyperParameters()
{
	m_batchTrajectoryCount = 1000;
	m_divergenceMaxPasses = ND_POLICY_MAX_KL_DIVERGENCE_PASSES;
	m_divergenceStopThreshold = ND_POLICY_KL_DIVERGENCE_STOP_THRESHHOLD;
}

ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::ndTrajectory()
	:m_alive()
	,m_reward()
	,m_expectedReward()
	,m_actions()
	,m_observations()
	,m_nextObservations()
	,m_actionsSize(0)
	,m_obsevationsSize(0)
{
}

ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::ndTrajectory(ndInt32 actionsSize, ndInt32 obsevationsSize)
	:m_alive()
	,m_reward()
	,m_expectedReward()
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
	m_alive[entry] = ndBrainFloat(0.0f);
	m_reward[entry] = ndBrainFloat(0.0f);
	m_expectedReward[entry] = ndBrainFloat(0.0f);
	ndMemSet(&m_actions[entry * m_actionsSize], ndBrainFloat(0.0f), m_actionsSize);
	ndMemSet(&m_observations[entry * m_obsevationsSize], ndBrainFloat(0.0f), m_obsevationsSize);
	ndMemSet(&m_nextObservations[entry * m_obsevationsSize], ndBrainFloat(0.0f), m_obsevationsSize);
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::CopyFrom(ndInt32 entry, ndTrajectory& src, ndInt32 srcEntry)
{
	m_alive[entry] = src.m_alive[srcEntry];
	m_reward[entry] = src.m_reward[srcEntry];
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
	m_alive.SetCount(count);
	m_reward.SetCount(count);
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

bool ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetAliveState(ndInt32 entry) const
{
	return (m_alive[entry] == ndBrainFloat(1.0f)) ? true : false;
}

void ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::SetAliveState(ndInt32 entry, bool alive)
{
	m_alive[entry] = alive ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
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

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetAliveOffset() const
{
	return GetExpectedRewardOffset() + 1;
}

ndInt32 ndBrainAgentOnPolicyGradient_Agent::ndTrajectory::GetActionOffset() const
{
	return GetAliveOffset() + 1;
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
	output[GetAliveOffset()] = m_alive[index];
	output[GetRewardOffset()] = m_reward[index];
	output[GetExpectedRewardOffset()] = m_expectedReward[index];
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
	ndAssert(0);
	return 0;
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

//#pragma optimize( "", off)
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
	m_trajectory.SetAliveState(entryIndex, !isdead);
	m_isDead = m_isDead || isdead;
}

ndBrainAgentOnPolicyGradient_Trainer::ndBrainAgentOnPolicyGradient_Trainer(const HyperParameters& parameters)
	:ndClassAlloc()
	,m_name()
	,m_parameters(parameters)
	,m_context()
	,m_valueTrainer(nullptr)
	,m_policyTrainer(nullptr)
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
	,m_minibatchClippedMinimunZeroGradient(nullptr)
	,m_minibatchClippedMaximumZeroGradient(nullptr)
	,m_minibatchClippedLikelihoodRatioBuffer(nullptr)
	,m_valueShuffleBuffer(nullptr)
	,m_minibatchValueShuffleBuffer(nullptr)
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
	m_minibatchValueShuffleBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minibatchClippedMinimunZeroGradient = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minibatchClippedMaximumZeroGradient = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));
	m_minibatchClippedLikelihoodRatioBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize));

	m_meanBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_sigmaBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_minibatchGaussianDistribution = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_zMeanBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_invSigmaBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_invSigma2Buffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_meanGradiendBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));
	m_sigmaGradiendBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions));

	m_trainingBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_trajectoryAccumulator.GetStride() * m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps));
	m_advantageBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps));
	m_valueShuffleBuffer = ndSharedPtr<ndBrainIntegerBuffer>(new ndBrainIntegerBuffer(*m_context, m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps));
	m_invLikelihoodBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps));
	m_policyActionBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, 2 * m_parameters.m_numberOfActions * m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps));

	//m_uniformDistributionBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, m_parameters.m_batchTrajectoryCount * m_parameters.m_maxTrajectorySteps * m_parameters.m_numberOfActions));
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

	layers.PushBack(new ndBrainLayerLinear(m_parameters.m_numberOfObservations, m_parameters.m_hiddenLayersNumberOfNeurons));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));

	for (ndInt32 i = 0; i < m_parameters.m_numberOfHiddenLayers; ++i)
	{
		ndAssert(layers[layers.GetCount() - 1]->GetOutputSize() == m_parameters.m_hiddenLayersNumberOfNeurons);
		layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
		layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));
	}
	layers.PushBack(new ndBrainLayerLinear(layers[layers.GetCount() - 1]->GetOutputSize(), m_parameters.m_numberOfActions * 2));
	layers.PushBack(new ndBrainLayerActivationTanh(layers[layers.GetCount() - 1]->GetOutputSize()));

	ndBrainFixSizeVector<256> bias;
	ndBrainFixSizeVector<256> slope;
	bias.SetCount(m_parameters.m_numberOfActions * 2);
	slope.SetCount(m_parameters.m_numberOfActions * 2);

	ndBrainFloat minSigma = ndBrainFloat(ndSqrt(m_parameters.m_minSigmaSquared));
	ndBrainFloat maxSigma = ndBrainFloat(ndSqrt(m_parameters.m_maxSigmaSquared));
	ndBrainFloat s = ndBrainFloat(0.5f) * (maxSigma - minSigma);
	ndBrainFloat b = s + minSigma;
	for (ndInt32 i = 0; i < m_parameters.m_numberOfActions; ++i)
	{
		bias[i] = ndBrainFloat(0.0f);
		slope[i] = ndBrainFloat(1.0f);
		bias[i + m_parameters.m_numberOfActions] = b;
		slope[i + m_parameters.m_numberOfActions] = s;
	}
	layers.PushBack(new ndBrainLayerActivationLinear(slope, bias));

	ndSharedPtr<ndBrain> policy (new ndBrain);
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		policy->AddLayer(layers[i]);
	}
	policy->InitWeights();

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
	m_valueTrainer = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptor, optimizer));
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
			// it is posible to be more than one deat state, 
			// since the enveroiment takes more than one substep per update
			ndInt32 start = ndMax(0, ndInt32(trajectory.GetCount() - 64));
			for (ndInt32 i = start; i < trajectory.GetCount(); ++i)
			{
				if (!trajectory.GetAliveState(i))
				{
					// clip that trajectory at the first dead state
					trajectory.SetCount(i + 1);
					break;
				}
			}
		}

		// using the Bellman equation to calculate trajectory expected rewards score.
		const ndBrainFloat gamma = m_parameters.m_discountRewardFactor;
		ndBrainFloat expectedReward = trajectory.GetReward(trajectory.GetCount() - 1);
		trajectory.SetExpectedReward(trajectory.GetCount() - 1, expectedReward);
		for (ndInt32 i = trajectory.GetCount() - 2; i >= 0; --i)
		{
			ndBrainFloat r = trajectory.GetReward(i);
			expectedReward = r + gamma * expectedReward;
			trajectory.SetExpectedReward(i, expectedReward);
		}

		// get the next obesevation from the array of obsevations
		ndMemCpy(trajectory.GetNextObservations(trajectory.GetCount() - 1), trajectory.GetObservations(trajectory.GetCount() - 1), m_parameters.m_numberOfObservations);
		for (ndInt32 i = trajectory.GetCount() - 2; i >= 0; --i)
		{
			ndMemCpy(trajectory.GetNextObservations(i), trajectory.GetObservations(i + 1), m_parameters.m_numberOfObservations);
		}

		// clip trajetory to that max horizon
		trajectory.SetCount(ndMin (trajectory.GetCount(), m_parameters.m_maxTrajectorySteps));

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
	// make sure the number of transitions is a integer multiple of the mini batch size
	m_trajectoryAccumulator.SetCount(m_trajectoryAccumulator.GetCount() - m_trajectoryAccumulator.GetCount() % m_parameters.m_miniBatchSize);
	ndAssert(m_trajectoryAccumulator.GetCount() >= m_parameters.m_miniBatchSize);

	// flatten the entire set of trajectores
	m_scratchBuffer.SetCount(0);
	const ndInt32 stride = m_trajectoryAccumulator.GetStride();
	ndAssert(ndInt32(m_trainingBuffer->GetCount()) >= ndInt32(m_trajectoryAccumulator.GetCount() * stride));
	for (ndInt32 i = 0; i < ndInt32(m_trajectoryAccumulator.GetCount()); ++i)
	{
		ndInt64 base = m_scratchBuffer.GetCount();
		m_scratchBuffer.SetCount(base + stride);
		ndBrainMemVector dst(&m_scratchBuffer[base], stride);
		m_trajectoryAccumulator.GetFlatArray(i, dst);
	}
	// check possible memory overflow 
	ndAssert(ndInt64(m_trainingBuffer->GetCount()) >= m_scratchBuffer.GetCount());
	m_trainingBuffer->VectorToDevice(m_scratchBuffer);
	
	// randominze the transitions for the value funtion training
	m_shuffleBuffer.SetCount(0);
	for (ndInt32 i = 0; i < ndInt32(m_trajectoryAccumulator.GetCount()); ++i)
	{
		m_shuffleBuffer.PushBack(i);
	}
	m_shuffleBuffer.RandomShuffle(m_shuffleBuffer.GetCount());
	ndAssert(m_valueShuffleBuffer->SizeInBytes() >= m_shuffleBuffer.GetCount() * sizeof(ndInt32));
	m_valueShuffleBuffer->MemoryToDevice(0, size_t(m_shuffleBuffer.GetCount()) * sizeof(ndInt32), &m_shuffleBuffer[0]);
	
	////m_scratchBuffer.SetCount(m_numberOfIterations * m_parameters.m_miniBatchSize);
	////for (ndInt32 i = ndInt32(m_numberOfIterations * m_parameters.m_miniBatchSize) - 1; i >= 0; --i)
	////{
	////	m_scratchBuffer[i] = ndBrainFloat(m_uniformDistribution());
	////}
	////m_uniformDistributionBuffer->VectorToDevice(m_scratchBuffer);
}

// using pure bootstrapping method, but the critic always blows up
void ndBrainAgentOnPolicyGradient_Trainer::CalculateAdvantage()
{
	ndBrainFloatBuffer* const inputBuffer = m_valueTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const outputBuffer = m_valueTrainer->GetOuputBuffer();
	
	ndCopyBufferCommandInfo advantageInfo;
	advantageInfo.m_srcOffsetInByte = 0;
	advantageInfo.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	advantageInfo.m_dstOffsetInByte = 0;
	advantageInfo.m_dstStrideInByte = advantageInfo.m_srcStrideInByte;
	advantageInfo.m_bytesToCopy = advantageInfo.m_srcStrideInByte;
	
	ndCopyBufferCommandInfo observationInfo;
	observationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	observationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	observationInfo.m_dstOffsetInByte = 0;
	observationInfo.m_dstStrideInByte = ndInt32(m_valueTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	observationInfo.m_bytesToCopy = observationInfo.m_dstStrideInByte;
	
	ndCopyBufferCommandInfo nextObservationInfo;
	nextObservationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	nextObservationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetNextObsevationOffset() * sizeof(ndReal));
	nextObservationInfo.m_dstOffsetInByte = 0;
	nextObservationInfo.m_dstStrideInByte = ndInt32(m_valueTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	nextObservationInfo.m_bytesToCopy = nextObservationInfo.m_dstStrideInByte;
	
	ndCopyBufferCommandInfo isAliveBufferInfo;
	isAliveBufferInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetAliveOffset() * sizeof(ndReal));
	isAliveBufferInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	isAliveBufferInfo.m_dstOffsetInByte = 0;
	isAliveBufferInfo.m_dstStrideInByte = ndInt32(sizeof(ndBrainFloat));
	isAliveBufferInfo.m_bytesToCopy = ndInt32(sizeof(ndBrainFloat));
	
	ndCopyBufferCommandInfo rewardBufferInfo;
	rewardBufferInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetRewardOffset() * sizeof(ndReal));
	rewardBufferInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	rewardBufferInfo.m_dstOffsetInByte = 0;
	rewardBufferInfo.m_dstStrideInByte = ndInt32(sizeof(ndBrainFloat));
	rewardBufferInfo.m_bytesToCopy = ndInt32(sizeof(ndBrainFloat));
	
	const ndInt32 numberOfIterations = ndInt32(m_trajectoryAccumulator.GetCount() / (m_parameters.m_miniBatchSize));
	ndAssert(numberOfIterations >= 1);
	const ndInt32 advantageStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	const ndInt32 transitionStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * m_trajectoryAccumulator.GetStride() * sizeof(ndReal));

	// make some alias buffer names
	ndWeakPtr<ndBrainFloatBuffer> advantage(*m_sigmaBuffer);
	ndWeakPtr<ndBrainFloatBuffer> isAlive(*m_invSigmaBuffer);
	
	// calculate GAE(l, 1) // very noisy, the policy collapse most of the time.
	// calculate GAE(l, 0) // too smooth, and doesn't seem to work either
	// 
	// just using bellman equation to calculate state expected reward.
	// advantage(i) = reward(i) + gamma * Value(i + 1) - value(i)
	// using the teminal condition as preciate
	// advantage(i) = reward(i) + alive * gamma * Value(i + 1) - value(i)
	for (ndInt32 i = 0; i < numberOfIterations; ++i)
	{
		// calculate m_minibatchAdvantageBuffer = gamma * Value(i + 1)
		inputBuffer->CopyBuffer(nextObservationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_valueTrainer->MakePrediction();
		outputBuffer->Scale(m_parameters.m_discountRewardFactor);

		// calculate outputBuffer = alive * gamma * Value(i + 1)
		isAlive->CopyBuffer(isAliveBufferInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		outputBuffer->Mul(**isAlive);

		// calculate advantage = reward + alive * gamma * Value(i + 1);
		advantage->CopyBuffer(rewardBufferInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		advantage->Add(*outputBuffer);

		// calculate advantage = reward + alive * gamma * Value(i + 1) - Value(i);
		inputBuffer->CopyBuffer(observationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_valueTrainer->MakePrediction();
		advantage->Sub(*outputBuffer);

		// save advantage
		m_advantageBuffer->CopyBuffer(advantageInfo, 1, **advantage);
	
		advantageInfo.m_dstOffsetInByte += advantageStrideInBytes;
		observationInfo.m_srcOffsetInByte += transitionStrideInBytes;
		rewardBufferInfo.m_srcOffsetInByte += transitionStrideInBytes;
		nextObservationInfo.m_srcOffsetInByte += transitionStrideInBytes;
		isAliveBufferInfo.m_srcOffsetInByte += transitionStrideInBytes;
	}
}

#if 1
// Using the Monte Carlo method:
// expectedReward is the target return calculated from the trajectory.
// The loss is the squared error between the estimated state value
// and the Monte Carlo return:
// objective = 1/2 * (expectedReward - stateValue(t))^2;
// Therefore, the gradient is based on:
// loss = expectedReward - stateValue(i);
void ndBrainAgentOnPolicyGradient_Trainer::OptimizeValue()
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
	observationInfo.m_dstStrideInByte = ndInt32(m_valueTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	observationInfo.m_bytesToCopy = observationInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo expectedRewardBufferInfo;
	expectedRewardBufferInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetExpectedRewardOffset() * sizeof(ndReal));
	expectedRewardBufferInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	expectedRewardBufferInfo.m_dstOffsetInByte = 0;
	expectedRewardBufferInfo.m_dstStrideInByte = ndInt32(sizeof(ndBrainFloat));
	expectedRewardBufferInfo.m_bytesToCopy = ndInt32(sizeof(ndBrainFloat));

	//ndCopyBufferCommandInfo isAliveBufferInfo;
	//isAliveBufferInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetAliveOffset() * sizeof(ndReal));
	//isAliveBufferInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	//isAliveBufferInfo.m_dstOffsetInByte = 0;
	//isAliveBufferInfo.m_dstStrideInByte = ndInt32(sizeof(ndBrainFloat));
	//isAliveBufferInfo.m_bytesToCopy = ndInt32(sizeof(ndBrainFloat));
	
	//ndCopyBufferCommandInfo rewardBufferInfo;
	//rewardBufferInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetRewardOffset() * sizeof(ndReal));
	//rewardBufferInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	//rewardBufferInfo.m_dstOffsetInByte = 0;
	//rewardBufferInfo.m_dstStrideInByte = ndInt32(sizeof(ndBrainFloat));
	//rewardBufferInfo.m_bytesToCopy = ndInt32(sizeof(ndBrainFloat));

	const ndInt32 numberOfIterations = ndInt32(m_trajectoryAccumulator.GetCount() / m_parameters.m_miniBatchSize);
	ndAssert(numberOfIterations >= 1);

	ndBrainFloatBuffer* const inputBuffer = m_valueTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const outputBuffer = m_valueTrainer->GetOuputBuffer();
	ndBrainFloatBuffer* const outputGradientBuffer = m_valueTrainer->GetOuputGradientBuffer();

	// alias pointers
	ndWeakPtr<ndBrainFloatBuffer> stateValue(outputBuffer);
	ndWeakPtr<ndBrainFloatBuffer> grad(outputGradientBuffer);

	const ndBrainFloat huberSlope = ndBrainFloat(1.0f);
	for (ndInt32 i = 0; i < numberOfIterations; ++i)
	{
		shuffleBufferInfo.m_srcOffsetInByte = i * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndInt32));
		m_minibatchValueShuffleBuffer->CopyBuffer(shuffleBufferInfo, 1, **m_valueShuffleBuffer);
		
		// calculate stateValue(i)
		inputBuffer->CopyBufferIndirect(observationInfo, **m_minibatchValueShuffleBuffer, **m_trainingBuffer);
		m_valueTrainer->MakePrediction();

		// calculate grad = expectedReward - stateValue(i);
		grad->CopyBufferIndirect(expectedRewardBufferInfo, **m_minibatchValueShuffleBuffer, **m_trainingBuffer);
		grad->Sub(**stateValue);

		grad->Scale(-1.0f);

		// maybe apply a Huber loss here
		outputGradientBuffer->Min(huberSlope);
		outputGradientBuffer->Max(-huberSlope);
		
		// back propagate the critic loss
		m_valueTrainer->BackPropagate();
		m_valueTrainer->AccumulateWeightAndBiasGradients();
		m_valueTrainer->ApplyLearnRate(m_learnRate);
	}
}

#elif 0
// using the implementation detail clip loss
// Q = 1/2 * (V(t) - ExpectedReward(t)) ^ 2
// Qclipped = 1/2 * [clip(V(t), V(t-1) - epsilon, V(t-1) + epsilon) - ExpectedReward(t)]^2 
// Loss = Gradient (max(Q, Qclipped));
void ndBrainAgentOnPolicyGradient_Trainer::OptimizeCritic()
{
	ndCopyBufferCommandInfo stateValueInfo;
	stateValueInfo.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	stateValueInfo.m_srcOffsetInByte = 0;
	stateValueInfo.m_dstOffsetInByte = 0;
	stateValueInfo.m_dstStrideInByte = stateValueInfo.m_srcStrideInByte;
	stateValueInfo.m_bytesToCopy = stateValueInfo.m_srcStrideInByte;
	
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
	
	ndCopyBufferCommandInfo nextObservationInfo;
	nextObservationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	nextObservationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetNextObsevationOffset() * sizeof(ndReal));
	nextObservationInfo.m_dstOffsetInByte = 0;
	nextObservationInfo.m_dstStrideInByte = ndInt32(m_criticTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	nextObservationInfo.m_bytesToCopy = ndInt32(m_criticTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));

	ndCopyBufferCommandInfo isTerminalBufferInfo;
	isTerminalBufferInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetTerminalOffset() * sizeof(ndReal));
	isTerminalBufferInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	isTerminalBufferInfo.m_dstOffsetInByte = 0;
	isTerminalBufferInfo.m_dstStrideInByte = ndInt32(sizeof(ndBrainFloat));
	isTerminalBufferInfo.m_bytesToCopy = ndInt32(sizeof(ndBrainFloat));

	ndCopyBufferCommandInfo monteCarlosRewardBufferInfo;
	monteCarlosRewardBufferInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetMonteCarlosRewardOffset() * sizeof(ndReal));
	monteCarlosRewardBufferInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	monteCarlosRewardBufferInfo.m_dstOffsetInByte = 0;
	monteCarlosRewardBufferInfo.m_dstStrideInByte = ndInt32(sizeof(ndBrainFloat));
	monteCarlosRewardBufferInfo.m_bytesToCopy = ndInt32(sizeof(ndBrainFloat));

	ndCopyBufferCommandInfo rewardBufferInfo;
	rewardBufferInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetRewardOffset() * sizeof(ndReal));
	rewardBufferInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	rewardBufferInfo.m_dstOffsetInByte = 0;
	rewardBufferInfo.m_dstStrideInByte = ndInt32(sizeof(ndBrainFloat));
	rewardBufferInfo.m_bytesToCopy = ndInt32(sizeof(ndBrainFloat));

	ndCopyBufferCommandInfo previousValueInfo;
	previousValueInfo.m_srcOffsetInByte = 0;
	previousValueInfo.m_srcStrideInByte = ndInt32(sizeof(ndInt32));
	previousValueInfo.m_dstOffsetInByte = 0;
	previousValueInfo.m_dstStrideInByte = ndInt32(sizeof(ndInt32));
	previousValueInfo.m_bytesToCopy = previousValueInfo.m_dstStrideInByte;
	
	const ndInt32 numberOfIterations = ndInt32(m_numberOfIterations);
	ndAssert(numberOfIterations >= 1);
	
	ndBrainFloatBuffer* const epsilon = *m_minibatchLikelihoodRatioBuffer;
	ndBrainFloatBuffer* const inputBuffer = m_criticTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const outputBuffer = m_criticTrainer->GetOuputBuffer();
	ndBrainFloatBuffer* const blendBuffer = *m_minibatchClippedLikelihoodRatioBuffer;
	ndBrainFloatBuffer* const outputGradientBuffer = m_criticTrainer->GetOuputGradientBuffer();

	// calculate the value reference, re use advantage buffer are storage
	for (ndInt32 i = 0; i < numberOfIterations; ++i)
	{
		// only the shuffle buffer is increment to the next batch
		stateValueInfo.m_dstOffsetInByte = i * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
		shuffleBufferInfo.m_srcOffsetInByte = i * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));

		m_minibatchRandomShuffleBuffer->CopyBuffer(shuffleBufferInfo, 1, **m_randomShuffleBuffer);
		inputBuffer->CopyBufferIndirect(observationInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
		m_criticTrainer->MakePrediction();

		// reuse advantage buffer to store the previous state value
		m_advantageBuffer->CopyBuffer(stateValueInfo, 1, *outputBuffer);
	}

	epsilon->Set(ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);
	const ndBrainFloat monetcarloDiscount = ndBrainFloat(ndPow(m_parameters.m_discountRewardFactor, ND_ON_POLICY_MONTE_CARLOS_STEPS));
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
	
			// calculate estimated target value v(target)
			{
				inputBuffer->CopyBufferIndirect(nextObservationInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
				m_criticTrainer->MakePrediction();

				outputBuffer->Scale(monetcarloDiscount);
				m_invSigmaBuffer->CopyBufferIndirect(isTerminalBufferInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
				m_sigmaBuffer->CopyBufferIndirect(rewardBufferInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
				m_sigmaBuffer->Blend(*outputBuffer, **m_invSigmaBuffer);
				m_minibatchAdvantageBuffer->CopyBufferIndirect(monteCarlosRewardBufferInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
				m_minibatchAdvantageBuffer->Add(**m_sigmaBuffer);
			}

			// calculate Q = V(t) - ExpectedReward(t)
			inputBuffer->CopyBufferIndirect(observationInfo, **m_minibatchRandomShuffleBuffer, **m_trainingBuffer);
			m_criticTrainer->MakePrediction();

			m_meanBuffer->Set(*outputBuffer);
			m_meanBuffer->Sub(**m_minibatchAdvantageBuffer);

			// calculate Qclipped = clip(V(t), V(t-1) - epsilon, V(t-1) + epsilon) - ExpectedReward(t)
			{
				// Max (V(t), V(t-1) - epsilon)
				m_sigmaBuffer->CopyBufferIndirect(previousValueInfo, **m_minibatchRandomShuffleBuffer, **m_advantageBuffer);
				m_sigmaBuffer->Sub(*epsilon);
				m_sigmaBuffer->Max(*outputBuffer);

				// -calculate Min (((V(t), V(t-1) - epsilon)), V(t-1) + epsilon))
				m_zMeanBuffer->CopyBufferIndirect(previousValueInfo, **m_minibatchRandomShuffleBuffer, **m_advantageBuffer);
				m_zMeanBuffer->Add(*epsilon);
				m_sigmaBuffer->Min(**m_zMeanBuffer);

				// calculate Qclipped = clip(V(t), V(t-1) - epsilon, V(t-1) + epsilon) - ExpectedReward(t)
				m_sigmaBuffer->Sub(**m_minibatchAdvantageBuffer);
			}
			
			// calculate predicate Q^2 > Qclip^2 ? 1.0 : 0.0
			{
				blendBuffer->Set(**m_meanBuffer);
				m_minibatchAdvantageBuffer->Set(**m_sigmaBuffer);
				blendBuffer->Mul(*blendBuffer);
				m_minibatchAdvantageBuffer->Mul(**m_minibatchAdvantageBuffer);
				blendBuffer->GreaterEqual(**m_minibatchAdvantageBuffer);
			}
			
			// calculate Gradient (max(Q, Qclipped));
			outputGradientBuffer->Set(**m_sigmaBuffer);
			outputGradientBuffer->Blend(**m_meanBuffer, *blendBuffer);
			
			#ifdef ND_DEBUG_CONTINUE_PROXIMA_POLICY
				// validate gradient using the automatic differentiation
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
						const ndBrainDualNumber vt(ndBrainDualNumber(v) + ndBrainDualNumber(0.0f, 1.0f));
			
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

#else

// Using a bootstrapping method based on the Bellman equation:
// expectedValue = r(i) + gamma * stateValue(i + 1);
// The loss is the squared TD error:
// loss = 1/2 * (expectedValue(t) - stateValue(t))^2;
// Therefore, the gradient is based on:
// loss = r(i) + gamma * stateValue(i + 1) - stateValue(i);
// For a terminal state, the future-state value must be excluded:
// loss = r(i) + (gamma * stateValue(i + 1) - stateValue(i)) * terminal;
// it si the worse performe I have,
// I am only keeping it for reference 
void ndBrainAgentOnPolicyGradient_Trainer::OptimizeValue()
{
	ndCopyBufferCommandInfo shuffleBufferInfo;
	shuffleBufferInfo.m_srcOffsetInByte = 0;
	shuffleBufferInfo.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	shuffleBufferInfo.m_dstOffsetInByte = 0;
	shuffleBufferInfo.m_dstStrideInByte = shuffleBufferInfo.m_srcStrideInByte;
	shuffleBufferInfo.m_bytesToCopy = shuffleBufferInfo.m_srcStrideInByte;

	ndCopyBufferCommandInfo nextObservationInfo;
	nextObservationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	nextObservationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetNextObsevationOffset() * sizeof(ndReal));
	nextObservationInfo.m_dstOffsetInByte = 0;
	nextObservationInfo.m_dstStrideInByte = ndInt32(m_valueTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	nextObservationInfo.m_bytesToCopy = nextObservationInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo observationInfo;
	observationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	observationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	observationInfo.m_dstOffsetInByte = 0;
	observationInfo.m_dstStrideInByte = ndInt32(m_valueTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	observationInfo.m_bytesToCopy = observationInfo.m_dstStrideInByte;
	
	ndCopyBufferCommandInfo isTerminalBufferInfo;
	isTerminalBufferInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetTerminalOffset() * sizeof(ndReal));
	isTerminalBufferInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	isTerminalBufferInfo.m_dstOffsetInByte = 0;
	isTerminalBufferInfo.m_dstStrideInByte = ndInt32(sizeof(ndBrainFloat));
	isTerminalBufferInfo.m_bytesToCopy = ndInt32(sizeof(ndBrainFloat));
	
	ndCopyBufferCommandInfo rewardBufferInfo;
	rewardBufferInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetRewardOffset() * sizeof(ndReal));
	rewardBufferInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	rewardBufferInfo.m_dstOffsetInByte = 0;
	rewardBufferInfo.m_dstStrideInByte = ndInt32(sizeof(ndBrainFloat));
	rewardBufferInfo.m_bytesToCopy = ndInt32(sizeof(ndBrainFloat));
	
	const ndInt32 numberOfIterations = ndInt32(m_trajectoryAccumulator.GetCount() / (m_parameters.m_miniBatchSize));
	ndAssert(numberOfIterations >= 1);
	
	ndBrainFloatBuffer* const inputBuffer = m_valueTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const outputBuffer = m_valueTrainer->GetOuputBuffer();
	ndBrainFloatBuffer* const outputGradientBuffer = m_valueTrainer->GetOuputGradientBuffer();
	
	const ndBrainFloat huberSlope = ndBrainFloat(1.0f);
	for (ndInt32 i = 0; i < numberOfIterations; ++i)
	{
		shuffleBufferInfo.m_srcOffsetInByte = i * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndInt32));
		m_minibatchValueShuffleBuffer->CopyBuffer(shuffleBufferInfo, 1, **m_valueShuffleBuffer);

		// using m_invSigmaBuffer as temp
		// calculate tmp = gamma * Q(i + 1)
		inputBuffer->CopyBufferIndirect(nextObservationInfo, **m_minibatchValueShuffleBuffer, **m_trainingBuffer);
		m_valueTrainer->MakePrediction();
		outputBuffer->Scale(m_parameters.m_discountRewardFactor);
		m_invSigmaBuffer->Set(*outputBuffer);

		// calculate Q(i)
		inputBuffer->CopyBufferIndirect(observationInfo, **m_minibatchValueShuffleBuffer, **m_trainingBuffer);
		m_valueTrainer->MakePrediction();

		// calculate (gamma * Q(i + 1) - Q(i))
		outputGradientBuffer->Set(**m_invSigmaBuffer);
		outputGradientBuffer->Sub(*outputBuffer);

		// calculate (gamma * Q(i + 1) - Q(i)) *isTerminal
		m_invSigmaBuffer->CopyBufferIndirect(isTerminalBufferInfo, **m_minibatchValueShuffleBuffer, **m_trainingBuffer);
		outputGradientBuffer->Mul(**m_invSigmaBuffer);

		// calculate loss = r(i) + (gamma * Q(i + 1) - Q(i)) * isTerminal
		m_sigmaBuffer->CopyBufferIndirect(rewardBufferInfo, **m_minibatchValueShuffleBuffer, **m_trainingBuffer);
		outputGradientBuffer->Add(**m_sigmaBuffer);
	
		// maybe apply a Huber loss here
		outputGradientBuffer->Min(huberSlope);
		outputGradientBuffer->Max(-huberSlope);
	
		// back propagate the critic loss
		m_valueTrainer->BackPropagate();
		m_valueTrainer->ApplyLearnRate(m_learnRate);
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
	
	ndSharedPtr<ndBrainFloatBuffer> minbatchDivergenceAcc(m_minibatchAdvantageBuffer);
	ndSharedPtr<ndBrainFloatBuffer> minbatchDivergence(m_minibatchCriticStateValueBuffer);
	
	ndCopyBufferCommandInfo policyActionInfo;
	policyActionInfo.m_srcOffsetInByte = 0;
	policyActionInfo.m_srcStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	policyActionInfo.m_dstOffsetInByte = 0;
	policyActionInfo.m_dstStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	policyActionInfo.m_bytesToCopy = policyActionInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo observationInfo;
	observationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	observationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	observationInfo.m_dstOffsetInByte = 0;
	observationInfo.m_dstStrideInByte = ndInt32(m_valueTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	observationInfo.m_bytesToCopy = observationInfo.m_dstStrideInByte;

	const ndInt32 numberOfIterations = ndInt32(m_trajectoryAccumulator.GetCount() / (m_parameters.m_miniBatchSize));
	ndAssert(numberOfIterations >= 1);

	minbatchDivergenceAcc->Set(ndBrainFloat(0.0f));
	for (ndInt32 i = 0; i < numberOfIterations; ++i)
	{
		inputBuffer->CopyBuffer(observationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
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

//#pragma optimize( "", off )
void ndBrainAgentOnPolicyGradient_Trainer::OptimizePolicy()
{
	ndCopyBufferCommandInfo actionInfo;
	actionInfo.m_srcOffsetInByte = 0;
	actionInfo.m_srcStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	actionInfo.m_dstOffsetInByte = 0;
	actionInfo.m_dstStrideInByte = ndInt32(2 * m_parameters.m_numberOfActions * sizeof(ndReal));
	actionInfo.m_bytesToCopy = actionInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo observationInfo;
	observationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	observationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	observationInfo.m_dstOffsetInByte = 0;
	observationInfo.m_dstStrideInByte = ndInt32(m_valueTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	observationInfo.m_bytesToCopy = observationInfo.m_dstStrideInByte;

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

	ndBrainFloatBuffer* const inputBuffer = m_policyTrainer->GetInputBuffer();
	ndBrainFloatBuffer* const outputBuffer = m_policyTrainer->GetOuputBuffer();

	const ndInt32 numberOfIterations = ndInt32(m_trajectoryAccumulator.GetCount() / (m_parameters.m_miniBatchSize));
	const ndInt32 transitionStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	const ndInt32 policyActionsStrideInBytes = ndInt32(2 * m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions * sizeof(ndReal));
	const ndInt32 minibatchStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	ndAssert(numberOfIterations >= 1);

	// calculate all of the base values
	// policy base action vector and inverx likelihood.
	for (ndInt32 i = 0; i < numberOfIterations; ++i)
	{
		// get the policy output
		inputBuffer->CopyBuffer(observationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_policyTrainer->MakePrediction();
	
		// save base mean and sigma for later use calculation KL divergence.
		m_policyActionBuffer->CopyBuffer(actionInfo, m_parameters.m_miniBatchSize, *outputBuffer);
	
		// calculate the z mean and sigma for the entropy
		m_meanBuffer->CopyBuffer(copyMeanActions, m_parameters.m_miniBatchSize, *outputBuffer);
		m_sigmaBuffer->CopyBuffer(copySigmaActions, m_parameters.m_miniBatchSize, *outputBuffer);
	
		m_zMeanBuffer->CopyBuffer(meanSampledActions, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_zMeanBuffer->Sub(**m_meanBuffer);
	
		// calculate the likelihood
		m_minibatchInvLikelihoodBuffer->CalculateLikelihood(**m_zMeanBuffer, **m_sigmaBuffer);
		m_minibatchInvLikelihoodBuffer->Reciprocal(**m_minibatchInvLikelihoodBuffer);
	
		// save the inverse likelihood
		m_invLikelihoodBuffer->CopyBuffer(advantageReadWriteInfo, m_parameters.m_miniBatchSize, **m_minibatchInvLikelihoodBuffer);
		
		// advance to the next minibatch
		actionInfo.m_dstOffsetInByte += policyActionsStrideInBytes;
		observationInfo.m_srcOffsetInByte += transitionStrideInBytes;
		meanSampledActions.m_srcOffsetInByte += transitionStrideInBytes;
		advantageReadWriteInfo.m_dstOffsetInByte += minibatchStrideInBytes;
	}

	ndInt32 passes = 0;
	ndBrainFloat divergence = ndBrainFloat(0.0f);
	const ndBrainFloat stopDivergence = m_parameters.m_divergenceStopThreshold;

	ndBrainFloatBuffer* const outputGradientBuffer = m_policyTrainer->GetOuputGradientBuffer();
	ndBrainFloatBuffer* const weightAndBiasGradientBuffer = m_policyTrainer->GetWeightAndBiasGradientBuffer();
	for (ndInt32 j = 0; (j < m_parameters.m_divergenceMaxPasses) && (divergence < stopDivergence); ++j)
	{
		passes++;
		// reset info descriptors
		actionInfo.m_dstOffsetInByte = 0;
		advantageReadWriteInfo.m_dstOffsetInByte = 0;
		advantageReadWriteInfo.m_srcOffsetInByte = 0;
		meanSampledActions.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetActionOffset() * sizeof(ndReal));
		observationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));

		m_policyGradientAccumulator->Set(ndBrainFloat(0.0f));
		for (ndInt32 i = 0; i < numberOfIterations; ++i)
		{
			// get the policy output
			inputBuffer->CopyBuffer(observationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
			m_policyTrainer->MakePrediction();

			m_meanBuffer->CopyBuffer(copyMeanActions, m_parameters.m_miniBatchSize, *outputBuffer);
			m_sigmaBuffer->CopyBuffer(copySigmaActions, m_parameters.m_miniBatchSize, *outputBuffer);

			m_zMeanBuffer->CopyBuffer(meanSampledActions, m_parameters.m_miniBatchSize, **m_trainingBuffer);
			m_zMeanBuffer->Sub(**m_meanBuffer);

			// get this mini batch advantage
			m_minibatchAdvantageBuffer->CopyBuffer(advantageReadWriteInfo, m_parameters.m_miniBatchSize, **m_advantageBuffer);
			
			//calculate the clip surrogate factor
			m_minibatchLikelihoodRatioBuffer->CalculateLikelihood(**m_zMeanBuffer, **m_sigmaBuffer);
			m_minibatchInvLikelihoodBuffer->CopyBuffer(advantageReadWriteInfo, m_parameters.m_miniBatchSize, **m_invLikelihoodBuffer);
			m_minibatchLikelihoodRatioBuffer->Mul(**m_minibatchInvLikelihoodBuffer);
			m_minibatchClippedLikelihoodRatioBuffer->Set(**m_minibatchLikelihoodRatioBuffer);
			m_minibatchClippedMinimunZeroGradient->Set(**m_minibatchClippedLikelihoodRatioBuffer);
			m_minibatchClippedMaximumZeroGradient->Set(**m_minibatchClippedLikelihoodRatioBuffer);
			m_minibatchClippedMaximumZeroGradient->LessEqual(ndBrainFloat(1.0f) + ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);
			m_minibatchClippedMinimunZeroGradient->GreaterEqual(ndBrainFloat(1.0f) - ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);
			m_minibatchClippedLikelihoodRatioBuffer->Min(ndBrainFloat(1.0f) + ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);
			m_minibatchClippedLikelihoodRatioBuffer->Max(ndBrainFloat(1.0f) - ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);

			m_minibatchLikelihoodRatioBuffer->Mul(**m_minibatchAdvantageBuffer);
			m_minibatchAdvantageBuffer->Mul(**m_minibatchClippedLikelihoodRatioBuffer);
			m_minibatchAdvantageBuffer->Min(**m_minibatchLikelihoodRatioBuffer);
			m_minibatchAdvantageBuffer->Mul(**m_minibatchClippedMaximumZeroGradient);
			m_minibatchAdvantageBuffer->Mul(**m_minibatchClippedMinimunZeroGradient);

			// calculate gradient
			outputGradientBuffer->CalculateEntropyRegularizationGradient(**m_zMeanBuffer, **m_sigmaBuffer, ndBrainFloat(-1.0f), m_parameters.m_numberOfActions);
			m_minibatchBrocastAdvantageBuffer->BroadcastScaler(**m_minibatchAdvantageBuffer);
			outputGradientBuffer->Mul(**m_minibatchBrocastAdvantageBuffer);
#if 0			
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
#endif			
			m_policyTrainer->BackPropagate();
			m_policyGradientAccumulator->Add(*weightAndBiasGradientBuffer);

			// advance to the next mini batch
			actionInfo.m_dstOffsetInByte += policyActionsStrideInBytes;
			observationInfo.m_srcOffsetInByte += transitionStrideInBytes;
			meanSampledActions.m_srcOffsetInByte += transitionStrideInBytes;
			advantageReadWriteInfo.m_srcOffsetInByte += minibatchStrideInBytes;
		}

		// average all minibaches gradients.
		// use positve gradient as ascend learning rate.
		m_policyGradientAccumulator->Scale(ndBrainFloat(1.0f) / ndBrainFloat(numberOfIterations));
		weightAndBiasGradientBuffer->Set(**m_policyGradientAccumulator);
	
		// update network weight and bias.
		m_policyTrainer->AccumulateWeightAndBiasGradients();
		m_policyTrainer->ApplyLearnRate(m_learnRate * ND_POLICY_DOWN_SAMPLE_LEARN_RATE);
	
		// calculate the KL divergence
		// this will make the agent a vanilla policy gradient
		divergence = CalculateKLdivergence();
	}
	if (passes > 1)
	{
		ndExpandTraceMessage("surrogate loss passes %d  kl: %.6f\n", passes, divergence);
	}
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
	OptimizeValue();
}