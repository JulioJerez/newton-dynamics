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
#include "ndBrainLayerActivationBatchNormalize.h"
#include "ndBrainAgentOnPolicyGradient_Trainer.h"

#define ND_POLICY_MAX_KL_DIVERGENCE_PASSES			8
#define ND_POLICY_DOWN_SAMPLE_LEARN_RATE			ndBrainFloat(0.5f)
#define ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON		ndBrainFloat(0.2f)
#define ND_POLICY_KL_DIVERGENCE_STOP_THRESHHOLD		ndBrainFloat(0.001f)

ndBrainAgentOnPolicyGradient_Trainer::HyperParameters::HyperParameters()
{
	m_batchTrajectoryCount = 1000;
	m_enableSelModifyingLayerPass = false;
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
	ndBrainFloat reward = CalculateReward();

	policy->MakePrediction(observation, actions);
	SampleActions(actions);
	ApplyActions(&actions[0]);
	bool isdead = IsTerminal();
	m_trajectory.SetReward(entryIndex, reward);
	m_trajectory.SetAliveState(entryIndex, !isdead);
	m_isDead = m_isDead || isdead;
}

ndBrainAgentOnPolicyGradient_Trainer::ndBrainAgentOnPolicyGradient_Trainer(const HyperParameters& parameters)
	:ndClassAlloc()
	,ndBrainContextUpdateCallback()
	,m_name()
	,m_parameters(parameters)
	,m_context()
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
	,m_averageExpectedRewards()
	,m_averageFramesPerEpisodes()
	,m_trajectoryAccumulator()
	,m_learnRate(m_parameters.m_learnRate)
	,m_frameCount(0)
	,m_horizonSteps(0)
	,m_eposideCount(0)
	,m_trajectiesCount(0)
{
	ndAssert(m_parameters.m_numberOfActions);
	ndAssert(m_parameters.m_numberOfObservations);

	// no entropy regularization
	m_parameters.m_entropyTemperature = ndBrainFloat(0.0f);

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

	layers.PushBack(new ndBrainLayerActivationBatchNormalize(m_parameters.m_numberOfObservations));
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
	auto BuildCriticNetwork = [this]()
	{
		ndFixSizeArray<ndBrainLayer*, 32> layers(0);
		const ndBrain& policy = **m_policyTrainer->GetBrain();

		layers.PushBack(new ndBrainLayerActivationBatchNormalize(policy.GetOutputSize() + policy.GetInputSize()));
		layers.PushBack(new ndBrainLayerLinear(policy.GetOutputSize() + policy.GetInputSize(), m_parameters.m_hiddenLayersNumberOfNeurons));
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
		return critic;
	};

	for (ndInt32 j = 0; j < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++j)
	{
		ndSharedPtr<ndBrain> critic(BuildCriticNetwork());
		ndSharedPtr<ndBrain> referenceCritic(new ndBrain(**critic));
		ndTrainerDescriptor referenceDescriptor(referenceCritic, m_context, m_parameters.m_miniBatchSize);
		referenceDescriptor.m_regularizer = m_parameters.m_criticRegularizer;
		referenceDescriptor.m_regularizerType = m_parameters.m_criticRegularizerType;
		m_referenceCriticTrainer[j] = ndSharedPtr<ndBrainTrainerInference>(new ndBrainTrainerInference(referenceDescriptor));

		ndSharedPtr<ndBrainOptimizer> optimizer(new ndBrainOptimizerAdam(m_context));

		optimizer->SetRegularizer(m_parameters.m_criticRegularizer);
		optimizer->SetRegularizerType(m_parameters.m_criticRegularizerType);

		ndTrainerDescriptor descriptor(critic, m_context, m_parameters.m_miniBatchSize);
		m_criticTrainer[j] = ndSharedPtr<ndBrainTrainer>(new ndBrainTrainer(descriptor, optimizer));
	}
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
}

// the commnet below is not longer valid
// just using bellman equation to calculate state expected reward.
// advantage(i) = reward(i) + gamma * Value(i + 1) - value(i)
// using the terminal condition as predicate
// advantage(i) = reward(i) + alive(t) * gamma * Value(i + 1) - value(i)

void ndBrainAgentOnPolicyGradient_Trainer::CalculateAdvantage()
{
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
	observationInfo.m_dstStrideInByte = ndInt32(m_referenceCriticTrainer[0]->GetBrain()->GetInputSize() * sizeof(ndReal));
	observationInfo.m_bytesToCopy = observationInfo.m_dstStrideInByte;
	
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

	ndCopyBufferCommandInfo expectedRewardBufferInfo;
	expectedRewardBufferInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetExpectedRewardOffset() * sizeof(ndReal));
	expectedRewardBufferInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	expectedRewardBufferInfo.m_dstOffsetInByte = 0;
	expectedRewardBufferInfo.m_dstStrideInByte = ndInt32(sizeof(ndBrainFloat));
	expectedRewardBufferInfo.m_bytesToCopy = ndInt32(sizeof(ndBrainFloat));

	const ndInt32 numberOfMiniBatches = ndInt32(m_trajectoryAccumulator.GetCount() / (m_parameters.m_miniBatchSize));
	ndAssert(numberOfMiniBatches >= 1);
	const ndInt32 advantageStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	const ndInt32 transitionStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * m_trajectoryAccumulator.GetStride() * sizeof(ndReal));

	// rename some buffers as aliases for better code reading 
	ndWeakPtr<ndBrainFloatBuffer> advantage(*m_sigmaBuffer);
	ndWeakPtr<ndBrainFloatBuffer> stateAlive(*m_zMeanBuffer);
	ndWeakPtr<ndBrainFloatBuffer> averageValue(*m_meanBuffer);
	ndWeakPtr<ndBrainFloatBuffer> stateReward(*m_invSigmaBuffer);
	for (ndInt32 j = 0; j < numberOfMiniBatches; ++j)
	{
		averageValue->Set(ndBrainFloat(0.0f));
		for (ndInt32 i = 0; i < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++i)
		{
			ndBrainFloatBuffer* const inputBuffer = m_referenceCriticTrainer[i]->GetInputBuffer();
			ndBrainFloatBuffer* const outputBuffer = m_referenceCriticTrainer[i]->GetOuputBuffer();
			inputBuffer->CopyBuffer(observationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
			m_referenceCriticTrainer[i]->MakePrediction();
			averageValue->Add(*outputBuffer);
		}
		// re user meanBuffer and Sigma and invSigma as temp buffers
		averageValue->Scale(ndBrainFloat(0.5f));
		advantage->CopyBuffer(expectedRewardBufferInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		advantage->Sub(**averageValue);
		stateAlive->CopyBuffer(isAliveBufferInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		stateReward->CopyBuffer(rewardBufferInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		stateReward->Blend(**advantage, **stateAlive);

		// the advantage is collected in stateReward vector 
		// since it is measuring again expected reward, it could be very high
		// I am using a = sign(a) * sqrt(abs(a)
		advantage->Abs(**stateReward);
		advantage->Sqrt(**advantage);
		stateReward->Sign(**stateReward);
		advantage->Mul(**stateReward);
		m_advantageBuffer->CopyBuffer(advantageInfo, 1, **advantage);

		advantageInfo.m_dstOffsetInByte += advantageStrideInBytes;
		observationInfo.m_srcOffsetInByte += transitionStrideInBytes;
		isAliveBufferInfo.m_srcOffsetInByte += transitionStrideInBytes;
		rewardBufferInfo.m_srcOffsetInByte += transitionStrideInBytes;
		expectedRewardBufferInfo.m_srcOffsetInByte += transitionStrideInBytes;
	}
}

// using the same method that DDPG and SAC are suing for the 
// Q value, that is take the minimu of two valsue functions
void ndBrainAgentOnPolicyGradient_Trainer::OptimizeCritics()
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
	observationInfo.m_dstStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	observationInfo.m_bytesToCopy = observationInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo rewardBufferInfo;
	rewardBufferInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetRewardOffset() * sizeof(ndReal));
	rewardBufferInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	rewardBufferInfo.m_dstOffsetInByte = 0;
	rewardBufferInfo.m_dstStrideInByte = ndInt32(sizeof(ndBrainFloat));
	rewardBufferInfo.m_bytesToCopy = ndInt32(sizeof(ndBrainFloat));

	ndCopyBufferCommandInfo expectedRewardBufferInfo;
	expectedRewardBufferInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetExpectedRewardOffset() * sizeof(ndReal));
	expectedRewardBufferInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	expectedRewardBufferInfo.m_dstOffsetInByte = 0;
	expectedRewardBufferInfo.m_dstStrideInByte = ndInt32(sizeof(ndBrainFloat));
	expectedRewardBufferInfo.m_bytesToCopy = ndInt32(sizeof(ndBrainFloat));

	ndCopyBufferCommandInfo isAliveBufferInfo;
	isAliveBufferInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetAliveOffset() * sizeof(ndReal));
	isAliveBufferInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	isAliveBufferInfo.m_dstOffsetInByte = 0;
	isAliveBufferInfo.m_dstStrideInByte = ndInt32(sizeof(ndBrainFloat));
	isAliveBufferInfo.m_bytesToCopy = ndInt32(sizeof(ndBrainFloat));

	const ndInt32 numberOfMiniBatches = ndInt32(m_trajectoryAccumulator.GetCount() / m_parameters.m_miniBatchSize);
	ndAssert(numberOfMiniBatches >= 1);

	ndWeakPtr<ndBrainFloatBuffer> stateAlive(*m_zMeanBuffer);
	ndWeakPtr<ndBrainFloatBuffer> stateReward(*m_invSigmaBuffer);

	const ndBrainFloat huberSlope = ndBrainFloat(1.0f);
	for (ndInt32 j = 0; j < numberOfMiniBatches; ++j)
	{
		shuffleBufferInfo.m_srcOffsetInByte = j * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndInt32));
		m_minibatchValueShuffleBuffer->CopyBuffer(shuffleBufferInfo, 1, **m_valueShuffleBuffer);

		for (ndInt32 i = 0; i < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++i)
		{
			// calculate stateValue(i)
			ndBrainFloatBuffer* const inputBuffer = m_criticTrainer[i]->GetInputBuffer();
			ndBrainFloatBuffer* const outputBuffer = m_criticTrainer[i]->GetOuputBuffer();
			ndBrainFloatBuffer* const outputGradientBuffer = m_criticTrainer[i]->GetOuputGradientBuffer();

			inputBuffer->CopyBufferIndirect(observationInfo, **m_minibatchValueShuffleBuffer, **m_trainingBuffer);
			m_criticTrainer[i]->MakePrediction();
		
			// calculate grad = expectedReward - stateValue(i);
			outputGradientBuffer->CopyBufferIndirect(expectedRewardBufferInfo, **m_minibatchValueShuffleBuffer, **m_trainingBuffer);
			outputGradientBuffer->Sub(*outputBuffer);

			// make sure terminal state get the state reward only
			stateAlive->CopyBufferIndirect(isAliveBufferInfo, **m_minibatchValueShuffleBuffer, **m_trainingBuffer);
			stateReward->CopyBufferIndirect(rewardBufferInfo, **m_minibatchValueShuffleBuffer, **m_trainingBuffer);
			stateReward->Blend(*outputGradientBuffer, **stateAlive);
			outputGradientBuffer->Set(**stateReward);

			// maybe apply a Huber loss here
			outputGradientBuffer->Min(huberSlope);
			outputGradientBuffer->Max(-huberSlope);

			// make gradient descend
			outputGradientBuffer->Scale(ndBrainFloat(-1.0f));
			
			// back propagate the critic loss
			m_criticTrainer[i]->BackPropagate();
			m_criticTrainer[i]->AccumulateWeightAndBiasGradients();
			m_criticTrainer[i]->ApplyLearnRate(m_learnRate);
		}
	}

	// update reference critics using exponential
	// low pass filer Polyak 
	for (ndInt32 i = 0; i < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++i)
	{
		const ndBrainFloatBuffer* const parameterBuffer = m_criticTrainer[i]->GetWeightAndBiasBuffer();
		ndBrainFloatBuffer* const referenceParameterBuffer = m_referenceCriticTrainer[i]->GetWeightAndBiasBuffer();
		referenceParameterBuffer->Blend(*parameterBuffer, m_parameters.m_polyakBlendFactor);
	}
}

void ndBrainAgentOnPolicyGradient_Trainer::UpdateSpecialLayers()
{
	ndCopyBufferCommandInfo policyObservationInfo;
	policyObservationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	policyObservationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	policyObservationInfo.m_dstOffsetInByte = 0;
	policyObservationInfo.m_dstStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	policyObservationInfo.m_bytesToCopy = policyObservationInfo.m_dstStrideInByte;

	ndCopyBufferCommandInfo shuffleBufferInfo;
	shuffleBufferInfo.m_srcOffsetInByte = 0;
	shuffleBufferInfo.m_srcStrideInByte = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	shuffleBufferInfo.m_dstOffsetInByte = 0;
	shuffleBufferInfo.m_dstStrideInByte = shuffleBufferInfo.m_srcStrideInByte;
	shuffleBufferInfo.m_bytesToCopy = shuffleBufferInfo.m_srcStrideInByte;

	ndCopyBufferCommandInfo criticObservationInfo;
	criticObservationInfo.m_srcStrideInByte = ndInt32(m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	criticObservationInfo.m_srcOffsetInByte = ndInt32(m_trajectoryAccumulator.GetObsevationOffset() * sizeof(ndReal));
	criticObservationInfo.m_dstOffsetInByte = 0;
	criticObservationInfo.m_dstStrideInByte = ndInt32(m_criticTrainer[0]->GetBrain()->GetInputSize() * sizeof(ndReal));
	criticObservationInfo.m_bytesToCopy = criticObservationInfo.m_dstStrideInByte;

	const ndInt32 numberOfMiniBatches = ndInt32(m_trajectoryAccumulator.GetCount() / (m_parameters.m_miniBatchSize));
	ndAssert(numberOfMiniBatches >= 1);
	const ndInt32 transitionStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * m_trajectoryAccumulator.GetStride() * sizeof(ndReal));

	ndBrainFloatBuffer* const policyBuffer = m_policyTrainer->GetInputBuffer();
	for (ndInt32 j = 0; j < numberOfMiniBatches; ++j)
	{
		policyBuffer->CopyBuffer(policyObservationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_policyTrainer->UpdateSelfModifyingLayers();

		shuffleBufferInfo.m_srcOffsetInByte = j * ndInt32(m_parameters.m_miniBatchSize * sizeof(ndInt32));
		m_minibatchValueShuffleBuffer->CopyBuffer(shuffleBufferInfo, 1, **m_valueShuffleBuffer);
		for (ndInt32 i = 0; i < ndInt32(sizeof(m_referenceCriticTrainer) / sizeof(m_referenceCriticTrainer[0])); ++i)
		{
			ndBrainFloatBuffer* const cricticTrainerBuffer = m_criticTrainer[i]->GetInputBuffer();
			cricticTrainerBuffer->CopyBufferIndirect(criticObservationInfo, **m_minibatchValueShuffleBuffer, **m_trainingBuffer);
			m_criticTrainer[i]->UpdateSelfModifyingLayers();

			ndBrainFloatBuffer* const cricticReferenceTrainerBuffer = m_referenceCriticTrainer[i]->GetInputBuffer();
			cricticReferenceTrainerBuffer->CopyBufferIndirect(criticObservationInfo, **m_minibatchValueShuffleBuffer, **m_trainingBuffer);
			m_referenceCriticTrainer[i]->UpdateSelfModifyingLayers();
		}

		policyObservationInfo.m_srcOffsetInByte += transitionStrideInBytes;
	}
}

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
	observationInfo.m_dstStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
	observationInfo.m_bytesToCopy = observationInfo.m_dstStrideInByte;

	const ndInt32 numberOfMiniBatches = ndInt32(m_trajectoryAccumulator.GetCount() / (m_parameters.m_miniBatchSize));
	ndAssert(numberOfMiniBatches >= 1);

	minbatchDivergenceAcc->Set(ndBrainFloat(0.0f));
	for (ndInt32 i = 0; i < numberOfMiniBatches; ++i)
	{
		inputBuffer->CopyBuffer(observationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
		m_policyTrainer->MakePrediction();
	
		policyMinibatchBaseDivergence->CopyBuffer(policyActionInfo, m_parameters.m_miniBatchSize, **m_policyActionBuffer);
		minbatchDivergence->CalculatePartialKlDivergence(*policyMinibatchBaseDivergence, *policyMinibatchDivergence);
		minbatchDivergenceAcc->Add(**minbatchDivergence);
	}
	
	minbatchDivergenceAcc->ReductionSum();
	ndBrainFloat sumDivergence = minbatchDivergenceAcc->GetElement(0);
	ndBrainFloat den = ndBrainFloat(numberOfMiniBatches * m_parameters.m_miniBatchSize);
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
	observationInfo.m_dstStrideInByte = ndInt32(m_policyTrainer->GetBrain()->GetInputSize() * sizeof(ndReal));
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

	const ndInt32 numberOfMiniBatches = ndInt32(m_trajectoryAccumulator.GetCount() / (m_parameters.m_miniBatchSize));
	const ndInt32 transitionStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * m_trajectoryAccumulator.GetStride() * sizeof(ndReal));
	const ndInt32 policyActionsStrideInBytes = ndInt32(2 * m_parameters.m_miniBatchSize * m_parameters.m_numberOfActions * sizeof(ndReal));
	const ndInt32 minibatchStrideInBytes = ndInt32(m_parameters.m_miniBatchSize * sizeof(ndReal));
	ndAssert(numberOfMiniBatches >= 1);

	// calculate all of the base values
	// policy base action vector and invert likelihood.
	for (ndInt32 i = 0; i < numberOfMiniBatches; ++i)
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
		
		// advance to the next mini batch
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
		for (ndInt32 i = 0; i < numberOfMiniBatches; ++i)
		{
			// get the policy output
			inputBuffer->CopyBuffer(observationInfo, m_parameters.m_miniBatchSize, **m_trainingBuffer);
			m_policyTrainer->MakePrediction();

			m_meanBuffer->CopyBuffer(copyMeanActions, m_parameters.m_miniBatchSize, *outputBuffer);
			m_sigmaBuffer->CopyBuffer(copySigmaActions, m_parameters.m_miniBatchSize, *outputBuffer);

			m_zMeanBuffer->CopyBuffer(meanSampledActions, m_parameters.m_miniBatchSize, **m_trainingBuffer);
			m_zMeanBuffer->Sub(**m_meanBuffer);

			//calculate the clip surrogate
			//according to the method in the paper
			m_minibatchLikelihoodRatioBuffer->CalculateLikelihood(**m_zMeanBuffer, **m_sigmaBuffer);
			m_minibatchInvLikelihoodBuffer->CopyBuffer(advantageReadWriteInfo, m_parameters.m_miniBatchSize, **m_invLikelihoodBuffer);
			m_minibatchLikelihoodRatioBuffer->Mul(**m_minibatchInvLikelihoodBuffer);

			// calculate positiveG (e, Advantage(s,a)) for A > 0.0
			m_minibatchClippedMinimunZeroGradient->Set(**m_minibatchLikelihoodRatioBuffer);
			m_minibatchClippedMinimunZeroGradient->LessEqual(ndBrainFloat(1.0f) + ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);
			m_minibatchClippedMinimunZeroGradient->Mul(**m_minibatchLikelihoodRatioBuffer);

			// calculate negativeG (e, Advantage(s,a)) for A < 0.0
			m_minibatchClippedMaximumZeroGradient->Set(**m_minibatchLikelihoodRatioBuffer);
			m_minibatchClippedMaximumZeroGradient->GreaterEqual(ndBrainFloat(1.0f) - ND_CONTINUE_PROXIMA_POLICY_CLIP_EPSILON);
			m_minibatchClippedMaximumZeroGradient->Mul(**m_minibatchLikelihoodRatioBuffer);

			// select between positiveG and negativeG based of
			// expected reward sign.
			m_minibatchAdvantageBuffer->CopyBuffer(advantageReadWriteInfo, m_parameters.m_miniBatchSize, **m_advantageBuffer);
			m_minibatchLikelihoodRatioBuffer->Set(**m_minibatchAdvantageBuffer);
			m_minibatchLikelihoodRatioBuffer->GreaterEqual(ndBrainFloat(0.0f));
			m_minibatchClippedMaximumZeroGradient->Blend(**m_minibatchClippedMinimunZeroGradient, **m_minibatchLikelihoodRatioBuffer);
			m_minibatchAdvantageBuffer->Mul(**m_minibatchClippedMaximumZeroGradient);

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

		// average all mini batches gradients.
		// use positive gradient as ascend learning rate.
		m_policyGradientAccumulator->Scale(ndBrainFloat(1.0f) / ndBrainFloat(numberOfMiniBatches));
		weightAndBiasGradientBuffer->Set(**m_policyGradientAccumulator);
	
		// update network weight and bias.
		m_policyTrainer->AccumulateWeightAndBiasGradients();
		m_policyTrainer->ApplyLearnRate(m_learnRate * ND_POLICY_DOWN_SAMPLE_LEARN_RATE);
	
		// calculate KL divergence between original and new policy
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

void ndBrainAgentOnPolicyGradient_Trainer::Update()
{
	UpdateScore();
	TrajectoryToGpuBuffers();

	CalculateAdvantage();
	OptimizePolicy();
	OptimizeCritics();

	if (m_parameters.m_enableSelModifyingLayerPass)
	{
		// Make sure all special self-modifying layers are updated
		// after training and before collecting the next set of trajectories.
		// Otherwise, the policy and critics may be modified by the self-modifying layers.
		UpdateSpecialLayers();
	}
}

void ndBrainAgentOnPolicyGradient_Trainer::Optimize()
{
	m_context->Update(this);
}