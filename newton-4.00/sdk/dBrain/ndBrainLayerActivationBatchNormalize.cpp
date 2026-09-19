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
#include "ndBrain.h"
#include "ndBrainTrainer.h"
#include "ndBrainContext.h"
#include "ndBrainSaveLoad.h"
#include "ndBrainGpuBuffer.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainLayerActivationBatchNormalize.h"

ndBrainLayerActivationBatchNormalize::ndBrainLayerActivationBatchNormalize(ndInt32 neurons)
	:ndBrainLayerActivationLinear(ndBrainVector(), ndBrainVector())
	,m_varianceBuffer(nullptr)
{
	m_neurons = neurons;
	m_slopes.SetCount(neurons);
	m_biases.SetCount(neurons);
	m_biases.Set(ndBrainFloat(0.0f));
	m_slopes.Set(ndBrainFloat(1.0f));
}

ndBrainLayerActivationBatchNormalize::ndBrainLayerActivationBatchNormalize(const ndBrainLayerActivationBatchNormalize& src)
	:ndBrainLayerActivationLinear(src)
	,m_varianceBuffer(nullptr)
{
}

ndBrainLayer* ndBrainLayerActivationBatchNormalize::Clone() const
{
	return new ndBrainLayerActivationBatchNormalize(*this);
}

const char* ndBrainLayerActivationBatchNormalize::GetLabelId() const
{
	return ND_BRAIN_LAYER_ACTIVATION_BATCH_NORMALIZE_NAME;
}

ndBrainLayer* ndBrainLayerActivationBatchNormalize::Load(const ndBrainLoad* const loadSave)
{
	return ndBrainLayerActivationLinear::Load(loadSave);
}

ndCommandArray ndBrainLayerActivationBatchNormalize::CreateSelfModyfingFeedForwardBufferCommand(
	ndBrainTrainerInference* const owner,
	ndBrainContext* const context,
	const ndCommandSharedInfo& info,
	ndInt32 miniBatchSize,
	ndBrainFloatBuffer* const inputOutputData,
	ndBrainFloatBuffer* const weightsAndBias) const
{
	if (context->GetAsCpuContext())
	{
		ndCommandArray commandArray(ndBrainLayerActivationLinear::CreateFeedForwardBufferCommand(
			owner, context, info, miniBatchSize, inputOutputData, weightsAndBias));

		ndBrainBufferCommand* const linearActivationCommand = commandArray[0];
		commandArray.SetCount(0);
		linearActivationCommand->GetDescriptor().m_info.m_matrixDimensionK = miniBatchSize * 256 + 1;

		ndBrainBufferCommandDesc descriptor(MakeFeedForwardDesctriptor(
			owner, context, info, miniBatchSize, 0,
			inputOutputData, weightsAndBias));

		descriptor.m_miniBatchSize = 1;
		descriptor.m_info.m_matrixDimensionK = miniBatchSize * 256 + 0;
		ndBrainBufferCommand* const varianceCommand = new ndBrainLayerSelfModyfyingFeedForwardCpuCommand(descriptor, (ndBrainLayer*)this);

		commandArray.PushBack(varianceCommand);
		commandArray.PushBack(linearActivationCommand);
		return commandArray;
	}
	else
	{	
		// add the bash sumation, reduction and normalization
		auto TwosPower = [](ndInt32 x)
		{
			ndInt32 exp = 0;
			for (x-- ; x > 0; x >>= 1)
			{
				exp++;
			}
			return exp;
		};

		ndCommandArray commandArray(0);
		ndBrainBufferCommandDesc descriptor(MakeFeedForwardDesctriptor(
			owner, context, info, miniBatchSize, 0, inputOutputData, weightsAndBias));

		ndInt32 twosPower = TwosPower(miniBatchSize);
		ndAssert(twosPower > 0);
		if (!m_varianceBuffer)
		{
			ndInt32 size = (1 << twosPower) * ndInt32(m_slopes.GetCount());
			m_varianceBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(context, size));
			m_varianceBuffer->Set(ndBrainFloat(0.0f));
		}
		ndAssert(m_slopesBuffer);
		ndAssert(m_biasesBuffer);

		descriptor.PushBack(*m_biasesBuffer);
		descriptor.PushBack(*m_slopesBuffer);
		descriptor.PushBack(*m_varianceBuffer);

		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerBatchNormalizationLoadInputActivation;
		ndBrainBufferCommand* const loadInputCommand = new ndBrainGpuCommand(descriptor, (ndBrainLayer*)this);
		commandArray.PushBack(loadInputCommand);

		ndInt32 savedWorkGroupSize = descriptor.m_workGroupSize;
		for (; twosPower > 0; --twosPower)
		{
			descriptor.m_miniBatchSize = 1<<(twosPower - 1);
			descriptor.m_workGroupSize = 1 << (twosPower - 1);
			descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerBatchNormalizationAddInputActivation;
			ndBrainBufferCommand* const reductionCommand = new ndBrainGpuCommand(descriptor, (ndBrainLayer*)this);
			commandArray.PushBack(reductionCommand);
		}
		descriptor.m_miniBatchSize = 1;
		descriptor.m_workGroupSize = savedWorkGroupSize;
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerBatchNormalizationNormalizeInputActivation;
		ndBrainBufferCommand* const varianceCommand = new ndBrainGpuCommand(descriptor, (ndBrainLayer*)this);
		commandArray.PushBack(varianceCommand);

		// add the linear layer ax + b manually
		descriptor.m_miniBatchSize = miniBatchSize;
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerLinearActivation;
		ndBrainBufferCommand* const linearCommand = new ndBrainGpuCommand(descriptor, (ndBrainLayer*)this);
		commandArray.PushBack(linearCommand);

		return commandArray;
	}
}

void ndBrainLayerActivationBatchNormalize::SelfModifyingFeedForward(const ndBrainLayerSelfModyfyingFeedForwardCpuCommand* const command, ndInt32) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainerInference* const trainer = (ndBrainTrainerInference*)*desc.m_owner;
	const ndBrainMemVector inputOutputBuffer((ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr(), ndInt32(trainer->GetHiddenLayerBuffer()->GetCount()));
	
	ndInt32 inputSize = info.m_inputSize;
	ndInt32 outputSize = info.m_outputSize;
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;
	
	ndBrainFixSizeVector<1024> tmp(inputSize);
	ndBrainFixSizeVector<1024> variance(inputSize);
	variance.Set(ndBrainFloat(0.0f));
	
	const ndInt64 startOffset = inputOutputStartOffset;
	const ndInt32 miniBatches = info.m_matrixDimensionK >> 8;
	for (ndInt32 i = 0; i < miniBatches; ++i)
	{
		ndAssert(inputOutputBuffer.BounceCheck(startOffset + i * ndInt64(inputOutputSize) + outputSize - 1));
		const ndBrainMemVector input(&inputOutputBuffer[startOffset + i * ndInt64(inputOutputSize)], outputSize);
		tmp.Set(input);
		tmp.Mul(input);
		variance.Add(tmp);
	}
	
	ndBrainFloat den = ndBrainFloat(1.0f) / ndBrainFloat(miniBatches);
	variance.Scale(den);
	variance.Max(ndBrainFloat(1.0e-12f));
	variance.Sqrt();
	tmp.Reciprocal(m_slopes);
	
	tmp.Blend(variance, ndBrainFloat(0.01f));
	m_slopes.Reciprocal(tmp);
}
