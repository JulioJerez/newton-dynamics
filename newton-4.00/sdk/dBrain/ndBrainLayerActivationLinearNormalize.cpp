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
#include "ndBrainFloat8.h"
#include "ndBrainTrainer.h"
#include "ndBrainContext.h"
#include "ndBrainSaveLoad.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainLayerActivationLinearNormalize.h"

ndBrainLayerActivationLinearNormalize::ndBrainLayerActivationLinearNormalize(ndInt32 neurons)
	:ndBrainLayerActivationLinear(ndBrainVector(), ndBrainVector())
	,m_minAverage()
	,m_maxAverage()
	,m_startNormalizing(ND_LINEAR_NORMALIZE_START_NORMALIZE)
{
	m_neurons = neurons;
	m_slopes.SetCount(m_neurons);
	m_biases.SetCount(m_neurons);
	m_minAverage.SetCount(m_neurons);
	m_maxAverage.SetCount(m_neurons);

	m_biases.Set(ndBrainFloat(0.0f));
	m_slopes.Set(ndBrainFloat(1.0f));
	m_minAverage.Set(ndBrainFloat(0.0));
	m_maxAverage.Set(ndBrainFloat(0.0f));
}

ndBrainLayerActivationLinearNormalize::ndBrainLayerActivationLinearNormalize(const ndBrainLayerActivationLinearNormalize& src)
	:ndBrainLayerActivationLinear(src)
	,m_minAverage(src.m_minAverage)
	,m_maxAverage(src.m_maxAverage)
	,m_startNormalizing(ND_LINEAR_NORMALIZE_START_NORMALIZE)
{
}

ndBrainLayer* ndBrainLayerActivationLinearNormalize::Clone() const
{
	return new ndBrainLayerActivationLinearNormalize(*this);
}

const char* ndBrainLayerActivationLinearNormalize::GetLabelId() const
{
	return ND_BRAIN_LAYER_ACTIVATION_LINEAR_NORMALIZE_NAME;
}

ndBrainLayer* ndBrainLayerActivationLinearNormalize::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);
	
	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	
	ndBrainVector activationBiases;
	ndBrainVector activationSlopes;
	activationBiases.SetCount(inputs);
	activationSlopes.SetCount(inputs);
	
	activationBiases.Set(ndFloat32(0.0f));
	activationSlopes.Set(ndFloat32(1.0f));
	ndBrainLayerActivationLinearNormalize* const layer = new ndBrainLayerActivationLinearNormalize(inputs);
	
	loadSave->ReadString(buffer);
	for (ndInt32 i = 0; i < inputs; ++i)
	{
		ndBrainFloat val = ndBrainFloat(loadSave->ReadFloat());
		layer->m_slopes[i] = val;
	}
	
	loadSave->ReadString(buffer);
	for (ndInt32 i = 0; i < inputs; ++i)
	{
		ndBrainFloat val = ndBrainFloat(loadSave->ReadFloat());
		layer->m_biases[i] = val;
	}
	
	loadSave->ReadString(buffer);
	return layer;
}

//#pragma optimize( "", off )
void ndBrainLayerActivationLinearNormalize::UpdateParameters(const ndBrainVector& parameters)
{
	ndAssert(parameters.GetCount() == m_minAverage.GetCount());
	// initilize the moving average
	if (m_startNormalizing == ND_LINEAR_NORMALIZE_START_NORMALIZE)
	{ 
		for (ndInt32 i = ndInt32(parameters.GetCount()) - 1; i >= 0; --i)
		{
			m_minAverage[i] = parameters[i];
			m_maxAverage[i] = parameters[i];
		}
	}
	else if (m_startNormalizing >= 0)
	{
		for (ndInt32 i = ndInt32(parameters.GetCount()) - 1; i >= 0; --i)
		{
			ndBrainFloat s0 = ND_LINEAR_NORMALIZE_MOVING_AVERAGE;
			ndBrainFloat s1 = ndBrainFloat(1.0f) - ND_LINEAR_NORMALIZE_MOVING_AVERAGE;
			if (parameters[i] > m_maxAverage[i])
			{
				m_maxAverage[i] = m_maxAverage[i] * s0 + parameters[i] * s1;
			}
			if (parameters[i] < m_minAverage[i])
			{
				m_minAverage[i] = m_minAverage[i] * s0 + parameters[i] * s1;
			}
		}
	}

	if (m_startNormalizing == 0)
	{
		for (ndInt32 i = ndInt32(m_minAverage.GetCount()) - 1; i >= 0; --i)
		{
			ndBrainFloat den = m_maxAverage[i] - m_minAverage[i];
			ndBrainFloat invDen = ndBrainFloat(1.0f) / (den + ndBrainFloat(1.0e-6f));
			ndBrainFloat slope = ndBrainFloat(2.0f) * invDen;
			ndBrainFloat bias = -(m_maxAverage[i] + m_minAverage[i]) * invDen;
			m_biases[i] = bias;
			m_slopes[i] = slope;
		}
	}

	if (m_startNormalizing)
	{
		m_startNormalizing--;
	}
}

#pragma optimize( "", off )
void ndBrainLayerActivationLinearNormalize::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == output.GetCount());
	ndAssert(m_slopes.GetCount() == m_biases.GetCount());
	ndBrainLayerActivationLinear::MakePrediction(input, output);
}

void ndBrainLayerActivationLinearNormalize::InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	//ndAssert(m_slopes.GetCount() == outputDerivative.GetCount());
	//ndAssert(inputDerivative.GetCount() == outputDerivative.GetCount());
	//
	//inputDerivative.Set(m_slopes);
	//inputDerivative.Mul(outputDerivative);
	ndBrainLayerActivation::InputDerivative(input, output, outputDerivative, inputDerivative);
}

void ndBrainLayerActivationLinearNormalize::FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	//const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	//const ndCommandSharedInfo& info = desc.m_info;
	//ndBrainTrainerInference* const trainer = desc.m_owner;
	//const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();
	//
	//ndInt32 inputSize = info.m_inputSize;
	//ndInt32 outputSize = info.m_outputSize;
	//ndInt32 inputOutputSize = info.m_inputOutputSize;
	//ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;
	//
	//ndInt64 inputOffset = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	//ndInt64 outputOffset = inputOffset + trainer->RoundOffOffset(inputSize);
	//
	//const ndBrainMemVector input(&inputOutputBuffer[inputOffset], inputSize);
	//ndBrainMemVector output(&inputOutputBuffer[outputOffset], outputSize);
	//
	//output.Set(input);
	//output.Mul(m_slopes);
	//output.Add(m_biases);
	ndBrainLayerActivationLinear::FeedForward(command, miniBatchIndex);
}

void ndBrainLayerActivationLinearNormalize::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	//const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	//const ndCommandSharedInfo& info = desc.m_info;
	//ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;
	//
	//const ndBrainFloat* const inputOutputGradientsBuffer = (ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr();
	//
	//ndInt32 inputSize = info.m_inputSize;
	//ndInt32 inputOutputSize = info.m_inputOutputSize;
	//ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;
	//
	//ndInt64 srcBase = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	//ndInt64 dstBase = srcBase + trainer->RoundOffOffset(inputSize);
	//ndAssert(srcBase >= 0);
	//ndAssert(dstBase >= 0);
	//ndAssert(inputSize == info.m_outputSize);
	//
	//const ndBrainMemVector outputDerivative(&inputOutputGradientsBuffer[dstBase], inputSize);
	//ndBrainMemVector inputDerivative(&inputOutputGradientsBuffer[srcBase], inputSize);
	//
	//inputDerivative.Set(m_slopes);
	//inputDerivative.Mul(outputDerivative);
	ndBrainLayerActivationLinear::BackPropagate(command, miniBatchIndex);
}

ndCommandArray ndBrainLayerActivationLinearNormalize::CreateFeedForwardBufferCommand(
	ndBrainTrainerInference* const owner,
	ndBrainContext* const context,
	const ndCommandSharedInfo& info,
	ndInt32 miniBatchSize,
	ndBrainFloatBuffer* const inputOutputData,
	ndBrainFloatBuffer* const weightsAndBias) const
{
	ndBrainBufferCommandDesc descriptor(MakeFeedForwardDesctriptor(
		owner, context, info, miniBatchSize, 0,
		inputOutputData, weightsAndBias));

	ndBrainBufferCommand* command = nullptr;
	if (context->GetAsCpuContext())
	{
		command = new ndBrainLayerFeedForwardCpuCommand(descriptor, (ndBrainLayer*)this);
	}
	else
	{
		ndAssert(0);
		m_slopesBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(context, m_slopes));
		m_biasesBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(context, m_biases));
		descriptor.PushBack(*m_biasesBuffer);
		descriptor.PushBack(*m_slopesBuffer);

		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerLinearActivation;
		command = new ndBrainGpuCommand(descriptor);
	}
	ndCommandArray commandArray(0);
	commandArray.PushBack(command);
	return commandArray;
}

ndCommandArray ndBrainLayerActivationLinearNormalize::CreateBackPropagateBufferCommand(
	ndBrainTrainerInference* const owner,
	ndBrainContext* const context,
	const ndCommandSharedInfo& info,
	ndInt32 miniBatchSize,
	ndBrainFloatBuffer* const inputOutputData,
	ndBrainFloatBuffer* const weightsAndBias,
	ndBrainFloatBuffer* const inputOutputGradients,
	ndBrainFloatBuffer* const weightsAndBiasGradients) const
{
	ndBrainBufferCommandDesc descriptor(MakeBackpropagateDesctriptor(
		owner, context, info, miniBatchSize, 0,
		inputOutputData, weightsAndBias,
		inputOutputGradients, weightsAndBiasGradients));

	ndCommandArray commands(0);

	if (context->GetAsCpuContext())
	{
		ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
		commands.PushBack(command);
	}
	else
	{
		ndAssert(0);
		//descriptor.PushBack(*m_biasesBuffer);
		descriptor.PushBack(*m_slopesBuffer);
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerLinearPropagate;
		ndBrainBufferCommand* const command = new ndBrainGpuCommand(descriptor);
		commands.PushBack(command);
	}
	return commands;
}
