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
#include "ndBrainLayerActivationSigmoid.h"

ndBrainLayerActivationSigmoid::ndBrainLayerActivationSigmoid(ndInt32 neurons)
	:ndBrainLayerActivation(neurons)
{
}

ndBrainLayerActivationSigmoid::ndBrainLayerActivationSigmoid(const ndBrainLayerActivationSigmoid& src)
	:ndBrainLayerActivation(src)
{
}

ndBrainLayer* ndBrainLayerActivationSigmoid::Clone() const
{
	return new ndBrainLayerActivationSigmoid(*this);
}

const char* ndBrainLayerActivationSigmoid::GetLabelId() const
{
	return "ndBrainLayerActivationSigmoid";
}

ndBrainLayer* ndBrainLayerActivationSigmoid::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	ndBrainLayerActivationSigmoid* const layer = new ndBrainLayerActivationSigmoid(inputs);
	loadSave->ReadString(buffer);
	return layer;
}

void ndBrainLayerActivationSigmoid::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == output.GetCount());
	//for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= 0; --i)
	//{
	//	ndBrainFloat value = ndClamp(input[i], ndBrainFloat(-30.0f), ndBrainFloat(30.0f));
	//	ndBrainFloat x = ndBrainFloat(ndExp(-value));
	//	output[i] = ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) + x);
	//	ndAssert(ndCheckFloat(output[i]));
	//	ndAssert(output[i] <= ndBrainFloat(1.0f));
	//	ndAssert(output[i] >= ndBrainFloat(0.0f));
	//}
	output.Set(input);
	output.Clamp(ndBrainFloat(-30.0f), ndBrainFloat(30.0f));
	output.Scale(ndBrainFloat(-1.0f));
	output.Exp(output);
	output.Add(ndBrainFloat(1.0f));
	output.Reciprocal(output);
	ndAssert(output.SanityCheck());
}

void ndBrainLayerActivationSigmoid::InputDerivative(const ndBrainVector&, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndAssert(output.GetCount() == outputDerivative.GetCount());
	ndAssert(output.GetCount() == inputDerivative.GetCount());

	inputDerivative.Set(ndBrainFloat(1.0f));
	inputDerivative.Sub(output);
	inputDerivative.Mul(output);

	inputDerivative.Mul(outputDerivative);
	inputDerivative.FlushToZero();
}

void ndBrainLayerActivationSigmoid::FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainerInference* const trainer = desc.m_owner;
	const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();
	
	ndInt32 inputSize = info.m_inputSize;
	ndInt32 outputSize = info.m_outputSize;
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;
	
	ndInt64 inputOffset = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	ndInt64 outputOffset = inputOffset + trainer->RoundOffOffset(inputSize);
	
	const ndBrainMemVector input(&inputOutputBuffer[inputOffset], inputSize);
	ndBrainMemVector output(&inputOutputBuffer[outputOffset], outputSize);

	output.Set(input);
	output.Clamp(ndBrainFloat(-30.0f), ndBrainFloat(30.0f));
	output.Scale(ndBrainFloat(-1.0f));
	output.Exp(output);
	output.Add(ndBrainFloat(1.0f));
	output.Reciprocal(output);
	ndAssert(output.SanityCheck());
}

void ndBrainLayerActivationSigmoid::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;
	
	const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();
	const ndBrainFloat* const inputOutputGradientsBuffer = (ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr();
	
	ndInt32 inputSize = info.m_inputSize;
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;
	
	ndInt64 srcBase = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	ndInt64 dstBase = srcBase + trainer->RoundOffOffset(inputSize);
	ndAssert(srcBase >= 0);
	ndAssert(dstBase >= 0);
	ndAssert(inputSize == info.m_outputSize);
	
	const ndBrainMemVector input(&inputOutputBuffer[srcBase], inputSize);
	const ndBrainMemVector output(&inputOutputBuffer[dstBase], inputSize);
	const ndBrainMemVector outputDerivative(&inputOutputGradientsBuffer[dstBase], inputSize);
	ndBrainMemVector inputDerivative(&inputOutputGradientsBuffer[srcBase], inputSize);
	
	inputDerivative.Set(ndBrainFloat(1.0f));
	inputDerivative.Sub(output);
	inputDerivative.Mul(output);
	inputDerivative.Mul(outputDerivative);
	ndAssert(inputDerivative.SanityCheck());
}

ndCommandArray ndBrainLayerActivationSigmoid::CreateFeedForwardBufferCommand(
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
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerReluActivation;
		command = new ndBrainGpuCommand(descriptor);
	}

	ndCommandArray commandArray(0);
	commandArray.PushBack(command);
	return commandArray;
}

ndCommandArray ndBrainLayerActivationSigmoid::CreateBackPropagateBufferCommand(
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
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerReluBackPropagate;
		ndBrainBufferCommand* const command = new ndBrainGpuCommand(descriptor);
		commands.PushBack(command);
	}
	return commands;
}

