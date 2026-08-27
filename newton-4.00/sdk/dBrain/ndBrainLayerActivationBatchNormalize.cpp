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
#include "ndBrainLayerLinear.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainLayerActivationBatchNormalize.h"

#ifdef ND_LEAR_BATCH_NOMALIZED_PARAMS
ndBrainLayerActivationBatchNormalize::ndBrainLayerActivationBatchNormalize(ndInt32 neurons)
	:ndBrainLayerActivation(neurons)
	,m_bias()
	,m_scale()
	,m_trainingBias()
	,m_trainingScale()
	,m_inferenceBias()
	,m_inferenceScale()
{
	m_bias.SetCount(neurons);
	m_scale.SetCount(neurons);
	m_trainingBias.SetCount(neurons);
	m_trainingScale.SetCount(neurons);
	m_inferenceBias.SetCount(neurons);
	m_inferenceScale.SetCount(neurons);

	m_bias.Set(ndBrainFloat(0.0f));
	m_scale.Set(ndBrainFloat(1.0f));
	m_inferenceBias.Set(ndBrainFloat(0.0f));
	m_inferenceScale.Set(ndBrainFloat(1.0f));
}

ndBrainLayerActivationBatchNormalize::ndBrainLayerActivationBatchNormalize(const ndBrainLayerActivationBatchNormalize& src)
	:ndBrainLayerActivation(src)
	,m_bias(src.m_bias)
	,m_scale(src.m_scale)
	,m_trainingBias(src.m_trainingBias)
	,m_trainingScale(src.m_trainingScale)
	,m_inferenceBias(src.m_inferenceBias)
	,m_inferenceScale(src.m_inferenceScale)
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

void ndBrainLayerActivationBatchNormalize::Save(const ndBrainSave* const loadSave) const
{
	ndBrainLayerActivation::Save(loadSave);

	char buffer[1024];
	auto Save = [&buffer, &loadSave](const char* const fmt, ...)
	{
		va_list v_args;
		buffer[0] = 0;
		va_start(v_args, fmt);
		vsnprintf(buffer, sizeof(buffer), fmt, v_args);
		va_end(v_args);
		loadSave->WriteData(buffer);
	};

	Save("\tbias ");
	for (ndInt32 i = 0; i < m_bias.GetCount(); ++i)
	{
		Save("%g ", m_bias[i]);
	}
	Save("\n");

	Save("\tscale ");
	for (ndInt32 i = 0; i < m_scale.GetCount(); ++i)
	{
		Save("%g ", m_scale[i]);
	}
	Save("\n");

	Save("\tinferenceBias ");
	for (ndInt32 i = 0; i < m_inferenceBias.GetCount(); ++i)
	{
		Save("%g ", m_inferenceBias[i]);
	}
	Save("\n");

	Save("\tinferenceScale ");
	for (ndInt32 i = 0; i < m_inferenceScale.GetCount(); ++i)
	{
		Save("%g ", m_inferenceScale[i]);
	}
	Save("\n");
}

ndBrainLayer* ndBrainLayerActivationBatchNormalize::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	ndBrainLayerActivationBatchNormalize* const layer = new ndBrainLayerActivationBatchNormalize(inputs);

	loadSave->ReadString(buffer);
	for (ndInt32 i = 0; i < inputs; ++i)
	{
		ndBrainFloat val = ndBrainFloat(loadSave->ReadFloat());
		layer->m_bias[i] = val;
	}

	loadSave->ReadString(buffer);
	for (ndInt32 i = 0; i < inputs; ++i)
	{
		ndBrainFloat val = ndBrainFloat(loadSave->ReadFloat());
		layer->m_scale[i] = val;
	}

	loadSave->ReadString(buffer);
	for (ndInt32 i = 0; i < inputs; ++i)
	{
		ndBrainFloat val = ndBrainFloat(loadSave->ReadFloat());
		layer->m_inferenceBias[i] = val;
	}

	loadSave->ReadString(buffer);
	for (ndInt32 i = 0; i < inputs; ++i)
	{
		ndBrainFloat val = ndBrainFloat(loadSave->ReadFloat());
		layer->m_inferenceScale[i] = val;
	}

	loadSave->ReadString(buffer);
	return layer;
}

void ndBrainLayerActivationBatchNormalize::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == output.GetCount());
	output.Set(input);
	output.Mul(m_inferenceScale);
	output.Sub(m_inferenceBias);
	ndAssert(output.SanityCheck());
}

ndCommandSharedInfo ndBrainLayerActivationBatchNormalize::GetCommandSharedInfo(ndBrainTrainerInference* const trainer) const
{
	ndCommandSharedInfo info(this);

	ndInt32 columns = ndInt32 (m_bias.GetCount());
	ndInt32 paddedColumns = (columns + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
	ndInt32 size = trainer->RoundOffOffset(2 * paddedColumns);

	info.m_inputSize = columns;
	info.m_outputSize = columns;
	info.m_parametersBatchSize = size;
	return info;
}

void ndBrainLayerActivationBatchNormalize::SetWeights(ndBrainTrainerInference* const, const ndBrainVector& input)
{
	ndInt32 columns = ndInt32(m_bias.GetCount());
	ndInt32 paddedColumns = (columns + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;

	const ndBrainMemVector bias(&input[0], m_bias.GetCount());
	const ndBrainMemVector scale(&input[paddedColumns], m_bias.GetCount());
	m_bias.Set(bias);
	m_scale.Set(scale);
}

void ndBrainLayerActivationBatchNormalize::CopyWeights(ndBrainTrainerInference* const, ndBrainVector& output) const
{
	ndInt32 columns = ndInt32(m_bias.GetCount());
	ndInt32 paddedColumns = (columns + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;

	ndBrainMemVector bias(&output[0], m_bias.GetCount());
	ndBrainMemVector scale(&output[paddedColumns], m_bias.GetCount());
	bias.Set(m_bias);
	scale.Set(m_scale);
}

//void ndBrainLayerActivationBatchNormalize::InputDerivative(const ndBrainVector& input, const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
void ndBrainLayerActivationBatchNormalize::InputDerivative(const ndBrainVector&, const ndBrainVector&, const ndBrainVector&, ndBrainVector&) const
{
	ndAssert(0);
	//ndAssert(input.GetCount() == outputDerivative.GetCount());
	//ndAssert(input.GetCount() == inputDerivative.GetCount());
	//
	//const ndBrainFloat8 one(ndBrainFloat(1.0f));
	//const ndBrainFloat8 zero(ndBrainFloat(0.0f));
	//ndBrainFloat* const dst = &inputDerivative[0];
	//const ndBrainFloat* const src = &input[0];
	//const ndInt32 roundCount = ndInt32(input.GetCount()) & -8;
	//for (ndInt32 i = 0; i < roundCount; i += 8)
	//{
	//	const ndBrainFloat8 x(&src[i]);
	//	const ndBrainFloat8 test(x >= zero);
	//	const ndBrainFloat8 value(test & one);
	//	value.Store(&dst[i]);
	//}
	//for (ndInt32 i = ndInt32(input.GetCount() - 1); i >= roundCount; --i)
	//{
	//	inputDerivative[i] = (input[i] >= ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
	//}
	//inputDerivative.Mul(outputDerivative);
	//ndAssert(inputDerivative.SanityCheck());
}

bool ndBrainLayerActivationBatchNormalize::HasGpuSupport() const
{
	return true;
}

void ndBrainLayerActivationBatchNormalize::CalculateMeanAndVariance(const ndBrainLayerFeedForwardCpuCommand* const command) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainerInference* const trainer = desc.m_owner;
	const ndBrainMemVector inputOutputBuffer((ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr(), ndInt32(trainer->GetHiddenLayerBuffer()->GetCount()));

	ndInt32 inputSize = info.m_inputSize;
	ndInt32 outputSize = info.m_outputSize;
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndBrainFixSizeVector<1024> mean(inputSize);
	ndBrainFixSizeVector<1024> deviation(inputSize);

	mean.Set(ndBrainFloat(0.0f));
	deviation.Set(ndBrainFloat(0.0f));

	const ndInt64 startOffset = inputOutputStartOffset;
	const ndInt32 miniBatches = info.m_matrixDimensionK >> 8;
	for (ndInt32 i = 0; i < miniBatches; ++i)
	{
		ndAssert(inputOutputBuffer.BounceCheck(startOffset + i * ndInt64(inputOutputSize) + outputSize - 1));
		ndBrainMemVector input(&inputOutputBuffer[startOffset + i * ndInt64(inputOutputSize)], outputSize);
		mean.Add(input);
		input.Mul(input);
		deviation.Add(input);
	}

	ndBrainFloat den = ndBrainFloat(1.0f) / ndBrainFloat(miniBatches);
	mean.Scale(den);

	ndBrainFixSizeVector<1024> mean2(inputSize);
	mean2.Set(mean);
	mean2.Mul(mean);
	deviation.Scale(den);
	deviation.Sub(mean2);
	deviation.Max(ndBrainFloat(1.0e-12f));
	deviation.Sqrt();
	deviation.Reciprocal(deviation);
	mean.Mul(deviation);

	m_trainingBias.Set(mean);
	m_trainingScale.Set(deviation);
	m_inferenceBias.Blend(mean, ndBrainFloat(0.01f));
	m_inferenceScale.Blend(deviation, ndBrainFloat(0.01f));
}

void ndBrainLayerActivationBatchNormalize::NormalizeMeanAndVariance(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainerInference* const trainer = desc.m_owner;
	const ndBrainMemVector inputOutputBuffer((ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr(), ndInt32(trainer->GetHiddenLayerBuffer()->GetCount()));

	ndInt32 inputSize = info.m_inputSize;
	ndInt32 outputSize = info.m_outputSize;
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndInt64 inputOffset = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	ndInt64 outputOffset = inputOffset + trainer->RoundOffOffset(inputSize);
	const ndBrainMemVector input(&inputOutputBuffer[inputOffset], inputSize);
	ndBrainMemVector output(&inputOutputBuffer[outputOffset], outputSize);
	output.Set(input);
	output.Mul(m_trainingScale);
	output.Sub(m_trainingBias);
	ndAssert(output.SanityCheck());
}

void ndBrainLayerActivationBatchNormalize::FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	if ((info.m_matrixDimensionK & 0xff) == 0)
	{
		NormalizeMeanAndVariance(command, miniBatchIndex);
	}
	else
	{
		CalculateMeanAndVariance(command);
	}
}

void ndBrainLayerActivationBatchNormalize::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndBrainMemVector inputOutputBuffer((ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr(), ndInt32(trainer->GetHiddenLayerBuffer()->GetCount()));
	const ndBrainMemVector inputOutputGradientsBuffer((ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr(), ndInt32(trainer->GetHiddenLayerGradientBuffer()->GetCount()));
	const ndBrainMemVector weightAndBiasGradients((ndBrainFloat*)trainer->GetWeightAndBiasGradientBuffer()->GetCpuPtr(), ndInt32(trainer->GetWeightAndBiasGradientBuffer()->GetCount()));

	ndInt32 inputSize = info.m_inputSize;
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndInt64 srcBase = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	ndInt64 dstBase = srcBase + trainer->RoundOffOffset(inputSize);
	ndAssert(srcBase >= 0);
	ndAssert(dstBase >= 0);
	ndAssert(inputSize == info.m_outputSize);

	// calculate input gradients
	const ndBrainMemVector outputDerivative(&inputOutputGradientsBuffer[dstBase], inputSize);
	ndBrainMemVector inputDerivative(&inputOutputGradientsBuffer[srcBase], inputSize);
	inputDerivative.Set(m_trainingScale);
	inputDerivative.Mul(outputDerivative);
	ndAssert(inputDerivative.SanityCheck());

	// calculate m_trainingBias gradient
	const ndInt32 columns = ndInt32(m_bias.GetCount());
	const ndInt32 paramSize = info.m_parametersBatchSize;
	ndBrainMemVector biasGrad(&weightAndBiasGradients[paramSize * miniBatchIndex], columns);
	biasGrad.Set(outputDerivative);
	ndAssert(biasGrad.SanityCheck());

	// calculate m_trainingScale gradient
	const ndInt32 paddedColumns = (columns + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
	const ndBrainMemVector input(&inputOutputBuffer[srcBase], inputSize);
	ndBrainMemVector scaleGrad(&weightAndBiasGradients[paramSize * miniBatchIndex + paddedColumns], columns);
	scaleGrad.Set(input);
	scaleGrad.Mul(outputDerivative);
	ndAssert(scaleGrad.SanityCheck());
}

ndCommandArray ndBrainLayerActivationBatchNormalize::CreateFeedForwardBufferCommand(
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

	ndCommandArray commandArray(0);
	if (context->GetAsCpuContext())
	{
		descriptor.m_info.m_matrixDimensionK = descriptor.m_miniBatchSize * 256 + 0;
		descriptor.m_miniBatchSize = 1;
		ndBrainBufferCommand* const meanCommand = new ndBrainLayerFeedForwardCpuCommand(descriptor, (ndBrainLayer*)this);
		commandArray.PushBack(meanCommand);

		descriptor.m_miniBatchSize = descriptor.m_info.m_matrixDimensionK / 256;
		descriptor.m_info.m_matrixDimensionK = 2;
		ndBrainBufferCommand* const normCommand = new ndBrainLayerFeedForwardCpuCommand(descriptor, (ndBrainLayer*)this);
		commandArray.PushBack(normCommand);
	}
	else
	{
		ndAssert(0);
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerReluActivation;
		ndBrainBufferCommand* command = new ndBrainGpuCommand(descriptor);
		commandArray.PushBack(command);
	}

	return commandArray;
}

ndCommandArray ndBrainLayerActivationBatchNormalize::CreateBackPropagateBufferCommand(
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

#else

ndBrainLayerActivationBatchNormalize::ndBrainLayerActivationBatchNormalize(ndInt32 neurons)
	:ndBrainLayerActivationLinear(ndBrainVector(), ndBrainVector())
{
	m_neurons = neurons;
	m_slopes.SetCount(neurons);
	m_biases.SetCount(neurons);

	m_slopes.Set(ndBrainFloat(1.0f));
	m_biases.Set(ndBrainFloat(0.0f));
}

ndBrainLayerActivationBatchNormalize::ndBrainLayerActivationBatchNormalize(const ndBrainLayerActivationBatchNormalize& src)
	:ndBrainLayerActivationLinear(src)
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

void ndBrainLayerActivationBatchNormalize::Save(const ndBrainSave* const loadSave) const
{
	ndBrainLayerActivationLinear::Save(loadSave);
}

ndCommandArray ndBrainLayerActivationBatchNormalize::CreateFeedForwardBufferCommand(
	ndBrainTrainerInference* const owner,
	ndBrainContext* const context,
	const ndCommandSharedInfo& info,
	ndInt32 miniBatchSize,
	ndBrainFloatBuffer* const inputOutputData,
	ndBrainFloatBuffer* const weightsAndBias) const
{
	ndCommandArray commandArray(ndBrainLayerActivationLinear::CreateFeedForwardBufferCommand(
		owner, context, info, miniBatchSize, inputOutputData, weightsAndBias));

	ndBrainBufferCommand* const normalizeCommand = commandArray[0];
	commandArray.SetCount(0);
	normalizeCommand->GetDescriptor().m_info.m_matrixDimensionK = miniBatchSize * 256 + 1;

	ndBrainBufferCommandDesc descriptor(MakeFeedForwardDesctriptor(
		owner, context, info, miniBatchSize, 0,
		inputOutputData, weightsAndBias));

	ndBrainBufferCommand* varianceCommand = nullptr;
	if (context->GetAsCpuContext())
	{
		descriptor.m_info.m_matrixDimensionK = miniBatchSize * 256 + 0;
		descriptor.m_miniBatchSize = 1;
		varianceCommand = new ndBrainLayerFeedForwardCpuCommand(descriptor, (ndBrainLayer*)this);
	}
	else
	{
		ndAssert(0);
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerReluActivation;
		ndBrainBufferCommand* command = new ndBrainGpuCommand(descriptor);
		commandArray.PushBack(command);
	}

	commandArray.PushBack(varianceCommand);
	commandArray.PushBack(normalizeCommand);

	return commandArray;
}

void ndBrainLayerActivationBatchNormalize::CalculateVariance(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainerInference* const trainer = desc.m_owner;
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
	variance.Max(tmp);

	tmp.Blend(variance, ndBrainFloat(0.01f));
	m_slopes.Reciprocal(tmp);
}

void ndBrainLayerActivationBatchNormalize::FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	if ((info.m_matrixDimensionK & 0xff) == 0)
	{
		CalculateVariance(command, miniBatchIndex);
	}
	else
	{
		ndBrainLayerActivationLinear::FeedForward(command, miniBatchIndex);
	}
}

#endif