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
#include "ndBrainContext.h"
#include "ndBrainTrainer.h"
#include "ndBrainSaveLoad.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainFloatBuffer.h"


#define ND_USE_CPU_TILE_MULTIPLY
// Tile-based matrix multiplication on the CPU is mostly an academic exercise.
// It can be up to 10ms faster per trainning steps on a single core. 
// However, in a multicore setup, the number of tiles is juts too small,
// and there’s significant L1/L2 cache contention. 
// Even a 256x256 matrix is too small for typical neural network training workloads.
// This limitation doesn’t apply to GPUs, since their L1 cache acts as a scratchpad,
// and they don’t need to manage cache coherence the same way CPUs do.#define ndBrainLayerLinearTileSize 32 
// therfere for CPU I am using dot-product based matrix multiplication
// with the wide 32 float wide vector class.

class ndBrainLayerFeedForwardCpuCommand_TiledMatrixMultiply : public ndBrainLayerFeedForwardCpuCommand
{
	public:
	ndBrainLayerFeedForwardCpuCommand_TiledMatrixMultiply(const ndBrainBufferCommandDesc& desc, ndBrainLayer* const layer)
		:ndBrainLayerFeedForwardCpuCommand(desc, layer)
	{
	}

	virtual void Execute(ndInt32 miniBatchIndex) override
	{
		ndBrainLayerLinear* const layer = (ndBrainLayerLinear*)*m_layer;
		layer->TiledMatrixMultiply(this, miniBatchIndex);
	}
};

class ndBrainLayerFeedForwardCpuCommand_TiledMatrixAddBias : public ndBrainLayerFeedForwardCpuCommand
{
	public:
	ndBrainLayerFeedForwardCpuCommand_TiledMatrixAddBias(const ndBrainBufferCommandDesc& desc, ndBrainLayer* const layer)
		:ndBrainLayerFeedForwardCpuCommand(desc, layer)
	{
	}

	virtual void Execute(ndInt32 miniBatchIndex) override
	{
		ndBrainLayerLinear* const layer = (ndBrainLayerLinear*)*m_layer;
		layer->TiledMatrixAddBias(this, miniBatchIndex);
	}
};

class ndBrainLayerFeedForwardCpuCommand_DotProductMatrixMultiply : public ndBrainLayerFeedForwardCpuCommand
{
	public:
	ndBrainLayerFeedForwardCpuCommand_DotProductMatrixMultiply(const ndBrainBufferCommandDesc& desc, ndBrainLayer* const layer)
		:ndBrainLayerFeedForwardCpuCommand(desc, layer)
	{
	}

	virtual void Execute(ndInt32 miniBatchIndex) override
	{
		ndBrainLayerLinear* const layer = (ndBrainLayerLinear*)*m_layer;
		layer->DotProductMatrixMultiply(this, miniBatchIndex);
	}
};

ndBrainLayerLinear::ndBrainLayerLinear(ndInt32 inputs, ndInt32 outputs)
	:ndBrainLayer()
	,m_bias()
	,m_weights(outputs, inputs)
{
	m_bias.SetCount(outputs);
}

ndBrainLayerLinear::ndBrainLayerLinear(const ndBrainLayerLinear& src)
	:ndBrainLayer(src)
	,m_bias(src.m_bias)
	,m_weights(src.m_weights)
{
}

ndBrainLayerLinear::~ndBrainLayerLinear()
{
}

const char* ndBrainLayerLinear::GetLabelId() const
{
	return ND_BRAIN_LAYER_LINEAR_NAME;
}

ndBrainLayer* ndBrainLayerLinear::Clone() const
{
	return new ndBrainLayerLinear(*this);
}

ndInt32 ndBrainLayerLinear::GetOutputSize() const
{
	ndAssert(m_bias.GetCount() == m_weights.GetRows());
	return ndInt32(m_bias.GetCount());
}

ndInt32 ndBrainLayerLinear::GetInputSize() const
{
	return m_weights.GetColumns();
}

void ndBrainLayerLinear::CalculateRoundedSize(ndInt32& width, ndInt32& height) const
{
	width = GetInputSize();
	height = GetOutputSize();
	width = (width + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
	height = (height + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
}

ndBrainVector* ndBrainLayerLinear::GetBias()
{
	return &m_bias;
}

ndBrainMatrix* ndBrainLayerLinear::GetWeights()
{
	return &m_weights;
}

ndInt32 ndBrainLayerLinear::GetNumberOfParameters() const
{
	return ndInt32(m_bias.GetCount()) + m_weights.GetColumns() * m_weights.GetRows();
}

bool ndBrainLayerLinear::HasParameters() const
{
	return true;
}

void ndBrainLayerLinear::InitWeights()
{
	m_bias.Set(ndBrainFloat(0.0f));
	ndBrainFloat variance = ndBrainFloat(ndSqrt(ndFloat32(1.0f) / ndFloat32(GetOutputSize())));
	for (ndInt32 i = ndInt32(m_weights.GetCount() - 1); i >= 0; --i)
	{
		m_weights[i].InitGaussianWeights(variance);
	}
}

void ndBrainLayerLinear::InitWeights_he()
{
	m_bias.Set(ndBrainFloat(0.0f));
	ndBrainFloat den = ndBrainFloat(GetOutputSize());
	// this is a big mistake, for guassian sampling
	//ndBrainFloat variance = ndBrainFloat(ndSqrt(ndFloat32(2.0f) / den));
	ndBrainFloat variance = ndBrainFloat(2.0f) / den;
	for (ndInt32 i = ndInt32(m_weights.GetCount() - 1); i >= 0; --i)
	{
		m_weights[i].InitGaussianWeights(variance);
	}
}

void ndBrainLayerLinear::InitWeights_xavier()
{
	m_bias.Set(ndBrainFloat(0.0f));
	ndBrainFloat den = ndBrainFloat(GetOutputSize() + GetInputSize());
	// uniform
	//ndBrainFloat variance = ndBrainFloat(ndSqrt(ndFloat32(6.0f) / den));
	// normal
	ndBrainFloat variance = ndBrainFloat(ndSqrt(ndFloat32(2.0f) / den));
	for (ndInt32 i = ndInt32(m_weights.GetCount() - 1); i >= 0; --i)
	{
		m_weights[i].InitGaussianWeights(variance);
	}
}

void ndBrainLayerLinear::Clear()
{
	m_bias.Set(ndBrainFloat(0.0f));
	m_weights.Set(ndBrainFloat(0.0f));
}

void ndBrainLayerLinear::FlushToZero()
{
	m_bias.FlushToZero();
	m_weights.FlushToZero();
}

void ndBrainLayerLinear::Scale(ndBrainFloat scale)
{
	m_bias.Scale(scale);
	m_weights.Scale(scale);
}

void ndBrainLayerLinear::Set(const ndBrainLayer& src)
{
	const ndBrainLayerLinear& linearSrc = (ndBrainLayerLinear&)src;
	m_bias.Set(linearSrc.m_bias);
	m_weights.Set(linearSrc.m_weights);
}

void ndBrainLayerLinear::Add(const ndBrainLayer& src)
{
	const ndBrainLayerLinear& linearSrc = (ndBrainLayerLinear&)src;
	m_bias.Add(linearSrc.m_bias);
	m_weights.Add(linearSrc.m_weights);
}

void ndBrainLayerLinear::Mul(const ndBrainLayer& src)
{
	const ndBrainLayerLinear& linearSrc = (ndBrainLayerLinear&)src;
	m_bias.Mul(linearSrc.m_bias);
	m_weights.Mul(linearSrc.m_weights);
}

void ndBrainLayerLinear::ScaleAdd(const ndBrainLayer& src, ndBrainFloat scale)
{
	const ndBrainLayerLinear& linearSrc = (ndBrainLayerLinear&)src;
	m_bias.ScaleAdd(linearSrc.m_bias, scale);
	m_weights.ScaleAdd(linearSrc.m_weights, scale);
}

void ndBrainLayerLinear::AddReqularizerL2(const ndBrainLayer& weights, ndBrainFloat regularizer)
{
	ScaleAdd(weights, regularizer);
}

void ndBrainLayerLinear::AddReqularizerL1(const ndBrainLayer& weights, ndBrainFloat regularizer)
{
	ScaleAdd(weights, regularizer);

	ndBrainFloat negativeRegularizer = -regularizer;
	for (ndInt32 i = ndInt32(m_bias.GetCount()) - 1; i >= 0; --i)
	{
		ndBrainFloat b = m_bias[i];
		m_bias[i] += (b > ndFloat32(0.0f)) ? regularizer : negativeRegularizer;

		ndBrainMemVector& row = m_weights[i];
		for (ndInt32 j = ndInt32(row.GetCount()) - 1; j >= 0; --j)
		{
			ndBrainFloat w = row[j];
			row[j] += (w > ndFloat32(0.0f)) ? regularizer : negativeRegularizer;
		}
	}
}

void ndBrainLayerLinear::Blend(const ndBrainLayer& src, ndBrainFloat blend)
{
	const ndBrainLayerLinear& linearSrc = (ndBrainLayerLinear&)src;
	m_bias.Blend(linearSrc.m_bias, blend);
	m_weights.Blend(linearSrc.m_weights, blend);
}

void ndBrainLayerLinear::AdamUpdate(const ndBrainLayer& u, const ndBrainLayer& v, ndBrainFloat epsilon)
{
	const ndBrainLayerLinear& linear_U = (ndBrainLayerLinear&)u;
	const ndBrainLayerLinear& linear_V = (ndBrainLayerLinear&)v;

	const ndBrainVector& bias_U = linear_U.m_bias;
	const ndBrainVector& bias_V = linear_V.m_bias;
	for (ndInt32 i = ndInt32(m_bias.GetCount() - 1); i >= 0; --i)
	{
		ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(bias_V[i])) + epsilon);
		m_bias[i] = bias_U[i] * bias_den;
	}

	const ndBrainMatrix& weight_U = linear_U.m_weights;
	const ndBrainMatrix& weight_V = linear_V.m_weights;
	for (ndInt32 i = m_weights.GetRows() - 1; i >= 0; --i)
	{
		ndBrainMemVector& row = m_weights[i];
		const ndBrainMemVector& row_U = weight_U[i];
		const ndBrainMemVector& row_V = weight_V[i];
		for (ndInt32 j = ndInt32(row.GetCount() - 1); j >= 0; --j)
		{
			ndBrainFloat weight_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(row_V[j])) + epsilon);
			row[j] = row_U[j] * weight_den;
		}
	}
}

void ndBrainLayerLinear::Save(const ndBrainSave* const loadSave) const
{
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

	Save("\tinputs %d\n", m_weights.GetColumns());
	Save("\toutputs %d\n", m_weights.GetCount());

	Save("\tbias ");
	for (ndInt32 i = 0; i < m_bias.GetCount(); ++i)
	{
		Save("%g ", m_bias[i]);
	}
	Save("\n");

	Save("\tweights\n");
	for (ndInt32 i = 0; i < m_weights.GetCount(); ++i)
	{
		Save("\t\trow_%d ", i);
		const ndBrainVector& row = m_weights[i];
		for (ndInt32 j = 0; j < GetInputSize(); ++j)
		{
			Save("%g ", row[j]);
		}
		Save("\n");
	}
}

ndBrainLayer* ndBrainLayerLinear::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	loadSave->ReadString(buffer);
	ndInt32 outputs = loadSave->ReadInt();
	ndBrainLayerLinear* const layer = new ndBrainLayerLinear(inputs, outputs);

	loadSave->ReadString(buffer);
	for (ndInt32 i = 0; i < outputs; ++i)
	{
		ndBrainFloat val = ndBrainFloat(loadSave->ReadFloat());
		layer->m_bias[i] = val;
	}

	loadSave->ReadString(buffer);
	for (ndInt32 i = 0; i < outputs; ++i)
	{
		loadSave->ReadString(buffer);
		for (ndInt32 j = 0; j < inputs; ++j)
		{
			ndBrainFloat val = ndBrainFloat(loadSave->ReadFloat());
			layer->m_weights[i][j] = val;
		}
	}

	loadSave->ReadString(buffer);
	return layer;
}

void ndBrainLayerLinear::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	m_weights.Mul(input, output);
	output.Add(m_bias);
}

void ndBrainLayerLinear::InputDerivative(const ndBrainVector&, const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	m_weights.TransposeMul(outputDerivative, inputDerivative);
}

void ndBrainLayerLinear::CalculateParamGradients(
	const ndBrainVector& input, const ndBrainVector& ,
	const ndBrainVector& outputDerivative, ndBrainVector& inputGradient, ndBrainLayer* const gradientOut) const
{
	ndAssert(!strcmp(GetLabelId(), gradientOut->GetLabelId()));
	ndBrainLayerLinear* const gradients = (ndBrainLayerLinear*)gradientOut;
	ndAssert(gradients->m_bias.GetCount() == outputDerivative.GetCount());

	gradients->m_bias.Set(outputDerivative);
	for (ndInt32 i = ndInt32(outputDerivative.GetCount() - 1); i >= 0; --i)
	{
		ndBrainFloat value = outputDerivative[i];
		gradients->m_weights[i].ScaleSet(input, value);
	}

	m_weights.TransposeMul(outputDerivative, inputGradient);
}

bool ndBrainLayerLinear::HasGpuSupport() const
{
	return true;
}

void ndBrainLayerLinear::CopyWeights(ndBrainTrainerInference* const trainer, ndBrainVector& output) const
{
	ndInt32 width;
	ndInt32 height;

	CalculateRoundedSize(width, height);
	ndInt32 matrixSize = trainer->RoundOffOffset(width * height);
	ndAssert(output.GetCount() >= (matrixSize + trainer->RoundOffOffset(GetOutputSize())));
	output.Set(ndBrainFloat(0.0f));

	ndInt32 offset = 0;
	ndInt32 columns = m_weights.GetColumns();
	for (ndInt32 i = 0; i < m_weights.GetRows(); ++i)
	{
		const ndBrainVector& src = m_weights[i];
		ndBrainMemVector dst(&output[offset], columns);
		dst.Set(src);
		offset += width;
		ndAssert(offset >= 0);
	}

	ndBrainMemVector bias(&output[matrixSize], m_bias.GetCount());
	bias.Set(m_bias);
}

void ndBrainLayerLinear::SetWeights(ndBrainTrainerInference* const trainer, const ndBrainVector& weightsAnBias)
{
	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	ndInt32 matrixSize = trainer->RoundOffOffset(width * height);
	ndAssert(weightsAnBias.GetCount() >= (matrixSize + trainer->RoundOffOffset(GetOutputSize())));

	ndInt32 offset = 0;
	ndInt32 columns = m_weights.GetColumns();
	for (ndInt32 i = 0; i < m_weights.GetRows(); ++i)
	{
		ndBrainVector& dst = m_weights[i];
		const ndBrainMemVector src(&weightsAnBias[offset], columns);
		dst.Set(src);
		offset += width;
		ndAssert(offset >= 0);
	}
	const ndBrainMemVector bias(&weightsAnBias[matrixSize], m_bias.GetCount());
	m_bias.Set(bias);
}

ndCommandSharedInfo ndBrainLayerLinear::GetCommandSharedInfo(ndBrainTrainerInference* const trainer) const
{
	ndCommandSharedInfo info(this);

	ndInt32 rows = m_weights.GetRows();
	ndInt32 columns = m_weights.GetColumns();

	info.m_outputSize = rows;
	info.m_inputSize = columns;

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	ndInt32 matrixSize = trainer->RoundOffOffset(width * height);
	info.m_parametersBatchSize = matrixSize + trainer->RoundOffOffset(rows);
	return info;
}

void ndBrainLayerLinear::DotProductMatrixMultiply(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex)
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainerInference* const trainer = desc.m_owner;

	const ndBrainMemVector weightAndBias ((ndBrainFloat*)trainer->GetWeightAndBiasBuffer()->GetCpuPtr(), ndInt32 (trainer->GetWeightAndBiasBuffer()->GetCount()));
	const ndBrainMemVector inputOutputBuffer ((ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr(), ndInt32 (trainer->GetHiddenLayerBuffer()->GetCount()));

	const ndInt32 inputSize = info.m_inputSize;
	const ndInt32 outputSize = info.m_outputSize;
	const ndInt32 inputOutputSize = info.m_inputOutputSize;
	const ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	const ndInt32 matrixSize = width * height;
	ndAssert(weightAndBias.BounceCheck(info.m_parametersStartOffset + matrixSize + outputSize - 1));
	const ndBrainMemVector parameters(&weightAndBias[info.m_parametersStartOffset], matrixSize + outputSize);

	const ndInt64 inputOffset = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	const ndInt64 outputOffset = inputOffset + trainer->RoundOffOffset(inputSize);

	ndAssert(inputOutputBuffer.BounceCheck(outputOffset + outputSize - 1));
	ndBrainMemVector output(&inputOutputBuffer[outputOffset], outputSize);

//ndBrainFixSizeVector<1024> xxxx(outputSize);
//xxxx.Set(output);
	const ndBrainMemVector input(&inputOutputBuffer[inputOffset], inputSize);
	for (ndInt32 i = outputSize - 1; i >= 0; --i)
	{
		const ndBrainMemVector row(&parameters[i * width], inputSize);
		output[i] = row.Dot(input);
	}
	const ndBrainMemVector bias(&parameters[matrixSize], outputSize);
	output.Add(bias);
	//for (ndInt32 i = 0; i < outputSize; ++i)
	//{
	//	ndBrainFloat error = ndAbs(output[i] - xxxx[i]);
	//	ndAssert(error < ndBrainFloat(1.0e-5f));
	//}

	ndAssert(output.SanityCheck());
}

void ndBrainLayerLinear::TiledMatrixMultiply(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex)
{
	ndBrainFloat tile_acc[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_ROWS];
	ndBrainFloat tile_weights[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_ROWS];
	ndBrainFloat tile_inputs[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_ROWS];

	for (ndInt32 j = 0; j < ND_GPU_TILED_MATRIX_ROWS; ++j)
	{
		for (ndInt32 i = 0; i < ND_GPU_TILED_MATRIX_ROWS; ++i)
		{
			tile_acc[j][i] = ndBrainFloat(0.0f);
		}
	}

	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainerInference* const trainer = desc.m_owner;

	const ndInt32 inputSize = info.m_inputSize;
	const ndInt32 inputOutputSize = info.m_inputOutputSize;
	const ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	const ndInt32 columns = GetInputSize();
	const ndInt32 kDim = (columns + ND_GPU_TILED_MATRIX_ROWS - 1) / ND_GPU_TILED_MATRIX_ROWS;

	const ndInt32 rowStart = miniBatchIndex / info.m_matrixDimensionK;
	const ndInt32 columStart = miniBatchIndex - rowStart * info.m_matrixDimensionK;

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	const ndInt32 matrixSize = width * height;
	const ndBrainFloat* const weightAndBiasPtr = (ndBrainFloat*)trainer->GetWeightAndBiasBuffer()->GetCpuPtr();
	const ndBrainMemVector weightAndBias(&weightAndBiasPtr[info.m_parametersStartOffset], matrixSize);
	ndAssert(weightAndBias.BounceCheck(matrixSize - 1));

	const ndBrainMemVector inputBuffer((ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr(), ndInt32(trainer->GetHiddenLayerBuffer()->GetCount()));

	const ndInt32 weightsBase = rowStart * width * ND_GPU_TILED_MATRIX_ROWS;
	const ndInt32 inputBase = columStart * inputOutputSize * ND_GPU_TILED_MATRIX_ROWS + inputOutputStartOffset;

	for (ndInt32 k = 0; k < kDim; ++k)
	{
		// load tiles
		ndInt32 inputOffset = inputBase + k * ND_GPU_TILED_MATRIX_ROWS;
		ndInt32 weightOffset = weightsBase + k * ND_GPU_TILED_MATRIX_ROWS;
		for (ndInt32 j = 0; j < ND_GPU_TILED_MATRIX_ROWS; ++j)
		{
			for (ndInt32 i = 0; i < ND_GPU_TILED_MATRIX_ROWS; ++i)
			{
				tile_weights[j][i] = weightAndBias[weightOffset + i];
				tile_inputs[j][i] = inputBuffer[inputOffset + i];
			}
			weightOffset += width;
			inputOffset += inputOutputSize;
		}

		// multiply tiles
		for (ndInt32 j = 0; j < ND_GPU_TILED_MATRIX_ROWS; ++j)
		{
			for (ndInt32 i = 0; i < ND_GPU_TILED_MATRIX_ROWS; ++i)
			{
				ndBrainFloat acc = ndBrainFloat(0.0f);
				for (ndInt32 m = 0; m < ND_GPU_TILED_MATRIX_ROWS; ++m)
				{
					ndBrainFloat input = tile_inputs[i][m];
					ndBrainFloat weight = tile_weights[j][m];
					acc += weight * input;
				}
				tile_acc[j][i] += acc;
			}
		}
	}

	// the tire is transposed, but  
	for (ndInt32 j = 0; j < ND_GPU_TILED_MATRIX_ROWS; ++j)
	{
		for (ndInt32 i = 0; i < ND_GPU_TILED_MATRIX_ROWS; ++i)
		{
			tile_inputs[j][i] = tile_acc[i][j];
		}
	}

	// store tile results
	ndInt64 outputOffset = inputBase + rowStart * ND_GPU_TILED_MATRIX_ROWS + trainer->RoundOffOffset(inputSize);
	ndBrainMemVector outputBuffer((ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr(), ndInt32(trainer->GetHiddenLayerBuffer()->GetCount()));
	for (ndInt32 j = 0; j < ND_GPU_TILED_MATRIX_ROWS; ++j)
	{
		for (ndInt32 i = 0; i < ND_GPU_TILED_MATRIX_ROWS; ++i)
		{
			ndBrainFloat acc = tile_inputs[j][i];
			outputBuffer[outputOffset + i] = acc;
		}
		outputOffset += inputOutputSize;
	}
	ndAssert(outputBuffer.SanityCheck());
}

void ndBrainLayerLinear::TiledMatrixAddBias(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex)
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainerInference* const trainer = desc.m_owner;

	const ndBrainMemVector weightAndBias((ndBrainFloat*)trainer->GetWeightAndBiasBuffer()->GetCpuPtr(), ndInt32(trainer->GetWeightAndBiasBuffer()->GetCount()));
	const ndBrainMemVector inputOutputBuffer((ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr(), ndInt32(trainer->GetHiddenLayerBuffer()->GetCount()));

	const ndInt32 inputSize = info.m_inputSize;
	const ndInt32 outputSize = info.m_outputSize;
	const ndInt32 inputOutputSize = info.m_inputOutputSize;
	const ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	const ndInt32 matrixSize = width * height;
	ndAssert(weightAndBias.BounceCheck(info.m_parametersStartOffset + matrixSize + outputSize - 1));
	const ndBrainMemVector parameters(&weightAndBias[info.m_parametersStartOffset], matrixSize + outputSize);

	const ndInt64 inputOffset = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	const ndInt64 outputOffset = inputOffset + trainer->RoundOffOffset(inputSize);

	ndAssert(inputOutputBuffer.BounceCheck(outputOffset + outputSize - 1));
	ndBrainMemVector output(&inputOutputBuffer[outputOffset], outputSize);
	const ndBrainMemVector bias(&parameters[matrixSize], outputSize);
	output.Add(bias);

	ndAssert(output.SanityCheck());
}

void ndBrainLayerLinear::FeedForward(const ndBrainLayerFeedForwardCpuCommand* const, ndInt32) const
{
	ndAssert(0);
}

ndCommandArray ndBrainLayerLinear::CreateFeedForwardBufferCommand(
	ndBrainTrainerInference* const owner,
	ndBrainContext* const context,
	const ndCommandSharedInfo& info,
	ndInt32 miniBatchSize,
	ndBrainFloatBuffer* const inputOutputData,
	ndBrainFloatBuffer* const weightsAndBias) const
{
	ndAssert(info.m_parametersBatchSize);
	ndCommandArray commandArray(0);

	if (context->GetAsCpuContext())
	{
#ifdef ND_USE_CPU_TILE_MULTIPLY
		// create a tiled based matrix multiply command buffer
		// but it is from two to three time slower.
		{
			// multiply matrix
			ndInt32 rows = GetOutputSize();
			ndInt32 columns = GetInputSize();

			CalculateRoundedSize(columns, rows);
			ndAssert((miniBatchSize & (ND_GPU_TILED_MATRIX_ROWS - 1)) == 0);

			ndInt32 rowDim = rows / ND_GPU_TILED_MATRIX_ROWS;
			ndInt32 columnDim = miniBatchSize / ND_GPU_TILED_MATRIX_ROWS;
			ndBrainBufferCommandDesc tileDescriptor(MakeFeedForwardDesctriptor(
				owner, context, info, columnDim * rowDim, columnDim, inputOutputData, weightsAndBias));

			ndBrainBufferCommand* const tiledCommand = new ndBrainLayerFeedForwardCpuCommand_TiledMatrixMultiply(tileDescriptor, (ndBrainLayer*)this);
			commandArray.PushBack(tiledCommand);
		}

		{
			// add matrix bias
			ndBrainBufferCommandDesc tileDescriptor(MakeFeedForwardDesctriptor(
				owner, context, info, miniBatchSize, 0, inputOutputData, weightsAndBias));

			ndBrainBufferCommand* const tiledCommand = new ndBrainLayerFeedForwardCpuCommand_TiledMatrixAddBias(tileDescriptor, (ndBrainLayer*)this);
			commandArray.PushBack(tiledCommand);
		}

		{
			//// for debug 
			//ndBrainBufferCommandDesc rowDescriptor(MakeFeedForwardDesctriptor(
			//	owner, context, info, miniBatchSize, 0,
			//	inputOutputData, weightsAndBias));
			//ndBrainBufferCommand* const rowCommand = new ndBrainLayerFeedForwardCpuCommand_DotProductMatrixMultiply(rowDescriptor, (ndBrainLayer*)this);
			//commandArray.PushBack(rowCommand);
		}

#else
		// create a dot product based matrix multiply command buffer
		ndBrainBufferCommandDesc rowDescriptor(MakeFeedForwardDesctriptor(
			owner, context, info, miniBatchSize, 0,
			inputOutputData, weightsAndBias));
		ndBrainBufferCommand* const rowCommand = new ndBrainLayerFeedForwardCpuCommand_DotProductMatrixMultiply(rowDescriptor, (ndBrainLayer*)this);
		commandArray.PushBack(rowCommand);
#endif
	}
	else
	{
		{

			ndInt32 rows = GetOutputSize();
			ndInt32 columns = GetInputSize();

			CalculateRoundedSize(columns, rows);
			ndAssert((miniBatchSize & (ND_GPU_TILED_MATRIX_ROWS - 1)) == 0);

			ndInt32 rowDim = rows / ND_GPU_TILED_MATRIX_ROWS;
			ndInt32 columnDim = miniBatchSize / ND_GPU_TILED_MATRIX_ROWS;

			ndBrainBufferCommandDesc descriptor(MakeFeedForwardDesctriptor(
				owner, context, info, columnDim * rowDim, columnDim, inputOutputData, weightsAndBias));
			descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixMatrixMultiply;
			ndBrainBufferCommand* const command = new ndBrainGpuCommand(descriptor, (ndBrainLayer*)this);
			commandArray.PushBack(command);
		}

		{
			// add matrix bias
			ndBrainBufferCommandDesc descriptor(MakeFeedForwardDesctriptor(
				owner, context, info, miniBatchSize, 0, inputOutputData, weightsAndBias));
			descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixMatrixAddBias;
			ndBrainBufferCommand* const command = new ndBrainLayerFeedForwardCpuCommand_TiledMatrixAddBias(descriptor, (ndBrainLayer*)this);
			commandArray.PushBack(command);
		}
	}

	return commandArray;
}

void ndBrainLayerLinear::BackPropagateInputGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndBrainMemVector weightAndBias ((ndBrainFloat*)trainer->GetWeightAndBiasBuffer()->GetCpuPtr(), ndInt64(trainer->GetWeightAndBiasBuffer()->GetCount()));
	const ndBrainMemVector inputOutputGradientsBuffer ((ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr(), ndInt64(trainer->GetHiddenLayerGradientBuffer()->GetCount()));

	const ndInt32 inputSize = info.m_inputSize;
	const ndInt32 outputSize = info.m_outputSize;
	const ndInt32 inputOutputSize = info.m_inputOutputSize;
	const ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	const ndInt32 matrixSize = width * height;

	const ndInt64 srcBase = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	const ndInt64 dstBase = srcBase + trainer->RoundOffOffset(inputSize);
	ndAssert(srcBase >= 0);
	ndAssert(dstBase >= 0);

	ndAssert(inputOutputGradientsBuffer.BounceCheck(dstBase + outputSize - 1));
	ndAssert(weightAndBias.BounceCheck(info.m_parametersStartOffset + matrixSize - 1));
	const ndBrainMemVector outputDerivative(&inputOutputGradientsBuffer[dstBase], outputSize);
	const ndBrainMemVector weightsMatrix(&weightAndBias[info.m_parametersStartOffset], matrixSize);

	ndAssert(inputOutputGradientsBuffer.BounceCheck(srcBase + inputSize - 1));
	ndBrainMemVector inputDerivative(&inputOutputGradientsBuffer[srcBase], inputSize);

//ndBrainFixSizeVector<1024>xxx(inputSize);
//xxx.Set(inputDerivative);

	inputDerivative.Set(ndBrainFloat(0.0f));
	for (ndInt32 i = 0; i < outputSize; ++i)
	{
		ndBrainFloat outDerivative = outputDerivative[i];
		ndAssert(weightsMatrix.BounceCheck(i * width + inputSize - 1));
		const ndBrainMemVector weightsRow(&weightsMatrix[i * width], inputSize);
		inputDerivative.ScaleAdd(weightsRow, outDerivative);
	}

//for (ndInt32 i = 0; i < inputSize; ++i)
//{
//	ndBrainFloat error = ndAbs(inputDerivative[i] - xxx[i]);
//	ndAssert(error < ndBrainFloat(1.0e-2f));
//}

	ndAssert(inputDerivative.SanityCheck());
}

void ndBrainLayerLinear::BackPropagateTileInputGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	ndBrainFloat tile_acc[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_ROWS];
	ndBrainFloat tile_weights[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_ROWS];
	ndBrainFloat tile_outputGrad[ND_GPU_TILED_MATRIX_ROWS][ND_GPU_TILED_MATRIX_ROWS];

	for (ndInt32 j = 0; j < ND_GPU_TILED_MATRIX_ROWS; ++j)
	{
		for (ndInt32 i = 0; i < ND_GPU_TILED_MATRIX_ROWS; ++i)
		{
			tile_acc[j][i] = ndBrainFloat(0.0f);
		}
	}

	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndInt32 inputSize = info.m_inputSize;
	const ndInt32 inputOutputSize = info.m_inputOutputSize;
	const ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	const ndInt32 matrixSize = width * height;

	const ndInt32 kDim = height / ND_GPU_TILED_MATRIX_ROWS;
	const ndInt32 minibatchSize = info.m_matrixDimensionK / m_dimFactor;
	const ndInt32 rowStart = miniBatchIndex / minibatchSize;
	const ndInt32 columStart = miniBatchIndex - rowStart * minibatchSize;

	const ndInt32 weightsBase = columStart * ND_GPU_TILED_MATRIX_ROWS;
	const ndBrainFloat* const weightAndBiasPtr = (ndBrainFloat*)trainer->GetWeightAndBiasBuffer()->GetCpuPtr();
	const ndBrainMemVector weightAndBias(&weightAndBiasPtr[info.m_parametersStartOffset], matrixSize);
	ndBrainMemVector inputOutputGradientsBuffer((ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr(), ndInt64(trainer->GetHiddenLayerGradientBuffer()->GetCount()));
	ndAssert(weightAndBias.BounceCheck(matrixSize - 1));

	const ndInt32 inputBase = rowStart * inputOutputSize * ND_GPU_TILED_MATRIX_ROWS + inputOutputStartOffset;
	const ndInt32 outputBase = inputBase + trainer->RoundOffOffset(inputSize);

	for (ndInt32 k = 0; k < kDim; ++k)
	{
		// load tiles
		ndInt32 outputOffset = outputBase + k * ND_GPU_TILED_MATRIX_ROWS;
		ndInt32 weightOffset = weightsBase + k * width * ND_GPU_TILED_MATRIX_ROWS;
		for (ndInt32 j = 0; j < ND_GPU_TILED_MATRIX_ROWS; ++j)
		{
			for (ndInt32 i = 0; i < ND_GPU_TILED_MATRIX_ROWS; ++i)
			{
				tile_weights[i][j] = weightAndBias[weightOffset + i];
				tile_outputGrad[j][i] = inputOutputGradientsBuffer[outputOffset + i];
			}
			weightOffset += width;
			outputOffset += inputOutputSize;
		}

		// multiply tiles
		for (ndInt32 j = 0; j < ND_GPU_TILED_MATRIX_ROWS; ++j)
		{
			for (ndInt32 i = 0; i < ND_GPU_TILED_MATRIX_ROWS; ++i)
			{
				ndBrainFloat acc = ndBrainFloat(0.0f);
				for (ndInt32 m = 0; m < ND_GPU_TILED_MATRIX_ROWS; ++m)
				{
					ndBrainFloat weight = tile_weights[i][m];
					ndBrainFloat outputGrad = tile_outputGrad[j][m];
					acc += outputGrad * weight;
				}
				tile_acc[j][i] += acc;
			}
		}
	}

	// store tile results
	ndInt64 inputOffset = (columStart + rowStart * inputOutputSize) * ND_GPU_TILED_MATRIX_ROWS + inputOutputStartOffset;
	for (ndInt32 j = 0; j < ND_GPU_TILED_MATRIX_ROWS; ++j)
	{
		for (ndInt32 i = 0; i < ND_GPU_TILED_MATRIX_ROWS; ++i)
		{
			ndBrainFloat acc = tile_acc[j][i];
			inputOutputGradientsBuffer[inputOffset + i] = acc;
		}
		inputOffset += inputOutputSize;
	}
	//ndAssert(outputBuffer.SanityCheck());
}

void ndBrainLayerLinear::BackPropagateBiasGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndBrainMemVector weightAndBiasGradients ((ndBrainFloat*)trainer->GetWeightAndBiasGradientBuffer()->GetCpuPtr(), ndInt32(trainer->GetWeightAndBiasGradientBuffer()->GetCount()));
	const ndBrainMemVector inputOutputGradientsBuffer ((ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr(), ndInt32(trainer->GetHiddenLayerGradientBuffer()->GetCount()));

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);

	const ndInt32 matrixSize = trainer->RoundOffOffset(width * height);
	const ndInt32 inputSize = info.m_inputSize;
	const ndInt32 outputSize = info.m_outputSize;
	const ndInt32 inputOutputSize = info.m_inputOutputSize;
	const ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;
	const ndInt64 inputGradientOffset = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	const ndInt64 outputGradientOffset = inputGradientOffset + trainer->RoundOffOffset(inputSize);

	ndAssert(inputOutputGradientsBuffer.BounceCheck(outputGradientOffset + outputSize - 1));
	ndAssert(weightAndBiasGradients.BounceCheck(info.m_parametersStartOffset + matrixSize + info.m_parametersBatchSize * miniBatchIndex + outputSize - 1));
	const ndBrainMemVector outputDerivative(&inputOutputGradientsBuffer[outputGradientOffset], outputSize);
	ndBrainMemVector biasRowGradients(&weightAndBiasGradients[info.m_parametersStartOffset + matrixSize + info.m_parametersBatchSize * miniBatchIndex], outputSize);
	biasRowGradients.Set(outputDerivative);
	ndAssert(biasRowGradients.SanityCheck());
}

void ndBrainLayerLinear::BackPropagateWeightsGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndBrainMemVector inputOutputBuffer((ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr(), ndInt32(trainer->GetHiddenLayerBuffer()->GetCount()));
	const ndBrainMemVector weightAndBiasGradients((ndBrainFloat*)trainer->GetWeightAndBiasGradientBuffer()->GetCpuPtr(), ndInt32(trainer->GetWeightAndBiasGradientBuffer()->GetCount()));

	const ndInt32 inputSize = info.m_inputSize;
	const ndInt32 outputSize = info.m_outputSize;
	const ndInt32 inputOutputSize = info.m_inputOutputSize;
	const ndInt64 inputOutputStartOffset = info.m_inputOutputStartOffset;

	const ndInt64 srcBase = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	const ndInt32 matrixSize = trainer->RoundOffOffset(width * height);

	ndAssert(inputOutputBuffer.BounceCheck(srcBase + inputSize - 1));
	ndAssert(weightAndBiasGradients.BounceCheck(info.m_parametersStartOffset + matrixSize + info.m_parametersBatchSize * miniBatchIndex + outputSize - 1));
	const ndBrainMemVector inputData(&inputOutputBuffer[srcBase], inputSize);
	const ndBrainMemVector biasRowGradients(&weightAndBiasGradients[info.m_parametersStartOffset + matrixSize + info.m_parametersBatchSize * miniBatchIndex], outputSize);
	const ndInt32 matrixOffsetStart = ndInt32(info.m_parametersStartOffset + info.m_parametersBatchSize * miniBatchIndex);
	for (ndInt32 i = 0; i < outputSize; ++i)
	{
		const ndBrainFloat scale = biasRowGradients[i];
		const ndInt32 matrixOffset = matrixOffsetStart + width * i;
		ndAssert(weightAndBiasGradients.BounceCheck(matrixOffset + inputSize - 1));
		ndBrainMemVector weightRowGradients(&weightAndBiasGradients[matrixOffset], inputSize);
		weightRowGradients.ScaleSet(inputData, scale);
		ndAssert(weightRowGradients.SanityCheck());
	}
}

void ndBrainLayerLinear::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	// Input gradient, weigh and bias pass code 
	// embeded in info.m_matrixDimensionK
	switch (info.m_matrixDimensionK & (m_dimFactor - 1))
	{
		case m_inputGradientsPass:
			BackPropagateInputGradients(command, miniBatchIndex);
			break;

		case m_tiledInputGradientsPass:
			BackPropagateTileInputGradients(command, miniBatchIndex);
			break;

		case m_biasGradientsPass:
			BackPropagateBiasGradients(command, miniBatchIndex);
			break;

		case m_weightGradientsPass:
			BackPropagateWeightsGradients(command, miniBatchIndex);
			break;

		default:;
			ndAssert(0);
	}
}

ndCommandArray ndBrainLayerLinear::CreateBackPropagateBufferCommand(
	ndBrainTrainerInference* const owner,
	ndBrainContext* const context,
	const ndCommandSharedInfo& info,
	ndInt32 miniBatchSize,
	ndBrainFloatBuffer* const inputOutputData,
	ndBrainFloatBuffer* const weightsAndBias,
	ndBrainFloatBuffer* const inputOutputGradients,
	ndBrainFloatBuffer* const weightsAndBiasGradients) const
{
	ndCommandArray commands(0);
	if (context->GetAsCpuContext())
	{
#ifdef ND_USE_CPU_TILE_MULTIPLY
		{
			// calculate the input Gradiends
			ndInt32 width;
			ndInt32 height;
			CalculateRoundedSize(width, height);
			ndInt32 blockColums = width / ND_GPU_TILED_MATRIX_ROWS;
			ndInt32 blockRows = miniBatchSize / ND_GPU_TILED_MATRIX_ROWS;
			ndBrainBufferCommandDesc descriptor(MakeBackpropagateDesctriptor(
				owner, context, info, blockRows * blockColums, blockColums * m_dimFactor + m_tiledInputGradientsPass,
				inputOutputData, weightsAndBias,
				inputOutputGradients, weightsAndBiasGradients));
			ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
			commands.PushBack(command);
		}

		{
			//// for debugging only
			//ndBrainBufferCommandDesc descriptor(MakeBackpropagateDesctriptor(
			//	owner, context, info, miniBatchSize, m_inputGradientsPass,
			//	inputOutputData, weightsAndBias,
			//	inputOutputGradients, weightsAndBiasGradients));
			//ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
			//commands.PushBack(command);
		}

#else

		{
			// calculate the input Gradiends
			ndBrainBufferCommandDesc descriptor(MakeBackpropagateDesctriptor(
				owner, context, info, miniBatchSize, m_inputGradientsPass,
				inputOutputData, weightsAndBias,
				inputOutputGradients, weightsAndBiasGradients));
			ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
			commands.PushBack(command);
		}
#endif

		{
			// calculate the bias gradient
			ndBrainBufferCommandDesc clearBiasDescriptor(MakeBackpropagateDesctriptor(
				owner, context, info, miniBatchSize, m_biasGradientsPass,
				inputOutputData, weightsAndBias,
				inputOutputGradients, weightsAndBiasGradients));
			ndBrainBufferCommand* const clearBiasCommand = new ndBrainLayerBackPropagateCpuCommand(clearBiasDescriptor, (ndBrainLayer*)this);
			commands.PushBack(clearBiasCommand);
		}

		{
			// calculate the weights gradient
			ndBrainBufferCommandDesc descriptor(MakeBackpropagateDesctriptor(
				owner, context, info, miniBatchSize, m_weightGradientsPass,
				inputOutputData, weightsAndBias,
				inputOutputGradients, weightsAndBiasGradients));
			ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
			commands.PushBack(command);
		}
	}
	else
	{
		{
			// calculate the imput/output gradients tile base multiplication 
			ndInt32 width;
			ndInt32 height;
			CalculateRoundedSize(width, height);
			ndInt32 blockColums = width / ND_GPU_TILED_MATRIX_ROWS;
			ndInt32 blockRows = miniBatchSize / ND_GPU_TILED_MATRIX_ROWS;
			ndBrainBufferCommandDesc inputGradDescriptor(MakeBackpropagateDesctriptor(
				owner, context, info, blockRows * blockColums, miniBatchSize,
				inputOutputData, weightsAndBias,
				inputOutputGradients, weightsAndBiasGradients));
			inputGradDescriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixBackPropagateInputGradients;
			ndBrainBufferCommand* const inputGradientCommand = new ndBrainGpuCommand(inputGradDescriptor, (ndBrainLayer*)this);
			commands.PushBack(inputGradientCommand);
		}

		{
			// add the bias gradient kernel;
			ndCommandSharedInfo biasInfo(info);
			ndBrainBufferCommandDesc biasDescriptor(
				MakeBackpropagateDesctriptor(
					owner, context, biasInfo, miniBatchSize, 0,
					inputOutputData, weightsAndBias,
					inputOutputGradients, weightsAndBiasGradients));
			biasDescriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixBackPropagateBiasGradients;
			ndBrainBufferCommand* const biasCommand = new ndBrainGpuCommand(biasDescriptor, (ndBrainLayer*)this);
			commands.PushBack(biasCommand);
		}

		{
			// add the weight gradient kernel;
			ndCommandSharedInfo weightsInfo(info);
			ndBrainBufferCommandDesc weightsDescriptor(
				MakeBackpropagateDesctriptor(
					owner, context, weightsInfo, miniBatchSize * info.m_outputSize, miniBatchSize,
					inputOutputData, weightsAndBias,
					inputOutputGradients, weightsAndBiasGradients));
			weightsDescriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixBackPropagateWeightGradients;
			ndBrainBufferCommand* const weightsCommand = new ndBrainGpuCommand(weightsDescriptor, (ndBrainLayer*)this);
			commands.PushBack(weightsCommand);
		}
	}
	return commands;
}
