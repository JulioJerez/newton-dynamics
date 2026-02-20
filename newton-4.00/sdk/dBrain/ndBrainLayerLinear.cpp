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

//#define ND_USE_CPU_TILE_MULTIPLY
//#define ndBrainLayerLinearTileSize 16 // this is better but still some is no correct.
#define ndBrainLayerLinearTileSize 32 // for some reason, this is too slow when multi threading

D_MSV_NEWTON_CLASS_ALIGN_32
class ndBrainFloatTileVector
{
	public:
	ndBrainFloatTileVector() = default;
	ndBrainFloatTileVector(const ndBrainFloat a);
	ndBrainFloatTileVector(const ndBrainFloatTileVector& a);
	ndBrainFloatTileVector(const ndInt32* const indexArray);
	ndBrainFloatTileVector(const ndBrainFloat* const baseAddr, const ndBrainFloatTileVector& index);

	ndBrainFloat HorizontalAdd() const;
	ndBrainFloatTileVector Scale(ndBrainFloat s) const;
	ndBrainFloatTileVector MulAdd(const ndBrainFloatTileVector& A, const ndBrainFloat scale) const;
	ndBrainFloatTileVector MulAdd(const ndBrainFloatTileVector& A, const ndBrainFloatTileVector& B) const;

	ndBrainFloatTileVector& operator= (const ndBrainFloatTileVector& A);
	ndBrainFloatTileVector operator+ (const ndBrainFloatTileVector& A) const;
	ndBrainFloatTileVector operator* (const ndBrainFloatTileVector& A) const;

	#ifdef D_NEWTON_USE_AVX2_OPTION
	ndBrainFloatTileVector(const __m256* const reg);
	#endif

	union
	{
		ndBrainFloat m_f[ndBrainLayerLinearTileSize];
		ndInt32 m_i[ndBrainLayerLinearTileSize];

		#ifdef D_NEWTON_USE_AVX2_OPTION
		__m256 m_register[ndBrainLayerLinearTileSize / 8];
		#endif	
	};
} D_GCC_NEWTON_CLASS_ALIGN_32;

#ifdef D_NEWTON_USE_AVX2_OPTION

	inline ndBrainFloatTileVector::ndBrainFloatTileVector(const ndInt32* const indexArray)
	{
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize; ++i)
		{
			m_i[i] = indexArray[i];
		}
	}

	inline ndBrainFloatTileVector::ndBrainFloatTileVector(const ndBrainFloat a)
	{
		__m256 reg (_mm256_set1_ps(a));
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize / 8; ++i)
		{
			m_register[i] = reg;
		}
	}

	inline ndBrainFloatTileVector::ndBrainFloatTileVector(const ndBrainFloatTileVector& a)
	{
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize / 8; ++i)
		{
			m_register[i] = a.m_register[i];
		}
	}

	inline ndBrainFloatTileVector::ndBrainFloatTileVector(const ndBrainFloat* const baseAddr, const ndBrainFloatTileVector& index)
	{
		__m256i* const indexReg = (__m256i*) & index.m_i[0];
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize / 8; ++i)
		{
			m_register[i] = _mm256_i32gather_ps(baseAddr, indexReg[i], 4);
		}
	}

	inline ndBrainFloatTileVector& ndBrainFloatTileVector::operator= (const ndBrainFloatTileVector& A)
	{
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize / 8; ++i)
		{
			m_register[i] = A.m_register[i];
		}
		return *this;
	}

	inline ndBrainFloatTileVector::ndBrainFloatTileVector(const __m256* const reg)
	{
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize / 8; ++i)
		{
			m_register[i] = reg[i];
		}
	}

	inline ndBrainFloat ndBrainFloatTileVector::HorizontalAdd() const
	{
		__m256 tmp(_mm256_setzero_ps());
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize / 8; ++i)
		{
			tmp = _mm256_add_ps(tmp, m_register[i]);
		}

		const __m128 hiQuad = _mm256_extractf128_ps(tmp, 1);
		const __m128 loQuad = _mm256_castps256_ps128(tmp);

		const __m128 tmp0(_mm_add_ps(loQuad, hiQuad));
		const __m128 tmp1(_mm_add_ps(tmp0, _mm_movehl_ps(tmp0, tmp0)));
		const __m128 tmp2(_mm_add_ps(tmp1, _mm_permute_ps(tmp1, PERMUTE_MASK(2, 3, 0, 1))));
		return _mm_cvtss_f32(tmp2);
	}

	inline ndBrainFloatTileVector ndBrainFloatTileVector::Scale(ndBrainFloat s) const
	{
		const __m256 scale(_mm256_set1_ps(s));
		__m256 tmp[ndBrainLayerLinearTileSize / 8];
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize / 8; ++i)
		{
			tmp[i] = _mm256_mul_ps(m_register[i], scale);
		}
		return ndBrainFloatTileVector(tmp);
	}

	inline ndBrainFloatTileVector ndBrainFloatTileVector::operator+ (const ndBrainFloatTileVector& A) const
	{
		__m256 tmp[ndBrainLayerLinearTileSize / 8];
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize / 8; ++i)
		{
			tmp[i] = _mm256_add_ps(m_register[i], A.m_register[i]);
		}
		return ndBrainFloatTileVector(tmp);
	}

	inline ndBrainFloatTileVector ndBrainFloatTileVector::operator* (const ndBrainFloatTileVector& A) const
	{
		__m256 tmp[ndBrainLayerLinearTileSize / 8];
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize / 8; ++i)
		{
			tmp[i] = _mm256_mul_ps(m_register[i], A.m_register[i]);
		}
		return ndBrainFloatTileVector(tmp);
	}

	ndBrainFloatTileVector ndBrainFloatTileVector::MulAdd(const ndBrainFloatTileVector& A, const ndBrainFloatTileVector& B) const
	{
		__m256 tmp[ndBrainLayerLinearTileSize / 8];
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize / 8; ++i)
		{
			tmp[i] = _mm256_fmadd_ps(A.m_register[i], B.m_register[i], m_register[i]);
		}
		return ndBrainFloatTileVector(tmp);
	}

	ndBrainFloatTileVector ndBrainFloatTileVector::MulAdd(const ndBrainFloatTileVector& A, const ndBrainFloat scale) const
	{
		const __m256 reg(_mm256_set1_ps(scale));
		__m256 tmp[ndBrainLayerLinearTileSize / 8];

		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize / 8; ++i)
		{
			tmp[i] = _mm256_fmadd_ps(A.m_register[i], reg, m_register[i]);
		}
		return ndBrainFloatTileVector(tmp);
	}

#else

	inline ndBrainFloatTileVector::ndBrainFloatTileVector(const ndBrainFloatTileVector& a)
	{
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize; ++i)
		{
			m_f[i] = a.m_f[i];
		}
	}

	inline ndBrainFloatTileVector::ndBrainFloatTileVector(const ndBrainFloat a)
	{
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize; ++i)
		{
			m_f[i] = a;
		}
	}

	inline ndBrainFloatTileVector::ndBrainFloatTileVector(const ndInt32* const indexArray)
	{
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize; ++i)
		{
			m_i[i] = indexArray[i];
		}
	}

	inline ndBrainFloatTileVector::ndBrainFloatTileVector(const ndBrainFloat* const baseAddr, const ndBrainFloatTileVector& index)
	{
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize; ++i)
		{
			const ndInt32 j = index.m_i[i];
			m_f[i] = baseAddr[j];
		}
	}

	inline ndBrainFloat ndBrainFloatTileVector::HorizontalAdd() const
	{
		ndBrainFloat acc = ndBrainFloat(0.0f);
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize; ++i)
		{
			acc += m_f[i];
		}
		return acc;
	}

	inline ndBrainFloatTileVector ndBrainFloatTileVector::Scale(ndBrainFloat s) const
	{
		ndBrainFloatTileVector tmp;
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize; ++i)
		{
			tmp.m_f[i] = m_f[i] + s;
		}
		return tmp;
	}

	inline ndBrainFloatTileVector& ndBrainFloatTileVector::operator= (const ndBrainFloatTileVector& A)
	{
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize; ++i)
		{
			m_f[i] = A.m_f[i];
		}
		return *this;
	}

	inline ndBrainFloatTileVector ndBrainFloatTileVector::operator+ (const ndBrainFloatTileVector& A) const
	{
		ndBrainFloatTileVector tmp;
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize; ++i)
		{
			tmp.m_f[i] = m_f[i] + A.m_f[i];
		}
		return tmp;
	}

	inline ndBrainFloatTileVector ndBrainFloatTileVector::operator* (const ndBrainFloatTileVector& A) const
	{
		ndBrainFloatTileVector tmp;
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize; ++i)
		{
			tmp.m_f[i] = m_f[i] * A.m_f[i];
		}
		return tmp;
	}

	ndBrainFloatTileVector ndBrainFloatTileVector::MulAdd(const ndBrainFloatTileVector& A, const ndBrainFloatTileVector& B) const
	{
		ndBrainFloatTileVector tmp;
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize; ++i)
		{
			tmp.m_f[i] = m_f[i] + A.m_f[i] * B.m_f[i];
		}
		return ndBrainFloatTileVector(tmp);
	}

	ndBrainFloatTileVector ndBrainFloatTileVector::MulAdd(const ndBrainFloatTileVector& A, const ndBrainFloat scale) const
	{
		ndBrainFloatTileVector tmp;
		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize; ++i)
		{
			tmp.m_f[i] = m_f[i] + A.m_f[i] * scale;
		}
		return ndBrainFloatTileVector(tmp);
	}

#endif

class ndBrainLayerFeedForwardCpuCommand_RowMatrixMultiply : public ndBrainLayerFeedForwardCpuCommand
{
	public:
	ndBrainLayerFeedForwardCpuCommand_RowMatrixMultiply(const ndBrainBufferCommandDesc& desc, ndBrainLayer* const layer)
		:ndBrainLayerFeedForwardCpuCommand(desc, layer)
	{
	}

	virtual void Execute(ndInt32 miniBatchIndex) override
	{
		ndBrainLayerLinear* const layer = (ndBrainLayerLinear*)m_layer;
		layer->MatrixMultiply_RowBased(this, miniBatchIndex);
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
	ndBrainFloat variance = ndBrainFloat(ndSqrt(ndFloat32(2.0f) / ndFloat32(GetInputSize())));
	for (ndInt32 i = ndInt32(m_weights.GetCount() - 1); i >= 0; --i)
	{
		m_weights[i].InitGaussianWeights(variance);
	}
}

void ndBrainLayerLinear::InitWeights_xavier()
{
	m_bias.Set(ndBrainFloat(0.0f));
	ndBrainFloat variance = ndBrainFloat(ndSqrt(ndFloat32(6.0f) / ndFloat32(GetOutputSize() + GetInputSize())));
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

void ndBrainLayerLinear::MatrixMultiply_RowBased(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex)
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainerInference* const trainer = desc.m_owner;

	const ndBrainFloat* const weightAndBias = (ndBrainFloat*)trainer->GetWeightAndBiasBuffer()->GetCpuPtr();
	const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();

	const ndInt32 inputSize = info.m_inputSize;
	const ndInt32 outputSize = info.m_outputSize;
	const ndInt32 inputOutputSize = info.m_inputOutputSize;
	const ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	const ndInt32 matrixSize = width * height;
	const ndBrainMemVector parameters(&weightAndBias[info.m_parametersStartOffset], matrixSize + outputSize);

	const ndInt64 inputOffset = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	const ndInt64 outputOffset = inputOffset + trainer->RoundOffOffset(inputSize);
	ndBrainMemVector output(&inputOutputBuffer[outputOffset], outputSize);

#if 0
	const ndBrainMemVector input(&inputOutputBuffer[inputOffset], inputSize);
	for (ndInt32 i = outputSize - 1; i >= 0; --i)
	{
		const ndBrainMemVector row(&parameters[i * width], inputSize);
		output[i] = row.Dot(input);
	}
	const ndBrainMemVector bias(&parameters[matrixSize], outputSize);
	output.Add(bias);

#else
	const ndInt32 numOfSimd = (inputSize + ndBrainLayerLinearTileSize - 1) / ndBrainLayerLinearTileSize;
	const ndBrainFloatTileVector* const inputSimd = (ndBrainFloatTileVector*) &inputOutputBuffer[inputOffset];

	const ndBrainFloatTileVector zero(ndBrainFloat(0.0f));
	const ndBrainMemVector bias(&parameters[matrixSize], outputSize);
	for (ndInt32 i = outputSize - 1; i >= 0; --i)
	{
		ndBrainFloatTileVector sum(zero);
		const ndBrainFloatTileVector* const rowSimd = (ndBrainFloatTileVector*) &parameters[i * width];
		for (ndInt32 j = numOfSimd - 1; j >= 0; --j)
		{
			sum = sum.MulAdd(rowSimd[j], inputSimd[j]);
		}
		const ndBrainFloat element = sum.HorizontalAdd();
		output[i] = bias[i] + element;
	}
#endif
}

void ndBrainLayerLinear::FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	// so far about twice slower.
	// the float16 class, seems it needs to be an intricics implemention

	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainerInference* const trainer = desc.m_owner;
	
	const ndBrainFloat* const weightsAndBias = (ndBrainFloat*)trainer->GetWeightAndBiasBuffer()->GetCpuPtr();
	const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();

	const ndInt32 inputSize = info.m_inputSize;
	const ndInt32 outputSize = info.m_outputSize;
	const ndInt32 inputOutputSize = info.m_inputOutputSize;
	const ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;
	const ndInt32 inputRows = (inputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
	
	//const ndInt32 minibatchBlock = (info.m_matrixDimensionK + ndBrainLayerLinearTileSize - 1) / ndBrainLayerLinearTileSize;
	//const ndInt32 groupId_y = miniBatchIndex / minibatchBlock;
	//const ndInt32 groupId_x = miniBatchIndex - groupId_y * minibatchBlock;
	const ndInt32 roundOutputSize = (outputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
	const ndInt32 minibatchBlock = roundOutputSize / ndBrainLayerLinearTileSize;
	const ndInt32 columnTileIndex = miniBatchIndex / minibatchBlock;
	const ndInt32 rowTileIndex = miniBatchIndex - columnTileIndex * minibatchBlock;

	const ndInt64 parameterBlockBase = rowTileIndex * ndBrainLayerLinearTileSize;
	const ndInt64 parametersStartOffset = parameterBlockBase * inputRows + info.m_parametersStartOffset;

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	const ndInt32 matrixSize = width * height;
	const ndInt64 parametersBiasOffset = parameterBlockBase + info.m_parametersStartOffset + matrixSize;
	ndAssert(parametersBiasOffset > 0);

	// initialice the transpose parametes
	ndInt32 indices[ndBrainLayerLinearTileSize];
	ndInt32 inputIndices[ndBrainLayerLinearTileSize];
	ndBrainFloatTileVector tile_accReg[ndBrainLayerLinearTileSize];
	const ndBrainFloatTileVector biasTranspose(*((ndBrainFloatTileVector*)&weightsAndBias[parametersBiasOffset]));

	const ndInt64 inputOffset = ndBrainLayerLinearTileSize * columnTileIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize; ++i)
	{
		indices[i] = i * ndBrainLayerLinearTileSize;
		inputIndices[i] = ndInt32(inputOffset + i * inputOutputSize);
		tile_accReg[i] = ndBrainFloatTileVector(biasTranspose.m_f[i]);
	}
	const ndBrainFloatTileVector indexArray(indices);
	const ndBrainFloatTileVector inputIndexArray(inputIndices);
	
	for (ndInt32 tile = 0; tile < inputRows; tile += ndBrainLayerLinearTileSize)
	{
#if 0
		// This is the naive tile-by-tile implementation.
		// Regardless of how much it is optimized, the fundamental issue is tile size.
		//
		// With an 8x8 tile, a 128x128 matrix, and a minibatch size of 256,
		// this results in 512 calls to this function.
		// Each call iterates 8 times in the inner loop,
		// so the accumulated function-call overhead becomes significant.
		//
		// In short, an 8x8 tile is simply too small.
		// The maximum tile size I can use is 16, which may still be a limiting factor.

		// load two tiles, the input tile is transposed
		const ndInt64 inputStartOffset = tile + inputOffset;
		const ndInt64 weightOffsetStart = tile + parametersStartOffset;

		for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize; ++i)
		{
			for (ndInt32 j = 0; j < ndBrainLayerLinearTileSize; ++j)
			{
				ndBrainFloat acc = ndBrainFloat(0.0f);
				for (ndInt32 k = 0; k < ndBrainLayerLinearTileSize; ++k)
				{
					ndBrainFloat weight = weightsAndBias[weightOffsetStart + i * inputRows + k];
					ndBrainFloat input = inputOutputBuffer[inputStartOffset + j * inputOutputSize + k];
					acc += weight * input;
				}
				tile_accReg[i].m_f[j] += acc;
			}
		}
#else

		// optimized using row linear scaling and add
		const ndInt64 weightOffsetStart = tile + parametersStartOffset;
		for (ndInt32 j = 0; j < ndBrainLayerLinearTileSize; ++j)
		{
			const ndBrainFloatTileVector input(inputOutputBuffer + tile + j, inputIndexArray);
			for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize; ++i)
			{
				const ndBrainFloat weight = weightsAndBias[weightOffsetStart + i * inputRows + j];
				//tile_accReg[i] = tile_accReg[i] + input.Scale(weight);
				tile_accReg[i] = tile_accReg[i].MulAdd(input, weight);
			}
		}
#endif
	}

	ndBrainFloat* const outputBuffer = (ndBrainFloat*)inputOutputBuffer;
	ndInt64 dstOffset = inputOutputStartOffset + trainer->RoundOffOffset(inputSize);
	dstOffset += rowTileIndex * ndBrainLayerLinearTileSize;
	dstOffset += columnTileIndex * ndInt64(inputOutputSize) * ndBrainLayerLinearTileSize;

	for (ndInt32 i = 0; i < ndBrainLayerLinearTileSize; ++i)
	{
		ndBrainFloatTileVector timeColumn(&tile_accReg[0].m_f[i], indexArray);
		((ndBrainFloatTileVector&)outputBuffer[dstOffset]) = timeColumn;
		dstOffset += inputOutputSize;
	}
}

void ndBrainLayerLinear::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	switch (info.m_matrixDimensionK & (m_dimFactor - 1))
	{
		case m_biasPass:
			BackPropagateBiasGradients(command, miniBatchIndex);
			break;
		case m_biasAddPartialSumPass:
			BackPropagateBiasAddPartialSumGradients(command, miniBatchIndex);
			break;
		case m_biasCachePartialSumPass:
			BackPropagateBiasCachePartialSumGradients(command, miniBatchIndex);
			break;
		case m_weightsPass:
			BackPropagateWeightsGradients(command, miniBatchIndex);
			break;
		case m_inputGradientsPass:
			BackPropagateInputGradients(command, miniBatchIndex);
		default:;
	}
}

void ndBrainLayerLinear::BackPropagateBiasGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndBrainFloat* const partialBiasSumBuffer = (ndBrainFloat*)trainer->GetPartialSumBiasGradientBuffer()->GetCpuPtr();
	const ndBrainFloat* const weightAndBiasGradients = (ndBrainFloat*)trainer->GetWeightAndBiasGradientBuffer()->GetCpuPtr();

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	ndInt32 matrixSize = trainer->RoundOffOffset(width * height);

	ndInt32 outputSize = info.m_outputSize;
	ndBrainMemVector biasRowGradients(&weightAndBiasGradients[info.m_parametersStartOffset + matrixSize], outputSize);
	const ndBrainMemVector outputDerivative(&partialBiasSumBuffer[0], outputSize);
	biasRowGradients.Set(outputDerivative);
}

void ndBrainLayerLinear::BackPropagateBiasCachePartialSumGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndBrainFloat* const inputOutputGradientsBuffer = (ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr();
	const ndBrainFloat* const partialBiasSumBuffer = (ndBrainFloat*)trainer->GetPartialSumBiasGradientBuffer()->GetCpuPtr();

	ndInt32 inputSize = info.m_inputSize;
	ndInt32 outputSize = info.m_outputSize;
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndInt32 alignedOffset = trainer->RoundOffOffset(outputSize);
	const ndInt32 dstOffset = miniBatchIndex * alignedOffset;

	ndInt64 inputGradientOffset = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	ndInt64 outputGradientOffset = inputGradientOffset + trainer->RoundOffOffset(inputSize);

	const ndBrainMemVector srcGradients(&inputOutputGradientsBuffer[outputGradientOffset], outputSize);
	ndBrainMemVector dstBias (&partialBiasSumBuffer[dstOffset], outputSize);
	dstBias.Set(srcGradients);
}

void ndBrainLayerLinear::BackPropagateBiasAddPartialSumGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;	

	const ndInt32 partisionSize = info.m_matrixDimensionK / m_dimFactor;
	const ndInt32 halfPartisionIndex = miniBatchIndex + (partisionSize + 1) / 2;
	if (halfPartisionIndex < partisionSize)
	{
		ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;
		ndBrainFloat* const partialBiasSumBuffer = (ndBrainFloat*)trainer->GetPartialSumBiasGradientBuffer()->GetCpuPtr();

		ndInt32 outputSize = info.m_outputSize;
		ndInt32 alignedOffset = trainer->RoundOffOffset(outputSize);

		ndInt32 dstOffset = miniBatchIndex * alignedOffset;
		ndInt32 srcOffset = halfPartisionIndex * alignedOffset;

		ndBrainMemVector dstGradients(&partialBiasSumBuffer[dstOffset], outputSize);
		const ndBrainMemVector srcGradients(&partialBiasSumBuffer[srcOffset], outputSize);
		dstGradients.Add(srcGradients);
	}
}

void ndBrainLayerLinear::BackPropagateWeightsGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	ndBrainFixSizeVector<1024 * 8> cachedRowGradient;

	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();
	const ndBrainFloat* const inputOutputGradientsBuffer = (ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr();
	ndBrainFloat* const weightAndBiasGradients = (ndBrainFloat*)trainer->GetWeightAndBiasGradientBuffer()->GetCpuPtr();

	ndInt32 inputSize = info.m_inputSize;
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 numberOfRows = info.m_matrixDimensionK / m_dimFactor;

	ndInt64 inputOutputStartOffset = info.m_inputOutputStartOffset;
	ndInt64 dstBase = inputOutputStartOffset + trainer->RoundOffOffset(inputSize);

	cachedRowGradient.SetCount(inputSize);
	cachedRowGradient.Set(ndBrainFloat(0.0f));
	for (ndInt32 row = 0; row < numberOfRows; ++row)
	{
		ndInt64 inputOffset = inputOutputStartOffset + row * inputOutputSize;
		ndInt64 outGradientOffset = dstBase + row * inputOutputSize + miniBatchIndex;
		ndBrainFloat outputDerivative = inputOutputGradientsBuffer[outGradientOffset];
	
		const ndBrainMemVector inputData(&inputOutputBuffer[inputOffset], inputSize);
		cachedRowGradient.ScaleAdd (inputData, outputDerivative);
	}

	// store this weight gradient sum
	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	ndInt64 parametersOffset = info.m_parametersStartOffset + miniBatchIndex * width;
	ndBrainMemVector weightGradientRow(&weightAndBiasGradients[parametersOffset], inputSize);
	weightGradientRow.Set(cachedRowGradient);
}

void ndBrainLayerLinear::BackPropagateInputGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndBrainFloat* const weightAndBias = (ndBrainFloat*)trainer->GetWeightAndBiasBuffer()->GetCpuPtr();
	const ndBrainFloat* const inputOutputGradientsBuffer = (ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr();

	ndInt32 inputSize = info.m_inputSize;
	ndInt32 outputSize = info.m_outputSize;
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	ndInt32 matrixSize = width * height;

	ndInt64 srcBase = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	ndInt64 dstBase = srcBase + trainer->RoundOffOffset(inputSize);
	ndAssert(srcBase >= 0);
	ndAssert(dstBase >= 0);
	
	const ndBrainMemVector outputDerivative(&inputOutputGradientsBuffer[dstBase], outputSize);
	ndBrainMemVector inputDerivative(&inputOutputGradientsBuffer[srcBase], inputSize);
	const ndBrainMemVector weightsMatrix(&weightAndBias[info.m_parametersStartOffset], matrixSize);

	inputDerivative.Set(ndBrainFloat(0.0f));
	for (ndInt32 i = 0; i < outputSize; ++i)
	{
		ndBrainFloat outDerivative = outputDerivative[i];
		const ndBrainMemVector weightsRow(&weightsMatrix[i * width], inputSize);
		inputDerivative.ScaleAdd(weightsRow, outDerivative);
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
		{
			// calculate the input Gradiends
			ndBrainBufferCommandDesc descriptor(MakeBackpropagateDesctriptor(
				owner, context, info, miniBatchSize, m_inputGradientsPass,
				inputOutputData, weightsAndBias,
				inputOutputGradients, weightsAndBiasGradients));
			ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
			commands.PushBack(command);
		}

		{
			// calculate the bias gradient
			ndInt32 size = miniBatchSize;
			ndBrainBufferCommandDesc clearBiasDescriptor(MakeBackpropagateDesctriptor(
				owner, context, info, size, (size * m_dimFactor) + m_biasCachePartialSumPass,
				inputOutputData, weightsAndBias,
				inputOutputGradients, weightsAndBiasGradients));
			ndBrainBufferCommand* const clearBiasCommand = new ndBrainLayerBackPropagateCpuCommand(clearBiasDescriptor, (ndBrainLayer*)this);
			commands.PushBack(clearBiasCommand);

			while (size > 1)
			{
				ndBrainBufferCommandDesc descriptor(MakeBackpropagateDesctriptor(
					owner, context, info, (size + 1)/2, (size * m_dimFactor) + m_biasAddPartialSumPass,
					inputOutputData, weightsAndBias,
					inputOutputGradients, weightsAndBiasGradients));
				ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
				commands.PushBack(command);

				size = (size + 1) / 2;
			}

			ndBrainBufferCommandDesc descriptor(MakeBackpropagateDesctriptor(
				owner, context, info, 1, m_biasPass,
				inputOutputData, weightsAndBias,
				inputOutputGradients, weightsAndBiasGradients));
			ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
			commands.PushBack(command);
		}

		{
			// calculate the weights gradient
			ndBrainBufferCommandDesc descriptor(MakeBackpropagateDesctriptor(
				owner, context, info, info.m_outputSize, (miniBatchSize * m_dimFactor) + m_weightsPass,
				inputOutputData, weightsAndBias,
				inputOutputGradients, weightsAndBiasGradients));
			ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
			commands.PushBack(command);
		}
	}
	else
	{
		ndInt32 id = 0;
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
			inputGradDescriptor.m_id += id++;
			inputGradDescriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixBackPropagateInputGradients;
			ndBrainBufferCommand* const inputGradientCommand = new ndBrainGpuCommand(inputGradDescriptor);
			commands.PushBack(inputGradientCommand);
		}

		{
			ndInt32 size = miniBatchSize;
			ndBrainTrainer* const trainer = (ndBrainTrainer*)owner;
			ndBrainFloatBuffer* const partialBiasSumBuffer = trainer->GetPartialSumBiasGradientBuffer();

			// init bias cradient cache buffer
			ndBrainBufferCommandDesc clearBiasDescriptor(
				MakeBackpropagateDesctriptor(
					owner, context, info, size, (size * m_dimFactor) + m_biasCachePartialSumPass,
					inputOutputData, partialBiasSumBuffer,
					inputOutputGradients, weightsAndBiasGradients));
			clearBiasDescriptor.m_id += id++;
			clearBiasDescriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixBackPropagateClearBiasGradients;
			ndBrainBufferCommand* const clearBiasCommand = new ndBrainGpuCommand(clearBiasDescriptor);
			commands.PushBack(clearBiasCommand);

			while (size > 1)
			{
				ndBrainBufferCommandDesc descriptor(
					MakeBackpropagateDesctriptor(
						owner, context, info, (size + 1) /2, size,
						inputOutputData, partialBiasSumBuffer,
						inputOutputGradients, weightsAndBiasGradients));
				descriptor.m_id += id++;
				descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixBackPropagateAddBiasGradients;

				ndBrainBufferCommand* const command = new ndBrainGpuCommand(descriptor);
				commands.PushBack(command);

				size = (size + 1) / 2;
			}

			// add the bias gradient kernel;
			ndCommandSharedInfo biasInfo(info);
			ndBrainBufferCommandDesc biasDescriptor(
				MakeBackpropagateDesctriptor(
					owner, context, biasInfo, 1, 0,
					inputOutputData, partialBiasSumBuffer,
					inputOutputGradients, weightsAndBiasGradients));
			biasDescriptor.m_id += id++;
			biasDescriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixBackPropagateBiasGradients;
			ndBrainBufferCommand* const biasCommand = new ndBrainGpuCommand(biasDescriptor);
			commands.PushBack(biasCommand);
		}

		{
			// add the weight gradient kernel;
			ndCommandSharedInfo weightsInfo(info);
			ndBrainBufferCommandDesc weightsDescriptor(
				MakeBackpropagateDesctriptor(
					owner, context, weightsInfo, info.m_outputSize, miniBatchSize,
					inputOutputData, weightsAndBias,
					inputOutputGradients, weightsAndBiasGradients));
			weightsDescriptor.m_id += id++;
			weightsDescriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixBackPropagateWeightGradients;
			ndBrainBufferCommand* const weightsCommand = new ndBrainGpuCommand(weightsDescriptor);
			commands.PushBack(weightsCommand);
		}
	}
	return commands;
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
		ndInt32 rows = GetOutputSize();
		ndInt32 columns = GetInputSize();

		CalculateRoundedSize(columns, rows);

		ndAssert((miniBatchSize & (ndBrainLayerLinearTileSize - 1)) == 0);
		ndInt32 rowDim = rows / ndBrainLayerLinearTileSize;
		ndInt32 columnDim = miniBatchSize / ndBrainLayerLinearTileSize;

		ndBrainBufferCommandDesc tileDescriptor(MakeFeedForwardDesctriptor(
			owner, context, info, columnDim * rowDim, miniBatchSize, inputOutputData, weightsAndBias));

		ndBrainBufferCommand* const tiledCommand = new ndBrainLayerFeedForwardCpuCommand(tileDescriptor, (ndBrainLayer*)this);
		commandArray.PushBack(tiledCommand);
#else
		// create a row based matrix multiply command buffer
		ndBrainBufferCommandDesc rowDescriptor(MakeFeedForwardDesctriptor(
			owner, context, info, miniBatchSize, 0,
			inputOutputData, weightsAndBias));
		ndBrainBufferCommand* const rowCommand = new ndBrainLayerFeedForwardCpuCommand_RowMatrixMultiply(rowDescriptor, (ndBrainLayer*)this);
		commandArray.PushBack(rowCommand);
#endif
	}
	else
	{
		ndInt32 width;
		ndInt32 height;
		CalculateRoundedSize(width, height);
		ndAssert((miniBatchSize & (ND_GPU_TILED_MATRIX_ROWS - 1)) == 0);

		ndInt32 dim_M = height / ND_GPU_TILED_MATRIX_ROWS;
		ndInt32 dim_N = miniBatchSize / ND_GPU_TILED_MATRIX_ROWS;

		ndBrainBufferCommandDesc descriptor(MakeFeedForwardDesctriptor(
			owner, context, info, dim_M * dim_N, 0,
			inputOutputData, weightsAndBias));
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixMatrixMultiply;
		ndBrainBufferCommand* const command = new ndBrainGpuCommand(descriptor);
		commandArray.PushBack(command);
	}

	return commandArray;
}
