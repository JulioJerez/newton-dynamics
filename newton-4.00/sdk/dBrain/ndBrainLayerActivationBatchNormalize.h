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

#ifndef _ND_BRAIN_LAYER_BATCH_NORMALIZE_ACTIVATION_H__
#define _ND_BRAIN_LAYER_BATCH_NORMALIZE_ACTIVATION_H__

#include "ndBrainStdafx.h"
#include "ndBrainLayerActivation.h"
#include "ndBrainLayerActivationLinear.h"

#define ND_BRAIN_LAYER_ACTIVATION_BATCH_NORMALIZE_NAME	"ndBrainLayerActivationBatchNormalize"

//#define ND_LEAR_BATCH_NOMALIZED_PARAMS

#ifdef ND_LEAR_BATCH_NOMALIZED_PARAMS
class ndBrainLayerActivationBatchNormalize : public ndBrainLayerActivation
{
	public:
	ndBrainLayerActivationBatchNormalize(ndInt32 neurons);
	ndBrainLayerActivationBatchNormalize(const ndBrainLayerActivationBatchNormalize& src);
	ndBrainLayer* Clone() const override;
	static ndBrainLayer* Load(const ndBrainLoad* const loadSave);
	virtual void Save(const ndBrainSave* const loadSave) const override;

	const char* GetLabelId() const override;
	void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const override;
	void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const override;

	virtual bool HasGpuSupport() const override;
	virtual void FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const override;
	virtual void BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const override;

	virtual void SetWeights(ndBrainTrainerInference* const trainer, const ndBrainVector& input) override;
	virtual void CopyWeights(ndBrainTrainerInference* const trainer, ndBrainVector& output) const override;
	virtual ndCommandSharedInfo GetCommandSharedInfo(ndBrainTrainerInference* const trainer) const override;

	void CalculateMeanAndVariance(const ndBrainLayerFeedForwardCpuCommand* const command) const;
	void NormalizeMeanAndVariance(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const;

	virtual ndCommandArray CreateFeedForwardBufferCommand(
		ndBrainTrainerInference* const owner,
		ndBrainContext* const context,
		const ndCommandSharedInfo& info,
		ndInt32 miniBatchSize,
		ndBrainFloatBuffer* const inputOutputData,
		ndBrainFloatBuffer* const weightsAndBias) const override;

	virtual ndCommandArray CreateBackPropagateBufferCommand(
		ndBrainTrainerInference* const owner,
		ndBrainContext* const context, 
		const ndCommandSharedInfo& info,
		ndInt32 miniBatchSize,
		ndBrainFloatBuffer* const inputOutputData,
		ndBrainFloatBuffer* const weightsAndBias,
		ndBrainFloatBuffer* const inputOutputGradients,
		ndBrainFloatBuffer* const weightsAndBiasGradients) const override;

	ndBrainVector m_bias;
	ndBrainVector m_scale;
	mutable ndBrainVector m_trainingBias;
	mutable ndBrainVector m_trainingScale;
	mutable ndBrainVector m_inferenceBias;
	mutable ndBrainVector m_inferenceScale;
};
#else

class ndBrainLayerActivationBatchNormalize : public ndBrainLayerActivationLinear
{
	public:
	ndBrainLayerActivationBatchNormalize(ndInt32 neurons);
	ndBrainLayerActivationBatchNormalize(const ndBrainLayerActivationBatchNormalize& src);
	ndBrainLayer* Clone() const override;
	static ndBrainLayer* Load(const ndBrainLoad* const loadSave);
	virtual void Save(const ndBrainSave* const loadSave) const override;
	virtual const char* GetLabelId() const override;

	void CalculateVariance(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const;
	virtual void FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const override;

	virtual ndCommandArray CreateFeedForwardBufferCommand(
		ndBrainTrainerInference* const owner,
		ndBrainContext* const context,
		const ndCommandSharedInfo& info,
		ndInt32 miniBatchSize,
		ndBrainFloatBuffer* const inputOutputData,
		ndBrainFloatBuffer* const weightsAndBias) const override;
};
#endif

#endif 

