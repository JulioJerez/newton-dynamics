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

#ifndef _ND_BRAIN_LAYER_ACTIVATION_LINEAR_NORMALIZE_H__
#define _ND_BRAIN_LAYER_ACTIVATION_LINEAR_NORMALIZE_H__

#include "ndBrainStdafx.h"
#include "ndBrainLayerActivationLinear.h"

#define ND_LINEAR_NORMALIZE_MOVING_AVERAGE	ndBrainFloat(0.95f)
#define ND_LINEAR_NORMALIZE_START_NORMALIZE	ndInt32 (1024 * 32)
#define ND_BRAIN_LAYER_ACTIVATION_LINEAR_NORMALIZE_NAME	"ndBrainLayerActivationLinearNormalize"

class ndBrainLayerActivationLinearNormalize : public ndBrainLayerActivationLinear
{
	public:
	ndBrainLayerActivationLinearNormalize(ndInt32 newrons);
	ndBrainLayerActivationLinearNormalize(const ndBrainLayerActivationLinearNormalize& src);
	ndBrainLayer* Clone() const override;

	void UpdateParameters(const ndBrainVector& parameters);

	//virtual void Save(const ndBrainSave* const loadSave) const override;
	static ndBrainLayer* Load(const ndBrainLoad* const loadSave);

	const char* GetLabelId() const override;
	void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const override;

	//virtual bool HasGpuSupport() const override;
	//void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const override;

	virtual void FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const override;
	virtual void BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const override;

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

	ndBrainVector m_minAverage;
	ndBrainVector m_maxAverage;
	ndInt32 m_startNormalizing;
};

#endif 

