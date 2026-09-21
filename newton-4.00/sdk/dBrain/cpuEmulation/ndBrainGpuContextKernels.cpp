/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndBrainStdafx.h"
#include "ndBrainLayer.h"
#include "ndBrainKernel.h"
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuContext.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainUniformBuffer.h"
#include "ndBrainIntegerBuffer.h"
#include "ndBrainOptimizerAdam.h"
#include "ndBrainLayerActivationBatchNormalize.h"

#define ND_GPU_LOCAL_BUFFER_SIZE	    1024 * 4

#define ND_GPU_LEAKY_LRU_GRADIENT		ndBrainFloat(0.01f)

inline ndInt32 __cpuKernelRoundoff(ndInt32 value, ndInt32 workgroupSize)
{
    return (value + workgroupSize - 1) & -workgroupSize;
}


class brainCopyInput : public ndBrainKernel
{
    public:
    brainCopyInput(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
       ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
       ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
       ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];

       ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        ndBrainMemVector inputOutputData ((ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr(), ndInt32(buffer1->SizeInItems()));
        const ndBrainMemVector inputBuffer ((ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr(), ndInt32(buffer2->SizeInItems()));

        ndInt64 srcBase = groupId * ndInt64(inputSize);
        ndInt64 dstBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);
        
        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat a = inputBuffer[srcBase + i + itemId];
                inputOutputData[dstBase + i + itemId] = a;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat a = inputBuffer[srcBase + modWorkGroupSize + itemId];
            inputOutputData[dstBase + modWorkGroupSize + itemId] = a;
        }
        //ndAssert(inputOutputData.SanityCheck());
    }
};

class brainCopyOutput : public ndBrainKernel
{
    public:
    brainCopyOutput(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];

        //ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        //ndBrainFloat* const outputBuffer = (ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 outputSize = parameters->m_outputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        const ndBrainMemVector inputOutputData((ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr(), ndInt32(buffer1->SizeInItems()));
        ndBrainMemVector outputBuffer((ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr(), ndInt32(buffer2->SizeInItems()));
            
        ndInt64 dstBase = groupId * (ndInt64)outputSize;
        ndInt64 srcBase = groupId * (ndInt64)inputOutputSize + inputOutputStartOffset;
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);
            
        ndInt32 workGroupSizeReminder = outputSize % workGroupSize;
        ndInt32 modWorkGroupSize = outputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat a = inputOutputData[srcBase + i + itemId];
                outputBuffer[dstBase + i + itemId] = a;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat a = inputOutputData[srcBase + modWorkGroupSize + itemId];
            outputBuffer[dstBase + modWorkGroupSize + itemId] = a;
        }
        //ndAssert(outputBuffer.SanityCheck());
    }
};

class brainLayerReluActivation : public ndBrainKernel
{
    public:
    brainLayerReluActivation(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];

        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        ndBrainMemVector inputOutputData((ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr(), ndInt32(buffer1->SizeInItems()));
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 inputOffset = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 outputOffset = inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(outputOffset >= 0);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = inputOutputData[inputOffset + i + itemId];
                ndBrainFloat outputValue = (inputValue >= ndBrainFloat(0.0f)) ? inputValue : ndBrainFloat(0.0f);
                inputOutputData[outputOffset + i + itemId] = outputValue;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            ndBrainFloat outputValue = (inputValue >= ndBrainFloat(0.0f)) ? inputValue : ndBrainFloat(0.0f);
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
        //ndAssert(inputOutputData.SanityCheck());
    }
};

class brainLayerLeakyReluActivation : public ndBrainKernel
{
    public:
    brainLayerLeakyReluActivation(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];

        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        ndBrainMemVector inputOutputData((ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr(), ndInt32(buffer1->SizeInItems()));

        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        ndInt64 inputOffset = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 outputOffset = inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(outputOffset >= 0);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = inputOutputData[inputOffset + i + itemId];
                ndBrainFloat outputValue = (inputValue >= ndBrainFloat(0.0f)) ? inputValue : ND_GPU_LEAKY_LRU_GRADIENT * inputValue;
                inputOutputData[outputOffset + i + itemId] = outputValue;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            ndBrainFloat outputValue = (inputValue >= ndBrainFloat(0.0f)) ? inputValue : ND_GPU_LEAKY_LRU_GRADIENT * inputValue;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
        //ndAssert(inputOutputData.SanityCheck());
    }
};

class brainLayerTanhActivation : public ndBrainKernel
{
    public:
    brainLayerTanhActivation(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        ndBrainMemVector inputOutputData((ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr(), ndInt32(buffer1->SizeInItems()));

        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 inputOffset = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 outputOffset = inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(outputOffset >= 0);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;

        const ndBrainMemVector srcData(&inputOutputData[inputOffset], inputSize);
        ndBrainMemVector dstData(&inputOutputData[outputOffset], inputSize);

        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = srcData[i + itemId];
                ndBrainFloat outputValue = (inputValue > ndBrainFloat(-30.0f)) ? ((inputValue < ndBrainFloat(30.0f)) ? inputValue : ndBrainFloat(30.0f)) : ndBrainFloat(-30.0f);
                dstData[i + itemId] = ndBrainFloat(ndTanh(outputValue));
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = srcData[modWorkGroupSize + itemId];
            ndBrainFloat outputValue = (inputValue > ndBrainFloat (-30.0f)) ? ((inputValue < ndBrainFloat(30.0f)) ? inputValue : ndBrainFloat(30.0f)) : ndBrainFloat (-30.0f);
            dstData[modWorkGroupSize + itemId] = ndBrainFloat(ndTanh(outputValue));
        }
#ifdef _DEBUG
        {
            ndInt32 padded = (inputSize + workGroupSize - 1) & -workGroupSize;
            const ndBrainMemVector checkData(&inputOutputData[outputOffset], workGroupSize);
            for (ndInt32 i = inputSize; i < padded; ++i)
            {
                ndBrainFloat a = checkData[modWorkGroupSize + i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
#endif

        //ndAssert(srcData.SanityCheck());
        //ndAssert(dstData.SanityCheck());
    }
};

class brainLayerLinearDropOutActivation : public ndBrainKernel
{
    public:
    brainLayerLinearDropOutActivation(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        ndInt64 inputOffset = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 outputOffset = inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(outputOffset >= 0);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = inputOutputData[inputOffset + i + itemId];
                ndBrainFloat outputValue = inputValue;
                inputOutputData[outputOffset + i + itemId] = outputValue;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            ndBrainFloat outputValue = inputValue;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
    }
};

class brainLayerLinearActivation : public ndBrainKernel
{
    public:
    brainLayerLinearActivation(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainFloatBuffer* const buffer4 = (ndBrainFloatBuffer*)m_parameters[4];

        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        ndBrainFloat* const inputOutputDataPtr = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        const ndBrainFloat* const biasDataPtr = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        const ndBrainFloat* const slopesDataPtr = (ndBrainFloat*)buffer4->GetGpuBuffer()->GetPtr();
        
        ndInt64 inputOffset = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 outputOffset = inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(outputOffset >= 0);

        const ndBrainMemVector biasPtr(biasDataPtr, inputSize);
        const ndBrainMemVector slopesPtr(slopesDataPtr, inputSize);
        ndBrainMemVector input(&inputOutputDataPtr[inputOffset], inputSize);
        ndBrainMemVector output(&inputOutputDataPtr[outputOffset], inputSize);
        
        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat bias = biasPtr[i + itemId];
                ndBrainFloat slope = slopesPtr[i + itemId];
                ndBrainFloat inputValue = input[i + itemId];
                ndBrainFloat outputValue = bias + slope * inputValue;
                output[i + itemId] = outputValue;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat bias = biasPtr[modWorkGroupSize + itemId];
            ndBrainFloat slope = slopesPtr[modWorkGroupSize + itemId];
            ndBrainFloat inputValue = input[modWorkGroupSize + itemId];

            ndBrainFloat outputValue = bias + slope * inputValue;
            output[modWorkGroupSize + itemId] = outputValue;
        }
        //ndAssert(inputOutputData.SanityCheck());

#ifdef _DEBUG
        {
            ndInt32 padded = (inputSize + workGroupSize - 1) & -workGroupSize;
            const ndBrainMemVector checkData(&inputOutputDataPtr[outputOffset], workGroupSize);
            for (ndInt32 i = inputSize; i < padded; ++i)
            {
                ndBrainFloat a = checkData[modWorkGroupSize + i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
#endif

    }
};

class brainLayerBatchNormalizationActivationInputSqr : public ndBrainKernel
{
    public:
       brainLayerBatchNormalizationActivationInputSqr(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;

        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer5 = (ndBrainFloatBuffer*)m_parameters[5];

        ndBrainFloat* const dstData = (ndBrainFloat*)buffer5->GetGpuBuffer()->GetPtr();
        const ndBrainFloat* const srcData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        
        ndBrainMemVector slopesBuffer(&dstData[inputSize * groupId], inputSize);
        const ndBrainMemVector inputOutputBuffer(&srcData[parameters->m_inputOutputStartOffset + inputOutputSize * groupId], inputSize);
        slopesBuffer.Set(inputOutputBuffer);
        slopesBuffer.Mul(slopesBuffer);
        //ndAssert(slopesBuffer.SanityCheck(ndBrainFloat(1.0e4f)));
    }
};

class brainLayerBatchNormalizationActivationVarianceSum : public ndBrainKernel
{
    public:
    brainLayerBatchNormalizationActivationVarianceSum(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        ndInt32 inputSize = parameters->m_inputSize;

        ndBrainFloatBuffer* const buffer5 = (ndBrainFloatBuffer*)m_parameters[5];
        ndBrainFloat* const data = (ndBrainFloat*)buffer5->GetGpuBuffer()->GetPtr();

        ndBrainMemVector dstBuffer(&data[inputSize * groupId], inputSize);
        const ndBrainMemVector srcBuffer(&data[inputSize * (groupId + workGroupSize)], inputSize);
        dstBuffer.Add(srcBuffer);
        //ndAssert(dstBuffer.SanityCheck(ndBrainFloat(1.0e4f)));
    }
};

class brainLayerBatchNormalizationActivationNormalize : public ndBrainKernel
{
    public:
    brainLayerBatchNormalizationActivationNormalize(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        const ndBrainFloatBuffer* const buffer4 = (ndBrainFloatBuffer*)m_parameters[4];
        const ndBrainFloatBuffer* const buffer5 = (ndBrainFloatBuffer*)m_parameters[5];

        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

        ndInt32 inputSize = parameters->m_inputSize;

        ndBrainFloat* const dstData = (ndBrainFloat*)buffer4->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const srcData = (ndBrainFloat*)buffer5->GetGpuBuffer()->GetPtr();

        ndBrainMemVector slopesBuffer(dstData, inputSize);
        ndBrainMemVector varianceBuffer(srcData, inputSize);

        ndBrainFloat den = ndBrainFloat(1.0f) / ndBrainFloat(workGroupSize);
        varianceBuffer.Scale(den);
        varianceBuffer.Max(ndBrainFloat(1.0e-12f));
        varianceBuffer.Sqrt();
        ndBrainFixSizeVector<1024> tmp(inputSize);
        tmp.Reciprocal(slopesBuffer);
        
        tmp.Blend(varianceBuffer, ND_BRAIN_LAYER_ACTIVATION_BATCH_NORMALIZE_BLEND);
        slopesBuffer.Reciprocal(tmp);
        //ndAssert(slopesBuffer.SanityCheck());
    }
};

class brainLayerSoftmaxActivation : public ndBrainKernel
{
    public:
    brainLayerSoftmaxActivation(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndFixSizeArray<ndBrainFloat, 1024> reductionBuffer(1024);
        ndFixSizeArray<ndBrainFloat, ND_GPU_LOCAL_BUFFER_SIZE> tmpInputBuffer(ND_GPU_LOCAL_BUFFER_SIZE);

        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];

        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        ndAssert(inputSize <= tmpInputBuffer.GetCount());
        
        ndInt64 inputOffset = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 outputOffset = inputOffset + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(outputOffset >= 0);

        ndFixSizeArray<ndBrainFloat, 1024> maxArgReg(1024);
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            maxArgReg[itemId] = ndBrainFloat(-1.0e30f);
        }

        const ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        const ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = inputOutputData[inputOffset + i + itemId];
                tmpInputBuffer[i + itemId] = inputValue;
                maxArgReg[itemId] = (inputValue > maxArgReg[itemId]) ? inputValue : maxArgReg[itemId];
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = inputOutputData[inputOffset + modWorkGroupSize + itemId];
            tmpInputBuffer[modWorkGroupSize + itemId] = inputValue;
            maxArgReg[itemId] = (inputValue > maxArgReg[itemId]) ? inputValue : maxArgReg[itemId];
        }

        for (ndInt32 j = workGroupSize / 2; j > 0; j = j >> 1)
        {
            for (ndInt32 itemId = 0; itemId < j; ++itemId)
            {
                reductionBuffer[itemId] = maxArgReg[itemId + j];
            }
            // barrier
            for (ndInt32 itemId = 0; itemId < j; ++itemId)
            {
                ndBrainFloat inputValue = reductionBuffer[itemId];
                maxArgReg[itemId] = (inputValue > maxArgReg[itemId]) ? inputValue : maxArgReg[itemId];
            }
            // barrier
        }
        reductionBuffer[0] = maxArgReg[0];
        
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            maxArgReg[itemId] = reductionBuffer[0];
        }

        ndFixSizeArray<ndBrainFloat, 1024> sumArgReg(1024);
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            sumArgReg[itemId] = ndBrainFloat(0.0f);
        }
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = tmpInputBuffer[i + itemId] - maxArgReg[itemId];
                ndBrainFloat outputValue = ndBrainFloat(ndExp(inputValue));
                sumArgReg[itemId] += outputValue;
                tmpInputBuffer[i + itemId] = outputValue;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = tmpInputBuffer[modWorkGroupSize + itemId] - maxArgReg[itemId];
            ndBrainFloat outputValue = ndBrainFloat(ndExp(inputValue));
            sumArgReg[itemId] += outputValue;
            tmpInputBuffer[modWorkGroupSize + itemId] = outputValue;
        }
        for (ndInt32 j = workGroupSize / 2; j > 0; j = j >> 1)
        {
            for (ndInt32 itemId = j; itemId < j * 2; ++itemId)
            {
                reductionBuffer[itemId - j] = sumArgReg[itemId];
            }
        
            for (ndInt32 itemId = 0; itemId < j; ++itemId)
            {
                ndBrainFloat inputValue = reductionBuffer[itemId];
                sumArgReg[itemId] += inputValue;
            }
        }
        reductionBuffer[0] = ndBrainFloat(1.0f) / sumArgReg[0];
        
        ndBrainFloat invDen = reductionBuffer[0];
        ndAssert(ndCheckFloat(invDen));
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inputValue = tmpInputBuffer[i + itemId];
                ndAssert(ndCheckFloat(inputValue));
                ndBrainFloat outputValue = invDen * inputValue;
                inputOutputData[outputOffset + i + itemId] = outputValue;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inputValue = tmpInputBuffer[modWorkGroupSize + itemId];
            ndAssert(ndCheckFloat(inputValue));
            ndBrainFloat outputValue = invDen * inputValue;
            inputOutputData[outputOffset + modWorkGroupSize + itemId] = outputValue;
        }
    }
};

// back propagation kernels
class brainCopyInputGradients : public ndBrainKernel
{
    public:
    brainCopyInputGradients(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];

        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

        //ndBrainFloat* const miniBatchGradients = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        //ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
        ndBrainMemVector miniBatchGradients((ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr(), ndInt32(buffer1->SizeInItems()));
        const ndBrainMemVector inputOutputGradients((ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr(), ndInt32(buffer2->SizeInItems()));

        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 dstBase = groupId * ndInt64(inputSize);
        ndInt64 srcBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);
        
        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat a = inputOutputGradients[srcBase + i + itemId];
                miniBatchGradients[dstBase + i + itemId] = a;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat a = inputOutputGradients[srcBase + modWorkGroupSize + itemId];
            miniBatchGradients[dstBase + modWorkGroupSize + itemId] = a;
        }
    }
};

class brainCopyOutputGradients : public ndBrainKernel
{
    public:
    brainCopyOutputGradients(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];

        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

        const ndBrainMemVector miniBatchGradients((ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr(), ndInt32(buffer1->SizeInItems()));
        ndBrainMemVector inputOutputGradients((ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr(), ndInt32(buffer2->SizeInItems()));

        ndInt32 outputSize = parameters->m_outputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 srcBase = groupId * ndInt64(outputSize);
        ndInt64 dstBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);
        
        ndInt32 workGroupSizeReminder = outputSize % workGroupSize;
        ndInt32 modWorkGroupSize = outputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat a = miniBatchGradients[srcBase + i + itemId];
                inputOutputGradients[dstBase + i + itemId] = a;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat a = miniBatchGradients[srcBase + modWorkGroupSize + itemId];
            inputOutputGradients[dstBase + modWorkGroupSize + itemId] = a;
        }
        //ndAssert(inputOutputGradients.SanityCheck());
    }
};

class brainLayerBrainReluBackPropagate : public ndBrainKernel
{
    public:
    brainLayerBrainReluBackPropagate(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];

        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 srcBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);
        
        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inpuData = inputOutputData[srcBase + i + itemId];
                ndBrainFloat gradient = (inpuData >= ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
                ndBrainFloat outputGrad = inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = gradient * outputGrad;
            }
        }
        
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inpuData = inputOutputData[srcBase + modWorkGroupSize + itemId];
            ndBrainFloat gradient = (inpuData >= ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
            ndBrainFloat outputGrad = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = gradient * outputGrad;
        }

        #ifdef _DEBUG
        {
            ndInt32 padded = (inputSize + workGroupSize - 1) & -workGroupSize;
            for (ndInt32 i = inputSize; i < padded; ++i)
            {
                ndBrainFloat a = inputOutputGradients[srcBase + i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
        #endif
    }
};

class brainLayerBrainLeakyReluBackPropagate : public ndBrainKernel
{
    public:
    brainLayerBrainLeakyReluBackPropagate(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];

        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;

        const ndBrainMemVector inputOutputData((ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr(), ndInt32(buffer1->SizeInItems()));
        ndBrainMemVector inputOutputGradients ((ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr(), ndInt32(buffer3->SizeInItems()));

        ndInt64 srcBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat inpuData = inputOutputData[srcBase + i + itemId];
                ndBrainFloat gradient = (inpuData >= ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ND_GPU_LEAKY_LRU_GRADIENT;
                ndBrainFloat outputGrad = inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = gradient * outputGrad;
            }
        }

        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat inpuData = inputOutputData[srcBase + modWorkGroupSize + itemId];
            ndBrainFloat gradient = (inpuData >= ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ND_GPU_LEAKY_LRU_GRADIENT;
            ndBrainFloat outputGrad = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = gradient * outputGrad;
        }

        #ifdef _DEBUG
        {
            const ndBrainMemVector checkPadding(&inputOutputGradients[srcBase], workGroupSize);
            ndInt32 padded = (inputSize + workGroupSize - 1) & -workGroupSize;
            for (ndInt32 i = inputSize; i < padded; ++i)
            {
                ndBrainFloat a = checkPadding[i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
        #endif
        //ndAssert(inputOutputGradients.SanityCheck());
    }
};

class brainLayerBrainTanhBackPropagate : public ndBrainKernel
{
    public:
    brainLayerBrainTanhBackPropagate(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 srcBase = groupId  * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat outputData = inputOutputData[dstBase + i + itemId];
                ndBrainFloat a = ndBrainFloat(1.0f) - outputData * outputData;
                ndBrainFloat b = inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = a * b;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat outputData = inputOutputData[dstBase + modWorkGroupSize + itemId];
            ndBrainFloat a = ndBrainFloat(1.0f) - outputData * outputData;
            ndBrainFloat b = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = a * b;
        }

        #ifdef _DEBUG
        {
            ndInt32 padded = (inputSize + workGroupSize - 1) & -workGroupSize;
            for (ndInt32 i = inputSize; i < padded; ++i)
            {
                ndBrainFloat a = inputOutputGradients[srcBase + i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
        #endif
    }
};

class brainLayerBrainCathegoricalSoftmaxBackPropagate : public ndBrainKernel
{
    public:
    brainLayerBrainCathegoricalSoftmaxBackPropagate(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 srcBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);
        
        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat a = inputOutputData[dstBase + i + itemId];
                a -= inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = a;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat a = inputOutputData[dstBase + modWorkGroupSize + itemId];
            a -= inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = a;
        }

        #ifdef _DEBUG
        {
            ndInt32 padded = (inputSize + workGroupSize - 1) & -workGroupSize;
            for (ndInt32 i = inputSize; i < padded; ++i)
            {
                ndBrainFloat a = inputOutputGradients[srcBase + i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
        #endif
    }
};

class brainLayerBrainLinearDropOutBackPropagate : public ndBrainKernel
{
    public:
    brainLayerBrainLinearDropOutBackPropagate(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 srcBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);

        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat outputGrad = inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = outputGrad;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat outputGrad = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = outputGrad;
        }

        #ifdef _DEBUG
        {
            ndInt32 padded = (inputSize + workGroupSize - 1) & -workGroupSize;
            for (ndInt32 i = inputSize; i < padded; ++i)
            {
                ndBrainFloat a = inputOutputGradients[srcBase + i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
        #endif
    }
};

class brainLayerBrainLinearBackPropagate : public ndBrainKernel
{
    public:
    brainLayerBrainLinearBackPropagate(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainUniformBuffer* const buffer5 = (ndBrainUniformBuffer*)m_parameters[5];

        ndBrainFloat* const slopesPtr = (ndBrainFloat*)buffer5->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 srcBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);
        
        ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat slope = slopesPtr[i + itemId];
                ndBrainFloat outputGrad = inputOutputGradients[dstBase + i + itemId];
                inputOutputGradients[srcBase + i + itemId] = slope * outputGrad;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat slope = slopesPtr[modWorkGroupSize + itemId];
            ndBrainFloat outputGrad = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = slope * outputGrad;
        }
        
        #ifdef _DEBUG
        {
            ndInt32 padded = (inputSize + workGroupSize - 1) & -workGroupSize;
            for (ndInt32 i = inputSize; i < padded; ++i)
            {
                ndBrainFloat a = inputOutputGradients[srcBase + i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
        #endif
    }
};

class brainLayerBrainPolicyGradientBackPropagate : public ndBrainKernel
{
    public:
    brainLayerBrainPolicyGradientBackPropagate(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainFloatBuffer* const buffer5 = (ndBrainFloatBuffer*)m_parameters[5];
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];

        ndBrainFloat* const logVariance = (ndBrainFloat*)buffer5->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputData = (ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr();
        ndBrainFloat* const inputOutputGradients = (ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr();
        ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        
        ndInt32 inputSize = parameters->m_inputSize;
        ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        
        ndInt64 srcBase = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        ndInt64 dstBase = srcBase + __cpuKernelRoundoff(inputSize, workGroupSize);
        ndAssert(srcBase >= 0);
        ndAssert(dstBase >= 0);

        ndBrainFloat logVarianceBias = logVariance[0];
        ndBrainFloat logVarianceSlope = logVariance[1];

        const ndInt32 halfSize = inputSize / 2;
        const ndInt32 workGroupSizeReminder = inputSize % workGroupSize;
        const ndInt32 modWorkGroupSize = inputSize - workGroupSizeReminder;
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat in = inputOutputData[srcBase + i + itemId];
                ndBrainFloat out = inputOutputData[dstBase + i + itemId];
                ndBrainFloat x1 = ndBrainFloat(ndTanh(in));
                ndBrainFloat x2 = logVarianceBias + logVarianceSlope * x1;

                ndBrainFloat meanGrad = ndBrainFloat(1.0f) - out * out;
                ndBrainFloat sigmaGrad = logVarianceSlope * ndBrainFloat(ndExp(x2)) * (ndBrainFloat(1.0f) - x1 * x1);

                ndBrainFloat blend = ((i + itemId) < halfSize) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
                ndBrainFloat gradiend = meanGrad * blend + sigmaGrad * (ndBrainFloat(1.0f) - blend);

                ndBrainFloat inputGradient = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
                inputOutputGradients[srcBase + i + itemId] = gradiend * inputGradient;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat in = inputOutputData[srcBase + modWorkGroupSize + itemId];
            ndBrainFloat out = inputOutputData[dstBase + modWorkGroupSize + itemId];
            ndBrainFloat x1 = ndBrainFloat(ndTanh(in));
            ndBrainFloat x2 = logVarianceBias + logVarianceSlope * x1;

            ndBrainFloat meanGrad = ndBrainFloat(1.0f) - out * out;
            ndBrainFloat sigmaGrad = logVarianceSlope * ndBrainFloat(ndExp(x2)) * (ndBrainFloat(1.0f) - x1 * x1);

            ndBrainFloat blend = ((modWorkGroupSize + itemId) < halfSize) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
            ndBrainFloat gradiend = meanGrad * blend + sigmaGrad * (ndBrainFloat(1.0f) - blend);
          
            ndBrainFloat inputGradient = inputOutputGradients[dstBase + modWorkGroupSize + itemId];
            inputOutputGradients[srcBase + modWorkGroupSize + itemId] = gradiend * inputGradient;
        }

        #ifdef _DEBUG
        {
            ndInt32 padded = (inputSize + workGroupSize - 1) & -workGroupSize;
            for (ndInt32 i = inputSize; i < padded; ++i)
            {
                ndBrainFloat a = inputOutputGradients[srcBase + i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
        #endif
    }
};

class brainAdamBiasCorrectionUpdate : public ndBrainKernel
{
    public:
    brainAdamBiasCorrectionUpdate(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32, ndInt32)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainOptimizerAdam::ndCommandSharedInfo* const parameters = (ndBrainOptimizerAdam::ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

        parameters->m_biasBetaCorrection *= parameters->m_beta;
        parameters->m_biasAlphaCorrection *= parameters->m_alpha;
        if (parameters->m_biasBetaCorrection < ndBrainFloat(1.0e-6f))
        {
            parameters->m_biasBetaCorrection = ndBrainFloat(0.0f);
        }
        if (parameters->m_biasAlphaCorrection < ndBrainFloat(1.0e-6f))
        {
            parameters->m_biasAlphaCorrection = ndBrainFloat(0.0f);
        }

        parameters->m_invBiasBetaCorrection = (ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) - parameters->m_biasBetaCorrection));
        parameters->m_invBiasAlphaCorrection = (ndBrainFloat(1.0f) / (ndBrainFloat(1.0f) - parameters->m_biasAlphaCorrection));
    }
};

class brainAdamUpdateLassoRegularizer : public ndBrainKernel
{
    public:
    brainAdamUpdateLassoRegularizer(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    //void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    void Execute(ndInt32, ndInt32)
    {
        ndAssert(0);
    }
};

class brainAdamUpdateRidgeRegularizer : public ndBrainKernel
{
    public:
    brainAdamUpdateRidgeRegularizer(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainFloatBuffer* const buffer4 = (ndBrainFloatBuffer*)m_parameters[4];
        ndBrainFloat* const buffer5 = (ndBrainFloat*)m_parameters[5];        

        const ndBrainOptimizerAdam::ndCommandSharedInfo* const parameters = (ndBrainOptimizerAdam::ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        const ndInt64 bufferSize = ndInt32(buffer1->SizeInItems());
        ndBrainMemVector weightAndBiasBuffer((ndBrainFloat*)buffer1->GetCpuPtr(), bufferSize);
        ndBrainMemVector vdw((ndBrainFloat*)buffer3->GetCpuPtr(), bufferSize);
        ndBrainMemVector vdw2((ndBrainFloat*)buffer4->GetCpuPtr(), bufferSize);
        const ndBrainMemVector weightAndBiasGradientBuffer((ndBrainFloat*)buffer2->GetCpuPtr(), bufferSize);

        const ndBrainFloat descendRate = -*buffer5;
        const ndBrainFloat regularizer = -parameters->m_decayRegularizer;
        
        const ndInt32 start = groupId * workGroupSize;
        const ndBrainFloat miniBatchWeight = parameters->m_minibathScale;
        for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
        {
            ndBrainFloat m = vdw[start + itemId];
            ndBrainFloat v = vdw2[start + itemId];
            ndBrainFloat g = miniBatchWeight * weightAndBiasGradientBuffer[start + itemId];
        
            // calculate moving average
            m = m * parameters->m_alpha + g * (ndBrainFloat(1.0f) - parameters->m_alpha);
        
            // calculate RMS
            v = v * parameters->m_beta + g * g * (ndBrainFloat(1.0f) - parameters->m_beta);
        
            // save veloc and accel for net update
            vdw[start + itemId] = m;
            vdw2[start + itemId] = v;
            const ndBrainFloat weight = weightAndBiasBuffer[start + itemId];
        
            // apply bias correction, until bias becomes 1.0
            ndBrainFloat vCorrected = v * parameters->m_invBiasBetaCorrection;
            ndBrainFloat mCorrected = m * parameters->m_invBiasAlphaCorrection;
        
            const ndBrainFloat denV = ndBrainFloat(ndSqrt(vCorrected)) + parameters->m_epsilon;
            const ndBrainFloat gradient = weight * regularizer + mCorrected / denV;
            weightAndBiasBuffer[start + itemId] = weight + gradient * descendRate;
        }
    }
};

// matrix vector operation kernels.
class brainLayerMatrixMatrixAddBias : public ndBrainKernel
{
    public:
    brainLayerMatrixMatrixAddBias(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];

        const ndCommandSharedInfo& info = *(ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

        const ndBrainMemVector inputOutputBuffer((ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr(), ndInt32(buffer1->SizeInItems()));
        const ndBrainMemVector weightsAndBias((ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr(), ndInt32(buffer2->SizeInItems()));

        const ndInt32 inputSize = info.m_inputSize;
        const ndInt32 outputSize = info.m_outputSize;
        const ndInt32 inputOutputSize = info.m_inputOutputSize;
        const ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

        const ndInt32 width = (inputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        const ndInt32 height = (outputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        const ndInt32 matrixSize = width * height;
        ndAssert(weightsAndBias.BounceCheck(info.m_parametersStartOffset + matrixSize + outputSize - 1));
        const ndBrainMemVector parameters(&weightsAndBias[info.m_parametersStartOffset], matrixSize + outputSize);

        const ndInt64 inputOffset = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        const ndInt64 outputOffset = inputOffset + ((inputSize + workGroupSize - 1) & -workGroupSize);

        ndAssert(inputOutputBuffer.BounceCheck(outputOffset + outputSize - 1));
        ndBrainMemVector output(&inputOutputBuffer[outputOffset], outputSize);
        const ndBrainMemVector bias(&parameters[matrixSize], outputSize);
        output.Add(bias);
        //ndAssert(output.SanityCheck());

#ifdef _DEBUG
        {
            const ndBrainMemVector checkPadding(&inputOutputBuffer[outputOffset], workGroupSize);
            ndInt32 padded = (outputSize + workGroupSize - 1) & -workGroupSize;
            for (ndInt32 i = outputSize; i < padded; ++i)
            {
                ndBrainFloat a = checkPadding[i];
                ndAssert(a == ndBrainFloat(0.0f));
            }
        }
#endif
    }
};

// matrix vector operation kernels.
class brainLayerMatrixMatrixMultiply : public ndBrainKernel
{
    public:
    brainLayerMatrixMatrixMultiply(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    // tile based matrix x matrix multiplication.
    // partion that matrices A into a (M x K) and matrix B into a (K x N) blocks 
    // them doing the block matrix multiplication. 
    // if tune propertlly, each block can be load into shared memory or lavel ine cache 
    // then do a sub tileA by TileB matrix multiplication in cache memory.
    // this is the trick that specialized, so call AI instruction do in hardware,
    // inpement in speciallized instartion to speed up the opertioan 100s of times, 
    // Unfortunatly OpenCl and my intel cpu do not exposes these instrutions to the user, 
    // maybe one day. for now this will do using avx and cumpute units.
    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
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

        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];

        const ndCommandSharedInfo& info = *(ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

        const ndInt32 inputSize = info.m_inputSize;
        const ndInt32 ouputSize = info.m_outputSize;
        const ndInt32 inputOutputSize = info.m_inputOutputSize;
        const ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

        const ndInt32 width = (inputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        const ndInt32 height = (ouputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        const ndInt32 matrixSize = width * height;

        const ndInt32 kDim = (inputSize + ND_GPU_TILED_MATRIX_ROWS - 1) / ND_GPU_TILED_MATRIX_ROWS;
        const ndInt32 rowStart = groupId / info.m_matrixDimensionK;
        const ndInt32 columStart = groupId - rowStart * info.m_matrixDimensionK;

        const ndBrainFloat* const weightAndBiasPtr = (ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
        const ndBrainMemVector weightsAndBias(&weightAndBiasPtr[info.m_parametersStartOffset], matrixSize);
        const ndBrainMemVector inputBuffer((ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr(), ndInt32(buffer1->SizeInItems()));

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
                    tile_weights[j][i] = weightsAndBias[weightOffset + i];
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
        ndInt64 outputOffset = inputBase + rowStart * ND_GPU_TILED_MATRIX_ROWS + ((inputSize + workGroupSize - 1) & -workGroupSize);
        ndBrainMemVector outputBuffer((ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr(), ndInt32(buffer1->SizeInItems()));
        for (ndInt32 j = 0; j < ND_GPU_TILED_MATRIX_ROWS; ++j)
        {
            for (ndInt32 i = 0; i < ND_GPU_TILED_MATRIX_ROWS; ++i)
            {
                ndBrainFloat acc = tile_inputs[j][i];
                outputBuffer[outputOffset + i] = acc;
            }
            outputOffset += inputOutputSize;
        }
        //ndAssert(outputBuffer.SanityCheck());
    }
};

class brainLayerBrainBackPropagateMatrixInputGradients : public ndBrainKernel
{
    public:
    brainLayerBrainBackPropagateMatrixInputGradients(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];

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

        const ndCommandSharedInfo& info = *(ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        const ndInt32 inputSize = info.m_inputSize;
        const ndInt32 outputSize = info.m_outputSize;
        const ndInt32 inputOutputSize = info.m_inputOutputSize;
        const ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

        const ndInt32 width = (inputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        const ndInt32 height = (outputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        const ndInt32 matrixSize = width * height;

        const ndInt32 kDim = height / ND_GPU_TILED_MATRIX_ROWS;
        const ndInt32 minibatchSize = info.m_matrixDimensionK / ndBrainLayerLinear::m_dimFactor;
        const ndInt32 rowStart = groupId / minibatchSize;
        const ndInt32 columStart = groupId - rowStart * minibatchSize;

        const ndInt32 weightsBase = columStart * ND_GPU_TILED_MATRIX_ROWS;
        const ndBrainFloat* const weightAndBiasPtr = (ndBrainFloat*)(ndBrainFloat*)buffer2->GetGpuBuffer()->GetPtr();
        const ndBrainMemVector weightAndBias(&weightAndBiasPtr[info.m_parametersStartOffset], matrixSize);
        ndBrainMemVector inputOutputGradientsBuffer((ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr(), ndInt32(buffer3->SizeInItems()));

        const ndInt32 inputBase = rowStart * inputOutputSize * ND_GPU_TILED_MATRIX_ROWS + inputOutputStartOffset;
        const ndInt32 outputBase = inputBase + ((inputSize + workGroupSize - 1) & -workGroupSize);

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
};

class brainLayerBrainBackPropagateMatrixBiasGradients : public ndBrainKernel
{
    public:
    brainLayerBrainBackPropagateMatrixBiasGradients(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer3 = (ndBrainFloatBuffer*)m_parameters[3];
        ndBrainFloatBuffer* const buffer4 = (ndBrainFloatBuffer*)m_parameters[4];

        const ndCommandSharedInfo* const parameters = (ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        ndBrainMemVector weightAndBiasGradients((ndBrainFloat*)buffer4->GetGpuBuffer()->GetPtr(), ndInt32(buffer4->SizeInItems()));
        const ndBrainMemVector inputOutputGradientsBuffer((ndBrainFloat*)buffer3->GetGpuBuffer()->GetPtr(), ndInt32(buffer3->SizeInItems()));

        const ndInt32 inputSize = parameters->m_inputSize;
        const ndInt32 outputSize = parameters->m_outputSize;
        const ndInt32 inputOutputSize = parameters->m_inputOutputSize;
        const ndInt32 inputOutputStartOffset = parameters->m_inputOutputStartOffset;
        const ndInt64 inputGradientOffset = groupId * ndInt64(inputOutputSize) + inputOutputStartOffset;
        const ndInt64 outputGradientOffset = inputGradientOffset + __cpuKernelRoundoff(inputSize, workGroupSize);

        const ndInt32 width = (inputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        const ndInt32 height = (outputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;

        const ndInt32 matrixSize = __cpuKernelRoundoff(width * height, ND_DEFAULT_WORKGROUP_SIZE);;
        const ndInt64 parametersStartOffset = ndInt64(parameters->m_parametersStartOffset) + matrixSize;
        const ndInt32 workGroupSizeReminder = outputSize % workGroupSize;
        const ndInt32 modWorkGroupSize = outputSize - workGroupSizeReminder;

        const ndBrainMemVector outputDerivative(&inputOutputGradientsBuffer[outputGradientOffset], outputSize);
        ndBrainMemVector biasRowGradients(&weightAndBiasGradients[parametersStartOffset + parameters->m_parametersBatchSize * groupId], outputSize);
        for (ndInt32 rowBlock = 0; rowBlock < modWorkGroupSize; rowBlock += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndBrainFloat biasDerivative = outputDerivative[rowBlock + itemId];
                biasRowGradients[rowBlock + itemId] = biasDerivative;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndBrainFloat biasDerivative = outputDerivative[modWorkGroupSize + itemId];
            biasRowGradients[modWorkGroupSize + itemId] = biasDerivative;
        }
        //ndAssert(biasRowGradients.SanityCheck());
    }
};

class brainLayerBrainBackPropagateMatrixWeightsGradients : public ndBrainKernel
{
    public:
    brainLayerBrainBackPropagateMatrixWeightsGradients(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer4 = (ndBrainFloatBuffer*)m_parameters[4];

        const ndCommandSharedInfo& info = *(ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();

        const ndInt32 inputSize = info.m_inputSize;
        const ndInt32 outputSize = info.m_outputSize;
        const ndInt32 inputOutputSize = info.m_inputOutputSize;
        const ndInt64 inputOutputStartOffset = info.m_inputOutputStartOffset;

        const ndInt32 width = (inputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        const ndInt32 height = (outputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
        const ndInt32 matrixSize = __cpuKernelRoundoff(width * height, ND_DEFAULT_WORKGROUP_SIZE);

        const ndInt32 dimK = info.m_matrixDimensionK / ndBrainLayerLinear::m_dimFactor;
        const ndInt32 matrixBlock = groupId / dimK;
        const ndInt32 rowBlock = groupId - dimK * matrixBlock;

        const ndInt32 matrixOffsetStart = ndInt32(info.m_parametersStartOffset + info.m_parametersBatchSize * matrixBlock);
        const ndInt64 srcBase = matrixBlock * ndInt64(inputOutputSize) + inputOutputStartOffset;

        const ndBrainMemVector inputOutputBuffer((ndBrainFloat*)buffer1->GetGpuBuffer()->GetPtr(), ndInt32(buffer1->GetCount()));
        ndBrainMemVector weightAndBiasGradients((ndBrainFloat*)buffer4->GetGpuBuffer()->GetPtr(), ndInt32(buffer4->SizeInItems()));

        const ndBrainMemVector inputData(&inputOutputBuffer[srcBase], inputSize);
        const ndBrainMemVector biasRowGradients(&weightAndBiasGradients[info.m_parametersStartOffset + matrixSize + info.m_parametersBatchSize * matrixBlock], outputSize);

        ndAssert(inputOutputBuffer.BounceCheck(srcBase + inputSize - 1));
        ndAssert(weightAndBiasGradients.BounceCheck(info.m_parametersStartOffset + matrixSize + info.m_parametersBatchSize * matrixBlock + outputSize - 1));

        const ndBrainFloat scale = biasRowGradients[rowBlock];
        const ndInt32 matrixOffset = matrixOffsetStart + width * rowBlock;
        ndAssert(weightAndBiasGradients.BounceCheck(matrixOffset + inputSize - 1));
        ndBrainMemVector weightRowGradients(&weightAndBiasGradients[matrixOffset], inputSize);
        weightRowGradients.ScaleSet(inputData, scale);
        //ndAssert(weightRowGradients.SanityCheck());
    }
};

class brainCopyBuffer : public ndBrainKernel
{
    public:
    brainCopyBuffer(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];

        ndUnsigned32* const srcBuffer = (ndUnsigned32*)buffer2->GetGpuBuffer()->GetPtr();
        ndUnsigned32* const dstBuffer = (ndUnsigned32*)buffer1->GetGpuBuffer()->GetPtr();
        ndCopyBufferCommandInfo* const parameters = (ndCopyBufferCommandInfo*)buffer0->GetGpuBuffer()->GetPtr();
        ndAssert((parameters->m_bytesToCopy & (sizeof(ndInt32) - 1)) == 0);

        ndInt32 stride = parameters->m_bytesToCopy / ndInt32(sizeof(ndInt32));
        ndInt32 srcStride = parameters->m_srcStrideInByte / ndInt32(sizeof(ndInt32));
        ndInt32 srcOffset = parameters->m_srcOffsetInByte / ndInt32(sizeof(ndInt32));
        ndInt32 dstStride = parameters->m_dstStrideInByte / ndInt32(sizeof(ndInt32));
        ndInt32 dstOffset = parameters->m_dstOffsetInByte / ndInt32(sizeof(ndInt32));

        ndInt32 workGroupSizeReminder = stride % workGroupSize;
        ndInt32 modWorkGroupSize = stride - workGroupSizeReminder;

        ndInt64 dstBase = ndInt64(dstOffset) + groupId * ndInt64(dstStride);
        ndInt64 srcBase = ndInt64(srcOffset) + groupId * ndInt64(srcStride);
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndUnsigned32 val = srcBuffer[srcBase + i + itemId];
                dstBuffer[dstBase + i + itemId] = val;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndUnsigned32 val = srcBuffer[srcBase + modWorkGroupSize + itemId];
            dstBuffer[dstBase + modWorkGroupSize + itemId] = val;
        }
    }
};

class brainCopyBufferIndirect : public ndBrainKernel
{
    public:
    brainCopyBufferIndirect(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    void Execute(ndInt32 groupId, ndInt32 workGroupSize)
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];
        ndBrainFloatBuffer* const buffer2 = (ndBrainFloatBuffer*)m_parameters[2];
        ndBrainIntegerBuffer* const buffer3 = (ndBrainIntegerBuffer*)m_parameters[3];

        ndUnsigned32* indexArray = (ndUnsigned32*)buffer3->GetGpuBuffer()->GetPtr();
        ndUnsigned32* const srcBuffer = (ndUnsigned32*)buffer2->GetGpuBuffer()->GetPtr();
        ndUnsigned32* const dstBuffer = (ndUnsigned32*)buffer1->GetGpuBuffer()->GetPtr();
        ndCopyBufferCommandInfo* const parameters = (ndCopyBufferCommandInfo*)buffer0->GetGpuBuffer()->GetPtr();
        ndAssert((parameters->m_bytesToCopy & (sizeof(ndInt32) - 1)) == 0);

        ndInt32 stride = parameters->m_bytesToCopy / ndInt32(sizeof(ndInt32));
        ndInt32 srcStride = parameters->m_srcStrideInByte / ndInt32(sizeof(ndInt32));
        ndInt32 srcOffset = parameters->m_srcOffsetInByte / ndInt32(sizeof(ndInt32));
        ndInt32 dstStride = parameters->m_dstStrideInByte / ndInt32(sizeof(ndInt32));
        ndInt32 dstOffset = parameters->m_dstOffsetInByte / ndInt32(sizeof(ndInt32));

        ndInt32 workGroupSizeReminder = stride % workGroupSize;
        ndInt32 modWorkGroupSize = stride - workGroupSizeReminder;

        ndInt64 dstBase = ndInt64(dstOffset) + groupId * ndInt64(dstStride);
        ndInt64 srcBase = ndInt64(srcOffset) + indexArray[groupId] * ndInt64(srcStride);
        for (ndInt32 i = 0; i < modWorkGroupSize; i += workGroupSize)
        {
            for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
            {
                ndUnsigned32 val = srcBuffer[srcBase + i + itemId];
                dstBuffer[dstBase + i + itemId] = val;
            }
        }
        for (ndInt32 itemId = 0; itemId < workGroupSizeReminder; ++itemId)
        {
            ndUnsigned32 val = srcBuffer[srcBase + modWorkGroupSize + itemId];
            dstBuffer[dstBase + modWorkGroupSize + itemId] = val;
        }
    }
};

class brainAccumulateWeigndAndBiasGradients : public ndBrainKernel
{
    public:
    brainAccumulateWeigndAndBiasGradients(ndBrainContext* const context)
        :ndBrainKernel(context)
    {
    }

    virtual void Execute(ndInt32 groupId, ndInt32 workGroupSize) override
    {
        ndBrainUniformBuffer* const buffer0 = (ndBrainUniformBuffer*)m_parameters[0];
        ndBrainFloatBuffer* const buffer1 = (ndBrainFloatBuffer*)m_parameters[1];

        const ndCommandSharedInfo& info = *(ndCommandSharedInfo*)buffer0->GetGpuBuffer()->GetPtr();
        ndAssert(info.m_matrixDimensionK == workGroupSize);
        ndInt32 start = groupId * workGroupSize;
        ndInt32 count = ndInt32(((start + workGroupSize) < info.m_inputOutputSize) ? workGroupSize : info.m_inputOutputSize - start);
        
        ndBrainMemVector buffer((ndBrainFloat*)buffer1->GetCpuPtr(), ndInt32(buffer1->SizeInItems()));
        ndBrainMemVector dst(&buffer[start], count);
        const ndBrainMemVector src(&buffer[start + info.m_inputOutputSize], count);
        dst.Add(src);
        dst.SanityCheck();
    }
};

void ndBrainGpuContext::CreateKerners()
{   
    // create all feed foward shaders
    m_brainCopyInput = ndSharedPtr<ndBrainKernel> (new brainCopyInput(this));
    m_brainCopyOutput = ndSharedPtr<ndBrainKernel> (new brainCopyOutput(this));
    m_brainLayerReluActivation = ndSharedPtr<ndBrainKernel>(new brainLayerReluActivation(this));
    m_brainLayerTanhActivation = ndSharedPtr<ndBrainKernel>(new brainLayerTanhActivation(this));
    m_brainLayerLinearActivation = ndSharedPtr<ndBrainKernel>(new brainLayerLinearActivation(this));
    m_brainLayerSoftmaxActivation = ndSharedPtr<ndBrainKernel>(new brainLayerSoftmaxActivation(this));
    m_brainLayerLeakyReluActivation = ndSharedPtr<ndBrainKernel>(new brainLayerLeakyReluActivation(this));
    m_brainLayerDropOutActivation = ndSharedPtr<ndBrainKernel>(new brainLayerLinearDropOutActivation(this));
    m_brainLayerMatrixMatrixAddBias = ndSharedPtr<ndBrainKernel>(new brainLayerMatrixMatrixAddBias(this));
    m_brainLayerMatrixMatrixMultiply = ndSharedPtr<ndBrainKernel>(new brainLayerMatrixMatrixMultiply(this));

    m_brainLayerBatchNormalizationLoadInputActivation = ndSharedPtr<ndBrainKernel>(new brainLayerBatchNormalizationActivationInputSqr(this));
    m_brainLayerBatchNormalizationAddInputActivation = ndSharedPtr<ndBrainKernel>(new brainLayerBatchNormalizationActivationVarianceSum(this));
    m_brainLayerBatchNormalizationNormalizeInputActivation = ndSharedPtr<ndBrainKernel>(new brainLayerBatchNormalizationActivationNormalize(this));

    // create all backpropagate shaders
    m_brainCopyInputGradients = ndSharedPtr<ndBrainKernel>(new brainCopyInputGradients(this));
    m_brainCopyOutputGradients = ndSharedPtr<ndBrainKernel>(new brainCopyOutputGradients(this));
    m_brainLayerLinearPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainLinearBackPropagate(this));
    m_brainLayerReluBackPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainReluBackPropagate(this));
    m_brainLayerTanhBackPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainTanhBackPropagate(this));
    m_brainLayerLeakyReluBackPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainLeakyReluBackPropagate(this));
    m_brainLayerDropOutBackPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainLinearDropOutBackPropagate(this));
    m_brainLayerPolicyGradientBackPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainPolicyGradientBackPropagate(this));
    m_brainLayerCathegoricalSoftmaxBackPropagate = ndSharedPtr<ndBrainKernel>(new brainLayerBrainCathegoricalSoftmaxBackPropagate(this));
    m_brainLayerMatrixBackPropagateBiasGradients = ndSharedPtr<ndBrainKernel>(new brainLayerBrainBackPropagateMatrixBiasGradients(this));
    m_brainLayerMatrixBackPropagateInputGradients = ndSharedPtr<ndBrainKernel>(new brainLayerBrainBackPropagateMatrixInputGradients(this));
    m_brainLayerMatrixBackPropagateWeightGradients = ndSharedPtr<ndBrainKernel>(new brainLayerBrainBackPropagateMatrixWeightsGradients(this));

    // optimizer kernels
    m_brainAdamBiasCorrectionUpdate = ndSharedPtr<ndBrainKernel>(new brainAdamBiasCorrectionUpdate(this));
    m_brainAdamRidgeOptimizerUpdate = ndSharedPtr<ndBrainKernel>(new brainAdamUpdateRidgeRegularizer(this));
    m_brainAdamLassoOptimizerUpdate = ndSharedPtr<ndBrainKernel>(new brainAdamUpdateLassoRegularizer(this));

    // optimizer kernels
    m_brainCopyBuffer = ndSharedPtr<ndBrainKernel>(new brainCopyBuffer(this));
    m_brainCopyBufferIndirect = ndSharedPtr<ndBrainKernel>(new brainCopyBufferIndirect(this));
    m_accumulateWeigndAndBiasGradiens = ndSharedPtr<ndBrainKernel>(new brainAccumulateWeigndAndBiasGradients(this));
}