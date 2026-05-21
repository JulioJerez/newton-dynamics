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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndSort.h"
#include "ndWorld.h"
#include "ndSkeletonContainer.h"

void ndSkeletonContainer::ParallelInitMassMatrix(const ndLeftHandSide* const matrixRow, ndRightHandSide* const rightHandSide)
{
	if (m_isResting)
	{
		return;
	}

	D_TRACKTIME();
	ndInt32 rowCount = 0;
	ndInt32 auxiliaryCount = 0;

	m_threadId = 0;
	m_leftHandSide = matrixRow;
	m_rightHandSide = rightHandSide;

	const ndInt32 nodeCount = m_nodeList.GetCount();
	ndSpatialMatrix* const bodyMassArray = ndAlloca(ndSpatialMatrix, nodeCount);
	ndSpatialMatrix* const jointMassArray = ndAlloca(ndSpatialMatrix, nodeCount);
	if (m_nodesOrder)
	{
		for (ndInt32 i = 0; i < nodeCount - 1; ++i)
		{
			ndNode* const node = m_nodesOrder[i];
			rowCount += node->m_joint->m_rowCount;
			auxiliaryCount += node->FactorizeChild(matrixRow, rightHandSide, bodyMassArray, jointMassArray);
		}
		m_nodesOrder[nodeCount - 1]->FactorizeRoot(bodyMassArray, jointMassArray);
	}

	m_rowCount = rowCount;
	m_auxiliaryRowCount = auxiliaryCount;

	ndInt32 loopRowCount = 0;
	for (ndInt32 i = ndInt32(m_permanentLoopingJoints.GetCount() - 1); i >= 0; --i)
	{
		const ndConstraint* const joint = m_permanentLoopingJoints[i];
		loopRowCount += joint->m_rowCount;
	}
	for (ndInt32 i = ndInt32(m_transientLoopingJoints.GetCount() - 1); i >= 0; --i)
	{
		const ndConstraint* const joint = m_transientLoopingJoints[i];
		loopRowCount += joint->m_rowCount;
	}
	for (ndInt32 i = ndInt32(m_transientLoopingContacts.GetCount() - 1); i >= 0; --i)
	{
		const ndConstraint* const joint = m_transientLoopingContacts[i];
		loopRowCount += joint->m_rowCount;
	}

	m_loopRowCount = loopRowCount;

	m_rowCount += m_loopRowCount;
	m_auxiliaryRowCount += m_loopRowCount;
	if (m_auxiliaryRowCount)
	{
		ParallelInitLoopMassMatrix();

		const ndInt32 stride = m_auxiliaryRowCount;
		const ndInt32 size = m_auxiliaryRowCount - m_blockSize;
		ndFloat32* const matrix = &m_massMatrix11[m_blockSize * stride + m_blockSize];

		for (ndInt32 i = 0; i < size; ++i)
		{
			ndFloat32 diagSqrt = ndSqrt(matrix[i * stride + i]);
			m_diagonalPreconditioner[i] = ndFloat32(1.0f) / diagSqrt;
		}

		ndFloat32* const preconditionMatrix = &m_precondinonedMassMatrix11[0];
		auto Precondition = ndMakeObject::ndFunction([this, stride, size, matrix, preconditionMatrix](ndInt32 groupId, ndInt32)
		{
			const ndFloat32* const srcRow = &matrix[groupId * stride];
			ndFloat32 diagonal = m_diagonalPreconditioner[groupId];
			ndFloat32* const dstRow = &preconditionMatrix[groupId * stride];
			for (ndInt32 j = 0; j < size; ++j)
			{
				dstRow[j] = diagonal * srcRow[j] * m_diagonalPreconditioner[j];
			}
		});
		ndScene* const scene = m_owner->GetScene();
		scene->ParallelExecute(Precondition, size, 4);
	}
}

void ndSkeletonContainer::ParallelInitLoopMassMatrix()
{
	CalculateBufferSizeInBytes();
	ndInt8* const memoryBuffer = &m_auxiliaryMemoryBuffer[0];
	const ndInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;

	#define ndAlignedPtr(type, ptr) (type*)((size_t(ptr) + 31) & -0x20)

	m_frictionIndex = ndAlignedPtr(ndInt32, memoryBuffer);
	m_matrixRowsIndex = ndAlignedPtr(ndInt32, &m_frictionIndex[m_rowCount]);

	m_bodyForceRemap0.m_index = ndAlignedPtr(ndBodyForceIndexPair, &m_matrixRowsIndex[m_rowCount]);
	m_bodyForceRemap0.m_indexSpan = ndAlignedPtr(ndInt32, &m_bodyForceRemap0.m_index[m_rowCount]);
	m_bodyForceRemap1.m_index = ndAlignedPtr(ndBodyForceIndexPair, &m_bodyForceRemap0.m_indexSpan[m_rowCount]);
	m_bodyForceRemap1.m_indexSpan = ndAlignedPtr(ndInt32, &m_bodyForceRemap1.m_index[m_rowCount]);

	m_pairs = ndAlignedPtr(ndNodePair, &m_bodyForceRemap1.m_indexSpan[m_rowCount]);
	m_diagonalPreconditioner = ndAlignedPtr(ndFloat32, &m_pairs[m_rowCount]);
	m_precondinonedMassMatrix11 = ndAlignedPtr(ndFloat32, &m_diagonalPreconditioner[m_rowCount]);

	m_massMatrix11 = ndAlignedPtr(ndFloat32, &m_precondinonedMassMatrix11[m_auxiliaryRowCount * m_auxiliaryRowCount]);
	m_massMatrix10 = ndAlignedPtr(ndFloat32, &m_massMatrix11[m_auxiliaryRowCount * m_auxiliaryRowCount]);
	m_deltaForce = ndAlignedPtr(ndFloat32, &m_massMatrix10[m_auxiliaryRowCount * primaryCount]);

	ndInt32* const boundRow = ndAlloca(ndInt32, m_auxiliaryRowCount);

	m_blockSize = 0;
	ndScene* const scene = m_owner->GetScene();
	ndFixSizeArray<ndInt32, 1024> scans;
	{
		scans.SetCount(0);
		const ndInt32 nodeCount = m_nodeList.GetCount() - 1;
		for (ndInt32 i = 0; i < nodeCount; ++i)
		{
			const ndNode* const node = m_nodesOrder[i];
			scans.PushBack(node->m_dof);
		}

		ndInt32 sum = 0;
		for (ndInt32 i = 0; i < scans.GetCount(); ++i)
		{
			const ndInt32 entryCount = scans[i];
			scans[i] = sum;
			sum += entryCount;
		}
		scans.PushBack(sum);

		auto SubmmitRows = ndMakeObject::ndFunction([this, &scans, boundRow](ndInt32 groupId, ndInt32)
		{
			const ndNode* const node = m_nodesOrder[groupId];
			ndJointBilateralConstraint* const joint = node->m_joint;
			const ndInt32 m0 = joint->GetBody0()->m_index;
			const ndInt32 m1 = joint->GetBody1()->m_index;
			const ndInt32 primaryDof = node->m_dof;
			const ndInt32 first = joint->m_rowStart;
			const ndInt32 primaryIndex = scans[groupId];

			ndAssert(primaryDof == (scans[groupId + 1] - scans[groupId]));
			for (ndInt32 j = 0; j < primaryDof; ++j)
			{
				const ndInt32 index = node->m_ordinal.m_sourceJacobianIndex[j];
				const ndInt32 dstIndex = primaryIndex + j;
				m_pairs[dstIndex].m_m0 = m0;
				m_pairs[dstIndex].m_m1 = m1;
				m_pairs[dstIndex].m_joint = joint;
				m_frictionIndex[dstIndex] = 0;
				m_matrixRowsIndex[dstIndex] = first + index;
			}
		});
		scene->ParallelExecute(SubmmitRows, nodeCount, 1);
	}
	ndAssert(scans[scans.GetCount() - 1] == primaryCount);

	ndInt32 auxiliaryIndexCount = 0;
	{
		scans.SetCount(0);
		const ndInt32 nodeCount = m_nodeList.GetCount() - 1;
		for (ndInt32 i = 0; i < nodeCount; ++i)
		{
			const ndNode* const node = m_nodesOrder[i];
			ndJointBilateralConstraint* const joint = node->m_joint;
			const ndInt32 auxiliaryDof = joint->m_rowCount - node->m_dof;
			scans.PushBack(auxiliaryDof);
		}

		ndInt32 sum = 0;
		for (ndInt32 i = 0; i < scans.GetCount(); ++i)
		{
			const ndInt32 entryCount = scans[i];
			scans[i] = sum;
			sum += entryCount;
		}
		scans.PushBack(sum);

		auto SubmmitRows = ndMakeObject::ndFunction([this, &scans, boundRow, primaryCount](ndInt32 groupId, ndInt32)
		{
			const ndNode* const node = m_nodesOrder[groupId];
			ndJointBilateralConstraint* const joint = node->m_joint;
			const ndInt32 m0 = joint->GetBody0()->m_index;
			const ndInt32 m1 = joint->GetBody1()->m_index;
			const ndInt32 primaryDof = node->m_dof;
			const ndInt32 first = joint->m_rowStart;
			const ndInt32 auxiliaryDof = joint->m_rowCount - node->m_dof;
			ndAssert (auxiliaryDof == (scans[groupId + 1] - scans[groupId]));

			ndInt32 auxiliaryIndex = scans[groupId];
			for (ndInt32 j = 0; j < auxiliaryDof; ++j)
			{
				const ndInt32 index = node->m_ordinal.m_sourceJacobianIndex[primaryDof + j];
				const ndRightHandSide* const rhs = &m_rightHandSide[first + index];

				const ndInt32 dstIndex = auxiliaryIndex + primaryCount;
				m_pairs[dstIndex].m_m0 = m0;
				m_pairs[dstIndex].m_m1 = m1;
				m_pairs[dstIndex].m_joint = joint;
				m_frictionIndex[dstIndex] = m_auxiliaryRowCount - auxiliaryIndex;
				m_matrixRowsIndex[dstIndex] = first + index;
				const ndInt32 boundIndex = (rhs->m_lowerBoundFrictionCoefficent <= ndFloat32(-D_MAX_SKELETON_LCP_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= ndFloat32(D_MAX_SKELETON_LCP_VALUE)) ? 1 : 0;
				ndAssert(joint->IsBilateral());
				ndAssert(rhs->SanityCheck());
				boundRow[auxiliaryIndex] = boundIndex;
				m_blockSize += boundIndex;
				auxiliaryIndex++;
			}
		});
		scene->ParallelExecute(SubmmitRows, nodeCount, 1);
		auxiliaryIndexCount += scans[scans.GetCount() - 1];
	}
	ndAssert(m_loopRowCount == (m_auxiliaryRowCount - scans[scans.GetCount() - 1]));
	
	if (m_permanentLoopingJoints.GetCount())
	{
		scans.SetCount(0);
		for (ndInt32 j = 0; j < ndInt32(m_permanentLoopingJoints.GetCount()); ++j)
		{
			const ndJointBilateralConstraint* const joint = m_permanentLoopingJoints[j];
			scans.PushBack(joint->m_rowCount);
		}
		ndInt32 sum = 0;
		for (ndInt32 i = 0; i < scans.GetCount(); ++i)
		{
			const ndInt32 entryCount = scans[i];
			scans[i] = sum;
			sum += entryCount;
		}
		scans.PushBack(sum);

		auto SubmmitRows = ndMakeObject::ndFunction([this, &scans, primaryCount, auxiliaryIndexCount, boundRow](ndInt32 groupId, ndInt32)
		{
			const ndJointBilateralConstraint* const joint = m_permanentLoopingJoints[groupId];
			const ndInt32 m0 = joint->GetBody0()->m_index;
			const ndInt32 m1 = joint->GetBody1()->m_index;
			
			const ndInt32 first = joint->m_rowStart;
			const ndInt32 auxiliaryDof = joint->m_rowCount;

			ndInt32 auxiliaryIndex = auxiliaryIndexCount + scans[groupId];
			for (ndInt32 i = 0; i < auxiliaryDof; ++i)
			{
				const ndRightHandSide* const rhs = &m_rightHandSide[first + i];
				const ndInt32 dstIndex = auxiliaryIndex + primaryCount;
				m_pairs[dstIndex].m_m0 = m0;
				m_pairs[dstIndex].m_m1 = m1;
				m_pairs[dstIndex].m_joint = joint;
				m_matrixRowsIndex[dstIndex] = first + i;
				m_frictionIndex[dstIndex] = (rhs->m_normalForceIndex < 0) ? m_auxiliaryRowCount - auxiliaryIndex : rhs->m_normalForceIndex - i;
				const ndInt32 boundIndex = (rhs->m_lowerBoundFrictionCoefficent <= ndFloat32(-D_MAX_SKELETON_LCP_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= ndFloat32(D_MAX_SKELETON_LCP_VALUE)) ? 1 : 0;
				ndAssert(rhs->SanityCheck());
				boundRow[auxiliaryIndex] = boundIndex;
				m_blockSize += boundIndex;
				auxiliaryIndex++;
			}
		});
		scene->ParallelExecute(SubmmitRows, ndInt32(m_permanentLoopingJoints.GetCount()), 1);
		auxiliaryIndexCount += scans[scans.GetCount() - 1];
	}

	if (m_transientLoopingJoints.GetCount())
	{
		scans.SetCount(0);
		for (ndInt32 j = 0; j < ndInt32(m_transientLoopingJoints.GetCount()); ++j)
		{
			const ndJointBilateralConstraint* const joint = m_transientLoopingJoints[j];
			scans.PushBack(joint->m_rowCount);
		}
		ndInt32 sum = 0;
		for (ndInt32 i = 0; i < scans.GetCount(); ++i)
		{
			const ndInt32 entryCount = scans[i];
			scans[i] = sum;
			sum += entryCount;
		}
		scans.PushBack(sum);

		auto SubmmitRows = ndMakeObject::ndFunction([this, &scans, primaryCount, auxiliaryIndexCount, boundRow](ndInt32 groupId, ndInt32)
		{
			const ndJointBilateralConstraint* const joint = m_transientLoopingJoints[groupId];
			const ndInt32 m0 = joint->GetBody0()->m_index;
			const ndInt32 m1 = joint->GetBody1()->m_index;

			const ndInt32 first = joint->m_rowStart;
			const ndInt32 auxiliaryDof = joint->m_rowCount;

			ndInt32 auxiliaryIndex = auxiliaryIndexCount + scans[groupId];
			for (ndInt32 i = 0; i < auxiliaryDof; ++i)
			{
				const ndRightHandSide* const rhs = &m_rightHandSide[first + i];
				const ndInt32 dstIndex = auxiliaryIndex + primaryCount;
				m_pairs[dstIndex].m_m0 = m0;
				m_pairs[dstIndex].m_m1 = m1;
				m_pairs[dstIndex].m_joint = joint;
				m_matrixRowsIndex[dstIndex] = first + i;
				m_frictionIndex[dstIndex] = (rhs->m_normalForceIndex < 0) ? m_auxiliaryRowCount - auxiliaryIndex : rhs->m_normalForceIndex - i;
				const ndInt32 boundIndex = (rhs->m_lowerBoundFrictionCoefficent <= ndFloat32(-D_MAX_SKELETON_LCP_VALUE)) && (rhs->m_upperBoundFrictionCoefficent >= ndFloat32(D_MAX_SKELETON_LCP_VALUE)) ? 1 : 0;
				ndAssert(rhs->SanityCheck());
				boundRow[auxiliaryIndex] = boundIndex;
				m_blockSize += boundIndex;
				auxiliaryIndex++;
			}
		});
		scene->ParallelExecute(SubmmitRows, ndInt32(m_transientLoopingJoints.GetCount()), 1);
		auxiliaryIndexCount += scans[scans.GetCount() - 1];
	}

	const ndInt32 contactRowsStart = auxiliaryIndexCount;
	if (m_transientLoopingContacts.GetCount())
	{
		scans.SetCount(0);
		for (ndInt32 j = 0; j < ndInt32(m_transientLoopingContacts.GetCount()); ++j)
		{
			const ndContact* const joint = m_transientLoopingContacts[j];
			scans.PushBack(joint->m_rowCount);
		}
		ndInt32 sum = 0;
		for (ndInt32 i = 0; i < scans.GetCount(); ++i)
		{
			const ndInt32 entryCount = scans[i];
			scans[i] = sum;
			sum += entryCount;
		}
		scans.PushBack(sum);

		auto SubmmitRows = ndMakeObject::ndFunction([this, &scans, primaryCount, auxiliaryIndexCount, boundRow](ndInt32 groupId, ndInt32)
		{
			const ndContact* const joint = m_transientLoopingContacts[groupId];
			const ndInt32 m0 = joint->GetBody0()->m_index;
			const ndInt32 m1 = joint->GetBody1()->m_index;

			const ndInt32 first = joint->m_rowStart;
			const ndInt32 auxiliaryDof = joint->m_rowCount;

			ndInt32 auxiliaryIndex = auxiliaryIndexCount + scans[groupId];
			for (ndInt32 i = 0; i < auxiliaryDof; ++i)
			{
				const ndInt32 dstIndex = auxiliaryIndex + primaryCount;
				const ndRightHandSide* const rhs = &m_rightHandSide[first + i];
				m_pairs[dstIndex].m_m0 = m0;
				m_pairs[dstIndex].m_m1 = m1;
				m_pairs[dstIndex].m_joint = joint;
				m_matrixRowsIndex[dstIndex] = first + i;
				m_frictionIndex[dstIndex] = (rhs->m_normalForceIndex < 0) ? m_auxiliaryRowCount - auxiliaryIndex : rhs->m_normalForceIndex - i;
				ndAssert(rhs->SanityCheck());
				ndAssert((rhs->m_lowerBoundFrictionCoefficent > ndFloat32(-D_MAX_SKELETON_LCP_VALUE)) || (rhs->m_upperBoundFrictionCoefficent < ndFloat32(D_MAX_SKELETON_LCP_VALUE)) && ((ndConstraint*)joint)->GetAsContact());
				boundRow[auxiliaryIndex] = 0;
				auxiliaryIndex++;
			}
		});
		scene->ParallelExecute(SubmmitRows, ndInt32(m_transientLoopingContacts.GetCount()), 1);
		auxiliaryIndexCount += scans[scans.GetCount() - 1];
	}

	ndAssert(auxiliaryIndexCount == m_auxiliaryRowCount);
	ndAssert(m_frictionIndex[primaryCount] == m_auxiliaryRowCount);

	for (ndInt32 i = 1; i < contactRowsStart; ++i)
	{
		ndInt32 tmpBoundRow = boundRow[i];
		ndNodePair tmpPair(m_pairs[primaryCount + i]);
		ndInt32 tmpMatrixRowsIndex = m_matrixRowsIndex[primaryCount + i];
		ndAssert((m_frictionIndex[primaryCount + i] + i) == m_auxiliaryRowCount);

		ndInt32 j = i;
		for (; j && (boundRow[j - 1] < tmpBoundRow); --j)
		{
			ndAssert(j > 0);
			ndAssert(m_frictionIndex[primaryCount + j - 1] > 0);

			boundRow[j] = boundRow[j - 1];
			m_pairs[primaryCount + j] = m_pairs[primaryCount + j - 1];
			m_matrixRowsIndex[primaryCount + j] = m_matrixRowsIndex[primaryCount + j - 1];
		}
		boundRow[j] = tmpBoundRow;
		m_pairs[primaryCount + j] = tmpPair;
		m_matrixRowsIndex[primaryCount + j] = tmpMatrixRowsIndex;
	}

	ndFloat32* const diagDamp = ndAlloca(ndFloat32, m_auxiliaryRowCount);
	ndMemSet(m_massMatrix10, ndFloat32(0.0f), primaryCount * m_auxiliaryRowCount);
	ndMemSet(m_massMatrix11, ndFloat32(0.0f), m_auxiliaryRowCount * m_auxiliaryRowCount);

	ParallelCalculateLoopMassMatrixCoefficients(diagDamp);
	ParallelConditionMassMatrix();
	ParallelRebuildMassMatrix(diagDamp);

	if (m_blockSize)
	{
		FactorizeMatrix(m_blockSize, m_auxiliaryRowCount, m_massMatrix11, diagDamp);

		ndInt32 rowStart = 0;
		const ndInt32 boundedSize = m_auxiliaryRowCount - m_blockSize;
		ndFloat32* const acc = ndAlloca(ndFloat32, m_auxiliaryRowCount);

		for (ndInt32 i = 0; i < m_blockSize; ++i)
		{
			ndMemSet(acc, ndFloat32(0.0f), boundedSize);
			const ndFloat32* const row = &m_massMatrix11[rowStart];
			for (ndInt32 j = 0; j < i; ++j)
			{
				const ndFloat32 s = row[j];
				const ndFloat32* const x = &m_massMatrix11[j * m_auxiliaryRowCount + m_blockSize];
				for (ndInt32 k = 0; k < boundedSize; ++k)
				{
					acc[k] += s * x[k];
				}
			}

			ndFloat32* const x = &m_massMatrix11[rowStart + m_blockSize];
			const ndFloat32 den = -ndFloat32(1.0f) / row[i];
			for (ndInt32 j = 0; j < boundedSize; ++j)
			{
				x[j] = (x[j] + acc[j]) * den;
			}
			rowStart += m_auxiliaryRowCount;
		}

		for (ndInt32 i = m_blockSize - 1; i >= 0; i--)
		{
			ndMemSet(acc, ndFloat32(0.0f), boundedSize);
			for (ndInt32 j = i + 1; j < m_blockSize; ++j)
			{
				const ndFloat32 s = m_massMatrix11[j * m_auxiliaryRowCount + i];
				const ndFloat32* const x = &m_massMatrix11[j * m_auxiliaryRowCount + m_blockSize];
				for (ndInt32 k = 0; k < boundedSize; ++k)
				{
					acc[k] += s * x[k];
				}
			}

			ndFloat32* const x = &m_massMatrix11[i * m_auxiliaryRowCount + m_blockSize];
			const ndFloat32 den = ndFloat32(1.0f) / m_massMatrix11[i * m_auxiliaryRowCount + i];
			for (ndInt32 j = 0; j < boundedSize; ++j)
			{
				x[j] = (x[j] - acc[j]) * den;
			}
		}

		auto InitMassMatrixBoundedBlock = ndMakeObject::ndFunction([this, boundedSize, diagDamp](ndInt32 groupId, ndInt32)
		{
			ndFixSizeArray<ndFloat32, 1024> acc(m_blockSize);
			ndAssert(m_blockSize <= acc.GetCapacity());
			for (ndInt32 j = 0; j < m_blockSize; ++j)
			{
				acc[j] = m_massMatrix11[j * m_auxiliaryRowCount + m_blockSize + groupId];
			}

			ndFloat32* const arow = &m_massMatrix11[(m_blockSize + groupId) * m_auxiliaryRowCount + m_blockSize];
			for (ndInt32 j = groupId + 1; j < boundedSize; ++j)
			{
				const ndFloat32* const row1 = &m_massMatrix11[(m_blockSize + j) * m_auxiliaryRowCount];
				ndFloat32 elem = row1[m_blockSize + groupId] + ndDotProduct(m_blockSize, &acc[0], row1);
				arow[j] = elem;
				m_massMatrix11[(m_blockSize + j) * m_auxiliaryRowCount + m_blockSize + groupId] = elem;
			}
			const ndFloat32* const row1 = &m_massMatrix11[(m_blockSize + groupId) * m_auxiliaryRowCount];
			ndFloat32 elem = row1[m_blockSize + groupId] + ndDotProduct(m_blockSize, &acc[0], row1);
			arow[groupId] = elem + diagDamp[m_blockSize + groupId];
		});
		//for (ndInt32 index = 0; index < boundedSize; ++index)
		//{
		//	InitMassMatrixBoundedBlock(index, 0);
		//}
		scene->ParallelExecute(InitMassMatrixBoundedBlock, boundedSize, 4);

		ndAssert(!boundedSize || ndTestPSDmatrix(m_auxiliaryRowCount - m_blockSize, m_auxiliaryRowCount, &m_massMatrix11[m_auxiliaryRowCount * m_blockSize + m_blockSize], GetScratchBuffer(m_auxiliaryRowCount * m_auxiliaryRowCount)));
	}

	if (m_transientLoopingContacts.GetCount() || m_transientLoopingJoints.GetCount())
	{
		ParallelRegularizeLcp();
	}

	auto SortIndexArray = [this](ndInt32 groupId)
	{
		class CompareKey
		{
			public:
			CompareKey(void* const)
			{
			}

			ndInt32 Compare(const ndBodyForceIndexPair& elementA, const ndBodyForceIndexPair& elementB) const
			{
				ndInt32 indexA = (elementA.m_bodyIndex << 16) + elementA.m_forceIndex;
				ndInt32 indexB = (elementB.m_bodyIndex << 16) + elementB.m_forceIndex;
				if (indexA < indexB)
				{
					return -1;
				}
				else if (indexA > indexB)
				{
					return 1;
				}
				return 0;
			}
		};
		ndBodyForcePtr& bodyForceRemap = groupId ? m_bodyForceRemap1 : m_bodyForceRemap0;
		ndSort<ndBodyForceIndexPair, CompareKey>(bodyForceRemap.m_index, m_rowCount, nullptr);

		ndInt32 spanIndex = 0;
		for (ndInt32 i = 0; i < m_rowCount; ++i)
		{
			bodyForceRemap.m_indexSpan[spanIndex] = i;
			ndInt32 test = bodyForceRemap.m_index[i].m_bodyIndex;
			for (++i; (i < m_rowCount) && (bodyForceRemap.m_index[i].m_bodyIndex == test); ++i);
			i--;
			spanIndex++;
		}
		ndAssert(spanIndex < m_rowCount);
		bodyForceRemap.m_indexSpan[spanIndex] = ndInt16(m_rowCount);
		bodyForceRemap.m_spansCount = spanIndex;
	};
	for (ndInt32 i = 0; i < m_rowCount; ++i)
	{
		const ndInt32 m0 = m_pairs[i].m_m0;
		const ndInt32 m1 = m_pairs[i].m_m1;
		m_bodyForceRemap0.m_index[i].m_bodyIndex = m0;
		m_bodyForceRemap0.m_index[i].m_forceIndex = i;
		m_bodyForceRemap1.m_index[i].m_bodyIndex = m1;
		m_bodyForceRemap1.m_index[i].m_forceIndex = i;
	}
	SortIndexArray(0);
	SortIndexArray(1);
}

void ndSkeletonContainer::ParallelCalculateLoopMassMatrixCoefficients(ndFloat32* const diagDamp)
{
	D_TRACKTIME();
	auto CalculateLoopMassMatrixCoefficients = ndMakeObject::ndFunction([this, diagDamp](ndInt32 groupId, ndInt32)
	{
		const ndInt32 index = groupId;
		const ndVector8 zero(ndVector8::m_zero);

		const ndInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
		const ndInt32 ii = m_matrixRowsIndex[primaryCount + index];
		const ndLeftHandSide* const row_i = &m_leftHandSide[ii];
		const ndRightHandSide* const rhs_i = &m_rightHandSide[ii];

		const ndVector8& JtM0 = (ndVector8&)row_i->m_Jt.m_jacobianM0;
		const ndVector8& JtM1 = (ndVector8&)row_i->m_Jt.m_jacobianM1;
		const ndVector8& JMinvM0 = (ndVector8&)row_i->m_JMinv.m_jacobianM0;
		const ndVector8& JMinvM1 = (ndVector8&)row_i->m_JMinv.m_jacobianM1;
		const ndVector8 element(JMinvM0 * JtM0 + JMinvM1 * JtM1);

		// I know I am doubling the matrix regularizer, but this makes the solution more robust.
		ndFloat32* const matrixRow11 = &m_massMatrix11[m_auxiliaryRowCount * index];
		ndFloat32 diagonal = element.AddHorizontal() + rhs_i->m_diagDamp * ndFloat32(2.0f);
		ndAssert(matrixRow11[index] == ndFloat32(0.0f));
		matrixRow11[index] = diagonal;
		diagDamp[index] = diagonal * ndFloat32(4.0e-3f);

		const ndInt32 m0_i = m_pairs[primaryCount + index].m_m0;
		const ndInt32 m1_i = m_pairs[primaryCount + index].m_m1;
		for (ndInt32 j = index + 1; j < m_auxiliaryRowCount; ++j)
		{
			const ndInt32 jj = m_matrixRowsIndex[primaryCount + j];
			const ndLeftHandSide* const row_j = &m_leftHandSide[jj];

			const ndInt32 k = primaryCount + j;
			const ndInt32 m0_j = m_pairs[k].m_m0;
			const ndInt32 m1_j = m_pairs[k].m_m1;

			bool hasEffect = false;
			ndVector8 acc(zero);
			if (m0_i == m0_j)
			{
				hasEffect = true;
				acc = acc.MulAdd(JMinvM0, (ndVector8&)row_j->m_Jt.m_jacobianM0);
			}
			else if (m0_i == m1_j)
			{
				hasEffect = true;
				acc = acc.MulAdd(JMinvM0, (ndVector8&)row_j->m_Jt.m_jacobianM1);
			}

			if (m1_i == m1_j)
			{
				hasEffect = true;
				acc = acc.MulAdd(JMinvM1, (ndVector8&)row_j->m_Jt.m_jacobianM1);
			}
			else if (m1_i == m0_j)
			{
				hasEffect = true;
				acc = acc.MulAdd(JMinvM1, (ndVector8&)row_j->m_Jt.m_jacobianM0);
			}

			if (hasEffect)
			{
				ndFloat32 offDiagValue = acc.AddHorizontal();
				if (ndAbs(offDiagValue) > ndFloat32(0.0f))
				{
					ndAssert(matrixRow11[j] == ndFloat32(0.0f));
					matrixRow11[j] = offDiagValue;
					m_massMatrix11[j * m_auxiliaryRowCount + index] = offDiagValue;
				}
			}
		}

		ndFloat32* const matrixRow10 = &m_massMatrix10[primaryCount * index];
		for (ndInt32 j = 0; j < primaryCount; ++j)
		{
			const ndInt32 jj = m_matrixRowsIndex[j];
			const ndLeftHandSide* const row_j = &m_leftHandSide[jj];

			const ndInt32 m0_j = m_pairs[j].m_m0;
			const ndInt32 m1_j = m_pairs[j].m_m1;

			bool hasEffect = false;
			ndVector8 acc(zero);
			if (m0_i == m0_j)
			{
				hasEffect = true;
				acc = acc.MulAdd(JMinvM0, (ndVector8&)row_j->m_Jt.m_jacobianM0);
			}
			else if (m0_i == m1_j)
			{
				hasEffect = true;
				acc = acc.MulAdd(JMinvM0, (ndVector8&)row_j->m_Jt.m_jacobianM1);
			}

			if (m1_i == m1_j)
			{
				hasEffect = true;
				acc = acc.MulAdd(JMinvM1, (ndVector8&)row_j->m_Jt.m_jacobianM1);
			}
			else if (m1_i == m0_j)
			{
				hasEffect = true;
				acc = acc.MulAdd(JMinvM1, (ndVector8&)row_j->m_Jt.m_jacobianM0);
			}

			if (hasEffect)
			{
				ndFloat32 val = acc.AddHorizontal();
				ndAssert(matrixRow10[j] == ndFloat32(0.0f));
				matrixRow10[j] = val;
			}
		}
	});
	//for (ndInt32 index = 0; index < m_auxiliaryRowCount; ++index)
	//{
	//	CalculateLoopMassMatrixCoefficients(index, 0);
	//}
	ndScene* const scene = m_owner->GetScene();
	scene->ParallelExecute(CalculateLoopMassMatrixCoefficients, m_auxiliaryRowCount, 4);
}

void ndSkeletonContainer::ParallelConditionMassMatrix() const
{
	D_TRACKTIME();
	//auto ConditionMassMatrix = [this](ndInt32 groupId)
	auto ConditionMassMatrix = ndMakeObject::ndFunction([this](ndInt32 groupId, ndInt32)
	{
		ndInt32 entry0 = 0;
		const ndInt32 nodeCount = m_nodeList.GetCount();
		const ndInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;

		const ndSpatialVector zero(ndSpatialVector::m_zero);
		ndForcePair* const forcePair = ndAlloca(ndForcePair, nodeCount);

		ndInt32 startjoint = nodeCount;
		const ndFloat32* const matrixRow10 = &m_massMatrix10[groupId * primaryCount];
		for (ndInt32 j = 0; j < nodeCount - 1; ++j)
		{
			const ndNode* const node = m_nodesOrder[j];
			const ndInt32 index = node->m_index;
			forcePair[index].m_body = zero;
			ndSpatialVector& a = forcePair[index].m_joint;

			const ndInt32 count = node->m_dof;
			for (ndInt32 k = 0; k < count; ++k)
			{
				const ndFloat32 value = matrixRow10[entry0];
				a[k] = value;
				startjoint = (value == 0.0f) ? startjoint : ndMin(startjoint, index);
				entry0++;
			}
		}

		startjoint = (startjoint == nodeCount) ? 0 : startjoint;
		ndAssert(startjoint < nodeCount);
		forcePair[nodeCount - 1].m_body = zero;
		forcePair[nodeCount - 1].m_joint = zero;
		SolveForward(forcePair, forcePair, startjoint);
		SolveBackward(forcePair);

		ndInt32 entry1 = 0;
		ndFloat32* const deltaForcePtr = &m_deltaForce[groupId * primaryCount];
		for (ndInt32 j = 0; j < nodeCount - 1; ++j)
		{
			const ndNode* const node = m_nodesOrder[j];
			const ndInt32 index = node->m_index;
			const ndSpatialVector& f = forcePair[index].m_joint;
			const ndInt32 count = node->m_dof;
			for (ndInt32 k = 0; k < count; ++k)
			{
				deltaForcePtr[entry1] = ndFloat32(f[k]);
				entry1++;
			}
		}
	});
	//for (ndInt32 index = 0; index < m_auxiliaryRowCount; ++index)
	//{
	//	ConditionMassMatrix(index, 0);
	//}
	ndScene* const scene = m_owner->GetScene();
	scene->ParallelExecute(ConditionMassMatrix, m_auxiliaryRowCount, 4);
}

void ndSkeletonContainer::ParallelRebuildMassMatrix(const ndFloat32* const diagDamp) const
{
	D_TRACKTIME();
	//auto RebuildMassMatrix = [this, diagDamp](ndInt32 groupId)
	auto RebuildMassMatrix = ndMakeObject::ndFunction([this, diagDamp](ndInt32 groupId, ndInt32)
	{
		const ndInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;
		const ndFloat32* const matrixRow10 = &m_massMatrix10[groupId * primaryCount];
		ndFloat32* const matrixRow11 = &m_massMatrix11[groupId * m_auxiliaryRowCount];

		ndInt32 indexCount = 0;
		ndInt16* const indexList = ndAlloca(ndInt16, primaryCount);
		for (ndInt32 k = 0; k < primaryCount; ++k)
		{
			indexList[indexCount] = ndInt16(k);
			indexCount += (matrixRow10[k] != ndFloat32(0.0f)) ? 1 : 0;
		}

		for (ndInt32 j = groupId; j < m_auxiliaryRowCount; ++j)
		{
			ndFloat32 offDiagonal = matrixRow11[j];
			const ndFloat32* const row10 = &m_deltaForce[j * primaryCount];
			for (ndInt32 k = 0; k < indexCount; ++k)
			{
				ndInt32 index = indexList[k];
				offDiagonal += matrixRow10[index] * row10[index];
			}
			matrixRow11[j] = offDiagonal;
			m_massMatrix11[j * m_auxiliaryRowCount + groupId] = offDiagonal;
		}

		matrixRow11[groupId] = ndMax(matrixRow11[groupId], diagDamp[groupId]);
	});
	//for (ndInt32 index = 0; index < m_auxiliaryRowCount; ++index)
	//{
	//	RebuildMassMatrix(index, 0);
	//}
	ndScene* const scene = m_owner->GetScene();
	scene->ParallelExecute(RebuildMassMatrix, m_auxiliaryRowCount, 4);

#if 0
	ndInt32 nonZeroCount = 0;
	for (ndInt32 i = 0; i < m_auxiliaryRowCount; ++i)
	{
		for (ndInt32 j = i + 1; j < m_auxiliaryRowCount; ++j)
		{
			nonZeroCount += (m_massMatrix11[i * m_auxiliaryRowCount + j] != ndFloat32(0.0f)) ? 1 : 0;
		}
	}
	ndTrace(("not zero %d %d\n", nonZeroCount * 2 + m_auxiliaryRowCount, m_auxiliaryRowCount * m_auxiliaryRowCount));
#endif
}

void ndSkeletonContainer::ParallelRegularizeLcp() const
{
	// note: this can be eliminated if I just regularize the diagonal unconditionally.
	ndAssert(m_threadId == 0);
	ndInt32 size = m_auxiliaryRowCount - m_blockSize;
	if (size < 128)
	{
		RegularizeLcp();
		return;
	}
	
	ndFloat32* const matrix = &m_massMatrix11[m_auxiliaryRowCount * m_blockSize + m_blockSize];
	if (!ParallelTestPSDmatrix(size, m_auxiliaryRowCount, matrix))
	{
		ndFloat32* const regularizer = ndAlloca(ndFloat32, size);
		ndMemSet(regularizer, ndFloat32(1.01f), size);
		ndInt32 step = m_auxiliaryRowCount + 1;
		ndFloat32 reg = ndFloat32(1.125f);
		do
		{
			ndFloat32* base = &m_massMatrix11[m_auxiliaryRowCount * m_auxiliaryRowCount - 1];
			for (ndInt32 i = size - 1; i >= 0; --i)
			{
				*base = *base * reg;
				base -= step;
			}
			reg *= ndFloat32(1.125f);
		} while (!ParallelTestPSDmatrix(size, m_auxiliaryRowCount, matrix));
	}
}

bool ndSkeletonContainer::ParallelTestPSDmatrix(ndInt32 size, ndInt32 stride, ndFloat32* const psdMatrix) const
{
	// make a copy of the source matrix
	#define ND_SKELETON_TILE_COUNT	4
	#define ND_SKELETON_TILE_SIZE	(ND_SIMD8_WORK_GROUP_SIZE * ND_SKELETON_TILE_COUNT)

	const ndInt32 bufferStride = ndInt32 (((size * sizeof(ndFloat32) + 32 - 1) & -32) / sizeof(ndFloat32));
	ndFloat32* const buffer = GetScratchBuffer(bufferStride * size);
	auto MakeCopy = ndMakeObject::ndFunction([this, size, stride, psdMatrix, buffer, bufferStride](ndInt32 groupId, ndInt32)
	{
		ndFloat32* const dstRow = &buffer[bufferStride * groupId];
		const ndFloat32* const srcRow = &psdMatrix[stride * groupId];
		for (ndInt32 i = 0; i < size; ++i)
		{	
			dstRow[i] = srcRow[i];
		}
	});
	ndScene* const scene = m_owner->GetScene();
	scene->ParallelExecute(MakeCopy, size, 4);

	D_MSV_NEWTON_CLASS_ALIGN_32
	class CholeskyTile
	{
		public:
		void Clear()
		{
			ndVector8* const row = (ndVector8*)&m_element[0][0];
			for (ndInt32 i = 0; i < ND_SKELETON_TILE_SIZE * ND_SKELETON_TILE_COUNT; ++i)
			{
				row[i] = ndVector8::m_zero;
			}
		}

		ndFloat32 m_element[ND_SKELETON_TILE_SIZE][ND_SKELETON_TILE_SIZE];
	} D_GCC_NEWTON_CLASS_ALIGN_32;

	auto GetTile = [buffer, size, bufferStride](ndInt32 row, ndInt32 column)
	{
		CholeskyTile tile;
		const ndFloat32* const bufferPtr = &buffer[(row * bufferStride + column) * ND_SKELETON_TILE_SIZE];
		for (ndInt32 i = 0; i < ND_SKELETON_TILE_SIZE; ++i)
		{
			const ndVector8* const src = (ndVector8*)&bufferPtr[i * bufferStride];
			ndVector8* const dst = (ndVector8*)&tile.m_element[i][0];
			for (ndInt32 j = 0; j < ND_SKELETON_TILE_COUNT; ++j)
			{
				dst[j] = src[j];
			}
		}
		return tile;
	};

	auto StoreTile = [buffer, size, bufferStride](const CholeskyTile& tile, ndInt32 row, ndInt32 column)
	{
		ndFloat32* const bufferPtr = &buffer[(row * bufferStride + column) * ND_SKELETON_TILE_SIZE];
		for (ndInt32 i = 0; i < ND_SKELETON_TILE_SIZE; ++i)
		{
			ndVector8* const dst = (ndVector8*)&bufferPtr[i * bufferStride];
			const ndVector8* const src = (ndVector8*)&tile.m_element[i][0];
			for (ndInt32 j = 0; j < ND_SKELETON_TILE_COUNT; ++j)
			{
				dst[j] = src[j];
			}
		}
	};

	auto TileCholesky = [](CholeskyTile& tile)
	{
		ndFloat32 invDiag[ND_SKELETON_TILE_SIZE];
		for (ndInt32 i = 0; i < ND_SKELETON_TILE_SIZE; ++i)
		{
			for (ndInt32 j = 0; j <= i; ++j)
			{
				ndFloat32 diag = tile.m_element[i][j];
				for (ndInt32 k = 0; k < j; ++k)
				{
					diag -= tile.m_element[i][k] * tile.m_element[j][k];
				}

				if (j < i)
				{
					tile.m_element[j][i] = ndFloat32(0.0f);
					tile.m_element[i][j] = diag * invDiag[j];
				}
				else
				{
					#ifdef D_NEWTON_USE_DOUBLE
					if (diag < ndFloat32(1.0e-12f))
					{
						return false;
					}
					#else
					if (diag < ndFloat32(1.0e-6f))
					{
						return false;
					}
					#endif
					diag = ndFloat32(sqrt(diag));
					tile.m_element[i][i] = diag;
					invDiag[i] = ndFloat32(1.0f) / diag;
				}
			}
		}
		return true;
	};

	auto InverseTile = [](CholeskyTile& inverse, CholeskyTile& tile)
	{
		for (ndInt32 i = 0; i < ND_SKELETON_TILE_SIZE; ++i)
		{
			for (ndInt32 j = 0; j < ND_SKELETON_TILE_SIZE; ++j)
			{
				inverse.m_element[i][j] = ndFloat32(0.0f);
			}
			inverse.m_element[i][i] = ndFloat32(1.0f);
		}

		for (ndInt32 i = 0; i < ND_SKELETON_TILE_SIZE; ++i)
		{
			ndAssert(tile.m_element[i][i] > ndFloat32(0.0f));
			const ndFloat32 invPivot = ndFloat32(1.0f) / tile.m_element[i][i];
			for (ndInt32 j = 0; j <= i; ++j)
			{
				tile.m_element[i][j] *= invPivot;
				inverse.m_element[i][j] *= invPivot;
			}

			for (ndInt32 j = i + 1; j < ND_SKELETON_TILE_SIZE; ++j)
			{
				const ndFloat32 pivot = tile.m_element[j][i];
				for (ndInt32 k = 0; k <= j; ++k)
				{
					tile.m_element[j][k] -= tile.m_element[i][k] * pivot;
					inverse.m_element[j][k] -= inverse.m_element[i][k] * pivot;
				}
			}
		}
	};

	auto CalculateOffDiagonalTile = [buffer, bufferStride](const CholeskyTile& tile, const CholeskyTile& invDiagonal, const ndInt32 row, ndInt32 column)
	{
		ndFloat32* const dst = &buffer[(row * bufferStride + column) * ND_SKELETON_TILE_SIZE];
		for (ndInt32 i = 0; i < ND_SKELETON_TILE_SIZE; ++i)
		{
			ndFloat32* const ptr = &dst[i * bufferStride];
			for (ndInt32 j = 0; j < ND_SKELETON_TILE_SIZE; ++j)
			{
				ndFloat32 acc(0.0f);
				for (ndInt32 k = 0; k < ND_SKELETON_TILE_SIZE; ++k)
				{
					acc += tile.m_element[i][k] * invDiagonal.m_element[j][k];
				}
				ptr[j] = acc;
			}
		}
	};

	auto MultAddTile = [buffer, size, bufferStride](CholeskyTile& tile, ndInt32 row, ndInt32 column, ndInt32 m)
	{
		const ndFloat32* const tileA = &buffer[(row * bufferStride + m) * ND_SKELETON_TILE_SIZE];
		const ndFloat32* const tileB = &buffer[(column * bufferStride + m) * ND_SKELETON_TILE_SIZE];

		for (ndInt32 i = 0; i < ND_SKELETON_TILE_SIZE; ++i)
		{
			const ndFloat32* const srcA = &tileA[i * bufferStride];
			for (ndInt32 j = 0; j < ND_SKELETON_TILE_SIZE; ++j)
			{
				ndFloat32 acc(0.0f);
				const ndFloat32* const srcB = &tileB[j * bufferStride];
				for (ndInt32 k = 0; k < ND_SKELETON_TILE_SIZE; ++k)
				{
					acc += srcA[k] * srcB[k];
				}
				tile.m_element[i][j] -= acc;
			}
		}
	};

	auto DebugIndentity = [](const CholeskyTile& A, const CholeskyTile& B)
	{
		CholeskyTile tile;
		bool pass = true;
		ndMatrixTimeMatrix(ND_SKELETON_TILE_SIZE, &A.m_element[0][0], &B.m_element[0][0], &tile.m_element[0][0]);
		for (ndInt32 j = 0; j < ND_SKELETON_TILE_SIZE; ++j)
		{
			ndFloat32 error = tile.m_element[j][j] - ndFloat32(1.0f);
			pass = pass && (ndAbs(error) < ndFloat32(1.0e-6f));
			for (ndInt32 k = j + 1; k < ND_SKELETON_TILE_SIZE; ++k)
			{
				pass = pass && (ndAbs(tile.m_element[j][k]) < ndFloat32(1.0e-6f));
				pass = pass && (ndAbs(tile.m_element[k][j]) < ndFloat32(1.0e-6f));
			}
		}
		return pass;
	};

	const ndInt32 maxSize = size / ND_SKELETON_TILE_SIZE;
	CholeskyTile* const invDiagonalTiles = (CholeskyTile*)ndAlloca(CholeskyTile, maxSize + 1);

	for (ndInt32 i = 0; i < maxSize; ++i)
	{
		for (ndInt32 j = 0; j <= i; ++j)
		{
			CholeskyTile tile(GetTile(i, j));
			for (ndInt32 k = 0; k < j; ++k)
			{
				MultAddTile(tile, i, j, k);
			}

			if (j < i)
			{
				const CholeskyTile& invDiagonal = invDiagonalTiles[j];
				CalculateOffDiagonalTile(tile, invDiagonal, i, j);
			}
			else
			{
				bool state = TileCholesky(tile);
				if (!state)
				{
					return false;
				}
				
				StoreTile(tile, i, i);
				CholeskyTile& invTile = invDiagonalTiles[i];
				
				InverseTile(invTile, tile);
				ndAssert(DebugIndentity(GetTile(i, i), invTile));
			}
		}
	}

	if (size > (maxSize * ND_SKELETON_TILE_SIZE))
	{
		const ndInt32 residual = size - maxSize * ND_SKELETON_TILE_SIZE;

		auto GetResidualTile = [buffer, size, bufferStride, residual](ndInt32 row, ndInt32 column)
		{
			CholeskyTile tile;
			tile.Clear();

			const ndFloat32* const bufferSrc = &buffer[(row * bufferStride + column) * ND_SKELETON_TILE_SIZE];
			if (row != column)
			{
				for (ndInt32 i = 0; i < residual; ++i)
				{
					//const ndFloat32* const ptr = &src[i * bufferStride];
					ndVector8* const dst = (ndVector8*)&tile.m_element[i];
					const ndVector8* const src = (ndVector8*) &bufferSrc[i * bufferStride];
					for (ndInt32 j = 0; j < ND_SKELETON_TILE_COUNT; ++j)
					{
						dst[j] = src[j];
					}
				}
			}
			else
			{
				for (ndInt32 i = 0; i < residual; ++i)
				{
					const ndFloat32* const src = &bufferSrc[i * bufferStride];
					for (ndInt32 j = 0; j < residual; ++j)
					{
						tile.m_element[i][j] = src[j];
					}
				}
			}
			return tile;
		};

		auto CalculateOffDiagonalResidualTile = [buffer, size, bufferStride, residual](const CholeskyTile& tile, const CholeskyTile& invDiagonal, const ndInt32 row, ndInt32 column)
		{
			ndFloat32* const dst = &buffer[(row * bufferStride + column) * ND_SKELETON_TILE_SIZE];
			for (ndInt32 i = 0; i < residual; ++i)
			{
				ndFloat32* const ptr = &dst[i * bufferStride];
				for (ndInt32 j = 0; j < ND_SKELETON_TILE_SIZE; ++j)
				{
					ndFloat32 acc(0.0f);
					for (ndInt32 k = 0; k < ND_SKELETON_TILE_SIZE; ++k)
					{
						acc += tile.m_element[i][k] * invDiagonal.m_element[j][k];
					}
					ptr[j] = acc;
				}
			}
		};

		auto MultAddResidualTile = [buffer, size, bufferStride, residual](CholeskyTile& tile, ndInt32 row, ndInt32 column, ndInt32 m)
		{
			const ndFloat32* const tileA = &buffer[(row * bufferStride + m) * ND_SKELETON_TILE_SIZE];
			const ndFloat32* const tileB = &buffer[(column * bufferStride + m) * ND_SKELETON_TILE_SIZE];

			const ndInt32 columnWidth = (row != column) ? ND_SKELETON_TILE_SIZE : residual;
			for (ndInt32 i = 0; i < residual; ++i)
			{
				const ndFloat32* const srcA = &tileA[i * bufferStride];
				for (ndInt32 j = 0; j < columnWidth; ++j)
				{
					ndFloat32 acc(0.0f);
					const ndFloat32* const srcB = &tileB[j * bufferStride];
					for (ndInt32 k = 0; k < ND_SKELETON_TILE_SIZE; ++k)
					{
						acc += srcA[k] * srcB[k];
					}
					tile.m_element[i][j] -= acc;
				}
			}
		};

		auto TileResidualCholesky = [residual](CholeskyTile& tile)
		{
			ndFloat32 invDiag[ND_SKELETON_TILE_SIZE];
			for (ndInt32 i = 0; i < residual; ++i)
			{
				for (ndInt32 j = 0; j <= i; ++j)
				{
					ndFloat32 diag = tile.m_element[i][j];
					for (ndInt32 k = 0; k < j; ++k)
					{
						diag -= tile.m_element[i][k] * tile.m_element[j][k];
					}

					if (j < i)
					{
						tile.m_element[j][i] = ndFloat32(0.0f);
						tile.m_element[i][j] = diag * invDiag[j];
					}
					else
					{
						#ifdef D_NEWTON_USE_DOUBLE
						if (diag < ndFloat32(1.0e-12f))
						{
							return false;
						}
						#else
						if (diag < ndFloat32(1.0e-6f))
						{
							return false;
						}
						#endif

						diag = ndFloat32(sqrt(diag));
						tile.m_element[i][i] = diag;
						invDiag[i] = ndFloat32(1.0f) / diag;
					}
				}
			}
			return true;
		};

		auto StoreResidualTile = [buffer, size, bufferStride, residual](const CholeskyTile& tile, ndInt32 row, ndInt32 column)
		{
			ndFloat32* const dst = &buffer[(row * bufferStride + column) * ND_SKELETON_TILE_SIZE];
			for (ndInt32 i = 0; i < residual; ++i)
			{
				ndFloat32* const ptr = &dst[i * bufferStride];
				for (ndInt32 j = 0; j < residual; ++j)
				{
					ptr[j] = tile.m_element[i][j];
				}
			}
		};

		const ndInt32 i = maxSize;
		for (ndInt32 j = 0; j <= i; ++j)
		{
			CholeskyTile tile(GetResidualTile(i, j));
			for (ndInt32 k = 0; k < j; ++k)
			{
				MultAddResidualTile(tile, i, j, k);
			}
			if (j < i)
			{
				const CholeskyTile& invDiagonal = invDiagonalTiles[j];
				CalculateOffDiagonalResidualTile(tile, invDiagonal, i, j);
			}
			else
			{
				bool state = TileResidualCholesky(tile);
				if (!state)
				{
					return false;
				}
				StoreResidualTile(tile, i, i);
			}
		}
	}
	return true;
}

void ndSkeletonContainer::ParallelCalculateReactionForces(ndJacobian* const internalForces)
{
	if (m_isResting)
	{
		return;
	}

	D_TRACKTIME();
	m_threadId = 0;
	const ndInt32 nodeCount = m_nodeList.GetCount();
	ndForcePair* const force = ndAlloca(ndForcePair, nodeCount);
	ndForcePair* const accel = ndAlloca(ndForcePair, nodeCount);

	ParallelCalculateJointAccel(internalForces, accel);
	CalculateForce(force, accel);
	if (m_auxiliaryRowCount)
	{
		ParallelSolveAuxiliary(internalForces, accel, force);
	}
	else
	{
		UpdateForces(internalForces, force);
	}
}

void ndSkeletonContainer::ParallelCalculateJointAccel(const ndJacobian* const internalForces, ndForcePair* const accel) const
{
	D_TRACKTIME();
	const ndInt32 nodeCount = m_nodeList.GetCount();
	const ndVector8* const internalForcesArray = (ndVector8*)internalForces;

	const ndSpatialVector zero(ndSpatialVector::m_zero);
	auto CalculateJointAccel = ndMakeObject::ndFunction([this, internalForcesArray, accel, &zero](ndInt32 groupId, ndInt32)
	{
		ndNode* const node = m_nodesOrder[groupId];
		ndAssert(groupId == node->m_index);

		ndForcePair& a = accel[groupId];
		ndAssert(node->m_body);
		ndAssert(node->m_joint);
		ndJointBilateralConstraint* const joint = node->m_joint;

		const ndInt32 first = joint->m_rowStart;
		const ndInt32 dof = joint->m_rowCount;
		const ndInt32 m0 = joint->GetBody0()->m_index;
		const ndInt32 m1 = joint->GetBody1()->m_index;

		const ndVector8& y0 = internalForcesArray[m0];
		const ndVector8& y1 = internalForcesArray[m1];

		a.m_body = zero;
		a.m_joint = zero;
		for (ndInt32 j = 0; j < dof; ++j)
		{
			const ndInt32 k = node->m_ordinal.m_sourceJacobianIndex[j];
			const ndLeftHandSide* const row = &m_leftHandSide[first + k];
			const ndRightHandSide* const rhs = &m_rightHandSide[first + k];
			const ndVector8 diag((ndVector8&)row->m_JMinv.m_jacobianM0 * y0 + (ndVector8&)row->m_JMinv.m_jacobianM1 * y1);
			a.m_joint[j] = -(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp - diag.AddHorizontal());
		}
	});
	//for (ndInt32 index = 0; index < nodeCount - 1; ++index)
	//{
	//	CalculateJointAccel(index, 0);
	//}
	ndScene* const scene = m_owner->GetScene();
	scene->ParallelExecute(CalculateJointAccel, nodeCount - 1, 4);

	ndAssert((nodeCount - 1) == m_nodesOrder[nodeCount - 1]->m_index);
	accel[nodeCount - 1].m_body = zero;
	accel[nodeCount - 1].m_joint = zero;
}

void ndSkeletonContainer::ParallelSolveAuxiliary(ndJacobian* const internalForces, const ndForcePair* const, ndForcePair* const force) const
{
	ndFloat32* const f = ndAlloca(ndFloat32, m_rowCount);
	ndFloat32* const b = ndAlloca(ndFloat32, m_auxiliaryRowCount);
	ndFloat32* const low = ndAlloca(ndFloat32, m_auxiliaryRowCount);
	ndFloat32* const high = ndAlloca(ndFloat32, m_auxiliaryRowCount);
	ndFloat32* const u = ndAlloca(ndFloat32, m_auxiliaryRowCount + 1);

	ndInt32 primaryIndex = 0;
	const ndInt32 nodeCount = m_nodeList.GetCount();
	const ndInt32 primaryCount = m_rowCount - m_auxiliaryRowCount;

	for (ndInt32 i = 0; i < nodeCount - 1; ++i)
	{
		const ndNode* const node = m_nodesOrder[i];
		const ndInt32 primaryDof = node->m_dof;
		const ndSpatialVector& forceSpatial = force[i].m_joint;

		for (ndInt32 j = 0; j < primaryDof; ++j)
		{
			f[primaryIndex] = ndFloat32(forceSpatial[j]);
			primaryIndex++;
		}
	}
	ndAssert(primaryIndex == primaryCount);

	ndVector8* const internalForcesArray = (ndVector8*)internalForces;
	auto SolveAuxiliary = ndMakeObject::ndFunction([this, primaryCount, u, f, b, low, high, internalForcesArray](ndInt32 groupId, ndInt32)
	{
		const ndInt32 index = m_matrixRowsIndex[primaryCount + groupId];
		const ndLeftHandSide* const row = &m_leftHandSide[index];
		const ndRightHandSide* const rhs = &m_rightHandSide[index];

		const ndInt32 m0 = m_pairs[primaryCount + groupId].m_m0;
		const ndInt32 m1 = m_pairs[primaryCount + groupId].m_m1;

		const ndVector8& y0 = internalForcesArray[m0];
		const ndVector8& y1 = internalForcesArray[m1];

		const ndVector8 acc((ndVector8&)row->m_JMinv.m_jacobianM0 * y0 + (ndVector8&)row->m_JMinv.m_jacobianM1 * y1);
		b[groupId] = rhs->m_coordenateAccel - acc.AddHorizontal();

		const ndFloat32* const matrixRow10 = &m_massMatrix10[groupId * primaryCount];
		b[groupId] -= ndDotProduct(primaryCount, matrixRow10, f);

		u[groupId] = rhs->m_force;
		low[groupId] = rhs->m_lowerBoundFrictionCoefficent;
		high[groupId] = rhs->m_upperBoundFrictionCoefficent;
		ndAssert(rhs->SanityCheck());
	});
	//for (ndInt32 index = 0; index < m_auxiliaryRowCount; ++index)
	//{
	//	SolveAuxiliary(index, 0);
	//}
	ndScene* const scene = m_owner->GetScene();
	scene->ParallelExecute(SolveAuxiliary, m_auxiliaryRowCount, 4);

	const ndInt32* const normalIndex = &m_frictionIndex[primaryCount];
	u[m_auxiliaryRowCount] = ndFloat32(1.0f);
	SolveBlockLcp(m_auxiliaryRowCount, m_blockSize, u, b, low, high, normalIndex, ndFloat32(0.5f));

	for (ndInt32 i = 0; i < m_auxiliaryRowCount; ++i)
	{
		const ndFloat32 s = u[i];
		f[primaryCount + i] = s;
		ndScaleAdd(primaryCount, f, &m_deltaForce[i * primaryCount], s);
	}

	auto AddForcesBody0 = ndMakeObject::ndFunction([this, f, internalForcesArray](ndInt32 groupId, ndInt32)
	{
		const ndBodyForcePtr& bodyForceRemap = m_bodyForceRemap0;
		const ndLeftHandSide* const leftHandSide = m_leftHandSide;

		const ndInt32 start = bodyForceRemap.m_indexSpan[groupId];
		const ndInt32 count = bodyForceRemap.m_indexSpan[groupId + 1] - start;

		const ndInt32 m = bodyForceRemap.m_index[start].m_bodyIndex;
		ndVector8 force(internalForcesArray[m]);
		for (ndInt32 j = 0; j < count; ++j)
		{
			const ndInt32 i = bodyForceRemap.m_index[j + start].m_forceIndex;
			const ndVector8 jointForce(f[i]);
			const ndInt32 index = m_matrixRowsIndex[i];
			const ndLeftHandSide* const row = &leftHandSide[index];
			force = force.MulAdd((ndVector8&)row->m_Jt.m_jacobianM0, jointForce);
		}
		internalForcesArray[m] = force;
	});
	//for (ndInt32 i = 0; i < m_bodyForceRemap0.m_spansCount; ++i)
	//{
	//	AddForcesBody0(i, 0);
	//}
	scene->ParallelExecute(AddForcesBody0, m_bodyForceRemap0.m_spansCount, 4);

	auto AddForcesBody1 = ndMakeObject::ndFunction([this, f, internalForcesArray](ndInt32 groupId, ndInt32)
	{
		const ndBodyForcePtr& bodyForceRemap = m_bodyForceRemap1;
		const ndLeftHandSide* const leftHandSide = (ndLeftHandSide*)(((ndJacobian*)m_leftHandSide) + 1);
	
		const ndInt32 start = bodyForceRemap.m_indexSpan[groupId];
		const ndInt32 count = bodyForceRemap.m_indexSpan[groupId + 1] - start;
	
		const ndInt32 m = bodyForceRemap.m_index[start].m_bodyIndex;
		ndVector8 force(internalForcesArray[m]);
		for (ndInt32 j = 0; j < count; ++j)
		{
			const ndInt32 i = bodyForceRemap.m_index[j + start].m_forceIndex;
			const ndVector8 jointForce(f[i]);
			const ndInt32 index = m_matrixRowsIndex[i];
			const ndLeftHandSide* const row = &leftHandSide[index];
			force = force.MulAdd((ndVector8&)row->m_Jt.m_jacobianM0, jointForce);
		}
		internalForcesArray[m] = force;
	});
	//for (ndInt32 i = 0; i < m_bodyForceRemap1.m_spansCount; ++i)
	//{
	//	AddForcesBody1(i, 0);
	//}
	scene->ParallelExecute(AddForcesBody1, m_bodyForceRemap1.m_spansCount, 4);
}
