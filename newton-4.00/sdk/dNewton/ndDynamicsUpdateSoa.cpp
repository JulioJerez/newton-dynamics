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
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndSkeletonList.h"
#include "ndDynamicsUpdateSoa.h"
#include "ndJointBilateralConstraint.h"

#define D_SOA_DEFAULT_BUFFER_SIZE	1024

D_MSV_NEWTON_CLASS_ALIGN_32
class ndSoaVector3
{
	public:
	ndVector8 m_x;
	ndVector8 m_y;
	ndVector8 m_z;
} D_GCC_NEWTON_CLASS_ALIGN_32;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndSoaVector6
{
	public:
	ndSoaVector3 m_linear;
	ndSoaVector3 m_angular;
} D_GCC_NEWTON_CLASS_ALIGN_32;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndSoaJacobianPair
{
	public:
	ndSoaVector6 m_jacobianM0;
	ndSoaVector6 m_jacobianM1;
}D_GCC_NEWTON_CLASS_ALIGN_32;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndSoaMatrixElement
{
	public:
	ndSoaJacobianPair m_Jt;
	ndSoaJacobianPair m_JMinv;

	ndVector8 m_force;
	ndVector8 m_diagDamp;
	ndVector8 m_JinvMJt;
	ndVector8 m_invJinvMJt;
	ndVector8 m_coordenateAccel;
	ndVector8 m_normalForceIndex;
	ndVector8 m_lowerBoundFrictionCoefficent;
	ndVector8 m_upperBoundFrictionCoefficent;
} D_GCC_NEWTON_CLASS_ALIGN_32;

class ndSoaMatrixArray : public ndArray<ndSoaMatrixElement>
{
};

class ndSoaJointMaskArray : public ndArray<ndVector8>
{
};

ndDynamicsUpdateSoa::ndDynamicsUpdateSoa(ndWorld* const world)
	:ndDynamicsUpdate(world)
	,m_groupType(D_SOA_DEFAULT_BUFFER_SIZE)
	,m_soaJointRows(D_SOA_DEFAULT_BUFFER_SIZE)
	,m_jointMask(new ndSoaJointMaskArray)
	,m_soaMassMatrixArray(new ndSoaMatrixArray)
{
}

ndDynamicsUpdateSoa::~ndDynamicsUpdateSoa()
{
	Clear();

	m_groupType.Resize(D_SOA_DEFAULT_BUFFER_SIZE);
	m_soaJointRows.Resize(D_SOA_DEFAULT_BUFFER_SIZE);

	delete m_jointMask;
	delete m_soaMassMatrixArray;
}

const char* ndDynamicsUpdateSoa::GetStringId() const
{
	return "Soa";
}

void ndDynamicsUpdateSoa::SortJoints()
{
	D_TRACKTIME();
	SortJointsScan();
	if (!m_activeJointCount)
	{
		return;
	}

	ndScene* const scene = m_world->GetScene();
	ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

#ifdef _DEBUG
	for (ndInt32 i = 1; i < m_activeJointCount; ++i)
	{
		ndConstraint* const joint0 = jointArray[i - 1];
		ndConstraint* const joint1 = jointArray[i - 0];
		ndAssert(!joint0->m_resting);
		ndAssert(!joint1->m_resting);
		ndAssert(joint0->m_rowCount >= joint1->m_rowCount);
		ndAssert(!(joint0->GetBody0()->m_equilibrium0 & joint0->GetBody1()->m_equilibrium0));
		ndAssert(!(joint1->GetBody0()->m_equilibrium0 & joint1->GetBody1()->m_equilibrium0));
	}

	for (ndInt32 i = m_activeJointCount + 1; i < ndInt32(jointArray.GetCount()); ++i)
	{
		ndConstraint* const joint0 = jointArray[i - 1];
		ndConstraint* const joint1 = jointArray[i - 0];
		ndAssert(joint0->m_resting);
		ndAssert(joint1->m_resting);
		ndAssert(joint0->m_rowCount >= joint1->m_rowCount);
		ndAssert(joint0->GetBody0()->m_equilibrium0 & joint0->GetBody1()->m_equilibrium0);
		ndAssert(joint1->GetBody0()->m_equilibrium0 & joint1->GetBody1()->m_equilibrium0);
	}
#endif

	const ndInt32 mask = -ndInt32(ND_SIMD8_WORK_GROUP_SIZE);
	const ndInt32 jointCount = ndInt32(jointArray.GetCount());
	const ndInt32 soaJointCount = (jointCount + ND_SIMD8_WORK_GROUP_SIZE - 1) & mask;
	ndAssert(jointArray.GetCapacity() > soaJointCount);
	ndConstraint** const jointArrayPtr = &jointArray[0];
	for (ndInt32 i = jointCount; i < soaJointCount; ++i)
	{
		jointArrayPtr[i] = nullptr;
	}

	if (m_activeJointCount - jointArray.GetCount())
	{
		const ndInt32 base = m_activeJointCount & mask;
		const ndInt32 count = jointArrayPtr[base + ND_SIMD8_WORK_GROUP_SIZE - 1] ? ND_SIMD8_WORK_GROUP_SIZE : ndInt32(jointArray.GetCount()) - base;
		ndAssert(count <= ND_SIMD8_WORK_GROUP_SIZE);
		ndConstraint** const array = &jointArrayPtr[base];
		for (ndInt32 j = 1; j < count; ++j)
		{
			ndInt32 slot = j;
			ndConstraint* const joint = array[slot];
			for (; (slot > 0) && array[slot - 1] && (array[slot - 1]->m_rowCount < joint->m_rowCount); slot--)
			{
				array[slot] = array[slot - 1];
			}
			array[slot] = joint;
		}
	}

	const ndInt32 soaJointCountBatches = soaJointCount / ND_SIMD8_WORK_GROUP_SIZE;
	m_jointMask->SetCount(soaJointCountBatches);
	m_groupType.SetCount(soaJointCountBatches);
	m_soaJointRows.SetCount(soaJointCountBatches);

	ndInt32 rowsCount = 0;
	ndInt32 soaJointRowCount = 0;
	auto SetRowStarts = ndMakeObject::ndFunction([this, &jointArray, &rowsCount, &soaJointRowCount](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(SetRowStarts);
		auto SetRowsCount = [&jointArray, &rowsCount]()
		{
			ndInt32 rowCount = 1;
			const ndInt32 count = ndInt32(jointArray.GetCount());
			for (ndInt32 i = 0; i < count; ++i)
			{
				ndConstraint* const joint = jointArray[i];
				joint->m_rowStart = rowCount;
				rowCount += joint->m_rowCount;
			}
			rowsCount = rowCount;
		};

		auto SetSoaRowsCount = [this, &jointArray, &soaJointRowCount]()
		{
			ndInt32 rowCount = 0;
			ndArray<ndInt32>& soaJointRows = m_soaJointRows;
			const ndInt32 count = ndInt32(soaJointRows.GetCount());
			for (ndInt32 i = 0; i < count; ++i)
			{
				const ndConstraint* const joint = jointArray[i * ND_SIMD8_WORK_GROUP_SIZE];
				soaJointRows[i] = rowCount;
				rowCount += joint->m_rowCount;
			}
			soaJointRowCount = rowCount;
		};

		if (groupId == 0)
		{
			SetRowsCount();
		}
		else if (groupId == 1)
		{
			SetSoaRowsCount();
		}
	});
	scene->ParallelExecute(SetRowStarts, 2, 1);

	m_leftHandSide.SetCount(rowsCount);
	m_rightHandSide.SetCount(rowsCount);
	m_soaMassMatrixArray->SetCount(soaJointRowCount);

#ifdef _DEBUG
	ndAssert(m_activeJointCount <= jointArray.GetCount());
	const ndInt32 maxRowCount = ndInt32(m_leftHandSide.GetCount());
	for (ndInt32 i = 0; i < ndInt32(jointArray.GetCount()); ++i)
	{
		ndConstraint* const joint = jointArray[i];
		ndAssert(joint->m_rowStart < ndInt32(m_leftHandSide.GetCount()));
		ndAssert((joint->m_rowStart + joint->m_rowCount) <= maxRowCount);
	}

	for (ndInt32 i = 0; i < jointCount; i += ND_SIMD8_WORK_GROUP_SIZE)
	{
		const ndInt32 count = jointArrayPtr[i + ND_SIMD8_WORK_GROUP_SIZE - 1] ? ND_SIMD8_WORK_GROUP_SIZE : jointCount - i;
		for (ndInt32 j = 1; j < count; ++j)
		{
			ndConstraint* const joint0 = jointArrayPtr[i + j - 1];
			ndConstraint* const joint1 = jointArrayPtr[i + j - 0];
			ndAssert(joint0->m_rowCount >= joint1->m_rowCount);
		}
	}
#endif
	SortBodyJointScan();
}

void ndDynamicsUpdateSoa::SortIslands()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	ndArray<ndBodyKinematic*>& activeBodyArray = GetBodyIslandOrder();
	GetInternalForces().SetCount(bodyArray.GetCount());
	activeBodyArray.SetCount(bodyArray.GetCount());

	const ndInt32 minGroupSize = 1024;
	const ndInt32 threadCount = scene->GetThreadCount();
	const ndInt32 numberOfGroups = ndInt32(bodyArray.GetCount());
	const ndInt32 itemsPerThreads = (numberOfGroups + threadCount - 1) / threadCount;
	const ndInt32 groupSize = (itemsPerThreads < minGroupSize) ? minGroupSize : itemsPerThreads;
	const ndInt32 numberOfWorkGroups = (numberOfGroups + groupSize - 1) / groupSize;
	ndAssert(numberOfWorkGroups <= threadCount);

	ndInt32 histogram[D_MAX_THREADS_COUNT][3];

	auto Scan0 = ndMakeObject::ndFunction([&bodyArray, &histogram, groupSize](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(Scan0);
		ndInt32* const hist = &histogram[groupId][0];
		hist[0] = 0;
		hist[1] = 0;
		hist[2] = 0;

		ndInt32 map[4];
		map[0] = 0;
		map[1] = 1;
		map[2] = 2;
		map[3] = 2;

		const ndInt32 size = ndInt32(bodyArray.GetCount());
		const ndInt32 start = groupId * groupSize;
		const ndInt32 count = ((start + groupSize) < size) ? groupSize : size - start;
		for (ndInt32 i = 0; i < count; ++i)
		{
			ndBodyKinematic* const body = bodyArray[start + i];
			ndInt32 key = map[body->m_equilibrium0 * 2 + 1 - body->m_isConstrained];
			ndAssert(key < 3);
			hist[key] = hist[key] + 1;
		}
	});
	scene->ParallelExecute(Scan0, numberOfWorkGroups, 1);

	ndInt32 scan[3];
	scan[0] = 0;
	scan[1] = 0;
	scan[2] = 0;

	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < 3; ++i)
	{
		for (ndInt32 j = 0; j < numberOfWorkGroups; ++j)
		{
			ndInt32 partialSum = histogram[j][i];
			histogram[j][i] = sum;
			sum += partialSum;
		}
		scan[i] = sum;
	}

	auto Sort0 = ndMakeObject::ndFunction([&bodyArray, &activeBodyArray, &histogram, groupSize](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(Sort0);
		ndInt32* const hist = &histogram[groupId][0];

		ndInt32 map[4];
		map[0] = 0;
		map[1] = 1;
		map[2] = 2;
		map[3] = 2;

		const ndInt32 size = ndInt32(bodyArray.GetCount());
		const ndInt32 start = groupId * groupSize;
		const ndInt32 count = ((start + groupSize) < size) ? groupSize : size - start;
		for (ndInt32 i = 0; i < count; ++i)
		{
			ndBodyKinematic* const body = bodyArray[start + i];
			ndInt32 key = map[body->m_equilibrium0 * 2 + 1 - body->m_isConstrained];
			ndAssert(key < 3);
			const ndInt32 entry = hist[key];
			activeBodyArray[entry] = body;
			hist[key] = entry + 1;
		}
	});
	scene->ParallelExecute(Sort0, numberOfWorkGroups, 1);

	activeBodyArray.SetCount(scan[1]);
	m_unConstrainedBodyCount = scan[1] - scan[0];
}

void ndDynamicsUpdateSoa::BuildIsland()
{
	m_unConstrainedBodyCount = 0;
	GetBodyIslandOrder().SetCount(0);
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	ndAssert(bodyArray.GetCount() >= 1);
	if (bodyArray.GetCount() - 1)
	{
		D_TRACKTIME();
		SortJoints();
		SortIslands();
	}
}

void ndDynamicsUpdateSoa::InitJacobianMatrix()
{
	ndScene* const scene = m_world->GetScene();
	ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

	auto TransposeMassMatrix = ndMakeObject::ndFunction([this, &jointArray](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(TransposeMassMatrix);
		const ndLeftHandSide* const leftHandSide = &GetLeftHandSide()[0];
		const ndRightHandSide* const rightHandSide = &GetRightHandSide()[0];
		ndSoaMatrixArray& massMatrix = *m_soaMassMatrixArray;

		ndInt8* const groupType = &m_groupType[0];
		ndVector8* const jointMask = &(*m_jointMask)[0];
		const ndInt32* const soaJointRows = &m_soaJointRows[0];

		ndConstraint** const jointsPtr = &jointArray[0];

		const ndInt32 m = groupId;
		const ndInt32 index = m * ND_SIMD8_WORK_GROUP_SIZE;
		ndInt32 maxRow = 0;
		ndInt32 minRow = 255;
		ndVector8 selectMask(-1);
		for (ndInt32 k = 0; k < ND_SIMD8_WORK_GROUP_SIZE; ++k)
		{
			ndConstraint* const joint = jointsPtr[index + k];
			if (joint)
			{
				const ndInt32 maxMask = (maxRow - joint->m_rowCount) >> 8;
				const ndInt32 minMask = (minRow - joint->m_rowCount) >> 8;
				maxRow = (maxMask & joint->m_rowCount) | (~maxMask & maxRow);
				minRow = (~minMask & joint->m_rowCount) | (minMask & minRow);
				if (!joint->m_rowCount)
				{
					selectMask[k] = ndFloat32(0.0f);
				}
			}
			else
			{
				minRow = 0;
				selectMask[k] = ndFloat32(0.0f);
			}
		}
		ndAssert(maxRow >= 0);
		ndAssert(minRow < 255);
		jointMask[m] = selectMask;

		const ndInt8 isUniformGroup = (maxRow == minRow) & (maxRow > 0);
		groupType[m] = isUniformGroup;

		const ndInt32 soaRowBase = soaJointRows[m];
		if (isUniformGroup)
		{
			const ndConstraint* const joint0 = jointsPtr[index + 0];
			const ndConstraint* const joint1 = jointsPtr[index + 1];
			const ndConstraint* const joint2 = jointsPtr[index + 2];
			const ndConstraint* const joint3 = jointsPtr[index + 3];
			const ndConstraint* const joint4 = jointsPtr[index + 4];
			const ndConstraint* const joint5 = jointsPtr[index + 5];
			const ndConstraint* const joint6 = jointsPtr[index + 6];
			const ndConstraint* const joint7 = jointsPtr[index + 7];

			ndVector8 dommy;
			const ndInt32 rowCount = joint0->m_rowCount;
			for (ndInt32 k = 0; k < rowCount; ++k)
			{
				const ndLeftHandSide* const row0 = &leftHandSide[joint0->m_rowStart + k];
				const ndLeftHandSide* const row1 = &leftHandSide[joint1->m_rowStart + k];
				const ndLeftHandSide* const row2 = &leftHandSide[joint2->m_rowStart + k];
				const ndLeftHandSide* const row3 = &leftHandSide[joint3->m_rowStart + k];
				const ndLeftHandSide* const row4 = &leftHandSide[joint4->m_rowStart + k];
				const ndLeftHandSide* const row5 = &leftHandSide[joint5->m_rowStart + k];
				const ndLeftHandSide* const row6 = &leftHandSide[joint6->m_rowStart + k];
				const ndLeftHandSide* const row7 = &leftHandSide[joint7->m_rowStart + k];
				ndSoaMatrixElement& row = massMatrix[soaRowBase + k];

				ndVector8::Transpose(
					row.m_Jt.m_jacobianM0.m_linear.m_x,
					row.m_Jt.m_jacobianM0.m_linear.m_y,
					row.m_Jt.m_jacobianM0.m_linear.m_z,
					dommy,
					row.m_Jt.m_jacobianM0.m_angular.m_x,
					row.m_Jt.m_jacobianM0.m_angular.m_y,
					row.m_Jt.m_jacobianM0.m_angular.m_z,
					dommy,
					(ndVector8&)row0->m_Jt.m_jacobianM0,
					(ndVector8&)row1->m_Jt.m_jacobianM0,
					(ndVector8&)row2->m_Jt.m_jacobianM0,
					(ndVector8&)row3->m_Jt.m_jacobianM0,
					(ndVector8&)row4->m_Jt.m_jacobianM0,
					(ndVector8&)row5->m_Jt.m_jacobianM0,
					(ndVector8&)row6->m_Jt.m_jacobianM0,
					(ndVector8&)row7->m_Jt.m_jacobianM0);

				ndVector8::Transpose(
					row.m_Jt.m_jacobianM1.m_linear.m_x,
					row.m_Jt.m_jacobianM1.m_linear.m_y,
					row.m_Jt.m_jacobianM1.m_linear.m_z,
					dommy,
					row.m_Jt.m_jacobianM1.m_angular.m_x,
					row.m_Jt.m_jacobianM1.m_angular.m_y,
					row.m_Jt.m_jacobianM1.m_angular.m_z,
					dommy,
					(ndVector8&)row0->m_Jt.m_jacobianM1,
					(ndVector8&)row1->m_Jt.m_jacobianM1,
					(ndVector8&)row2->m_Jt.m_jacobianM1,
					(ndVector8&)row3->m_Jt.m_jacobianM1,
					(ndVector8&)row4->m_Jt.m_jacobianM1,
					(ndVector8&)row5->m_Jt.m_jacobianM1,
					(ndVector8&)row6->m_Jt.m_jacobianM1,
					(ndVector8&)row7->m_Jt.m_jacobianM1);

				ndVector8::Transpose(
					row.m_JMinv.m_jacobianM0.m_linear.m_x,
					row.m_JMinv.m_jacobianM0.m_linear.m_y,
					row.m_JMinv.m_jacobianM0.m_linear.m_z,
					dommy,
					row.m_JMinv.m_jacobianM0.m_angular.m_x,
					row.m_JMinv.m_jacobianM0.m_angular.m_y,
					row.m_JMinv.m_jacobianM0.m_angular.m_z,
					dommy,
					(ndVector8&)row0->m_JMinv.m_jacobianM0,
					(ndVector8&)row1->m_JMinv.m_jacobianM0,
					(ndVector8&)row2->m_JMinv.m_jacobianM0,
					(ndVector8&)row3->m_JMinv.m_jacobianM0,
					(ndVector8&)row4->m_JMinv.m_jacobianM0,
					(ndVector8&)row5->m_JMinv.m_jacobianM0,
					(ndVector8&)row6->m_JMinv.m_jacobianM0,
					(ndVector8&)row7->m_JMinv.m_jacobianM0);

				ndVector8::Transpose(
					row.m_JMinv.m_jacobianM1.m_linear.m_x,
					row.m_JMinv.m_jacobianM1.m_linear.m_y,
					row.m_JMinv.m_jacobianM1.m_linear.m_z,
					dommy,
					row.m_JMinv.m_jacobianM1.m_angular.m_x,
					row.m_JMinv.m_jacobianM1.m_angular.m_y,
					row.m_JMinv.m_jacobianM1.m_angular.m_z,
					dommy,
					(ndVector8&)row0->m_JMinv.m_jacobianM1,
					(ndVector8&)row1->m_JMinv.m_jacobianM1,
					(ndVector8&)row2->m_JMinv.m_jacobianM1,
					(ndVector8&)row3->m_JMinv.m_jacobianM1,
					(ndVector8&)row4->m_JMinv.m_jacobianM1,
					(ndVector8&)row5->m_JMinv.m_jacobianM1,
					(ndVector8&)row6->m_JMinv.m_jacobianM1,
					(ndVector8&)row7->m_JMinv.m_jacobianM1);

				#ifdef D_NEWTON_USE_DOUBLE
					ndInt64* const normalIndex = (ndInt64*)&row.m_normalForceIndex[0];
				#else
					ndInt32* const normalIndex = (ndInt32*)&row.m_normalForceIndex[0];
				#endif
				for (ndInt32 n = 0; n < ND_SIMD8_WORK_GROUP_SIZE; ++n)
				{
					const ndConstraint* const soaJoint = jointsPtr[index + n];
					const ndRightHandSide* const rhs = &rightHandSide[soaJoint->m_rowStart + k];
					row.m_force[n] = rhs->m_force;
					row.m_diagDamp[n] = rhs->m_diagDamp;
					row.m_JinvMJt[n] = rhs->m_JinvMJt;
					row.m_invJinvMJt[n] = rhs->m_invJinvMJt;
					row.m_coordenateAccel[n] = rhs->m_coordenateAccel;
					normalIndex[n] = (rhs->m_normalForceIndex + 1) * ND_SIMD8_WORK_GROUP_SIZE + n;
					row.m_lowerBoundFrictionCoefficent[n] = rhs->m_lowerBoundFrictionCoefficent;
					row.m_upperBoundFrictionCoefficent[n] = rhs->m_upperBoundFrictionCoefficent;
					ndAssert(rhs->SanityCheck());
				}
			}
		}
		else
		{
			const ndVector8 zero(ndVector8::m_zero);
			const ndVector8 ordinals(ndVector8::m_ordinals);
			const ndConstraint* const firstJoint = jointsPtr[index];
			for (ndInt32 k = 0; k < firstJoint->m_rowCount; ++k)
			{
				ndSoaMatrixElement& row = massMatrix[soaRowBase + k];
				row.m_Jt.m_jacobianM0.m_linear.m_x = zero;
				row.m_Jt.m_jacobianM0.m_linear.m_y = zero;
				row.m_Jt.m_jacobianM0.m_linear.m_z = zero;
				row.m_Jt.m_jacobianM0.m_angular.m_x = zero;
				row.m_Jt.m_jacobianM0.m_angular.m_y = zero;
				row.m_Jt.m_jacobianM0.m_angular.m_z = zero;
				row.m_Jt.m_jacobianM1.m_linear.m_x = zero;
				row.m_Jt.m_jacobianM1.m_linear.m_y = zero;
				row.m_Jt.m_jacobianM1.m_linear.m_z = zero;
				row.m_Jt.m_jacobianM1.m_angular.m_x = zero;
				row.m_Jt.m_jacobianM1.m_angular.m_y = zero;
				row.m_Jt.m_jacobianM1.m_angular.m_z = zero;

				row.m_JMinv.m_jacobianM0.m_linear.m_x = zero;
				row.m_JMinv.m_jacobianM0.m_linear.m_y = zero;
				row.m_JMinv.m_jacobianM0.m_linear.m_z = zero;
				row.m_JMinv.m_jacobianM0.m_angular.m_x = zero;
				row.m_JMinv.m_jacobianM0.m_angular.m_y = zero;
				row.m_JMinv.m_jacobianM0.m_angular.m_z = zero;
				row.m_JMinv.m_jacobianM1.m_linear.m_x = zero;
				row.m_JMinv.m_jacobianM1.m_linear.m_y = zero;
				row.m_JMinv.m_jacobianM1.m_linear.m_z = zero;
				row.m_JMinv.m_jacobianM1.m_angular.m_x = zero;
				row.m_JMinv.m_jacobianM1.m_angular.m_y = zero;
				row.m_JMinv.m_jacobianM1.m_angular.m_z = zero;

				row.m_force = zero;
				row.m_JinvMJt = zero;
				row.m_diagDamp = zero;
				row.m_invJinvMJt = zero;
				row.m_coordenateAccel = zero;
				row.m_normalForceIndex = ordinals;
				row.m_lowerBoundFrictionCoefficent = zero;
				row.m_upperBoundFrictionCoefficent = zero;
			}

			for (ndInt32 k = 0; k < ND_SIMD8_WORK_GROUP_SIZE; ++k)
			{
				const ndConstraint* const joint = jointsPtr[index + k];
				if (joint)
				{
					for (ndInt32 n = 0; n < joint->m_rowCount; ++n)
					{
						ndSoaMatrixElement& row = massMatrix[soaRowBase + n];
						const ndLeftHandSide* const lhs = &leftHandSide[joint->m_rowStart + n];

						row.m_Jt.m_jacobianM0.m_linear.m_x[k] = lhs->m_Jt.m_jacobianM0.m_linear.m_x;
						row.m_Jt.m_jacobianM0.m_linear.m_y[k] = lhs->m_Jt.m_jacobianM0.m_linear.m_y;
						row.m_Jt.m_jacobianM0.m_linear.m_z[k] = lhs->m_Jt.m_jacobianM0.m_linear.m_z;
						row.m_Jt.m_jacobianM0.m_angular.m_x[k] = lhs->m_Jt.m_jacobianM0.m_angular.m_x;
						row.m_Jt.m_jacobianM0.m_angular.m_y[k] = lhs->m_Jt.m_jacobianM0.m_angular.m_y;
						row.m_Jt.m_jacobianM0.m_angular.m_z[k] = lhs->m_Jt.m_jacobianM0.m_angular.m_z;
						row.m_Jt.m_jacobianM1.m_linear.m_x[k] = lhs->m_Jt.m_jacobianM1.m_linear.m_x;
						row.m_Jt.m_jacobianM1.m_linear.m_y[k] = lhs->m_Jt.m_jacobianM1.m_linear.m_y;
						row.m_Jt.m_jacobianM1.m_linear.m_z[k] = lhs->m_Jt.m_jacobianM1.m_linear.m_z;
						row.m_Jt.m_jacobianM1.m_angular.m_x[k] = lhs->m_Jt.m_jacobianM1.m_angular.m_x;
						row.m_Jt.m_jacobianM1.m_angular.m_y[k] = lhs->m_Jt.m_jacobianM1.m_angular.m_y;
						row.m_Jt.m_jacobianM1.m_angular.m_z[k] = lhs->m_Jt.m_jacobianM1.m_angular.m_z;

						row.m_JMinv.m_jacobianM0.m_linear.m_x[k] = lhs->m_JMinv.m_jacobianM0.m_linear.m_x;
						row.m_JMinv.m_jacobianM0.m_linear.m_y[k] = lhs->m_JMinv.m_jacobianM0.m_linear.m_y;
						row.m_JMinv.m_jacobianM0.m_linear.m_z[k] = lhs->m_JMinv.m_jacobianM0.m_linear.m_z;
						row.m_JMinv.m_jacobianM0.m_angular.m_x[k] = lhs->m_JMinv.m_jacobianM0.m_angular.m_x;
						row.m_JMinv.m_jacobianM0.m_angular.m_y[k] = lhs->m_JMinv.m_jacobianM0.m_angular.m_y;
						row.m_JMinv.m_jacobianM0.m_angular.m_z[k] = lhs->m_JMinv.m_jacobianM0.m_angular.m_z;
						row.m_JMinv.m_jacobianM1.m_linear.m_x[k] = lhs->m_JMinv.m_jacobianM1.m_linear.m_x;
						row.m_JMinv.m_jacobianM1.m_linear.m_y[k] = lhs->m_JMinv.m_jacobianM1.m_linear.m_y;
						row.m_JMinv.m_jacobianM1.m_linear.m_z[k] = lhs->m_JMinv.m_jacobianM1.m_linear.m_z;
						row.m_JMinv.m_jacobianM1.m_angular.m_x[k] = lhs->m_JMinv.m_jacobianM1.m_angular.m_x;
						row.m_JMinv.m_jacobianM1.m_angular.m_y[k] = lhs->m_JMinv.m_jacobianM1.m_angular.m_y;
						row.m_JMinv.m_jacobianM1.m_angular.m_z[k] = lhs->m_JMinv.m_jacobianM1.m_angular.m_z;

						const ndRightHandSide* const rhs = &rightHandSide[joint->m_rowStart + n];
						row.m_force[k] = rhs->m_force;
						row.m_diagDamp[k] = rhs->m_diagDamp;
						row.m_JinvMJt[k] = rhs->m_JinvMJt;
						row.m_invJinvMJt[k] = rhs->m_invJinvMJt;
						row.m_coordenateAccel[k] = rhs->m_coordenateAccel;

						#ifdef D_NEWTON_USE_DOUBLE
							ndInt64* const normalIndex = (ndInt64*)&row.m_normalForceIndex[0];
						#else
							ndInt32* const normalIndex = (ndInt32*)&row.m_normalForceIndex[0];
						#endif
						normalIndex[k] = (rhs->m_normalForceIndex + 1) * ND_SIMD8_WORK_GROUP_SIZE + k;
						row.m_lowerBoundFrictionCoefficent[k] = rhs->m_lowerBoundFrictionCoefficent;
						row.m_upperBoundFrictionCoefficent[k] = rhs->m_upperBoundFrictionCoefficent;
						ndAssert(rhs->SanityCheck());
					}
				}
			}
		}
	});

	if (scene->GetActiveContactArray().GetCount())
	{
		D_TRACKTIME();
		ndDynamicsUpdate::InitJacobianMatrix();

		const ndInt32 numberOfJoints = ndInt32(jointArray.GetCount());
		const ndInt32 mask = -ndInt32(ND_SIMD8_WORK_GROUP_SIZE);
		const ndInt32 soaJointCount = ((numberOfJoints + ND_SIMD8_WORK_GROUP_SIZE - 1) & mask) / ND_SIMD8_WORK_GROUP_SIZE;
		scene->ParallelExecute(TransposeMassMatrix, soaJointCount, 1);
	}
}

void ndDynamicsUpdateSoa::RegenerateSkeletonJacobians(ndSkeletonContainer* const skeleton)
{
	const ndInt32 nodeCount = skeleton->m_nodeList.GetCount();
	ndSkeletonContainer::ndNode** const nodeArray = skeleton->m_nodesOrder;

	ndConstraintDescritor constraintParam;
	constraintParam.m_timestep = m_timestep;
	constraintParam.m_invTimestep = m_invTimestep;

	ndVector8* const internalForces = (ndVector8*) &GetTempInternalForces()[0];
	auto BuildJacobianMatrix = [this, &internalForces](ndConstraint* const joint)
	{
		ndAssert(joint->GetBody0());
		ndAssert(joint->GetBody1());
		const ndBodyKinematic* const body0 = joint->GetBody0();
		const ndBodyKinematic* const body1 = joint->GetBody1();

		ndVector8 force0(body0->GetForce(), body0->GetTorque());
		ndVector8 force1(body1->GetForce(), body1->GetTorque());

		const ndInt32 index = joint->m_rowStart;
		const ndInt32 count = joint->m_rowCount;

		const bool isBilateral = joint->IsBilateral();

		const ndMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
		const ndMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;
		const ndVector invMass0(body0->m_invMass[3]);
		const ndVector invMass1(body1->m_invMass[3]);

		ndVector8 forceAcc0(ndVector8::m_zero);
		ndVector8 forceAcc1(ndVector8::m_zero);
		const ndVector8 weigh0(body0->m_weigh);
		const ndVector8 weigh1(body1->m_weigh);

		for (ndInt32 i = 0; i < count; ++i)
		{
			ndLeftHandSide* const row = &m_leftHandSide[index + i];
			ndRightHandSide* const rhs = &m_rightHandSide[index + i];

			row->m_JMinv.m_jacobianM0.m_linear = row->m_Jt.m_jacobianM0.m_linear * invMass0;
			row->m_JMinv.m_jacobianM0.m_angular = invInertia0.RotateVector(row->m_Jt.m_jacobianM0.m_angular);
			row->m_JMinv.m_jacobianM1.m_linear = row->m_Jt.m_jacobianM1.m_linear * invMass1;
			row->m_JMinv.m_jacobianM1.m_angular = invInertia1.RotateVector(row->m_Jt.m_jacobianM1.m_angular);

			const ndVector8& JMinvM0 = (ndVector8&)row->m_JMinv.m_jacobianM0;
			const ndVector8& JMinvM1 = (ndVector8&)row->m_JMinv.m_jacobianM1;

			const ndVector8 tmpAccel((JMinvM0 * force0).MulAdd(JMinvM1, force1));

			ndFloat32 extenalAcceleration = -tmpAccel.AddHorizontal();
			rhs->m_deltaAccel = extenalAcceleration;
			rhs->m_coordenateAccel += extenalAcceleration;
			ndAssert(rhs->m_jointFeebackForce);
			const ndFloat32 force = rhs->m_jointFeebackForce->GetInitialGuess();

			ndAssert(rhs->SanityCheck());
			rhs->m_force = isBilateral ? ndClamp(force, rhs->m_lowerBoundFrictionCoefficent, rhs->m_upperBoundFrictionCoefficent) : force;
			rhs->m_maxImpact = ndFloat32(0.0f);

			const ndVector8& JtM0 = (ndVector8&)row->m_Jt.m_jacobianM0;
			const ndVector8& JtM1 = (ndVector8&)row->m_Jt.m_jacobianM1;
			const ndVector8 tmpDiag(weigh0 * JMinvM0 * JtM0 + weigh1 * JMinvM1 * JtM1);

			ndFloat32 diag = tmpDiag.AddHorizontal();
			ndAssert(diag > ndFloat32(0.0f));
			rhs->m_diagDamp = diag * rhs->m_diagonalRegularizer;

			diag *= (ndFloat32(1.0f) + rhs->m_diagonalRegularizer);
			rhs->m_JinvMJt = diag;
			rhs->m_invJinvMJt = ndFloat32(1.0f) / diag;

			forceAcc0 = forceAcc0.MulAdd(JtM0, ndVector8(rhs->m_force));
			forceAcc1 = forceAcc1.MulAdd(JtM1, ndVector8(rhs->m_force));
		}

		const ndInt32 jointIndex = joint->m_forceIndex;

		const ndInt32 index0 = jointIndex * 2 + 0;
		ndVector8& outBody0 = internalForces[index0];
		outBody0 = forceAcc0;

		const ndInt32 index1 = jointIndex * 2 + 1;
		ndVector8& outBody1 = internalForces[index1];
		outBody1 = forceAcc1;
	};

	for (ndInt32 j = 0; j < nodeCount - 1; ++j)
	{
		ndSkeletonContainer::ndNode* const node = nodeArray[j];
		ndJointBilateralConstraint* const joint = node->m_joint;
		ndAssert(joint);
		ndAssert(joint->GetRowsCount() <= D_CONSTRAINT_MAX_ROWS);
		for (ndInt32 i = ndInt32(joint->GetRowsCount() - 1); i >= 0; i--)
		{
			constraintParam.m_forceBounds[i].m_low = D_MIN_BOUND;
			constraintParam.m_forceBounds[i].m_upper = D_MAX_BOUND;
			constraintParam.m_forceBounds[i].m_jointForce = nullptr;
			constraintParam.m_forceBounds[i].m_normalIndex = D_INDEPENDENT_ROW;
		}

		constraintParam.m_rowsCount = 0;
		joint->JacobianDerivative(constraintParam);
		const ndInt32 dof = constraintParam.m_rowsCount;
		ndAssert(dof <= joint->m_rowCount);

		const ndInt32 baseIndex = joint->m_rowStart;
		for (ndInt32 i = 0; i < dof; ++i)
		{
			ndAssert(constraintParam.m_forceBounds[i].m_jointForce);

			ndLeftHandSide* const row = &m_leftHandSide[baseIndex + i];
			ndRightHandSide* const rhs = &m_rightHandSide[baseIndex + i];

			row->m_Jt = constraintParam.m_jacobian[i];
			rhs->m_diagDamp = ndFloat32(0.0f);
			rhs->m_diagonalRegularizer = ndMax(constraintParam.m_diagonalRegularizer[i], ndFloat32(1.0e-5f));

			rhs->m_positError = constraintParam.m_positError[i];
			rhs->m_speedError = constraintParam.m_speedError[i];
			rhs->m_coordenateAccel = constraintParam.m_jointAccel[i];
			rhs->m_restitution = constraintParam.m_restitution[i];
			rhs->m_penetration = constraintParam.m_penetration[i];
			rhs->m_penetrationStiffness = constraintParam.m_penetrationStiffness[i];
			rhs->m_lowerBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_low;
			rhs->m_upperBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_upper;
			rhs->m_jointFeebackForce = constraintParam.m_forceBounds[i].m_jointForce;

			ndAssert(constraintParam.m_forceBounds[i].m_normalIndex >= -2);
			rhs->m_normalForceIndex = constraintParam.m_forceBounds[i].m_normalIndex;

			ndAssert(rhs->SanityCheck());
			if (rhs->m_normalForceIndex == D_OVERRIDE_FRICTION_ROW)
			{
				rhs->m_normalForceIndex = D_INDEPENDENT_ROW;
			}
		}
		BuildJacobianMatrix(joint);
	}

	const ndVector8 zero(ndVector8::m_zero);
	const ndArray<ndInt32>& bodyIndex = GetJointForceIndexBuffer();
	const ndVector8* const jointInternalForces = (ndVector8*)&GetTempInternalForces()[0];
	const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &GetJointBodyPairIndexBuffer()[0];
	for (ndInt32 j = 0; j < nodeCount; ++j)
	{
		ndVector8 forceTorque(zero);

		ndSkeletonContainer::ndNode* const node = nodeArray[j];
		ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
		ndAssert(body);

		const ndInt32 m = body->m_index;
		const ndInt32 index = bodyIndex[m];

		#ifdef _DEBUG
			ndBodyKinematic** const bodyArray = &m_world->GetScene()->GetActiveBodyArray()[0];
			const ndJointBodyPairIndex* const jointBodyPairIndex = &GetJointBodyPairIndexBuffer()[0];
			const ndJointBodyPairIndex& scan = jointBodyPairIndex[index];
			ndAssert(body == bodyArray[scan.m_body]);
		#endif

		ndAssert(!body->m_isStatic);
		//const ndInt32 mask = ndInt32(body->m_isStatic) - 1;
		//const ndInt32 count = mask & (bodyIndex[m + 1] - index);
		const ndInt32 count = bodyIndex[m + 1] - index;

		for (ndInt32 k = 0; k < count; ++k)
		{
			const ndInt32 jointIndex = jointBodyPairIndexBuffer[index + k].m_joint;
			forceTorque = forceTorque + jointInternalForces[jointIndex];
		}
		internalForces[m] = forceTorque;
	}

	const ndLeftHandSide* const leftHandSide = &GetLeftHandSide()[0];
	const ndRightHandSide* const rightHandSide = &GetRightHandSide()[0];
	ndSoaMatrixArray& massMatrix = *m_soaMassMatrixArray;

	const ndVector8 ordinals(ndVector8::m_ordinals);
	const ndInt32* const soaJointRows = &m_soaJointRows[0];
	//const ndArray<ndConstraint*>& jointArray = m_world->GetScene()->GetActiveContactArray();
	for (ndInt32 j = 0; j < nodeCount - 1; ++j)
	{
		ndSkeletonContainer::ndNode* const node = nodeArray[j];
		ndJointBilateralConstraint* const joint = node->m_joint;
		ndAssert(joint);
		
		const ndInt32 m = joint->m_forceIndex / ND_SIMD8_WORK_GROUP_SIZE;
		const ndInt32 soaRowBase = soaJointRows[m];

		const ndInt32 k = joint->m_forceIndex - m * ND_SIMD8_WORK_GROUP_SIZE;
		ndAssert(joint == m_world->GetScene()->GetActiveContactArray()[joint->m_forceIndex]);
		for (ndInt32 n = 0; n < joint->m_rowCount; ++n)
		{
			ndSoaMatrixElement& row = massMatrix[soaRowBase + n];
			const ndLeftHandSide* const lhs = &leftHandSide[joint->m_rowStart + n];

			row.m_Jt.m_jacobianM0.m_linear.m_x[k] = lhs->m_Jt.m_jacobianM0.m_linear.m_x;
			row.m_Jt.m_jacobianM0.m_linear.m_y[k] = lhs->m_Jt.m_jacobianM0.m_linear.m_y;
			row.m_Jt.m_jacobianM0.m_linear.m_z[k] = lhs->m_Jt.m_jacobianM0.m_linear.m_z;
			row.m_Jt.m_jacobianM0.m_angular.m_x[k] = lhs->m_Jt.m_jacobianM0.m_angular.m_x;
			row.m_Jt.m_jacobianM0.m_angular.m_y[k] = lhs->m_Jt.m_jacobianM0.m_angular.m_y;
			row.m_Jt.m_jacobianM0.m_angular.m_z[k] = lhs->m_Jt.m_jacobianM0.m_angular.m_z;
			row.m_Jt.m_jacobianM1.m_linear.m_x[k] = lhs->m_Jt.m_jacobianM1.m_linear.m_x;
			row.m_Jt.m_jacobianM1.m_linear.m_y[k] = lhs->m_Jt.m_jacobianM1.m_linear.m_y;
			row.m_Jt.m_jacobianM1.m_linear.m_z[k] = lhs->m_Jt.m_jacobianM1.m_linear.m_z;
			row.m_Jt.m_jacobianM1.m_angular.m_x[k] = lhs->m_Jt.m_jacobianM1.m_angular.m_x;
			row.m_Jt.m_jacobianM1.m_angular.m_y[k] = lhs->m_Jt.m_jacobianM1.m_angular.m_y;
			row.m_Jt.m_jacobianM1.m_angular.m_z[k] = lhs->m_Jt.m_jacobianM1.m_angular.m_z;

			row.m_JMinv.m_jacobianM0.m_linear.m_x[k] = lhs->m_JMinv.m_jacobianM0.m_linear.m_x;
			row.m_JMinv.m_jacobianM0.m_linear.m_y[k] = lhs->m_JMinv.m_jacobianM0.m_linear.m_y;
			row.m_JMinv.m_jacobianM0.m_linear.m_z[k] = lhs->m_JMinv.m_jacobianM0.m_linear.m_z;
			row.m_JMinv.m_jacobianM0.m_angular.m_x[k] = lhs->m_JMinv.m_jacobianM0.m_angular.m_x;
			row.m_JMinv.m_jacobianM0.m_angular.m_y[k] = lhs->m_JMinv.m_jacobianM0.m_angular.m_y;
			row.m_JMinv.m_jacobianM0.m_angular.m_z[k] = lhs->m_JMinv.m_jacobianM0.m_angular.m_z;
			row.m_JMinv.m_jacobianM1.m_linear.m_x[k] = lhs->m_JMinv.m_jacobianM1.m_linear.m_x;
			row.m_JMinv.m_jacobianM1.m_linear.m_y[k] = lhs->m_JMinv.m_jacobianM1.m_linear.m_y;
			row.m_JMinv.m_jacobianM1.m_linear.m_z[k] = lhs->m_JMinv.m_jacobianM1.m_linear.m_z;
			row.m_JMinv.m_jacobianM1.m_angular.m_x[k] = lhs->m_JMinv.m_jacobianM1.m_angular.m_x;
			row.m_JMinv.m_jacobianM1.m_angular.m_y[k] = lhs->m_JMinv.m_jacobianM1.m_angular.m_y;
			row.m_JMinv.m_jacobianM1.m_angular.m_z[k] = lhs->m_JMinv.m_jacobianM1.m_angular.m_z;

			const ndRightHandSide* const rhs = &rightHandSide[joint->m_rowStart + n];
			row.m_force[k] = rhs->m_force;
			row.m_diagDamp[k] = rhs->m_diagDamp;
			row.m_JinvMJt[k] = rhs->m_JinvMJt;
			row.m_invJinvMJt[k] = rhs->m_invJinvMJt;
			row.m_coordenateAccel[k] = rhs->m_coordenateAccel;

			#ifdef D_NEWTON_USE_DOUBLE
				ndInt64* const normalIndex = (ndInt64*)&row.m_normalForceIndex[0];
			#else
				ndInt32* const normalIndex = (ndInt32*)&row.m_normalForceIndex[0];
			#endif
			normalIndex[k] = (rhs->m_normalForceIndex + 1) * ND_SIMD8_WORK_GROUP_SIZE + k;
			row.m_lowerBoundFrictionCoefficent[k] = rhs->m_lowerBoundFrictionCoefficent;
			row.m_upperBoundFrictionCoefficent[k] = rhs->m_upperBoundFrictionCoefficent;
			ndAssert(rhs->SanityCheck());
		}
	}
}

void ndDynamicsUpdateSoa::CalculateJointsAcceleration()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();
	auto UpdateAcceleration = ndMakeObject::ndFunction([this, &jointArray](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(UpdateAcceleration);
		const ndArray<ndRightHandSide>& rightHandSide = m_rightHandSide;

		const ndInt32* const soaJointRows = &m_soaJointRows[0];

		const ndInt8* const groupType = &m_groupType[0];

		const ndConstraint* const* jointArrayPtr = &jointArray[0];
		ndSoaMatrixArray& massMatrix = *m_soaMassMatrixArray;

		const ndInt32 m = groupId;
		if (groupType[m])
		{
			const ndInt32 soaRowStartBase = soaJointRows[m];
			const ndConstraint* const* jointGroup = &jointArrayPtr[m * ND_SIMD8_WORK_GROUP_SIZE];
			const ndConstraint* const firstJoint = jointGroup[0];
			const ndInt32 rowCount = firstJoint->m_rowCount;
			for (ndInt32 k = 0; k < ND_SIMD8_WORK_GROUP_SIZE; ++k)
			{
				const ndConstraint* const Joint = jointGroup[k];
				const ndInt32 base = Joint->m_rowStart;
				for (ndInt32 n = 0; n < rowCount; ++n)
				{
					ndSoaMatrixElement* const row = &massMatrix[soaRowStartBase + n];
					row->m_coordenateAccel[k] = rightHandSide[base + n].m_coordenateAccel;
				}
			}
		}
		else
		{
			const ndInt32 soaRowStartBase = soaJointRows[m];
			const ndConstraint* const* jointGroup = &jointArrayPtr[m * ND_SIMD8_WORK_GROUP_SIZE];
			for (ndInt32 k = 0; k < ND_SIMD8_WORK_GROUP_SIZE; ++k)
			{
				const ndConstraint* const Joint = jointGroup[k];
				if (Joint)
				{
					const ndInt32 base = Joint->m_rowStart;
					const ndInt32 rowCount = Joint->m_rowCount;
					for (ndInt32 n = 0; n < rowCount; ++n)
					{
						ndSoaMatrixElement* const row = &massMatrix[soaRowStartBase + n];
						row->m_coordenateAccel[k] = rightHandSide[base + n].m_coordenateAccel;
					}
				}
			}
		}
	});
	ndDynamicsUpdate::CalculateJointsAcceleration();

	const ndInt32 mask = -ndInt32(ND_SIMD8_WORK_GROUP_SIZE);
	const ndInt32 jointCount = ndInt32(jointArray.GetCount());
	const ndInt32 soaJointCountBatches = ((jointCount + ND_SIMD8_WORK_GROUP_SIZE - 1) & mask) / ND_SIMD8_WORK_GROUP_SIZE;
	scene->ParallelExecute(UpdateAcceleration, soaJointCountBatches, scene->OptimalGroupBatch(soaJointCountBatches));
}

void ndDynamicsUpdateSoa::CalculateJointsForce()
{
	D_TRACKTIME();
	const ndUnsigned32 passes = m_solverPasses;
	ndScene* const scene = m_world->GetScene();

	ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

	auto CalculateJointsForce = ndMakeObject::ndFunction([this, &jointArray](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(CalculateJointsForce);
		ndVector8* const jointPartialForces = (ndVector8*)&GetTempInternalForces()[0];

		const ndInt32* const soaJointRows = &m_soaJointRows[0];
		ndSoaMatrixArray& soaMassMatrixArray = *m_soaMassMatrixArray;
		ndSoaMatrixElement* const soaMassMatrix = &soaMassMatrixArray[0];

		auto JointForce = [this, &jointArray, jointPartialForces](ndInt32 group, ndSoaMatrixElement* const massMatrix)
		{
			ndSoaVector6 forceM0;
			ndSoaVector6 forceM1;
			ndVector8 preconditioner0;
			ndVector8 preconditioner1;
			ndFixSizeArray<ndVector8, D_CONSTRAINT_MAX_ROWS + 1> normalForce(D_CONSTRAINT_MAX_ROWS + 1);

			const ndInt32 block = group * ND_SIMD8_WORK_GROUP_SIZE;
			ndConstraint** const jointGroup = &jointArray[block];

			ndVector8 zero(ndVector8::m_zero);
			const ndInt8 isUniformGroup = m_groupType[group];
			if (isUniformGroup)
			{
				for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
				{
					const ndConstraint* const joint = jointGroup[i];
					const ndBodyKinematic* const body0 = joint->GetBody0();
					const ndBodyKinematic* const body1 = joint->GetBody1();

					const ndInt32 m0 = body0->m_index;
					const ndInt32 m1 = body1->m_index;

					preconditioner0[i] = body0->m_weigh;
					preconditioner1[i] = body1->m_weigh;

					forceM0.m_linear.m_x[i] = m_internalForces[m0].m_linear.m_x;
					forceM0.m_linear.m_y[i] = m_internalForces[m0].m_linear.m_y;
					forceM0.m_linear.m_z[i] = m_internalForces[m0].m_linear.m_z;
					forceM0.m_angular.m_x[i] = m_internalForces[m0].m_angular.m_x;
					forceM0.m_angular.m_y[i] = m_internalForces[m0].m_angular.m_y;
					forceM0.m_angular.m_z[i] = m_internalForces[m0].m_angular.m_z;

					forceM1.m_linear.m_x[i] = m_internalForces[m1].m_linear.m_x;
					forceM1.m_linear.m_y[i] = m_internalForces[m1].m_linear.m_y;
					forceM1.m_linear.m_z[i] = m_internalForces[m1].m_linear.m_z;
					forceM1.m_angular.m_x[i] = m_internalForces[m1].m_angular.m_x;
					forceM1.m_angular.m_y[i] = m_internalForces[m1].m_angular.m_y;
					forceM1.m_angular.m_z[i] = m_internalForces[m1].m_angular.m_z;
				}
			}
			else
			{
				preconditioner0 = zero;
				preconditioner1 = zero;
				forceM0.m_linear.m_x = zero;
				forceM0.m_linear.m_y = zero;
				forceM0.m_linear.m_z = zero;
				forceM0.m_angular.m_x = zero;
				forceM0.m_angular.m_y = zero;
				forceM0.m_angular.m_z = zero;

				forceM1.m_linear.m_x = zero;
				forceM1.m_linear.m_y = zero;
				forceM1.m_linear.m_z = zero;
				forceM1.m_angular.m_x = zero;
				forceM1.m_angular.m_y = zero;
				forceM1.m_angular.m_z = zero;
				for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
				{
					const ndConstraint* const joint = jointGroup[i];
					if (joint && joint->m_rowCount)
					{
						const ndBodyKinematic* const body0 = joint->GetBody0();
						const ndBodyKinematic* const body1 = joint->GetBody1();

						const ndInt32 m0 = body0->m_index;
						const ndInt32 m1 = body1->m_index;
						preconditioner0[i] = body0->m_weigh;
						preconditioner1[i] = body1->m_weigh;

						forceM0.m_linear.m_x[i] = m_internalForces[m0].m_linear.m_x;
						forceM0.m_linear.m_y[i] = m_internalForces[m0].m_linear.m_y;
						forceM0.m_linear.m_z[i] = m_internalForces[m0].m_linear.m_z;
						forceM0.m_angular.m_x[i] = m_internalForces[m0].m_angular.m_x;
						forceM0.m_angular.m_y[i] = m_internalForces[m0].m_angular.m_y;
						forceM0.m_angular.m_z[i] = m_internalForces[m0].m_angular.m_z;

						forceM1.m_linear.m_x[i] = m_internalForces[m1].m_linear.m_x;
						forceM1.m_linear.m_y[i] = m_internalForces[m1].m_linear.m_y;
						forceM1.m_linear.m_z[i] = m_internalForces[m1].m_linear.m_z;
						forceM1.m_angular.m_x[i] = m_internalForces[m1].m_angular.m_x;
						forceM1.m_angular.m_y[i] = m_internalForces[m1].m_angular.m_y;
						forceM1.m_angular.m_z[i] = m_internalForces[m1].m_angular.m_z;
					}
				}
			}

			//normalForce[0] = ndVector8(ndFloat32(1.0f));
			normalForce[0] = ndVector8::m_one;
			const ndInt32 rowsCount = jointGroup[0]->m_rowCount;

			for (ndInt32 j = 0; j < rowsCount; ++j)
			{
				const ndSoaMatrixElement* const row = &massMatrix[j];
				normalForce[j + 1] = row->m_force;
			}

			const ndFloat32 tol = ndFloat32(0.125f);
			const ndFloat32 tol2 = tol * tol;

			ndVector8 accNorm(ndFloat32(10.0f));
			for (ndInt32 k = 0; (k < 4) && (accNorm.GetMax() > tol2); ++k)
			{
				accNorm = zero;
				for (ndInt32 j = 0; j < rowsCount; ++j)
				{
					ndSoaMatrixElement* const row = &massMatrix[j];

					ndVector8 a(row->m_JMinv.m_jacobianM0.m_linear.m_x * forceM0.m_linear.m_x);
					a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_x, forceM0.m_angular.m_x);
					a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_linear.m_y, forceM0.m_linear.m_y);
					a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_y, forceM0.m_angular.m_y);
					a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_linear.m_z, forceM0.m_linear.m_z);
					a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_z, forceM0.m_angular.m_z);

					a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_x, forceM1.m_linear.m_x);
					a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_x, forceM1.m_angular.m_x);
					a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_y, forceM1.m_linear.m_y);
					a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_y, forceM1.m_angular.m_y);
					a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_z, forceM1.m_linear.m_z);
					a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_z, forceM1.m_angular.m_z);

					const ndVector8 force(normalForce[j + 1]);
					a = row->m_coordenateAccel.MulSub(force, row->m_diagDamp) - a;
					ndVector8 f(force.MulAdd(row->m_invJinvMJt, a));

					const ndVector8 frictionNormal(&normalForce[0], row->m_normalForceIndex);
					const ndVector8 lowerFrictionForce(frictionNormal * row->m_lowerBoundFrictionCoefficent);
					const ndVector8 upperFrictionForce(frictionNormal * row->m_upperBoundFrictionCoefficent);

					f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
					normalForce[j + 1] = f;

					const ndVector8 deltaForce(f - force);
					const ndVector8 residual(deltaForce * row->m_JinvMJt);
					accNorm = accNorm.MulAdd(residual, residual);

					const ndVector8 deltaForce0(deltaForce * preconditioner0);
					const ndVector8 deltaForce1(deltaForce * preconditioner1);

					forceM0.m_linear.m_x = forceM0.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_x, deltaForce0);
					forceM0.m_linear.m_y = forceM0.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_y, deltaForce0);
					forceM0.m_linear.m_z = forceM0.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_z, deltaForce0);
					forceM0.m_angular.m_x = forceM0.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_x, deltaForce0);
					forceM0.m_angular.m_y = forceM0.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_y, deltaForce0);
					forceM0.m_angular.m_z = forceM0.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_z, deltaForce0);

					forceM1.m_linear.m_x = forceM1.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_x, deltaForce1);
					forceM1.m_linear.m_y = forceM1.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_y, deltaForce1);
					forceM1.m_linear.m_z = forceM1.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_z, deltaForce1);
					forceM1.m_angular.m_x = forceM1.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_x, deltaForce1);
					forceM1.m_angular.m_y = forceM1.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_y, deltaForce1);
					forceM1.m_angular.m_z = forceM1.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_z, deltaForce1);
				}
			}

			ndVector8 mask(ndVector8::m_mask);
			for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
			{
				const ndConstraint* const joint = jointGroup[i];
				if (joint && joint->m_rowCount)
				{
					const ndBodyKinematic* const body0 = joint->GetBody0();
					const ndBodyKinematic* const body1 = joint->GetBody1();
					ndAssert(body0);
					ndAssert(body1);
					const ndInt32 resting = body0->m_equilibrium0 & body1->m_equilibrium0;
					if (resting)
					{
						mask[i] = ndFloat32(0.0f);
					}
				}
			}

			forceM0.m_linear.m_x = zero;
			forceM0.m_linear.m_y = zero;
			forceM0.m_linear.m_z = zero;
			forceM0.m_angular.m_x = zero;
			forceM0.m_angular.m_y = zero;
			forceM0.m_angular.m_z = zero;

			forceM1.m_linear.m_x = zero;
			forceM1.m_linear.m_y = zero;
			forceM1.m_linear.m_z = zero;
			forceM1.m_angular.m_x = zero;
			forceM1.m_angular.m_y = zero;
			forceM1.m_angular.m_z = zero;
			for (ndInt32 i = 0; i < rowsCount; ++i)
			{
				ndSoaMatrixElement* const row = &massMatrix[i];
				const ndVector8 force(row->m_force.Select(normalForce[i + 1], mask));
				row->m_force = force;

				forceM0.m_linear.m_x = forceM0.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_x, force);
				forceM0.m_linear.m_y = forceM0.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_y, force);
				forceM0.m_linear.m_z = forceM0.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_z, force);
				forceM0.m_angular.m_x = forceM0.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_x, force);
				forceM0.m_angular.m_y = forceM0.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_y, force);
				forceM0.m_angular.m_z = forceM0.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_z, force);

				forceM1.m_linear.m_x = forceM1.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_x, force);
				forceM1.m_linear.m_y = forceM1.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_y, force);
				forceM1.m_linear.m_z = forceM1.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_z, force);
				forceM1.m_angular.m_x = forceM1.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_x, force);
				forceM1.m_angular.m_y = forceM1.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_y, force);
				forceM1.m_angular.m_z = forceM1.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_z, force);
			}

			ndVector8 force0[8];
			ndVector8::Transpose(
				force0[0], force0[1], force0[2], force0[3], force0[4], force0[5], force0[6], force0[7],
				forceM0.m_linear.m_x, forceM0.m_linear.m_y, forceM0.m_linear.m_z, ndVector8::m_zero,
				forceM0.m_angular.m_x, forceM0.m_angular.m_y, forceM0.m_angular.m_z, ndVector8::m_zero);

			ndVector8 force1[8];
			ndVector8::Transpose(
				force1[0], force1[1], force1[2], force1[3], force1[4], force1[5], force1[6], force1[7],
				forceM1.m_linear.m_x, forceM1.m_linear.m_y, forceM1.m_linear.m_z, ndVector8::m_zero,
				forceM1.m_angular.m_x, forceM1.m_angular.m_y, forceM1.m_angular.m_z, ndVector8::m_zero);

			ndRightHandSide* const rightHandSide = &m_rightHandSide[0];
			for (ndInt32 i = 0; i < ND_SIMD8_WORK_GROUP_SIZE; ++i)
			{
				const ndConstraint* const joint = jointGroup[i];
				if (joint)
				{
					const ndInt32 rowCount = joint->m_rowCount;
					const ndInt32 rowStartBase = joint->m_rowStart;
					for (ndInt32 j = 0; j < rowCount; ++j)
					{
						const ndSoaMatrixElement* const row = &massMatrix[j];
						rightHandSide[j + rowStartBase].m_force = row->m_force[i];
						rightHandSide[j + rowStartBase].m_maxImpact = ndMax(ndAbs(row->m_force[i]), rightHandSide[j + rowStartBase].m_maxImpact);
					}

					const ndInt32 index0 = (block + i) * 2 + 0;
					ndVector8& outBody0 = jointPartialForces[index0];
					outBody0 = force0[i];

					const ndInt32 index1 = (block + i) * 2 + 1;
					ndVector8& outBody1 = jointPartialForces[index1];
					outBody1 = force1[i];
				}
			}
		};

		const ndInt32 m = groupId;
		JointForce(m, &soaMassMatrix[soaJointRows[m]]);
	});

	auto ApplyJacobianAccumulatePartialForces = ndMakeObject::ndFunction([this, &bodyArray](ndInt32 group, ndInt32)
	{
		D_TRACKTIME_NAMED(ApplyJacobianAccumulatePartialForces);
		const ndInt32* const bodyIndex = &GetJointForceIndexBuffer()[0];
		ndVector8* const internalForces = (ndVector8*)&GetInternalForces()[0];
		const ndVector8* const jointInternalForces = (ndVector8*)&GetTempInternalForces()[0];
		const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &GetJointBodyPairIndexBuffer()[0];

		const ndInt32 m = group;
		const ndBodyKinematic* const body = bodyArray[m];
		const ndInt32 startIndex = bodyIndex[m];
		const ndInt32 mask = body->m_isStatic - 1;
		const ndInt32 count = mask & (bodyIndex[m + 1] - startIndex);

		ndVector8 forceTorque(ndVector8::m_zero);
		for (ndInt32 k = 0; k < count; ++k)
		{
			const ndInt32 index = jointBodyPairIndexBuffer[startIndex + k].m_joint;
			forceTorque = forceTorque + jointInternalForces[index];
		}
		internalForces[m] = forceTorque;
	});

	for (ndInt32 i = 0; i < ndInt32(passes); ++i)
	{
		const ndInt32 mask = -ndInt32(ND_SIMD8_WORK_GROUP_SIZE);
		const ndInt32 jointCount = ndInt32(jointArray.GetCount());
		const ndInt32 soaJointCount = ((jointCount + ND_SIMD8_WORK_GROUP_SIZE - 1) & mask) / ND_SIMD8_WORK_GROUP_SIZE;
		scene->ParallelExecute(CalculateJointsForce, soaJointCount, scene->OptimalGroupBatch(soaJointCount) / 8);

		const ndInt32 bodyCount = ndInt32(bodyArray.GetCount());
		scene->ParallelExecute(ApplyJacobianAccumulatePartialForces, bodyCount, scene->OptimalGroupBatch(bodyCount));
	}
}

void ndDynamicsUpdateSoa::CalculateForces()
{
	D_TRACKTIME();
	if (m_world->GetScene()->GetActiveContactArray().GetCount())
	{
		m_firstPassCoef = ndFloat32(0.0f);

		InitSkeletons();
		for (ndInt32 step = 0; step < 4; step++)
		{
			CalculateJointsAcceleration();
			CalculateJointsForce();
			UpdateSkeletons();
			IntegrateBodiesVelocity();
		}
	}
}

void ndDynamicsUpdateSoa::Update()
{
	D_TRACKTIME();
	m_timestep = m_world->GetScene()->GetTimestep();

	BuildIsland();
	IntegrateUnconstrainedBodies();
	InitWeights();
	InitBodyArray();
	InitJacobianMatrix();
	CalculateForces();
	IntegrateBodies();
	UpdateForceFeedback();
	DetermineSleepStates();
}
