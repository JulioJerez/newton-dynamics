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
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndBodyDynamic.h"
#include "ndBodyKinematic.h"
#include "ndConvexCastVehicle.h"
#include "ndMultiBodyVehicleMotor.h"
#include "ndMultiBodyVehicleGearBox.h"
#include "ndMultiBodyVehicleTireJoint.h"
#include "ndMultiBodyVehicleTorsionBar.h"
#include "ndMultiBodyVehicleDifferential.h"
#include "ndMultiBodyVehicleDifferentialAxle.h"


ndConvexCastVehicle::ndConvexCastVehicle(ndFloat32 gravityMagnitud)
	:ndMultiBodyVehicle(gravityMagnitud)
	,m_solver()
{
}

void ndConvexCastVehicle::ConvertToMotorVehicle()
{
	ndMultiBodyVehicle::ConvertToMotorVehicle();

	// clone the skeleton structural joints.
	m_skeleton = ndSharedPtr<ndSkeletonContainer>(new ndSkeletonContainer);
	ndFixSizeArray<ndNode*, 64> stack;
	ndFixSizeArray<ndSkeletonContainer::ndNode*, 64> parents;
	stack.PushBack(m_rootNode);
	parents.PushBack(nullptr);
	while (stack.GetCount())
	{
		ndNode* const node = stack.Pop();
		ndSkeletonContainer::ndNode* parent = parents.Pop();
		if (parent)
		{
			parent = m_skeleton->AddChild(*node->m_joint, parent);
			parent->m_body->SetSkeleton(nullptr);
			parent->m_joint->SetSkeletonFlag(false);
		}
		else
		{
			ndBodyDynamic* const rootBody = node->m_body->GetAsBodyDynamic();
			m_skeleton->Init(nullptr, rootBody, 0);
			rootBody->SetSkeleton(nullptr);
			parent = m_skeleton->GetRoot();
		}

		for (ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
		{
			stack.PushBack(child);
			parents.PushBack(parent);
		}
	}

	// clone the skeleton loop joints.
	ndFixSizeArray<ndJointBilateralConstraint*, 256> loopJoints;
	ndList<ndNode, ndContainersFreeListAlloc<ndNode>>& loops = GetCloseLoops();
	for (ndList<ndNode, ndContainersFreeListAlloc<ndNode>>::ndNode* node = loops.GetFirst(); node; node = node->GetNext())
	{
		loopJoints.PushBack(*node->GetInfo().m_joint);
	}
	m_skeleton->Finalize(loopJoints.GetCount(), &loopJoints[0]);

	// remove the drive train from simulation
	ndList<ndNode, ndContainersFreeListAlloc<ndNode>>::ndNode* nextNode;
	for (ndList<ndNode, ndContainersFreeListAlloc<ndNode>>::ndNode* node = loops.GetFirst(); node; node = nextNode)
	{
		nextNode = node->GetNext();
		ndSharedPtr<ndJointBilateralConstraint> joint(node->GetInfo().m_joint);
		if (joint->IsType(ndMultiBodyVehicleGearBox::StaticClassName()))
		{
			m_castGearBox = joint;
			loops.Remove(node);
		}
		else if (joint->IsType(ndMultiBodyVehicleDifferentialAxle::StaticClassName()))
		{
			m_castDifferentialAxelList.Append(joint);
			loops.Remove(node);
		}
	}

	//disable wheel, body, and differntials 
	auto DisableStructuralNodes = [](ndNode* const node)
	{
		if (node->m_body && node->m_joint)
		{
			ndSharedPtr<ndJointBilateralConstraint> joint(node->m_joint);
			if (joint->IsType(ndMultiBodyVehicleTireJoint::StaticClassName()))
			{
				joint->SetActive(false);
			}
			else if (joint->IsType(ndMultiBodyVehicleMotor::StaticClassName()))
			{
				joint->SetActive(false);
			}
			else if (joint->IsType(ndMultiBodyVehicleDifferential::StaticClassName()))
			{
				joint->SetActive(false);
			}
		}
	};
	NodeIterator(DisableStructuralNodes);
}

void ndConvexCastVehicle::CalculateContacts(ndFixSizeArray<ndConstraint*, 32>& contacts)
{
	m_skeleton->ClearCloseLoopJoints();
	ndList<ndSharedPtr<ndContact>>::ndNode* cachePtr = m_contactCache.GetFirst();
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* tireNode = m_tireList.GetFirst(); tireNode; tireNode = tireNode->GetNext())
	{
		const ndMultiBodyVehicleTireJoint* const joint = tireNode->GetInfo();
		const ndBodyDynamic* const body = joint->GetBody0()->GetAsBodyDynamic();
		const ndMatrix matrix(joint->CalculateUpperBumperMatrix());
		const ndShapeInstance* const tireShape = &body->GetCollisionShape();
		
	}
}

void ndConvexCastVehicle::Update(ndFloat32 timestep, ndInt32 threadId)
{
	ndWorld* const world = GetWorld();
	ndAssert(world);

	// update tire contacts 
	ndFixSizeArray<ndConstraint*, 32> contacts;
	m_skeleton->m_owner = world;
	CalculateContacts(contacts);

	// update model
	ndMultiBodyVehicle::Update(timestep, threadId);

	// solve using immediate solver.
	m_solver.SolverBegin(*m_skeleton, &contacts[0], contacts.GetCount(), world, timestep, 0);
	m_solver.Solve();
	m_solver.SolverEnd();
}