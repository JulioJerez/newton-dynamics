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
#include "ndWorld.h"
#include "ndScene.h"
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

void ndConvexCastVehicle::CalculateContacts(ndFixSizeArray<ndConstraint*, 32>& contacts, ndInt32 threadId)
{
	m_skeleton->ClearCloseLoopJoints();

	//ndVector p0;
	//ndVector p1;
	//m_chassis->GetCollisionShape().CalculateAabb(m_chassis->GetCollisionShape().GetLocalMatrix() * m_chassis->GetMatrix(), p0, p1);
	//for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* tireNode = m_tireList.GetFirst(); tireNode; tireNode = tireNode->GetNext())
	//{
	//	ndVector q0;
	//	ndVector q1;
	//	const ndMultiBodyVehicleTireJoint* const joint = tireNode->GetInfo();
	//	const ndBodyDynamic* const wheel = joint->GetBody0()->GetAsBodyDynamic();
	//	wheel->GetCollisionShape().CalculateAabb(wheel->GetCollisionShape().GetLocalMatrix() * wheel->GetMatrix(), q0, q1);
	//	p0 = p0.GetMin(q0);
	//	p1 = p1.GetMax(q1);
	//}

	class ndShapeCast : public ndConvexCastNotify
	{
		public:
		ndShapeCast(ndConvexCastVehicle* const self, const ndMatrix& start, ndFloat32 length, const ndShapeInstance& shape)
			:ndConvexCastNotify()
			,m_self(self)
		{
			ndVector end(start.m_posit - start.m_up.Scale(length));
			m_self->GetWorld()->ConvexCast(*this, shape, start, end);
		}

		virtual ndUnsigned32 OnRayPrecastAction(const ndBody* const body, const ndShapeInstance* const) override
		{
			const ndBodyKinematic* const dynBody = ((ndBody*)body)->GetAsBodyKinematic();
			ndModel* const model = dynBody->GetModel();
			return (!model || (model->GetAsModelArticulation() != *m_self));
		}

		ndFloat32 OnRayCastAction(const ndContactPoint&, ndFloat32 param) override
		{
			return param;
		}

		ndWeakPtr<ndConvexCastVehicle> m_self;
	};

	ndList<ndContact>::ndNode* cachePtr = m_contactCache.GetFirst();
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* tireNode = m_tireList.GetFirst(); tireNode; tireNode = tireNode->GetNext())
	{
		const ndMultiBodyVehicleTireJoint* const joint = tireNode->GetInfo();
		ndBodyDynamic* const wheelBody = joint->GetBody0()->GetAsBodyDynamic();
		const ndMatrix matrix(joint->CalculateUpperBumperMatrix());
		const ndShapeInstance* const wheelShape = &wheelBody->GetCollisionShape();

		const ndWheelDescriptor& wheelInfo = joint->GetInfo();
		ndFloat32 dist = ndAbs(wheelInfo.m_lowerStop - wheelInfo.m_upperStop);
		ndShapeCast caster(this, matrix, dist, *wheelShape);

		if (caster.m_contacts.GetCount())
		{
			// set first body to the wheel body
			for (ndInt32 i = 0; i < caster.m_contacts.GetCount(); ++i)
			{
				caster.m_contacts[i].m_body0 = wheelBody;
			}

			// first sort the contacts by the secund body.
			for (ndInt32 i = 1; i < caster.m_contacts.GetCount(); ++i)
			{
				ndAssert(0);
				ndContactPoint point(caster.m_contacts[i]);
				ndAssert(point.m_body0 == wheelBody);
				ndInt32 j = i - 1;
				while (j >= 0 && caster.m_contacts[j].m_body1 > point.m_body1)
				{
					caster.m_contacts[j + 1] = caster.m_contacts[j];
					j--;
				}
				// Place 'key' into the gap created by shifting.
				caster.m_contacts[j + 1] = point;
			}

			// calculate the scans of contacts
			ndInt32 count = 0;
			ndFixSizeArray<ndInt32, 16> scans;
			for (ndInt32 i = 1; i < caster.m_contacts.GetCount(); ++i)
			{
				count++;
				if (caster.m_contacts[i - 1].m_body1 != caster.m_contacts[i].m_body1)
				{
					scans.PushBack(count);
					count = 0;
				}
			}

			ndInt32 sum = 0;
			scans.PushBack(count + 1);
			for (ndInt32 i = 0; i < scans.GetCount(); ++i)
			{
				ndInt32 scanCount = scans[i];
				scans[i] = sum;
				sum += scanCount;
			}
			scans.PushBack(sum);
			ndAssert(sum == caster.m_contacts.GetCount());

			// add the contacts 
			ndScene* const scene = GetWorld()->GetScene();
			for (ndInt32 i = 0; i < scans.GetCount() - 1; ++i)
			{
				// get a new contacts from cache
				if (!cachePtr)
				{
					cachePtr = m_contactCache.Append();
				}
				ndContact* const contact = &cachePtr->GetInfo();
				cachePtr = cachePtr->GetNext();
				contacts.PushBack(contact);

				// craete a contact joint
				const ndInt32 start = scans[i];
				const ndInt32 pointCount = scans[i + 1] - start;
				ndBodyKinematic* const body1 = ((ndBody*)caster.m_contacts[start].m_body1)->GetAsBodyKinematic();
				contact->SetBodies(wheelBody, body1);
				//contact->AttachToBodies();
				contact->GetContactPoints().RemoveAll();

				ndContactSolver contactSolver;
				contactSolver.m_threadId = threadId;
				contactSolver.m_contact = contact;
				contactSolver.m_contactBuffer = &caster.m_contacts[start];
				scene->ProcessContacts(threadId, pointCount, &contactSolver);
			}
		}
	}
}

void ndConvexCastVehicle::Update(ndFloat32 timestep, ndInt32 threadId)
{
	ndWorld* const world = GetWorld();
	ndAssert(world);

	// update tire contacts 
	ndFixSizeArray<ndConstraint*, 32> contacts;
	m_skeleton->m_owner = world;
	CalculateContacts(contacts, threadId);

	// update model
	ndMultiBodyVehicle::Update(timestep, threadId);

	//// solve using immediate solver.
	//m_solver.SolverBegin(*m_skeleton, &contacts[0], contacts.GetCount(), world, timestep, 0);
	//m_solver.Solve();
	//m_solver.SolverEnd();
}