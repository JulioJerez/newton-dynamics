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
	,m_sleepCounter(0)
{
}

void ndConvexCastVehicle::OnAddToWorld()
{
	ndMultiBodyVehicle::OnAddToWorld();

	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* tireNode = m_tireList.GetFirst(); tireNode; tireNode = tireNode->GetNext())
	{
		// support up to two contacts per tire
		ndBodyDynamic* const wheelBody = tireNode->GetInfo()->GetBody0()->GetAsBodyDynamic();
		ndBodyKinematic::ndContactMap& contacts = wheelBody->GetContactMap();
		contacts.AttachContact(new ndContact(), 1);
		contacts.AttachContact(new ndContact(), 2);
	}
}

void ndConvexCastVehicle::OnRemoveFromWorld()
{
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* tireNode = m_tireList.GetFirst(); tireNode; tireNode = tireNode->GetNext())
	{
		// support up to two contacts per tire
		ndBodyDynamic* const wheelBody = tireNode->GetInfo()->GetBody0()->GetAsBodyDynamic();
		ndBodyKinematic::ndContactMap& contacts = wheelBody->GetContactMap();

		while (contacts.GetCount())
		{
			ndContact* const contact = contacts.GetRoot()->GetInfo();
			contacts.Remove(contacts.GetRoot());
			delete contact;
		}
	}

	ndMultiBodyVehicle::OnRemoveFromWorld();
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

	//disable wheels, motor and differentials 
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

void ndConvexCastVehicle::CalculateConveCastTireContacts(ndInt32 threadId)
{
	class ndShapeCast : public ndConvexCastNotify
	{
		public:
		ndShapeCast(ndConvexCastVehicle* const self, const ndMatrix& start, ndFloat32 length, const ndShapeInstance& shape)
			:ndConvexCastNotify()
			,m_self(self)
		{
			ndVector end(start.m_posit + start.m_up.Scale(length));
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

	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* tireNode = m_tireList.GetFirst(); tireNode; tireNode = tireNode->GetNext())
	{
		const ndMultiBodyVehicleTireJoint* const wheelJoint = tireNode->GetInfo();
		ndBodyDynamic* const wheelBody = wheelJoint->GetBody0()->GetAsBodyDynamic();
		
		// deactive contacts
		ndBodyKinematic::ndContactMap& contactMap = wheelBody->GetContactMap();
		ndBodyKinematic::ndContactMap::Iterator it(contactMap);
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = it.GetNode()->GetInfo();
			contact->SetActive(false);
		}
		
		// shot a convex cast to generate wheel contact joint manually
		const ndMatrix matrix(wheelJoint->CalculateUpperBumperMatrix());
		const ndShapeInstance* const wheelShape = &wheelBody->GetCollisionShape();
		
		const ndWheelDescriptor& wheelInfo = wheelJoint->GetInfo();
		ndFloat32 dist = wheelInfo.m_lowerStop - wheelInfo.m_upperStop;
		ndAssert(dist < ndFloat32(0.0f));
		ndShapeCast caster(this, matrix, dist, *wheelShape);

		//if there are contact point, set the tire contact joints
		for (ndInt32 i = caster.m_contacts.GetCount() - 1; i >= 0; --i)
		{
			caster.m_contacts[i].m_body0 = wheelBody;
			// convex cast reports zero penetration, 
			// we must extract the panetration by using the the current tore local position.
			// if not, the tire slowtlly thinks into the support object.
			ndFloat32 castPosit = wheelInfo.m_upperStop + dist * caster.m_param;
			ndFloat32 currentPosit = wheelJoint->GetPosit();
			ndFloat32 penetration = castPosit - currentPosit;
			caster.m_contacts[i].m_penetration = penetration;
			if (penetration < ndFloat32(0.0f))
			{
				const ndInt32 n = caster.m_contacts.GetCount() - 1;
				caster.m_contacts[i] = caster.m_contacts[n];
				caster.m_contacts.SetCount(n);
			}
		}

		if (caster.m_contacts.GetCount())
		{
			// first sort the contacts by the secund body.
			for (ndInt32 i = 1; i < caster.m_contacts.GetCount(); ++i)
			{
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
			scans.PushBack(count + 1);
			
			ndInt32 sum = 0;
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
			const ndInt32 contactJointCount = ndMin(scans.GetCount() - 1, 2);
			for (ndInt32 i = 0; i < contactJointCount; ++i)
			{
				// get a new contacts from cache
				const ndBodyKinematic::ndContactkey key(ndUnsigned32(i + 1), 0);
				ndBodyKinematic::ndContactMap::ndNode* const contactNode = contactMap.Find(key);
				ndAssert(contactNode);
				ndContact* const contact = contactNode->GetInfo();
			
				// craete a contact joint
				const ndInt32 start = scans[i];
				const ndInt32 pointCount = scans[i + 1] - start;
				ndBodyKinematic* const body1 = ((ndBody*)caster.m_contacts[start].m_body1)->GetAsBodyKinematic();
				contact->SetActive(true);
				contact->m_maxDof = 0;
				contact->SetBodies(wheelBody, body1);
				contact->GetContactPoints().RemoveAll();
			
				ndContactSolver contactSolver;
				contactSolver.m_threadId = threadId;
				contactSolver.m_contact = contact;
				contactSolver.m_contactBuffer = &caster.m_contacts[start];
				scene->ProcessContacts(threadId, pointCount, &contactSolver);
				if (contact->m_maxDof == 0)
				{
					contact->SetActive(false);
				}
			}
		}
	}
}

void ndConvexCastVehicle::Debug(ndConstraintDebugCallback& context) const
{
	ndMultiBodyVehicle::Debug(context);

	//ndVector red(ndVector::m_wOne);
	//red.m_x = 1.0f;
	//for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* tireNode = m_tireList.GetFirst(); tireNode; tireNode = tireNode->GetNext())
	//{
	//	const ndMultiBodyVehicleTireJoint* const wheelJoint = tireNode->GetInfo();
	//	ndBodyDynamic* const wheelBody = wheelJoint->GetBody0()->GetAsBodyDynamic();
	//
	//	// deactive contacts
	//	ndBodyKinematic::ndContactMap& contactMap = wheelBody->GetContactMap();
	//	ndBodyKinematic::ndContactMap::Iterator it(contactMap);
	//	for (it.Begin(); it; it++)
	//	{
	//		ndContact* const contact = it.GetNode()->GetInfo();
	//		if (contact->IsActive())
	//		{
	//			ndContactPointList& contactPoints = contact->GetContactPoints();
	//			for (ndContactPointList::ndNode* node = contactPoints.GetFirst(); node; node = node->GetNext())
	//			{
	//				const ndContactMaterial& contactMaterial = node->GetInfo();
	//				context.DrawPoint(contactMaterial.m_point, red);
	//			}
	//		}
	//	}
	//}
}

void ndConvexCastVehicle::TransformUpdate(ndFloat32 timestep)
{
	ndMultiBodyVehicle::TransformUpdate(timestep);

	auto UpdateIntenalTransforms = [timestep](ndNode* const node)
	{
		if (node->m_body)
		{
			ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
			if (node->m_joint)
			{
				const ndJointBilateralConstraint* const joint = *node->m_joint;
				if (joint->IsType(ndMultiBodyVehicleTireJoint::StaticClassName()))
				{
					body->GetNotifyCallback()->OnTransform(timestep, body->GetMatrix());
				}
				else if (joint->IsType(ndMultiBodyVehicleMotor::StaticClassName()))
				{
					body->GetNotifyCallback()->OnTransform(timestep, body->GetMatrix());
				}
				else if (joint->IsType(ndMultiBodyVehicleDifferential::StaticClassName()))
				{
					body->GetNotifyCallback()->OnTransform(timestep, body->GetMatrix());
				}
			}
		}
	};
	NodeIterator(UpdateIntenalTransforms);
}

void ndConvexCastVehicle::PostUpdate(ndFloat32 timestep, ndInt32 threadId)
{
	ndMultiBodyVehicle::PostUpdate(timestep, threadId);

	for (ndInt32 i = 0; i < m_savedBody.GetCount(); ++i)
	{
		ndBodyDynamic* const body = m_savedBody[i];
		const ndJacobian& forceTorque = m_savedForceTorque[i];
		body->m_savedExternalForce = forceTorque.m_linear;
		body->m_savedExternalTorque = forceTorque.m_angular;
	}
}

void ndConvexCastVehicle::Update(ndFloat32 timestep, ndInt32 threadId)
{
	ndWorld* const world = GetWorld();
	ndAssert(world);
	m_savedBody.SetCount(0);
	m_savedForceTorque.SetCount(0);

	// check for equilibrium state here
	if (IsSleeping())
	{
		m_sleepCounter++;
		if (m_sleepCounter >= 8)
		{
			return;
		}
	}
	else 
	{
		m_sleepCounter = 0;
	}

	// update tire contacts 
	m_skeleton->m_owner = world;
	CalculateConveCastTireContacts(threadId);

	// apply all external forces to intenal bodies
	m_originaSkeleton = GetRoot()->m_body->GetAsBodyKinematic()->GetSkeleton();
	auto ApplyExternalForces = [this, timestep, threadId](ndNode* const node)
	{
		if (node->m_body)
		{
			ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
			ndAssert(body->GetSkeleton() == *m_originaSkeleton);
			body->SetSkeleton(*m_skeleton);

			if (node->m_joint)
			{
				ndSharedPtr<ndJointBilateralConstraint> joint(node->m_joint);
				if (joint->IsType(ndMultiBodyVehicleTireJoint::StaticClassName()))
				{
					body->GetNotifyCallback()->OnApplyExternalForce(threadId, timestep);
				}
				else if (joint->IsType(ndMultiBodyVehicleMotor::StaticClassName()))
				{
					body->GetNotifyCallback()->OnApplyExternalForce(threadId, timestep);
				}
				else if (joint->IsType(ndMultiBodyVehicleDifferential::StaticClassName()))
				{
					body->GetNotifyCallback()->OnApplyExternalForce(threadId, timestep);
				}
			}
		}
	};
	NodeIterator(ApplyExternalForces);

	// update model
	ndMultiBodyVehicle::Update(timestep, threadId);

	// solve using immediate solver.
	m_solver.SolverBegin(*m_skeleton, nullptr, 0, world, timestep, threadId);

	// calculate forces
	m_solver.Solve();

	// integrate tires, and internal drive train components
	// apply reaction impulses to other bodies
	// apply the impulse to model bodies. 
	// restore the orginal skeleton
	auto IntegrateBodyParts = [this, timestep](ndNode* const node)
	{
		if (node->m_body)
		{
			ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
			body->SetSkeleton(*m_originaSkeleton);

			auto IntegrateStructuralPart = [this, body, timestep]()
			{
				// for model bodies we just set the net force
				const ndMatrix inertia(body->CalculateInertiaMatrix());
				const ndVector torque(inertia.RotateVector(body->m_alpha) + body->m_gyroTorque);
				const ndVector force(body->m_accel.Scale(body->m_mass.m_w));

				ndUnsigned8 equilibrium = body->m_equilibrium;
				ndJacobian forceTorque;
				forceTorque.m_linear = body->m_externalForce;
				forceTorque.m_angular = body->m_externalTorque;
				m_savedBody.PushBack(body);
				m_savedForceTorque.PushBack(forceTorque);
				body->SetForce(force);
				body->SetTorque(torque);
				body->m_equilibrium = equilibrium;
			};

			if (node->m_joint)
			{
				auto IntegrateInternalPart = [body, timestep]()
				{
				#ifdef _DEBUG
					const ndVector angularMomentum(body->CalculateAngularMomentum());
					const ndVector xxxx(body->m_omega.CrossProduct(angularMomentum) - body->m_gyroTorque);
					ndAssert(xxxx.DotProduct(xxxx).GetScalar() - ndFloat32(1.0e-3f));
				#endif

					const ndMatrix inertia(body->CalculateInertiaMatrix());
					const ndVector torque(inertia.RotateVector(body->m_alpha) + body->m_gyroTorque);
					const ndVector force(body->m_accel.Scale(body->m_mass.m_w));
					body->SetForce(force);
					body->SetTorque(torque);
					body->IntegrateExternalForce(timestep);
					body->IntegrateVelocity(timestep);
				};

				// internal bodies are full integrared
				ndSharedPtr<ndJointBilateralConstraint>& joint(node->m_joint);
				if (joint->IsType(ndMultiBodyVehicleTireJoint::StaticClassName()))
				{
					IntegrateInternalPart();
				}
				else if (joint->IsType(ndMultiBodyVehicleMotor::StaticClassName()))
				{
					IntegrateInternalPart();
				}
				else if (joint->IsType(ndMultiBodyVehicleDifferential::StaticClassName()))
				{
					IntegrateInternalPart();
				}
				else
				{
					IntegrateStructuralPart();
				}
			}
			else
			{
				IntegrateStructuralPart();
			}
		}
	};
	NodeIterator(IntegrateBodyParts);

	m_solver.SolverEnd();
}