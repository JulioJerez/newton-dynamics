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

//#define ND_MAX_SIDESLIP (ndFloat32(15.0f) * ndDegreeToRad)
#define ND_MAX_SIDESLIP (ndFloat32(1.0f) * ndDegreeToRad)

class ndConvexCastVehicle::ndBicycleModelStability : public ndJointBilateralConstraint
{
	public:
	ndBicycleModelStability(ndConvexCastVehicle* const owner)
		:ndJointBilateralConstraint(1, *owner->m_chassis, &m_sentinel, owner->m_chassis->GetMatrix())
		,m_owner(owner)
		,m_u(ndFloat32(0.0f))
		,m_r(ndFloat32(0.0f))
		,m_vx(ndFloat32(0.0f))
		,m_vz(ndFloat32(0.0f))
		,m_beta(ndFloat32(0.0f))
		,m_betaRate(ndFloat32(0.0f))
		,m_modelIsValid(false)
	{
	}

	//void JacobianDerivative(ndConstraintDescritor& desc) override
	void JacobianDerivative(ndConstraintDescritor&) override
	{
		// only add row when the model is applicable
		if (m_modelIsValid)
		{
			m_modelIsValid = true;
			//ndAssert(0);
		}
	}

	ndBodyDynamic m_sentinel;
	ndWeakPtr<ndConvexCastVehicle> m_owner;
	ndFloat32 m_u;
	ndFloat32 m_r;
	ndFloat32 m_vx;
	ndFloat32 m_vz;
	ndFloat32 m_beta;
	ndFloat32 m_betaRate;
	bool m_modelIsValid;
};

class ndConvexCastVehicle::ndBicycleModelContact: public ndContact
{
	public:
	ndBicycleModelContact(ndConvexCastVehicle* const owner)
		:ndContact()
		,m_owner(owner)
	{
	}

	void JacobianDerivative(ndConstraintDescritor& desc) override
	{
		//const ndBicycleModelStability& bicycle = **m_owner->m_bicycleModel;
		//if (bicycle.m_modelIsValid)
		//{
		//
		//}

		ndContact::JacobianDerivative(desc);
	}

	ndWeakPtr<ndConvexCastVehicle> m_owner;
};

ndConvexCastVehicle::ndConvexCastVehicle(ndFloat32 gravityMagnitud)
	:ndMultiBodyVehicle(gravityMagnitud)
	,m_solver()
	,m_sleepCounter(0)
	,m_bicycleModel(nullptr)
{
}

void ndConvexCastVehicle::OnAddToWorld()
{
	ndMultiBodyVehicle::OnAddToWorld();

	m_bicycleModel = ndSharedPtr<ndBicycleModelStability>(new ndBicycleModelStability(this));

	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* tireNode = m_tireList.GetFirst(); tireNode; tireNode = tireNode->GetNext())
	{
		// support up to two contacts per tire
		ndMultiBodyVehicleTireJoint* const tire = tireNode->GetInfo();
		ndBodyDynamic* const wheelBody = tire->GetBody0()->GetAsBodyDynamic();
		ndBodyKinematic::ndContactMap& contacts = wheelBody->GetContactMap();
		contacts.AttachContact(new ndBicycleModelContact(this), 1);
		contacts.AttachContact(new ndBicycleModelContact(this), 2);
	}
}

void ndConvexCastVehicle::OnRemoveFromWorld()
{
	m_bicycleModel = ndSharedPtr<ndBicycleModelStability>(nullptr);
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

void ndConvexCastVehicle::CalculateConvexCastTireContacts(ndInt32 threadId)
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
				//contact->GetContactPoints().RemoveAll();
			
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

	// try to render the a represenation of teh tire her, for now nothong
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
	CalculateConvexCastTireContacts(threadId);

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

	// apply vehicle stability control
	ApplyBicycleModelLateralStability(timestep);

	// solve using immediate solver.
	ndConstraint* const bicycleModel = *m_bicycleModel;
	m_solver.SolverBegin(*m_skeleton, &bicycleModel, 1, world, timestep, threadId);

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
					// save contact normal force 
					ndBodyKinematic::ndContactMap::Iterator it(node->m_body->GetAsBodyKinematic()->GetContactMap());
					const ndVector normalForce(joint->GetForceBody0());
					for (it.Begin(); it; it++)
					{
						ndContact* const contact = it.GetNode()->GetInfo();
						if (contact->IsActive())
						{
							for (ndContactPointList::ndNode* pointNode = contact->GetContactPoints().GetFirst(); pointNode; pointNode = pointNode->GetNext())
							{
								ndContactMaterial& point = pointNode->GetInfo();
								ndFloat32 forceMag = point.m_normal.DotProduct(normalForce).GetScalar();
								point.m_normal_Force.Push(ndAbs(forceMag));
							}
						}
					}
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

#if 0
//It seems the open - loop lateral dynamics of the bicycle model are not going to work.
//I will try a closed - loop approach that enforces 
//the beta and beta rates during the forward pass.
// Then I can use the force solution to generate the 
// least constrained tire friction forces.
// I’m not sure how well this will work, but it is worth trying.

void ndConvexCastVehicle::ApplyBicycleModelLateralStability(ndFloat32 timestep)
{
	// we first check if the vehicel mets the steady state 
	// wher the bicycle model apply, by apply a series of checks
	// 
	// 1-if less than two contacts, stability does not apply 
	ndInt32 contactCount = 0;
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tireJoint = node->GetInfo();
		ndBodyKinematic* const tireBody = tireJoint->GetBody0()->GetAsBodyDynamic();

		// draw tire normal forces
		ndBodyKinematic::ndContactMap::Iterator it(tireBody->GetContactMap());
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = *it;
			if (contact->IsActive())
			{
				contactCount++;
				break;
			}
		}
	}
	if (contactCount < 2)
	{
		return;
	}

	// 2-check that the vehicle is not tilted, more than 20 degrees 
	const ndMatrix frameMatrix(m_localFrame * m_chassis->GetMatrix());
	if (frameMatrix.m_up.m_y < ndFloat32(0.94f))
	{
		return;
	}

	// 3 -check longitiduianl velocity isn't too low. 
	const ndVector veloc(frameMatrix.UnrotateVector(m_chassis->GetVelocity()));
	ndFloat32 localSpeed = ndSqrt(veloc.m_x * veloc.m_x + veloc.m_z * veloc.m_z);
	if (localSpeed < ndFloat32(2.0f))
	{
		// less than 2 m/s is just too speed for no linear lateral dynamics
		return;
	}

	// 4- check that slip beta is above the max allowed. 
	const ndFloat32 beta = ndAtan2(veloc.m_z, veloc.m_x);
	if (ndAbs(beta) < ND_MAX_SIDESLIP)
	{
		return;
	}

	// if all check pass, the vehicle is in a state 
	// where bicycle model corrections apply.
	// 
	// Based on pages 222–230 of Giancarlo Genta's book.
	//
	// When the tires operate outside the linear slip region, 
	// the vehicle's rigid-body equations must be corrected 
	// by introducing the chassis sideslip angle (beta) 
	// and its time derivative (betaRate).
	//
	// Genta derives these equations using Lagrangian dynamics 
	// for the bicycle vehicle model. 
	// On page 230, the equations are linearized
	// under the assumption that the sideslip angle is small:
	//
	// cos(beta) ~= 1
	// sin(beta) ~= beta
	//
	// This yields the following equations, whe all parameter are in teh body frame coodinate system:
	// 
	// 1) mass * localSpeed * (betaRate + yawRate) + mass * beta * localAccel = lateralForce
	// 2) mass * (localAccel - yawRate * localSpeed * beta) = longitudinalForce
	// 3) yawInertia * yawAlpha = yawTorque
	//
	// The resulting system is overdetermined, 
	// so additional constraints are required. 
	// This implementation applies the following assumptions:
	//
	// - The sideslip angle (beta) is clamped to +-25 degrees.
	// - Yaw angular acceleration (yawAlpha) is driven toward zero.
	// All calculations are performed in the vehicle's local reference frame.

	// we first calculate beta, betaRate, and yaw rate
	// for the current vehicle state.

	// calculate yawrate
	const ndVector omega(frameMatrix.UnrotateVector(m_chassis->GetOmega()));
	const ndFloat32 yawRate = omega.m_y;

	// using the forces from previuos frame calculate rear and front forces.
	ndVector rearForce(m_chassis->GetForce());
	ndVector frontForce(rearForce);
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* tireNode = m_tireList.GetFirst(); tireNode; tireNode = tireNode->GetNext())
	{
		const ndMultiBodyVehicleTireJoint* const tireJoint = tireNode->GetInfo();

		// Get the tire force and torque.
		const ndMatrix tireMatrix(tireJoint->CalculateGlobalMatrix1());
		const ndVector8 jointForce(tireJoint->GetForceTorqueBody1());
		const ndVector locaTirePosit(frameMatrix.UntransformVector(tireMatrix.m_posit));
		if (locaTirePosit.m_x < ndFloat32(0.0f))
		{
			rearForce += jointForce.GetLow();
		}
		else
		{
			frontForce += jointForce.GetLow();
		}
	}
	rearForce = frameMatrix.UnrotateVector(rearForce);
	frontForce = frameMatrix.UnrotateVector(frontForce);

	ndFloat32 rearLateralForce = rearForce.m_z;
	ndFloat32 frontLateralForce = frontForce.m_z;
	ndFloat32 rearLongitudinalForce = rearForce.m_x;
	ndFloat32 frontLongitudinalForce = frontForce.m_x;

	// calculate acceleration, from equation 2 we get
	// 2) mass * (localAccel - yawRate * localSpeed * beta) = longitudinalForce
	ndVector massMatrix(m_chassis->GetMassMatrix());
	ndFloat32 longitudinalForce = frontLongitudinalForce + rearLongitudinalForce;
	ndFloat32 localAccel = (longitudinalForce - massMatrix.m_w * yawRate * localSpeed * beta) / massMatrix.m_w;

	// calculate beta rate, from equation 1
	// 1) mass * localSpeed * (betaRate + yawRate) + mass * beta * localAccel = lateralForce

	// betaRate = (lateralForce - mass * beta * localAccel) / (mass * localSpeed) - yawRate;

	const ndFloat32 invTimeStep = ndFloat32(1.0) / timestep;
	const ndFloat32 lateralForce = frontLateralForce + rearLateralForce;
	const ndFloat32 betaRate = (beta - m_bicycleModel->m_beta) * invTimeStep;
	//const ndFloat32 equationBetaRate = (lateralForce - massMatrix.m_w * beta * localAccel) / (massMatrix.m_w * localSpeed) - yawRate;
	ndFloat32 equationBetaRate = (lateralForce - massMatrix.m_w * beta * localAccel) / (massMatrix.m_w * localSpeed) - yawRate;
	equationBetaRate *= 1;

	// update bicycle model corrent state parameters, 
	// for using with tire contacts joints
	m_bicycleModel->m_r = yawRate;
	m_bicycleModel->m_beta = beta;
	m_bicycleModel->m_u = localSpeed;
	m_bicycleModel->m_vx = veloc.m_x;
	m_bicycleModel->m_vz = veloc.m_z;
	m_bicycleModel->m_betaRate = betaRate;

	// the stability critirial in to tweak the lateral
	// tire friction conefficient in such way that 
	// betaRate and Yaw rate and identical. 
	// for that, using equation 1 
	// 1) mass * localSpeed * (betaRate + yawRate) + mass * beta * localAccel = lateralForce

	// the expression betaRate + yawRate should be set to zero.
	// therefore the target yawRate should be 
	// targetYawRate = -betaRate

	// this information is now used by each tire contact
	// to inforce a sideslip that forces yawRate match -betaRate

	// we only need beta, and U 
	// but we are calcuation all the bicycle because they 
	// are good for fedback monitoring,

	ndTrace(("beta=%g v=%g betaRate=%g yawRate=%g\n", beta * ndRadToDegree, localSpeed, betaRate, yawRate))
	if (ndAbs(beta * ndRadToDegree) > 30.0f)
	{
		m_bicycleModel->m_betaRate = betaRate;
	}

	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* tireNode = m_tireList.GetFirst(); tireNode; tireNode = tireNode->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tireJoint = tireNode->GetInfo();
		ndBodyKinematic* const tireBody = tireJoint->GetBody0()->GetAsBodyDynamic();
		ndBodyKinematic::ndContactMap::Iterator it(tireBody->GetContactMap());
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = *it;
			if (contact->IsActive())
			{
				// I can calculate the velocity inline, but the 
				// method takes com into account, so use the methof and then restore omega
				const ndVector tireHubVeloc(m_chassis->GetVelocityAtPoint(tireBody->GetMatrix().m_posit));
				const ndVector savedChassisOmega(m_chassis->GetOmega());

				ndVector desiredChassiOmega(savedChassisOmega);
				desiredChassiOmega.m_y = -m_bicycleModel->m_betaRate;
				m_chassis->SetOmegaNoSleep(desiredChassiOmega);
				const ndVector targetTireHubVeloc(m_chassis->GetVelocityAtPoint(tireBody->GetMatrix().m_posit));
				// restore ChassisOmega
				m_chassis->SetOmegaNoSleep(savedChassisOmega);

				// now this these velocities and the tire joint 
				// calculate the requires tire slip ratio.
				ndVector tireHubRelativeVeloc(targetTireHubVeloc - tireHubVeloc);
				ndContactPointList& contactPoints = contact->GetContactPoints();
				for (ndContactPointList::ndNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
				{
					//ndContactMaterial& contactPoint = contactNode->GetInfo();
					//const ndVector larealDir(contactPoint.m_dir1);
					//ndFloat32 relAccel = larealDir.DotProduct(tireHubRelativeVeloc).GetScalar() * invTimeStep;
					//contactPoint.OverrideFriction1Accel(relAccel);
				}
			}
		}
	}
}

#else

void ndConvexCastVehicle::ApplyBicycleModelLateralStability(ndFloat32 timestep)
{
	// we first check if the vehicel mets the steady state 
	// wher the bicycle model apply, by apply a series of checks
	// 
	// 1-if less than two contacts, stability does not apply 
	ndInt32 contactCount = 0;
	m_bicycleModel->m_modelIsValid = false;
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tireJoint = node->GetInfo();
		ndBodyKinematic* const tireBody = tireJoint->GetBody0()->GetAsBodyDynamic();

		// draw tire normal forces
		ndBodyKinematic::ndContactMap::Iterator it(tireBody->GetContactMap());
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = *it;
			if (contact->IsActive())
			{
				contactCount++;
				break;
			}
		}
	}
	if (contactCount < 2)
	{
		return;
	}

	// 2-check that the vehicle is not tilted, more than 20 degrees 
	const ndMatrix frameMatrix(m_localFrame * m_chassis->GetMatrix());
	if (frameMatrix.m_up.m_y < ndFloat32(0.94f))
	{
		return;
	}

	// 3 -check longitiduianl velocity isn't too low. 
	const ndVector veloc(frameMatrix.UnrotateVector(m_chassis->GetVelocity()));
	ndFloat32 localSpeed = ndSqrt(veloc.m_x * veloc.m_x + veloc.m_z * veloc.m_z);
	if (localSpeed < ndFloat32(2.0f))
	{
		// less than 2 m/s is just too speed for no linear lateral dynamics
		return;
	}

	// 4- check that slip beta is above the max allowed. 
	//const ndFloat32 beta = ndAtan2(veloc.m_z, veloc.m_x);
	//actaully the book use the aprocimation 
	const ndFloat32 beta = veloc.m_z / veloc.m_x;
	if (ndAbs(beta) < ND_MAX_SIDESLIP)
	{
		return;
	}

	// if all check pass, the vehicle is in a state 
	// where bicycle model corrections apply.
	// 
	// Based on pages 222–230 of Giancarlo Genta's book.
	//
	// When the tires operate outside the linear slip region, 
	// the vehicle's rigid-body equations must be corrected 
	// by introducing the chassis sideslip angle (beta) 
	// and its time derivative (betaRate).
	//
	// Genta derives these equations using Lagrangian dynamics 
	// for the bicycle vehicle model. 
	// On page 230, the equations are linearized
	// under the assumption that the sideslip angle is small:
	//
	// cos(beta) ~= 1
	// sin(beta) ~= beta
	//
	// This yields the following equations, whe all parameter are in teh body frame coodinate system:
	// 
	// 1) mass * localSpeed * (betaRate + yawRate) + mass * beta * localAccel = lateralForce
	// 2) mass * (localAccel - yawRate * localSpeed * beta) = longitudinalForce
	// 3) yawInertia * yawAlpha = yawTorque
	//
	// The resulting system is overdetermined, 
	// so additional constraints are required. 
	// This implementation applies the following assumptions:
	//
	// - The sideslip angle (beta) is clamped to +-25 degrees.
	// - Yaw angular acceleration (yawAlpha) is driven toward zero.
	// All calculations are performed in the vehicle's local reference frame.
	
	// we first calculate beta, betaRate, and yaw rate
	// for the current vehicle state.
	//
	//// calculate yawrate
	//const ndVector omega(frameMatrix.UnrotateVector(m_chassis->GetOmega()));
	//const ndFloat32 yawRate = omega.m_y;
	//
	//// using the forces from previuos frame calculate rear and front forces.
	//ndVector rearForce(m_chassis->GetForce());
	//ndVector frontForce(rearForce);
	//for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* tireNode = m_tireList.GetFirst(); tireNode; tireNode = tireNode->GetNext())
	//{
	//	const ndMultiBodyVehicleTireJoint* const tireJoint = tireNode->GetInfo();
	//
	//	// Get the tire force and torque.
	//	const ndMatrix tireMatrix(tireJoint->CalculateGlobalMatrix1());
	//	const ndVector8 jointForce(tireJoint->GetForceTorqueBody1());
	//	const ndVector locaTirePosit(frameMatrix.UntransformVector(tireMatrix.m_posit));
	//	if (locaTirePosit.m_x < ndFloat32(0.0f))
	//	{
	//		rearForce += jointForce.GetLow();
	//	}
	//	else
	//	{
	//		frontForce += jointForce.GetLow();
	//	}
	//}
	//rearForce = frameMatrix.UnrotateVector(rearForce);
	//frontForce = frameMatrix.UnrotateVector(frontForce);
	//
	//ndFloat32 rearLateralForce = rearForce.m_z;
	//ndFloat32 frontLateralForce = frontForce.m_z;
	//ndFloat32 rearLongitudinalForce = rearForce.m_x;
	//ndFloat32 frontLongitudinalForce = frontForce.m_x;
	//
	//// calculate acceleration, from equation 2 we get
	//// 2) mass * (localAccel - yawRate * localSpeed * beta) = longitudinalForce
	//ndVector massMatrix(m_chassis->GetMassMatrix());
	//ndFloat32 longitudinalForce = frontLongitudinalForce + rearLongitudinalForce;
	//ndFloat32 localAccel = (longitudinalForce - massMatrix.m_w * yawRate * localSpeed * beta) / massMatrix.m_w;
	//
	//// calculate beta rate, from equation 1
	//// 1) mass * localSpeed * (betaRate + yawRate) + mass * beta * localAccel = lateralForce
	//
	//// betaRate = (lateralForce - mass * beta * localAccel) / (mass * localSpeed) - yawRate;
	//
	//const ndFloat32 invTimeStep = ndFloat32(1.0) / timestep;
	//const ndFloat32 lateralForce = frontLateralForce + rearLateralForce;
	//const ndFloat32 betaRate = (beta - m_bicycleModel->m_beta) * invTimeStep;
	////const ndFloat32 equationBetaRate = (lateralForce - massMatrix.m_w * beta * localAccel) / (massMatrix.m_w * localSpeed) - yawRate;
	//ndFloat32 equationBetaRate = (lateralForce - massMatrix.m_w * beta * localAccel) / (massMatrix.m_w * localSpeed) - yawRate;
	//equationBetaRate *= 1;
	//
	//// update bicycle model corrent state parameters, 
	//// for using with tire contacts joints
	//m_bicycleModel->m_r = yawRate;
	//m_bicycleModel->m_beta = beta;
	//m_bicycleModel->m_u = localSpeed;
	//m_bicycleModel->m_vx = veloc.m_x;
	//m_bicycleModel->m_vz = veloc.m_z;
	//m_bicycleModel->m_betaRate = betaRate;
	//
	//// the stability critirial in to tweak the lateral
	//// tire friction conefficient in such way that 
	//// betaRate and Yaw rate and identical. 
	//// for that, using equation 1 
	//// 1) mass * localSpeed * (betaRate + yawRate) + mass * beta * localAccel = lateralForce
	//
	//// the expression betaRate + yawRate should be set to zero.
	//// therefore the target yawRate should be 
	//// targetYawRate = -betaRate
	//
	//// this information is now used by each tire contact
	//// to inforce a sideslip that forces yawRate match -betaRate
	//
	//// we only need beta, and U 
	//// but we are calcuation all the bicycle because they 
	//// are good for fedback monitoring,
	//
	//ndTrace(("beta=%g v=%g betaRate=%g yawRate=%g\n", beta * ndRadToDegree, localSpeed, betaRate, yawRate))
	//	if (ndAbs(beta * ndRadToDegree) > 30.0f)
	//	{
	//		m_bicycleModel->m_betaRate = betaRate;
	//	}
	//
	//for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* tireNode = m_tireList.GetFirst(); tireNode; tireNode = tireNode->GetNext())
	//{
	//	ndMultiBodyVehicleTireJoint* const tireJoint = tireNode->GetInfo();
	//	ndBodyKinematic* const tireBody = tireJoint->GetBody0()->GetAsBodyDynamic();
	//	ndBodyKinematic::ndContactMap::Iterator it(tireBody->GetContactMap());
	//	for (it.Begin(); it; it++)
	//	{
	//		ndContact* const contact = *it;
	//		if (contact->IsActive())
	//		{
	//			// I can calculate the velocity inline, but the 
	//			// method takes com into account, so use the methof and then restore omega
	//			const ndVector tireHubVeloc(m_chassis->GetVelocityAtPoint(tireBody->GetMatrix().m_posit));
	//			const ndVector savedChassisOmega(m_chassis->GetOmega());
	//
	//			ndVector desiredChassiOmega(savedChassisOmega);
	//			desiredChassiOmega.m_y = -m_bicycleModel->m_betaRate;
	//			m_chassis->SetOmegaNoSleep(desiredChassiOmega);
	//			const ndVector targetTireHubVeloc(m_chassis->GetVelocityAtPoint(tireBody->GetMatrix().m_posit));
	//			// restore ChassisOmega
	//			m_chassis->SetOmegaNoSleep(savedChassisOmega);
	//
	//			// now this these velocities and the tire joint 
	//			// calculate the requires tire slip ratio.
	//			ndVector tireHubRelativeVeloc(targetTireHubVeloc - tireHubVeloc);
	//			ndContactPointList& contactPoints = contact->GetContactPoints();
	//			for (ndContactPointList::ndNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
	//			{
	//				//ndContactMaterial& contactPoint = contactNode->GetInfo();
	//				//const ndVector larealDir(contactPoint.m_dir1);
	//				//ndFloat32 relAccel = larealDir.DotProduct(tireHubRelativeVeloc).GetScalar() * invTimeStep;
	//				//contactPoint.OverrideFriction1Accel(relAccel);
	//			}
	//		}
	//	}
	//}

	// the model is valid, just calcuate beta and bete rate
	// from the current chassis state, and save with the class.
	const ndVector omega(frameMatrix.UnrotateVector(m_chassis->GetOmega()));
	const ndFloat32 yawRate = omega.m_y;

	const ndFloat32 invTimeStep = ndFloat32(1.0f) / timestep;
	const ndFloat32 betaRate = (beta - m_bicycleModel->m_beta) * invTimeStep;
	m_bicycleModel->m_u = localSpeed;
	m_bicycleModel->m_r = yawRate;
	m_bicycleModel->m_beta = beta;
	m_bicycleModel->m_betaRate = betaRate;

	m_bicycleModel->m_modelIsValid = true;
}
#endif
