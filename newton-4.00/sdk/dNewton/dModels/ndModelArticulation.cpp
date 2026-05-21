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
#include "ndMesh.h"
#include "ndWorld.h"
#include "ndModel.h"
#include "ndMeshLoader.h"
#include "ndBodyDynamic.h"
#include "ndMeshComponents.h"
#include "ndModelArticulation.h"

ndModelArticulation::ndCenterOfMassDynamics::ndCenterOfMassDynamics()
	:m_omega(ndVector::m_zero)
	,m_veloc(ndVector::m_zero)
	,m_alpha(ndVector::m_zero)
	,m_accel(ndVector::m_zero)
	,m_force(ndVector::m_zero)
	,m_torque(ndVector::m_zero)
	,m_momentum(ndVector::m_zero)
	,m_angularMomentum(ndVector::m_zero)
	,m_centerOfMass(ndGetIdentityMatrix())
	,m_inertiaMatrix(ndGetZeroMatrix())
	,m_mass(ndFloat32 (0.0f))
{
}

ndModelArticulation::ndNode::ndNode(const ndSharedPtr<ndBody>& body, const ndSharedPtr<ndJointBilateralConstraint>& joint, ndNode* const parent)
	:ndNodeHierarchy<ndNode>()
	,m_body(body)
	,m_joint(joint)
	,m_name("")
{
	if (parent)
	{
		Attach(parent);
	}
}

ndModelArticulation::ndNode::ndNode(const ndNode& src)
	:ndNodeHierarchy<ndNode>(src)
	,m_body(src.m_body)
	,m_joint(src.m_joint)
	,m_name(src.m_name)
{
}

ndModelArticulation::ndNode::~ndNode()
{
}

ndModelArticulation::ndModelArticulation()
	:ndModel()
	,m_name("")
	,m_rootNode(nullptr)
	,m_closeLoops()
	,m_solveMulticore(true)
{
}

ndModelArticulation::ndModelArticulation(const ndModelArticulation& src)
	:ndModel(src)
	,m_name(src.m_name)
	,m_rootNode(nullptr)
	,m_closeLoops()
{
	ndAssert(0);
	ndAssert(src.GetRoot()->m_body->GetAsBodyDynamic());

	ndFixSizeArray<ndNode*, D_INV_IK_MAX_LINKS> stack;
	stack.PushBack(src.GetRoot());
	while (stack.GetCount())
	{
		ndNode* const node = stack.Pop();
		AddRootBody(new ndBodyDynamic(*node->m_body->GetAsBodyDynamic()));
		if (*node->m_joint)
		{

		}

		for (ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
		{
			stack.PushBack(child);
		}
	}
}

ndModelArticulation::~ndModelArticulation()
{
	if (m_rootNode)
	{
		delete m_rootNode;
	}
}

ndModel* ndModelArticulation::Clone() const
{
	return new ndModelArticulation(*this);
}

ndModelArticulation* ndModelArticulation::GetAsModelArticulation()
{
	return this;
}

const ndString& ndModelArticulation::GetName() const
{
	return m_name;
}

void ndModelArticulation::SetName(const ndString& name)
{
	m_name = name;
}

ndModelArticulation::ndNode* ndModelArticulation::GetRoot() const
{
	return m_rootNode;
}

void ndModelArticulation::ClearMemory()
{
	auto ClearBodies = [](ndModelArticulation::ndNode* node)
	{
		if (node->m_body)
		{
			ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
			body->SetOmega(ndVector::m_zero);
			body->SetVelocity(ndVector::m_zero);
		}
	};
	NodeIterator(ClearBodies);

	auto ClearJoints = [](ndModelArticulation::ndNode* node)
	{
		if (node->m_body)
		{
			ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
			ndBodyKinematic::ndContactMap& contactMap = body->GetContactMap();
			ndBodyKinematic::ndContactMap::Iterator it(contactMap);
			for (it.Begin(); it; it++)
			{
				ndContact* const contact = it.GetNode()->GetInfo();
				contact->ClearMemory();
			}
		}

		if (*node->m_joint)
		{
			node->m_joint->ClearMemory();
		}
	};
	NodeIterator(ClearJoints);
}

ndModelArticulation::ndNode* ndModelArticulation::AddRootBody(const ndSharedPtr<ndBody>& rootBody)
{
	ndAssert(!m_rootNode);
	ndSharedPtr <ndJointBilateralConstraint> dummyJoint;
	m_rootNode = new ndNode(rootBody, dummyJoint, nullptr);

	ndBodyKinematic* const dynBody = rootBody->GetAsBodyDynamic();
	ndAssert(dynBody);
	if (dynBody)
	{
		dynBody->SetModel(this);
	}
	return m_rootNode;
}

ndModelArticulation::ndNode* ndModelArticulation::AddLimb(ndNode* const parent, const ndSharedPtr<ndBody>& body, const ndSharedPtr<ndJointBilateralConstraint>& joint)
{
	ndAssert(m_rootNode);
	ndAssert(joint->GetBody0() == body->GetAsBodyKinematic());
	ndAssert(joint->GetBody1() == parent->m_body->GetAsBodyKinematic());

	ndBodyKinematic* const dynBody = body->GetAsBodyDynamic();
	ndAssert(dynBody);
	if (dynBody)
	{
		dynBody->SetModel(this);
	}
	return new ndNode(body, joint, parent);
}

const ndList<ndModelArticulation::ndNode, ndContainersFreeListAlloc<ndModelArticulation::ndNode>>& ndModelArticulation::GetCloseLoops() const
{
	return m_closeLoops;
}

void ndModelArticulation::AddCloseLoop(const ndSharedPtr<ndJointBilateralConstraint>& joint, const char* const name)
{
	#ifdef _DEBUG
	auto Check = [&](const ndBodyKinematic* const body)
	{
		if (body->GetInvMass() == ndFloat32(0.0f))
		{
			return false;
		}
		ndFixSizeArray<ndNode*, D_INV_IK_MAX_LINKS> stack;
		stack.PushBack(m_rootNode);
		while (stack.GetCount())
		{
			ndInt32 index = stack.GetCount() - 1;
			ndNode* const node = stack[index];
			stack.SetCount(index);
			if (node->m_body->GetAsBodyKinematic() == body)
			{
				return true;
			}

			for (ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
			{
				stack.PushBack(child);
			}
		}

		return false;
	};

	ndAssert(Check(joint->GetBody0()) || Check(joint->GetBody1()));
	#endif

	for (ndList<ndNode, ndContainersFreeListAlloc<ndNode>>::ndNode* node = m_closeLoops.GetFirst(); node; node = node->GetNext())
	{
		if (*node->GetInfo().m_joint == *joint)
		{
			return;
		}
	}

	char loopName[D_INV_IK_MAX_LINKS];
	snprintf(loopName, sizeof(loopName), "%s", name);

	ndSharedPtr<ndBody> body;
	ndList<ndNode, ndContainersFreeListAlloc<ndNode>>::ndNode* const node = m_closeLoops.Append(ndNode(body, joint, nullptr));
	node->GetInfo().m_name = loopName;
}

ndModelArticulation::ndNode* ndModelArticulation::FindByBodyId(ndInt32 bodyId) const
{
	if (m_rootNode)
	{
		for (ndModelArticulation::ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			if (node->m_body->GetId() == ndUnsigned32(bodyId))
			{
				return node;
			}
		}
	}

	return nullptr;
}

ndModelArticulation::ndNode* ndModelArticulation::FindByBody(const ndBody* const body) const
{
	return FindByBodyId(ndInt32 (body->GetId()));
}

ndModelArticulation::ndNode* ndModelArticulation::FindByName(const char* const name) const
{
	if (m_rootNode)
	{
		for (ndModelArticulation::ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			//if (strcmp(node->m_name.GetStr(), name) == 0)
			if (node->m_name.CompareIgnoreCase(name))
			{
				return node;
			}
		}

		for (ndList<ndNode, ndContainersFreeListAlloc<ndNode>>::ndNode* ptr = m_closeLoops.GetFirst(); ptr; ptr = ptr->GetNext())
		{
			ndNode* const node = &ptr->GetInfo();
			if (node->m_name.CompareIgnoreCase(name))
			{
				return node;
			}
		}
	}

	return nullptr;
}

ndModelArticulation::ndNode* ndModelArticulation::FindLoopByName(const char* const name) const
{
	if (m_rootNode)
	{
		for (ndList<ndNode, ndContainersFreeListAlloc<ndNode>>::ndNode* node = m_closeLoops.GetFirst(); node; node = node->GetNext())
		{
			if (node->GetInfo().m_name.CompareIgnoreCase(name))
			{
				return &node->GetInfo();
			}
		}
	}

	return nullptr;
}

ndModelArticulation::ndNode* ndModelArticulation::FindLoopByJoint(const ndJointBilateralConstraint* const joint) const
{
	if (m_rootNode)
	{
		for (ndList<ndNode, ndContainersFreeListAlloc<ndNode>>::ndNode* node = m_closeLoops.GetFirst(); node; node = node->GetNext())
		{
			if (*node->GetInfo().m_joint == joint)
			{
				return &node->GetInfo();
			}
		}
	}

	return nullptr;
}

void ndModelArticulation::OnAddWorld()
{
	ndAssert(m_world);
	ndFixSizeArray<ndNode*, D_INV_IK_MAX_LINKS> stack;
	if (m_rootNode)
	{
		stack.PushBack(m_rootNode);
		while (stack.GetCount())
		{
			ndInt32 index = stack.GetCount() - 1;
			ndNode* const node = stack[index];
			stack.SetCount(index);
			m_world->AddBody(node->m_body);
			if (node->m_joint)
			{
				m_world->AddJoint(node->m_joint);
			}
	
			for (ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
			{
				stack.PushBack(child);
			}
		}
	}
	
	for (ndList<ndNode, ndContainersFreeListAlloc<ndNode>>::ndNode* node = m_closeLoops.GetFirst(); node; node = node->GetNext())
	{
		m_world->AddJoint(node->GetInfo().m_joint);
	}
}

void ndModelArticulation::OnRemoveFromWorld()
{
	ndAssert(m_world);
	ndFixSizeArray<ndNode*, D_INV_IK_MAX_LINKS> stack;
	if (m_rootNode)
	{
		for (ndList<ndNode, ndContainersFreeListAlloc<ndNode>>::ndNode* node = m_closeLoops.GetFirst(); node; node = node->GetNext())
		{
			if (node->GetInfo().m_joint->m_worldNode)
			{
				m_world->RemoveJoint(*node->GetInfo().m_joint);
			}
		}
	
		stack.PushBack(m_rootNode);
		while (stack.GetCount())
		{
			ndInt32 index = stack.GetCount() - 1;
			ndNode* const node = stack[index];
			stack.SetCount(index);
			if (node->m_joint)
			{
				if (node->m_joint->m_worldNode)
				{
					m_world->RemoveJoint(*node->m_joint);
				}
			}
			if (node->m_body->GetAsBodyKinematic()->m_sceneNode)
			{
				m_world->RemoveBody(*node->m_body);
			}
	
			for (ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
			{
				stack.PushBack(child);
			}
		}
	}
}

void ndModelArticulation::SetMulticoreHint(bool hint)
{
	m_solveMulticore = hint;
}

void ndModelArticulation::SetSleep(ndFloat32 speed, ndFloat32 angularSpeed, ndFloat32 accel, ndFloat32 alpha) const
{
	accel *= accel;
	speed *= speed;
	alpha *= alpha;
	angularSpeed *= angularSpeed;

	bool isSleeping = true;
	for (ndModelArticulation::ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
	{
		const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
		if (!body->GetSleepState())
		{
			const ndVector bodyAccel(body->GetAccel());
			if (bodyAccel.DotProduct(bodyAccel).GetScalar() > accel)
			{
				isSleeping = false;
				break;
			}
			const ndVector bodyAlpha(body->GetAlpha());
			if (bodyAlpha.DotProduct(bodyAlpha).GetScalar() > alpha)
			{
				isSleeping = false;
				break;
			}
			const ndVector veloc(body->GetOmega());
			if (veloc.DotProduct(veloc).GetScalar() > speed)
			{
				isSleeping = false;
				break;
			}
			const ndVector omega(body->GetVelocity());
			if (omega.DotProduct(omega).GetScalar() > angularSpeed)
			{
				isSleeping = false;
				break;
			}
		}
	}
	if (isSleeping)
	{
		for (ndModelArticulation::ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
			body->RestoreSleepState(true);
		}
	}
}

ndModelArticulation::ndCenterOfMassDynamics ndModelArticulation::CalculateCentreOfMassKinematics(const ndMatrix& localFrame) const
{
	ndCenterOfMassDynamics dynamics;
	if (!m_rootNode)
	{
		return dynamics;
	}

	ndFixSizeArray<ndVector, D_INV_IK_MAX_LINKS> bodyCenter;
	ndFixSizeArray<const ndBodyKinematic*, D_INV_IK_MAX_LINKS> bodyArray;
	for (ndModelArticulation::ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
	{
		const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
		bodyArray.PushBack(body);
		const ndMatrix matrix(body->GetMatrix());
		const ndVector bodyCom(matrix.TransformVector(body->GetCentreOfMass()));
		bodyCenter.PushBack(bodyCom);

		ndFloat32 mass = body->GetMassMatrix().m_w;
		dynamics.m_mass += mass;
		dynamics.m_centerOfMass.m_posit += bodyCom.Scale(mass);
	}
	dynamics.m_centerOfMass.m_posit = dynamics.m_centerOfMass.m_posit.Scale(ndFloat32(1.0f) / dynamics.m_mass);
	dynamics.m_centerOfMass.m_posit.m_w = ndFloat32(1.0f);

	const ndInt32 numOfBodies = bodyArray.GetCount();
	for (ndInt32 i = 0; i < numOfBodies; ++i)
	{
		bodyCenter[i] = (bodyCenter[i] - dynamics.m_centerOfMass.m_posit) & ndVector::m_triplexMask;
	}

	for (ndInt32 i = 0; i < numOfBodies; ++i)
	{
		const ndBodyKinematic* const body = bodyArray[i];
		ndFloat32 mass = body->GetMassMatrix().m_w;
		ndMatrix bodyInertia(body->CalculateInertiaMatrix());

		const ndVector linearMomentum(body->GetVelocity().Scale(mass));
		const ndVector angularMomentum(bodyInertia.RotateVector(body->GetOmega()));
		dynamics.m_momentum += linearMomentum;
		dynamics.m_angularMomentum += angularMomentum;
		dynamics.m_angularMomentum += bodyCenter[i].CrossProduct(linearMomentum);

		ndFloat32 mag2 = bodyCenter[i].DotProduct(bodyCenter[i]).GetScalar();
		ndMatrix covariance(ndCovarianceMatrix(bodyCenter[i], bodyCenter[i]));
		for (ndInt32 j = 0; j < 3; j++)
		{
			bodyInertia[j][j] += mass * mag2;
			bodyInertia[j] -= covariance[j].Scale(mass);
			dynamics.m_inertiaMatrix[j] += bodyInertia[j];
		}
	}
	dynamics.m_inertiaMatrix.m_posit.m_w = ndFloat32(1.0f);

	dynamics.m_momentum = localFrame.UnrotateVector(dynamics.m_momentum);
	dynamics.m_angularMomentum = localFrame.UnrotateVector(dynamics.m_angularMomentum);
	dynamics.m_inertiaMatrix = localFrame * dynamics.m_inertiaMatrix * localFrame.OrthoInverse();
	dynamics.m_inertiaMatrix.m_posit = ndVector::m_wOne;

	dynamics.m_invInertiaMatrix = dynamics.m_inertiaMatrix.Inverse4x4();
	dynamics.m_omega = dynamics.m_invInertiaMatrix.RotateVector(dynamics.m_angularMomentum);
	dynamics.m_veloc = dynamics.m_momentum.Scale(ndFloat32(1.0f) / dynamics.m_mass);

	dynamics.m_centerOfMass.m_front = localFrame.m_front;
	dynamics.m_centerOfMass.m_up = localFrame.m_up;
	dynamics.m_centerOfMass.m_right = localFrame.m_right;

	return dynamics;
}

#if 0
// I already derived the equations for the linear acceleration and angular velocity
// of the center of mass. It is interesting to see that this paper follows essentially
// the same approach to derive the Zero Moment Point (ZMP):
// https://techunited.nl/media/files/humanoid/MaartenDekker_OPEN2009_Zero_Moment_Point_Method_for_Stable_Biped_Walking.pdf
//
// That said, my goal here is not to compute a ZMP.
// Instead, I am computing the dynamics of a solid point located at the center of mass.
// My contention is that these properties are far more general than the Zero Moment Point
// formulation.
ndModelArticulation::ndCenterOfMassDynamics ndModelArticulation::CalculateCentreOfMassDynamics(const ndMatrix& localFrame) const
{
	ndCenterOfMassDynamics dynamics;
	if (!m_rootNode)
	{
		return dynamics;
	}

	ndFixSizeArray<ndVector, D_INV_IK_MAX_LINKS> bodyCenter;
	ndFixSizeArray<const ndBodyKinematic*, D_INV_IK_MAX_LINKS> bodyArray;
	for (ndModelArticulation::ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
	{
		const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
		bodyArray.PushBack(body);
		const ndMatrix matrix(body->GetMatrix());
		const ndVector bodyCom(matrix.TransformVector(body->GetCentreOfMass()));
		bodyCenter.PushBack(bodyCom);

		ndFloat32 mass = body->GetMassMatrix().m_w;
		dynamics.m_mass += mass;
		dynamics.m_centerOfMass.m_posit += bodyCom.Scale(mass);
	}
	dynamics.m_centerOfMass.m_posit = dynamics.m_centerOfMass.m_posit.Scale(ndFloat32(1.0f) / dynamics.m_mass);
	dynamics.m_centerOfMass.m_posit.m_w = ndFloat32(1.0f);

	const ndInt32 numOfBodies = bodyArray.GetCount();
	for (ndInt32 i = 0; i < numOfBodies; ++i)
	{
		bodyCenter[i] = (bodyCenter[i] - dynamics.m_centerOfMass.m_posit) & ndVector::m_triplexMask;
	}

	for (ndInt32 i = 0; i < numOfBodies; ++i)
	{
		const ndBodyKinematic* const body = bodyArray[i];
		ndFloat32 mass = body->GetMassMatrix().m_w;
		ndMatrix bodyInertia(body->CalculateInertiaMatrix());

		const ndVector linearMomentum(body->GetVelocity().Scale(mass));
		const ndVector angularMomentum(bodyInertia.RotateVector(body->GetOmega()));
		
		ndVector netForce(body->GetForce());
		ndVector nextTorque(body->GetTorque() - body->GetOmega().CrossProduct(angularMomentum));
		const ndBodyKinematic::ndJointList& joints = body->GetJointList();
		for (ndBodyKinematic::ndJointList::ndNode* node = joints.GetFirst(); node; node = node->GetNext())
		{
			const ndConstraint* const joint = node->GetInfo();
			const ndVector8 forceTorqueBody ((joint->GetBody0() == body) ?
				joint->GetForceTorqueBody0() : joint->GetForceTorqueBody1());
			netForce += forceTorqueBody.GetLow();
			nextTorque += forceTorqueBody.GetHigh();
		}

		ndBodyKinematic::ndContactMap::Iterator it(body->GetContactMap());
		for (it.Begin(); it; it ++)
		{
			const ndConstraint* const joint = it.GetNode()->GetInfo();
			if (joint->IsActive())
			{
				const ndVector8 forceTorqueBody((joint->GetBody0() == body) ?
					joint->GetForceTorqueBody0() : joint->GetForceTorqueBody1());
				netForce += forceTorqueBody.GetLow();
				nextTorque += forceTorqueBody.GetHigh();
			}
		}
		nextTorque += bodyCenter[i].CrossProduct(netForce);

		dynamics.m_momentum += linearMomentum;
		dynamics.m_angularMomentum += angularMomentum;
		dynamics.m_angularMomentum += bodyCenter[i].CrossProduct(linearMomentum);

		dynamics.m_force += netForce;
		dynamics.m_torque += nextTorque;

		ndFloat32 mag2 = bodyCenter[i].DotProduct(bodyCenter[i]).GetScalar();
		ndMatrix covariance(ndCovarianceMatrix(bodyCenter[i], bodyCenter[i]));
		for (ndInt32 j = 0; j < 3; j++)
		{
			bodyInertia[j][j] += mass * mag2;
			bodyInertia[j] -= covariance[j].Scale(mass);
			dynamics.m_inertiaMatrix[j] += bodyInertia[j];
		}
	}
	dynamics.m_inertiaMatrix.m_posit.m_w = ndFloat32(1.0f);

	dynamics.m_force = localFrame.UnrotateVector(dynamics.m_force);
	dynamics.m_torque = localFrame.UnrotateVector(dynamics.m_torque);
	dynamics.m_momentum = localFrame.UnrotateVector(dynamics.m_momentum);
	dynamics.m_angularMomentum = localFrame.UnrotateVector(dynamics.m_angularMomentum);
	dynamics.m_inertiaMatrix = localFrame * dynamics.m_inertiaMatrix * localFrame.OrthoInverse();
	dynamics.m_inertiaMatrix.m_posit = ndVector::m_wOne;

	dynamics.m_invInertiaMatrix = dynamics.m_inertiaMatrix.Inverse4x4();
	dynamics.m_omega = dynamics.m_invInertiaMatrix.RotateVector(dynamics.m_angularMomentum);
	dynamics.m_veloc = dynamics.m_momentum.Scale(ndFloat32(1.0f) / dynamics.m_mass);
	dynamics.m_alpha = dynamics.m_invInertiaMatrix.RotateVector(dynamics.m_torque);
	dynamics.m_accel = dynamics.m_force.Scale(ndFloat32(1.0f) / dynamics.m_mass);
	
	dynamics.m_centerOfMass.m_front = localFrame.m_front;
	dynamics.m_centerOfMass.m_up = localFrame.m_up;
	dynamics.m_centerOfMass.m_right = localFrame.m_right;

	return dynamics;
}
#endif

// I already derived the equations for the linear acceleration and angular velocity
// of the center of mass. It is interesting to see that this paper follows essentially
// the same approach to derive the Zero Moment Point (ZMP):
// https://techunited.nl/media/files/humanoid/MaartenDekker_OPEN2009_Zero_Moment_Point_Method_for_Stable_Biped_Walking.pdf
//
// That said, my goal here is not to compute a ZMP.
// Instead, I am computing the dynamics of a solid point located at the center of mass.
// My contention is that these properties are far more general than the Zero Moment Point
// formulation.
ndModelArticulation::ndCenterOfMassDynamics ndModelArticulation::CalculateCentreOfMassDynamics(ndIkSolver& solver, const ndMatrix& localFrame, ndFixSizeArray<ndJointBilateralConstraint*, D_INV_IK_MAX_LINKS>& extraJoints, ndFloat32 timestep) const
{
	ndCenterOfMassDynamics dynamics;
	if (!m_rootNode)
	{
		return dynamics;
	}

	ndBodyKinematic* const rootBody = GetRoot()->m_body->GetAsBodyKinematic();
	ndSkeletonContainer* const skeleton = rootBody->GetSkeleton();
	ndAssert(skeleton);
	if (!skeleton)
	{
		return dynamics;
	}

	ndFixSizeArray<ndVector, D_INV_IK_MAX_LINKS> bodyCenter;
	ndFixSizeArray<const ndBodyKinematic*, D_INV_IK_MAX_LINKS> bodyArray;
	for (ndModelArticulation::ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
	{
		const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
		bodyArray.PushBack(body);
		const ndMatrix matrix(body->GetMatrix());
		const ndVector bodyCom(matrix.TransformVector(body->GetCentreOfMass()));
		bodyCenter.PushBack(bodyCom);

		ndFloat32 mass = body->GetMassMatrix().m_w;
		dynamics.m_mass += mass;
		dynamics.m_centerOfMass.m_posit += bodyCom.Scale(mass);
	}
	dynamics.m_centerOfMass.m_posit = dynamics.m_centerOfMass.m_posit.Scale(ndFloat32(1.0f) / dynamics.m_mass);
	dynamics.m_centerOfMass.m_posit.m_w = ndFloat32(1.0f);

	const ndInt32 numOfBodies = bodyArray.GetCount();
	for (ndInt32 i = 0; i < numOfBodies; ++i)
	{
		bodyCenter[i] = (bodyCenter[i] - dynamics.m_centerOfMass.m_posit) & ndVector::m_triplexMask;
	}

	ndJointBilateralConstraint** const extraJointsPtr = extraJoints.GetCount() ? &extraJoints[0] : nullptr;
	solver.SolverBegin(skeleton, extraJointsPtr, extraJoints.GetCount(), GetWorld(), timestep, 0);
	solver.Solve();
	auto CalculateComFullDynamics = [&dynamics, &bodyArray, &bodyCenter]()
	{
		for (ndInt32 i = bodyArray.GetCount() - 1; i >= 0; --i)
		{
			const ndBodyKinematic* const body = bodyArray[i];
			ndFloat32 mass = body->GetMassMatrix().m_w;

			ndMatrix bodyInertia(body->CalculateInertiaMatrix());
			const ndVector extForce(body->GetAccel().Scale(mass));
			const ndVector extForceTorque(bodyCenter[i].CrossProduct(extForce));
			const ndVector extTorque(bodyInertia.RotateVector(body->GetAlpha()));
			const ndVector rotationalAngularMomentum(bodyInertia.RotateVector(body->GetOmega()));
			const ndVector gyroTorque(body->GetOmega().CrossProduct(rotationalAngularMomentum));

			const ndVector linearMomentum(body->GetVelocity().Scale(mass));
			const ndVector linearAngularMomentum(bodyCenter[i].CrossProduct(linearMomentum));

			dynamics.m_momentum += linearMomentum;
			dynamics.m_angularMomentum += linearAngularMomentum;
			dynamics.m_angularMomentum += rotationalAngularMomentum;

			// centripetal should always be zero, or else the bodies will be flying apart from each other
			//const ndVector centripetal((comOmega.CrossProduct(bodyCom)).CrossProduct(linearMomentum));
			//totalToque += centripetal;
			dynamics.m_force += extForce;
			dynamics.m_torque += extTorque;
			dynamics.m_torque += gyroTorque;
			dynamics.m_torque += extForceTorque;

			ndFloat32 mag2 = bodyCenter[i].DotProduct(bodyCenter[i]).GetScalar();
			ndMatrix covariance(ndCovarianceMatrix(bodyCenter[i], bodyCenter[i]));
			for (ndInt32 j = 0; j < 3; j++)
			{
				bodyInertia[j][j] += mass * mag2;
				bodyInertia[j] -= covariance[j].Scale(mass);
				dynamics.m_inertiaMatrix[j] += bodyInertia[j];
			}
		}
		dynamics.m_inertiaMatrix.m_posit.m_w = ndFloat32(1.0f);
	};
	CalculateComFullDynamics();
	solver.SolverEnd();

	dynamics.m_force = localFrame.UnrotateVector(dynamics.m_force);
	dynamics.m_torque = localFrame.UnrotateVector(dynamics.m_torque);
	dynamics.m_momentum = localFrame.UnrotateVector(dynamics.m_momentum);
	dynamics.m_angularMomentum = localFrame.UnrotateVector(dynamics.m_angularMomentum);
	dynamics.m_inertiaMatrix = localFrame * dynamics.m_inertiaMatrix * localFrame.OrthoInverse();
	dynamics.m_inertiaMatrix.m_posit = ndVector::m_wOne;

	dynamics.m_invInertiaMatrix = dynamics.m_inertiaMatrix.Inverse4x4();
	dynamics.m_omega = dynamics.m_invInertiaMatrix.RotateVector(dynamics.m_angularMomentum);
	dynamics.m_veloc = dynamics.m_momentum.Scale(ndFloat32(1.0f) / dynamics.m_mass);
	dynamics.m_alpha = dynamics.m_invInertiaMatrix.RotateVector(dynamics.m_torque);
	dynamics.m_accel = dynamics.m_force.Scale(ndFloat32(1.0f) / dynamics.m_mass);

	dynamics.m_centerOfMass.m_front = localFrame.m_front;
	dynamics.m_centerOfMass.m_up = localFrame.m_up;
	dynamics.m_centerOfMass.m_right = localFrame.m_right;

	return dynamics;
}

void ndModelArticulation::SetTransform(const ndMatrix& matrix)
{
	if (m_rootNode)
	{
		const ndMatrix offset(m_rootNode->m_body->GetMatrix().OrthoInverse() * matrix);
		auto ApplyTransfrom = [this, &offset](ndModelArticulation::ndNode* const node)
		{
			if (node->m_body)
			{
				ndSharedPtr<ndBody> body(node->m_body);
				const ndMatrix matrix(body->GetMatrix() * offset);
				body->SetMatrix(matrix);
			}
		};
		NodeIterator(ApplyTransfrom);
	}
}

bool ndModelArticulation::IsCloseLoop(const ndNode* const loopJointNode) const
{
	for (ndList<ndNode, ndContainersFreeListAlloc<ndNode>>::ndNode* node = m_closeLoops.GetFirst(); node; node = node->GetNext())
	{
		if (&node->GetInfo() == loopJointNode)
		{
			return true;
		}
	}

	return false;
}

void ndModelArticulation::Serialize(ndMesh* const meshRootNode) const
{
	ndModelArticulation* const self = (ndModelArticulation*)this;
	auto SerializeToMesh = [this, meshRootNode](ndModelArticulation::ndNode* const node)
	{
		if (IsCloseLoop(node))
		{
			ndCloseLoopConstraints* const loopContainer = meshRootNode->GetLoopJoints();
			ndNode* const node0 = FindByBody(node->m_joint->GetBody0());
			ndNode* const node1 = FindByBody(node->m_joint->GetBody1());
			ndAssert(node0);
			ndAssert(node1);
			ndMesh* const meshNode0 = meshRootNode->FindByName(node0->m_name);
			ndMesh* const meshNode1 = meshRootNode->FindByName(node1->m_name);
			ndAssert(meshNode0);
			ndAssert(meshNode1);
			ndSharedPtr<ndMeshJoint> joint(node->m_joint->GetMeshJoint(meshNode0));
			ndSharedPtr<ndMeshLoopJoint> loopJoint(new ndMeshLoopJoint(loopContainer, joint, meshNode0, meshNode1));
			loopJoint->m_name = node->m_name;
			loopContainer->AddLoopJoint(loopJoint);
		}
		else
		{
			// it is a structural node, it will have a body
			ndMesh* meshNode = meshRootNode->FindByName(node->m_name);
			if (!meshNode)
			{
				ndAssert(node->GetParent());
				meshNode = new ndMesh();
				ndMesh* parentMeshNode = meshRootNode->FindByName(node->GetParent()->m_name);
				parentMeshNode->AddChild(meshNode);

				const ndMatrix offsetMatrix(node->m_body->GetMatrix() * node->GetParent()->m_body->GetMatrix().OrthoInverse());
				meshNode->SetName(node->m_name);
				meshNode->SetMatrix(offsetMatrix);
			}
			const ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
			if (node->m_body)
			{
				body->Serialize(meshNode);
				if (node->m_joint)
				{
					ndJointBilateralConstraint* const joint = *node->m_joint;
					meshNode->SetJoint(joint->GetMeshJoint(meshNode));

					// see if this node has a surrogate body
					ndMesh* parentMesh = meshNode->GetParent();
					while (!parentMesh->GetRigidBody())
					{
						parentMesh = parentMesh->GetParent();
					}

					//if (parentMesh->GetName() != node->GetParent()->m_name)
					if (!parentMesh->GetName().CompareIgnoreCase(node->GetParent()->m_name))
					{
						// this node has a surrogate parent,
						const ndMesh* const surrogateMesh = meshRootNode->FindByName(node->GetParent()->m_name);
						ndAssert(surrogateMesh);
						meshNode->GetJoint()->SetSurrogateParent(surrogateMesh);
					}
				}
			}
		}
	};
	// generate the ndMesh
	self->NodeIterator(SerializeToMesh);

	// add the colliding pairs
	for (ndInt32 i = ndInt32(m_collisionPairs.GetCount()) - 1; i >= 0; --i)
	{
		const ndNode* const node0 = FindByBodyId(ndInt32(m_collisionPairs[i].m_id0));
		const ndNode* const node1 = FindByBodyId(ndInt32(m_collisionPairs[i].m_id1));
		ndAssert(node0);
		ndAssert(node1);
		const ndMesh* const meshNode0 = meshRootNode->FindByName(node0->m_name);
		const ndMesh* const meshNode1 = meshRootNode->FindByName(node1->m_name);
		meshRootNode->SetCollidingSubSelection(meshNode0, meshNode1);
	}
}

void ndModelArticulation::Deserialize(const ndMesh* const rootNode)
{
	ndAssert(!m_rootNode);
	m_closeLoops.RemoveAll();
	if (m_rootNode)
	{
		delete m_rootNode;
		m_rootNode = nullptr;
	}

	ndFixSizeArray<ndMesh*, 256> sorrugatesNodes;
	auto BuildHiearchy = [this, &sorrugatesNodes](ndMesh* const meshNode)
	{
		if (meshNode->GetRigidBody())
		{ 
			if (!m_rootNode)
			{
				ndSharedPtr<ndBody> body(meshNode->GetRigidBody()->CreateObject());
				m_rootNode = AddRootBody(body);
				m_rootNode->m_name = meshNode->GetName();
			}
			else 
			{
				const ndSharedPtr<ndMeshJoint>& meshJoint = meshNode->GetJoint();
				if (!meshJoint->GetSurrogateParent())
				{
					ndSharedPtr<ndBody> body(meshNode->GetRigidBody()->CreateObject());
					const ndMesh* parentMesh = meshNode->GetParent();
					while (!parentMesh->GetRigidBody())
					{
						parentMesh = parentMesh->GetParent();
					}
					ndModelArticulation::ndNode* const parentNode = FindByName(parentMesh->GetName().GetStr());
					ndAssert(parentNode);
					ndBodyKinematic* const childBody = body->GetAsBodyKinematic();
					ndBodyKinematic* const parentBody = parentNode->m_body->GetAsBodyKinematic();
					ndSharedPtr<ndJointBilateralConstraint> joint(meshNode->GetJoint()->CreateObject(childBody, parentBody));
					ndModelArticulation::ndNode* const limbNode = AddLimb(parentNode, body, joint);
					limbNode->m_name = meshNode->GetName();
				}
				else
				{
					sorrugatesNodes.PushBack(meshNode);
				}
			}
		}
	};
	((ndMesh*)rootNode)->NodeIterator(BuildHiearchy);

	while (sorrugatesNodes.GetCount())
	{
		for (ndInt32 i = sorrugatesNodes.GetCount() - 1; i >= 0; --i)
		{
			const ndMesh* const surrogateMeshNode = sorrugatesNodes[i]->GetJoint()->GetSurrogateParent();
			ndModelArticulation::ndNode* parentNode = FindByName(surrogateMeshNode->GetName().GetStr());
			if (parentNode)
			{
				ndSharedPtr<ndBody> body(sorrugatesNodes[i]->GetRigidBody()->CreateObject());
				ndBodyKinematic* const childBody = body->GetAsBodyKinematic();
				ndBodyKinematic* const parentBody = parentNode->m_body->GetAsBodyKinematic();
				ndSharedPtr<ndJointBilateralConstraint> joint(sorrugatesNodes[i]->GetJoint()->CreateObject(childBody, parentBody));
				ndModelArticulation::ndNode* const limbNode = AddLimb(parentNode, body, joint);
				limbNode->m_name = sorrugatesNodes[i]->GetName();

				sorrugatesNodes[i] = sorrugatesNodes[sorrugatesNodes.GetCount() - 1];
				sorrugatesNodes.SetCount(sorrugatesNodes.GetCount() - 1);
				break;
			}
		}
	}

	const ndCloseLoopConstraints* const loops = rootNode->GetLoopJoints();
	if (loops)
	{
		for (ndList<ndSharedPtr<ndMeshLoopJoint>>::ndNode* loopPtr = loops->m_loopJoints.GetFirst(); loopPtr; loopPtr = loopPtr->GetNext())
		{
			const ndSharedPtr<ndMeshLoopJoint>& loopMeshJoint = loopPtr->GetInfo();
			ndModelArticulation::ndNode* const child = FindByName(loopMeshJoint->m_childNode->GetName().GetStr());
			ndModelArticulation::ndNode* const parent = FindByName(loopMeshJoint->m_parentNode->GetName().GetStr());
			ndSharedPtr<ndJointBilateralConstraint> loopJoint(loopMeshJoint->m_joint->CreateObject(child->m_body->GetAsBodyDynamic(), parent->m_body->GetAsBodyDynamic()));

			ndString name(parent->m_name + "_" + child->m_name);
			AddCloseLoop(loopJoint, name.GetStr());
		}
	}

	const ndCollidingPairs* const collidingPairs = rootNode->GetCollingPairs();
	if (collidingPairs)
	{
		for (ndList<ndSharedPtr<ndMeshCollidingPair>>::ndNode* pairPtr = collidingPairs->m_collidingPairs.GetFirst(); pairPtr; pairPtr = pairPtr->GetNext())
		{
			const ndSharedPtr<ndMeshCollidingPair>& pairMesh = pairPtr->GetInfo();
			ndModelArticulation::ndNode* const reference0 = FindByName(pairMesh->m_childNode->GetName().GetStr());
			ndModelArticulation::ndNode* const reference1 = FindByName(pairMesh->m_parentNode->GetName().GetStr());
			ndAssert(reference0);
			ndAssert(reference1);
			SetCollidingSubSelection(reference0, reference1);
		}
	}
}

ndMesh* ndModelArticulation::CreateDefaultMesh() const
{
	ndMesh* rootMesh = nullptr;
	auto BuildDefaultMesh = [this, &rootMesh](ndModelArticulation::ndNode* const node)
	{
		if (node->m_body)
		{
			const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
			const ndShapeInstance& collisionShape = body->GetCollisionShape();
			bool hasGeometry = ((ndShape*)collisionShape.GetShape())->GetAsShapeNull() ? false : true;
			ndMesh* const newMesh = hasGeometry ? new ndMesh(collisionShape) : new ndMesh();

			ndMatrix matrix(node->m_body->GetMatrix());
			if (!rootMesh)
			{
				rootMesh = newMesh;
			}
			else
			{
				ndMesh* const parent = rootMesh->FindByName(node->GetParent()->m_name);
				ndAssert(parent);
				parent->AddChild(ndSharedPtr<ndMesh>(newMesh));
				matrix = matrix * node->GetParent()->m_body->GetMatrix().OrthoInverse();
			}
			newMesh->SetName(node->m_name);
			newMesh->SetMatrix(matrix);
		}
	};
	// generate the ndMesh
	((ndModelArticulation*)this)->NodeIterator(BuildDefaultMesh);
	Serialize(rootMesh);

	return rootMesh;
}

void ndModelArticulation::SaveNdMesh(const char* const path) const
{
	ndSharedPtr<ndMesh> mesh(CreateDefaultMesh());
	ndMeshLoader loader(mesh);
	loader.SaveMesh(path);
}

bool ndModelArticulation::PairCollide(const ndBody* const body0, const ndBody* const body1) const
{
	ndInt32 i0 = 0;
	ndInt32 i1 = ndInt32 (m_collisionPairs.GetCount()) - 1;

	const ndCollindPairs pair(body0, body1);
	while ((i1 - i0) > 4)
	{
		ndInt32 index = (i1 + i0) / 2;
		if (m_collisionPairs[index].m_id >= pair.m_id)
		{
			i1 = index;
		}
		else
		{
			i0 = index;
		}
	}

	for (ndInt32 i = i0; i <= i1; ++i)
	{
		if (m_collisionPairs[i].m_id == pair.m_id)
		{
			return true;
		}
	}
	return false;
}

void ndModelArticulation::SetCollidingSubSelection(const ndNode* const node0, const ndNode* const node1)
{
	if (PairCollide(*node0->m_body, *node1->m_body))
	{
		return;
	}
	const ndCollindPairs newPair(*node0->m_body, *node1->m_body);

	m_collisionPairs.PushBack(newPair);
	ndInt32 index = ndInt32 (m_collisionPairs.GetCount()) - 2;
	while ((index >= 0) && (m_collisionPairs[index].m_id > newPair.m_id))
	{
		m_collisionPairs[index + 1] = m_collisionPairs[index];
		index--;
	}
	m_collisionPairs[index + 1] = newPair;
}