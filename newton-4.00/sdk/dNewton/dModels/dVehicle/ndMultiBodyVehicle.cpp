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
#include "ndJointHinge.h"
#include "ndBodyDynamic.h"
#include "ndBodyKinematic.h"
#include "ndMultiBodyVehicle.h"
#include "ndMultiBodyVehicleMotor.h"
#include "ndMultiBodyVehicleGearBox.h"
#include "ndMultiBodyVehicleTireJoint.h"
#include "ndMultiBodyVehicleTorsionBar.h"
#include "ndMultiBodyVehicleDifferential.h"
#include "ndMultiBodyVehicleDifferentialAxle.h"

#define D_MAX_CONTACT_SPEED_TRESHOLD	ndFloat32 (0.1f)
#define D_MAX_CONTACT_PENETRATION		ndFloat32 (1.0e-2f)
#define D_MIN_CONTACT_CLOSE_DISTANCE2	ndFloat32 (5.0e-2f * 5.0e-2f)

#define D_MAX_SIDESLIP_ANGLE			ndFloat32(20.0f)
#define D_MAX_STEERING_RATE				ndFloat32(0.03f)
#define D_MAX_SIZE_SLIP_RATE			ndFloat32(2.0f)

//#define D_PACEJKA_USE_REST_SPRUNG_WEIGHT
#define D_CONVERT_PACEJKA_FORCES_TO_FRICTION_COEFFICIENT

ndMultiBodyVehicle::ndDownForce::ndDownForce()
	:m_suspensionStiffnessModifier(ndFloat32(1.0f))
{
	m_downForceTable[0].m_speed = ndFloat32(0.0f) * ndFloat32(0.27f);
	m_downForceTable[0].m_forceFactor = 0.0f;
	m_downForceTable[0].m_aerodynamicDownforceConstant = ndFloat32(0.0f);

	m_downForceTable[1].m_speed = ndFloat32(30.0f) * ndFloat32(0.27f);
	m_downForceTable[1].m_forceFactor = 0.5f;
	m_downForceTable[1].m_aerodynamicDownforceConstant = CalculateFactor(&m_downForceTable[0]);

	m_downForceTable[2].m_speed = ndFloat32(60.0f) * ndFloat32(0.27f);
	m_downForceTable[2].m_forceFactor = 1.0f;
	m_downForceTable[2].m_aerodynamicDownforceConstant = CalculateFactor(&m_downForceTable[1]);

	m_downForceTable[3].m_speed = ndFloat32(140.0f) * ndFloat32(0.27f);
	m_downForceTable[3].m_forceFactor = 2.0f;
	m_downForceTable[3].m_aerodynamicDownforceConstant = CalculateFactor(&m_downForceTable[2]);

	m_downForceTable[4].m_speed = ndFloat32(1000.0f) * ndFloat32(0.27f);
	m_downForceTable[4].m_forceFactor = 2.0f;
	m_downForceTable[4].m_aerodynamicDownforceConstant = CalculateFactor(&m_downForceTable[3]);
}

ndFloat32 ndMultiBodyVehicle::ndDownForce::CalculateFactor(const ndSpeedForcePair* const entry0) const
{
	const ndSpeedForcePair* const entry1 = entry0 + 1;
	ndFloat32 num = ndMax(entry1->m_forceFactor - entry0->m_forceFactor, ndFloat32(0.0f));
	ndFloat32 den = ndMax(ndAbs(entry1->m_speed - entry0->m_speed), ndFloat32(1.0f));
	return num / (den * den);
}

ndFloat32 ndMultiBodyVehicle::ndDownForce::GetDownforceFactor(ndFloat32 speed) const
{
	ndAssert(speed >= ndFloat32(0.0f));
	ndInt32 index = 0;
	for (ndInt32 i = sizeof(m_downForceTable) / sizeof(m_downForceTable[0]) - 1; i; i--)
	{
		if (m_downForceTable[i].m_speed <= speed)
		{
			index = i;
			break;
		}
	}

	index = ndMin(index, ndInt32(sizeof(m_downForceTable) / sizeof(m_downForceTable[0])) - 2);
	ndFloat32 deltaSpeed = speed - m_downForceTable[index].m_speed;
	ndFloat32 downForceFactor = m_downForceTable[index].m_forceFactor + m_downForceTable[index + 1].m_aerodynamicDownforceConstant * deltaSpeed * deltaSpeed;
	//return downForceFactor * m_gravity;
	return downForceFactor;
}

class ndMultiBodyVehicle::ndComponentNotify : public ndBodyNotify
{
	public:
	D_CLASS_REFLECTION(ndComponentNotify, ndBodyNotify)

		ndComponentNotify(ndMultiBodyVehicle* const owner)
		:ndBodyNotify(ndVector::m_zero)
		, m_owner(owner)
	{
	}

	ndComponentNotify(const ndComponentNotify& src)
		:ndBodyNotify(src)
	{
	}

	ndBodyNotify* Clone() const override
	{
		return new ndComponentNotify(*this);
	}

	void OnApplyExternalForce(ndInt32, ndFloat32) override
	{
		ndBodyDynamic* const selfBody = GetBody()->GetAsBodyDynamic();
		selfBody->SetForce(ndVector::m_zero);
	}

	ndWeakPtr<ndMultiBodyVehicle> m_owner;
};

class ndMultiBodyVehicle::ndMotorNotify : public ndMultiBodyVehicle::ndComponentNotify
{
	public:
	ndMotorNotify(ndMultiBodyVehicle* const owner)
		:ndComponentNotify(owner)
	{
		// add some drag after the engine reach pick rpm
		const ndMultiBodyVehicleMotor::ndEngineTorqueCurve& curve = owner->m_motor->GetCurve();
		ndFloat32 rpm = curve.GetPickPowerRpm();
		ndFloat32 torque = curve.GetTorque(rpm);
		ndFloat32 omega = rpm * ndRpmToRadPerSec;
		m_dragCoeff = torque / (omega * omega);
	}

	void OnApplyExternalForce(ndInt32, ndFloat32) override
	{
		ndComponentNotify::OnApplyExternalForce(0, ndFloat32(0.0f));

		ndBodyDynamic* const selfBody = GetBody()->GetAsBodyDynamic();
		const ndMatrix axis(m_owner->GetMotor()->CalculateGlobalMatrix0());
		const ndVector omega(selfBody->GetOmega());
		ndFloat32 omegaSpeed = axis.m_front.DotProduct(selfBody->GetOmega()).GetScalar();
		if (omegaSpeed > ndFloat32 (1.0e-4f))
		{
			const ndVector clampOmega(omega - axis.m_front.Scale(omegaSpeed));
			selfBody->SetOmega(clampOmega);
			omegaSpeed = axis.m_front.DotProduct(clampOmega).GetScalar();
		}
		ndAssert(omegaSpeed <= ndFloat32(0.01f));

		ndVector torque(axis.m_front.Scale(m_dragCoeff * omegaSpeed * omegaSpeed));
		selfBody->SetTorque(torque);
	}

	ndFloat32 m_dragCoeff;
};

ndMultiBodyVehicle::ndMultiBodyVehicle(ndFloat32 gravityMagnitud)
	:ndModelArticulation()
	,m_localFrame(ndGetIdentityMatrix())
	,m_tireShape(new ndShapeWheel())
	,m_downForce()
	,m_sleepCounter(0)
	,m_debugFlags(DebugFlags(0))
{
	m_initialized = false;
	m_motor = nullptr;
	m_gearBox = nullptr;
	m_chassis = nullptr;

	m_steeringRate = D_MAX_STEERING_RATE;
	m_maxSideslipRate = D_MAX_SIZE_SLIP_RATE;
	m_maxSideslipAngle = D_MAX_SIDESLIP_ANGLE;

	m_gravityMagnitud = -ndAbs(gravityMagnitud);
	ndAssert(ndAbs(m_gravityMagnitud) > ndFloat32 (0.0f));
}

ndMultiBodyVehicle::DebugFlags ndMultiBodyVehicle::GetDebugFlags() const
{
	return m_debugFlags;
}

void ndMultiBodyVehicle::SetDebugFlags(DebugFlags flags)
{
	m_debugFlags = flags;
}

const ndMatrix& ndMultiBodyVehicle::GetLocalFrame() const
{
	return m_localFrame;
}

void ndMultiBodyVehicle::SetLocalFrame(const ndMatrix& localframe)
{
	m_localFrame.m_front = (localframe.m_front & ndVector::m_triplexMask).Normalize();
	m_localFrame.m_up = localframe.m_up & ndVector::m_triplexMask;
	m_localFrame.m_right = m_localFrame.m_front.CrossProduct(m_localFrame.m_up).Normalize();
	m_localFrame.m_up = m_localFrame.m_right.CrossProduct(m_localFrame.m_front).Normalize();
}

ndBodyDynamic* ndMultiBodyVehicle::GetChassis() const
{
	return (ndBodyDynamic*)*m_chassis;
}

ndMultiBodyVehicleMotor* ndMultiBodyVehicle::GetMotor() const
{
	return (ndMultiBodyVehicleMotor*)*m_motor;
}

ndMultiBodyVehicleGearBox* ndMultiBodyVehicle::GetGearBox() const
{
	return (ndMultiBodyVehicleGearBox*)*m_gearBox;
}

const ndList<ndMultiBodyVehicleTireJoint*>& ndMultiBodyVehicle::GetTireList() const
{
	return m_tireList;
}

ndMultiBodyVehicle* ndMultiBodyVehicle::GetAsMultiBodyVehicle()
{
	return this;
}

ndFloat32 ndMultiBodyVehicle::GetSpeed() const
{
	const ndVector dir(m_chassis->GetMatrix().RotateVector(m_localFrame.m_front));
	const ndFloat32 speed = ndAbs(m_chassis->GetVelocity().DotProduct(dir).GetScalar());
	return speed;
}

void ndMultiBodyVehicle::AddChassis(const ndSharedPtr<ndBody>& chassis)
{
	m_initialized = false;
	m_chassis = chassis->GetAsBodyDynamic();
	ndAssert(m_chassis);
	ndAssert(!GetRoot() || (GetRoot()->m_body == chassis));
	if (!FindByBody(*chassis))
	{
		AddRootBody(chassis);
	}
	//m_chassis->SetAngularDamping(ndVector(m_descriptor.m_chassisAngularDrag));
}

void ndMultiBodyVehicle::AddTire(const ndSharedPtr<ndBody>& tireBody, const ndSharedPtr<ndJointBilateralConstraint>& joint)
{
	m_initialized = false;
	ndAssert(m_chassis);
	ndAssert(!strcmp(joint->ClassName(), "ndMultiBodyVehicleTireJoint"));
	ndMultiBodyVehicleTireJoint* const tireJoint = (ndMultiBodyVehicleTireJoint*) * joint;
	m_tireList.Append(tireJoint);
	tireJoint->SetVehicleOwner(this);

	// make the inertial spherical
	ndBodyKinematic* const body = tireBody->GetAsBodyKinematic();
	ndVector inertia(body->GetMassMatrix());
	ndFloat32 maxInertia(ndMax(ndMax(inertia.m_x, inertia.m_y), inertia.m_z));
	inertia.m_x = maxInertia;
	inertia.m_y = maxInertia;
	inertia.m_z = maxInertia;
	body->SetMassMatrix(inertia);

	ndNode* const node = FindByBody(body);
	ndAssert(!node || ((node->m_body->GetAsBody() == body) && ((*node->m_joint == tireJoint))));
	if (!node)
	{
		ndAssert(tireJoint->GetBody1() == GetRoot()->m_body->GetAsBody());
		AddLimb(GetRoot(), tireBody, tireJoint);
	}
	body->GetAsBodyDynamic()->SetMaxLinearAndAngularIntegrationStep(ndFloat32(360.0f) * ndDegreeToRad, ndFloat32(10.0f));
}

ndMultiBodyVehicleTireJoint* ndMultiBodyVehicle::AddTire(const ndWheelDescriptor& desc, const ndSharedPtr<ndBody>& tire)
{
	ndAssert(m_chassis);
	ndMatrix tireFrame(ndGetIdentityMatrix());
	tireFrame.m_front = ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f));
	tireFrame.m_up = ndVector(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	tireFrame.m_right = ndVector(ndFloat32(-1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	const ndMatrix chassiMatrix(m_chassis->GetMatrix());
	ndMatrix matrix(tireFrame * m_localFrame * chassiMatrix);
	matrix.m_posit = tire->GetMatrix().m_posit;

	m_initialized = false;
	ndBodyDynamic* const tireBody = tire->GetAsBodyDynamic();

	// make tire inertia spherical
	//ndVector inertia(tireBody->GetMassMatrix());
	//ndFloat32 maxInertia(ndMax(ndMax(inertia.m_x, inertia.m_y), inertia.m_z));
	//inertia.m_x = maxInertia;
	//inertia.m_y = maxInertia;
	//inertia.m_z = maxInertia;
	//tireBody->SetMassMatrix(inertia);
	ndSharedPtr<ndJointBilateralConstraint> tireJoint (new ndMultiBodyVehicleTireJoint(matrix, tireBody, *m_chassis, desc, this));
	AddTire(tire, tireJoint);
	return m_tireList.GetLast()->GetInfo();
}

void ndMultiBodyVehicle::AddMotor(const ndSharedPtr<ndBody>& motorBody, const ndSharedPtr<ndJointBilateralConstraint>& motorJoint)
{
	m_initialized = false;
	ndAssert(m_chassis);
	ndAssert(!strcmp(motorJoint->ClassName(), "ndMultiBodyVehicleMotor"));
	m_motor = (ndMultiBodyVehicleMotor*)*motorJoint;
	m_motor->m_vehicle = this;
	
	ndNode* const node = FindByBody(*motorBody);
	ndAssert(!node || ((node->m_body->GetAsBody() == *motorBody) && ((*node->m_joint == *motorJoint))));
	if (!node)
	{
		ndAssert(motorJoint->GetBody1() == GetRoot()->m_body->GetAsBody());
		AddLimb(GetRoot(), motorBody, motorJoint);
	}
	motorBody->GetAsBodyDynamic()->SetMaxLinearAndAngularIntegrationStep(ndFloat32(2.0f * 360.0f) * ndDegreeToRad, ndFloat32(10.0f));
}

ndMultiBodyVehicleMotor* ndMultiBodyVehicle::AddMotor(ndFloat32 mass, ndFloat32 radius)
{
	ndAssert(m_chassis);
	m_initialized = false;
	ndSharedPtr<ndBody> motorBody(CreateInternalBodyPart(mass, radius));
	ndSharedPtr<ndJointBilateralConstraint> motorJoint(new ndMultiBodyVehicleMotor(motorBody->GetAsBodyKinematic(), this));
	AddMotor(motorBody, motorJoint);
	return *m_motor;
}

//ndMultiBodyVehicleTireJoint* ndMultiBodyVehicle::AddAxleTire(const ndMultiBodyVehicleTireJointInfo& desc, const ndSharedPtr<ndBody>& tire, const ndSharedPtr<ndBody>& axleBody)
//{
//	ndAssert(m_chassis);
//
//	m_initialized = false;
//	ndMatrix tireFrame(ndGetIdentityMatrix());
//	tireFrame.m_front = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
//	tireFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
//	tireFrame.m_right = ndVector(-1.0f, 0.0f, 0.0f, 0.0f);
//	ndMatrix matrix(tireFrame * m_localFrame * axleBody->GetMatrix());
//	matrix.m_posit = tire->GetMatrix().m_posit;
//	
//	ndBodyDynamic* const tireBody = tire->GetAsBodyDynamic();
//	// make tire inertia spherical
//	ndVector inertia(tireBody->GetMassMatrix());
//	ndFloat32 maxInertia(ndMax(ndMax(inertia.m_x, inertia.m_y), inertia.m_z));
//	inertia.m_x = maxInertia;
//	inertia.m_y = maxInertia;
//	inertia.m_z = maxInertia;
//	tireBody->SetMassMatrix(inertia);
//	
//	ndSharedPtr<ndJointBilateralConstraint> tireJoint(new ndMultiBodyVehicleTireJoint(matrix, tireBody, axleBody->GetAsBodyDynamic(), desc, this));
//	m_tireList.Append((ndMultiBodyVehicleTireJoint*)*tireJoint);
//	ndNode* const parentNode = FindByBody(*axleBody);
//	ndAssert(parentNode);
//	AddLimb(parentNode, tire, tireJoint);
//
//	tireBody->SetMaxLinearAndAngularIntegrationStep(ndFloat32(2.0f * 360.0f) * ndDegreeToRad, ndFloat32(10.0f));
//	return m_tireList.GetLast()->GetInfo();
//}

ndShapeInstance ndMultiBodyVehicle::CreateTireShape(ndFloat32 radius, ndFloat32 width) const
{
	ndShapeInstance tireCollision((ndShape*)*m_tireShape);
	ndVector scale(ndFloat32 (2.0f) * width, radius, radius, 0.0f);
	tireCollision.SetScale(scale);
	return tireCollision;
}

ndBodyKinematic* ndMultiBodyVehicle::CreateInternalBodyPart(ndFloat32 mass, ndFloat32 radius) const
{
	ndShapeInstance diffCollision(new ndShapeSphere(radius));
	diffCollision.SetCollisionMode(false);

	ndBodyDynamic* const body = new ndBodyDynamic();
	ndAssert(m_chassis);
	const ndMatrix matrix(m_localFrame * m_chassis->GetMatrix());
	body->SetMatrix(matrix);
	body->SetCollisionShape(diffCollision);
	body->SetMassMatrix(mass, diffCollision);
	body->SetMaxLinearAndAngularIntegrationStep(ndFloat32(2.0f * 360.0f) * ndDegreeToRad, ndFloat32(10.0f));
	return body;
}

void ndMultiBodyVehicle::AddDifferential(const ndSharedPtr<ndBody>& differentialBody, const ndSharedPtr<ndJointBilateralConstraint>& differentialJoint)
{
	ndAssert(m_chassis);
	ndAssert(!strcmp(differentialJoint->ClassName(), "ndMultiBodyVehicleDifferential"));

	ndMultiBodyVehicleDifferential* const joint = (ndMultiBodyVehicleDifferential*)*differentialJoint;
	m_differentialList.Append(joint);

	// make internal body parts non collidable
	//ndShapeInstance& collision = differentialBody->GetAsBodyKinematic()->GetCollisionShape();
	//collision.SetCollisionMode(false);

	ndNode* const node = FindByBody(*differentialBody);
	ndAssert(!node || ((node->m_body->GetAsBody() == *differentialBody) && ((*node->m_joint == *differentialJoint))));
	if (!node)
	{
		ndAssert(differentialJoint->GetBody1() == GetRoot()->m_body->GetAsBody());
		AddLimb(GetRoot(), differentialBody, differentialJoint);
	}
	differentialBody->GetAsBodyDynamic()->SetMaxLinearAndAngularIntegrationStep(ndFloat32(2.0f * 360.0f) * ndDegreeToRad, ndFloat32(10.0f));
}

ndMultiBodyVehicleDifferential* ndMultiBodyVehicle::AddDifferential(ndFloat32 mass, ndFloat32 radius, ndMultiBodyVehicleTireJoint* const leftTire, ndMultiBodyVehicleTireJoint* const rightTire, ndFloat32 slipOmegaLock)
{
	ndAssert(m_chassis);
	ndSharedPtr<ndBody> differentialBody (CreateInternalBodyPart(mass, radius));
	ndSharedPtr<ndJointBilateralConstraint> differentialJoint(new ndMultiBodyVehicleDifferential(differentialBody->GetAsBodyDynamic(), *m_chassis, slipOmegaLock));
	AddDifferential(differentialBody, differentialJoint);
	
	m_initialized = false;
	const ndVector pin(differentialBody->GetMatrix().RotateVector(differentialJoint->GetLocalMatrix0().m_front));
	const ndVector upPin(differentialBody->GetMatrix().RotateVector(differentialJoint->GetLocalMatrix0().m_up));
	const ndVector drivePin(leftTire->GetBody0()->GetMatrix().RotateVector(leftTire->GetLocalMatrix0().m_front));
	
	ndAssert(0);
	//ndSharedPtr<ndJointBilateralConstraint> leftAxle (new ndMultiBodyVehicleDifferentialAxle(pin, upPin, differentialBody->GetAsBodyKinematic(), drivePin, leftTire->GetBody0()));
	//ndSharedPtr<ndJointBilateralConstraint> rightAxle (new ndMultiBodyVehicleDifferentialAxle(pin, upPin.Scale(ndFloat32(-1.0f)), differentialBody->GetAsBodyKinematic(), drivePin, rightTire->GetBody0()));
	//AddDifferentialAxle(leftAxle);
	//AddDifferentialAxle(rightAxle);

	ndMultiBodyVehicleDifferential* const joint = (ndMultiBodyVehicleDifferential*)*differentialJoint;
	return joint;
}

ndMultiBodyVehicleDifferential* ndMultiBodyVehicle::AddDifferential(ndFloat32 mass, ndFloat32 radius, ndMultiBodyVehicleDifferential* const leftDifferential, ndMultiBodyVehicleDifferential* const rightDifferential, ndFloat32 slipOmegaLock)
{
	ndAssert(m_chassis);
	ndSharedPtr<ndBody> differentialBody(CreateInternalBodyPart(mass, radius));
	ndSharedPtr<ndJointBilateralConstraint> differentialJoint(new ndMultiBodyVehicleDifferential(differentialBody->GetAsBodyKinematic(), *m_chassis, slipOmegaLock));
	AddDifferential(differentialBody, differentialJoint);

	m_initialized = false;
	const ndVector pin(differentialBody->GetMatrix().RotateVector(differentialJoint->GetLocalMatrix0().m_front));
	const ndVector upPin(differentialBody->GetMatrix().RotateVector(differentialJoint->GetLocalMatrix0().m_up));
	const ndVector drivePin(leftDifferential->GetBody0()->GetMatrix().RotateVector(leftDifferential->GetLocalMatrix0().m_front.Scale(ndFloat32(-1.0f))));
	
	ndAssert(0);
	//ndSharedPtr<ndJointBilateralConstraint> leftAxle (new ndMultiBodyVehicleDifferentialAxle(pin, upPin, differentialBody->GetAsBodyKinematic(), drivePin, leftDifferential->GetBody0()));
	//ndSharedPtr<ndJointBilateralConstraint> rightAxle (new ndMultiBodyVehicleDifferentialAxle(pin, upPin.Scale(ndFloat32(-1.0f)), differentialBody->GetAsBodyKinematic(), drivePin, rightDifferential->GetBody0()));
	//AddDifferentialAxle(leftAxle);
	//AddDifferentialAxle(rightAxle);

	ndMultiBodyVehicleDifferential* const joint = (ndMultiBodyVehicleDifferential*)*differentialJoint;
	return joint;
}

void ndMultiBodyVehicle::AddDifferentialAxle(const ndSharedPtr<ndJointBilateralConstraint>& differentialAxleJoint)
{
	ndMultiBodyVehicleDifferentialAxle* const joint = (ndMultiBodyVehicleDifferentialAxle*)*differentialAxleJoint;
	ndNode* const node = FindLoopByJoint(joint);
	if (!node)
	{
		AddCloseLoop(differentialAxleJoint);
	}
}

void ndMultiBodyVehicle::AddTorsionBar(const ndSharedPtr<ndJointBilateralConstraint>& torsionBar)
{
	ndMultiBodyVehicleTorsionBar* const joint = (ndMultiBodyVehicleTorsionBar*)*torsionBar;

	const ndNode* const wheelNode = FindByBody(joint->GetBody0());
	if (wheelNode)
	{
		const ndMatrix matrix (joint->CalculateGlobalMatrix0());
		joint->m_referenceBody = wheelNode->GetParent()->m_body->GetAsBodyKinematic();
		joint->localReferenceFrame = matrix * joint->m_referenceBody->GetMatrix().OrthoInverse();
	}

	ndNode* const node = FindLoopByJoint(joint);
	if (!node)
	{
		AddCloseLoop(torsionBar);
	}
}

void ndMultiBodyVehicle::AddGearBox(const ndSharedPtr<ndJointBilateralConstraint>& gearBoxJoint)
{
	m_gearBox = (ndMultiBodyVehicleGearBox*)*gearBoxJoint;
	ndNode* const node = FindLoopByJoint(*m_gearBox);
	if (!node)
	{
		AddCloseLoop(gearBoxJoint);
	}
}

ndMultiBodyVehicleGearBox* ndMultiBodyVehicle::AddGearBox(ndMultiBodyVehicleDifferential* const differential)
{
	ndAssert(m_motor);
	m_initialized = false;
	const ndMatrix motorPinMatrix(m_motor->GetLocalMatrix0() * m_motor->GetBody0()->GetMatrix());
	const ndMatrix differentialPinMatrix(differential->GetLocalMatrix0() * differential->GetBody0()->GetMatrix());
	ndSharedPtr<ndJointBilateralConstraint> gearBox(new ndMultiBodyVehicleGearBox(ndFloat32 (1.0f), motorPinMatrix.m_front, m_motor->GetBody0(), differentialPinMatrix.m_front, differential->GetBody0()));
	AddGearBox(gearBox);
	return *m_gearBox;
}

void ndMultiBodyVehicle::ApplyAerodynamics(ndFloat32)
{
	m_downForce.m_suspensionStiffnessModifier = ndFloat32(1.0f);
	ndFloat32 gravity = m_downForce.GetDownforceFactor(GetSpeed()) * m_gravityMagnitud;
	if (ndAbs (gravity) > ndFloat32(1.0e-2f))
	{
		const ndVector up(m_chassis->GetMatrix().RotateVector(m_localFrame.m_up));
		const ndVector weight(m_chassis->GetForce());
		const ndVector downForce(up.Scale(gravity * m_chassis->GetMassMatrix().m_w));
		m_chassis->SetForce(weight + downForce);
		m_downForce.m_suspensionStiffnessModifier = up.DotProduct(weight).GetScalar() / up.DotProduct(weight + downForce.Scale (0.5f)).GetScalar();
		//dTrace(("%f\n", m_suspensionStiffnessModifier));
		
		for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
		{
			ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
			ndBodyKinematic* const tireBody = tire->GetBody0();
			const ndVector tireWeight(tireBody->GetForce());
			const ndVector tireDownForce(up.Scale(gravity * tireBody->GetMassMatrix().m_w));
			tireBody->SetForce(tireWeight + tireDownForce);
		}
	}
}

bool ndMultiBodyVehicle::CalculateNormalizedAlgniningTorque(ndMultiBodyVehicleTireJoint* const, ndFloat32 sideSlipTangent) const
{
	//I need to calculate the integration of the align torque 
	//using the calculate contact patch, form the standard brush model.
	//for now just set the torque to zero.
	ndFloat32 angle = ndAtan(sideSlipTangent);
	ndFloat32 a = ndFloat32(0.1f);

	ndFloat32 slipCos(ndCos(angle));
	ndFloat32 slipSin(ndSin(angle));
	ndFloat32 y1 = ndFloat32(2.0f) * slipSin * slipCos;
	ndFloat32 x1 = -a + ndFloat32(2.0f) * slipCos * slipCos;

	ndVector p1(x1, y1, ndFloat32(0.0f), ndFloat32(0.0f));
	ndVector p0(-a, ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));

	//ndFloat32 alignTorque = ndFloat32(0.0f);
	//ndFloat32 sign = ndSign(alignTorque);
	//tire->m_normalizedAligningTorque = sign * ndMax(ndAbs(alignTorque), ndAbs(tire->m_normalizedAligningTorque));
	return true;
}

void ndMultiBodyVehicle::ApplyAlignmentAndBalancing()
{
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
		ndBodyKinematic* const tireBody = tire->GetBody0()->GetAsBodyDynamic();
		ndBodyKinematic* const chassisBody = tire->GetBody1()->GetAsBodyDynamic();
	
		bool savedSleepState = tireBody->GetSleepState();
		tire->UpdateTireSteeringAngleMatrix();
		
		ndMatrix tireMatrix;
		ndMatrix chassisMatrix;
		tire->CalculateGlobalMatrix(tireMatrix, chassisMatrix);
		
		// align tire velocity
		const ndVector chassisVelocity(chassisBody->GetVelocityAtPoint(tireMatrix.m_posit));
		const ndVector relVeloc(tireBody->GetVelocity() - chassisVelocity);
		ndVector localVeloc(chassisMatrix.UnrotateVector(relVeloc));
		bool applyProjection = (localVeloc.m_x * localVeloc.m_x + localVeloc.m_z * localVeloc.m_z) > (ndFloat32(0.05f) * ndFloat32(0.05f));
		localVeloc.m_x *= ndFloat32(0.3f);
		localVeloc.m_z *= ndFloat32(0.3f);
		const ndVector tireVelocity(chassisVelocity + chassisMatrix.RotateVector(localVeloc));
		
		// align tire angular velocity
		const ndVector chassisOmega(chassisBody->GetOmega());
		const ndVector relOmega(tireBody->GetOmega() - chassisOmega);
		ndVector localOmega(chassisMatrix.UnrotateVector(relOmega));
		applyProjection = applyProjection || (localOmega.m_y * localOmega.m_y + localOmega.m_z * localOmega.m_z) > (ndFloat32(0.05f) * ndFloat32(0.05f));
		localOmega.m_y *= ndFloat32(0.3f);
		localOmega.m_z *= ndFloat32(0.3f);
		const ndVector tireOmega(chassisOmega + chassisMatrix.RotateVector(localOmega));
		
		if (applyProjection)
		{
			tireBody->SetOmega(tireOmega);
			tireBody->SetVelocity(tireVelocity);
		}
		tireBody->RestoreSleepState(savedSleepState);
	}
	
	for (ndList<ndMultiBodyVehicleDifferential*>::ndNode* node = m_differentialList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleDifferential* const diff = node->GetInfo();
		diff->AlignMatrix();
	}
	
	if (m_motor)
	{
		m_motor->AlignMatrix();
	}
}

void ndMultiBodyVehicle::Debug(ndConstraintDebugCallback& context) const
{
	if (!GetRoot())
	{
		return;
	}

	if (!GetRoot()->m_body->GetAsBodyKinematic()->GetSkeleton())
	{
		return;
	}

	// draw vehicle coordinade system;
	const ndBodyKinematic* const chassis = *m_chassis;
	ndAssert(chassis);
	const ndMatrix chassisMatrix(chassis->GetMatrix());

	// draw center of mass;
	const ndCenterOfMassDynamics kinematics(CalculateCentreOfMassKinematics(chassisMatrix));
	context.DrawFrame(kinematics.m_centerOfMass);
	
	// draw vehicle velocity
	const ndVector veloc(kinematics.m_veloc);
	const ndVector p0(kinematics.m_centerOfMass.m_posit + kinematics.m_centerOfMass.m_up.Scale(1.5f));
	const ndVector p1(p0 + kinematics.m_centerOfMass.m_front.Scale(2.0f));
	const ndVector p2(p0 + kinematics.m_centerOfMass.RotateVector(veloc.Scale(0.25f)));
	
	context.DrawLine(p0, p2, ndVector(1.0f, 1.0f, 0.0f, 0.0f));
	context.DrawLine(p0, p1, ndVector(1.0f, 0.0f, 0.0f, 0.0f));
	
	//// draw tires info
	//ndFloat32 scale = ndFloat32(3.0f);
	//const ndVector forceColor(ndFloat32(0.8f), ndFloat32(0.8f), ndFloat32(0.8f), ndFloat32(0.0f));
	//const ndVector lateralColor(ndFloat32(0.3f), ndFloat32(0.7f), ndFloat32(0.0f), ndFloat32(0.0f));
	//const ndVector longitudinalColor(ndFloat32(0.7f), ndFloat32(0.3f), ndFloat32(0.0f), ndFloat32(0.0f));
	//for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	//{
	//	ndMultiBodyVehicleTireJoint* const tireJoint = node->GetInfo();
	//	ndBodyKinematic* const tireBody = tireJoint->GetBody0()->GetAsBodyDynamic();
	//
	//	tireJoint->DebugJoint(context);
	//
	//	// draw tire forces
	//	const ndBodyKinematic::ndContactMap& contactMap = tireBody->GetContactMap();
	//	ndFloat32 tireGravities = scale / (kinematics.m_mass * m_gravityMagnitud);
	//	ndBodyKinematic::ndContactMap::Iterator it(contactMap);
	//	for (it.Begin(); it; it++)
	//	{
	//		ndContact* const contact = *it;
	//		if (contact->IsActive())
	//		{
	//			const ndContactPointList& contactPoints = contact->GetContactPoints();
	//			for (ndContactPointList::ndNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
	//			{
	//				const ndContactMaterial& contactPoint = contactNode->GetInfo();
	//				ndMatrix frame(contactPoint.m_normal, contactPoint.m_dir0, contactPoint.m_dir1, contactPoint.m_point);
	//
	//				ndVector localPosit(m_localFrame.UntransformVector(chassisMatrix.UntransformVector(contactPoint.m_point)));
	//				ndFloat32 offset = (localPosit.m_z > ndFloat32(0.0f)) ? ndFloat32(0.2f) : ndFloat32(-0.2f);
	//				frame.m_posit += contactPoint.m_dir0.Scale(offset);
	//				frame.m_posit += contactPoint.m_normal.Scale(0.1f);
	//
	//				// normal force
	//				ndFloat32 normalForce = -tireGravities * contactPoint.m_normal_Force.m_force;
	//				context.DrawLine(frame.m_posit, frame.m_posit + contactPoint.m_normal.Scale(normalForce), forceColor);
	//
	//				// lateral force
	//				ndFloat32 lateralForce = -tireGravities * contactPoint.m_dir0_Force.m_force;
	//				context.DrawLine(frame.m_posit, frame.m_posit + contactPoint.m_dir0.Scale(lateralForce), lateralColor);
	//
	//				// longitudinal force
	//				ndFloat32 longitudinalForce = tireGravities * contactPoint.m_dir1_Force.m_force;
	//				context.DrawLine(frame.m_posit, frame.m_posit + contactPoint.m_dir1.Scale(longitudinalForce), longitudinalColor);
	//			}
	//		}
	//	}
	//}

	auto DrawJoints = [this, &context](ndNode* const node)
	{
		if (node->m_joint)
		{
			if (strcmp(node->m_joint->ClassName(), ndMultiBodyVehicleTireJoint::StaticClassName()) == 0)
			{
				if (m_debugFlags & m_wheel)
				{
					node->m_joint->DebugJoint(context);
				}
			}
			else if (strcmp(node->m_joint->ClassName(), ndMultiBodyVehicleTorsionBar::StaticClassName()) == 0)
			{
				if (m_debugFlags & m_torsionBar)
				{
					node->m_joint->DebugJoint(context);
				}
			}
		}
	};
	((ndMultiBodyVehicle*)this)->NodeIterator(DrawJoints);
}

void ndMultiBodyVehicle::ApplyStabilityControl()
{
	ndAssert(m_chassis);
	const ndBodyKinematic* const chassis = *m_chassis;
	const ndVector veloc(chassis->GetVelocity());
	const ndMatrix chassisMatrix(chassis->GetMatrix());

#if 0
	// control sideslip beta by manipulation the steering
	// ignoring beta rate

	// this is really terrible
	const ndVector localVeloc(m_localFrame.UnrotateVector(chassisMatrix.UnrotateVector(veloc)));
	if (ndAbs(localVeloc.m_x) > ndFloat32(1.0f))
	{
		ndFloat32 sideslip = ndAtan2(localVeloc.m_z, localVeloc.m_x);
		if (ndAbs(sideslip * ndRadToDegree) > m_maxSideslipAngle)
		{
			ndFloat32 targetSteering = (sideslip > 0.0f) ? ndFloat32(-1.0f) : ndFloat32(1.0f);
			for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = GetTireList().GetFirst(); node; node = node->GetNext())
			{
				ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
				if (tire->m_info.m_steeringAngle != 0)
				{
					//ndFloat32 steering = tire->m_normalizedSteering0 + (targetSteering - tire->m_normalizedSteering0) * m_steeringRate;
					ndFloat32 steering = tire->m_normalizedSteering0 + (targetSteering - tire->m_normalizedSteering0) * 0.002;
					tire->m_normalizedSteering = steering;
				}
			}
		}
		else
		{
			for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = GetTireList().GetFirst(); node; node = node->GetNext())
			{
				ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
				if (tire->m_info.m_steeringAngle != 0)
				{
					ndFloat32 steering = tire->m_normalizedSteering0 + (tire->m_normalizedSteering - tire->m_normalizedSteering0) * m_steeringRate;
					tire->m_normalizedSteering = steering;
				}
			}
		}
	}

#elif 1
	// control beta rate by manipulation the steering
	// this may not be the be mode, but it does works;

	// this seem to be the best controller I got, but I really need a closed loop control
	const ndVector localVeloc(m_localFrame.UnrotateVector(chassisMatrix.UnrotateVector(veloc)));
	if (ndAbs(localVeloc.m_x) > ndFloat32(1.0f))
	{
		ndFloat32 sideslip = ndAtan2(localVeloc.m_z, localVeloc.m_x);
		if (ndAbs(sideslip * ndRadToDegree) > m_maxSideslipAngle)
		{
			const ndVector omega(chassis->GetOmega());
			const ndVector accel(chassis->GetAccel());
			const ndVector localOmega(m_localFrame.UnrotateVector(chassisMatrix.UnrotateVector(omega)));
			const ndVector localAccel(m_localFrame.UnrotateVector(chassisMatrix.UnrotateVector(accel)));

			// From Giancarlo Genta's book *Motor Vehicle Dynamics* (page 231, equation 5.52)
			// Original equation:
			// lateralAcceleration = longitudinalSpeed * (betaRate + yawRate) + beta * longitudinalAcceleration
			//
			// Note: When deriving the equation in a y-up coordinate system, it transforms into:
			// lateralAcceleration = longitudinalSpeed * (betaRate - yawRate) + beta * longitudinalAcceleration
			// 
			// In my opinion, this version makes more sense.
			//
			// Assuming constant longitudinal velocity, the term beta * longitudinalAcceleration becomes zero:
			// lateralAcceleration = longitudinalSpeed * (betaRate - yawRate)
			// from where we can get the beta rate
			// betaRate = lateralAcceleration / longitudinalSpeed + yawRate;
			ndFloat32 betaRate = localAccel.m_z / localVeloc.m_x + localOmega.m_y;

			if (ndAbs(betaRate) > m_maxSideslipRate)
			{
				ndFloat32 targetSteering = (betaRate > m_maxSideslipRate) ? ndFloat32(1.0f) : ndFloat32(-1.0f);
				//ndTrace(("a=%f b=%f b'=%f fz=%f w=%f steer=(", localAccel.m_z, sideslip * ndRadToDegree, betaRate, sideslipRate, localOmega.m_y));
				for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = GetTireList().GetFirst(); node; node = node->GetNext())
				{
					ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
					if (tire->m_info.m_steeringAngle != 0)
					{
						ndFloat32 steering = tire->m_normalizedSteering0 + (targetSteering - tire->m_normalizedSteering0) * m_steeringRate * 0.5f;
						tire->m_normalizedSteering = steering;
					}
				}
			}
			else
			{
				for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = GetTireList().GetFirst(); node; node = node->GetNext())
				{
					ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
					if (tire->m_info.m_steeringAngle != 0)
					{
						ndFloat32 steering = tire->m_normalizedSteering0 + (tire->m_normalizedSteering - tire->m_normalizedSteering0) * m_steeringRate;
						tire->m_normalizedSteering = steering;
					}
				}
			}
		}
	}

#else

	const ndVector omega(chassis->GetOmega());
	const ndVector accel(chassis->GetAccel());
	const ndVector alpha(chassis->GetAlpha());
	const ndVector localVeloc(m_localFrame.UnrotateVector(chassisMatrix.UnrotateVector(veloc)));
	const ndVector localOmega(m_localFrame.UnrotateVector(chassisMatrix.UnrotateVector(omega)));
	const ndVector localAccel(m_localFrame.UnrotateVector(chassisMatrix.UnrotateVector(accel)));
	const ndVector localAlpha(m_localFrame.UnrotateVector(chassisMatrix.UnrotateVector(alpha)));

	if (ndAbs(localVeloc.m_x) > ndFloat32(3.0f))
	{
		// From Giancarlo Genta's book *Motor Vehicle Dynamics* (page 231, equation 5.52)
		// Original equation:
		// lateralAcceleration = longitudinalSpeed * (betaRate + yawRate) + beta * longitudinalAcceleration
		//
		// Note: When deriving the equation in a y-up coordinate system, it transforms into:
		// lateralAcceleration = longitudinalSpeed * (betaRate - yawRate) + beta * longitudinalAcceleration
		// 
		// In my opinion, this version makes more sense.
		//
		// Assuming constant longitudinal velocity, the term beta * longitudinalAcceleration becomes zero:
		// lateralAcceleration = longitudinalSpeed * (betaRate - yawRate)
		// from where we can get the beta rate
		// betaRate = lateralAcceleration / longitudinalSpeed + yawRate;
		ndFloat32 betaRate = localAccel.m_z / localVeloc.m_x + localOmega.m_y;
		//if (ndAbs(betaRate) > D_MAX_SIZE_SLIP_RATE)
		if (ndAbs(betaRate) > ndFloat32 (0.15f))
		{
			const ndMatrix vehicleMatrix(m_chassis->GetMatrix());
			const ndVector com(vehicleMatrix.TransformVector(m_chassis->GetCentreOfMass()));
			for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
			{
				ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
				const ndMatrix hubMatrix(tire->CalculateBaseFrame());
				const ndVector hubPosit(hubMatrix.m_posit - com);
				const ndVector tireTorque(hubPosit.CrossProduct(tire->GetForceBody1()));
				const ndVector locaTorque(m_localFrame.UnrotateVector(vehicleMatrix.UnrotateVector(tireTorque)));

				ndVector force1(m_localFrame.UnrotateVector(hubMatrix.UnrotateVector(tire->GetForceBody1())));
				ndVector force0(tire->GetForceBody0());

				if (betaRate < 0.0f)
				{
					if (tire->GetBody0()->GetId() == 4)
					{
						ndTrace(("applyBreakControl: "));
						tire->SetHandBrake(0.02f);
					}
				}
				else
				{
					if (tire->GetBody0()->GetId() == 3)
					{
						ndTrace(("applyBreakControl: "));
						tire->SetHandBrake(0.02f);
					}
				}
			}
		}
	}
#endif
}

void ndMultiBodyVehicle::ApplyTireModel(ndFixSizeArray<ndTireContactPair, 128>& tireContacts)
{
	ndInt32 savedContactCount = tireContacts.GetCount();
	for (ndInt32 i = tireContacts.GetCount() - 1; i >= 0; --i)
	{
		ndContact* const contact = tireContacts[i].m_contact;
		ndMultiBodyVehicleTireJoint* const tire = tireContacts[i].m_tireJoint;
		ndContactPointList& contactPoints = contact->GetContactPoints();
		ndMatrix tireBasisMatrix(tire->GetLocalMatrix1() * tire->GetBody1()->GetMatrix());
		tireBasisMatrix.m_posit = tire->GetBody0()->GetMatrix().m_posit;
		bool useCoulombModel = (tire->m_frictionModel.m_frictionModel == ndTireFrictionModel::ndFrictionModel::m_coulomb) ? true : false;

		const ndVector tireUp(m_localFrame.UnrotateVector(tireBasisMatrix.m_up));
		const ndVector tireFront(m_localFrame.UnrotateVector(tireBasisMatrix.m_front));
		for (ndContactPointList::ndNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
		{
			ndContactMaterial& contactPoint = contactNode->GetInfo();
			const ndVector localNormal(m_localFrame.UntransformVector(contactPoint.m_normal));
			ndFloat32 contactPathLocation = ndAbs(localNormal.DotProduct(tireFront).GetScalar());
			if (contactPathLocation < ndFloat32(0.71f))
			{
				// align tire friction direction
				const ndVector longitudinalDir(localNormal.CrossProduct(tireFront).Normalize());
				const ndVector lateralDir(longitudinalDir.CrossProduct(localNormal));

				contactPoint.m_dir1 = m_localFrame.RotateVector(lateralDir);
				contactPoint.m_dir0 = m_localFrame.RotateVector(longitudinalDir);

				bool isOutOfContactPatch = useCoulombModel;
				if (!isOutOfContactPatch)
				{
					// check if the contact is in the contact patch,
					// the is the 45 degree point around the tire vehicle axis. 
					const ndVector dir(m_localFrame.UnrotateVector(contactPoint.m_point - tireBasisMatrix.m_posit));
					ndAssert(dir.DotProduct(dir).GetScalar() > ndFloat32(0.0f));
					ndFloat32 contactPatch = tireUp.DotProduct(dir.Normalize()).GetScalar();
					isOutOfContactPatch = (contactPatch > ndFloat32(-0.71f));
				}
				if (isOutOfContactPatch)
				{
					// remove this contact
					tireContacts[i] = tireContacts[tireContacts.GetCount() - 1];
					tireContacts.Pop();
					break;
				}
			}
		}
	}

	if (tireContacts.GetCount() && (tireContacts.GetCount() == savedContactCount))
	{
		for (ndInt32 i = tireContacts.GetCount() - 1; i >= 0 ; --i)
		{
			ndContact* const contact = tireContacts[i].m_contact;
			ndMultiBodyVehicleTireJoint* const tire = tireContacts[i].m_tireJoint;
			ndContactPointList& contactPoints = contact->GetContactPoints();
			for (ndContactPointList::ndNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
			{
				ndContactMaterial& contactPoint = contactNode->GetInfo();
				switch (tire->m_frictionModel.m_frictionModel)
				{
					case ndTireFrictionModel::m_pacejkaSport:
					case ndTireFrictionModel::m_pacejkaTruck:
					case ndTireFrictionModel::m_pacejkaCustom:
					case ndTireFrictionModel::m_pacejkaUtility:
					{
						if (PacejkaTireModel(tire, contactPoint))
						{
							contact->InvalicatdeCache();
						}
						break;
					}

					case ndTireFrictionModel::m_coulombCicleOfFriction:
					{
						CoulombFrictionCircleTireModel(tire, contactPoint);
						break;
					}

					case ndTireFrictionModel::m_coulomb:
					default:
					{
						CoulombTireModel(tire, contactPoint);
						break;
					}
				}
			}
		}
		//ApplyStabilityControl();
	}
}

void ndMultiBodyVehicle::ApplyTireModel()
{
	ndFixSizeArray<ndTireContactPair, 128> tireContacts;
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
		ndAssert(((ndShape*)tire->GetBody0()->GetCollisionShape().GetShape())->GetAsShapeChamferCylinder());

		tire->m_lateralSlip = ndFloat32(0.0f);
		tire->m_longitudinalSlip = ndFloat32(0.0f);
		tire->m_normalizedAligningTorque = ndFloat32(0.0f);

		const ndBodyKinematic::ndContactMap& contactMap = tire->GetBody0()->GetContactMap();
		ndBodyKinematic::ndContactMap::Iterator it(contactMap);
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = *it;
			if (contact->IsActive())
			{
				ndContactPointList& contactPoints = contact->GetContactPoints();
				// for mesh collision we need to remove contact duplicates, 
				// these are contact produced by two or more polygons, 
				// that can produce two contact so are close that they can generate 
				// ill formed rows in the solver mass matrix
				for (ndContactPointList::ndNode* contactNode0 = contactPoints.GetFirst(); contactNode0; contactNode0 = contactNode0->GetNext())
				{
					const ndContactPoint& contactPoint0 = contactNode0->GetInfo();
					for (ndContactPointList::ndNode* contactNode1 = contactNode0->GetNext(); contactNode1; contactNode1 = contactNode1->GetNext())
					{
						const ndContactPoint& contactPoint1 = contactNode1->GetInfo();
						const ndVector error(contactPoint1.m_point - contactPoint0.m_point);
						ndFloat32 err2 = error.DotProduct(error).GetScalar();
						if (err2 < D_MIN_CONTACT_CLOSE_DISTANCE2)
						{
							contactPoints.Remove(contactNode1);
							break;
						}
					}
				}
				ndTireContactPair pair;
				pair.m_contact = contact;
				pair.m_tireJoint = tire;
				tireContacts.PushBack(pair);
			}
		}
	}

	ApplyTireModel(tireContacts);

	// save the steering
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
		tire->m_normalizedSteering0 = tire->m_normalizedSteering;
	}
}

bool ndMultiBodyVehicle::CoulombTireModel(ndMultiBodyVehicleTireJoint* const joint, ndContactMaterial& contactPoint) const
{
	const ndFloat32 frictionCoefficient = contactPoint.m_material.m_staticFriction0;

	// handling dynamics friction manually
	ndFloat32 dynamicFrictionCoef = joint->m_isApplyingBrakes ? ndFloat32(0.75f) : ndFloat32(1.0f);

	contactPoint.m_material.m_staticFriction0 = frictionCoefficient;
	contactPoint.m_material.m_staticFriction1 = frictionCoefficient;
	contactPoint.m_material.m_dynamicFriction0 = frictionCoefficient * dynamicFrictionCoef;
	contactPoint.m_material.m_dynamicFriction1 = frictionCoefficient * dynamicFrictionCoef;
	return true;
}

bool ndMultiBodyVehicle::CoulombFrictionCircleTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint) const
{
	return CoulombTireModel(tire, contactPoint);
}

bool ndMultiBodyVehicle::PacejkaTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint) const
{
	// According to Wikipedia, the Pacejka Magic Formula is typically written as:
	//
	// F = D * sin(C * atan(Bx * (1 - E) + E * atan(Bx)))
	//
	// This form does not include the parameter phi.
	//
	// Giancarlo Genta introduces horizontal and vertical shifts to extend the model
	// to operating conditions near zero slip:
	//
	// F = D * sin(C * atan(Bx * (1 - E) * (phi + Sh) + E * atan(Bx * (phi + Sh)))) + Sv
	//
	// My primary challenge with this formulation is determining the values of
	// C, D, E, Bx, phi, Sh, and Sv for each force and moment component.
	//
	// Genta provides tables of coefficients (a1 through a13) for several example
	// vehicles, but does not clearly explain how these coefficients map to the
	// Magic Formula parameters B, C, D, and E. The relationship must largely be
	// inferred from the surrounding examples and equations.
	//
	// The most useful reference I have found on this topic is:
	// http://www-cdr.stanford.edu/dynamic/bywire/tires.pdf
	//
	// More generally, I have struggled with Pacejka implementations because the
	// treatment of units in Genta's discussion (pp. 60–78) appears inconsistent in
	// several places, making it difficult to verify that the implementation is
	// producing physically meaningful results.
	//
	// I have attempted to implement the model multiple times over the years, 
	// but have never been fully satisfied with the results. 
	// Currently, I am comparing it against the Brush tire model, 
	// which seems to produce more intuitive behavior.
	//
	// One aspect that concerns me is that tire forces do not appear to depend
	// explicitly on normal load. 
	// Instead, load sensitivity is incorporated through parameter D.
	//
	// Conceptually, this seems counterintuitive. For example, a pickup truck with
	// a heavy payload should exhibit different lateral force characteristics than
	// the same vehicle when unloaded. If D remains constant, the model predicts
	// identical behavior regardless of tire load, which does not appear physically
	// realistic.
	const ndBodyKinematic* const tireBody = tire->GetBody0()->GetAsBodyKinematic();
	const ndBodyKinematic* const otherBody = (contactPoint.m_body0 == tireBody) ? contactPoint.m_body1 : contactPoint.m_body0;

	const ndVector longitudDir(contactPoint.m_dir0);
	const ndVector contactVeloc0(tireBody->GetVelocity());
	const ndVector contactVeloc1(otherBody->GetVelocityAtPoint(contactPoint.m_point));
	const ndVector wheelComVeloc(contactVeloc0 - contactVeloc1);
	const ndFloat32 wheelComSpeed_x = wheelComVeloc.DotProduct(longitudDir).GetScalar();
	if (ndAbs(wheelComSpeed_x) < D_MAX_CONTACT_SPEED_TRESHOLD)
	{
		// handle vehicle is at rest by just doing normal rigid body dynamics.
		return true;
	}

	// calculate lateral slip angle
	const ndVector lateralDir(contactPoint.m_dir1);
	// use a dead zone and them use Sv for nonzero lateral force
	const ndFloat32 speed_z = wheelComVeloc.DotProduct(lateralDir).GetScalar();
	const ndFloat32 wheelComSpeed_z = (speed_z > ndFloat32(1.0e-3f) || (speed_z < ndFloat32(-1.0e-3f))) ? speed_z : ndFloat32(0.0f);
	const ndFloat32 sideSlipAngleInRadians = ndAtan2(wheelComSpeed_z, ndAbs(wheelComSpeed_x));
	tire->m_lateralSlip = ndMax(tire->m_lateralSlip, ndAbs(sideSlipAngleInRadians));

	// calculate longitudinal slip
	const ndVector contactVeloc(tireBody->GetVelocityAtPoint(contactPoint.m_point) - contactVeloc1);
	const ndFloat32 vr_x = contactVeloc.DotProduct(longitudDir).GetScalar();
	const ndFloat32 longitudialSlip = ndClamp(vr_x / wheelComSpeed_x, ndFloat32(-100.0f), ndFloat32(100.0f));
	tire->m_longitudinalSlip = ndMax(tire->m_longitudinalSlip, longitudialSlip);

	//I am now using the direct coeficients B, C, E, D as explained in 
	//The multibody system approach to vehicle dynamics page 300 to 306
	auto LongitudinalForce = [](const ndTireFrictionModel::ndPacejkaTireModel& model, ndFloat32 phi, ndFloat32 frictionCoefficient)
	{
		return model.Evaluate(phi, frictionCoefficient);
	};

	auto LateralForce = [](const ndTireFrictionModel::ndPacejkaTireModel& model, ndFloat32 phi, ndFloat32 frictionCoefficient)
	{
		// phi should be in degress
		phi = ndAbs(ndRadToDegree * phi);
		return model.Evaluate(phi, frictionCoefficient);
	};

	//now apply the combine effect, according to Genta book page 76
	const ndTireFrictionModel& frictionModel = tire->m_frictionModel;

	// ----- Combined‑force calculation notes -----
	//
	// The derivation below isn’t documented in either Pacejka or Genta books.
	// Both books only give a brief example and leave the details unexplained.
	// After two decades, the method still feels more like bad heuristicks than
	// sound engineering: the equations mix units and fail dimensional checks.
	// It may be possible that works in some confined experimetal steady state 
	// laboratory condition, but the are not scalable for symulations. 

	// Pages 80‑83 outline *two* inconsistent ways to calculate combined slip.
	// - Method 1 (implemented here) produces sensible, stable results,
	//   yet the book itself calls it “incorrect.”
	// - Method 2 is recommended by the authors, but I’ve never managed
	//   to get anything remotely realistic from it.

#if 1
	// Until a better reference turns up, Method 0 remains the least‑bad
	// option I’ve found stable in practice, if not theoretically satisfying, therefore I am going with that. 
	// I still find the lateral force some what too strong.

	//I am assuming sv and sv to be zero.
	//under these conditions u and v become
	ndFloat32 den = ndFloat32(1.0f) + ndAbs(longitudialSlip);
	ndFloat32 phi_x = -longitudialSlip / den;
	ndFloat32 phi_z = ndTan(sideSlipAngleInRadians) / den;
	ndFloat32 phi2 = phi_x * phi_x + phi_z * phi_z;
	if (phi2 < ndFloat32 (1.0e-6f))
	{
		// this is the vanishing phi
		return true;
	}
	const ndFloat32 phi = ndSqrt(phi2);

	const ndFloat32 isotropicMaterialFriction = contactPoint.m_material.m_staticFriction0;
#ifdef D_PACEJKA_USE_REST_SPRUNG_WEIGHT
	const ndFloat32 sprungWeight = frictionModel.m_sprungWeight;
	const ndFloat32 pacekaAmplitud = sprungWeight * isotropicMaterialFriction;
#else
	const ndFloat32 hackStiffness = ndFloat32(5.0f);
	const ndFloat32 sprungWeight = contactPoint.m_normal_Force.GetInitialGuess() + ndFloat32 (1.0f);
	const ndFloat32 pacekaAmplitud = sprungWeight * hackStiffness * isotropicMaterialFriction;
#endif
	const ndFloat32 pure_fz = LateralForce(frictionModel.m_lateralPacejka, sideSlipAngleInRadians, tire->m_lateralStiffness * pacekaAmplitud);
	const ndFloat32 pure_fx = LongitudinalForce(frictionModel.m_longitudinalPacejka, longitudialSlip, tire->m_longitudinalStiffness * pacekaAmplitud);

	const ndFloat32 fx = pure_fx * phi_x / phi;
	const ndFloat32 fz = pure_fz * phi_z / phi;
#else

	// In the second method the slips and the sizdeSlip use a 
	// normalized dimensionless slip for both lateral and longitudinal
	// with not explanation as to how the Pacekka equation can be called 
	// with these dimension less values.
	// to me, this seems like nonsence, unless lots of details are omitted in the book.
	const ndFloat32 dimensionLessPhi_x = longitudialSlip / frictionModel.m_longitudinalPacejka.m_normalizingPhi;
	const ndFloat32 dimensionLessPhi_z = sideSlipAngleInRadians / (frictionModel.m_lateralPacejka.m_normalizingPhi * ndDegreeToRad * 0.5f);

	// since Sv and Sh are zero of very small, them delta alpha is zero. 
	const ndFloat32 dimensionLessCombined_phi = ndSqrt(dimensionLessPhi_x * dimensionLessPhi_x + dimensionLessPhi_z * dimensionLessPhi_z);

	const ndFloat32 modifiedPhi_x = longitudialSlip * dimensionLessCombined_phi;
	const ndFloat32 modifiedPhiInRadians_z = sideSlipAngleInRadians * dimensionLessCombined_phi;

	const ndFloat32 sprungWeight = frictionModel.m_sprungWeight * contactPoint.m_material.m_staticFriction0;
	//const ndFloat32 frictionCoefficient = contactPoint.m_material.m_staticFriction0;
	ndFloat32 pureFz = LateralForce(frictionModel.m_lateralPacejka, modifiedPhiInRadians_z, sprungWeight * tire->m_lateralStiffness);
	ndFloat32 pureFx = LongitudinalForce(frictionModel.m_longitudinalPacejka, modifiedPhi_x, sprungWeight * tire->m_longitudinalStiffness);

	// after we calculate the compensated longitudinal and lateral forces,
	// they have to be scaled back
	ndFloat32 fx = pureFx * dimensionLessPhi_x / dimensionLessCombined_phi;
	ndFloat32 fz = pureFz * dimensionLessPhi_z / dimensionLessCombined_phi;
#endif

#ifdef D_CONVERT_PACEJKA_FORCES_TO_FRICTION_COEFFICIENT	
	// this method should be more relistic, because it uses the 
	// actual tire sprung weight from previos time step. 
	// however the lag seems to make the vehicle more unresponsive.
	// and it needs to be compensated with higher stiffness.
	ndFloat32 lateralFrictionCoefficient = ndAbs(fz) / sprungWeight;
	ndFloat32 longitudinalFrictionCoefficient = ndAbs(fx) / sprungWeight;
	contactPoint.m_material.m_staticFriction0 = longitudinalFrictionCoefficient;
	contactPoint.m_material.m_dynamicFriction0 = longitudinalFrictionCoefficient;
	contactPoint.m_material.m_staticFriction1 = lateralFrictionCoefficient;
	contactPoint.m_material.m_dynamicFriction1 = lateralFrictionCoefficient;

#else
	// using the calculated forces as if they are the final solver calculation.
	// make the vehicle very reponsive, but I really don't like that 
	// the lateral and longitudinal forces use the rest tire sprung weight.
	contactPoint.m_material.m_staticFriction0 = ndAbs(fx);
	contactPoint.m_material.m_dynamicFriction0 = ndAbs(fx);
	contactPoint.m_material.m_staticFriction1 = ndAbs(fz);
	contactPoint.m_material.m_dynamicFriction1 = ndAbs(fz);
	ndUnsigned32 newFlags = contactPoint.m_material.m_flags | m_override0Friction | m_override1Friction;
	contactPoint.m_material.m_flags = newFlags;
#endif
	return true;
}

void ndMultiBodyVehicle::CalculateRestSprungWeight()
{
	const ndMatrix savedMatrix(GetRoot()->m_body->GetMatrix());
	SetTransform(ndGetIdentityMatrix());

	const ndInt32 buffersCapacity = 128;
	ndFixSizeArray<ndInt32, buffersCapacity> pairM0;
	ndFixSizeArray<ndInt32, buffersCapacity> pairM1;
	ndFixSizeArray<ndInt32, buffersCapacity> bodyIndex;
	ndFixSizeArray<ndFloat32, buffersCapacity> rhsAccel;
	ndFixSizeArray<ndBodyDynamic*, buffersCapacity> bodyArray;
	ndFixSizeArray<ndJacobianPair, buffersCapacity> jacobianArray;
	ndFixSizeArray<ndMultiBodyVehicleTireJoint*, buffersCapacity> tireArray;

	ndBodyDynamic emptyBody;
	bodyIndex.PushBack(0);
	bodyArray.PushBack(&emptyBody);
	auto GetStructuralJacobians = [&bodyArray, &bodyIndex, &jacobianArray, &rhsAccel, &tireArray, &pairM0, &pairM1](ndNode* const node)
	{
		if (node->m_body)
		{
			bodyIndex.PushBack(bodyIndex.GetCount());
			bodyArray.PushBack(node->m_body->GetAsBodyDynamic());
			node->m_body->GetAsBodyDynamic()->UpdateInvInertiaMatrix();

			if (node->m_joint)
			{
				const ndBodyDynamic* const body0 = node->m_body->GetAsBodyDynamic();
				const ndBodyDynamic* const body1 = node->GetParent()->m_body->GetAsBodyDynamic();
				const ndJointBilateralConstraint* const joint = *node->m_joint;
				const ndVector com0(body0->GetMatrix().TransformVector(body0->GetCentreOfMass()));
				const ndVector com1(body1->GetMatrix().TransformVector(body1->GetCentreOfMass()));
				const ndVector r0(joint->CalculateGlobalMatrix0().m_posit - com0);
				const ndVector r1(joint->CalculateGlobalMatrix1().m_posit - com1);

				ndMatrix matrix(ndGetIdentityMatrix());
				matrix.m_posit = com0;

				auto FindParentId = [&bodyArray, &bodyIndex](const ndBodyDynamic* const body)
				{
					for (ndInt32 i = bodyArray.GetCount() - 1; i >= 0; --i)
					{
						if (bodyArray[i] == body)
						{
							return bodyIndex[i];
						}
					}
					ndAssert(0);
					return -1;
				};

				ndInt32 m0 = bodyIndex[bodyIndex.GetCount() - 1];
				ndInt32 m1 = FindParentId(body1);
				for (ndInt32 i = 0; i < 3; ++i)
				{
					ndJacobianPair jacobianPair;
					jacobianPair.m_jacobianM0.m_linear = matrix[i].Scale(ndFloat32(-1.0f));
					jacobianPair.m_jacobianM0.m_angular = jacobianPair.m_jacobianM0.m_linear.CrossProduct(r0);
					jacobianPair.m_jacobianM1.m_linear = matrix[i];
					jacobianPair.m_jacobianM1.m_angular = jacobianPair.m_jacobianM1.m_linear.CrossProduct(r1);
					rhsAccel.PushBack(ndFloat32(0.0f));
					jacobianArray.PushBack(jacobianPair);
					pairM0.PushBack(m0);
					pairM1.PushBack(m1);

					jacobianPair.m_jacobianM0.m_linear = ndVector::m_zero;
					jacobianPair.m_jacobianM0.m_angular = matrix[i].Scale(ndFloat32(-1.0f));
					jacobianPair.m_jacobianM1.m_linear = ndVector::m_zero;
					jacobianPair.m_jacobianM1.m_angular = matrix[i];
					rhsAccel.PushBack(ndFloat32(0.0f));
					jacobianArray.PushBack(jacobianPair);
					pairM0.PushBack(m0);
					pairM1.PushBack(m1);
				}
				if (strcmp(node->m_joint->ClassName(), ndMultiBodyVehicleTireJoint::StaticClassName()) == 0)
				{
					tireArray.PushBack((ndMultiBodyVehicleTireJoint*)joint);
				}
			}
		}
	};
	NodeIterator(GetStructuralJacobians);
	ndInt32 tireStart = rhsAccel.GetCount();

	const ndInt32 count = ndMin(tireArray.GetCount(), 4);
	const ndMatrix rotation(ndYawMatrix(90.0f * ndDegreeToRad));
	ndVector pin(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f));
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndFloat32 dist = ndFloat32 (-1.0e10f);
		for (ndInt32 j = i; j < tireArray.GetCount(); ++j)
		{
			const ndMultiBodyVehicleTireJoint* const joint = tireArray[j];
			const ndBodyDynamic* const body = joint->GetBody0()->GetAsBodyDynamic();
			const ndVector origin(body->GetMatrix().m_posit);
			ndFloat32 project = origin.DotProduct(pin).GetScalar();
			if (project > dist)
			{
				dist = project;
				ndSwap(tireArray[i], tireArray[j]);
			}
		}
		pin = rotation.RotateVector(pin);
	}
	tireArray.SetCount(count);

	const ndVector upDir(m_localFrame.m_up.Scale(ndFloat32(1.0f)));
	for (ndInt32 i = 0; i < tireArray.GetCount(); ++i)
	{
		const ndMultiBodyVehicleTireJoint* const joint = tireArray[i];
		const ndBodyDynamic* const body = joint->GetBody0()->GetAsBodyDynamic();

		auto FindParentId = [&bodyArray, &bodyIndex](const ndBodyDynamic* const body)
		{
			for (ndInt32 i = bodyArray.GetCount() - 1; i >= 0; --i)
			{
				if (bodyArray[i] == body)
				{
					return bodyIndex[i];
				}
			}
			ndAssert(0);
			return -1;
		};
		ndInt32 m0 = FindParentId(body);
		ndInt32 m1 = 0;
		ndJacobianPair jacobianPair;
		jacobianPair.m_jacobianM0.m_linear = upDir;
		jacobianPair.m_jacobianM0.m_angular = ndVector::m_zero;
		jacobianPair.m_jacobianM1.m_linear = upDir.Scale(ndFloat32(-1.0f));
		jacobianPair.m_jacobianM1.m_angular = ndVector::m_zero;

		pairM0.PushBack(m0);
		pairM1.PushBack(m1);
		rhsAccel.PushBack(-m_gravityMagnitud);
		jacobianArray.PushBack(jacobianPair);
	}

	const ndInt32 stride = rhsAccel.GetCount();

	// build Mass Matrix
	ndFixSizeArray<ndJacobian, buffersCapacity> Jt;
	ndFixSizeArray<ndJacobian, buffersCapacity> JinvMass;
	ndFixSizeArray<ndFloat32, buffersCapacity * buffersCapacity> massMatrix(stride * stride);

	ndJacobian zeroJacobian;
	zeroJacobian.m_linear = ndVector::m_zero;
	zeroJacobian.m_angular = ndVector::m_zero;
	for (ndInt32 i = 0; i < rhsAccel.GetCount(); ++i)
	{
		Jt.PushBack(zeroJacobian);
		JinvMass.PushBack(zeroJacobian);
	}

	for (ndInt32 i = 0; i < stride; ++i)
	{
		ndInt32 m0 = pairM0[i];
		ndInt32 m1 = pairM1[i];

		ndFloat32 invMass0 = bodyArray[m0]->GetInvMass();
		ndFloat32 invMass1 = bodyArray[m1]->GetInvMass();
		const ndMatrix& invInertia0 = bodyArray[m0]->GetInvInertiaMatrix();
		const ndMatrix& invInertia1 = bodyArray[m1]->GetInvInertiaMatrix();

		const ndJacobian& J01invMass(jacobianArray[i].m_jacobianM0);
		const ndJacobian& J10invMass(jacobianArray[i].m_jacobianM1);

		JinvMass[m0].m_linear = J01invMass.m_linear.Scale(invMass0);
		JinvMass[m1].m_linear = J10invMass.m_linear.Scale(invMass1);
		JinvMass[m0].m_angular = invInertia0.RotateVector(J01invMass.m_angular);
		JinvMass[m1].m_angular = invInertia1.RotateVector(J10invMass.m_angular);

		ndVector diagDot(
			JinvMass[m0].m_linear * jacobianArray[i].m_jacobianM0.m_linear +
			JinvMass[m0].m_angular * jacobianArray[i].m_jacobianM0.m_angular +
			JinvMass[m1].m_linear * jacobianArray[i].m_jacobianM1.m_linear +
			JinvMass[m1].m_angular * jacobianArray[i].m_jacobianM1.m_angular);
		ndFloat32 diagonal = diagDot.AddHorizontal().GetScalar() * ndFloat32(1.001f);
		massMatrix[i * stride + i] = diagonal;

		for (ndInt32 j = i + 1; j < stride; ++j)
		{
			ndInt32 n0 = pairM0[j];
			ndInt32 n1 = pairM1[j];
			Jt[n0] = jacobianArray[j].m_jacobianM0;
			Jt[n1] = jacobianArray[j].m_jacobianM1;

			ndVector sum(ndVector::m_zero);
			for (ndInt32 k = 0; k < stride; ++k)
			{
				sum += JinvMass[k].m_linear * Jt[k].m_linear + JinvMass[k].m_angular * Jt[k].m_angular;
			}
			ndFloat32 offDiag = sum.AddHorizontal().GetScalar();
			massMatrix[i * stride + j] = offDiag;
			massMatrix[j * stride + i] = offDiag;

			Jt[n0] = zeroJacobian;
			Jt[n1] = zeroJacobian;
		}
		JinvMass[m0] = zeroJacobian;
		JinvMass[m1] = zeroJacobian;
	}
#ifdef _DEBUG
	ndArray<ndFloat32> buffer;
	buffer.SetCount(stride * stride);
	ndAssert(ndTestPSDmatrix(stride, stride, &massMatrix[0], &buffer[0]));
#endif

	ndFixSizeArray<ndFloat32, buffersCapacity> force(stride);
	ndCholeskyFactorization(rhsAccel.GetCount(), rhsAccel.GetCount(), &massMatrix[0]);
	ndSolveCholesky(stride, stride, &massMatrix[0], &force[0], &rhsAccel[0]);

	for (ndInt32 i = 0; i < tireArray.GetCount(); ++i)
	{
		ndFloat32 sprungWeight = ndAbs(force[tireStart + i]);
		ndMultiBodyVehicleTireJoint* const tire = tireArray[i];
		tire->m_frictionModel.m_sprungWeight = sprungWeight;
	}
	SetTransform(savedMatrix);
}

void ndMultiBodyVehicle::ConvertToMotorVehicle()
{
	auto SetChassisAndMotor = [this](ndNode* const node)
	{
		if (node->m_joint && (strcmp(node->m_joint->ClassName(), ndMultiBodyVehicleMotor::StaticClassName()) == 0))
		{
			AddChassis(node->GetParent()->m_body);
			AddMotor(node->m_body, node->m_joint);
		}
	};
	NodeIterator(SetChassisAndMotor);

	auto AddStructureParts = [this](ndNode* const node)
	{
		if (node->m_joint)
		{
			if (strcmp(node->m_joint->ClassName(), ndMultiBodyVehicleTireJoint::StaticClassName()) == 0)
			{
				AddTire(node->m_body, node->m_joint);
			}
			else if (strcmp(node->m_joint->ClassName(), ndMultiBodyVehicleDifferential::StaticClassName()) == 0)
			{
				AddDifferential(node->m_body, node->m_joint);
			}
		}
	};
	NodeIterator(AddStructureParts);

	auto AddDriveTrain = [this](ndNode* const node)
	{
		if (node->m_joint)
		{
			if (strcmp(node->m_joint->ClassName(), ndMultiBodyVehicleGearBox::StaticClassName()) == 0)
			{
				AddGearBox(node->m_joint);
			}
			else if (strcmp(node->m_joint->ClassName(), ndMultiBodyVehicleDifferentialAxle::StaticClassName()) == 0)
			{
				AddDifferentialAxle(node->m_joint);
			}
			else if (strcmp(node->m_joint->ClassName(), ndMultiBodyVehicleTorsionBar::StaticClassName()) == 0)
			{
				AddTorsionBar(node->m_joint);
			}
		}
	};
	NodeIterator(AddDriveTrain);

	m_debugFlags = m_wheel;
	//m_debugFlags = m_torsionBar;
}

void ndMultiBodyVehicle::Update(ndFloat32 timestep, ndInt32)
{
	if (!m_initialized)
	{
		m_initialized = true;

		CalculateRestSprungWeight();

		// reset forces of assesories attached to chassis
		if (m_motor)
		{
			ndBodyDynamic* const selfBody = m_motor->GetBody0()->GetAsBodyDynamic();
			ndSharedPtr<ndBodyNotify> notify(new ndMotorNotify(this));
			selfBody->SetNotifyCallback(notify);
		}
		for (ndList<ndMultiBodyVehicleDifferential*>::ndNode* node = m_differentialList.GetFirst(); node; node = node->GetNext())
		{
			ndBodyDynamic* const selfBody = node->GetInfo()->GetBody0()->GetAsBodyDynamic();
			ndSharedPtr<ndBodyNotify> notify(new ndComponentNotify(this));
			selfBody->SetNotifyCallback(notify);
		}
	}

	//const ndFloat32 sleepValue = ndFloat32(0.02f);
	//bool sleeping = SetSleep(sleepValue, sleepValue, sleepValue, sleepValue);
	//if (IsSleeping())
	//{
	//	m_sleepCounter++;
	//	if (m_sleepCounter >= 8)
	//	{
	//		return;
	//	}
	//}
	//else
	//{
	//	m_sleepCounter = 0;
	//}

	// apply down force
	ApplyAerodynamics(timestep);

	// apply tire model
	ApplyTireModel();
}

void ndMultiBodyVehicle::PostUpdate(ndFloat32, ndInt32)
{
	ApplyAlignmentAndBalancing();
}
