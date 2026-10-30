// NewtonTractor.cpp: implementation of the RaceCar class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
//#include "CustomSlider.h"
#include "NewtonTractor.h"
//#include "NewtonCustomJoint.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


// vehicle hierachy:
//
//	MainBody
//		front_axel
//			fl_tire
//			fr_tire
//		rear_axel
//			rl_tire
//			rr_tire
//
//		arms
//			armHydraulic 
//			frontBucket
//				frontBucketHydraulicPiston
//			frontBucketHydraulic
//		armHydraulicPiston
//
//		boom
//			boomHydraulicPiston
//			boomArm
//				bucket
//				   buckectPivot
//				strut
//					strut2
//					buckectHydraulicPiston
//				buckectHydraulic
//				boomArmHydraulicPiston
//			boomArmHydraulic
//		boomHydraulic


extern dInt32 woodID; 
extern bool hideVehicle;

#define MAX_TORQUE (5.0f)
#define VEHICLE_MASS (400.0f)

#define BUFFER_SIZE	20000

// set frequency to 2 hertz
//#define SUSPENSION_FREQUENCE (2.0f)
//#define SUSPENSION_LENGTH	(0.15f)
//#define SUSPENSION_LENGTH	(0.3f)
#define MAX_STEER_ANGLE (30.0f * 3.1416f / 180.0f)


#if 0
class TractorControlJoint: public NewtonCustomJoint  
{
	public:

	TractorControlJoint (dSceneNode *me, dInt32 maxDOF, NewtonBody* child, NewtonBody* parent)
		:NewtonCustomJoint  (maxDOF, child, parent)
	{
		m_me = me;

		// store the joint as user data 
		NewtonBodySetUserData (child, this);

		// set the transform call back function
		NewtonBodySetTransformCallback (child, SetTransform);
	}

	static void SetTransform (const NewtonBody* body, const dFloat* matrixPtr)
	{

		TractorControlJoint* me;

		// get the graphic object form the rigid body
		me = (TractorControlJoint*) NewtonBodyGetUserData (body);

		// save the matrix for latte used
		me->m_bodyGlobalMatrix = *((dMatrix*)matrixPtr);
	}

	virtual void SetLocalMatrix()
	{
		dMatrix matrix (m_bodyGlobalMatrix * m_me->GetParent()->CalcGlobalMatrix().Inverse());
		m_me->SetMatrix(matrix);
	}

	dSceneNode* m_me;
	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;
	dMatrix m_bodyGlobalMatrix;
};


class TractorTire: public TractorControlJoint
{
	public:
	TractorTire (dSceneNode *me, const dMatrix& matrix, NewtonBody* tire, NewtonBody* axel)
		:TractorControlJoint(me, 6, tire, axel)
	{
		CalculateLocalMatrix (matrix.m_posit, matrix.m_front, m_localMatrix0, m_localMatrix1);
	}

	virtual void SetSteerAngle (dFloat angle) 
	{
	}

	virtual void SetTorque (dFloat torque) 
	{
	}
};


class RearTractorTire: public TractorTire
{
	public:
	RearTractorTire (dSceneNode *me, const dMatrix& matrix, NewtonBody* tire, NewtonBody* axel)
		:TractorTire(me, matrix, tire, axel)
	{
		m_torque = 0.0f;
	}

	void SubmitConstrainst ()
	{
		dMatrix matrix0;
		dMatrix matrix1;
		#define PIN_LENTH 40.0f

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

		// Restrict the movement on the pivot point along all tree orthonormal direction
		NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_front[0]);
		NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_up[0]);
		NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_right[0]);
		
		// get a point along the pin axis at some reasonable large distance from the pivot
		dVector q0 (matrix0.m_posit + matrix0.m_front.Scale (PIN_LENTH));
		dVector q1 (matrix1.m_posit + matrix1.m_front.Scale (PIN_LENTH));

		// two constraints row perpendicular to the pin vector
 		NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix0.m_up[0]);
		NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix0.m_right[0]);

		// add tire torque
		NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix1.m_front[0]);
		NewtonUserJointSetRowAcceleration (m_joint, m_torque);
	}

	void SetTorque (dFloat torque) 
	{
		m_torque = -torque;
	}

	dFloat m_torque;
};


class FrontTractorTire: public TractorTire
{
	public:
	FrontTractorTire (dSceneNode *me, const dMatrix& matrix, NewtonBody* tire, NewtonBody* axel)
		:TractorTire(me, matrix, tire, axel)
	{
		m_steerAngle = 0.0f;

	}

	virtual void SetSteerAngle (dFloat angle) 
	{
		m_steerAngle = -angle;
	}

	void SubmitConstrainst ()
	{
		dFloat angle;
		dFloat relAngle;
		dFloat sinAngle;
		dFloat cosAngle;
		dMatrix matrix0;
		dMatrix matrix1;
		#define PIN_LENTH 40.0f

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

		// get the pin fixed to the first body
		const dVector& dir0 = matrix0.m_front;
		// get the pin fixed to the second body
		const dVector& dir1 = matrix1.m_up;

		// construct an orthogonal coordinate system with these two vectors
		dVector dir2 (dir0 * dir1);
		dVector dir3 (dir2 * dir0);
		dir3 = dir3.Scale (1.0f / dSqrt (dir3 % dir3));

		const dVector& p0 = matrix0.m_posit;
		const dVector& p1 = matrix1.m_posit;

		dVector q0 (p0 + dir3.Scale(PIN_LENTH));
		dVector q1 (p1 + dir1.Scale(PIN_LENTH));

		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &dir0[0]);
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &dir1[0]);
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &dir2[0]);
		NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &dir0[0]);

		// set the steer angle
		sinAngle = (matrix0.m_front * matrix1.m_front) % matrix1.m_up;
		cosAngle = matrix0.m_front % matrix1.m_front;
		angle = dAtan2 (sinAngle, cosAngle);
		relAngle = angle - m_steerAngle;
		NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix1.m_up[0]);
	}

	dFloat m_steerAngle;
};


class RealisticHeavyAxel: public TractorControlJoint
{
	public:
	RealisticHeavyAxel(dSceneNode *me, const dMatrix& matrix, NewtonBody* axel, NewtonBody* engine)
		:TractorControlJoint(me, 6, axel, engine)
	{
		dMatrix matrix0;
		dMatrix matrix1;

		// Get the global matrices of each rigid body.
		NewtonBodyGetMatrix(m_body0, &matrix0[0][0]);
		NewtonBodyGetMatrix(m_body1, &matrix1[0][0]);

		m_localMatrix0 = matrix * matrix0.Inverse();
		m_localMatrix1 = matrix * matrix1.Inverse();
	}

	NewtonBody* CreateTireBody (
		NewtonWorld* nWorld, 
		dSceneNode* tireNode,
		dVector* vertex) const
	{
		dInt32 i;
		dInt32 vertexCount; 
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		dFloat mass;
		dFloat width;
		dFloat radius;
		dVector origin;
		dVector inertia;
		NewtonBody* body;
		NewtonCollision* collision;
	

		// create the collision geometry iterate again collecting the vertex array		
		width = 0.0f;
		radius = 0.0f;
		vertexCount = tireNode->GetGeometry()->GetFlatVertexArray (vertex, BUFFER_SIZE);
		for (i = 0; i < vertexCount; i ++) {
			width = max (vertex[i].m_x, width);
			radius = max (vertex[i].m_y, radius);
		}
		collision = NewtonCreateChamferCylinder(nWorld, radius, width * 2.2f, NULL);


		// Get the node local Matrix;
		dMatrix matrix (tireNode->CalcGlobalMatrix ());

		//create the rigid body
		body = NewtonCreateBody (nWorld, collision);

		// set the material group id for vehicle
		NewtonBodySetMaterialGroupID (body, woodID);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (body, NewtonTractor::ApplyGravityForce);

		// disable collision with linked elements
		NewtonBodySetJointRecursiveCollision (body, 0);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = VEHICLE_MASS * 0.25f;
		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];

		// set the mass matrix
		NewtonBodySetMassMatrix (body, mass, Ixx, Iyy, Izz);

		// set the mass matrix
		NewtonBodySetMassMatrix (body, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (body, &matrix[0][0]);

		// release the collision
		NewtonReleaseCollision (nWorld, collision);	

		return body;
	}


	TractorTire* CreateFrontTire (
		NewtonWorld* nWorld, 
		const char* name,
		dVector* vertex)
	{
		NewtonBody* body;
		dSceneNode* tireNode;
		TractorTire* tireJoint;

		// find the geometry mode;
		tireNode = m_me->Find (name);
		_ASSERTE (tireNode);

		body = CreateTireBody (nWorld, tireNode, vertex);
		
		dMatrix matrix (tireNode->CalcGlobalMatrix ());
		tireJoint = new FrontTractorTire (tireNode, matrix, body, m_body0);

		return tireJoint;
	}

	TractorTire* CreateRearTire (
		NewtonWorld* nWorld, 
		const char* name,
		dVector* vertex)
	{
		NewtonBody* body;
		dSceneNode* tireNode;
		TractorTire* tireJoint;


		// find the geometry mode;
		tireNode = m_me->Find (name);
		_ASSERTE (tireNode);

		body = CreateTireBody (nWorld, tireNode, vertex);
		
		dMatrix matrix (tireNode->CalcGlobalMatrix ());
		tireJoint = new RearTractorTire (tireNode, matrix, body, m_body0);
		return tireJoint;
	}


	virtual void SetLocalMatrix()
	{
		TractorControlJoint::SetLocalMatrix();
		m_leftTire->SetLocalMatrix();
		m_rightTire->SetLocalMatrix();
	}

	void SubmitConstrainst ()
	{
		
		dMatrix matrix0;
		dMatrix matrix1;
		dFloat relAngle;
		#define JOINT_LENGTH 20.0f

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

		// Restrict the movement on the pivot point along all tree orthonormal direction
		dVector p1 (matrix1.m_posit);
		dVector p0 (matrix0.m_posit + matrix0.m_front.Scale ((p1 - matrix0.m_posit) % matrix0.m_front));

		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_up[0]);
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_right[0]);
		
		// get a point along the ping axis at some reasonable large distance from the pivot
		dVector q0 (p0 + matrix0.m_front.Scale(JOINT_LENGTH));
		dVector q1 (p1 + matrix1.m_front.Scale(JOINT_LENGTH));

		// two constraints row perpendicular to the pin
 		NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix1.m_up[0]);

		// get a point along the ping axis at some reasonable large distance from the pivot
		dVector r0 (p0 + matrix0.m_up.Scale(JOINT_LENGTH));
		dVector r1 (p1 + matrix1.m_up.Scale(JOINT_LENGTH));

		// one constraint row perpendicular to the pin					 
 		NewtonUserJointAddLinearRow (m_joint, &r0[0], &r1[0], &matrix1.m_right[0]);

		// set the linear suspension limit
		p0 = matrix0.m_posit;
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_front[0]);
		
		// set a critical linear spring and damper constant
		dFloat linearSpringK = 80.0f;
		dFloat linearSpringD = 0.2f * 2.0f * dSqrt (linearSpringK);
		NewtonUserJointSetRowStiffness (m_joint, 0.4f);
		NewtonUserJointSetRowSpringDamperAcceleration (m_joint, linearSpringK, linearSpringD);


		// set the linear suspension limit
		// the relative angle between the axel and the engine is given by the 
		// dSin of the angle between the right vectors
		relAngle = dAsin ((matrix0.m_right * matrix1.m_right) % matrix1.m_up);
		NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix1.m_up[0]);

		// set a critical angular spring and damper constant
		dFloat angularSpringK = 50.0f;
		dFloat angularSpringD = 0.2f * 2.0f * dSqrt (angularSpringK);
		NewtonUserJointSetRowStiffness (m_joint, 0.1f);
		NewtonUserJointSetRowSpringDamperAcceleration (m_joint, angularSpringK, angularSpringD);
	}

	void SetSteer (dFloat value)
	{
		m_leftTire->SetSteerAngle(value);
		m_rightTire->SetSteerAngle(value);
	}

	void SetTorque (dFloat value)
	{
		m_leftTire->SetTorque(value);
		m_rightTire->SetTorque(value);
	}

	TractorTire* m_leftTire;
	TractorTire* m_rightTire;
};


class AngularManipulator: public TractorControlJoint
{
	public:
	AngularManipulator (
		dSceneNode *me, 
		const dMatrix& matrix, 
		NewtonBody* actuator, 
		NewtonBody* base,
		dFloat maxAngle)
		:TractorControlJoint(me, 6, actuator, base)
	{
		m_angle = 0.0f;
		m_maxAngle = maxAngle;
		CalculateLocalMatrix (matrix.m_posit, matrix.m_front, m_localMatrix0, m_localMatrix1);
	}

	void SetAngle (dFloat angle)
	{
		m_angle += angle * 0.015f;
		m_angle = (m_angle > m_maxAngle) ? m_maxAngle : (m_angle < 0.0f) ? 0.0f : m_angle; 
	}

	dFloat m_angle;
	dFloat m_maxAngle;
};


class SimpleIKHydrolicManipulator: public AngularManipulator
{
	public:
	SimpleIKHydrolicManipulator (
		dSceneNode *me, 
		const dMatrix& matrix, 
		NewtonBody* actuator, 
		NewtonBody* base,
		dFloat maxAngle)
		:AngularManipulator(me, matrix, actuator, base, maxAngle)
	{
	}

	void SubmitConstrainst ()
	{
		dFloat angle;
		dFloat relAngle;
		dFloat sinAngle;
		dFloat cosAngle;

		dMatrix matrix0;
		dMatrix matrix1;
		#define PIN_LENTH 40.0f

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

		// Restrict the movement on the pivot point along all tree orthonormal direction
		NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_front[0]);
		NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_up[0]);
		NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_right[0]);
		
		// get a point along the pin axis at some reasonable large distance from the pivot
		dVector q0 (matrix0.m_posit + matrix0.m_front.Scale (PIN_LENTH));
		dVector q1 (matrix1.m_posit + matrix1.m_front.Scale (PIN_LENTH));

		// two constraints row perpendicular to the pin vector
 		NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix0.m_up[0]);
		NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix0.m_right[0]);

		// set the Arm angle
		sinAngle = (matrix0.m_right * matrix1.m_right) % matrix1.m_front;
		cosAngle = matrix0.m_right % matrix1.m_right;
		angle = dAtan2 (sinAngle, cosAngle);

		relAngle = angle - m_angle;
		NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix1.m_front[0]);
	}


	void SetLocalMatrix()
	{

		// calculate the arm matrix first
		AngularManipulator::SetLocalMatrix();

		dFloat angle; 
		dVector p0 (m_IKNode0_LocalMatrix.UntransformVector (m_IKNode1->GetParent()->GetMatrix().TransformVector(m_IKNode1->GetMatrix().m_posit)));
		angle = dAtan2 (p0.m_y, -p0.m_z);
		dMatrix matrix0 (dgPitchMatrix(angle));
		m_IKNode0->SetMatrix(matrix0 * m_IKNode0_LocalMatrix);


		dVector p1 (m_IKNode1_LocalMatrix.UntransformVector (m_IKNode1->GetParent()->GetMatrix().UntransformVector (m_IKNode0->GetMatrix().m_posit)));
		angle = -dAtan2 (p1.m_y, p1.m_z);
		dMatrix matrix1 (dgPitchMatrix(angle));
		m_IKNode1->SetMatrix(matrix1 * m_IKNode1_LocalMatrix);
	}
	
	dSceneNode* m_IKNode0;
	dSceneNode* m_IKNode1;
 	dMatrix m_IKNode0_LocalMatrix;
	dMatrix m_IKNode1_LocalMatrix;
};


class BoomIKHydrolicManipulator: public SimpleIKHydrolicManipulator
{
	public:
	BoomIKHydrolicManipulator (
		dSceneNode *me, 
		const dMatrix& matrix, 
		NewtonBody* actuator, 
		NewtonBody* base,
		dFloat maxAngle,
		dFloat maxAngle2)
		:SimpleIKHydrolicManipulator(me, matrix, actuator, base, maxAngle)
	{
		m_yawAngle = 0.0f;
		m_maxYawAngle = maxAngle2 * 0.5f;

		m_pitchAngle = 0.0f;
		m_relYawAngle = 0.0f;
	}

	void SetAngle2 (dFloat angle)
	{
		m_yawAngle += angle * 0.015f;
		m_yawAngle = (m_yawAngle > m_maxYawAngle) ? m_maxYawAngle : (m_yawAngle < -m_maxYawAngle) ? -m_maxYawAngle : m_yawAngle; 
	}


	void SetLocalMatrix()
	{

		// calculate the arm matrix first
		AngularManipulator::SetLocalMatrix();


//		dFloat angle0; 
//		dFloat angle1; 
//		dVector p0 (m_IKNode0_LocalMatrix.UntransformVector (m_IKNode1->GetParent()->GetMatrix().TransformVector(m_IKNode1->GetMatrix().m_posit)));
//		angle0 = dAtan2 (p0.m_y, -p0.m_z);
//angle0 = 0;
//		angle1 = -dAtan2 (p0.m_x, -p0.m_z);
//		dMatrix matrix0 (dgPitchMatrix(angle0));
//		m_IKNode0->SetMatrix(matrix0 * m_IKNode0_LocalMatrix * dgYawMatrix(angle1));
//		dMatrix matrix0 (m_IKNode0_LocalMatrix * dgRollMatrix(m_pitchAngle) * dgYawMatrix(m_relYawAngle));
//		matrix0.m_posit = m_IKNode0_LocalMatrix.m_posit;
//		m_IKNode0->SetMatrix(matrix0);


//		dVector p1 (m_IKNode1_LocalMatrix.UntransformVector (m_IKNode1->GetParent()->GetMatrix().UntransformVector (m_IKNode0->GetMatrix().m_posit)));
//		angle0 = -dAtan2 (p1.m_y, p1.m_z);
//		dMatrix matrix1 (dgPitchMatrix(angle0));
//		m_IKNode1->SetMatrix(matrix1 * m_IKNode1_LocalMatrix);


		dVector p0 (m_IKNode0->GetMatrix().m_posit);
		dVector p1 (m_me->GetMatrix().TransformVector(m_IKNode1->GetMatrix().m_posit));

		dVector p10 (p1 - p0);
		dVector dir (p10.Scale (1.0f / dSqrt (p10 % p10)));

		dFloat tetha = dAsin (dir.m_y);
		dFloat alpha = dAtan2 (-dir.m_z, dir.m_x);
		dMatrix matrix0 = dgRollMatrix(tetha) * dgYawMatrix(alpha);
		matrix0.m_posit = m_IKNode0->GetMatrix().m_posit;
		m_IKNode0->SetMatrix(matrix0);

		dVector q0 (m_IKNode1->GetMatrix().m_posit);
		dVector q1 (m_me->GetMatrix().UntransformVector(p0));
		dVector q10 (q1 - q0);

		dFloat angle = dAtan2 (q10.m_y, q10.m_z);		
		dFloat sinAngle = dSin (angle);		
		dFloat cosAngle = dCos (angle);		
		dMatrix matrix1 (GetZeroMatrix ());

		matrix1[0][1] = sinAngle;
		matrix1[0][2] = cosAngle;
		matrix1[1][1] = cosAngle;
		matrix1[1][2] = -sinAngle;
		matrix1[2][0] = -1.0f;
		matrix1.m_posit = m_IKNode1->GetMatrix().m_posit;
		m_IKNode1->SetMatrix(matrix1);
	}

	void SubmitConstrainst ()
	{
		dFloat angle;
		dFloat relAngle;
		dFloat sinAngle;
		dFloat cosAngle;
		dMatrix matrix0;
		dMatrix matrix1;
		#define PIN_LENTH 40.0f

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

		// get the pin fixed to the first body
		const dVector& dir0 = matrix0.m_front;
		// get the pin fixed to the second body
		const dVector& dir1 = matrix1.m_up;

		// construct an orthogonal coordinate system with these two vectors
		dVector dir2 (dir0 * dir1);
		dVector dir3 (dir2 * dir0);
		dir3 = dir3.Scale (1.0f / dSqrt (dir3 % dir3));

		const dVector& p0 = matrix0.m_posit;
		const dVector& p1 = matrix1.m_posit;

		dVector q0 (p0 + dir3.Scale(PIN_LENTH));
		dVector q1 (p1 + dir1.Scale(PIN_LENTH));

		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &dir0[0]);
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &dir1[0]);
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &dir2[0]);
		NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &dir0[0]);

		// set the steer angle
		sinAngle = (matrix0.m_front * matrix1.m_front) % matrix1.m_up;
		cosAngle = matrix0.m_front % matrix1.m_front;
		angle = dAtan2 (sinAngle, cosAngle);
		relAngle = angle - m_yawAngle;
		m_relYawAngle = angle;
		NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix1.m_up[0]);


		// set the Arm angle
		sinAngle = (matrix0.m_right * matrix1.m_right) % matrix1.m_front;
		cosAngle = matrix0.m_right % matrix1.m_right;
		angle = dAtan2 (sinAngle, cosAngle);
		m_pitchAngle = angle;

		relAngle = angle - m_angle;
		NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix1.m_front[0]);
	}


	dFloat m_yawAngle;
	dFloat m_maxYawAngle;

	dFloat m_pitchAngle;
	dFloat m_relYawAngle;

};


class BackBuckectIKHydrolicManipulator: public SimpleIKHydrolicManipulator
{
	public:
	BackBuckectIKHydrolicManipulator (
		dSceneNode *me, 
		const dMatrix& matrix, 
		NewtonBody* actuator, 
		NewtonBody* base,
		dFloat maxAngle)
		:SimpleIKHydrolicManipulator(me, matrix, actuator, base, maxAngle)
	{
	}

	void SetStrutIK (dSceneNode* strut, dSceneNode* strut2, dSceneNode* strutPivot)
	{
		m_IKstuct = strut;
		m_IKstuct2 = strut2;
		m_IKstuctTarget = strutPivot;

		dVector p0 (m_IKstuct->GetMatrix().m_posit);
		dVector p1 (m_IKstuct->GetMatrix().TransformVector(m_IKstuct2->GetMatrix().m_posit));
		dVector p2 (m_IKstuctTarget->GetParent()->GetMatrix().TransformVector(m_IKstuctTarget->GetMatrix().m_posit));

		dVector p10 (p1 - p0);
		dVector p21 (p2 - p1);

		m_l0_2 = p10 % p10;
		m_l1_2 = p21 % p21;

		m_l0 = dSqrt (m_l0_2);
		m_l1 = dSqrt (m_l1_2);
	}

	void SetLocalMatrix()
	{

		dFloat a;
		dFloat b;
		dFloat c;
		dFloat a2;
		dFloat b2;
		dFloat c2;

		dFloat angle0;
		dFloat angle1;
		dFloat angle2;

		
		// calculate the arm matrix first
		AngularManipulator::SetLocalMatrix();


		dVector p0 (m_IKstuct->GetMatrix().m_posit);
		dVector p2 (m_IKstuctTarget->GetParent()->GetMatrix().TransformVector(m_IKstuctTarget->GetMatrix().m_posit));
		dVector p20 (p2 - p0);

		a = p20.m_y;
		b = p20.m_z;
		a2 = a * a;
		b2 = b * b;
		c2 = a2 + b2;
		c = dSqrt (c2);

		angle2 = dAcos (0.5f * (c2 - m_l0_2 - m_l1_2) / (m_l0 * m_l1));
		dMatrix strut2_Matrix (dgPitchMatrix(angle2));
		strut2_Matrix.m_posit = m_IKstuct2->GetMatrix().m_posit;
		m_IKstuct2->SetMatrix(strut2_Matrix);
  
		
		angle0 = dAtan2 (b, a);
		angle1 = dAcos (0.5f * (c2 + m_l0_2 - m_l1_2) / (m_l0 * c));
		angle0 = angle0 - angle1;

		dMatrix strut_Matrix (dgPitchMatrix(angle0));
		strut_Matrix.m_posit = m_IKstuct->GetMatrix().m_posit;
		m_IKstuct->SetMatrix(strut_Matrix);

		


		dFloat angle; 
		p0 = m_IKNode0_LocalMatrix.UntransformVector (m_IKNode1->GetParent()->GetMatrix().TransformVector(m_IKNode1->GetMatrix().m_posit));
		angle = dAtan2 (p0.m_y, -p0.m_z);
		dMatrix matrix0 (dgPitchMatrix(angle));
		m_IKNode0->SetMatrix(matrix0 * m_IKNode0_LocalMatrix);

		dVector p1 (m_IKNode1_LocalMatrix.UntransformVector (m_IKNode1->GetParent()->GetMatrix().UntransformVector (m_IKNode0->GetMatrix().m_posit)));
		angle = - dFloat(dAtan2 (p1.m_y, p1.m_z));
		dMatrix matrix1 (dgPitchMatrix(angle));
		m_IKNode1->SetMatrix(matrix1 * m_IKNode1_LocalMatrix);
	}

	dFloat m_l0;
	dFloat m_l1;
	dFloat m_l0_2;
	dFloat m_l1_2;

	dSceneNode* m_IKstuct;
	dSceneNode* m_IKstuct2;
	dSceneNode* m_IKstuctTarget;

};


NewtonTractor::NewtonTractor(
	dSceneNode* parent, 
	const char* fileName, 
	NewtonWorld* nWorld, 
	const dMatrix& matrix)
	:NewtonVehicleNode (parent, matrix)
{
	dInt32 vertexCount; 
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	dVector origin;
	dVector inertia;
	dSceneNode* bodyPart;	
	NewtonCollision* vehicleCollision;
	dLoaderContext context;

	// allocate a memory chunk of the stack
	dVector* vertex = new dVector[BUFFER_SIZE]; 	

	// open up and load the model geometry
	char fullPathName[256];
	GetWorkingFileName (fileName, fullPathName);
	LoadFromFile (fullPathName, context);

	SetMatrix (matrix);


	// create the main vehicle rigid body

	// find the main body part
	bodyPart = (dSceneNode*) Find ("MainBody");
	_ASSERTE (bodyPart);


	// iterate again collecting the vertex array
	vertexCount = bodyPart->GetGeometry()->GetFlatVertexArray (vertex, BUFFER_SIZE);

	// create the collision geometry
	dMatrix localMatrix (bodyPart->CalcGlobalMatrix (this));
	vehicleCollision = NewtonCreateConvexHull (nWorld, vertexCount, &vertex[0].m_x, sizeof (dVector), &localMatrix[0][0]);

	//create the rigid body
	m_vehicleBody = NewtonCreateBody (nWorld, vehicleCollision);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (m_vehicleBody, this);

	// set the material group id for vehicle
	NewtonBodySetMaterialGroupID (m_vehicleBody, woodID);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (m_vehicleBody, DestroyVehicle);

	// set the transform call back function
	NewtonBodySetTransformCallback (m_vehicleBody, SetTransform);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (m_vehicleBody, ApplyGravityForce);

	// disable collision with linked elements
	NewtonBodySetJointRecursiveCollision (m_vehicleBody, 0);
	

	// calculate the moment of inertia and the relative center of mass of the solid
	NewtonConvexCollisionCalculateInertialMatrix (vehicleCollision, &inertia[0], &origin[0]);	

	mass = VEHICLE_MASS;
	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];

	// set the mass matrix
	NewtonBodySetMassMatrix (m_vehicleBody, mass, Ixx, Iyy, Izz);

	// Set the vehicle Center of mass
	// the rear spoilers race the center of mass by a lot for a race car
	// we need to lower some more for the geometrical value of the y axis
	origin.m_y *= 0.6f;
	NewtonBodySetCentreOfMass (m_vehicleBody, &origin[0]);

	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (m_vehicleBody, &matrix[0][0]);

	// release the collision 
	NewtonReleaseCollision (nWorld, vehicleCollision);	

  	// ////////////////////////////////////////////////////////////////////////////////////////////////
	//
	//  second add all vehicle components
	//
	// ////////////////////////////////////////////////////////////////////////////////////////////////
 	AddRearAxel (nWorld, vertex);
	AddFrontAxel (nWorld, vertex);
	AddBackBucket (nWorld, vertex);
	AddFrontBucket (nWorld, vertex);

	delete[] vertex;
}

NewtonTractor::~NewtonTractor()
{
}



void NewtonTractor::DestroyVehicle (const NewtonBody* body)
{
	NewtonTractor* vehicle;

	// get the graphic object form the rigid body
	vehicle = (NewtonTractor*) NewtonBodyGetUserData (body);

	// destroy the graphic object
	delete vehicle;
}


void NewtonTractor::SetTransform (const NewtonBody* body, const dFloat* matrixPtr)
{
	NewtonTractor* vehicle;

	// get the graphic object form the rigid body
	vehicle = (NewtonTractor*) NewtonBodyGetUserData (body);

	// set the transformation matrix for this rigid body
	dMatrix& matrix = *((dMatrix*)matrixPtr);
	vehicle->SetMatrix (matrix);
}


void NewtonTractor::ApplyGravityForce (const NewtonBody* body)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);

	dVector force (0.0f, mass * GRAVITY, 0.0f);
	NewtonBodySetForce (body, &force.m_x);
}


void NewtonTractor::Render() const
{
	_ASSERT(0);
	//m_rearAxel->SetLocalMatrix();
	//m_frontAxel->SetLocalMatrix();
	//m_frontBucketArm->SetLocalMatrix();
	//m_frontBucket->SetLocalMatrix();
	//m_backBucketArm1->SetLocalMatrix();
	//m_backBucketArm2->SetLocalMatrix();
	//m_backBucket->SetLocalMatrix();
	//
	//if (!hideVehicle) {
	//	NewtonVehicleNode::Render();  
	//}
}


void NewtonTractor::SetSteering(dFloat value)
{
	_ASSERT(0);
	//dFloat speed;
	//dFloat scale;
	//speed = dAbs (GetSpeed());
	//scale = (1.0f - speed / 60.0f) > 0.1f ? (1.0f - speed / 60.0f) : 0.1f;
	//if (value > 0.0f) {
	//	value = MAX_STEER_ANGLE * scale;
	//} else if (value < 0.0f) {
	//	value = -MAX_STEER_ANGLE * scale;
	//} else {
	//	value = 0.0f;
	//}
	//
	//((RealisticHeavyAxel*)m_frontAxel)->SetSteer (value);

}


void NewtonTractor::SetTireTorque(dFloat value)
{
	_ASSERT(0);
	//dFloat speed;
	//
	//speed = dAbs (GetSpeed());
	//if (value > 0.0f) {
	//	value = MAX_TORQUE * (1.0f - 0.3f * speed);
	//} else if (value < 0.0f) {
	//	value = -MAX_TORQUE * 0.5f * (1.0f - 0.3f * speed);
	//} else {
	//	value = 0.0f;
	//}
	//
	//((RealisticHeavyAxel*)m_rearAxel)->SetTorque (value);

}

void NewtonTractor::SetControl(dInt32 index, dFloat value)
{

	switch (index)
	{
		case 0:
		{
			((AngularManipulator*)m_frontBucketArm)->SetAngle (value);
			break;
		}

		case 1:
		{
			((AngularManipulator*)m_frontBucket)->SetAngle (value);
			break;
		}

		case 2:
		{
			((AngularManipulator*)m_backBucketArm1)->SetAngle (value);
			break;
		}

		case 3:
		{
			((BoomIKHydrolicManipulator*)m_backBucketArm1)->SetAngle2 (value);
			break;
		}
  

		case 4:
		{
			((AngularManipulator*)m_backBucketArm2)->SetAngle (value);
			break;
		}

		case 5:
		{
			((AngularManipulator*)m_backBucket)->SetAngle (value);
			break;
		}

	}
}


void NewtonTractor::SetApplyHandBrakes (dFloat value)
{
//	_ASSERTE (0);
//	m_rearLeftTire.SetBrakes (value);
//	m_rearRighTire.SetBrakes (value);
}


TractorControlJoint* NewtonTractor::CreateAxel (NewtonWorld* nWorld, const char *name)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonBody* body;
	dSceneNode* axelNode;
	NewtonCollision* collision;
	TractorControlJoint* joint;

	// find the geometry mode;
	axelNode = Find (name);
	_ASSERTE (axelNode);

	// Get the node local Matrix;
	dMatrix matrix (axelNode->CalcGlobalMatrix ());

	// Create the rigid body
	collision = NewtonCreateNull(nWorld);
 
	//create the rigid body
	body = NewtonCreateBody (nWorld, collision);


	// set the material group id for vehicle
	NewtonBodySetMaterialGroupID (body, woodID);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (body, ApplyGravityForce);


	mass = VEHICLE_MASS * 0.25f;
	// used a spherical inertial of 1 x 1 x 1
	Ixx = mass * 0.4f * 1.0f * 1.0f;
	Iyy = mass * 0.4f * 1.0f * 1.0f;
	Izz = mass * 0.4f * 1.0f * 1.0f;

	// set the mass matrix
	NewtonBodySetMassMatrix (body, mass, Ixx, Iyy, Izz);

	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (body, &matrix[0][0]);

	// release the collision
	NewtonReleaseCollision (nWorld, collision);	

	// disable collision with linked elements
	NewtonBodySetJointRecursiveCollision (body, 0);


	// create the axel joint
	joint = new RealisticHeavyAxel(axelNode, matrix, body, m_vehicleBody);


	return joint;
}



void NewtonTractor::AddRearAxel (NewtonWorld* nWorld, dVector* vertex)
{
	RealisticHeavyAxel* axel;
	
	axel = (RealisticHeavyAxel*) CreateAxel (nWorld, "rear_axel");
	axel->m_leftTire = axel->CreateRearTire (nWorld, "rl_tire", vertex);
	axel->m_rightTire = axel->CreateRearTire (nWorld, "rr_tire", vertex);
	m_rearAxel = axel;
}

void NewtonTractor::AddFrontAxel (NewtonWorld* nWorld, dVector* vertex)
{
	RealisticHeavyAxel* axel;
	
	axel = (RealisticHeavyAxel*) CreateAxel (nWorld, "front_axel");
	axel->m_leftTire = axel->CreateFrontTire (nWorld, "fl_tire", vertex);
	axel->m_rightTire = axel->CreateFrontTire (nWorld, "fr_tire", vertex);

	m_frontAxel = axel;
}


void NewtonTractor::AddFrontBucket (NewtonWorld* nWorld, dVector* vertex)
{
	dInt32 vertexCount;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	dVector origin;
	dVector inertia;
	NewtonBody* body0;
	NewtonBody* body1;
	dSceneNode* armNode;
	NewtonCollision* collision;


	// find the geometry mode;
	armNode = Find ("arms");
	_ASSERTE (armNode);

	// Get the node local Matrix;
	dMatrix matrix (armNode->CalcGlobalMatrix ());


	// iterate again collecting the vertex array
	vertexCount = armNode->GetGeometry()->GetFlatVertexArray (vertex, BUFFER_SIZE);

	// create the collision geometry
	collision = NewtonCreateConvexHull (nWorld, vertexCount, &vertex[0].m_x, sizeof (dVector), NULL);

	//create the rigid body
	body0 = NewtonCreateBody (nWorld, collision);

	// set the material group id for vehicle
	NewtonBodySetMaterialGroupID (body0, woodID);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (body0, ApplyGravityForce);

	// calculate the moment of inertia and the relative center of mass of the solid
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

	mass = VEHICLE_MASS * 0.20f;
	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];

	// set the mass matrix
	NewtonBodySetMassMatrix (body0, mass, Ixx, Iyy, Izz);

	// set the center of mass
	NewtonBodySetCentreOfMass (body0, &origin[0]);

	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (body0, &matrix[0][0]);

	// disable collision with linked elements
	NewtonBodySetJointRecursiveCollision (body0, 0);

	// release the collision
	NewtonReleaseCollision (nWorld, collision);	

	// create the control joint joint
	SimpleIKHydrolicManipulator *minipulator; 
	minipulator = new SimpleIKHydrolicManipulator(armNode, matrix, body0, m_vehicleBody, 3.1316f * 50.0f / 180.0f);
	minipulator->m_angle = 0.8f * 3.1316f * 50.0f / 180.0f;
	m_frontBucketArm = minipulator;
 
	// set hydraulic initial position
 	// find the geometry mode;
	minipulator->m_IKNode0 = Find ("armHydraulicPiston");;
	_ASSERTE (minipulator->m_IKNode0);
	minipulator->m_IKNode0_LocalMatrix = minipulator->m_IKNode0->GetMatrix();

	minipulator->m_IKNode1 = Find ("armHydraulic");	
	_ASSERTE (minipulator->m_IKNode1);
	minipulator->m_IKNode1_LocalMatrix = minipulator->m_IKNode1->GetMatrix();

	// ***************************************************************************
	// add the bucket controller

	// find the geometry mode;
	armNode = Find ("frontBucket");
	_ASSERTE (armNode);

	// Get the node local Matrix;
	dMatrix matrix1 (armNode->CalcGlobalMatrix ());

	// iterate again collecting the vertex array
	vertexCount = armNode->GetGeometry()->GetFlatVertexArray (vertex, BUFFER_SIZE);

	// create the collision geometry
	collision = NewtonCreateConvexHull (nWorld, vertexCount, &vertex[0].m_x, sizeof (dVector), NULL);

	//create the rigid body
	body1 = NewtonCreateBody (nWorld, collision);

	// set the material group id for vehicle
	NewtonBodySetMaterialGroupID (body1, woodID);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (body1, ApplyGravityForce);

	// calculate the moment of inertia and the relative center of mass of the solid
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

	mass = VEHICLE_MASS * 0.15f;
	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];

	// set the mass matrix
	NewtonBodySetMassMatrix (body1, mass, Ixx, Iyy, Izz);

	// set the center of mass
	NewtonBodySetCentreOfMass (body1, &origin[0]);

	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (body1, &matrix1[0][0]);

	// disable collision with linked elements
	NewtonBodySetJointRecursiveCollision (body1, 0);

	// release the collision
	NewtonReleaseCollision (nWorld, collision);	


	// create the control joint joint
	SimpleIKHydrolicManipulator *minipulator1; 
	minipulator1 = new SimpleIKHydrolicManipulator(armNode, matrix1, body1, body0, 3.1316f * 150.0f / 180.0f);
	m_frontBucket = minipulator1;
 
	// set hydraulic initial position
 	// find the geometry mode;
	minipulator1->m_IKNode0 = Find ("frontBucketHydraulic");
	_ASSERTE (minipulator1->m_IKNode0);
	minipulator1->m_IKNode0_LocalMatrix = minipulator1->m_IKNode0->GetMatrix();

	minipulator1->m_IKNode1 = Find ("frontBucketHydraulicPiston");	
	_ASSERTE (minipulator1->m_IKNode1);
	minipulator1->m_IKNode1_LocalMatrix = minipulator1->m_IKNode1->GetMatrix();
}



void NewtonTractor::AddBackBucket (NewtonWorld* nWorld, dVector* vertex)
{
	dInt32 vertexCount;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	dVector origin;
	dVector inertia;
	NewtonBody* body0;
	NewtonBody* body1;
	dSceneNode* armNode;
	NewtonCollision* collision;


	// find the geometry mode;
	armNode = Find ("boom");
	_ASSERTE (armNode);

	// Get the node local Matrix;
	dMatrix matrix (armNode->CalcGlobalMatrix ());


	// iterate again collecting the vertex array
	vertexCount = armNode->GetGeometry()->GetFlatVertexArray (vertex, BUFFER_SIZE);

	// create the collision geometry
	collision = NewtonCreateConvexHull (nWorld, vertexCount, &vertex[0].m_x, sizeof (dVector), NULL);

	//create the rigid body
	body0 = NewtonCreateBody (nWorld, collision);

	// set the material group id for vehicle
	NewtonBodySetMaterialGroupID (body0, woodID);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (body0, ApplyGravityForce);

	// calculate the moment of inertia and the relative center of mass of the solid
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

	mass = VEHICLE_MASS * 0.1f;
	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];

	// set the mass matrix
	NewtonBodySetMassMatrix (body0, mass, Ixx, Iyy, Izz);

	// set the center of mass
	NewtonBodySetCentreOfMass (body0, &origin[0]);

	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (body0, &matrix[0][0]);

	// disable collision with linked elements
	NewtonBodySetJointRecursiveCollision (body0, 0);

	// release the collision
	NewtonReleaseCollision (nWorld, collision);	

	// create the control joint joint
	BoomIKHydrolicManipulator *minipulator1; 
	minipulator1 = new BoomIKHydrolicManipulator(armNode, matrix, body0, m_vehicleBody, 3.1316f * 80.0f / 180.0f, 3.1316f * 90.0f / 180.0f);
	m_backBucketArm1 = minipulator1;

	
	// set hydraulic initial position
 	// find the geometry mode;
	minipulator1->m_IKNode0 = Find ("boomHydraulic");
	_ASSERTE (minipulator1->m_IKNode0);
	minipulator1->m_IKNode0_LocalMatrix = minipulator1->m_IKNode0->GetMatrix();

	minipulator1->m_IKNode1 = Find ("boomHydraulicPiston");	
	_ASSERTE (minipulator1->m_IKNode1);
	minipulator1->m_IKNode1_LocalMatrix = minipulator1->m_IKNode1->GetMatrix();



	// ****************************************************************************
	//
	// add second arm

	// find the geometry mode;
	armNode = Find ("boomArm");
	_ASSERTE (armNode);

	// Get the node local Matrix;
	matrix = armNode->CalcGlobalMatrix ();

	// iterate again collecting the vertex array
	vertexCount = armNode->GetGeometry()->GetFlatVertexArray (vertex, BUFFER_SIZE);

	// create the collision geometry
	collision = NewtonCreateConvexHull (nWorld, vertexCount, &vertex[0].m_x, sizeof (dVector), NULL);

	//create the rigid body
	body1 = NewtonCreateBody (nWorld, collision);

	// set the material group id for vehicle
	NewtonBodySetMaterialGroupID (body1, woodID);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (body1, ApplyGravityForce);

	// calculate the moment of inertia and the relative center of mass of the solid
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

	mass = VEHICLE_MASS * 0.10f;
	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];

	// set the mass matrix
	NewtonBodySetMassMatrix (body1, mass, Ixx, Iyy, Izz);

	// set the center of mass
	NewtonBodySetCentreOfMass (body1, &origin[0]);

	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (body1, &matrix[0][0]);

	// disable collision with linked elements
	NewtonBodySetJointRecursiveCollision (body1, 0);

	// release the collision
	NewtonReleaseCollision (nWorld, collision);	

	// create the control joint joint
	SimpleIKHydrolicManipulator *minipulator2; 
	minipulator2 = new SimpleIKHydrolicManipulator(armNode, matrix, body1, body0, 3.1316f * 65.0f / 180.0f);
	minipulator2->m_angle = 3.1316f * 65.0f / 180.0f;
	m_backBucketArm2 = minipulator2;

	
	// set hydraulic initial position
 	// find the geometry mode;
	minipulator2->m_IKNode0 = Find ("boomArmHydraulic");
	_ASSERTE (minipulator2->m_IKNode0);
	minipulator2->m_IKNode0_LocalMatrix = minipulator2->m_IKNode0->GetMatrix();

	minipulator2->m_IKNode1 = Find ("boomArmHydraulicPiston");	
	_ASSERTE (minipulator2->m_IKNode1);
	minipulator2->m_IKNode1_LocalMatrix = minipulator2->m_IKNode1->GetMatrix();




	// ***************************************************************************
	// add the bucket controller
	body0 = body1;

	// find the geometry mode;
	armNode = Find ("bucket");
	_ASSERTE (armNode);

	// Get the node local Matrix;
	dMatrix matrix1 (armNode->CalcGlobalMatrix ());

	// iterate again collecting the vertex array
	vertexCount = armNode->GetGeometry()->GetFlatVertexArray (vertex, BUFFER_SIZE);

	// create the collision geometry
	collision = NewtonCreateConvexHull (nWorld, vertexCount, &vertex[0].m_x, sizeof (dVector), NULL);

	//create the rigid body
	body1 = NewtonCreateBody (nWorld, collision);

	// set the material group id for vehicle
	NewtonBodySetMaterialGroupID (body1, woodID);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (body1, ApplyGravityForce);

	// calculate the moment of inertia and the relative center of mass of the solid
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

	mass = VEHICLE_MASS * 0.1f;
	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];

	// set the mass matrix
	NewtonBodySetMassMatrix (body1, mass, Ixx, Iyy, Izz);

	// set the center of mass
	NewtonBodySetCentreOfMass (body1, &origin[0]);

	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (body1, &matrix1[0][0]);

	// disable collision with linked elements
	NewtonBodySetJointRecursiveCollision (body1, 0);

	// release the collision
	NewtonReleaseCollision (nWorld, collision);	


	// create the control joint joint
	BackBuckectIKHydrolicManipulator *minipulator3; 
	minipulator3 = new BackBuckectIKHydrolicManipulator(armNode, matrix1, body1, body0, 3.1316f * 179.0f / 180.0f);
	m_backBucket = minipulator3;
 
	// set hydraulic initial position
 	// find the geometry mode;

	minipulator3->SetStrutIK (Find ("strut"), Find ("strut2"), Find ("buckectPivot"));


	minipulator3->m_IKNode0 = Find ("buckectHydraulic");
	_ASSERTE (minipulator3->m_IKNode0);
	minipulator3->m_IKNode0_LocalMatrix = minipulator3->m_IKNode0->GetMatrix();

	minipulator3->m_IKNode1 = Find ("buckectHydraulicPiston");	
	_ASSERTE (minipulator3->m_IKNode1);
	minipulator3->m_IKNode1_LocalMatrix = minipulator3->m_IKNode1->GetMatrix();
}

#endif