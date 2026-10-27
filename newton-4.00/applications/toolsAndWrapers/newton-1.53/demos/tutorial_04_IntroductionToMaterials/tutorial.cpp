//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************

#include <stdafx.h>
#include "tutorial.h"
#include "HiResTimer.h"
#include "Material.h"
#include "LevelPrimitive.h"
#include "RenderPrimitive.h"

CHiResTimer timer;


#define GRAVITY	-10.0f


static NewtonWorld* nWorld;
static RenderPrimitive* player;
static dSceneNode* nGrapphicWorld;

static dVector cameraDir (0.0f, 0.0f, -1.0f);
static dVector cameraEyepoint (0.0f, 5.0f, 0.0f);

static bool forceMode;
static dVector cameraTorqueVector (0.0f, 0.0f, 0.0f); 

static LevelPrimitive* gLevel;

static void CleanUp ();
static void  Keyboard();
static void*  PhysicsAlloc (int sizeInBytes);
static void  PhysicsFree (void *ptr, int sizeInBytes);
static void  PhysicsBodyDestructor (const NewtonBody* body);
static void  PhysicsApplyForceAndTorque (const NewtonBody* body);
static void  PhysicsSetTransform (const NewtonBody* body, const dFloat* matrix);




static void AddBallCharacter ();
static void  BallCharacterApplyForceAndTorque (const NewtonBody* body);

//********************************************************************
// 
//	Newton Tutorial 2 Using Callbacks
//
//********************************************************************
int main(int argc, char **argv)
{
	// initialize opengl	
	InitOpenGl (argc, argv, "Newton Tutotial 4: Introduction to materials", DrawScene);

	// create the physics scene
	InitScene();
	
	// Execute opengl main loop for ever
	glutMainLoop();   

	return 0;
} 

// memory allocation for Newton
void* PhysicsAlloc (int sizeInBytes)
{
	return malloc (sizeInBytes);
}

// memory de-allocation for Newton
void PhysicsFree (void *ptr, int sizeInBytes)
{
	free (ptr);
}

// destroy the world and every thing in it
void CleanUp ()
{
	// clear all resources use with materials
	CleanUpMaterials (nWorld);

	// destroy the Newton world
	NewtonDestroy (nWorld);
}

// rigid body destructor
void PhysicsBodyDestructor (const NewtonBody* body)
{
	RenderPrimitive* primitive;

	// get the graphic object form the rigid body
	primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);

	// destroy the graphic object
	delete primitive;
}

// set the transformation of a rigid body
void PhysicsApplyForceAndTorque (const NewtonBody* body)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
	dVector force (0.0f, mass * GRAVITY, 0.0f);
	NewtonBodySetForce (body, &force.m_x);
}

// add force and torque to rigid body
void PhysicsSetTransform (const NewtonBody* body, const dFloat* matrix)
{
	RenderPrimitive* primitive;

	// get the graphic object form the rigid body
	primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);

	// set the transformation matrix for this rigid body
	dMatrix& mat = *((dMatrix*)matrix);
	primitive->SetMatrix (mat);
}


// create physics scene
void InitScene()
{
	BoxPrimitive* box;

	NewtonBody* boxBody;
	LevelPrimitive *level;
	NewtonCollision* collision;

	// /////////////////////////////////////////////////////////////////////
	//
	// create the newton world
	nWorld = NewtonCreate (PhysicsAlloc, PhysicsFree);

	// Set the termination function
	atexit(CleanUp); 

	// /////////////////////////////////////////////////////////////////////
	//
	// set up all material and  material interactions
	SetupMaterials (nWorld);

	// /////////////////////////////////////////////////////////////////////
	//
	// create the sky box,
	nGrapphicWorld = new SkyBoxPrimitive (NULL);


	// /////////////////////////////////////////////////////////////////////
	//
	// Create a ball controllable by the player
	AddBallCharacter ();


	// /////////////////////////////////////////////////////////////////////
	//
	// Load level geometry and add it to the display list 
	level = new LevelPrimitive (nGrapphicWorld, "level1.dg", nWorld, NULL);
	gLevel = level;

	// assign the concrete id to the level geometry
	NewtonBodySetMaterialGroupID (level->m_level, levelID);
	

	// /////////////////////////////////////////////////////////////////////
	//
	// add some object to the background

	// set the initial size
	dVector size (1.0f, 1.0f, 1.0f);

	// create the collision 
	collision = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, NULL); 
		
	// create 100 tack of 10 boxes each
	dMatrix location (GetIdentityMatrix());
	location.m_posit.m_x = -10.0f; 

	for (int k = 0; k < 10; k ++) { 
		location.m_posit.m_z = 0.0f; 
		for (int j = 0; j < 10; j ++) { 
			location.m_posit.m_y = 10.0f; 
			for (int i = 0; i < 1; i ++) {

				// create a graphic box
				box = new BoxPrimitive (nGrapphicWorld, location, size);

				//create the rigid body
				boxBody = NewtonCreateBody (nWorld, collision);

				//dVector omega(5, 0, 0);
				//NewtonBodySetOmega (boxBody, &omega[0]);

				// Set Material Id for this object
				if (dRand() & 1) {
					NewtonBodySetMaterialGroupID (boxBody, metalID);
				} else {
					NewtonBodySetMaterialGroupID (boxBody, woodID);
				}

				// save the pointer to the graphic object with the body.
				NewtonBodySetUserData (boxBody, box);

				// set a destructor for this rigid body
				NewtonBodySetDestructorCallback (boxBody, PhysicsBodyDestructor);

				// set the transform call back function
				NewtonBodySetTransformCallback (boxBody, PhysicsSetTransform);

				// set the force and torque call back function
				NewtonBodySetForceAndTorqueCallback (boxBody, PhysicsApplyForceAndTorque);

				// set the mass matrix
				NewtonBodySetMassMatrix (boxBody, 1.0f, 1.0f, 1.0f, 1.0f);

				// set the matrix for both the rigid body and the graphic body
				NewtonBodySetMatrix (boxBody, &location[0][0]);
				PhysicsSetTransform (boxBody, &location[0][0]);

				location.m_posit.m_y += size.m_y * 2.0f;
			}
			location.m_posit.m_z -= size.m_z * 4.0f; 	
		}
		location.m_posit.m_x += size.m_x * 4.0f; 
	}

	// release the collision geometry when not need it
	NewtonReleaseCollision (nWorld, collision);

}


// DrawScene()
void DrawScene ()
{
	dFloat timeStep;


	// test running a fix frame rate
	NewtonSetMinimumFrameRate (nWorld, 0.0f);


	// get the time step
	timeStep = timer.GetElapsedSeconds();

	// update the Newton physics world
	NewtonUpdate (nWorld, timeStep);


	// read the keyboard
	Keyboard ();

	// move the camera
	dVector target (cameraEyepoint + cameraDir);
	SetCamera (cameraEyepoint, target);


	// render the scene
	glEnable (GL_LIGHTING);
	glPushMatrix();	
	nGrapphicWorld->Render ();
	glPopMatrix();

	dVector color (1.0, 1.0f, 1.0f);
	Print (color, 4, 4, "fps %f", 1.0f / (timeStep + 1.0e-6f));
} 


//	Keyboard handler. 
void  Keyboard()
{
	// check for termination
	if (dGetKeyState (VK_ESCAPE) & 0x8000) {
		exit(0);
	}

	// read the mouse position and set the camera direction
	static MOUSE_POINT mouse0;
	static dFloat yawAngle = 90.0f * 3.1416 / 180.0f;
	static dFloat rollAngle = 0.0f;

	MOUSE_POINT mouse1;
	GetCursorPos(mouse1);

	if (dGetKeyState (VK_LBUTTON) & 0x8000) {
		if (mouse1.x > (mouse0.x + 1)) {
			yawAngle += 5.0f * 3.1416 / 180.0f;
			if (yawAngle > (360.0f * 3.1416 / 180.0f)) {
				yawAngle -= (360.0f * 3.1416 / 180.0f);
			}
		} else if (mouse1.x < (mouse0.x - 1)) {
			yawAngle -= 5.0f * 3.1416 / 180.0f;
			if (yawAngle < 0.0f) {
				yawAngle += (360.0f * 3.1416 / 180.0f);
			}
		}

		if (mouse1.y > (mouse0.y + 1)) {
			rollAngle += 2.0f * 3.1416 / 180.0f;
			if (rollAngle > (80.0f * 3.1416 / 180.0f)) {
				rollAngle = 80.0f * 3.1416 / 180.0f;
			}
		} else if (mouse1.y < (mouse0.y - 1)) {
			rollAngle -= 2.0f * 3.1416 / 180.0f;
			if (rollAngle < -(80.0f * 3.1416 / 180.0f)) {
				rollAngle = -80.0f * 3.1416 / 180.0f;
			}
		}
		dMatrix cameraDirMat (dgRollMatrix(rollAngle) * dgYawMatrix(yawAngle));
		cameraDir = cameraDirMat.m_front;
	}

	mouse0 = mouse1;


	cameraTorqueVector = dVector (0.0, 0.0, 0.0); 

	// camera control
	if (dGetKeyState ('W') & 0x8000) {
		cameraTorqueVector.m_z = -1.0f;
	} else if (dGetKeyState ('S') & 0x8000) {
		cameraTorqueVector.m_z = 1.0f;
	}

	if (dGetKeyState ('D') & 0x8000) {
		cameraTorqueVector.m_x = 1.0f;
	} else if (dGetKeyState ('A') & 0x8000) {
		cameraTorqueVector.m_x = -1.0f;
	}


	forceMode = false;
	if (dGetKeyState (' ') & 0x8000) {
		forceMode = true;
	}

	if (player) {
		cameraEyepoint = player->GetMatrix().m_posit;
		cameraEyepoint.m_y += 1.0f;

		cameraEyepoint -= cameraDir.Scale (4.0f);
	}
} 




// set the transformation of a rigid body
void BallCharacterApplyForceAndTorque (const NewtonBody* body)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;


	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
	dVector force (0.0f, mass * GRAVITY, 0.0f);
	NewtonBodySetForce (body, &force.m_x);


	// calculate a rotation matrix from the camera heading
	dMatrix matrix (GetIdentityMatrix()); 
	dVector lateralDir (cameraDir * dVector (0.0f, 1.0f, 0.0f));
	matrix.m_right = lateralDir.Scale (1.0f / (lateralDir % lateralDir));
	matrix.m_front = matrix.m_up * matrix.m_right;


	if (forceMode) {
		// this is the flight mode
		dVector force;
		dVector velocity;

		// rotate the force direction to align with the camera
		dVector forceDir (matrix.RotateVector (cameraTorqueVector));
 		forceDir = dgYawMatrix(-3.1416f * 0.5f).RotateVector (forceDir);
		forceDir.m_y = 1.0f;

		
		NewtonBodyGetForce(body, &force.m_x);
		NewtonBodyGetVelocity(body, &velocity.m_x);

		force += (forceDir.Scale (mass * 30.0f) - forceDir.Scale (5.0f * (velocity % forceDir))); 
		NewtonBodySetForce (body, &force.m_x);
	} else {
		// this is the rolling mode
		dVector omega;

		// rotate the torque direction to align with the camera
		dVector torqueDir (matrix.RotateVector (cameraTorqueVector));
		NewtonBodyGetOmega(body, &omega.m_x);
		dVector torque (torqueDir.Scale (Ixx * 75.0f) - torqueDir.Scale (2.0f * Ixx * (omega % torqueDir))); 
		NewtonBodySetTorque (body, &torque.m_x);
	}
}


#if 0
void AddBallCharacter ()
{
	dFloat radius;
	NewtonBody* sphBody;
	SpherePrimitive* sph;
	NewtonCollision* collision;

	radius = 0.5f;
	// create the collision 
	collision = NewtonCreateSphere (nWorld, radius, radius, radius, NULL); 
		
	// create 100 tack of 10 boxes each
	dMatrix location (GetIdentityMatrix());
	location.m_posit.m_y = 10; 

	// create the graphic object
	sph = new SpherePrimitive (nGrapphicWorld, location, radius, radius, radius);

	// save as player
	player = sph;

	//create the rigid body
	sphBody = NewtonCreateBody (nWorld, collision);

	// disable auto freeze management for the player
	NewtonBodySetAutoFreeze (sphBody, 0);

	// keep the player always active 
	NewtonWorldUnfreezeBody (nWorld, sphBody);

	// Set Material Id for this object
	NewtonBodySetMaterialGroupID (sphBody, characterID);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (sphBody, sph);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (sphBody, PhysicsBodyDestructor);

	// set the transform call back function
	NewtonBodySetTransformCallback (sphBody, PhysicsSetTransform);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (sphBody, BallCharacterApplyForceAndTorque);

	// set the mass matrix
	dFloat mass = 10.0f;
	dFloat I = (2.0f / 5.0f) * mass * radius * radius;
	NewtonBodySetMassMatrix (sphBody, mass, I, I, I);

	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (sphBody, &location[0][0]);
	PhysicsSetTransform (sphBody, &location[0][0]);

	// release the collision geometry when not need it
	NewtonReleaseCollision (nWorld, collision);
}


#else


static int wheelMaterial;

static NewtonBody* CreateRobotBody (const dMatrix& matrix, const dVector& size)
{
	BoxPrimitive* box;
	NewtonBody* boxBody;
	NewtonCollision* boxCollision;

	// create the collision 
	boxCollision = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, NULL); 


	//create the rigid body
	boxBody = NewtonCreateBody (nWorld, boxCollision);

	// relase the collision
	NewtonReleaseCollision (nWorld, boxCollision);

	box = new BoxPrimitive (nGrapphicWorld, matrix, size);

	// save as player
	player = box;

	// use wood id for robo body
	NewtonBodySetMaterialGroupID (boxBody, woodID);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (boxBody, box);

	// set a destrutor for this rigid body
	NewtonBodySetDestructorCallback (boxBody, PhysicsBodyDestructor);

	// set the tranform call back function
	NewtonBodySetTransformCallback (boxBody, PhysicsSetTransform);

	// set the force and torque call back funtion
	NewtonBodySetForceAndTorqueCallback (boxBody, PhysicsApplyForceAndTorque);

	// set the mass matrix
	float mass = 20.0f;
	float Ixx = mass * (size.m_y * size.m_y + size.m_z * size.m_z) / 12.0f;
	float Iyy = mass * (size.m_x * size.m_x + size.m_z * size.m_z) / 12.0f;
	float Izz = mass * (size.m_x * size.m_x + size.m_y * size.m_y) / 12.0f;

	NewtonBodySetMassMatrix (boxBody, mass, Ixx, Iyy, Izz);

	// set the matrix for tboth the rigid nody and the graphic body
	NewtonBodySetMatrix (boxBody, &matrix[0][0]);
	PhysicsSetTransform (boxBody, &matrix[0][0]);

	return boxBody;
}



static NewtonBody* CreateWheel (const dMatrix& location)
{
	float radius;
	NewtonBody* sphBody;
	SpherePrimitive* sph;
	NewtonCollision* sphCollision;


	radius = 0.5f;

	// create the collision 
	sphCollision = NewtonCreateSphere (nWorld, radius, radius, radius, NULL); 

	//create the rigid body
	sphBody = NewtonCreateBody (nWorld, sphCollision);

	// relase the collision
	NewtonReleaseCollision (nWorld, sphCollision);

	sph = new SpherePrimitive (nGrapphicWorld, location, radius, radius, radius);


	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (sphBody, sph);

	// set a destrutor for this rigid body
	NewtonBodySetDestructorCallback (sphBody, PhysicsBodyDestructor);

	// set the tranform call back function
	NewtonBodySetTransformCallback (sphBody, PhysicsSetTransform);

	// set the force and torque call back funtion
	NewtonBodySetForceAndTorqueCallback (sphBody, PhysicsApplyForceAndTorque);

	// set the mass matrix
	float mass = 2.0f;
	float Ixx = 0.4f * mass * radius * radius;
	NewtonBodySetMassMatrix (sphBody, mass, Ixx, Ixx, Ixx);

	// set the matrix for tboth the rigid nody and the graphic body
	NewtonBodySetMatrix (sphBody, &location[0][0]);
	PhysicsSetTransform (sphBody, &location[0][0]);

	return sphBody;

}





dVector bodyHeadingVector;

int _cdecl RobotContactBegin (const NewtonMaterial* material, const NewtonBody* body0, const NewtonBody* body1)
{
	const NewtonBody* robotChassis;

	// I am using the material ID of the wheel to determine which one of this two bodies is the robot chassis
	// The application can used a diffrent method 
	robotChassis = (NewtonBodyGetMaterialGroupID (body1) == wheelMaterial) ? body0 : body1;

	dMatrix matrix;
	NewtonBodyGetMatrix(robotChassis, &matrix[0][0]);
	bodyHeadingVector = matrix.m_front;
	return 1;
}


// this callback is called for every contact between the two bodies
int _cdecl RobotContactProcess (const NewtonMaterial* material, const NewtonContact* contact)
{
	NewtonMaterialContactRotateTangentDirections (material, &bodyHeadingVector[0]);	
	return 1;
}



// robot servo controller
struct ROBOT_SERVO_CONTROLLOER
{

	static void _cdecl DestroyWheel (const NewtonJoint* joint)
	{
		ROBOT_SERVO_CONTROLLOER* me;

		me = (ROBOT_SERVO_CONTROLLOER*) NewtonJointGetUserData (joint);
		delete me;
	}

	static unsigned _cdecl RobotMotor (const NewtonJoint* joint, NewtonHingeSliderUpdateDesc* desc)
	{
		ROBOT_SERVO_CONTROLLOER* me;
		me = (ROBOT_SERVO_CONTROLLOER*) NewtonJointGetUserData (joint);
		return me->ApplyControl (joint, desc);
	}


	ROBOT_SERVO_CONTROLLOER (bool left_right)
	{
		// only strore the infomrtation to know what side thi wheel is,
		// but you can store more ibformation here, like sound handles, and other feedback data.
		m_wheelSide = left_right ? -1.0f : 1.0f;
		m_wheelDamp = 0.1f;
		m_turnSpeed = 2.0f;
		m_fowardSpeed = 10.0f;
		m_backwardSpeed = -10.0f;
		m_accelConst = 0.15f;
	}

	~ROBOT_SERVO_CONTROLLOER()
	{
	}

	unsigned ApplyControl (const NewtonJoint* hinge, NewtonHingeSliderUpdateDesc* desc)
	{

		float accel;
		float omega;
		float desiredOmega;

		omega = NewtonHingeGetJointOmega (hinge);
		if (cameraTorqueVector.m_z > 0.1f) {
			desiredOmega = m_fowardSpeed;
		} else if (cameraTorqueVector.m_z < -0.1f) {
			desiredOmega = m_backwardSpeed;
		} else {
			desiredOmega = 0.0f;
		}

		if (cameraTorqueVector.m_x > 0.1f) {
			desiredOmega += m_turnSpeed  * m_wheelSide;
		} else if (cameraTorqueVector.m_x < -0.1f) {
			desiredOmega -= m_turnSpeed  * m_wheelSide;
		} 

		accel = m_accelConst * (desiredOmega - omega) / desc->m_timestep;
		desc->m_accel = accel - m_wheelDamp * omega;
		//desc->m_accel = accel;

		return 1;
	}


	float m_wheelDamp;
	float m_accelConst;
	float m_turnSpeed;
	float m_fowardSpeed;
	float m_backwardSpeed;
	float m_wheelSide;
} ;



void AddBallCharacter ()
{
	NewtonBody* robotBody;
	NewtonBody* leftWheel;
	NewtonBody* rightWheel;
	NewtonBody* frontWheel;

	// create the robot body parts
	dVector size (2.0f, 2.0f, 2.0f);
	dMatrix location (GetIdentityMatrix());
	location.m_posit.m_y = 10.0f; 
	robotBody = CreateRobotBody (location, size);
	NewtonBodySetAutoFreeze (robotBody, 0);

	dMatrix leftMatrix (location);
	leftMatrix.m_posit.m_x -= size.m_x * 0.5f;
	leftMatrix.m_posit.m_y -= size.m_y * 0.5f;
	leftMatrix.m_posit.m_z += size.m_z * 0.5f;
	leftWheel = CreateWheel (leftMatrix);

	dMatrix rightMatrix (location);
	rightMatrix.m_posit.m_x += size.m_x * 0.5f;
	rightMatrix.m_posit.m_y -= size.m_y * 0.5f;
	rightMatrix.m_posit.m_z += size.m_z * 0.5f;
	rightWheel = CreateWheel (rightMatrix);


	dMatrix frontMatrix (location);
	frontMatrix.m_posit.m_z -= size.m_x * 0.5f;
	frontMatrix.m_posit.m_y -= size.m_y * 0.5f;
	frontWheel = CreateWheel (frontMatrix);

	NewtonJoint* leftJoint;
	NewtonJoint* rightJoint;
	NewtonJoint* frontJoint;

	// connect the wheels to the body
	frontJoint = NewtonConstraintCreateBall (nWorld, &frontMatrix.m_posit[0], frontWheel, robotBody);
	leftJoint = NewtonConstraintCreateHinge (nWorld, &leftMatrix.m_posit[0], &leftMatrix.m_front[0], leftWheel, robotBody);
	rightJoint = NewtonConstraintCreateHinge (nWorld, &rightMatrix.m_posit[0], &rightMatrix.m_front[0], rightWheel, robotBody);


	// use a pdi to keep the wheel spin constants

	ROBOT_SERVO_CONTROLLOER* leftController = new ROBOT_SERVO_CONTROLLOER(true);
	NewtonJointSetUserData (leftJoint, leftController);
	NewtonJointSetDestructor (leftJoint, ROBOT_SERVO_CONTROLLOER::DestroyWheel);
	NewtonHingeSetUserCallback (leftJoint, ROBOT_SERVO_CONTROLLOER::RobotMotor);


	ROBOT_SERVO_CONTROLLOER* rightController = new ROBOT_SERVO_CONTROLLOER(false);
	NewtonJointSetUserData (rightJoint, rightController);
	NewtonJointSetDestructor (rightJoint, ROBOT_SERVO_CONTROLLOER::DestroyWheel);
	NewtonHingeSetUserCallback (rightJoint, ROBOT_SERVO_CONTROLLOER::RobotMotor);


	// CREATE A mATERIAL TO CONTROL THE WHEEL CONTACTS
	int defaultID;
	defaultID = NewtonMaterialGetDefaultGroupID(nWorld);
	wheelMaterial = NewtonMaterialCreateGroupID(nWorld);

	NewtonBodySetMaterialGroupID (leftWheel, wheelMaterial);
	NewtonBodySetMaterialGroupID (rightWheel, wheelMaterial);
	NewtonBodySetMaterialGroupID (frontWheel, wheelMaterial);

	NewtonMaterialSetDefaultElasticity (nWorld, levelID, wheelMaterial, 0.1f);
	NewtonMaterialSetDefaultFriction (nWorld, levelID, wheelMaterial, 1.0f, 1.0f);
	NewtonMaterialSetCollisionCallback (nWorld, levelID, wheelMaterial, NULL, RobotContactBegin, RobotContactProcess, NULL); 
}

#endif




